#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <image_transport/image_transport.hpp>
#include <chrono>
#include <iostream>
#include <string>

using namespace std;

class ImageUndistortNode : public rclcpp::Node
{
public:
    ImageUndistortNode()
    : Node("image_undistort_node")
    {
        declare_parameter("image_topic", "/dwe/image_raw");
        declare_parameter("undistorted_image_topic", "/dwe/image_rect_color");
        declare_parameter("camera_info_topic", "/dwe/camera_info");

        image_topic_ = get_parameter("image_topic").as_string();
        undistorted_image_topic_ = get_parameter("undistorted_image_topic").as_string();
        camera_info_topic_ = get_parameter("camera_info_topic").as_string();

        // Subscribe to camera info
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 1,
            std::bind(&ImageUndistortNode::cameraInfoCallback, this, std::placeholders::_1));

        // Create an image_transport subscriber
        image_sub_ = image_transport::create_subscription(
            this, image_topic_, std::bind(&ImageUndistortNode::imageCallback, this, std::placeholders::_1), "raw");

        image_pub_ = image_transport::create_publisher(this, undistorted_image_topic_);
    }

private:
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        // Store camera matrix and distortion coefficients
        camera_matrix_ = cv::Mat(3, 3, CV_64F, (void*)msg->k.data()).clone();
        dist_coeffs_ = cv::Mat(msg->d.size(), 1, CV_64F, (void*)msg->d.data()).clone();
        camera_info_received_ = true;
        RCLCPP_INFO_ONCE(this->get_logger(), "Camera info received and stored.");
    }

    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        if (!camera_info_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for camera info...");
            return;
        }

        try
        {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat temp_image = cv_ptr->image;

            // // Compute optimal new camera matrix to avoid cropping
            // if (new_camera_matrix_.empty()) {
            //     new_camera_matrix_ = cv::getOptimalNewCameraMatrix(
            //         camera_matrix_, dist_coeffs_, temp_image.size(), 1, temp_image.size(), 0);
            // }

            // Undistort the image
            cv::Mat undistorted_image;
            cv::undistort(temp_image, undistorted_image, camera_matrix_, dist_coeffs_);

            // Convert to ROS message
            cv_bridge::CvImage undistorted_cv_image;
            undistorted_cv_image.header = msg->header;
            undistorted_cv_image.encoding = sensor_msgs::image_encodings::BGR8;
            undistorted_cv_image.image = undistorted_image;
            sensor_msgs::msg::Image undistorted_image_msg = *undistorted_cv_image.toImageMsg();

            // Publish the undistorted image
            image_pub_.publish(undistorted_image_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

    std::string image_topic_;
    std::string undistorted_image_topic_;
    std::string camera_info_topic_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;
    bool camera_info_received_ = false;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageUndistortNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
