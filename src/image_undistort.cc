#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
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

        image_topic_ = get_parameter("image_topic").as_string();
        undistorted_image_topic_ = get_parameter("undistorted_image_topic").as_string();

        // Create an image_transport subscriber
        image_sub_ = image_transport::create_subscription(
            this, image_topic_, std::bind(&ImageUndistortNode::imageCallback, this, std::placeholders::_1), "raw");

        image_pub_ = image_transport::create_publisher(this, undistorted_image_topic_);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        try
        {

            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat temp_image = cv_ptr->image;

            // Load calibration only once
            static cv::Mat camera_matrix, dist_coeffs;
            static bool calib_loaded = false;
            if (!calib_loaded) {
                cv::FileStorage fs("/home/gg/workspaces/miniauv/src/dwe_ros2_parser/config/dwe_calib.yaml", cv::FileStorage::READ);
                if (!fs.isOpened()) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to open calibration file");
                    return;
                }
                fs["camera_matrix"] >> camera_matrix;
                fs["distortion_coefficients"] >> dist_coeffs;
                calib_loaded = true;
            }

            // Undistort the image
            cv::Mat undistorted_image;
            cv::undistort(temp_image, undistorted_image, camera_matrix, dist_coeffs);

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
    chrono::high_resolution_clock::time_point start_time;

    image_transport::Publisher image_pub_;
    std::string image_topic_;
    std::string undistorted_image_topic_;


};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageUndistortNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
