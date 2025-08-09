import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthToRGBNode(Node):
    def __init__(self):
        super().__init__("depth_to_rgb_node")

        self.declare_parameter("input_image_topic", "/depth_image")
        self.declare_parameter("output_image_topic", "/image")

        input_image_topic = (
            self.get_parameter("input_image_topic").get_parameter_value().string_value
        )
        output_image_topic = (
            self.get_parameter("output_image_topic").get_parameter_value().string_value
        )

        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, input_image_topic, self.image_callback, qos_profile_sensor_data
        )
        self.publisher = self.create_publisher(
            Image, output_image_topic, qos_profile_sensor_data
        )

    def image_callback(self, msg: Image):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")

            norm_image = cv2.normalize(cv_image, None, 0, 255, cv2.NORM_MINMAX)
            norm_image = norm_image.astype(np.uint8)

            color_map = cv2.applyColorMap(norm_image, cv2.COLORMAP_INFERNO)

            rgb_msg = self.bridge.cv2_to_imgmsg(color_map, encoding="bgr8")
            rgb_msg.header = msg.header

            self.publisher.publish(rgb_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = DepthToRGBNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
