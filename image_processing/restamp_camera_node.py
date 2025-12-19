#!/usr/bin/env python3
"""
Restamp Image (and optional CameraInfo) messages with *wall time*.

- Subscribes:
    <in_image>      (sensor_msgs/msg/Image)
    <in_info>       (sensor_msgs/msg/CameraInfo) [optional]
- Publishes:
    <out_image>     (sensor_msgs/msg/Image)
    <out_info>      (sensor_msgs/msg/CameraInfo) [optional]

Notes:
- Uses a SYSTEM_TIME clock so it stamps with wall time even if /clock exists.
- Copies the message then overwrites header.stamp.

Temporary node for syncing camera data from simulation to system wall time.
Use override_timestamps_with_wall_time instead in Jazzy or when it's backported to Humble.
"""

import message_filters
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraRestamper(Node):
    def __init__(self):
        super().__init__("camera_restamper")

        self.in_image = (
            self.declare_parameter("in_image", "image_raw")
            .get_parameter_value()
            .string_value
        )
        self.out_image = (
            self.declare_parameter("out_image", "image_raw_wall")
            .get_parameter_value()
            .string_value
        )
        self.in_info = (
            self.declare_parameter("in_info", "camera_info")
            .get_parameter_value()
            .string_value
        )
        self.out_info = (
            self.declare_parameter("out_info", "camera_info_wall")
            .get_parameter_value()
            .string_value
        )

        # Force wall time regardless of ROS sim time settings
        self.wall_clock = Clock(clock_type=ClockType.SYSTEM_TIME)

        # Publishers
        self.pub_img = self.create_publisher(Image, self.out_image, 10)
        self.pub_info = self.create_publisher(CameraInfo, self.out_info, 10)

        # Subscriptions using message_filters for synchronization
        self.sub_img = message_filters.Subscriber(
            self, Image, self.in_image, qos_profile=qos_profile_sensor_data
        )
        self.sub_info = message_filters.Subscriber(
            self, CameraInfo, self.in_info, qos_profile=qos_profile_sensor_data
        )

        # Synchronize messages based on their timestamps
        self.ts = message_filters.TimeSynchronizer(
            [self.sub_img, self.sub_info], queue_size=10
        )
        self.ts.registerCallback(self.on_synchronized)

        self.get_logger().info(
            f"Restamping images:\n"
            f"  {self.in_image}  ->  {self.out_image}\n"
            f"  {self.in_info}  ->  {self.out_info}\n"
        )

    def on_synchronized(self, img_msg: Image, info_msg: CameraInfo):
        """Handle synchronized image and camera_info messages."""
        # Get a single timestamp for both messages
        timestamp = self.wall_clock.now().to_msg()

        # Process image
        out_img = Image()
        out_img.header = img_msg.header
        out_img.height = img_msg.height
        out_img.width = img_msg.width
        out_img.encoding = img_msg.encoding
        out_img.is_bigendian = img_msg.is_bigendian
        out_img.step = img_msg.step
        out_img.data = img_msg.data
        out_img.header.stamp = timestamp

        # Process camera_info
        out_info = CameraInfo()
        out_info.header = info_msg.header
        out_info.height = info_msg.height
        out_info.width = info_msg.width
        out_info.distortion_model = info_msg.distortion_model
        out_info.d = info_msg.d
        out_info.k = info_msg.k
        out_info.r = info_msg.r
        out_info.p = info_msg.p
        out_info.binning_x = info_msg.binning_x
        out_info.binning_y = info_msg.binning_y
        out_info.roi = info_msg.roi
        out_info.header.stamp = timestamp

        # Publish both with the same timestamp
        self.pub_img.publish(out_img)
        self.pub_info.publish(out_info)


def main():
    rclpy.init()
    node = CameraRestamper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
