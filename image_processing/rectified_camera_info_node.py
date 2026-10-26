"""Publish pinhole calibration for an already-rectified image."""

from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPolicyKind
from rclpy.qos_overriding_options import QoSOverridingOptions
from sensor_msgs.msg import CameraInfo, RegionOfInterest


def rectified_camera_info(message):
    if any(abs(value) > 1e-12 for value in message.d):
        raise ValueError("Expected zero distortion from the rectifier")
    result = deepcopy(message)
    result.distortion_model = "plumb_bob"
    result.d = [0.0] * 5
    # P describes the actual rectified image, while K may describe the raw lens.
    result.k = [message.p[i] for i in (0, 1, 2, 4, 5, 6, 8, 9, 10)]
    result.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    result.binning_x = result.binning_y = 0
    result.roi = RegionOfInterest()
    return result


class RectifiedCameraInfoNode(Node):
    def __init__(self):
        super().__init__("rectified_camera_info_node")
        input_topic = self.declare_parameter("input_topic", "rect/camera_info_internal").value
        output_topic = self.declare_parameter("output_topic", "rect/camera_info").value
        qos_overrides = QoSOverridingOptions([
            QoSPolicyKind.RELIABILITY,
            QoSPolicyKind.DURABILITY,
            QoSPolicyKind.HISTORY,
            QoSPolicyKind.DEPTH,
        ])
        self.publisher = self.create_publisher(
            CameraInfo, output_topic, 10, qos_overriding_options=qos_overrides
        )
        self.subscription = self.create_subscription(
            CameraInfo, input_topic, self.callback, 10,
            qos_overriding_options=qos_overrides,
        )

    def callback(self, message):
        try:
            self.publisher.publish(rectified_camera_info(message))
        except ValueError as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = RectifiedCameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
