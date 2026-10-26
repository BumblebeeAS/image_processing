"""Rectified calibration must describe pinhole pixels without changing stamps."""

import pytest
from sensor_msgs.msg import CameraInfo

from image_processing.rectified_camera_info_node import rectified_camera_info


@pytest.mark.parametrize("length", [4, 5])
def test_fisheye_rectified_info_uses_projection_intrinsics(length):
    message = CameraInfo(width=640, height=360, distortion_model="equidistant")
    message.header.frame_id = "uav/wide_a_optical"
    message.header.stamp.sec = 123
    message.d = [0.0] * length
    message.k = [400.0, 0.0, 320.0, 0.0, 400.0, 180.0, 0.0, 0.0, 1.0]
    message.p = [390.0, 0.0, 310.0, 0.0, 0.0, 389.0, 179.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    result = rectified_camera_info(message)
    assert result.distortion_model == "plumb_bob"
    assert list(result.d) == [0.0] * 5
    assert list(result.k) == [390.0, 0.0, 310.0, 0.0, 389.0, 179.0, 0.0, 0.0, 1.0]
    assert list(result.p) == list(message.p)
    assert result.header == message.header
    assert (result.width, result.height) == (640, 360)
    assert message.distortion_model == "equidistant"
    assert len(message.d) == length


def test_distorted_input_is_not_silently_relabelled():
    with pytest.raises(ValueError, match="zero distortion"):
        rectified_camera_info(CameraInfo(d=[0.1, 0.0, 0.0, 0.0]))
