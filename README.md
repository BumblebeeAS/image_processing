# Image Processing

ROS 2 utilities for preparing, visualizing, and timestamping camera data.

## Nodes

### Depth image visualization

`depth_to_rgb_node` expects a `32FC1` depth image. It normalizes the values to 8-bit intensity, applies OpenCV's inferno color map, and publishes a `bgr8` image.

```bash
ros2 run image_processing depth_to_rgb_node --ros-args \
  -p input_image_topic:=/camera/depth/image_raw \
  -p output_image_topic:=/camera/depth/visualization
```

### Image brightening

`image_brighten_node` scales the brightness of a `bgr8` image and applies a sharpening filter. This is useful for improving downstream thresholding or [image matching](https://github.com/bumblebeeas/image_matching) performance.

Example:

```bash
ros2 run image_processing image_brighten_node --ros-args \
  -p input_image_topic:=/camera/image_raw \
  -p output_image_topic:=/camera/image_enhanced \
  -p brightness_factor:=1.4
```

### Camera timestamp restamping

`restamp_camera_node` synchronizes an image with its camera-info message and replaces both timestamps with the same system-wall-time timestamp. This is useful when simulated camera messages need to be synchronized with processes that use wall time.

```bash
ros2 run image_processing restamp_camera_node --ros-args \
  -p in_image:=/camera/image_raw \
  -p in_info:=/camera/camera_info \
  -p out_image:=/camera/image_raw_wall \
  -p out_info:=/camera/camera_info_wall
```

> **Note:** This node is primarily a ROS 2 Humble workaround. On distributions
> that support `override_timestamps_with_wall_time`, prefer that option where
> appropriate.

## Utility modules

- `image_processing.utils.image_annotations` creates `foxglove_msgs/msg/ImageAnnotations` from collections of 2D points.
- `image_processing.utils.ros_np_multiarray` converts NumPy arrays to and from standard ROS multi-array message types.
