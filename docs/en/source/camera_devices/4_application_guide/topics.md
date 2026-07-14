# Available Topics

Topics are organized by stream and function. By default, all topics are published under the `/camera` namespace, which can be changed with the `camera_name` launch parameter.

> **Note:** Topics for a specific stream (e.g., `/camera/color/...`) are only published if their corresponding launch parameter (e.g., `enable_color`) is set to `true`.

### Image Streams

These topics provide the raw image data and corresponding calibration information for each enabled camera stream. The pattern is consistent for `color`, `depth`, `left_ir`, and `right_ir` streams.

*   `/camera/color/image_raw`
    *   Raw image data from the color stream. Subscribe to this topic for non-MJPG color formats such as RGB or YUYV.
*   `/camera/color/image_raw/compressed`
    *   Compressed image data from the MJPG color stream. When `color_format:=MJPG` is used, subscribe to this topic to avoid extra decoding in the ROS wrapper and reduce CPU usage.
*   `/camera/color/camera_info`
    *   Camera calibration data and metadata for the color stream.
*   `/camera/color/metadata`
    *   Low-level metadata from Color stream firmware.

*   `/camera/depth/image_raw`
    *   Raw image data from the depth stream.
*   `/camera/depth/camera_info`
    *   Camera calibration data and metadata for the depth stream.
*   `/camera/depth/metadata`
    *   Low-level metadata from Depth stream firmware.

*   `/camera/left_ir/image_raw`
    *   Raw image data from the left infrared (IR) stream.
*   `/camera/left_ir/camera_info`
    *   Camera calibration data and metadata for the left IR stream.
*   `/camera/left_ir/metadata`
    *   Low-level metadata from Left IR stream firmware.

*   `/camera/right_ir/image_raw`
    *   Raw image data from the right infrared (IR) stream.
*   `/camera/right_ir/camera_info`
    *   Camera calibration data and metadata for the right IR stream.
*   `/camera/right_ir/metadata`
    *   Low-level metadata from Right IR stream firmware.

### LRM Obstacle Distance

*   `/camera/lrm/obstacle_distance`
    *   Publishes the obstacle distance measured by LRM. The message type is `std_msgs/Int32`, and the unit is millimeters.
    *   **Condition:** Published only when `enable_lrm_obstacle_distance_publish` is `true`. Set the rate with `lrm_obstacle_distance_publish_rate`; the default is `10.0` Hz.

### Point Cloud Topics

*   `/camera/depth/points`
    *   Point cloud data generated from the depth stream.
    *   **Condition:** Published only when `enable_point_cloud` is `true`.

*   `/camera/depth_registered/points`
    *   Colored point cloud data, where the depth points are registered to the color image frame.
    *   **Condition:** Published only when `enable_colored_point_cloud` is `true`.

### Device Status & Diagnostics

*   `/diagnostics`
    *   Publishes diagnostic information about the camera node. Currently, this includes the device temperature.

*   `/camera/depth_filters/status`
    *   Publishes the current enable state and parameters of depth filters, and updates when filter state changes.
