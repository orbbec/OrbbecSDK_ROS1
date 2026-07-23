# 可用话题

话题按流和功能组织。默认情况下，所有话题都在`/camera`命名空间下发布，可以通过`camera_name`启动参数进行更改。

> **注意：** 特定流的话题（例如，`/camera/color/...`）仅在其对应的启动参数（例如，`enable_color`）设置为`true`时才会发布。

### 图像流

这些话题为每个启用的相机流提供原始图像数据和相应的标定信息。`color`、`depth`、`left_ir`和`right_ir`流的模式是一致的。

*   `/camera/color/image_raw`
    *   来自彩色流的原始图像数据。彩色格式为 RGB/YUYV 等非 MJPG 时，通常订阅该话题。
*   `/camera/color/image_raw/compressed`
    *   来自 MJPG 彩色流的压缩图像数据。使用 `color_format:=MJPG` 时建议订阅该话题，以避免 ROS wrapper 侧额外解码并降低 CPU 占用。
*   `/camera/color/camera_info`
    *   彩色流的相机标定数据和元数据。
*   `/camera/color/metadata`
    *   来自彩色流固件的底层元数据。

*   `/camera/depth/image_raw`
    *   来自深度流的原始图像数据。
*   `/camera/depth/camera_info`
    *   深度流的相机标定数据和元数据。
*   `/camera/depth/metadata`
    *   来自深度流固件的底层元数据。
*   `/camera/depth/image_unaligned`
    *   软件对齐前的深度图像。
    *   **条件：** 在 `depth_registration` 为 `true` 且 `align_mode` 为 `SW` 时发布；硬件对齐时不发布。
*   `/camera/confidence/image_raw`
    *   `EnhancedDepthFilter` 生成的置信度图像，根据置信度帧格式编码为 `mono8` 或 `mono16`。
    *   **条件：** 增强深度滤波成功且该话题有订阅者时发布。

*   `/camera/left_ir/image_raw`
    *   来自左红外（IR）流的原始图像数据。
*   `/camera/left_ir/camera_info`
    *   左红外流的相机标定数据和元数据。
*   `/camera/left_ir/metadata`
    *   来自左红外流固件的底层元数据。

*   `/camera/right_ir/image_raw`
    *   来自右红外（IR）流的原始图像数据。
*   `/camera/right_ir/camera_info`
    *   右红外流的相机标定数据和元数据。
*   `/camera/right_ir/metadata`
    *   来自右红外流固件的底层元数据。

### LRM 障碍物距离

*   `/camera/lrm/obstacle_distance`
    *   发布 LRM 测得的障碍物距离，消息类型为 `std_msgs/Int32`，单位为毫米。
    *   **条件：** 仅在 `enable_lrm_obstacle_distance_publish` 为 `true` 时发布。发布频率由 `lrm_obstacle_distance_publish_rate` 设置，默认值为 `10.0` Hz。

### 点云话题

*   `/camera/depth/points`
    *   从深度流生成的点云数据。
    *   **条件：** 仅在`enable_point_cloud`为`true`时发布。

*   `/camera/depth_registered/points`
    *   彩色点云数据，其中深度点注册到彩色图像帧。
    *   **条件：** 仅在`enable_colored_point_cloud`为`true`时发布。

### IMU 话题

惯性测量单元（IMU）话题提供加速度计和陀螺仪数据及其标定信息。

*   `/camera/accel/sample`
    *   单独的加速度计数据流。
    *   **条件：** 在 `enable_accel` 为 `true` 且 `enable_sync_output_accel_gyro` 为 `false` 时发布。

*   `/camera/gyro/sample`
    *   单独的陀螺仪数据流。
    *   **条件：** 在 `enable_gyro` 为 `true` 且 `enable_sync_output_accel_gyro` 为 `false` 时发布。

*   `/camera/gyro_accel/sample`
    *   在单条消息中发布同步的加速度计和陀螺仪数据。
    *   **条件：** 在 `enable_sync_output_accel_gyro` 为 `true` 时发布。

*   `/camera/accel/imu_info`
    *   加速度计标定信息和噪声特性，类型为 `orbbec_camera/IMUInfo`。
    *   **条件：** 随加速度计输出提供。

*   `/camera/gyro/imu_info`
    *   陀螺仪标定信息和噪声特性，类型为 `orbbec_camera/IMUInfo`。
    *   **条件：** 随陀螺仪输出提供。

### 外参话题

以下话题使用 `orbbec_camera/Extrinsics` 类型发布数据流之间的外参：

*   `/camera/depth_to_ir`
*   `/camera/depth_to_color`
*   `/camera/depth_to_left_ir`
*   `/camera/depth_to_right_ir`
*   `/camera/depth_to_accel`
*   `/camera/depth_to_gyro`

**条件：** 所选话题对应的两个数据流均须启用。仅发布连接设备支持的话题。

### 设备状态和诊断

*   `/diagnostics`
    *   发布关于相机节点的诊断信息。目前包括设备温度。

*   `/camera/depth_filters/status`
    *   发布当前深度滤波器的使能状态和参数，并在滤波状态变化后更新。
