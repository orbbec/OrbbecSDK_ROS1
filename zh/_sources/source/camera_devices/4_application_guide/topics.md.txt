# 可用话题

话题按流和功能组织。默认情况下，所有话题都在`/camera`命名空间下发布，可以通过`camera_name`启动参数进行更改。

> **注意：** 特定流的话题（例如，`/camera/color/...`）仅在其对应的启动参数（例如，`enable_color`）设置为`true`时才会发布。

### 图像流

这些话题为每个启用的相机流提供原始图像数据和相应的标定信息。`color`、`depth`、`left_ir`和`right_ir`流的模式是一致的。

*   `/camera/color/image_raw`
    *   来自彩色流的原始图像数据。
*   `/camera/color/camera_info`
    *   彩色流的相机标定数据和元数据。

*   `/camera/depth/image_raw`
    *   来自深度流的原始图像数据。
*   `/camera/depth/camera_info`
    *   深度流的相机标定数据和元数据。

*   `/camera/left_ir/image_raw`
    *   来自左红外（IR）流的原始图像数据。
*   `/camera/left_ir/camera_info`
    *   左红外流的相机标定数据和元数据。

*   `/camera/right_ir/image_raw`
    *   来自右红外（IR）流的原始图像数据。
*   `/camera/right_ir/camera_info`
    *   右红外流的相机标定数据和元数据。

### 点云话题

*   `/camera/depth/points`
    *   从深度流生成的点云数据。
    *   **条件：** 仅在`enable_point_cloud`为`true`时发布。

*   `/camera/depth_registered/points`
    *   彩色点云数据，其中深度点注册到彩色图像帧。
    *   **条件：** 仅在`enable_colored_point_cloud`为`true`时发布。

### 设备状态和诊断

*   `/diagnostics`
    *   发布关于相机节点的诊断信息。目前包括设备温度。
