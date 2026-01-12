# 启动参数

> 如果您不确定如何设置参数，可以连接奥比中光相机并打开[OrbbecViewer](https://github.com/orbbec/OrbbecSDK_v2/releases/tag/v2.3.5)参数设置。

以下是可用的启动参数：

### 核心和流配置

*   **`camera_name`**
    *   启动节点命名空间。
*   **`serial_number`**
    *   相机的序列号。当使用多个相机时，这是必需的。
*   **`usb_port`**
    *   相机的USB端口。当使用多个相机时，这是必需的。
*   **`device_num`**
    *   设备数量。如果需要多个相机，必须填写此项。
*   **`device_preset`**
    *   默认值为 `Default`。可以使用下面命令查看可设置模式
    ```bash
    ros2 run orbbec_camera list_camera_profile_mode_node
    ```
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   传感器流的分辨率和帧率。
*   **`enable_point_cloud`**
    *   启用点云。
*   **`enable_colored_point_cloud`**
    *   启用RGB点云。
*   **`point_cloud_decimation_filter_factor`**
    *   点云下采样因子。范围：`1–8`，`1`表示不下采样，数值越大下采样倍数越大。
*   **`ordered_pc`**
    *   启用无效点云的过滤。


### Sensor Controls

#### Color Stream

* **`enable_color_auto_exposure`**
  * 启用彩色相机自动曝光。
* **`enable_color_auto_exposure_priority`**
  * 启用彩色相机自动曝光优先模式。
* **`color_exposure`**
  * 设置彩色相机曝光值。
* **`color_gain`**
  * 设置彩色相机增益。
* **`enable_color_auto_white_balance`**
  * 启用彩色相机自动白平衡。
* **`color_white_balance`**
  * 设置彩色相机白平衡。
* **`color_ae_max_exposure`**
  * 设置彩色相机自动曝光的最大曝光值。
* **`color_brightness`**, **`color_sharpness`**, **`color_gamma`**, **`color_saturation`**, **`color_contrast`**, **`color_hue`**
  * 设置彩色相机的亮度、锐度、伽马、饱和度、对比度和色调。
* **`color_backlight_compensation`**
  * 启用彩色相机背光补偿功能。取值范围：`0–6`，默认值：`3`。
* **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
  * 启用彩色抽取（降采样）滤波器并设置其缩放比例。
* **`color_ae_roi_[left|right|top|bottom]`**
  * 设置彩色相机自动曝光（AE）的 ROI 区域边界。


#### 深度流

*   **`enable_depth_auto_exposure_priority`**
    *   启用深度自动曝光优先级。
*   **`mean_intensity_set_point`**
    *   设置深度图像的目标平均强度。
    > **注意：** 这取代了已弃用的`depth_brightness`，但为了向后兼容仍然支持该参数。
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   设置深度自动曝光ROI区域。

#### 红外流

*   **`enable_ir_auto_exposure`**
    *   启用红外自动曝光。
*   **`ir_exposure`**
    *   设置红外曝光。
*   **`ir_ae_max_exposure`**
    *   设置红外自动曝光的最大曝光值。
*   **`ir_brightness`**
    *   设置红外亮度。

#### 激光 / LDP

*   **`enable_laser`**
    *   启用激光。默认值为`true`。
*   **`laser_energy_level`**
    *   设置激光能量等级。
*   **`enable_ldp`**
    *   启用LDP。

### 设备、同步和高级功能

#### 多相机同步

*   **`sync_mode`**
    *   设置同步模式。默认值为`standalone`。
*   **`depth_delay_us`** / **`color_delay_us`**
    *   接收捕获命令或触发信号后深度/彩色图像捕获的延迟时间（微秒）。
*   **`trigger2image_delay_us`**
    *   接收捕获命令或触发信号后图像捕获的延迟时间（微秒）。
*   **`trigger_out_delay_us`**
    *   接收捕获命令或触发信号后触发信号输出的延迟时间（微秒）。
*   **`trigger_out_enabled`**
    *   启用触发输出信号。

> 用于[多相机同步](../5_advanced_guide/multi_camera/multi_camera_synced.md)。

#### 网络相机

*   **`enumerate_net_device`**
    *   启用自动枚举网络设备。
*   **`ip_address`** / **`port`**
    *   设置网络设备的IP地址和端口（通常为`8090`）。
*   **`force_ip_enable`**
    *   启用强制IP功能。**默认值：** `false`
*   **`force_ip_mac`**
    *   连接多个相机时的目标设备MAC地址（例如，`"54:14:FD:06:07:DA"`）。您可以使用`list_devices_node`查找每个设备的MAC。**默认值：** `""`

*   **`force_ip_address`**
    *   要分配的静态IP地址。**默认值：** `192.168.1.10`
*   **`force_ip_subnet_mask`**
    *   静态IP的子网掩码。**默认值：** `255.255.255.0`
*   **`force_ip_gateway`**
    *   静态IP的网关地址。**默认值：** `192.168.1.1`

> 用于[网络相机](../5_advanced_guide/configuration/net_camera.md)。

#### 设备特定参数
*   **`preset_resolution_config`**
    * 摄像头设备的预设分辨率配置。格式: "width,height,ir_decimation_factor,depth_decimation_factor". Example: "1280,720,4,4". 仅在 Gemini435Le 设备上受支持。留空禁用。
* **`ae_mode`**
  * `colorbased`：自动曝光基于彩色流。`depthbased`：自动曝光基于深度流。**默认值：** `depthbased`
    > 仅支持Gemini 305。
    >
* **`enalbe_sports_mode`**
  * 是否开启运动模式。**默认值：** `false`
    > 仅支持Gemini 305。
    >
* **`depth_downscale`** / **`left_ir_downscale`** /**`right_ir_downscale`**
  * 设置下采样倍数。可用`ros2 run orbbec_camera list_camera_profile_mode_node`查看可设置分辨率。**默认值：** `1`
    > 仅支持Gemini 305。
    >
*   **`enable_ptp_config`**
    *   启用PTP时间同步。仅适用于Gemini 335Le。需要 `enable_sync_host_time` 设置为 `false`。

#### 视差

*   **`disparity_to_depth_mode`**
    *   `HW`: 使用硬件视差到深度转换。`SW`: 使用软件视差到深度转换。
*   **`disparity_range_mode`**, **`disparity_search_offset`**, **`disparity_offset_config`**
    *   视差搜索偏移参数。

> 用于[视差搜索偏移](../5_advanced_guide/configuration/disparity_search_offset.md)。

#### 交替AE模式

*   **`interleave_ae_mode`**
    *   设置`laser`或`hdr`交替。
*   **`interleave_frame_enable`**, **`interleave_skip_enable`**, **`interleave_skip_index`**
    *   控制交替帧模式的参数。
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   在交替帧模式下，设置hdr或激光交替帧的第0帧和第1帧参数。

*所有交替参数用于[交替ae模式](../5_advanced_guide/configuration/interleave_ae_mode.md)*。

#### 相机内同步

* **`depth_registration`**
  * 启用深度帧与彩色帧的对齐。当 `enable_colored_point_cloud` 设置为 `true` 时，该参数为必填项。
* **`align_mode`**
  * 设置对齐模式。可选值：`HW`（硬件对齐）、`SW`（软件对齐）。
* **`align_target_stream`**
  * 设置对齐的目标流类型。
  * 可选值：`COLOR`、`DEPTH`。
  * `COLOR`：将深度图对齐到彩色图。
  * `DEPTH`：将彩色图对齐到深度图。
* **`intra_camera_sync_reference`**
  * 设置相机内部同步的参考点。当 `sync_mode` 设置为 **software** 或 **hardware trigger** 模式时，适用于 Gemini 330 系列设备。
  * 可选值：`Start`、`Middle`、`End`。
  * 默认值：`Middle`。


### Basic & General Parameters

#### Firmware & Backend

* **`preset_firmware_path`**
  * 预置固件路径输入参数。如果输入多个路径，需要使用英文逗号 `,` 分隔，最多支持输入 3 个固件路径。
* **`uvc_backend`**
  * UVC 后端类型，可选值：`v4l2`、`libuvc`。
* **`connection_delay`**
  * 重新打开设备前的延迟时间（单位：毫秒）。部分设备（如 Astra mini）初始化时间较长，热插拔时如果立即重新打开设备，可能会导致固件崩溃。
* **`retry_on_usb3_detection_failure`**
  * 当相机连接在 USB 2.0 端口上且未被检测到时，系统将最多尝试重置相机三次。若使用 USB 2.0 连接，建议将该参数设置为 `false`，以避免不必要的重置操作。

#### TF, Extrinsics & Calibration

* **`publish_tf`** / **`tf_publish_rate`**
  * 启用 TF 发布并设置其发布频率。
* **`ir_info_uri`** / **`color_info_uri`**
  * 设置红外（IR）/彩色相机 CameraInfo 的 URL 地址。


#### Time Synchronization

* **`enable_sync_host_time`**
  * 启用主机时间与相机时间的同步。默认值为 `true`；如果使用全局时间（global time），请设置为 `false`。
* **`time_domain`**
  * 选择时间戳类型：`device`（设备时间）、`global`（全局时间）或 `system`（系统时间）。
* **`enable_frame_sync`**
  * 启用帧同步功能。


#### Logging & Diagnostics

*   **`log_level`**
    *   SDK log level. Default is `info`. Optional values: `debug`, `info`, `warn`, `error`, `fatal`.
*   **`log_file_name`**
    *   保存的SDK日志文件名。当`log_level`为`debug`时生效。
*   **`diagnostics_frequency`**
    *   Diagnostic period in seconds.
*   **`enable_heartbeat`**
    *   启用心跳功能。默认为`false`。如果设为`true`，相机节点将向固件发送心跳信号。

#### 其他设置

*   **`config_file_path`**
    *   YAML配置文件的路径。默认为`""`。如果未指定，将使用启动文件中的默认参数。
*   **`frame_aggregate_mode`**
    *   设置帧聚合输出模式。可选值：`full_frame`、`color_frame`、`ANY`、`disable`。
*   **`enable_d2c_viewer`**
    *   发布D2C叠加图像（仅用于测试）。

### IMU参数

*   **`enable_accel`** / **`enable_gyro`**
    *   启用加速度计/陀螺仪并输出其信息话题数据。
*   **`enable_sync_output_accel_gyro`**
    *   启用同步`accel_gyro`，并输出IMU话题实时数据。
*   **`accel_rate`** / **`gyro_rate`**
    *   加速度计/陀螺仪的频率。值范围从`1.5625hz`到`32khz`。
*   **`accel_range`** / **`gyro_range`**
    *   加速度计的范围（`2g`、`4g`、`8g`、`16g`）和陀螺仪的范围（`16dps`到`2000dps`）。
*   **`linear_accel_cov`**
    *   线性加速度的协方差。

### 深度滤波器

*   **`enable_decimation_filter`**
    *   启用深度抽取滤波器。使用`decimation_filter_scale`进行设置。
*   **`enable_hdr_merge`**
    *   启用深度HDR合并滤波器。使用`hdr_merge_exposure_1`等进行设置。
*   **`enable_sequenced_filter`**
    *   启用深度序列ID滤波器。使用`sequence_id_filter_id`进行设置。
*   **`enable_threshold_filter`**
    *   启用深度阈值滤波器。使用`threshold_filter_max`、`threshold_filter_min`进行设置。
*   **`enable_hardware_noise_removal_filter`**
    *   启用深度硬件噪声移除滤波器。
*   **`enable_noise_removal_filter`**
    *   启用深度软件噪声移除滤波器。使用`noise_removal_filter_min_diff`等进行设置。
*   **`enable_spatial_filter`**
    *   启用深度空间滤波器。使用`spatial_filter_alpha`等进行设置。
*   **`enable_temporal_filter`**
    *   启用深度时间滤波器。使用`temporal_filter_diff_threshold`等进行设置。
*   **`enable_hole_filling_filter`**
    *   启用深度空洞填充滤波器。使用`hole_filling_filter_mode`进行设置。

---

> **_重要_**：请仔细阅读[此链接](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/)关于软件滤波设置的说明。如果不确定，请不要修改这些设置。
