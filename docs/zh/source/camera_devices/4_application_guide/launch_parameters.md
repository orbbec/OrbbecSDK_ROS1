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
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   传感器流的分辨率和帧率。
*   **`enable_point_cloud`**
    *   启用点云。
*   **`enable_colored_point_cloud`**
    *   启用RGB点云。
*   **`ordered_pc`**
    *   启用无效点云的过滤。

### Sensor Controls

#### Color Stream

*   **`enable_color_auto_exposure`**
    *   Enable the Color auto exposure.
*   **`enable_color_auto_exposure_priority`**
    *   Enable the Color auto exposure priority.
*   **`color_exposure`**
    *   Set the Color exposure.
*   **`color_gain`**
    *   Set the Color gain.
*   **`enable_color_auto_white_balance`**
    *   Enable the Color auto white balance.
*   **`color_white_balance`**
    *   Set the Color white balance.
*   **`color_ae_max_exposure`**
    *   Set the maximum exposure value for Color auto exposure.
*   **`color_brightness`**, **`color_sharpness`**, **`color_gamma`**, **`color_saturation`**, **`color_contrast`**, **`color_hue`**
    *   Set the Color brightness, sharpness, gamma, saturation, contrast, and hue.
*   **`color_backlight_compensation`**
    *    Enables the color camera’s backlight compensation feature. Range: `0–6`, Default: `3`.
*   **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
    *   Enable the Color decimation filter and set its scale.
*   **`color_ae_roi_[left|right|top|bottom]`**
    *   设置彩色自动曝光ROI区域。

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

*   **`device_preset`**
    *   默认值为`Default`。仅支持G330系列。更多信息请参考[G330文档](https://www.orbbec.com/docs/g330-use-depth-presets/)。该值应为[表格中](../5_advanced_guide/configuration/predefined_presets.md)列出的预设名称之一。

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

- **`depth_registration`**
  *   Enable alignment of the depth frame to the color frame. This field is required when the `enable_colored_point_cloud` is set to `true`.
- **`align_mode`**
  *   The alignment mode to be used. Options are `HW` for hardware alignment and `SW` for software alignment.
- **`align_target_stream`**
  *   Set align target stream mode.
  *   The possible values are `COLOR`, `DEPTH`.
  *   `COLOR`: Align depth to color.
  *   `DEPTH`: Align color to depth.
- **`intra_camera_sync_reference`**
  - Sets the reference point for intra-camera synchronization. Applicable for Gemini 330 series devices when `sync_mode` is set to **software** or **hardware trigger** mode. **Options:** `Start`, `Middle`, `End`. **Default:** `Middle`

### Basic & General Parameters

#### Firmware & Backend

*   **`preset_firmware_path`**
    *   The input parameter is the preset firmware path. If multiple paths are input, each path needs to be separated by `,` and a maximum of 3 firmware paths can be input.
*   **`uvc_backend`**
    *   Optional values: `v4l2`, `libuvc`.
*   **`connection_delay`**
    *   The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
*   **`retry_on_usb3_detection_failure`**
    *   If the camera is connected to a USB 2.0 port and is not detected, the system will attempt to reset the camera up to three times. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.

#### TF, Extrinsics & Calibration

*   **`publish_tf`** / **`tf_publish_rate`**
    *   Enable the TF publish and set its publication rate.
*   **`ir_info_uri`** / **`color_info_uri`**
    *   Set URL of the IR/color camera info.

#### Time Synchronization

*   **`enable_sync_host_time`**
    *   Enable synchronization of the host time with the camera time. The default value is `true`. If using global time, set to `false`.
*   **`time_domain`**
    *   Select timestamp type: `device`, `global`, and `system`.
*   **`enable_ptp_config`**
    *   Enable PTP time synchronization.
*   **`enable_frame_sync`**
    *   Enable the frame synchronization.

#### Logging & Diagnostics

*   **`log_level`**
    *   SDK log level. Default is `info`. Optional values: `debug`, `info`, `warn`, `error`, `fatal`.
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
