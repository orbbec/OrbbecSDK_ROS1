# 启动参数

> 如果您不确定如何设置参数，可以连接 Orbbec 相机并打开 [OrbbecViewer](https://github.com/orbbec/OrbbecSDK/releases)，也可以查看第 1 章中的 [设备规格书](../1_overview/introduction.md)。

## 如何修改启动参数

启动参数可以通过两种方式修改：

1. **在启动命令中临时覆盖参数**

   适合调试、验证某个参数或只在本次启动中生效的场景。格式为 `参数名:=参数值`，可以在同一条命令中追加多个参数。

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch camera_name:=camera_02
   ```

   示例：同时修改相机名称并启用点云。

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch camera_name:=camera_02 enable_point_cloud:=true
   ```

2. **在 launch 文件中修改默认值**

   适合将参数作为长期默认配置使用，例如团队固定使用某个分辨率、帧率、相机名称或同步模式。各设备的 launch 文件位于 [launch](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/launch)，请选择与设备型号对应的 `*.launch` 文件。

   例如，将相机名称的默认值修改为 `camera_02`：

   ```xml
   <arg name="camera_name" default="camera_02"/>
   ```

   如果您是从源码构建，修改 launch 文件后通常需要回到工作空间重新编译并重新 source：

   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

   若使用 apt/deb 安装，不建议直接修改系统安装目录中的 launch 文件；建议优先使用命令行覆盖参数，或复制 launch 文件后维护自己的启动配置。

以下是可用的启动参数：

## 核心与数据流配置

*   **`camera_name`**
    *   启动节点的命名空间。
*   **`serial_number`**
    *   相机的序列号。当使用多个相机时需要此参数。多相机启动方式参考 [多相机](../5_advanced_guide/multi_camera/multi_camera.md)。
*   **`usb_port`**
    *   相机的USB端口。当使用多个相机时需要此参数。多相机启动方式参考 [多相机](../5_advanced_guide/multi_camera/multi_camera.md)。
*   **`device_num`**
    *   设备数量。如果需要多个相机，必须填写此参数。多相机启动方式参考 [多相机](../5_advanced_guide/multi_camera/multi_camera.md)。
*   **`device_preset`**
    *   深度预设。可选预设和推荐场景请参考 [设备预设](../5_advanced_guide/configuration/predefined_presets.md)。可以使用下面命令查看可设置模式；该工具会同时打印 preset 列表和 preset 版本信息。
    ```bash
    rosrun orbbec_camera list_devices_node
    ```
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   传感器流的分辨率和帧率。
    *   Femto Mega / Femto Bolt 的深度 NFOV、WFOV 模式通过深度和 IR 分辨率组合配置，参考 [深度 NFOV 和 WFOV 模式配置](../5_advanced_guide/configuration/configuration_of_depth_NFOV_and_WFOV_modes.md)。
    *   如需降低 CPU 使用率，可参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md) 中的 `color_format` 建议。
*   **`enable_[color|depth|left_ir|right_ir|ir]`**
    *   启用或关闭对应图像流。
*   **`[color|depth|left_ir|right_ir|ir]_rotation`**
    *   设置流图像旋转。
    *   可能的值为 `0`、`90`、`180`、`270`。
*   **`[color|depth|left_ir|right_ir|ir]_flip`**
    *   启用流图像翻转。
*   **`[color|depth|left_ir|right_ir|ir]_mirror`**
    *   启用流图像镜像。
*   **`enable_point_cloud`**
    *   启用点云。使用和 RViz 可视化方法参考 [点云](point_cloud.md)。
*   **`enable_colored_point_cloud`**
    *   启用RGB点云。使用和 RViz 可视化方法参考 [点云](point_cloud.md)。
*   **`cloud_frame_id`**
    *   修改ros消息中的 `frame_id` 名称。
*   **`ordered_pc`**
    *   启用无效点云过滤。
*   **`point_cloud_qos`、`[stream]_qos`、`[stream]_camera_info_qos`**
    *   这些 QoS 参数在 ROS1 wrapper 中不适用。ROS1 中话题通信行为主要由发布者/订阅者队列长度控制。
* **`point_cloud_decimation_filter_factor`**
  * 点云下采样因子。范围：`1–8`，`1`表示不下采样，数值越大下采样倍数越大。
* **`enable_image_transport_plugins`**
  * 启用 ROS `image_transport` 插件发布普通图像流。默认值：`true`。设置为 `false` 时，普通图像流仅使用原始 `sensor_msgs/Image` 发布；MJPG 彩色流仍会额外发布 `/compressed` 话题。压缩图像订阅方法参考 [压缩图像](compressed_image.md)。
* **`bag_record_filename`**
  * 启动后使用 SDK 录制设备数据到指定 `.bag` 文件。为空时不自动录制。开始录制时会同时导出同名 JSON preset 文件，例如 `record.bag` 对应 `record.json`。
* **`bag_filename`**
  * 使用 SDK 回放指定 `.bag` 文件。设置后节点从 bag 文件创建回放设备，而不是连接真实相机。
* **`bag_loop`**
  * SDK bag 回放结束后是否循环播放。默认值：`false`。仅在设置 `bag_filename` 时生效。

## 传感器控制

### 彩色流
*   **`enable_color_auto_exposure`**
    *   启用彩色自动曝光。
*   **`enable_color_auto_exposure_priority`**
    *   启用彩色自动曝光优先级。
*   **`color_exposure`**
    *   设置彩色曝光。
*   **`color_gain`**
    *   设置彩色增益。
*   **`enable_color_auto_white_balance`**
    *   启用彩色自动白平衡。
*   **`color_white_balance`**
    *   设置彩色白平衡。
*   **`color_ae_max_exposure`**
    *   设置彩色自动曝光的最大曝光值。
*   **`color_ae_max_gain`**
    *   设置彩色自动曝光的最大增益。Gemini 2 固件 `1.5.04` 及以上、Gemini 2L 固件 `1.5.09` 及以上支持。**范围：** `16–112`。
*   **`color_brightness`**、**`color_sharpness`**、**`color_gamma`**、**`color_saturation`**、**`color_contrast`**、**`color_hue`**
    *   设置彩色亮度、锐度、伽马、饱和度、对比度和色调。
*   **`color_backlight_compensation`**
    *   启用彩色相机的背光补偿功能。**范围**：`0–6`，**默认值**：`3`。
*   **`color_powerline_freq`**
    *   设置电源线频率。可能的值为 `disable`、`50hz`、`60hz`、`auto`。
*   **`color_preset`**
    *   通过名称设置彩色 preset。Gemini 330 系列和 Gemini 301 系列设备支持。常见可选值包括 `Default`、`Warm Biased AWB`、`Cold Biased AWB`，具体列表以设备返回为准。名称匹配大小写不敏感。
*   **`color_anti_flicker`**
    *   启用彩色防闪烁功能。Gemini 330 系列固件 `1.7.13` 及以上、Gemini 301 系列固件 `1.0.54` 及以上支持。
*   **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
    *   启用彩色抽取滤波器并设置其比例。
*   **`color_ae_roi_[left|right|top|bottom]`**
    *   设置彩色自动曝光ROI。
*   **`color_denoising_level`**
    *   启用 ISP 彩色降噪功能。**范围：** `0–8`，`0` 表示自动。支持 Gemini 2 固件 `1.5.04` 及以上、Gemini 2L 固件 `1.5.09` 及以上；该功能需要开启彩色自动曝光并依赖新固件支持。


### 深度流
*   **`enable_depth_auto_exposure_priority`**
    *   启用深度自动曝光优先级。
* **`mean_intensity_set_point`**
  * 设置开启自动曝光时深度图像的目标平均强度。例如：`mean_intensity_set_point:=100`。
  > **注意：** 在 2.4.7 版本之后，该参数取代已弃用的 `depth_brightness`，但为了向后兼容，仍会支持 `depth_brightness`。
*   **`enable_depth_scale`**
    *   设置D2C后是否启用深度缩放。`true`表示启用，默认为`true`。
*   **`depth_precision`**
    *   深度精度应为 `1mm` 格式。默认值为 `1mm`。
*   **`depth_work_mode`**
    *   设置深度工作模式。支持的设备、模式列表查看命令和启动示例参考 [深度工作模式切换](../5_advanced_guide/configuration/depth_work_mode_switch.md)。
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   设置深度自动曝光ROI。

### 红外流
*   **`enable_ir_auto_exposure`**
    *   启用红外自动曝光。
*   **`ir_exposure`** / **`ir_gain`**
    *   设置红外曝光和增益。
*   **`ir_ae_max_exposure`**
    *   设置红外自动曝光的最大曝光值。
*   **`ir_brightness`**
    *   设置开启自动曝光时ir图像的目标平均强度。

### 激光 / LDP
*   **`enable_laser`**
    *   启用激光。默认值为 `true`。
*   **`laser_energy_level`**
    *   设置激光能量级别。
*   **`enable_ldp`** / **`ldp_power_level`**
    *   启用LDP并设置其功率级别。

## 设备、同步与高级功能

### 多相机同步
*   **`sync_mode`**
    *   设置同步模式。默认值为 `standalone`。多相机连接、同步模式和触发配置参考 [多相机同步](../5_advanced_guide/multi_camera/multi_camera_synced.md)。
*   **`depth_delay_us`** / **`color_delay_us`**
    *   接收捕获命令或触发信号后深度/彩色图像捕获的延迟时间（微秒）。
*   **`trigger2image_delay_us`**
    *   接收捕获命令或触发信号后图像捕获的延迟时间（微秒）。
*   **`trigger_out_delay_us`**
    *   接收捕获命令或触发信号后触发信号输出的延迟时间（微秒）。
*   **`trigger_out_enabled`**
    *   启用触发输出信号。
*   **`software_trigger_enabled`** / **`software_trigger_period`**
    *   启用软件触发输出信号 / 设置软件触发周期（毫秒）。
*   **`frames_per_trigger`**
    *   触发模式下每次触发后每个流的帧数。
*   **`sync_io_voltage_level`**
    *   设置同步 IO 电压等级。默认值为 `-1`，表示不设置。仅支持具备该属性的设备；可通过 `/camera/set_sync_io_voltage_level` 服务在运行时修改。

### 网络相机
* **`enumerate_net_device`**
  * 启用自动枚举网络设备。网络相机启动、指定 IP 和 Force IP 配置参考 [网络相机](../5_advanced_guide/configuration/net_camera.md)。
* **`ip_address`** / **`port`**
  * 设置网络设备的IP地址和端口（通常为 `8090`）。网络相机启动、指定 IP 和 Force IP 配置参考 [网络相机](../5_advanced_guide/configuration/net_camera.md)。
* **`force_ip_enable`**
  * 启用强制IP功能。**默认值：** `false`
* **`force_ip_mac`**
  * 连接多个相机时的目标设备MAC地址（例如，`"54:14:FD:06:07:DA"`）。您可以使用 `list_devices_node` 查找每个设备的MAC。**默认值：** `""`
* **`force_ip_address`**
  * 要分配的静态IP地址。**默认值：** `192.168.1.10`
* **`force_ip_subnet_mask`**
  * 静态IP的子网掩码。**默认值：** `255.255.255.0`
* **`force_ip_gateway`**
  * 静态IP的网关地址。**默认值：** `192.168.1.1`

### 视差
*   **`disparity_to_depth_mode`**
    *   `HW`：使用硬件视差到深度转换。`SW`：使用软件视差到深度转换。也可以设置为 `disable` 关闭。
    *   该参数大小写不敏感；非法值会报错并回退默认值。
*   **`disparity_range_mode`**、**`disparity_search_offset`**、**`disparity_offset_config`**
    *   视差搜索偏移参数。用于 [视差搜索偏移](../5_advanced_guide/configuration/disparity_search_offset.md)。

### 交错AE模式
*   **`interleave_ae_mode`**
    *   设置 `laser` 或 `hdr` 交错。
*   **`interleave_frame_enable`**、**`interleave_skip_enable`**、**`interleave_skip_index`**
    *   控制交错帧模式的参数。
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   在交错帧模式下，设置hdr或laser交错帧的第0和第1帧参数。
*   *所有交错参数用于 [交错ae模式](../5_advanced_guide/configuration/interleave_ae_mode.md)。*

### 相机内同步

- **`depth_registration`**
  *   启用深度帧与彩色帧的对齐。当 `enable_colored_point_cloud` 设置为 `true` 时需要此字段。启动和查看方法参考 [对齐深度到彩色](../5_advanced_guide/configuration/align_depth_color.md)。
- **`align_mode`**
  *   要使用的对齐模式。选项为 `HW`（硬件对齐）和 `SW`（软件对齐）。
  *   该参数大小写不敏感；非法值会报错并回退默认值。
- **`align_target_stream`**
  *   设置对齐目标流模式。
  *   可能的值为 `COLOR`、`DEPTH`。
  *   `COLOR`：将深度对齐到彩色。
  *   `DEPTH`：将彩色对齐到深度。
  *   该参数大小写不敏感。硬件 D2C 仅支持 `COLOR` 作为对齐目标；如需对齐到 `DEPTH`，请使用 `align_mode:=SW`。启动和查看方法参考 [对齐深度到彩色](../5_advanced_guide/configuration/align_depth_color.md)。
- **`intra_camera_sync_reference`**
  - 设置相机内同步的参考点。适用于Gemini 330系列设备，当 `sync_mode` 设置为**软件**或**硬件触发**模式时。**选项：** `Start`、`Middle`、`End`。设置为空时，长基线设备默认End，短基线设备默认Middle。

## 设备特定参数
*   **`enable_gmsl_trigger`** / **`gmsl_trigger_fps`**
    *   启用gmsl触发输出信号 / 设置gmsl触发fps。用于 [gmsl相机](../5_advanced_guide/multi_camera/gmsl_cameras.md)。
* **`enable_ptp_config`**
  * 启用PTP时间同步。仅适用于Gemini 335Le。需要 `enable_sync_host_time` 设置为 `false`。
  > **支持模组**：Gemini 335Le。
* **`preset_resolution_config`**
  * 摄像头设备的预设分辨率配置。格式: "width,height,ir_decimation_factor,depth_decimation_factor". Example: "1280,720,4,4". 留空禁用。
  > **支持模组**：Gemini 435Le。
* **`ae_reference_stream`**
  * 设置 Gemini 301 系列设备的 AE 参考流。可选值：`color`、`depth`。
  > **支持模组**：Gemini 301 系列。
* **`ae_strategy`**
  * 设置 Gemini 301 系列设备的 AE 策略。可选值：`default`、`motion`。
  > **支持模组**：Gemini 301 系列。
* **`depth_decimation_factor`** / **`left_ir_decimation_factor`** / **`right_ir_decimation_factor`**
  * 设置下采样倍数。可用`rosrun orbbec_camera list_camera_profile_mode_node`查看可设置分辨率。**默认值：** `1`
  > **支持模组**：Gemini 301 系列。
* **`enable_false_positive_filter`**
  * 启用鬼影滤波。可减少重影噪声。
  > **支持模组**：DaBaiA / DaBaiAL / Gemini 330 系列 / Gemini345 / Gemini345Lg。
* **`enable_edge_noise_removal_filter`**
  * 启用 EdgeNoiseRemovalFilter，用于减少深度图边缘噪声。
  > **支持模组**：DaBai Max Pro。
* **`enable_disp_outliers_filter`**
  * 启用 DispOutliersFilter，用于移除深度图中的视差离群点。
  > **支持模组**：DaBai Max Pro。
* **`disp_outliers_filter_search_mode`**
  * 设置 DispOutliersFilter 的搜索模式。留空表示使用 SDK 默认值。可选值：`FULL`、`OFFSET_80`，大小写不敏感。
  > **支持模组**：DaBai Max Pro。

## 基础与通用参数

### 固件与后端
*   **`upgrade_firmware`**
    *   输入参数为固件路径。新版本建议使用独立工具 `firmware_update_tool` 进行固件升级，参考 [firmware_update_tool 工具](../6_benchmark/firmware_update_tool.md)。
*   **`preset_firmware_path`**
    *   输入参数为预设固件路径。如果输入多个路径，每个路径需要用 `,` 分隔，最多可输入3个固件路径。新版本建议使用独立工具烧录 preset，参考 [firmware_update_tool 工具](../6_benchmark/firmware_update_tool.md)。
*   **`uvc_backend`**
    *   可选值：`v4l2`、`libuvc`。低 CPU 场景建议参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。
*   **`connection_delay`**
    *   重新打开设备的延迟时间（毫秒）。某些设备（如Astra mini）需要较长时间初始化，热插拔时立即重新打开设备可能导致固件崩溃。
*   **`retry_on_usb3_detection_failure`**
    *   如果相机连接到USB 2.0端口且未检测到，系统将尝试重置相机最多三次。使用USB 2.0连接时建议将此参数设置为 `false`，以避免不必要的重置。

### TF、外参与校准
*   **`publish_tf`** / **`tf_publish_rate`**
    *   启用TF发布并设置其发布速率。坐标系、TF 树查看和可视化方法参考 [坐标系和 TF 变换](coordinate_and_tf.md)。
*   **`enable_publish_extrinsic`**
    *   启用外参发布。
*   **`ir_info_url`** / **`color_info_url`**
    *   设置IR/彩色相机信息的URL。
*   **`enable_[color|depth|ir|left_ir|right_ir]_undistortion`**
    *   启用对应图像流的 SDK 去畸变滤波器。双 IR 设备使用 `enable_left_ir_undistortion` / `enable_right_ir_undistortion`，单 IR 设备使用 `enable_ir_undistortion`。

### 时间同步
*   **`enable_sync_host_time`**
    *   启用主机时间与相机时间的同步。默认值为 `true`。如果使用全局时间，设置为 `false`。
*   **`time_domain`**
    *   选择时间戳类型：`device`、`global` 和 `system`。
    *   该参数大小写不敏感；非法值会报错并回退默认值。
*   **`timestamp_clock_type`**
    *   设置 SDK 时间戳时钟类型。可选值：`realtime`、`monotonic`。默认使用 `realtime`。
* **`time_sync_period`**
  * 相机时间与主机系统同步的间隔（秒）。
  > **注意**：仅当 **`enable_sync_host_time = true`** 且 **`time_domain = device`** 时需要设置此参数。
*   **`enable_frame_drop_log`**
    *   启用帧丢失日志。日志会分别统计 SDK 接收阶段和 ROS 发布阶段检测到的丢帧。
*   **`frame_timestamp_csv_file`**
    *   帧时间戳统计 CSV 输出路径。为空时不写 CSV；如需保存 CSV，请指定文件路径，例如 `/tmp/frame_timestamp.csv`。
*   **`enable_frame_sync`**
    *   启用帧同步。

### 日志与诊断
*   **`log_level`**
    *   SDK日志级别。默认只输出设备当前状态，更多调试日志可通过 `debug` 开启。可选值：`debug`、`info`、`warn`、`error`、`fatal`。
    *   SDK 日志和崩溃文件保存在 `~/.ros/Log`；ROS1 运行日志保存在 `~/.ros/log/<run_id>`。
* **`log_file_name`**
  * 保存的SDK日志文件名。当`log_level`为`debug`时生效；实际路径通常为 `~/.ros/Log/<camera_name>/<log_file_name>`。多相机场景下，不同 `camera_name` 会分别写入各自目录。
*   **`diagnostic_period`**
    *   诊断周期（秒）。
*   **`enable_heartbeat`**
    *   启用心跳功能。默认为 `false`。如果为 `true`，相机节点将向固件发送心跳信号。
*   **`enable_firmware_log`**
    *   启用固件日志抓取。该开关与 `enable_heartbeat` 解耦。

### 其他
*   **`config_file_path`**
    *   YAML配置文件的路径。默认为 `""`。如果未指定，将使用启动文件中的默认参数。部分 preset 或特殊模式会通过 YAML 配置，示例参考 [设备预设](../5_advanced_guide/configuration/predefined_presets.md)。
*   **`load_config_json_file_path`**
    *   SDK JSON 配置导入路径。设置后节点会在初始化时调用 SDK 导入 JSON 配置。Gemini 330 系列可使用 `gemini_330_series_sdk_json.launch` 作为专用启动文件。
    *   如果 JSON 中包含 `application_config`，节点会在未被 launch 参数显式覆盖时同步其中的流开关、分辨率、帧率、格式、去畸变、点云、HDR 合并和设备级下采样配置。
*   **`export_config_json_file_path`**
    *   SDK JSON 配置导出路径。设置后节点会在初始化完成后将当前设备配置导出为 JSON。也可以通过 `/camera/export_config_json` 服务运行时导出。
    *   导出前会把当前 ROS 参数中的传感器流、点云和 HDR 合并配置同步到 SDK `application_config`（设备支持时）。
*   **`frame_aggregate_mode`**
    *   设置帧聚合输出模式。可选值：`full_frame`、`color_frame`、`ANY`、`disable`。
    *   该参数大小写不敏感；非法值会报错并回退默认值。
*   **`enable_d2c_viewer`**
    *   发布D2C叠加图像（仅用于测试）。使用示例参考 [对齐深度到彩色](../5_advanced_guide/configuration/align_depth_color.md)。

## IMU

*   **`enable_accel`** / **`enable_gyro`**
    *   启用加速度计/陀螺仪并输出其信息话题数据。
*   **`enable_sync_output_accel_gyro`**
    *   启用同步 `accel_gyro`，并输出IMU话题实时数据。
*   **`accel_rate`** / **`gyro_rate`**
    *   加速度计/陀螺仪的频率。值范围从 `1.5625hz` 到 `32khz`。
*   **`accel_range`** / **`gyro_range`**
    *   加速度计（`2g`、`4g`、`8g`、`16g`）和陀螺仪（`16dps` 到 `2000dps`）的范围。
*   **`enable_accel_data_correction`** / **`enable_gyro_data_correction`**
    *   启用加速度计/陀螺仪的数据校正。
*   **`linear_accel_cov`** / **`angular_vel_cov`**
    *   线性加速度和角速度的协方差。

## 深度滤波器

*   **`enable_decimation_filter`**
    *   启用深度抽取滤波器。使用 `decimation_filter_scale` 设置。
*   **`enable_hdr_merge`**
    *   启用深度hdr合并滤波器。使用 `hdr_merge_exposure_1` 等设置。
*   **`enable_sequence_id_filter`**
    *   启用深度序列id滤波器。使用 `sequence_id_filter_id` 设置。
*   **`enable_threshold_filter`**
    *   启用深度阈值滤波器。使用 `threshold_filter_max`、`threshold_filter_min` 设置。
*   **`enable_hardware_noise_removal_filter`**
    *   启用深度硬件降噪滤波器。低 CPU 配置建议参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。
*   **`enable_noise_removal_filter`**
    *   启用深度软件降噪滤波器。使用 `noise_removal_filter_min_diff` 等设置。低 CPU 配置建议参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。
*   **`enable_spatial_filter`**
    *   启用深度空间滤波器。使用 `spatial_filter_alpha` 等设置。低 CPU 配置建议参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。
*   **`enable_temporal_filter`**
    *   启用深度时间滤波器。使用 `temporal_filter_diff_threshold` 等设置。
*   **`enable_hole_filling_filter`**
    *   启用深度孔洞填充滤波器。使用 `hole_filling_filter_mode` 设置。
*   **`enable_spatial_fast_filter`**
    *   启用深度空间快速滤波器。使用 `spatial_fast_filter_radius` 设置。
*   **`enable_spatial_moderate_filter`**
    *   启用深度空间中等滤波器。使用 `spatial_moderate_filter_diff_threshold` 等设置。
*   **`enable_mgc_noise_removal_filter`**
    *   启用 MGC 降噪滤波器。适配 Astra Mini (S) Pro、DaBai Pro Max、DaBai DCW2 等 OpenNI 设备。
*   **`enable_lut_noise_removal_filter`**
    *   启用 LUT 降噪滤波器。适配 Astra Mini (S) Pro、DaBai Pro Max、DaBai DCW2 等 OpenNI 设备。

---

> **_重要_**：请仔细阅读 [此链接](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/) 中有关软件滤波设置的说明。如果不确定，请勿修改这些设置。
