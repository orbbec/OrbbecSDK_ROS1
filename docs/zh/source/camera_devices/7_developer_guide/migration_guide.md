# 从 main 迁移到开源 v2-main 分支

## 简介

最初，奥比中光提供了一个**闭源 SDK —— Orbbec SDK v1**，它是 [OrbbecSDK ROS1 Wrapper main 分支](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main) 的基础。虽然 ROS 封装层本身是开源的，但底层 SDK 是闭源的，这种架构限制了灵活性，也阻碍了社区驱动的改进。

随着开发者对透明性、可维护性和更广泛设备支持的需求不断增长，奥比中光发布了全新的开源 v2-main —— **Orbbec SDK_v2** ([GitHub链接](https://github.com/orbbec/OrbbecSDK/tree/v2-main))。基于该 SDK，[OrbbecSDK ROS1 的 v2-main 分支](https://github.com/orbbec/OrbbecSDK_ROS1)现已完全开源，具备更强的可扩展性，并与奥比中光未来产品路线图保持一致。

本文档介绍了将 ROS 包从 main 分支（基于 SDK v1）迁移到**开源 v2-main 分支**（基于 Orbbec SDK_v2）的动机和优势，重点对比了**启动文件、参数、话题和服务**的主要差异，并提供迁移指南，帮助开发者顺利过渡。

**注意：**下文中，**main** 指闭源分支，**v2-main** 指开源分支。

## 迁移到 v2-main 的优势

2024年10月，奥比中光发布了重大更新：**OrbbecSDK ROS1 Wrapper v2**，完全基于开源 Orbbec SDK_v2。与旧版 main 分支（SDK v1.x）相比，v2-main 分支（Orbbec SDK_v2.x）具备更高的灵活性和可扩展性，并全面支持所有符合 UVC 标准的奥比中光 USB 产品。main --> v2-main 的迁移带来以下关键优势：

### **全面设备支持**

v2-main 分支支持所有符合 UVC 标准的奥比中光 USB 相机，并将成为未来新设备的主要支持平台。

### **透明与可扩展性**

Orbbec SDK_v2 完全开源，开发者可直接访问底层实现，便于调试、优化和二次开发。而 SDK v1 为闭源，存在“黑盒”限制。

### **维护与更新优势**

v2-main 分支提供完整功能支持，包括新特性开发、性能优化和 bug 修复。main 分支已进入仅维护模式，仅修复关键 bug，不再开发新功能。

### **社区与生态支持**

开源 SDK 允许开发者直接在 GitHub 或 Gitee 提交 issue 和 pull request，推动功能演进，加速问题解决，促进更开放活跃的奥比中光生态。

## main 与 v2-main 分支对比

### **启动文件差异**

1. v2-main 新增 Gemini 330 系列低功耗启动文件：
   - `gemini_330_series_low_cpu.launch`
   - `gemini_330_series_nodelet_low_cpu.launch`
2. v2-main 支持 **Gemini 210** 和 **Gemini 435Le** 相机。
3. 由于 **OrbbecSDK_v2 仅支持 UVC 设备**，v2-main 支持的相机型号略少于 main，详见下表。

| **相机系列**   | **main**                                                     | **v2-main**                                                  |
| -------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Gemini (330系列) | gemini_330_series.launch                                     | gemini_330_series.launch<br>gemini_330_series_low_cpu.launch<br>gemini_330_series_nodelet_low_cpu.launch<br>gemini_330_series_nodelet.launch |
| Gemini2系列      | gemini2.launch<br>gemini2L.launch<br>gemini2XL.launch<br>gemini2_nodelet.launch | gemini2.launch<br>gemini2L.launch                                |
| Gemini (E/EW/UW) | gemini_e.launch<br>gemini_e_lite.launch<br>gemini_ew.launch<br>gemini_ew_lite.launch<br>gemini_uw.launch | 不支持                                                |
| Gemini 其它      | 不支持                                                | gemini210.launch<br>gemini435_le.launch                          |
| Femto           | femto_bolt.launch<br>femto_mega.launch<br>femto.launch               | femto_bolt.launch<br>femto_mega.launch<br>femto.launch               |
| Astra           | astra_adv.launch<br>astra_embedded_s.launch<br>astra_pro2.launch<br>astra_stereo_u3.launch<br>astra.launch<br>astra2.launch | astra.launch<br>astra2.launch                                    |
| Dabai           | dabai_d1.launch<br>dabai_dcl.launch<br>dabai_dcw.launch<br>dabai_dcw2.launch<br>dabai_dw.launch<br>dabai_dw2.launch<br>dabai_max_pro.launch<br>dabai_max.launch<br>dabai_pro.launch<br>dabai.launch | dabai_a.launch<br>dabai_al.launch                                |
| Deeya           | deeya.launch                                                 | 不支持                                                |
| 多相机          | multi_camera.launch                                          | multi_camera.launch<br>multi_camera_nodelet.launch<br>multi_camera_synced.launch |
| 调试/通用        | ob_camera.launch<br>ob_camera_gdb.launch                         | ob_camera.launch<br>ob_camera_gdb.launch                         |

### **参数差异**

1. **参数变更（main --> v2-main）：**
   - enable_hardware_d2d --> disparity_to_depth_mode
   - laser_on_off_mode ---> enable_laser
   - color_backlight_compensation --> enable_color_backlight_compensation
2. **v2-main 新增参数（main 不具备）**

| **参数名**                      | **main** | **v2-main** | **说明**                                              |
| ------------------------------- | -------- | ----------- | ----------------------------------------------------- |
| uvc_backend                     | -        | 新增        | 选择 UVC 后端实现（如 V4L2/LibUVC，依平台而定）        |
| depth_precision                 | -        | 新增        | 设置深度数据精度模式（如毫米、亚毫米）                 |
| depth_delay_us                  | -        | 新增        | 深度图像采集延迟（微秒）                              |
| color_delay_us                  | -        | 新增        | 彩色图像采集延迟（微秒）                              |
| trigger2image_delay_us          | -        | 新增        | 外部触发信号到图像采集的延迟                          |
| trigger_out_delay_us            | -        | 新增        | 采集到输出触发信号的延迟                              |
| trigger_out_enabled             | -        | 新增        | 启用外部触发输出                                      |
| frames_per_trigger              | -        | 新增        | 每次触发采集的帧数                                    |
| software_trigger_period         | -        | 新增        | 软件触发模式下的触发周期                              |
| enable_ptp_config               | -        | 新增        | 启用 PTP（精确时间协议）同步                          |
| upgrade_firmware                | -        | 新增        | 固件升级开关                                          |
| preset_firmware_path            | -        | 新增        | 固件升级预设文件路径                                  |
| config_file_path                | -        | 新增        | 配置文件路径                                          |
| load_config_json_file_path      | -        | 新增        | 从 JSON 文件加载相机配置                              |
| export_config_json_file_path    | -        | 新增        | 导出相机配置到 JSON 文件                              |
| enumerate_net_device            | -        | 新增        | 枚举可用网络设备（以太网相机）                        |
| ip_address                      | -        | 新增        | 设备 IP 地址                                          |
| port                            | -        | 新增        | 网络端口号                                            |
| exposure_range_mode             | -        | 新增        | 曝光范围模式（自动/手动）                             |
| ir_info_uri                     | -        | 新增        | 红外相机参数文件路径（yaml/ini）                      |
| color_info_uri                  | -        | 新增        | 彩色相机参数文件路径                                  |
| color_mirror                    | -        | 新增        | 彩色图像水平镜像                                      |
| color_flip                      | -        | 新增        | 彩色图像垂直翻转                                      |
| enable_color_auto_exposure_priority | -    | 新增        | 启用彩色图像自动曝光优先级                            |
| enable_color_backlight_compensation | -    | 新增        | 启用彩色图像背光补偿                                  |
| color_powerline_freq            | -        | 新增        | 电源频率（50Hz/60Hz）防止闪烁                         |
| enable_color_decimation_filter  | -        | 新增        | 启用彩色图像降采样滤波器                              |
| color_decimation_filter_scale   | -        | 新增        | 彩色降采样滤波器缩放因子                              |
| depth_flip                      | -        | 新增        | 深度图像垂直翻转                                      |
| depth_mirror                    | -        | 新增        | 深度图像水平镜像                                      |
| enable_depth_auto_exposure_priority | -    | 新增        | 启用深度图像自动曝光优先级                            |
| mean_intensity_set_point        | -        | 新增        | 自动曝光目标亮度                                      |
| left_ir_flip                    | -        | 新增        | 左红外图像垂直翻转                                    |
| left_ir_mirror                  | -        | 新增        | 左红外图像水平镜像                                    |
| enable_left_ir_sequence_id_filter | -      | 新增        | 启用左红外帧序列号过滤                                |
| left_ir_sequence_id_filter_id   | -        | 新增        | 左红外指定序列号过滤                                  |
| right_ir_flip                   | -        | 新增        | 右红外图像垂直翻转                                    |
| right_ir_mirror                 | -        | 新增        | 右红外图像水平镜像                                    |
| enable_right_ir_sequence_id_filter | -     | 新增        | 启用右红外帧序列号过滤                                |
| right_ir_sequence_id_filter_id  | -        | 新增        | 右红外指定序列号过滤                                  |
| ir_brightness                   | -        | 新增        | 红外图像亮度调节                                      |
| align_target_stream             | -        | 新增        | 点云对齐目标流（如彩色/深度）                         |
| enable_hardware_noise_removal_filter | -   | 新增        | 启用硬件去噪滤波器                                    |
| enable_spatial_fast_filter      | -        | 新增        | 启用快速空间滤波器                                    |
| enable_spatial_moderate_filter  | -        | 新增        | 启用中等空间滤波器                                    |
| hardware_noise_removal_filter_threshold | - | 新增        | 硬件去噪滤波器阈值                                    |
| spatial_fast_filter_radius      | -        | 新增        | 快速空间滤波器半径                                    |
| spatial_moderate_filter_diff_threshold | - | 新增        | 中等空间滤波器差分阈值                                |
| spatial_moderate_filter_magnitude | -     | 新增        | 中等空间滤波器强度                                    |
| spatial_moderate_filter_radius  | -        | 新增        | 中等空间滤波器半径                                    |
| decimation_filter_scale_range   | -        | 新增        | 降采样缩放范围                                        |
| interleave_ae_mode              | -        | 新增        | 交替模式下自动曝光模式                                |
| interleave_frame_enable         | -        | 新增        | 启用交替帧模式                                        |
| interleave_skip_enable          | -        | 新增        | 交替模式下启用跳帧                                    |
| interleave_skip_index           | -        | 新增        | 跳帧索引                                              |
| hdr_index1_*                    | -        | 新增        | HDR模式配置（通道1参数）                              |
| hdr_index0_*                    | -        | 新增        | HDR模式配置（通道0参数）                              |
| laser_index1_*                  | -        | 新增        | 激光配置（通道1参数）                                 |
| laser_index0_*                  | -        | 新增        | 激光配置（通道0参数）                                 |
| enable_accel_data_correction    | -        | 新增        | 启用加速度计数据校正                                  |
| enable_gyro_data_correction     | -        | 新增        | 启用陀螺仪数据校正                                    |
| linear_accel_cov                | -        | 新增        | 线性加速度协方差配置                                  |
| disparity_range_mode            | -        | 新增        | 视差范围模式                                          |
| disparity_search_offset         | -        | 新增        | 视差搜索偏移                                          |
| disparity_offset_config         | -        | 新增        | 视差偏移配置                                          |
| offset_index0                   | -        | 新增        | 视差偏移索引0                                         |
| offset_index1                   | -        | 新增        | 视差偏移索引1                                         |
| force_ip_enable                 | -        | 新增        | 启用强制IP配置                                        |
| force_ip_mac                    | -        | 新增        | 强制IP的MAC地址                                       |
| force_ip_dhcp                   | -        | 新增        | 强制IP启用DHCP                                        |
| force_ip_address                | -        | 新增        | 强制分配IP地址                                        |
| force_ip_subnet_mask            | -        | 新增        | 强制分配子网掩码                                      |
| force_ip_gateway                | -        | 新增        | 强制分配网关                                          |

### **话题差异**

**v2-main** 新增如下话题：

| **话题**             | **main** | **v2-main** | **说明**                                              |
| --------------------- | -------- | ----------- | ----------------------------------------------------- |
| /camera/device_status | -        | 新增        | 发布设备状态（帧率延迟、设备连接状态等）              |

### **服务差异**

**v2-main** 新增如下服务：

| **服务**           | **main** | **v2-main** | **说明**                     |
| --------------------- | -------- | ----------- | ---------------------------- |
| get_ptp_config        | -        | 新增        | 获取PTP配置                  |
| set_ptp_config        | -        | 新增        | 设置PTP配置                  |
| set_color_ae_roi      | -        | 新增        | 设置彩色流自动曝光ROI        |
| set_color_flip        | -        | 新增        | 设置彩色流翻转               |
| set_color_rotation    | -        | 新增        | 设置彩色流旋转               |
| set_depth_ae_roi      | -        | 新增        | 设置深度流自动曝光ROI        |
| set_depth_flip        | -        | 新增        | 设置深度流翻转               |
| set_depth_rotation    | -        | 新增        | 设置深度流旋转               |
| set_right_ir_flip     | -        | 新增        | 设置右红外流翻转             |
| set_right_ir_ae_roi   | -        | 新增        | 设置右红外流自动曝光ROI      |
| set_right_ir_rotation | -        | 新增        | 设置右红外流旋转             |
| set_left_ir_flip      | -        | 新增        | 设置左红外流翻转             |
| set_left_ir_ae_roi    | -        | 新增        | 设置左红外流自动曝光ROI      |
| set_left_ir_rotation  | -        | 新增        | 设置左红外流旋转             |
| read_customer_data    | -        | 新增        | 读取用户自定义数据           |
| write_customer_data   | -        | 新增        | 写入用户自定义数据           |
| set_filter            | -        | 新增        | 配置深度/点云滤波器          |

