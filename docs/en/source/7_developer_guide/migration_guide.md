# Migrating from main to Open-Source v2-main

## **Introduction**

Initially, Orbbec provided a **closed-source SDK — Orbbec SDK v1**, which formed the foundation of the [OrbbecSDK ROS1 Wrapper main branch](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main). Although the ROS wrapper layer itself was open-source, it relied on a closed-source underlying SDK. This architecture imposed limitations on flexibility and hindered community-driven improvements.

As developers increasingly demanded transparency, maintainability, and broader device support, Orbbec released a brand-new open-source v2-main **— Orbbec SDK_v2** ([GitHub link](https://github.com/orbbec/OrbbecSDK/tree/v2-main)). Based on this SDK, the [open-source v2-main branch of OrbbecSDK ROS1](https://github.com/orbbec/OrbbecSDK_ROS1) is now fully open source, offering greater extensibility and alignment with Orbbec’s future product roadmap.

This document introduces the motivations and benefits of migrating ROS packages from the main branch (based on SDK v1) to the **open-source v2-main** branch (based on Orbbec SDK_v2). It highlights the key differences in **launch files, parameters, topics, and services**, and provides a migration guide to help developers smoothly transition.    

**Note:** In the following content, **main** refers to the closed-source branch, while **v2-main** refers to the open-source branch.

## **Advantages of Migrating from main to v2-main**

In October 2024, Orbbec released a major update: **OrbbecSDK ROS1 Wrapper v2**, which is entirely based on the open-source Orbbec SDK_v2. Compared to the legacy main branch (SDK v1.x), the v2-main branch (Orbbec SDK_v2.x) provides greater flexibility and scalability, while offering comprehensive support for all Orbbec USB products that comply with the UVC standard. The migration from main --> v2-main brings the following key advantages:

### **Comprehensive Device Support**

The v2-main branch supports all UVC-compliant Orbbec USB cameras and will be the primary platform for supporting all newly released devices.

### **Transparency and Extensibility**

Orbbec SDK_v2 is fully open-source, allowing developers to directly access the underlying implementation for easier debugging, optimization, and secondary development. By contrast, SDK v1 was closed-source, introducing a “black-box” constraint.

### Advantages in Maintenance and Updates

The v2-main branch offers full-featured support, including new feature development, performance optimization, and bug fixes. The main branch has entered a maintenance-only mode, where only critical bugs may receive limited updates, and no new features are being developed.

### **Community and Ecosystem Support**

With an open-source SDK, developers can directly submit issues and pull requests on GitHub or Gitee, contributing to feature evolution. This not only accelerates problem resolution but also fosters a more open and active Orbbec ecosystem.

## Comparison Between main and v2-main Branches

### **Launch File Differences**

1. In v2-main, new low-power launch files have been added for the Gemini 330 series:
   - `gemini_330_series_low_cpu.launch`
   - `gemini_330_series_nodelet_low_cpu.launch`
2. v2-main introduces support for **Gemini 210** and **Gemini 435Le** cameras.
3. Since **OrbbecSDK_v2 only supports UVC devices**, the range of camera models supported in v2-main is slightly narrower than in main. Detailed information is provided in the comparison table below.

| **Camera Series**   | **main**                                                     | **v2-main**                                                  |
| ------------------- | ------------------------------------------------------------ | ------------------------------------------------------------ |
| Gemini (330 series) | gemini_330_series.launch                                     | gemini_330_series.launchgemini_330_series_low_cpu.launchgemini_330_series_nodelet_low_cpu.launchgemini_330_series_nodelet.launch |
| Gemini2 series      | gemini2.launchgemini2L.launchgemini2XL.launchgemini2_nodelet.launch | gemini2.launchgemini2L.launch                                |
| Gemini (E/EW/UW)    | gemini_e.launchgemini_e_lite.launchgemini_ew.launchgemini_ew_lite.launchgemini_uw.launch | Not supported                                                |
| Gemini Other        | Not supported                                                | gemini210.launchgemini435_le.launch                          |
| Femto               | femto_bolt.launchfemto_mega.launchfemto.launch               | femto_bolt.launchfemto_mega.launchfemto.launch               |
| Astra               | astra_adv.launchastra_embedded_s.launchastra_pro2.launchastra_stereo_u3.launchastra.launchastra2.launch | astra.launchastra2.launch                                    |
| Dabai               | dabai_d1.launchdabai_dcl.launchdabai_dcw.launchdabai_dcw2.launchdabai_dw.launchdabai_dw2.launchdabai_max_pro.launchdabai_max.launchdabai_pro.launchdabai.launch | dabai_a.launchdabai_al.launch                                |
| Deeya               | deeya.launch                                                 | Not supported                                                |
| Multi-camera        | multi_camera.launch                                          | multi_camera.launchmulti_camera_nodelet.launchmulti_camera_synced.launch |
| Debug/General       | ob_camera.launchob_camera_gdb.launch                         | ob_camera.launchob_camera_gdb.launch                         |

### **Parameter Differences**

1. **Parameter changes ( main --> v2-main )：**
   - enable_hardware_d2d --> disparity_to_depth_mode
   - laser_on_off_mode ---> enable_laser
   - color_backlight_compensation --> enable_color_backlight_compensation
2. **New Parameters in v2-main (not available in main)**

| **Parameter Name**                      | **main** | **v2-main** | **Description**                                              |
| --------------------------------------- | -------- | ----------- | ------------------------------------------------------------ |
| uvc_backend                             | -        | Added       | Select UVC backend implementation (e.g., V4L2/LibUVC depending on platform) |
| depth_precision                         | -        | Added       | Set depth data precision mode (e.g., millimeter, sub-millimeter) |
| depth_delay_us                          | -        | Added       | Acquisition delay for depth image (in microseconds)          |
| color_delay_us                          | -        | Added       | Acquisition delay for color image (in microseconds)          |
| trigger2image_delay_us                  | -        | Added       | Delay between external trigger signal and image acquisition  |
| trigger_out_delay_us                    | -        | Added       | Delay between acquisition and output trigger signal          |
| trigger_out_enabled                     | -        | Added       | Enable external trigger output                               |
| frames_per_trigger                      | -        | Added       | Number of frames per trigger                                 |
| software_trigger_period                 | -        | Added       | Trigger period in software trigger mode                      |
| enable_ptp_config                       | -        | Added       | Enable PTP (Precision Time Protocol) time synchronization    |
| upgrade_firmware                        | -        | Added       | Firmware upgrade switch                                      |
| preset_firmware_path                    | -        | Added       | Preset firmware file path for upgrade                        |
| config_file_path                        | -        | Added       | Configuration file path                                      |
| load_config_json_file_path              | -        | Added       | Load camera configuration from JSON file                     |
| export_config_json_file_path            | -        | Added       | Export camera configuration to JSON file                     |
| enumerate_net_device                    | -        | Added       | Enumerate available network devices (for Ethernet cameras)   |
| ip_address                              | -        | Added       | Device IP address                                            |
| port                                    | -        | Added       | Network port number                                          |
| exposure_range_mode                     | -        | Added       | Exposure range mode (Auto/Manual)                            |
| ir_info_uri                             | -        | Added       | IR camera parameter file path (yaml/ini)                     |
| color_info_uri                          | -        | Added       | Color camera parameter file path                             |
| color_mirror                            | -        | Added       | Horizontal mirror for color image                            |
| color_flip                              | -        | Added       | Vertical flip for color image                                |
| enable_color_auto_exposure_priority     | -        | Added       | Enable auto-exposure priority for color image                |
| enable_color_backlight_compensation     | -        | Added       | Enable backlight compensation for color image                |
| color_powerline_freq                    | -        | Added       | Power line frequency (50Hz/60Hz) for flicker reduction       |
| enable_color_decimation_filter          | -        | Added       | Enable decimation filter for color image                     |
| color_decimation_filter_scale           | -        | Added       | Scale factor for color decimation filter                     |
| depth_flip                              | -        | Added       | Vertical flip for depth image                                |
| depth_mirror                            | -        | Added       | Horizontal mirror for depth image                            |
| enable_depth_auto_exposure_priority     | -        | Added       | Enable auto-exposure priority for depth image                |
| mean_intensity_set_point                | -        | Added       | Target mean brightness for auto-exposure                     |
| left_ir_flip                            | -        | Added       | Vertical flip for left IR image                              |
| left_ir_mirror                          | -        | Added       | Horizontal mirror for left IR image                          |
| enable_left_ir_sequence_id_filter       | -        | Added       | Enable sequence ID filter for left IR frames                 |
| left_ir_sequence_id_filter_id           | -        | Added       | Specific sequence ID to filter for left IR                   |
| right_ir_flip                           | -        | Added       | Vertical flip for right IR image                             |
| right_ir_mirror                         | -        | Added       | Horizontal mirror for right IR image                         |
| enable_right_ir_sequence_id_filter      | -        | Added       | Enable sequence ID filter for right IR frames                |
| right_ir_sequence_id_filter_id          | -        | Added       | Specific sequence ID to filter for right IR                  |
| ir_brightness                           | -        | Added       | Adjust IR image brightness                                   |
| align_target_stream                     | -        | Added       | Target stream for point cloud alignment (e.g., color/depth)  |
| enable_hardware_noise_removal_filter    | -        | Added       | Enable hardware noise removal filter                         |
| enable_spatial_fast_filter              | -        | Added       | Enable fast spatial filter                                   |
| enable_spatial_moderate_filter          | -        | Added       | Enable moderate spatial filter                               |
| hardware_noise_removal_filter_threshold | -        | Added       | Threshold for hardware noise removal filter                  |
| spatial_fast_filter_radius              | -        | Added       | Radius for fast spatial filter                               |
| spatial_moderate_filter_diff_threshold  | -        | Added       | Differential threshold for moderate spatial filter           |
| spatial_moderate_filter_magnitude       | -        | Added       | Magnitude for moderate spatial filter                        |
| spatial_moderate_filter_radius          | -        | Added       | Radius for moderate spatial filter                           |
| decimation_filter_scale_range           | -        | Added       | Decimation scale range                                       |
| interleave_ae_mode                      | -        | Added       | Auto-exposure mode in interleave mode                        |
| interleave_frame_enable                 | -        | Added       | Enable interleaved frame mode                                |
| interleave_skip_enable                  | -        | Added       | Enable frame skipping in interleave mode                     |
| interleave_skip_index                   | -        | Added       | Index of skipped frames                                      |
| hdr_index1_*                            | -        | Added       | HDR mode configuration (channel 1 parameters)                |
| hdr_index0_*                            | -        | Added       | HDR mode configuration (channel 0 parameters)                |
| laser_index1_*                          | -        | Added       | Laser configuration (channel 1 parameters)                   |
| laser_index0_*                          | -        | Added       | Laser configuration (channel 0 parameters)                   |
| enable_accel_data_correction            | -        | Added       | Enable accelerometer data correction                         |
| enable_gyro_data_correction             | -        | Added       | Enable gyroscope data correction                             |
| linear_accel_cov                        | -        | Added       | Linear acceleration covariance configuration                 |
| disparity_range_mode                    | -        | Added       | Disparity range mode                                         |
| disparity_search_offset                 | -        | Added       | Disparity search offset                                      |
| disparity_offset_config                 | -        | Added       | Disparity offset configuration                               |
| offset_index0                           | -        | Added       | Disparity offset index 0                                     |
| offset_index1                           | -        | Added       | Disparity offset index 1                                     |
| force_ip_enable                         | -        | Added       | Enable Force IP configuration                                |
| force_ip_mac                            | -        | Added       | MAC address for Force IP                                     |
| force_ip_dhcp                           | -        | Added       | Enable DHCP for Force IP                                     |
| force_ip_address                        | -        | Added       | Forced IP address assignment                                 |
| force_ip_subnet_mask                    | -        | Added       | Forced subnet mask assignment                                |
| force_ip_gateway                        | -        | Added       | Forced gateway assignment                                    |

### **Topic Differences**

**v2-main** adds the following topics based on main:

| **Topic**             | **main** | **v2-main** | **Description**                                              |
| --------------------- | -------- | ----------- | ------------------------------------------------------------ |
| /camera/device_status | -        | Added       | Publishes device status (frame rate delay, device connection status, etc.) |

### **Service Differences**

**v2-main** adds the following services based on main:

| **Service**           | **main** | **v2-main** | **Description**                     |
| --------------------- | -------- | ----------- | ----------------------------------- |
| get_ptp_config        | -        | Added       | Get PTP configuration               |
| set_ptp_config        | -        | Added       | Set PTP configuration               |
| set_color_ae_roi      | -        | Added       | Set AE ROI for color stream         |
| set_color_flip        | -        | Added       | Set flip for color stream           |
| set_color_rotation    | -        | Added       | Set rotation for color stream       |
| set_depth_ae_roi      | -        | Added       | Set AE ROI for depth stream         |
| set_depth_flip        | -        | Added       | Set flip for depth stream           |
| set_depth_rotation    | -        | Added       | Set rotation for depth stream       |
| set_right_ir_flip     | -        | Added       | Flip right IR stream                |
| set_right_ir_ae_roi   | -        | Added       | Set AE ROI for right IR stream      |
| set_right_ir_rotation | -        | Added       | Rotate right IR stream              |
| set_left_ir_flip      | -        | Added       | Flip left IR stream                 |
| set_left_ir_ae_roi    | -        | Added       | Set AE ROI for left IR stream       |
| set_left_ir_rotation  | -        | Added       | Rotate left IR stream               |
| read_customer_data    | -        | Added       | Read user-stored custom data        |
| write_customer_data   | -        | Added       | Write user-stored custom data       |
| set_filter            | -        | Added       | Configure depth/point cloud filters |