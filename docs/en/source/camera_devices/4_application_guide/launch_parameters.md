# Launch parameters

> If you are not sure how to set the parameters, connect the Orbbec camera and open [OrbbecViewer](https://github.com/orbbec/OrbbecSDK/releases), or refer to the [camera datasheet](../1_overview/introduction.md) in Chapter 1.

## How to modify launch parameters

Launch parameters can be modified in two ways:

1. **Override parameters in the launch command**

   This is recommended for debugging, testing a parameter, or applying a setting only for the current launch. Use the format `parameter_name:=parameter_value`. You can append multiple parameters to the same command.

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch camera_name:=camera_02
   ```

   Example: change the camera name and enable point cloud output at the same time.

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch camera_name:=camera_02 enable_point_cloud:=true
   ```

2. **Modify the default value in a launch file**

   This is useful when you want a setting to become a long-term default, such as a fixed resolution, frame rate, camera name, or synchronization mode. Device launch files are located in [launch](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/launch). Choose the `*.launch` file that matches your camera model.

   For example, modify the default camera name to `camera_02`:

   ```xml
   <arg name="camera_name" default="camera_02"/>
   ```

   If you build from source, rebuild the workspace and source it again after modifying a launch file:

   ```bash
   cd ~/ros_ws
   catkin_make
   source devel/setup.bash
   ```

   If you installed with apt/deb, it is not recommended to edit files in the system installation directory directly. Prefer command-line parameter overrides, or copy the launch file and maintain your own launch configuration.

The following are the launch parameters available:

## Core & Stream Configuration

*   **`camera_name`**
    *   Start the node namespace.
*   **`serial_number`**
    *   The serial number of the camera. This is required when multiple cameras are used. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
*   **`usb_port`**
    *   The USB port of the camera. This is required when multiple cameras are used. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
*   **`device_num`**
    *   The number of devices. This must be filled in if multiple cameras are required. See [multi camera](../5_advanced_guide/multi_camera/multi_camera.md) for multi-camera startup.
* **`device_preset`**
    * Depth preset. See [predefined presets](../5_advanced_guide/configuration/predefined_presets.md) for available presets and recommended scenarios. You can use the following command to view the configurable modes; the tool also prints the preset list and preset version information.
    ```bash
    rosrun orbbec_camera list_devices_node
    ```
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   The resolution and frame rate of the sensor stream.
    *   For Femto Mega / Femto Bolt, depth NFOV and WFOV modes are configured by combining depth and IR resolutions. See [Configuration of depth NFOV and WFOV modes](../5_advanced_guide/configuration/configuration_of_depth_NFOV_and_WFOV_modes.md).
    *   For lower CPU usage, see the `color_format` recommendations in [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md).
*   **`[color|depth|left_ir|right_ir|ir]_rotation`**
    *   Set stream image rotation.
    *   The possible values are `0`, `90`, `180`, `270`.
*   **`[color|depth|left_ir|right_ir|ir]_flip`**
    *   Enable the stream image flip.
*   **`[color|depth|left_ir|right_ir|ir]_mirror`**
    *   Enable the stream image mirror.
*   **`enable_point_cloud`**
    *   Enable the point cloud. See [Point Cloud](point_cloud.md) for usage and RViz visualization.
*   **`enable_colored_point_cloud`**
    *   Enable the RGB point cloud. See [Point Cloud](point_cloud.md) for usage and RViz visualization.
*   **`cloud_frame_id`**
    *   Modify the `frame_id` name within the ros message.
*   **`ordered_pc`**
    *   Enable filtering of invalid point clouds.
*   **`point_cloud_qos`, `[stream]_qos`, `[stream]_camera_info_qos`**
    *   These QoS parameters are not applicable in the ROS1 wrapper. In ROS1, topic behavior is mainly controlled by publisher/subscriber queue sizes.
* **`point_cloud_decimation_filter_factor`**
  * Point cloud downsampling factor. Range: `1–8`. `1` means no downsampling.

## Sensor Controls

### Color Stream
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
*   **`color_ae_max_gain`**
    *   Set the maximum gain for Color auto exposure. Supported by Gemini 2 firmware `1.5.04` and above, and Gemini 2L firmware `1.5.09` and above. **Range:** `16–112`.
*   **`color_brightness`**, **`color_sharpness`**, **`color_gamma`**, **`color_saturation`**, **`color_contrast`**, **`color_hue`**
    *   Set the Color brightness, sharpness, gamma, saturation, contrast, and hue.
*   **`color_backlight_compensation`**
    *    Enables the color camera’s backlight compensation feature. **Range**: `0–6`, **Default**: `3`.
*   **`color_powerline_freq`**
    *   Set the power line freq. The possible values are `disable`, `50hz`, `60hz`, `auto`.
*   **`color_preset`**
    *   Set the Color preset for Gemini 330 series devices. Options: `Default`, `Warm Biased AWB`.
*   **`color_anti_flicker`**
    *   Enable Color anti-flicker. Supported by Gemini 330 series firmware `1.7.13` and above, and Gemini 305 series firmware `1.0.54` and above.
*   **`enable_color_decimation_filter`** / **`color_decimation_filter_scale`**
    *   Enable the Color decimation filter and set its scale.
*   **`color_ae_roi_[left|right|top|bottom]`**
    *   Set Color auto exposure ROI.
*   **`color_denoising_level`**
    *   Enable ISP Color denoising. **Range:** `0–8`; `0` means auto. Supported by Gemini 2 firmware `1.5.04` and above, and Gemini 2L firmware `1.5.09` and above. This feature requires Color auto exposure and new firmware support.


### Depth Stream
* **`enable_depth_auto_exposure_priority`**
  * Enable the Depth auto exposure priority.
* **`mean_intensity_set_point`**
  * Set the target average intensity of the depth image when auto-exposure is turned on. For example: `mean_intensity_set_point:=100`.
  > **Note:** In wrapper version 2.4.7 and later, this parameter replaces the deprecated `depth_brightness`, but `depth_brightness` will still be supported for backward compatibility.
*   **`enable_depth_scale`**
    *   Whether to enable depth scaling after setting D2C. `true` means enabled, the default is `true`.
*   **`depth_precision`**
    *   The depth precision should be in the format `1mm`. The default value is `1mm`.
*   **`depth_work_mode`**
    *   Set the depth work mode. See [Depth Work Mode Switch](../5_advanced_guide/configuration/depth_work_mode_switch.md) for supported devices, mode query commands, and launch examples.
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   Set Depth auto exposure ROI.

### IR Stream
*   **`enable_ir_auto_exposure`**
    *   Enable the IR auto exposure.
*   **`ir_exposure`** / **`ir_gain`**
    *   Set the IR exposure and gain.
*   **`ir_ae_max_exposure`**
    *   Set the maximum exposure value for IR auto exposure.
*   **`ir_brightness`**
    *   Set the target average intensity of the ir image when auto-exposure is turned on.
### Laser / LDP
*   **`enable_laser`**
    *   Enable the laser. The default value is `true`.
*   **`laser_energy_level`**
    *   Set the laser energy level.
*   **`enable_ldp`** / **`ldp_power_level`**
    *   Enable the LDP and set its power level.

## Device, Sync & Advanced Features

### Multi-Camera Synchronization
*   **`sync_mode`**
    *   Set sync mode. The default value is `standalone`. See [multi camera synced](../5_advanced_guide/multi_camera/multi_camera_synced.md) for multi-camera connection, synchronization modes, and trigger configuration.
*   **`depth_delay_us`** / **`color_delay_us`**
    *   The delay time (microseconds) of the depth/color image capture after receiving the capture command or trigger signal.
*   **`trigger2image_delay_us`**
    *   The delay time (microseconds) of the image capture after receiving the capture command or trigger signal. Us
*   **`trigger_out_delay_us`**
    *   The delay time (microseconds) of the trigger signal output after receiving the capture command or trigger signal.
*   **`trigger_out_enabled`**
    *   Enable the trigger out signal.
*   **`software_trigger_enabled`** / **`software_trigger_period`**
    *   Enable the software trigger out signal / set the software trigger period in ms.
*   **`frames_per_trigger`**
    *   The frame number of each stream after each trigger in triggering mode.

### Network Cameras
* **`enumerate_net_device`**
  * Enable automatically enumerate network devices. See [net camera](../5_advanced_guide/configuration/net_camera.md) for network camera startup, specified IP startup, and Force IP configuration.
* **`ip_address`** / **`port`**
  * Set net device's IP address and port (usually `8090`). See [net camera](../5_advanced_guide/configuration/net_camera.md) for network camera startup, specified IP startup, and Force IP configuration.
* **`force_ip_enable`**
  * Enable the Force IP function. **Default:** `false`
* **`force_ip_mac`**
  * Target device MAC address when multiple cameras are connected (e.g., `"54:14:FD:06:07:DA"`). You can use the `list_devices_node` to find the MAC of each device. **Default:** `""`
* **`force_ip_address`**
  * Static IP address to assign. **Default:** `192.168.1.10`
* **`force_ip_subnet_mask`**
  * Subnet mask for the static IP. **Default:** `255.255.255.0`
* **`force_ip_gateway`**
  * Gateway address for the static IP. **Default:** `192.168.1.1`
### Device-Specific
* **`enable_gmsl_trigger`** / **`gmsl_trigger_fps`**
  * Enable the gmsl trigger out signal / set gmsl trigger fps.
  > Only supports [gmsl camera](../5_advanced_guide/multi_camera/gmsl_cameras.md).
  >
* **`enable_ptp_config`**
  * Enable PTP time synchronization. Requires `enable_sync_host_time` to be `false`.
  > **Supported Modules**: Gemini 335Le
* **`preset_resolution_config`**
  * Preset resolution configuration for the camera device. Format: "width,height,ir_decimation_factor,depth_decimation_factor". Example: "1280,720,4,4". Leave empty to disable.
  > **Supported Modules**: Gemini 435Le
* **`ae_reference_stream`**
  * Select the AE reference stream for Gemini 305 series devices. Options: `color`, `depth`.
  > **Supported Modules**: Gemini 305
* **`ae_strategy`**
  * Select the AE strategy for Gemini 305 series devices. Options: `default`, `motion`.
  > **Supported Modules**: Gemini 305
* **`depth_downscale`** / **`left_ir_downscale`** /**`right_ir_downscale`**
  * Set the downsampling multiple. You can use `rosrun orbbec_camera list_camera_profile_mode_node` to view the settable resolution. **Default value:** `1`
  > **Supported Modules**: Gemini 305
* **`enable_false_positive_filter`**
  * Enable this option to reduce ghosting noise.
  > **Supported Modules**: DaBaiA / DaBaiAL / Gemini 330 series / Gemini345 / Gemini345Lg
### Disparity
*   **`disparity_to_depth_mode`**
    *   `HW`: use hardware disparity to depth conversion. `SW`: use software disparity to depth conversion. Use `disable` to turn it off.
    *   This parameter is case-insensitive. Invalid values are reported and replaced with the default value.
*   **`disparity_range_mode`**, **`disparity_search_offset`**, **`disparity_offset_config`**
    *   Parameters for disparity search offset. Used for [disparity search offset](../5_advanced_guide/configuration/disparity_search_offset.md).

### Interleave AE Mode
*   **`interleave_ae_mode`**
    *   Set `laser` or `hdr` interleave.
*   **`interleave_frame_enable`**, **`interleave_skip_enable`**, **`interleave_skip_index`**
    *   Parameters to control interleave frame mode.
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   In interleave frame mode, set the 0th and 1st frame parameters of hdr or laser interleaving frames.
*   *All interleave parameters are used for [interleave ae mode](../5_advanced_guide/configuration/interleave_ae_mode.md).*

### Intra-Camera Synchronization

- **`depth_registration`**
  *   Enable alignment of the depth frame to the color frame. This field is required when the `enable_colored_point_cloud` is set to `true`. See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for startup and viewing examples.
- **`align_mode`**
  *   The alignment mode to be used. Options are `HW` for hardware alignment and `SW` for software alignment.
  *   This parameter is case-insensitive. Invalid values are reported and replaced with the default value.
- **`align_target_stream`**
  *   Set align target stream mode.
  *   The possible values are `COLOR`, `DEPTH`.
  *   `COLOR`: Align depth to color.
  *   `DEPTH`: Align color to depth.
  *   This parameter is case-insensitive. Hardware D2C only supports `COLOR` as the target stream; use `align_mode:=SW` if you need to align to `DEPTH`. See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for startup and viewing examples.
- **`intra_camera_sync_reference`**
  - Sets the reference point for intra-camera synchronization. Applicable for Gemini 330 series devices when `sync_mode` is set to **software** or **hardware trigger** mode. **Options:** `Start`, `Middle`, `End`. When set to empty, the long baseline device defaults to End, and the short baseline device defaults to Middle.

## Basic & General Parameters

### Firmware & Backend
* **`upgrade_firmware`**
  * The input parameter is the firmware path. For new versions, use the standalone `firmware_update_tool` for firmware updates. See [firmware_update_tool](../6_benchmark/firmware_update_tool.md).
* **`preset_firmware_path`**
  * The input parameter is the preset firmware path. If multiple paths are input, each path needs to be separated by `,` and a maximum of 3 firmware paths can be input. For new versions, use the standalone tool to burn presets. See [firmware_update_tool](../6_benchmark/firmware_update_tool.md).
* **`uvc_backend`**
  * Optional values: `v4l2`, `libuvc`. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU scenarios.
* **`connection_delay`**
  * The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
* **`retry_on_usb3_detection_failure`**
  * If the camera is connected to a USB 2.0 port and is not detected, the system will attempt to reset the camera up to three times. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.

### TF, Extrinsics & Calibration
*   **`publish_tf`** / **`tf_publish_rate`**
    *   Enable the TF publish and set its publication rate. See [Coordinate Systems and TF Transforms](coordinate_and_tf.md) for coordinate frames, TF tree inspection, and visualization.
*   **`enable_publish_extrinsic`**
    *   Enable the extrinsics publish.
*   **`ir_info_url`** / **`color_info_url`**
    *   Set URL of the IR/color camera info.
*   **`enable_[color|depth|ir|left_ir|right_ir]_undistortion`**
    *   Enable the SDK undistortion filter for the selected image stream. Dual-IR devices use `enable_left_ir_undistortion` / `enable_right_ir_undistortion`; single-IR devices use `enable_ir_undistortion`.

### Time Synchronization
* **`enable_sync_host_time`**
  * Enable synchronization of the host time with the camera time. The default value is `true`. If using global time, set to `false`.
* **`time_domain`**
  * Select timestamp type: `device`, `global`, and `system`.
  * This parameter is case-insensitive. Invalid values are reported and replaced with the default value.
* **`timestamp_clock_type`**
  * Set the SDK timestamp clock type. Optional values: `realtime`, `monotonic`. The default is `realtime`.
* **`time_sync_period`**
  * Interval (in seconds) for synchronizing the camera time with the host system.
  > **Note**: This parameter only needs to be set when `enable_sync_host_time = true` and `time_domain = device`.

* **`enable_frame_drop_log`**
  * Enable frame drop logging. The log reports drops detected at both the SDK receive stage and the ROS publish stage.
* **`frame_timestamp_csv_file`**
  * CSV output path for frame timestamp statistics. If empty, no CSV file is written; set a path such as `/tmp/frame_timestamp.csv` to save CSV data.
* **`enable_frame_sync`**
  * Enable the frame synchronization.

### Logging & Diagnostics
* **`log_level`**
  * SDK log level. Default output keeps only the current device status; use `debug` for more detailed logs. Optional values: `debug`, `info`, `warn`, `error`, `fatal`.
  * SDK logs and crash files are stored in `~/.ros/Log`; ROS1 runtime logs are stored in `~/.ros/log/<run_id>`.
* **`log_file_name`**
  * Saved SDK log file name. Effective when `log_level` is `debug`; the actual path is usually `~/.ros/Log/<camera_name>/<log_file_name>`. In multi-camera setups, different `camera_name` values write into separate directories.
* **`diagnostic_period`**
  * Diagnostic period in seconds.
* **`enable_heartbeat`**
  * Enable the heartbeat function. Default is `false`. If `true`, the camera node will send heartbeat signals to the firmware.
* **`enable_firmware_log`**
  * Enable firmware log capture independently from `enable_heartbeat`.

### Miscellaneous
*   **`config_file_path`**
    *   The path to the YAML configuration file. Default is `""`. If not specified, default parameters from the launch file will be used. Some presets or special modes are configured through YAML. See [predefined presets](../5_advanced_guide/configuration/predefined_presets.md).
*   **`load_config_json_file_path`**
    *   SDK JSON configuration import path. When set, the node imports the JSON configuration during initialization. For Gemini 330 series, use `gemini_330_series_sdk_json.launch` as the dedicated SDK JSON launch file.
*   **`export_config_json_file_path`**
    *   SDK JSON configuration export path. When set, the node exports the current device configuration to JSON after initialization. You can also export at runtime with the `/camera/export_config_json` service.
*   **`frame_aggregate_mode`**
    *   Set frame aggregate output mode. Optional values: `full_frame`, `color_frame`, `ANY`, `disable`.
    *   This parameter is case-insensitive. Invalid values are reported and replaced with the default value.
*   **`enable_d2c_viewer`**
    *   Publishes the D2C overlay image (for testing only). See [Aligning Depth to Color](../5_advanced_guide/configuration/align_depth_color.md) for usage examples.

## IMU

*   **`enable_accel`** / **`enable_gyro`**
    *   Enable the Accelerometer/gyroscope and output its info topic data.
*   **`enable_sync_output_accel_gyro`**
    *   Enable the sync `accel_gyro`, and output IMU topic real-time data.
*   **`accel_rate`** / **`gyro_rate`**
    *   The frequency of the accelerometer/gyroscope. Values range from `1.5625hz` to `32khz`.
*   **`accel_range`** / **`gyro_range`**
    *   The range of the accelerometer (`2g`, `4g`, `8g`, `16g`) and gyroscope (`16dps` to `2000dps`).
*   **`enable_accel_data_correction`** / **`enable_gyro_data_correction`**
    *   Enable data correction for the accelerometer/gyroscope.
*   **`linear_accel_cov`** / **`angular_vel_cov`**
    *   Covariance of the linear acceleration and angular velocity.

## Depth Filters

*   **`enable_decimation_filter`**
    *   Enable the Depth decimation filter. Set with `decimation_filter_scale`.
*   **`enable_hdr_merge`**
    *   Enable the Depth hdr merge filter. Set with `hdr_merge_exposure_1`, etc.
*   **`enable_sequence_id_filter`**
    *   Enable the Depth sequence id filter. Set with `sequence_id_filter_id`.
*   **`enable_threshold_filter`**
    *   Enable the Depth threshold filter. Set with `threshold_filter_max`, `threshold_filter_min`.
*   **`enable_hardware_noise_removal_filter`**
    *   Enable the Depth hardware noise removal filter. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
*   **`enable_noise_removal_filter`**
    *   Enable the Depth software noise removal filter. Set with `noise_removal_filter_min_diff`, etc. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
*   **`enable_spatial_filter`**
    *   Enable the Depth spatial filter. Set with `spatial_filter_alpha`, etc. See [Lower CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md) for low-CPU configuration recommendations.
*   **`enable_temporal_filter`**
    *   Enable the Depth temporal filter. Set with `temporal_filter_diff_threshold`, etc.
*   **`enable_hole_filling_filter`**
    *   Enable the Depth hole filling filter. Set with `hole_filling_filter_mode`.
*   **`enable_spatial_fast_filter`**
    *   Enable the Depth spatial fast filter. Set with `spatial_fast_filter_radius`.
*   **`enable_spatial_moderate_filter`**
    *   Enable the Depth spatial moderate filter. Set with `spatial_moderate_filter_diff_threshold`, etc.
*   **`enable_mgc_noise_removal_filter`**
    *   Enable the MGC noise removal filter for OpenNI devices including Astra Mini (S) Pro, DaBai Pro Max, and DaBai DCW2.
*   **`enable_lut_noise_removal_filter`**
    *   Enable the LUT noise removal filter for OpenNI devices including Astra Mini (S) Pro, DaBai Pro Max, and DaBai DCW2.

---

> **_IMPORTANT_**: Please carefully read the instructions regarding software filtering settings at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify these settings.
