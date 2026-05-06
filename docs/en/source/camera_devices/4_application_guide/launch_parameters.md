# Launch parameters

> If you are not sure how to set the parameters, you can connect the orbbec camera and open the [OrbbecViewer](https://github.com/orbbec/OrbbecSDK/releases).

The following are the launch parameters available:

### Core & Stream Configuration

*   **`camera_name`**
    *   Start the node namespace.
*   **`serial_number`**
    *   The serial number of the camera. This is required when multiple cameras are used.
*   **`usb_port`**
    *   The USB port of the camera. This is required when multiple cameras are used.
*   **`device_num`**
    *   The number of devices. This must be filled in if multiple cameras are required.
* **`device_preset`**
    * The default value is `Default`. You can use the following command to view the configurable mode
    ```bash
    rosrun orbbec_camera list_camera_profile_mode_node
    ```
*   **`[color|depth|left_ir|right_ir|ir]_[width|height|fps|format]`**
    *   The resolution and frame rate of the sensor stream.
*   **`[color|depth|left_ir|right_ir|ir]_rotation`**
    *   Set stream image rotation.
    *   The possible values are `0`, `90`, `180`, `270`.
*   **`[color|depth|left_ir|right_ir|ir]_flip`**
    *   Enable the stream image flip.
*   **`[color|depth|left_ir|right_ir|ir]_mirror`**
    *   Enable the stream image mirror.
*   **`enable_point_cloud`**
    *   Enable the point cloud.
*   **`enable_colored_point_cloud`**
    *   Enable the RGB point cloud.
*   **`cloud_frame_id`**
    *   Modify the `frame_id` name within the ros message.
*   **`ordered_pc`**
    *   Enable filtering of invalid point clouds.
*   **`point_cloud_qos`, `[stream]_qos`, `[stream]_camera_info_qos`**
    *   These QoS parameters are not applicable in the ROS1 wrapper. In ROS1, topic behavior is mainly controlled by publisher/subscriber queue sizes.
* **`color.image_raw.enable_pub_plugins`**
  * Enable Color image transport plugins. Default: `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`.
* **`depth.image_raw.enable_pub_plugins`**
  * Enable Depth image transport plugins. Default: `["image_transport/compressedDepth", "image_transport/raw"]`.
* **`left_ir.image_raw.enable_pub_plugins`**
  * Enable Left IR image transport plugins. Default: `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`.
* **`right_ir.image_raw.enable_pub_plugins`**
  * Enable Right IR image transport plugins. Default: `["image_transport/compressed", "image_transport/raw", "image_transport/theora"]`.
* **`point_cloud_decimation_filter_factor`**
  * Point cloud downsampling factor. Range: `1–8`. `1` means no downsampling.

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


#### Depth Stream
* **`enable_depth_auto_exposure_priority`**
  * Enable the Depth auto exposure priority.
* **`mean_intensity_set_point`**
  * Set the target average intensity of the depth image when auto-exposure is turned on. For example: `mean_intensity_set_point:=100`.
  > **Note:** In wrapper version 2.4.7 and later, this parameter replaces the deprecated `depth_brightness`, but `depth_brightness` will still be supported for backward compatibility.
*   **`enable_depth_scale`**
    *   Whether to enable depth scaling after setting D2C. `true` means enabled, the default is `true`.
*   **`depth_precision`**
    *   The depth precision should be in the format `1mm`. The default value is `1mm`.
*   **`depth_ae_roi_[left|right|top|bottom]`**
    *   Set Depth auto exposure ROI.

#### IR Stream
*   **`enable_ir_auto_exposure`**
    *   Enable the IR auto exposure.
*   **`ir_exposure`** / **`ir_gain`**
    *   Set the IR exposure and gain.
*   **`ir_ae_max_exposure`**
    *   Set the maximum exposure value for IR auto exposure.
*   **`ir_brightness`**
    *   Set the target average intensity of the ir image when auto-exposure is turned on.
#### Laser / LDP
*   **`enable_laser`**
    *   Enable the laser. The default value is `true`.
*   **`laser_energy_level`**
    *   Set the laser energy level.
*   **`enable_ldp`** / **`ldp_power_level`**
    *   Enable the LDP and set its power level.

### Device, Sync & Advanced Features

#### Multi-Camera Synchronization
*   **`sync_mode`**
    *   Set sync mode. The default value is `standalone`.
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

> Used for [multi camera synced](../5_advanced_guide/multi_camera/multi_camera_synced.md).

#### Network Cameras
* **`enumerate_net_device`**
  * Enable automatically enumerate network devices.
* **`net_device_ip`** / **`net_device_port`**
  * Set net device's IP address and port (Usually `8090`).
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
> Used for [net camera](../5_advanced_guide/configuration/net_camera.md).

#### Device-Specific
* **`enable_gmsl_trigger`** / **`gmsl_trigger_fps`**
  * Enable the gmsl trigger out signal / set gmsl trigger fps.
  > Only supports [gmsl camera](../5_advanced_guide/multi_camera/gmsl_camera.md).
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
  > **Supported Modules**: DaBaiA / DaBaiAL / Gemini345 / Gemini345Lg
#### Disparity
*   **`disparity_to_depth_mode`**
    *   `HW`: use hardware disparity to depth conversion. `SW`: use software disparity to depth conversion.
*   **`disparity_range_mode`**, **`disparity_search_offset`**, **`disparity_offset_config`**
    *   Parameters for disparity search offset. Used for [disparity search offset](../5_advanced_guide/configuration/disparity_search_offset.md).

#### Interleave AE Mode
*   **`interleave_ae_mode`**
    *   Set `laser` or `hdr` interleave.
*   **`interleave_frame_enable`**, **`interleave_skip_enable`**, **`interleave_skip_index`**
    *   Parameters to control interleave frame mode.
*   **`[hdr|laser]_index[0|1]_[...]`**
    *   In interleave frame mode, set the 0th and 1st frame parameters of hdr or laser interleaving frames.
*   *All interleave parameters are used for [interleave ae mode](../5_advanced_guide/configuration/interleave_ae_mode.md).*

#### Intra-Camera Synchronization

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
  - Sets the reference point for intra-camera synchronization. Applicable for Gemini 330 series devices when `sync_mode` is set to **software** or **hardware trigger** mode. **Options:** `Start`, `Middle`, `End`. When set to empty, the long baseline device defaults to End, and the short baseline device defaults to Middle.

### Basic & General Parameters

#### Firmware & Backend
* **`upgrade_firmware`**
  * The input parameter is the firmware path. For new versions, use the standalone `firmware_update_tool` for firmware updates.
* **`preset_firmware_path`**
  * The input parameter is the preset firmware path. If multiple paths are input, each path needs to be separated by `,` and a maximum of 3 firmware paths can be input.
* **`uvc_backend`**
  * Optional values: `v4l2`, `libuvc`.
* **`connection_delay`**
  * The delay time in milliseconds for reopening the device. Some devices, such as Astra mini, require a longer time to initialize and reopening the device immediately can cause firmware crashes when hot plugging.
* **`retry_on_usb3_detection_failure`**
  * If the camera is connected to a USB 2.0 port and is not detected, the system will attempt to reset the camera up to three times. It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.

#### TF, Extrinsics & Calibration
*   **`publish_tf`** / **`tf_publish_rate`**
    *   Enable the TF publish and set its publication rate.
*   **`enable_publish_extrinsic`**
    *   Enable the extrinsics publish.
*   **`ir_info_url`** / **`color_info_url`**
    *   Set URL of the IR/color camera info.
*   **`enable_color_undistortion`**
    *   Enable the Color undistortion.

#### Time Synchronization
* **`enable_sync_host_time`**
  * Enable synchronization of the host time with the camera time. The default value is `true`. If using global time, set to `false`.
* **`time_domain`**
  * Select timestamp type: `device`, `global`, and `system`.
* **`time_sync_period`**
  * Interval (in seconds) for synchronizing the camera time with the host system.
  > **Note**: This parameter only needs to be set when `enable_sync_host_time = true` and `time_domain = device`.

* **`enable_frame_timestamp_csv`**
  * Enable CSV logging for frame timestamp statistics.
* **`frame_timestamp_csv_file`**
  * CSV output path for frame timestamp statistics. If empty, the wrapper writes to the default ROS log path.
* **`enable_frame_sync`**
  * Enable the frame synchronization.

#### Logging & Diagnostics
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

#### Miscellaneous
*   **`config_file_path`**
    *   The path to the YAML configuration file. Default is `""`. If not specified, default parameters from the launch file will be used. Set this to `gemini2L_dual_ir.yaml` to enable Gemini 2L dual IR mode.
*   **`frame_aggregate_mode`**
    *   Set frame aggregate output mode. Optional values: `full_frame`, `color_frame`, `ANY`, `disable`.
*   **`enable_d2c_viewer`**
    *   Publishes the D2C overlay image (for testing only).

### IMU

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

### Depth Filters

*   **`enable_decimation_filter`**
    *   Enable the Depth decimation filter. Set with `decimation_filter_scale`.
*   **`enable_hdr_merge`**
    *   Enable the Depth hdr merge filter. Set with `hdr_merge_exposure_1`, etc.
*   **`enable_sequence_id_filter`**
    *   Enable the Depth sequence id filter. Set with `sequence_id_filter_id`.
*   **`enable_threshold_filter`**
    *   Enable the Depth threshold filter. Set with `threshold_filter_max`, `threshold_filter_min`.
*   **`enable_hardware_noise_removal_filter`**
    *   Enable the Depth hardware noise removal filter.
*   **`enable_noise_removal_filter`**
    *   Enable the Depth software noise removal filter. Set with `noise_removal_filter_min_diff`, etc.
*   **`enable_spatial_filter`**
    *   Enable the Depth spatial filter. Set with `spatial_filter_alpha`, etc.
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
