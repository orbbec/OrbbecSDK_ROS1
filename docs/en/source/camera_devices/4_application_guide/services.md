# Available services

The service names intuitively reflect their purposes. It's crucial to understand that services related to setting or getting parameters—denoted as `set_*` and `get_*`—become available only when the respective `enable_*` parameters are activated. For instance, enabling features such as left infrared (IR) with `enable_left_ir`, right IR with `enable_right_ir`, depth sensing with `enable_depth`, or color processing with `enable_color` (refer to [Launch Parameters](./launch_parameters.md)) is a prerequisite for their corresponding services to be operational. This configuration ensures that services are accessible only when their specific stream is enabled in the launch file's stream argument.

## Color Stream

* `/camera/get_color_camera_info`

```bash
rosservice call /camera/get_color_camera_info
```

* `/camera/get_color_exposure`

```bash
rosservice call /camera/get_color_exposure
```

* `/camera/get_color_gain`

```bash
rosservice call /camera/get_color_gain
```

* `/camera/set_color_auto_exposure`

```bash
rosservice call /camera/set_color_auto_exposure 1
```

* `/camera/get_color_auto_exposure`

```bash
rosservice call /camera/get_color_auto_exposure
```

* `/camera/set_color_exposure`

```bash
rosservice call /camera/set_color_exposure 100
```

* `/camera/reset_color_exposure`

```bash
rosservice call /camera/reset_color_exposure
```

* `/camera/set_color_gain`

```bash
rosservice call /camera/set_color_gain 16
```

* `/camera/reset_color_gain`

```bash
rosservice call /camera/reset_color_gain
```

* `/camera/set_color_ae_roi`

```bash
# In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'
```

* `/camera/set_color_mirror`

```bash
rosservice call /camera/set_color_mirror 1
```

* `/camera/toggle_color`

```bash
rosservice call /camera/toggle_color 1
```

## Depth Stream

* `/camera/get_depth_camera_info`

```bash
rosservice call /camera/get_depth_camera_info
```

* `/camera/get_depth_exposure`

```bash
rosservice call /camera/get_depth_exposure
```

* `/camera/reset_depth_exposure`

```bash
rosservice call /camera/reset_depth_exposure
```

* `/camera/get_depth_gain`

```bash
rosservice call /camera/get_depth_gain
```

* `/camera/reset_depth_gain`

```bash
rosservice call /camera/reset_depth_gain
```

* `/camera/set_depth_auto_exposure`

```bash
rosservice call /camera/set_depth_auto_exposure 1
```

* `/camera/get_depth_auto_exposure`

```bash
rosservice call /camera/get_depth_auto_exposure
```

* `/camera/set_depth_exposure`

```bash
rosservice call /camera/set_depth_exposure 3000
```

* `/camera/set_depth_gain`

```bash
# v2 version is not supported yet
rosservice call /camera/set_depth_gain 64
```

* `/camera/set_depth_ae_roi`

```bash
# In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
rosservice call /camera/set_depth_ae_roi '{data_param:[0,1279,0,719]}'
```

* `/camera/set_depth_mirror`

```bash
rosservice call /camera/set_depth_mirror 1
```

* `/camera/toggle_depth`

```bash
rosservice call /camera/toggle_depth 1
```

## IR Stream

* `/camera/get_ir_camera_info`

```bash
rosservice call /camera/get_ir_camera_info
```

* `/camera/get_ir_exposure`

```bash
rosservice call /camera/get_ir_exposure
```

* `/camera/reset_ir_exposure`

```bash
rosservice call /camera/reset_ir_exposure
```

* `/camera/get_ir_gain`

```bash
rosservice call /camera/get_ir_gain
```

* `/camera/reset_ir_gain`

```bash
rosservice call /camera/reset_ir_gain
```

* `/camera/set_ir_auto_exposure`

```bash
rosservice call /camera/set_ir_auto_exposure 1
```

* `/camera/set_ir_exposure`

```bash
rosservice call /camera/set_ir_exposure 3000
```

* `/camera/set_ir_gain`

```bash
rosservice call /camera/set_ir_gain 64
```

* `/camera/set_ir_mirror`

```bash
rosservice call /camera/set_ir_mirror 1
```

* `/camera/switch_ir`

```bash
rosservice call /camera/switch_ir left
```

* `/camera/toggle_ir`

```bash
rosservice call /camera/toggle_ir 1
```

## Runtime Stream Configuration

* `/camera/set_stream_profile`

Switch one or more enabled image stream profiles while the node is running. `stream_name` accepts `color`, `left_color`, `right_color`, `depth`, `ir`, `left_ir`, and `right_ir`. Specify only the fields to change; use `0` for unchanged numeric fields and an empty string for an unchanged format. The node stops and restarts the streams during the switch. The service reports failure if the requested profile is already active.

```bash
rosservice call /camera/set_stream_profile "{profiles: [{stream_name: 'color', width: 1280, height: 720, fps: 30, format: 'MJPG'}]}"
```

* `/camera/set_image_registration_mode`

Switch depth-color image registration at runtime. Valid values are `OFF`, `HW_D2C`, `SW_D2C`, and `SW_C2D`, case-insensitive. Both color and depth streams must be enabled for every mode except `OFF`. The node restarts streams automatically and restores the previous mode if the switch fails.

```bash
rosservice call /camera/set_image_registration_mode "{data: 'HW_D2C'}"
```

## Sensor & Emitter Control

* `/camera/get_auto_white_balance`

```bash
rosservice call /camera/get_auto_white_balance
```

* `/camera/set_auto_white_balance`

```bash
rosservice call /camera/set_auto_white_balance 1
```

* `/camera/get_white_balance`

```bash
rosservice call /camera/get_white_balance
```

* `/camera/reset_white_balance`

```bash
rosservice call /camera/reset_white_balance
```

* `/camera/set_laser`

```bash
rosservice call /camera/set_laser 1
```

* `/camera/get_laser_status`
```bash
rosservice call /camera/get_laser_status
```

* `/camera/set_ldp`

```bash
rosservice call /camera/set_ldp 1
```

* `/camera/get_ldp_status`

```bash
rosservice call /camera/get_ldp_status
```

* `/camera/get_lrm_measure_distance`

```bash
rosservice call /camera/get_lrm_measure_distance
```

* `/camera/set_flood`

```bash
rosservice call /camera/set_flood 1
```

* `/camera/set_fan_work_mode`

```bash
rosservice call /camera/set_fan_work_mode 1
```

## Device Information & Management

* `/camera/get_device_info`

```bash
rosservice call /camera/get_device_info
```

* `/camera/get_device_config`

Get the currently effective device configuration state, such as preset, alignment mode, time domain, sync mode, and frame aggregate mode.

```bash
rosservice call /camera/get_device_config
```

* `/camera/get_device_type`

```bash
rosservice call /camera/get_device_type
```

* `/camera/get_serial`

```bash
rosservice call /camera/get_serial
```

* `/camera/get_sdk_version`

```bash
rosservice call /camera/get_sdk_version
```

* `/camera/get_camera_params`

```bash
rosservice call /camera/get_camera_params
```

* `/camera/export_config_json`

Export the current device configuration to an SDK JSON file. For the Gemini 330 series JSON import and export workflow, see [SDK JSON Import and Export for Gemini 330 Series](../5_advanced_guide/configuration/sdk_json_config.md).

```bash
rosservice call /camera/export_config_json "data: '/tmp/orbbec_camera_config.json'"
```

* `/camera/set_bag_recording`

Record current device data with SDK bag recording. `enable: true` starts recording and `enable: false` stops recording. When `file_path` is empty, the default file name is created in the current working directory.

```bash
rosservice call /camera/set_bag_recording "enable: true
file_path: '/tmp/orbbec_record.bag'"
```

```bash
rosservice call /camera/set_bag_recording "enable: false
file_path: ''"
```

* `/camera/reboot_device`

```bash
rosservice call /camera/reboot_device
```

## Synchronization

* `/camera/get_ptp_config`

```bash
rosservice call /camera/get_ptp_config
```

* `/camera/set_ptp_config`

```bash
rosservice call /camera/set_ptp_config true
```

* `/camera/set_sync_io_voltage_level`

Set the sync IO voltage level. This is only supported on devices that expose the property.

```bash
rosservice call /camera/set_sync_io_voltage_level 0
```

## Disparity Configuration

* `/camera/set_disparity_range_mode`

```bash
rosservice call /camera/set_disparity_range_mode 1
```

* `/camera/set_disparity_search_offset`

```bash
rosservice call /camera/set_disparity_search_offset 0
```

## AE Configuration

* `/camera/set_ae_reference_stream`

```bash
rosservice call /camera/set_ae_reference_stream color
```

> **Supported Modules**: Gemini 301 series.

* `/camera/set_ae_strategy`

```bash
rosservice call /camera/set_ae_strategy motion
```

> **Supported Modules**: Gemini 301 series.

## Depth Filter Configuration

* `/camera/set_filter`

For `FalsePositiveFilter` startup parameters, status checks, and named-parameter tuning examples, see [False Positive Filtering for Gemini 330 Series](../5_advanced_guide/configuration/false_positive_filter.md).

```bash
# filter_name is the filter name, and filter_enable indicates whether the filter is enabled.
# filter_param is the legacy positional parameter form; filter_config is the new named parameter form.
# filter_param and filter_config cannot be used at the same time.

# Set DecimationFilter: [scale]
rosservice call /camera/set_filter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

# Set SpatialAdvancedFilter: [alpha, disp_diff, magnitude, radius]
rosservice call /camera/set_filter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

# Set SequenceIdFilter: [sequence_id]
rosservice call /camera/set_filter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

# Set ThresholdFilter: [min, max]
rosservice call /camera/set_filter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

# Set NoiseRemovalFilter: [min_diff, max_size]
rosservice call /camera/set_filter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

# Set HardwareNoiseRemoval: [threshold]
rosservice call /camera/set_filter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: [0.2]}'

# Set SpatialFastFilter: [radius]
rosservice call /camera/set_filter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

# Set SpatialModerateFilter: [disp_diff, magnitude, radius]
rosservice call /camera/set_filter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'

# Set FalsePositiveFilter: []
rosservice call /camera/set_filter '{filter_name: FalsePositiveFilter, filter_enable: true, filter_param: []}'

# Set MgcNoiseRemovalFilter / LutNoiseRemovalFilter: []
rosservice call /camera/set_filter '{filter_name: MgcNoiseRemovalFilter, filter_enable: true, filter_param: []}'
rosservice call /camera/set_filter '{filter_name: LutNoiseRemovalFilter, filter_enable: true, filter_param: []}'

# Set EdgeNoiseRemovalFilter: []
rosservice call /camera/set_filter '{filter_name: EdgeNoiseRemovalFilter, filter_enable: true, filter_param: []}'

# Tune filters with named parameters
rosservice call /camera/set_filter "{filter_name: NoiseRemovalFilter, filter_enable: true, filter_config: [{name: min_diff, value: '256'}, {name: max_size, value: '80'}]}"
rosservice call /camera/set_filter "{filter_name: HardwareNoiseRemovalFilter, filter_enable: true, filter_config: [{name: threshold, value: '0.2'}]}"
rosservice call /camera/set_filter "{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_config: [{name: alpha, value: '0.5'}, {name: disp_diff, value: '160'}, {name: magnitude, value: '1'}, {name: radius, value: '8'}]}"

# Set DispOutliersFilter. search_mode accepts FULL or OFFSET_80, case-insensitive.
rosservice call /camera/set_filter "{filter_name: DispOutliersFilter, filter_enable: true, filter_config: [{name: search_mode, value: 'FULL'}]}"
```

## Data Capture & Calibration Management

* `/camera/get_color_camera_info`

```bash
rosservice call /camera/get_color_camera_info
```

* `/camera/get_depth_camera_info`

```bash
rosservice call /camera/get_depth_camera_info
```

* `/camera/get_ir_camera_info`

```bash
rosservice call /camera/get_ir_camera_info
```

* `/camera/save_images`

```bash
rosservice call /camera/save_images
```

* `/camera/save_point_cloud`

```bash
rosservice call /camera/save_point_cloud
```
