# Gemini 330 Series SDK JSON Usage Guide

This document describes how to import and export SDK JSON configuration files for Gemini 330 series cameras in ROS1.

SDK JSON files can be used to restore or migrate camera configuration. The JSON file can come from the SDK, OrbbecViewer, or a configuration exported by ROS. The ROS node passes the file path to the SDK, triggers import or export, and reads back the final camera configuration after initialization.

## Scope

This document only covers SDK JSON import and export for the Gemini 330 series. Prefer the dedicated launch file:

```bash
gemini_330_series_sdk_json.launch
```

This launch file is designed for SDK JSON workflows. It keeps only the parameters required for ROS runtime and device management, reducing the chance that default camera parameters from launch override the JSON configuration.

Full launch files such as `gemini_330_series.launch`, `gemini_330_series_low_cpu.launch`, and `gemini_330_series_nodelet.launch` also provide `load_config_json_file_path`, but they contain many camera parameters. When parameters conflict, launch or YAML parameters that have already been passed to the node take precedence over the corresponding configuration in the JSON file.

## Partial Import

SDK JSON supports partial import. You can remove modules and parameters that are not needed and keep only the content to be imported. Configuration items omitted from the JSON file are not modified by that JSON import; their final values are still determined by launch / YAML parameters, the current device state, or defaults.

It is recommended to minimize the JSON file for the actual use case. This reduces duplicate configuration and override relationships between JSON and launch / YAML parameters. For example, if only a depth preset and one depth post-processing filter are required, other sensor, point cloud, and unrelated filter configuration can be removed.

After removing modules or parameters, the file must remain valid JSON, and the hierarchy and field names that remain must conform to the SDK JSON format supported by the current device and firmware. Re-import the minimized file and verify the final effective configuration as described below.

## Parameter Priority

When SDK JSON and ROS launch / YAML parameters configure the same item, use the following rule to understand the final effective value:

```text
launch / YAML parameters passed to the node > same configuration in SDK JSON
```

"Passed to the node" includes default parameters written by `<param>` in the launch file. Therefore, when using full launch files such as `gemini_330_series.launch`, some camera parameters may already be passed to the node even if the user did not explicitly set them on the command line, and those values may override the corresponding JSON configuration.

SDK JSON modules such as `application_config`, `parameters.sensor_depth`, and `parameters.sensor_color` can all contain configuration that corresponds to ROS parameters. Regardless of which module a field belongs to, if it controls the same configuration item as a launch / YAML parameter, use the priority rule above to determine the final effective value.

For example, if the JSON file sets `color_brightness=10`, but launch or YAML passes `color_brightness=0`, the camera finally uses `0`. The final effective value is reflected by startup logs such as `Config final readback ...`.

Therefore:

* To restore configuration from JSON as much as possible, use `gemini_330_series_sdk_json.launch`.
* If you must use a full launch file, leave parameters that conflict with JSON empty or unset, or set them to the expected values.

For available depth preset names, see [Predefined Presets](predefined_presets.md). To flash or upgrade a preset file, see [firmware_update_tool Device Maintenance Tool](../../6_benchmark/firmware_update_tool.md).

## Import an SDK JSON File

Use `load_config_json_file_path` to specify the SDK JSON file to import:

```bash
roslaunch orbbec_camera gemini_330_series_sdk_json.launch \
  load_config_json_file_path:=/path/to/camera_config.json
```

The file path can be an absolute path, a relative path, or start with `~`. Relative paths are resolved to absolute paths based on the current working directory.

When import succeeds, the log contains:

```text
Config JSON loaded file=/path/to/camera_config.json
```

If the file does not exist, the log contains:

```text
Config JSON load skip file=/path/to/camera_config.json reason=file_not_found
```

If the SDK fails to load the JSON file, the log contains:

```text
Config JSON load failed file=/path/to/camera_config.json error="..."
```

## Confirm the Final Effective Configuration

After importing JSON, the node reads back the final camera configuration during initialization and prints logs similar to:

```text
Config final readback [depth] device_preset=Default
Config final readback [color] color_brightness=0
Config final readback [filter.depth.DecimationFilter] scale=2
```

These logs show the final effective camera configuration. If imported JSON conflicts with launch / YAML parameters, the readback logs reflect the overridden final values.

For depth filters, you can also check the current enabled state and parameters from the status topic:

```bash
rostopic echo /camera/depth_filters/status
```

## Recommended On-Site Workflow

Use the following sequence to configure, verify, and persist settings during on-site debugging:

1. Minimize the SDK JSON file so it contains only the modules and parameters that need to be imported.
2. Start the node with `load_config_json_file_path`. When using a full launch file, check every launch / YAML parameter that overlaps with JSON and set it to the intended value. For example, if the JSON file retains a depth preset or false positive filter configuration, handle `device_preset` and `enable_false_positive_filter` as described above.
3. Check the `Config JSON loaded ...` and `Config final readback ...` logs. For configuration with a query or status interface, use that interface to verify the final state and parameters. For example, depth filters can be verified through `/camera/depth_filters/status`.
4. If on-site tuning is required, use the runtime parameters or services provided for that configuration. For example, depth post-processing filters can be adjusted temporarily through `/camera/set_filter`.
5. After confirming the state, export the current final configuration through `/camera/export_config_json`.
6. If only part of the exported configuration is needed for delivery, remove unrelated modules and parameters, then re-import the file to verify it.

Runtime parameters and services are intended for verification and tuning while the current node is running. To reuse a confirmed configuration after restarting the node, export the state to SDK JSON and import it again on subsequent startup.

## Export an SDK JSON File

After the camera node is running, use the `/camera/export_config_json` service to export the current configuration:

```bash
rosservice call /camera/export_config_json "data: '/tmp/orbbec_camera_config.json'"
```

After a successful call, the specified path contains an SDK JSON configuration file. The file can be imported later with `load_config_json_file_path`.

The export path can be an absolute path, a relative path, or start with `~`. If the parent directory does not exist, the node creates it automatically.

When export succeeds, the service response and log contain:

```text
Exported config json file path: /tmp/orbbec_camera_config.json
```

If the path is empty or export fails, the service returns failure information and the log contains the error reason.

## Condensed Field Mapping

The table below lists common SDK JSON fields and their related ROS parameters. It is intended to help understand conflicts and overrides; it does not mean every field should be configured through launch.

| SDK JSON field | Related ROS parameters | Description |
| --- | --- | --- |
| `application_config.sensors.Color.profile.*` | `enable_color`, `color_width`, `color_height`, `color_fps`, `color_format`, `enable_color_undistortion` | Color stream enable state, resolution, frame rate, format, and undistortion. |
| `application_config.sensors.Depth.profile.*` | `enable_depth`, `depth_width`, `depth_height`, `depth_fps`, `depth_format`, `enable_depth_undistortion` | Depth stream enable state, resolution, frame rate, format, and undistortion. |
| `application_config.sensors.LeftIR.profile.*` | `enable_left_ir`, `left_ir_width`, `left_ir_height`, `left_ir_fps`, `left_ir_format`, `enable_left_ir_undistortion` | Left IR stream configuration. |
| `application_config.sensors.RightIR.profile.*` | `enable_right_ir`, `right_ir_width`, `right_ir_height`, `right_ir_fps`, `right_ir_format`, `enable_right_ir_undistortion` | Right IR stream configuration. |
| `application_config.sensors.Accel.profile.*` | `enable_accel`, `accel_rate`, `accel_range` | Accelerometer enable state, sample rate, and range. |
| `application_config.sensors.Gyro.profile.*` | `enable_gyro`, `gyro_rate`, `gyro_range` | Gyroscope enable state, sample rate, and range. |
| `application_config.point_cloud.*` | `enable_point_cloud`, `enable_colored_point_cloud`, `point_cloud_decimation_filter_factor`, `depth_registration`, `align_mode`, `align_target_stream`, `enable_frame_sync`, `frame_aggregate_mode` | Point cloud, colored point cloud, alignment, frame sync, and frame aggregation. |
| `application_config.hdr_merge.*` | `enable_hdr_merge` | HDR merge configuration. |
| `application_config.device_decimation.*` | `preset_resolution_config` | Device-level decimation configuration. |
| `parameters.sensor_depth.depth_preset` | `device_preset` | Depth preset. When importing JSON with a full launch file, set the `device_preset` passed through launch / YAML to an empty value so that it does not override JSON. |
| Exposure, gain, AE ROI, depth unit, laser, disparity, and image orientation fields under `parameters.sensor_depth` | `depth_exposure`, `depth_gain`, `enable_ir_auto_exposure`, `ir_ae_max_exposure`, `depth_ae_roi_*`, `depth_precision`, `enable_laser`, `laser_energy_level`, `disparity_to_depth_mode`, `disparity_range_mode`, `disparity_search_offset`, `depth_rotation`, `depth_flip`, `depth_mirror`, etc. | Depth-related device configuration. |
| `parameters.sensor_depth.frame_interleave.*` | `interleave_frame_enable`, `interleave_ae_mode`, `interleave_skip_index`, `hdr_index*_*`, `laser_index*_*` | HDR / laser interleave configuration. |
| `parameters.sensor_depth.post_processing_filter.*` | `enable_*_filter` and related filter parameters | Depth post-processing filter configuration. Detailed `FalsePositiveFilter` parameters are configured through JSON or `/camera/set_filter` `filter_config`. |
| Exposure, white balance, brightness, sharpness, anti-flicker, AE ROI, and image orientation fields under `parameters.sensor_color` | `enable_color_auto_exposure`, `color_exposure`, `color_gain`, `enable_color_auto_white_balance`, `color_white_balance`, `color_brightness`, `color_sharpness`, `color_powerline_freq`, `color_ae_roi_*`, `color_rotation`, `color_flip`, `color_mirror`, etc. | Color-related device configuration. |
| `parameters.sensor_color.post_processing_filter.DecimationFilter` | `enable_color_decimation_filter`, `color_decimation_filter_scale` | Color decimation filter configuration. |
| `parameters.sensor_left_ir` / `parameters.sensor_right_ir` | `left_ir_rotation`, `right_ir_rotation`, `enable_left_ir_sequence_id_filter`, `enable_right_ir_sequence_id_filter`, `left_ir_sequence_id_filter_id`, `right_ir_sequence_id_filter_id`, etc. | Left and right IR image orientation and sequence id filter configuration. |

## FAQ

### JSON import does not take effect

Check the following:

* Whether `load_config_json_file_path` points to an existing file.
* Whether the log contains `Config JSON loaded file=...`.
* Whether a full launch file passed the same launch / YAML parameter and overrode the JSON configuration.
* Whether the current device and firmware support the fields in the JSON file.

### Should I use the dedicated launch file or the full launch file?

If the goal is to restore configuration from JSON as much as possible, use `gemini_330_series_sdk_json.launch`.

If you want to keep the larger set of parameter controls from a launch file, you can continue to use full launch files such as `gemini_330_series.launch`, but you need to know which parameters will override JSON.

### Does the exported JSON contain the final configuration?

The exported JSON is based on the current final camera state. Before export, the node syncs current ROS-side sensor, point cloud, and HDR merge configuration to the SDK `application_config`, and then calls the SDK JSON export.

If JSON was imported at startup and some configuration was overridden by launch / YAML, the exported file contains the overridden final configuration.
