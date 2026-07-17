# ROS1 EnhancedDepthFilter Usage Guide

The LingBot Enhanced Depth Filter (`EnhancedDepthFilter`) uses both color and depth information to improve depth image quality by reducing noise, filling depth holes, and refining object edges.

## Scope and Environment Requirements

EnhancedDepthFilter requires:

* an NVIDIA Jetson running Linux ARM64;
* a supported Gemini 330 series camera;
* CUDA Runtime 12;
* TensorRT 10 Runtime;
* a valid LingBot-Depth License;
* the EnhancedDepthFilter extension and `model.sm4` from the same OrbbecSDK release.

The current ROS1 package includes the EnhancedDepthFilter libraries only in the Linux ARM64 directory. Enabling the filter on another platform causes the node to fail during startup.

## License

The device must support License authorization and contain a valid LingBot-Depth License. For LicenseTool downloads, License applications, and device activation, see [LingBot-Depth-LicenseTool](https://github.com/orbbec/LingBot-Depth-LicenseTool).

Before creating the filter, the driver checks whether the device supports License authorization and whether License information exists on the device. The filter still fails to initialize if the License is invalid, expired, or does not match the device.

## Model File

Download `model.sm4` from the [OrbbecSDK_ROS2 v2.9.3 Release](https://github.com/orbbec/OrbbecSDK_ROS2/releases/tag/v2.9.3). The model must come from the same release as the OrbbecSDK and EnhancedDepthFilter extension used by the current ROS1 package. GitHub-generated Source Code archives do not include the model file.

Pass the model path through a ROS launch parameter:

```text
enhanced_depth_model_path:=/path/to/model.sm4
```

The ROS1 driver requires `enhanced_depth_model_path` to be set explicitly. An absolute path is recommended. If enhanced depth is enabled while the model path is empty or the file does not exist, the node fails to start. The model file cannot be changed at runtime.

## Startup Requirements

The filter input must be an aligned frameset containing both Color and Depth frames:

* both the Color and Depth streams must be enabled;
* software alignment (`align_mode:=SW`) supports both D2C and C2D;
* hardware alignment (`align_mode:=HW`) supports only alignment to Color, that is, D2C;
* `frame_aggregate_mode:=full_frame` is recommended to ensure that each frameset contains both Color and Depth. EnhancedDepthFilter is skipped for a frameset if either frame is missing.

## Launch Examples

For initial verification, use Color `640x480 RGB` and Depth `640x480 Y16`.

Standard node:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_color:=true \
  color_width:=640 \
  color_height:=480 \
  color_format:=RGB \
  enable_depth:=true \
  depth_width:=640 \
  depth_height:=480 \
  depth_format:=Y16 \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model.sm4 \
  enhanced_depth_confidence_threshold:=51
```

Nodelet:

```bash
roslaunch orbbec_camera gemini_330_series_nodelet.launch \
  enable_color:=true \
  color_width:=640 \
  color_height:=480 \
  color_format:=RGB \
  enable_depth:=true \
  depth_width:=640 \
  depth_height:=480 \
  depth_format:=Y16 \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model.sm4 \
  enhanced_depth_confidence_threshold:=51
```

When using hardware D2C, the alignment target must be Color:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model.sm4 \
  depth_registration:=true \
  align_mode:=HW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

## Parameters

* `enable_enhanced_depth`: Whether to enable enhanced depth filtering. The default is `false`.
* `enhanced_depth_model_path`: Path to the LingBot model file. This parameter is required when enhanced depth is enabled.
* `enhanced_depth_confidence_threshold`: Depth confidence threshold. It must be an integer from `0` to `255`. The default is `51`.
* `depth_registration`: Must be `true` so that aligned images enter EnhancedDepthFilter.
* `align_mode`: Alignment method. It can be `SW` or `HW`.
* `align_target_stream`: Software alignment accepts `COLOR` or `DEPTH`; hardware alignment requires `COLOR`.
* `frame_aggregate_mode`: The recommended value is `full_frame`, which ensures that Color and Depth frames are received together.

## Image Requirements

For D2C, where Depth is aligned to Color:

* the Color resolution must be `640x480`, `1280x720`, or `1280x800`;
* the Depth resolution is not constrained by EnhancedDepthFilter, but the selected configuration must support D2C.

For C2D, where Color is aligned to Depth:

* the Depth resolution must be `640x480`, `1280x720`, or `1280x800`;
* the Color resolution is not constrained by EnhancedDepthFilter.

Supported Depth input formats are:

```text
Y10, Y11, Y12, Y14, Y16, Z16
```

EnhancedDepthFilter requires RGB Color input. The ROS1 driver also accepts the following Color stream formats and converts them to RGB in the frameset copy used only by the filter:

```text
RGB, YUYV, UYVY, MJPG, BGR, RGBA, Y16, Y8
```

This conversion is used only by EnhancedDepthFilter and does not change the Color image format published to downstream consumers.

## SDK JSON Configuration

When an SDK JSON file is loaded through `load_config_json_file_path`, the stream profiles and alignment configuration under `application_config.point_cloud` affect the final Color/Depth resolutions, formats, alignment method, and frame aggregation mode.

When EnhancedDepthFilter is enabled, the final effective configuration must still meet these requirements:

* both Color and Depth are enabled;
* D2C or C2D alignment is enabled;
* the D2C Color target resolution or C2D Depth target resolution is one of `640x480/1280x720/1280x800`;
* the Depth format is one of `Y10/Y11/Y12/Y14/Y16/Z16`.

If launch/YAML parameters and SDK JSON configure the same item, parameters already passed to the node take precedence. Default parameters written through `<param>` by a full launch file also count as passed parameters and may override the corresponding JSON configuration. For details, see [SDK JSON Import and Export for Gemini 330 Series](sdk_json_config.md).

## Output Topics

The following topic names assume the default `camera_name:=camera`. If `camera_name` is changed, replace `/camera` with the actual namespace.

| Topic | Description |
| --- | --- |
| `/camera/depth/image_raw` | Publishes the enhanced aligned depth image when filtering succeeds. If filtering fails at runtime, the driver continues to publish the current unenhanced aligned depth image. |
| `/camera/depth/image_unaligned` | Publishes the depth image before software alignment. This topic is not published in hardware D2C mode. |
| `/camera/confidence/image_raw` | Publishes the confidence image with `mono8` encoding when filtering succeeds. |

The standard node and Nodelet use the same topic names and publishing behavior.

## Check the Runtime Status

Use the depth filter status topic to confirm whether EnhancedDepthFilter is enabled and check its current confidence threshold:

```bash
rostopic echo /camera/depth_filters/status
```

Find `filter_name: "EnhancedDepthFilter"` in the output and check `enabled` and `confidence_threshold`.

## Runtime Adjustment

Use `/camera/set_filter` to enable or disable EnhancedDepthFilter or adjust `confidence_threshold`. Positional and named parameters cannot be used together, and the threshold must be an integer from `0` to `255`.

Enable the filter and set the threshold to `60` with a positional parameter:

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: [60]
filter_config: []"
```

Set the threshold with a named parameter:

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: []
filter_config:
- name: 'confidence_threshold'
  value: '60'"
```

Disable EnhancedDepthFilter:

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: false
filter_param: []
filter_config: []"
```

This service cannot change the model path, stream configuration, or alignment mode. To use a different model file, change the launch parameter and restart the node.

## Troubleshooting

| Symptom | Recommended action |
| --- | --- |
| The node fails to start because EnhancedDepthFilter cannot be found | Confirm that the system is NVIDIA Jetson/Linux ARM64 and that the ROS1 package contains the EnhancedDepthFilter extension. |
| A missing or unsupported License is reported | Use LicenseTool to check device support and apply for or activate a valid LingBot-Depth License. |
| `model.sm4` cannot be found or model initialization fails | Check `enhanced_depth_model_path` and confirm that the model, SDK, and extension come from the same release. |
| The resolution or format is not supported | Verify the feature first with Color `640x480 RGB` and Depth `640x480 Y16`. |
| D2C/C2D is required | Confirm `depth_registration:=true`, then check `align_mode` and `align_target_stream`. |
| `/camera/depth/image_raw` has data but no confidence image is published | Check the node log and `/camera/depth_filters/status`. If filtering fails at runtime, the driver falls back to the unenhanced depth frame. |
