# ROS1 EnhancedDepthFilter Usage Guide

## Scope

`EnhancedDepthFilter` provides enhanced depth output for the Gemini 330 series. In the current ROS1 package, the SDK includes the enhanced depth filter extension library only in the Linux ARM64 directory. If the filter is enabled on another platform, the SDK may report an error directly.

## Prerequisites

1. The new LingBot License must be written to the device.

2. An external model file must be prepared.

3. Both the color and depth streams must be enabled at startup.

4. D2C/C2D alignment and frame aggregation must be enabled so that color and depth are available in the same frameset.

Pass the model file path through a ROS launch parameter:

```plaintext
enhanced_depth_model_path:=/path/to/model/file
```

If enhanced depth is enabled but the model path is empty or the file does not exist, the node fails to start.

## Launch Examples

Standard node:

```plaintext
roslaunch orbbec_camera gemini_330_series.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  enhanced_depth_confidence_threshold:=51 \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

Nodelet:

```plaintext
roslaunch orbbec_camera gemini_330_series_nodelet.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  enhanced_depth_confidence_threshold:=51 \
  depth_registration:=true \
  align_mode:=SW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

When using hardware D2C, the alignment target must be `COLOR`:

```plaintext
roslaunch orbbec_camera gemini_330_series.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  depth_registration:=true \
  align_mode:=HW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

## Parameters

* `enable_enhanced_depth`: Whether to enable enhanced depth filtering. The default is `false`.

* `enhanced_depth_model_path`: Path to the LingBot model file. This parameter is required when enhanced depth is enabled.

* `enhanced_depth_confidence_threshold`: Confidence threshold. The default is `51`.

* `depth_registration`: Whether to enable alignment. `EnhancedDepthFilter` requires this parameter to be `true`.

* `align_mode`: Alignment mode. Supported values are `HW` and `SW`. `HW` supports only D2C.

* `align_target_stream`: Alignment target. Supported values are `COLOR` and `DEPTH`. `COLOR` means D2C, and `DEPTH` means C2D.

* `frame_aggregate_mode`: Frame aggregation mode. The recommended value is `full_frame`.

## Image Requirements

For D2C, where depth is aligned to color:

* Use `align_mode:=HW` or `align_target_stream:=COLOR`.

* The color resolution must be one of `640x480`, `1280x720`, or `1280x800`.

* The depth resolution is unrestricted, but the depth format must be an enhanced depth format supported by the SDK.

For C2D, where color is aligned to depth:

* Use `align_mode:=SW align_target_stream:=DEPTH`.

* The depth resolution must be one of `640x480`, `1280x720`, or `1280x800`.

* The color resolution is unrestricted.

Format requirements:

* The final color input to `EnhancedDepthFilter` is `RGB`. The driver attempts to convert `YUYV`, `UYVY`, `MJPG`, `BGR`, `RGBA`, `Y16`, and `Y8` to `RGB`.

* Supported depth formats are `Y10`, `Y11`, `Y12`, `Y14`, `Y16`, and `Z16`.

## SDK JSON Configuration

When an SDK JSON file is loaded through `load_config_json_file_path`, the profiles and `point_cloud.align_mode` in the JSON file affect the final color/depth resolutions and alignment mode.

When EnhancedDepthFilter is enabled, make sure that:

* color and depth are enabled in the effective JSON or launch configuration;

* D2C/C2D alignment is enabled in the effective JSON or launch configuration;

* the D2C color target resolution or C2D depth target resolution is one of `640x480/1280x720/1280x800`.

If a launch parameter and SDK JSON configure the same setting, the explicitly passed launch parameter takes precedence.

## Output Topics

The enhanced depth image is published on:

```plaintext
/camera/depth/image_raw
```

The original depth image before alignment is published on:

```plaintext
/camera/depth/image_unaligned
```

The confidence image is published on:

```plaintext
/camera/confidence/image_raw
```

If `camera_name` is not the default `camera`, the topic prefix changes with `camera_name`.

## Runtime Adjustment

Use the filter service to enable or disable EnhancedDepthFilter or adjust `confidence_threshold`.

Set the threshold with a positional parameter:

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: [60]
filter_config: []"
```

Set the threshold with a named parameter:

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'filter_enable: truefilter_param: []filter_config:- name: 'confidence_threshold'  value: '60'"
```

Disable EnhancedDepthFilter:

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'filter_enable: falsefilter_param: []filter_config: []"
```

Changing `model_path` at runtime is not supported. To use a different model file, change the launch parameter and restart the node.

## Common Errors

`Enhanced depth filter requires enhanced_depth_model_path`

EnhancedDepthFilter is enabled, but no model file path was provided.

`Enhanced depth model file not found`

The supplied model file path does not exist or cannot be accessed by the node process.

`Enhanced depth filter requires D2C/C2D align mode`

`depth_registration` is not enabled, or the alignment mode is not `HW`/`SW`.

`Enhanced depth filter requires supported target resolutions: 640x480/1280x720/1280x800`

The current alignment target stream resolution is not in the supported list. For D2C, check the color resolution. For C2D, check the depth resolution.

`Enhanced depth filter requires supported target resolutions: 640x480/1280x720/1280x800 and depth formats: Y10/Y11/Y12/Y14/Y16/Z16`

The depth resolution or depth format does not meet the requirements.
