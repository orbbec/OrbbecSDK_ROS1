# False Positive Filtering for Gemini 330 Series

This document describes how to use the `FalsePositiveFilter` in ROS1 for Gemini 330 series cameras, including enabling it at startup, checking its status, enabling or disabling it at runtime, and temporarily tuning its parameters.

The examples below use the default camera name `camera`. If you set a different `camera_name` at startup, replace `/camera/...` in the commands with the actual name.

## Scope

False positive filtering can reduce ghosting noise in depth frames. For the modules supported by the `enable_false_positive_filter` launch parameter, see [Launch Parameters](../../4_application_guide/launch_parameters.md).

This document focuses on Gemini 330 series usage. Before using the filter, confirm that the device firmware, ROS package version, and selected depth preset match the test scenario.

## Optional: Select a Depth Preset

If you need to select an existing depth preset on the device, set `device_preset`:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="<preset_name>"
```

For example, to use the default preset:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="Default"
```

If you manage parameters with YAML, set:

```yaml
device_preset: "<preset_name>"
```

Notes:

* If no specific preset is required, use the launch default.
* `<preset_name>` must be a preset name supported by the device. See [Predefined Presets](predefined_presets.md).
* To flash or upgrade a preset file, see [firmware_update_tool Device Maintenance Tool](../../6_benchmark/firmware_update_tool.md).
* The startup log `Loaded device preset: <preset_name>` indicates that the preset was loaded successfully.

## Optional: Import an SDK JSON File

If you already have an SDK JSON configuration file, import it with `load_config_json_file_path`:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  load_config_json_file_path:=/path/to/config.json
```

If the JSON file contains depth configuration, leave the corresponding launch or YAML depth parameters empty or unset to avoid overriding the depth configuration from the JSON file.

Common cases:

* If the JSON file contains `parameters.sensor_depth.depth_preset`, set `device_preset` to an empty value.
* If the JSON file contains detailed false positive filter parameters, the parameters from the JSON file take precedence.
* If the JSON file contains depth engine parameters, they take effect through the JSON import.

Example:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="" \
  load_config_json_file_path:=/path/to/config.json
```

If you manage parameters with YAML, set:

```yaml
device_preset: ""
load_config_json_file_path: "/path/to/config.json"
```

To verify that the JSON file was loaded successfully, check the startup log:

```text
Config JSON loaded file=/path/to/config.json
```

If the JSON file configures false positive filtering, continue to check `/camera/depth_filters/status` and confirm that the `enabled` and `params` fields for `FalsePositiveFilter` match expectations.

## Enable False Positive Filtering at Startup

`gemini_330_series.launch` provides a startup switch for false positive filtering:

```xml
<arg name="enable_false_positive_filter" default="false"/>
```

Set it to `true` at startup:

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_false_positive_filter:=true
```

If you manage parameters with YAML, set:

```yaml
enable_false_positive_filter: true
```

Notes:

* `enable_false_positive_filter` only controls whether the filter is enabled.
* Detailed false positive filter parameters are not configured through launch parameters.
* To tune detailed parameters at runtime, use `/camera/set_filter`.

## Check the Filter Status

After starting the node, check the depth filter status topic:

```bash
rostopic echo /camera/depth_filters/status
```

Find `FalsePositiveFilter` in the output, and check both the enabled state and the detailed parameter state:

```text
filter_name: "FalsePositiveFilter"
enabled: True
params:
  - name: "fpEdgeBleedFilterEnable"
    value: "true"
  - name: "fpebfROIMinXRatio"
    value: "0.000000"
```

How to read the result:

* `enabled: True`: false positive filtering is enabled.
* `enabled: False`: false positive filtering is disabled.
* `params`: the current detailed parameter state of the filter.

For the publishing behavior of `/camera/depth_filters/status`, see [Topics](../../4_application_guide/topics.md).

## Enable or Disable the Filter at Runtime

While the node is running, use `/camera/set_filter` to temporarily enable or disable the filter.

Enable:

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: true, filter_param: [], filter_config: []}"
```

Disable:

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: false, filter_param: [], filter_config: []}"
```

After calling the service, check the status again:

```bash
rostopic echo /camera/depth_filters/status
```

Notes:

* Service calls are temporary runtime operations.
* To enable the filter automatically after the next startup, set `enable_false_positive_filter:=true` in launch or YAML.
* For more `/camera/set_filter` usage, see [Services](../../4_application_guide/services.md).

## Tune Filter Parameters at Runtime

The false positive filter has many parameters. When tuning them at runtime, use `filter_config` and set parameters by name.

### Partial Parameter Example

Enable the filter and temporarily tune a few parameters:

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: true, filter_param: [], filter_config: [
{name: 'fpEdgeBleedFilterEnable', value: 'true'},
{name: 'fpebfROIMinXRatio', value: '0.0'},
{name: 'fpebfROIMaxXRatio', value: '1.0'}
]}"
```

Notes:

* `filter_name` must be `FalsePositiveFilter`.
* `filter_enable` indicates whether the filter is enabled after this call.
* Keep `filter_param` empty.
* Use `filter_config` to specify the parameters to tune.
* Parameters omitted from `filter_config` keep their current values.

### Full Parameter Example

To set the full false positive filter parameter set in one call, use the following example:

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: true, filter_param: [], filter_config: [
{name: 'fpEdgeBleedFilterEnable', value: 'true'},
{name: 'fpebfROIMinXRatio', value: '0.0'},
{name: 'fpebfROIMaxXRatio', value: '1.0'},
{name: 'fpebfROIMinYRatio', value: '0.0'},
{name: 'fpebfROIMaxYRatio', value: '0.6'},
{name: 'fpebfMinBleedLength', value: '40'},
{name: 'fpTextureSparsityFilterEnable', value: 'false'},
{name: 'fptsfROIMinXRatio', value: '0.0'},
{name: 'fptsfROIMaxXRatio', value: '1.0'},
{name: 'fptsfROIMinYRatio', value: '0.0'},
{name: 'fptsfROIMaxYRatio', value: '0.45'},
{name: 'fptsfMaxNoiseLevel', value: '6000'},
{name: 'fptsfMaxSpeckleSize', value: '1300'},
{name: 'fpPatternAmbiguityFilterEnable', value: 'false'},
{name: 'fppafROIMinXRatio', value: '0.0'},
{name: 'fppafROIMaxXRatio', value: '0.99'},
{name: 'fppafROIMinYRatio', value: '0.0'},
{name: 'fppafROIMaxYRatio', value: '0.9'},
{name: 'fppafMaxNoiseLevel', value: '6000'},
{name: 'fppafMaxSpeckleSize', value: '4000'},
{name: 'fppafMaxWidthRatio', value: '0.3'},
{name: 'fppafMaxHeightRatio', value: '0.3'},
{name: 'fppafTolerance', value: '0.15'},
{name: 'fppafScore', value: '50'}
]}"
```

After calling the service, check the status and parameters:

```bash
rostopic echo /camera/depth_filters/status
```

## FAQ

### The startup parameter is set, but the status is still False

Check the following:

* Whether the startup command contains `enable_false_positive_filter:=true`.
* Whether YAML sets `enable_false_positive_filter` to `false`.
* Whether the node was not restarted after changing parameters.
* Whether the ROS1 parameter server kept old parameters. Restart `roscore` if needed.

### Are service tuning changes preserved after restart?

`/camera/set_filter` is temporary runtime tuning and only affects the currently running node. After the node restarts, whether false positive filtering is enabled is controlled by the `enable_false_positive_filter` startup parameter. Detailed parameters set through `filter_config` at runtime are not automatically saved through launch or YAML.

### Tuning fails with an unknown parameter error

Check whether each `name` in `filter_config` is a false positive filter parameter supported by the current device and SDK. Parameter names are case-sensitive.
