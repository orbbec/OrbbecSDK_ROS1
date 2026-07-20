# Gemini 330 系列 FalsePositiveFilter 使用说明

本文档说明如何在 ROS1 中为 Gemini 330 系列相机使用鬼影滤波 `FalsePositiveFilter`，包括启动时开启、状态确认、运行时开关和临时调参。

以下示例默认相机名称为 `camera`。如果启动时修改了 `camera_name`，请将命令中的 `/camera/...` 替换为实际名称。

## 适用范围

鬼影滤波可用于减少深度图中的重影噪声。启动参数 `enable_false_positive_filter` 的支持模组请参考[启动参数](../../4_application_guide/launch_parameters.md)。

本文重点描述 Gemini 330 系列的使用方式。使用前建议确认设备固件、ROS 包版本和所选 depth preset 与当前测试场景匹配。

## 可选：选择 Depth Preset

如果需要指定设备已有的 depth preset，可通过 `device_preset` 设置：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="<preset_name>"
```

例如使用默认 preset：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="Default"
```

如果使用 YAML 管理参数，可写为：

```yaml
device_preset: "<preset_name>"
```

说明：

* 不需要指定时，可以使用 launch 默认值。
* `<preset_name>` 必须是设备支持的 preset 名称，可参考[设备预设](predefined_presets.md)。
* 如果需要烧录或升级 preset 文件，请参考 [firmware_update_tool 设备维护工具](../../6_benchmark/firmware_update_tool.md)。
* 启动日志中看到 `Loaded device preset: <preset_name>` 表示加载成功。

## 可选：导入 SDK JSON

如果鬼影滤波参数已经写入 SDK JSON，可通过 `load_config_json_file_path` 导入。Gemini 330 系列 SDK JSON 的导入导出流程、参数优先级和日志确认方法，请参考 [Gemini 330 系列 SDK JSON 使用说明](sdk_json_config.md)。

常见情况：

* 使用完整启动文件且 JSON 中包含 `parameters.sensor_depth.depth_preset` 时，应将 launch / YAML 中传入的 `device_preset` 置空，避免它覆盖 JSON 中的 depth preset。
* SDK JSON 支持部分导入，可以删除不需要的模块和参数，只保留 depth preset、鬼影滤波等本次需要导入的配置。
* JSON 中包含鬼影滤波详细参数时，详细参数以 JSON 中保留的字段为准；滤波器开关仍需按下面的启动文件差异处理。

使用 `gemini_330_series_sdk_json.launch` 时，启动文件不会传入 `enable_false_positive_filter`，JSON 中的鬼影滤波开关可以直接生效。使用 `gemini_330_series.launch` 等完整启动文件时，其默认的 `enable_false_positive_filter=false` 会覆盖 JSON 中的开关。如果 JSON 需要开启鬼影滤波，应同时设置：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  device_preset:="" \
  enable_false_positive_filter:=true \
  load_config_json_file_path:=/path/to/camera_config.json
```

其中 `enable_false_positive_filter` 只控制开关，不会替代 JSON 中的鬼影滤波详细参数。

如果 JSON 中配置了鬼影滤波，请继续通过 `/camera/depth_filters/status` 确认 `FalsePositiveFilter` 的 `enabled` 和 `params` 是否符合预期。

## 启动时开启鬼影滤波

`gemini_330_series.launch` 中提供了鬼影滤波启动开关：

```xml
<arg name="enable_false_positive_filter" default="false"/>
```

启动时设置为 `true` 即可开启：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_false_positive_filter:=true
```

如果使用 YAML 管理参数，可写为：

```yaml
enable_false_positive_filter: true
```

说明：

* `enable_false_positive_filter` 只控制鬼影滤波是否开启。
* 鬼影滤波详细参数不通过 launch 参数配置。
* 如需运行时调整详细参数，请使用 `/camera/set_filter`。

## 确认鬼影滤波状态

启动节点后，查看深度滤波状态话题：

```bash
rostopic echo /camera/depth_filters/status
```

在输出中查找 `FalsePositiveFilter`，同时确认开关状态和详细参数状态：

```text
filter_name: "FalsePositiveFilter"
enabled: True
params:
  - name: "fpEdgeBleedFilterEnable"
    value: "true"
  - name: "fpebfROIMinXRatio"
    value: "0.000000"
```

判断标准：

* `enabled: True`：鬼影滤波已开启。
* `enabled: False`：鬼影滤波未开启。
* `params`：当前鬼影滤波的详细参数状态。

`/camera/depth_filters/status` 的发布说明可参考[话题](../../4_application_guide/topics.md)。

## 运行时开启或关闭鬼影滤波

节点运行过程中，可通过 `/camera/set_filter` 临时开启或关闭鬼影滤波。

开启：

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: true, filter_param: [], filter_config: []}"
```

关闭：

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: false, filter_param: [], filter_config: []}"
```

调用后再次确认状态：

```bash
rostopic echo /camera/depth_filters/status
```

说明：

* service 调用是运行时临时操作。
* 如果希望下次启动后自动开启，请在 launch 或 YAML 中设置 `enable_false_positive_filter:=true`。
* `/camera/set_filter` 的更多用法可参考[服务](../../4_application_guide/services.md)。

## 运行时调整鬼影滤波参数

鬼影滤波参数较多，运行时调参时推荐使用 `filter_config`，按参数名称设置。

### 部分参数示例

开启鬼影滤波，并临时调整部分参数：

```bash
rosservice call /camera/set_filter "{filter_name: 'FalsePositiveFilter', filter_enable: true, filter_param: [], filter_config: [
{name: 'fpEdgeBleedFilterEnable', value: 'true'},
{name: 'fpebfROIMinXRatio', value: '0.0'},
{name: 'fpebfROIMaxXRatio', value: '1.0'}
]}"
```

说明：

* `filter_name` 固定为 `FalsePositiveFilter`。
* `filter_enable` 表示本次调用后滤波器是否开启。
* `filter_param` 保持为空。
* `filter_config` 用于指定需要调整的参数。
* 未写入 `filter_config` 的参数保持当前值。

### 完整参数示例

如需一次性设置鬼影滤波的完整参数，可参考以下示例：

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

调用后查看状态和参数：

```bash
rostopic echo /camera/depth_filters/status
```

## 常见问题

### 设置了启动参数，但状态仍为 False

请检查：

* 启动命令是否包含 `enable_false_positive_filter:=true`。
* YAML 中是否将 `enable_false_positive_filter` 设置为 `false`。
* 是否修改参数后未重启节点。
* ROS1 参数服务器是否保留了旧参数，必要时重启 `roscore`。

### service 调参后重启是否保留

`/camera/set_filter` 是运行时临时调参，只对当前运行的节点生效。节点重启后，鬼影滤波是否开启由 `enable_false_positive_filter` 启动参数决定；运行时通过 `filter_config` 设置的详细参数不会通过 launch 或 YAML 自动保存。

### 调参时报未知参数

请检查 `filter_config` 中的 `name` 是否为当前设备和 SDK 支持的鬼影滤波参数名。参数名大小写需要保持一致。
