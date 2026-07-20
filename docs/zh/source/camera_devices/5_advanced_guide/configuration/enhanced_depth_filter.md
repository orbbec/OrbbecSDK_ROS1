# Gemini 330 系列 EnhancedDepthFilter 使用说明

LingBot 增强深度滤波器（`EnhancedDepthFilter`）同时使用彩色和深度信息，通过降低噪声、填补深度空洞和优化物体边缘来改善深度图像质量。

## 适用范围与环境要求

使用 EnhancedDepthFilter 需要满足以下条件：

* NVIDIA Jetson，操作系统为 Linux ARM64；
* 支持该功能的 Gemini 330 系列相机；
* CUDA Runtime 12；
* TensorRT 10 Runtime；
* 有效的 LingBot-Depth License；
* 来自同一 OrbbecSDK 版本的 EnhancedDepthFilter 扩展库和 `model.sm4`。

当前 ROS1 包仅在 Linux ARM64 目录下打包 EnhancedDepthFilter 相关库。在其它平台启用该滤波器会导致节点启动失败。

## License

设备必须支持 License 授权，并已写入有效的 LingBot-Depth License。LicenseTool 下载、License 申请和设备激活方法请参考 [LingBot-Depth-LicenseTool](https://github.com/orbbec/LingBot-Depth-LicenseTool)。

驱动创建滤波器前会检查设备是否支持 License 授权以及设备中是否存在 License 信息。License 无效、过期或与设备不匹配时，滤波器仍会初始化失败。

## 模型文件

从 [OrbbecSDK_ROS1 v2.9.3 Release](https://github.com/orbbec/OrbbecSDK_ROS1/releases/tag/v2.9.3) 下载 `model.sm4`。模型文件必须与当前 ROS1 包使用的 OrbbecSDK 和 EnhancedDepthFilter 扩展库来自同一版本。GitHub 自动生成的 Source Code 压缩包不包含该模型文件。

通过 ROS launch 参数传入模型文件路径：

```text
enhanced_depth_model_path:=/path/to/model.sm4
```

ROS1 驱动要求显式设置 `enhanced_depth_model_path`，建议使用绝对路径。如果启用增强深度时模型路径为空或文件不存在，节点会启动失败。运行过程中不能更换模型文件。

## 启动要求

滤波器输入必须是同时包含 Color 和 Depth 帧的已对齐 frameset：

* 必须同时启用 Color 和 Depth 数据流；
* 软件对齐（`align_mode:=SW`）支持 D2C 和 C2D；
* 硬件对齐（`align_mode:=HW`）仅支持对齐到 Color，即 D2C；
* 建议设置 `frame_aggregate_mode:=full_frame`，保证每个 frameset 同时包含 Color 和 Depth。缺少其中任意一帧时，本帧不会执行 EnhancedDepthFilter。

## 启动示例

首次验证建议使用 Color `640x480 RGB` 和 Depth `640x480 Y16`。

普通节点：

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

Nodelet：

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

使用硬件 D2C 时，对齐目标只能是 Color：

```bash
roslaunch orbbec_camera gemini_330_series.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model.sm4 \
  depth_registration:=true \
  align_mode:=HW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

## 参数说明

* `enable_enhanced_depth`：是否启用增强深度滤波，默认 `false`。
* `enhanced_depth_model_path`：LingBot 模型文件路径，启用增强深度时必填。
* `enhanced_depth_confidence_threshold`：深度置信度阈值，必须是 `0` 到 `255` 之间的整数，默认 `51`。
* `depth_registration`：必须设置为 `true`，使对齐后的图像进入 EnhancedDepthFilter。
* `align_mode`：对齐方式，可设置为 `SW` 或 `HW`。
* `align_target_stream`：软件对齐可设置为 `COLOR` 或 `DEPTH`；硬件对齐必须设置为 `COLOR`。
* `frame_aggregate_mode`：建议设置为 `full_frame`，保证同时接收 Color 和 Depth 帧。

## 图像要求

D2C 时，Depth 对齐到 Color：

* Color 分辨率必须是 `640x480`、`1280x720` 或 `1280x800`；
* Depth 分辨率不受 EnhancedDepthFilter 限制，但所选配置必须支持 D2C。

C2D 时，Color 对齐到 Depth：

* Depth 分辨率必须是 `640x480`、`1280x720` 或 `1280x800`；
* Color 分辨率不受 EnhancedDepthFilter 限制。

支持的 Depth 输入格式为：

```text
Y10、Y11、Y12、Y14、Y16、Z16
```

EnhancedDepthFilter 的 Color 输入格式为 RGB。ROS1 驱动还接受以下 Color 流格式，并在滤波器专用的 frameset 副本中转换为 RGB：

```text
RGB、YUYV、UYVY、MJPG、BGR、RGBA、Y16、Y8
```

该转换只用于 EnhancedDepthFilter，不会改变驱动向下游发布的 Color 图像格式。

## SDK JSON 配置

通过 `load_config_json_file_path` 加载 SDK JSON 时，JSON 中的 stream profile 和 `application_config.point_cloud` 对齐配置会影响最终的 Color/Depth 分辨率、格式、对齐方式和帧汇聚模式。

启用 EnhancedDepthFilter 时，最终生效的配置仍需满足以下条件：

* Color 和 Depth 均已启用；
* D2C 或 C2D 对齐已启用；
* D2C 的 Color 目标分辨率或 C2D 的 Depth 目标分辨率属于 `640x480/1280x720/1280x800`；
* Depth 格式属于 `Y10/Y11/Y12/Y14/Y16/Z16`。

如果 launch/YAML 参数和 SDK JSON 配置同一项，已传入节点的 launch/YAML 参数优先。完整启动文件通过 `<param>` 写入的默认参数也属于已传入参数，可能覆盖 JSON 中的同名配置。详细规则请参考 [Gemini 330 系列 SDK JSON 使用说明](sdk_json_config.md)。

## 输出话题

以下话题名称以默认的 `camera_name:=camera` 为例。修改 `camera_name` 后，需要将 `/camera` 替换为实际命名空间。

| 话题 | 说明 |
| --- | --- |
| `/camera/depth/image_raw` | 滤波成功时发布增强后的对齐深度图像。运行中滤波失败时，驱动继续发布当前未增强的对齐深度图像。 |
| `/camera/depth/image_unaligned` | 软件对齐时发布对齐前的深度图像。硬件 D2C 模式不发布该话题。 |
| `/camera/confidence/image_raw` | 滤波成功时发布 `mono8` 编码的置信度图像。 |

普通节点和 Nodelet 使用相同的话题名称和发布规则。

## 确认运行状态

通过深度滤波状态话题确认 EnhancedDepthFilter 是否启用以及当前置信度阈值：

```bash
rostopic echo /camera/depth_filters/status
```

在输出中查找 `filter_name: "EnhancedDepthFilter"`，检查 `enabled` 和 `confidence_threshold`。

## 运行时调整

可以通过 `/camera/set_filter` 服务启停 EnhancedDepthFilter 或调整 `confidence_threshold`。位置参数和命名参数不能同时使用，阈值必须是 `0` 到 `255` 之间的整数。

使用位置参数启用滤波器并将阈值设置为 `60`：

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: [60]
filter_config: []"
```

使用命名参数设置阈值：

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: []
filter_config:
- name: 'confidence_threshold'
  value: '60'"
```

关闭 EnhancedDepthFilter：

```bash
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: false
filter_param: []
filter_config: []"
```

不支持通过该服务修改模型路径、数据流配置或对齐模式。如需更换模型文件，请修改 launch 参数后重新启动节点。

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 节点启动失败，提示找不到 EnhancedDepthFilter | 确认运行平台为 NVIDIA Jetson/Linux ARM64，并确认 ROS1 包中包含 EnhancedDepthFilter 扩展库。 |
| 提示缺少或不支持 License | 使用 LicenseTool 检查设备支持情况，并重新申请或激活有效的 LingBot-Depth License。 |
| 找不到 `model.sm4` 或模型初始化失败 | 检查 `enhanced_depth_model_path`，并确认模型、SDK 和扩展库来自同一版本。 |
| 提示分辨率或格式不支持 | 首先使用 Color `640x480 RGB` 和 Depth `640x480 Y16` 验证。 |
| 提示需要 D2C/C2D | 确认 `depth_registration:=true`，并检查 `align_mode` 和 `align_target_stream`。 |
| `/camera/depth/image_raw` 有数据但没有置信度图像 | 检查节点日志和 `/camera/depth_filters/status`。运行中滤波失败时，驱动会回退到未增强的深度帧。 |
