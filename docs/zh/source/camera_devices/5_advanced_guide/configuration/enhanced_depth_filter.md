# ROS1 EnhancedDepthFilter 使用说明

## 适用范围

EnhancedDepthFilter 用于 Gemini 330 系列的增强深度输出。当前 ROS1 包随 SDK 只在 Linux arm64 目录下打包了 enhanced depth filter 扩展库；其它平台如果启用该滤波，SDK 可能会直接报错。

## 前置条件

1.  设备需要写入新版 LingBot License。
    
2.  需要准备外挂模型文件。
    
3.  启动时必须同时开启 color 和 depth。
    
4.  必须启用 D2C/C2D 对齐，并开启帧汇聚，保证同一帧集中同时拿到 color 和 depth。
    

模型文件路径通过 ROS launch 参数传入：

```plaintext
enhanced_depth_model_path:=/path/to/model/file
```

如果开启增强深度但模型路径为空或文件不存在，节点会启动失败。

## 启动示例

普通节点：

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

nodelet：

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

如果要用硬件 D2C，对齐目标只能是 COLOR：

```plaintext
roslaunch orbbec_camera gemini_330_series.launch \
  enable_enhanced_depth:=true \
  enhanced_depth_model_path:=/path/to/model/file \
  depth_registration:=true \
  align_mode:=HW \
  align_target_stream:=COLOR \
  frame_aggregate_mode:=full_frame
```

## 参数说明

*   `enable_enhanced_depth`: 是否启用增强深度滤波，默认 `false`。
    
*   `enhanced_depth_model_path`: LingBot 模型文件路径，启用增强深度时必填。
    
*   `enhanced_depth_confidence_threshold`: 置信度阈值，默认 `51`。
    
*   `depth_registration`: 是否启用对齐，EnhancedDepthFilter 要求为 `true`。
    
*   `align_mode`: 对齐模式，支持 `HW` 或 `SW`。`HW` 只能做 D2C。
    
*   `align_target_stream`: 对齐目标，支持 `COLOR` 或 `DEPTH`。`COLOR` 表示 D2C，`DEPTH` 表示 C2D。
    
*   `frame_aggregate_mode`: 帧汇聚模式，建议设置为 `full_frame`。
    

## 图像要求

D2C 时，depth 对齐到 color：

*   `align_mode:=HW` 或 `align_target_stream:=COLOR`。
    
*   color 分辨率必须是 `640x480`、`1280x720` 或 `1280x800` 之一。
    
*   depth 分辨率不限制，但 depth 格式必须是 SDK 支持的增强深度格式。
    

C2D 时，color 对齐到 depth：

*   `align_mode:=SW align_target_stream:=DEPTH`。
    
*   depth 分辨率必须是 `640x480`、`1280x720` 或 `1280x800` 之一。
    
*   color 分辨率不限制。
    

格式要求：

*   EnhancedDepthFilter 最终输入的 color 为 `RGB`。驱动会尝试把 `YUYV`、`UYVY`、`MJPG`、`BGR`、`RGBA`、`Y16`、`Y8` 转换为 `RGB`。
    
*   depth 格式支持 `Y10`、`Y11`、`Y12`、`Y14`、`Y16`、`Z16`。
    

## SDK JSON 配置

如果通过 `load_config_json_file_path` 加载 SDK JSON，JSON 中的 profile 和 `point_cloud.align_mode` 会影响最终的 color/depth 分辨率和对齐方式。

启用 EnhancedDepthFilter 时，仍需要保证：

*   JSON 或 launch 中最终启用了 color 和 depth。
    
*   JSON 或 launch 中最终启用了 D2C/C2D 对齐。
    
*   D2C 的 color 目标分辨率，或 C2D 的 depth 目标分辨率，在 `640x480/1280x720/1280x800` 之内。
    

如果 launch 参数和 SDK JSON 同时设置了同一项，显式传入的 launch 参数优先。

## 输出话题

优化后的深度图像发布在：

```plaintext
/camera/depth/image_raw
```

对齐前的原始深度图像发布在：

```plaintext
/camera/depth/image_unaligned
```

置信度图像发布在：

```plaintext
/camera/confidence/image_raw
```

如果 `camera_name` 不是默认的 `camera`，话题前缀会随 `camera_name` 改变。

## 运行时调整

可以通过滤波服务启停 EnhancedDepthFilter 或调整 `confidence_threshold`。

使用位置参数设置阈值：

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'
filter_enable: true
filter_param: [60]
filter_config: []"
```

使用命名参数设置阈值：

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'filter_enable: truefilter_param: []filter_config:- name: 'confidence_threshold'  value: '60'"
```

关闭 EnhancedDepthFilter：

```plaintext
rosservice call /camera/set_filter "filter_name: 'EnhancedDepthFilter'filter_enable: falsefilter_param: []filter_config: []"
```

不支持运行时修改 `model_path`。如需更换模型文件，请修改 launch 参数后重新启动节点。

## 常见错误

`Enhanced depth filter requires enhanced_depth_model_path`

表示启用了 EnhancedDepthFilter，但没有传入模型文件路径。

`Enhanced depth model file not found`

表示传入的模型文件路径不存在或节点进程无法访问。

`Enhanced depth filter requires D2C/C2D align mode`

表示没有启用 `depth_registration`，或对齐模式不是 `HW`/`SW`。

`Enhanced depth filter requires supported target resolutions: 640x480/1280x720/1280x800`

表示当前对齐目标流的分辨率不在支持列表内。D2C 时检查 color 分辨率，C2D 时检查 depth 分辨率。

`Enhanced depth filter requires supported target resolutions: 640x480/1280x720/1280x800 and depth formats: Y10/Y11/Y12/Y14/Y16/Z16`

表示 depth 分辨率或 depth 格式不满足要求。
