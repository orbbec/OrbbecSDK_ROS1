# 可用服务

服务名称直观地反映了其用途。需要注意的是，所有与设置/获取参数相关的服务（以 `set_*` 与 `get_*` 表示）只有在对应的 `enable_*` 参数被启用时才会生效。例如，启用左红外（IR）`enable_left_ir`、右红外 `enable_right_ir`、深度 `enable_depth` 或彩色 `enable_color`（参见 [启动参数](./launch_parameters.md)）是使用相应服务的前提条件。这样可以确保只有当启动文件的流参数开启时，才会提供对应流的服务。

## 彩色流（Color Stream）

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
# 在 data_param 中，四个值分别为：左、右、上、下
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

## 深度流（Depth Stream）

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
# v2 版本暂不支持
rosservice call /camera/set_depth_gain 64
```

* `/camera/set_depth_ae_roi`

```bash
# 在 data_param 中，四个值分别为：左、右、上、下
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

## 红外流（IR Stream）

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

## 运行时数据流配置

* `/camera/set_stream_profile`

用于在节点运行期间切换一个或多个已启用图像流的 Profile。`stream_name` 支持 `color`、`left_color`、`right_color`、`depth`、`ir`、`left_ir` 和 `right_ir`。宽、高、帧率或格式可以只填写需要修改的字段；未修改的数值字段填写 `0`，格式填写空字符串。切换时节点会停止并重新启动数据流；如果目标 Profile 已经生效，服务会返回失败。

```bash
rosservice call /camera/set_stream_profile "{profiles: [{stream_name: 'color', width: 1280, height: 720, fps: 30, format: 'MJPG'}]}"
```

* `/camera/set_image_registration_mode`

运行时切换深度和彩色图像对齐模式。可选值为 `OFF`、`HW_D2C`、`SW_D2C` 和 `SW_C2D`，大小写不敏感。除 `OFF` 外，彩色流和深度流必须同时启用。切换时节点会自动重启数据流；失败时会恢复原对齐模式。

```bash
rosservice call /camera/set_image_registration_mode "{data: 'HW_D2C'}"
```

## 传感器与发射器控制（Sensor & Emitter Control）

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

## 设备信息与管理（Device Information & Management）

* `/camera/get_device_info`

```bash
rosservice call /camera/get_device_info
```

* `/camera/get_device_config`

获取当前生效的设备配置状态，例如 preset、对齐模式、时间域、同步模式、帧聚合模式等。

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

导出当前设备配置为 SDK JSON 文件。Gemini 330 系列的 JSON 导入导出流程请参考 [Gemini 330 系列 SDK JSON 使用说明](../5_advanced_guide/configuration/sdk_json_config.md)。

```bash
rosservice call /camera/export_config_json "data: '/tmp/orbbec_camera_config.json'"
```

* `/camera/set_bag_recording`

使用 SDK bag 录制当前设备数据。`enable: true` 开始录制，`enable: false` 停止录制；`file_path` 为空时使用当前工作目录下的默认文件名。

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

## 同步（Synchronization）

* `/camera/get_ptp_config`

```bash
rosservice call /camera/get_ptp_config
```

* `/camera/set_ptp_config`

```bash
rosservice call /camera/set_ptp_config true
```

* `/camera/send_software_trigger`

```bash
rosservice call /camera/send_software_trigger "data: true"
```

* `/camera/set_sync_io_voltage_level`

设置同步 IO 电压等级。仅支持具备该属性的设备。

```bash
rosservice call /camera/set_sync_io_voltage_level 0
```

## 视差配置

* `/camera/set_disparity_range_mode`

```bash
rosservice call /camera/set_disparity_range_mode 1
```

* `/camera/set_disparity_search_offset`

```bash
rosservice call /camera/set_disparity_search_offset 0
```

## AE 配置

* `/camera/set_ae_reference_stream`

```bash
rosservice call /camera/set_ae_reference_stream color
```

> **支持模组**：Gemini 301 系列。

* `/camera/set_ae_strategy`

```bash
rosservice call /camera/set_ae_strategy motion
```

> **支持模组**：Gemini 301 系列。

## 深度滤波器配置（Depth Filter Configuration）

* `/camera/set_filter`

`FalsePositiveFilter` 的启动参数、状态确认和命名参数调参示例可参考 [Gemini 330 系列 FalsePositiveFilter 使用说明](../5_advanced_guide/configuration/false_positive_filter.md)。`EnhancedDepthFilter` 的环境要求、启动参数和状态确认方法可参考 [Gemini 330 系列 EnhancedDepthFilter 使用说明](../5_advanced_guide/configuration/enhanced_depth_filter.md)。

```bash
# filter_name 为滤波器名称，filter_enable 表示是否开启滤波器开关。
# filter_param 为旧的按位置传参方式；filter_config 为新的命名参数方式。
# filter_param 和 filter_config 不能同时使用。

# 设置 DecimationFilter: [scale]
rosservice call /camera/set_filter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

# 设置 SpatialAdvancedFilter: [alpha, disp_diff, magnitude, radius]
rosservice call /camera/set_filter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

# 设置 SequenceIdFilter: [sequence_id]
rosservice call /camera/set_filter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

# 设置 ThresholdFilter: [min, max]
rosservice call /camera/set_filter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

# 设置 NoiseRemovalFilter: [min_diff, max_size]
rosservice call /camera/set_filter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

# 设置 HardwareNoiseRemoval: [threshold]
rosservice call /camera/set_filter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: [0.2]}'

# 设置 SpatialFastFilter：[radius]
rosservice call /camera/set_filter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

# 设置 SpatialModerateFilter: [disp_diff, magnitude, radius]
rosservice call /camera/set_filter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'

# 设置 FalsePositiveFilter: []
rosservice call /camera/set_filter '{filter_name: FalsePositiveFilter, filter_enable: true, filter_param: []}'

# 设置 EnhancedDepthFilter: [confidence_threshold]，阈值必须是 0 到 255 之间的整数
rosservice call /camera/set_filter '{filter_name: EnhancedDepthFilter, filter_enable: true, filter_param: [60]}'

# 设置 MgcNoiseRemovalFilter / LutNoiseRemovalFilter: []
rosservice call /camera/set_filter '{filter_name: MgcNoiseRemovalFilter, filter_enable: true, filter_param: []}'
rosservice call /camera/set_filter '{filter_name: LutNoiseRemovalFilter, filter_enable: true, filter_param: []}'

# 设置 EdgeNoiseRemovalFilter: []
rosservice call /camera/set_filter '{filter_name: EdgeNoiseRemovalFilter, filter_enable: true, filter_param: []}'

# 使用 filter_config 按参数名调参
rosservice call /camera/set_filter "{filter_name: NoiseRemovalFilter, filter_enable: true, filter_config: [{name: min_diff, value: '256'}, {name: max_size, value: '80'}]}"
rosservice call /camera/set_filter "{filter_name: HardwareNoiseRemovalFilter, filter_enable: true, filter_config: [{name: threshold, value: '0.2'}]}"
rosservice call /camera/set_filter "{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_config: [{name: alpha, value: '0.5'}, {name: disp_diff, value: '160'}, {name: magnitude, value: '1'}, {name: radius, value: '8'}]}"
rosservice call /camera/set_filter "{filter_name: EnhancedDepthFilter, filter_enable: true, filter_config: [{name: confidence_threshold, value: '60'}]}"

# 设置 DispOutliersFilter。search_mode 支持 FULL 或 OFFSET_80，大小写不敏感。
rosservice call /camera/set_filter "{filter_name: DispOutliersFilter, filter_enable: true, filter_config: [{name: search_mode, value: 'FULL'}]}"
```

`/camera/set_filter` 只能启停 EnhancedDepthFilter 或调整 `confidence_threshold`，不能修改模型路径、数据流配置或对齐方式；这些内容必须通过启动参数配置。

## 数据采集与标定管理（Data Capture & Calibration Management）

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
