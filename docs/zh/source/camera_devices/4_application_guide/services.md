# 相机控制的所有可用服务

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

## 深度滤波器配置（Depth Filter Configuration）

* `/camera/set_filter`

```bash
# filter_name 为滤波器名称，filter_enable 表示是否开启滤波器开关，filter_param 表示滤波参数

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
```

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
