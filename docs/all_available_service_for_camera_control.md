# All available service for camera control

The service names intuitively reflect their purposes. It's crucial to understand that services related to setting or
getting parameters—denoted as `set_*` and `get_*`—become available only when the respective `enable_*` parameters are
activated. For instance, enabling features such as left infrared (IR) with `enable_left_ir`, right IR
with `enable_right_ir`, depth sensing with `enable_depth`, or color processing with `enable_color` (refer
to [Launch Parameters](./launch_parameters.md)) is a prerequisite for their corresponding services to be operational. This
configuration ensures that services are accessible only when their specific stream is enabled in the
launch file's stream argument.


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
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'
```

* `/camera/set_color_mirror`

```bash
rosservice call /camera/set_color_mirror 1
```

* `/camera/toggle_color`

```bash
rosservice call /camera/set_toggle_color 1
```

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
#v2 version is not supported yet
rosservice call /camera/set_depth_gain 64
```

* `/camera/set_depth_ae_roi`

```bash
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.
rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'
```

* `/camera/set_depth_mirror`

```bash
rosservice call /camera/set_depth_mirror 1
```

* `/camera/toggle_depth`

```bash
rosservice call /camera/set_toggle_depth 1
```

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
rosservice call /camera/set_toggle_ir 1
```

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

* `/camera/get_camera_params`

```bash
rosservice call /camera/get_camera_params
```

* `/camera/get_sdk_version`

```bash
rosservice call /camera/get_sdk_version
```

* `/camera/reboot_device`

```bash
rosservice call /camera/reboot_device
```

* `/camera/save_images`

```bash
rosservice call /camera/save_images
```

* `/camera/save_point_cloud`

```bash
rosservice call /camera/save_point_cloud
```

* `/camera/set_fan_work_mode`

```bash
rosservice call /camera/set_fan_work_mode 1
```

* `/camera/set_filter`

```bash
#filter_name is the filter name,filter_enable is whether to enable the filter switch,and filter_param is the filter parameter

#set DecimationFilter
rosservice call /camera/set_filter '{filter_name: DecimationFilter,filter_enable: false,filter_param: [5]}'

#set SpatialAdvancedFilter
rosservice call /camera/set_filter '{filter_name: SpatialAdvancedFilter,filter_enable: true,filter_param: [0.5,160,1,8]}'

#set SequenceIdFilter
rosservice call /camera/set_filter '{filter_name: SequenceIdFilter,filter_enable: true,filter_param: [1]}'

#set ThresholdFilter
rosservice call /camera/set_filter '{filter_name: SequenceIdFilter,filter_enable: true,filter_param: [0,15999]}'

#set NoiseRemovalFilter
rosservice call /camera/set_filter '{filter_name: NoiseRemovalFilter,filter_enable: true,filter_param: [256,80]}'

#set HardwareNoiseRemoval
rosservice call /camera/set_filter '{filter_name: HardwareNoiseRemoval,filter_enable: true,filter_param: []}'
```
