# 相机控制的所有可用服务

> **注意：** 与特定流相关的服务（例如，`/camera/set_color_*`）仅在启动文件中启用该流时才可用（例如，通过将相应的`enable_color`参数设置为true）。

### 流控制

#### 彩色流

*   `/camera/toggle_color`
    ```bash
    rosservice call /camera/toggle_color 1
    ```
*   `/camera/get_color_exposure` & `/camera/set_color_exposure`
    ```bash
    rosservice call /camera/get_color_exposure
    rosservice call /camera/set_color_exposure 100
    ```
*   `/camera/get_color_gain` & `/camera/set_color_gain`
    ```bash
    rosservice call /camera/get_color_gain
    rosservice call /camera/set_color_gain 16
    ```
*   `/camera/get_color_auto_exposure` & `/camera/set_color_auto_exposure`
    ```bash
    rosservice call /camera/get_color_auto_exposure
    rosservice call /camera/set_color_auto_exposure 1
    ```
*   `/camera/reset_color_exposure` & `/camera/reset_color_gain`
    ```bash
    rosservice call /camera/reset_color_exposure
    rosservice call /camera/reset_color_gain
    ```
*   `/camera/set_color_ae_roi`
    ```bash
    # data_param: [左，右，上，下]
    rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'
    ```
*   `/camera/set_color_mirror`
    ```bash
    rosservice call /camera/set_color_mirror 1
    ```

#### 深度流

*   `/camera/toggle_depth`
    ```bash
    rosservice call /camera/toggle_depth 1
    ```
*   `/camera/get_depth_exposure` & `/camera/set_depth_exposure`
    ```bash
    rosservice call /camera/get_depth_exposure
    rosservice call /camera/set_depth_exposure 3000
    ```
*   `/camera/get_depth_gain` & `/camera/set_depth_gain`
    ```bash
    rosservice call /camera/get_depth_gain
    # v2版本暂不支持
    rosservice call /camera/set_depth_gain 64
    ```
*   `/camera/get_depth_auto_exposure` & `/camera/set_depth_auto_exposure`
    ```bash
    rosservice call /camera/get_depth_auto_exposure
    rosservice call /camera/set_depth_auto_exposure 1
    ```
*   `/camera/reset_depth_exposure` & `/camera/reset_depth_gain`
    ```bash
    rosservice call /camera/reset_depth_exposure
    rosservice call /camera/reset_depth_gain
    ```
*   `/camera/set_depth_ae_roi`
    ```bash
    # data_param: [左，右，上，下]
    rosservice call /camera/set_depth_ae_roi '{data_param:[0,1279,0,719]}'
    ```
*   `/camera/set_depth_mirror`
    ```bash
    rosservice call /camera/set_depth_mirror 1
    ```

#### 红外流

*   `/camera/toggle_ir`
    ```bash
    rosservice call /camera/toggle_ir 1
    ```
*   `/camera/get_ir_exposure` & `/camera/set_ir_exposure`
    ```bash
    rosservice call /camera/get_ir_exposure
    rosservice call /camera/set_ir_exposure 3000
    ```
*   `/camera/get_ir_gain` & `/camera/set_ir_gain`
    ```bash
    rosservice call /camera/get_ir_gain
    rosservice call /camera/set_ir_gain 64
    ```
*   `/camera/set_ir_auto_exposure`
    ```bash
    rosservice call /camera/set_ir_auto_exposure 1
    ```
*   `/camera/reset_ir_exposure` & `/camera/reset_ir_gain`
    ```bash
    rosservice call /camera/reset_ir_exposure
    rosservice call /camera/reset_ir_gain
    ```
*   `/camera/set_ir_mirror`
    ```bash
    rosservice call /camera/set_ir_mirror 1
    ```
*   `/camera/switch_ir`
    ```bash
    rosservice call /camera/switch_ir left
    ```

### 传感器和发射器控制

*   `/camera/get_auto_white_balance` & `/camera/set_auto_white_balance`
    ```bash
    rosservice call /camera/get_auto_white_balance
    rosservice call /camera/set_auto_white_balance 1
    ```
*   `/camera/get_white_balance` & `/camera/reset_white_balance`
    ```bash
    rosservice call /camera/get_white_balance
    rosservice call /camera/reset_white_balance
    ```
*   `/camera/set_laser`
    ```bash
    rosservice call /camera/set_laser 1
    ```
*   `/camera/get_ldp_status` & `/camera/set_ldp`

    ```bash
    rosservice call /camera/get_ldp_status
    rosservice call /camera/set_ldp 1
    ```
*   `/camera/set_flood`
    ```bash
    rosservice call /camera/set_flood 1
    ```
*   `/camera/set_fan_work_mode`
    ```bash
    rosservice call /camera/set_fan_work_mode 1
    ```

### 设备信息和管理

*   `/camera/get_device_info`
    ```bash
    rosservice call /camera/get_device_info
    ```
*   `/camera/get_sdk_version`
    ```bash
    rosservice call /camera/get_sdk_version
    ```
*   `/camera/get_serial`
    ```bash
    rosservice call /camera/get_serial
    ```
*   `/camera/get_device_type`
    ```bash
    rosservice call /camera/get_device_type
    ```
*   `/camera/get_lrm_measure_distance`

    ```bash
    rosservice call /camera/get_lrm_measure_distance
    ```
*   `/camera/reboot_device`
    ```bash
    rosservice call /camera/reboot_device
    ```

### 同步

*   `/camera/get_ptp_config` & `/camera/set_ptp_config`
    ```bash
    rosservice call /camera/get_ptp_config
    rosservice call /camera/set_ptp_config true
    ```

### 深度滤波器配置

*   `/camera/set_filter`
    ```bash
    # filter_name, filter_enable, filter_param

    # 设置抽取滤波器
    rosservice call /camera/set_filter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'

    # 设置空间高级滤波器
    rosservice call /camera/set_filter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'

    # 设置序列ID滤波器
    rosservice call /camera/set_filter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'

    # 设置阈值滤波器
    rosservice call /camera/set_filter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'

    # 设置噪声移除滤波器
    rosservice call /camera/set_filter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'

    # 设置硬件噪声移除
    rosservice call /camera/set_filter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: []}'

    # 设置空间快速滤波器：[半径]
    rosservice call /camera/set_filter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'

    # 设置空间中等滤波器：[视差差异, 幅度, 半径]
    rosservice call /camera/set_filter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'
    ```

### 数据捕获和标定管理

*   `/camera/get_camera_params`

    ```bash
    rosservice call /camera/get_camera_params
    ```
*   `/camera/get_color_camera_info`、`/camera/get_depth_camera_info`、`/camera/get_ir_camera_info`
    ```bash
    rosservice call /camera/get_color_camera_info
    rosservice call /camera/get_depth_camera_info
    rosservice call /camera/get_ir_camera_info
    ```
*   `/camera/save_images`
    ```bash
    rosservice call /camera/save_images
    ```
*   `/camera/save_point_cloud`
    ```bash
    rosservice call /camera/save_point_cloud
    ```
