Of course. I have analyzed the ROS2 service documentation and am ready to reformat the ROS1 version. Here is the reformatted document.

***

# All available services for camera control

> **Note:** Services related to a specific stream (e.g., `/camera/set_color_*`) are only available if that stream is enabled in the launch file (e.g., by setting the corresponding `enable_color` parameter to true).

### Stream Control

#### Color Stream

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
    # data_param: [Left, Right, Top, Bottom]
    rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'
    ```
*   `/camera/set_color_mirror`
    ```bash
    rosservice call /camera/set_color_mirror 1
    ```

#### Depth Stream

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
    # v2 version is not supported yet
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
    # data_param: [Left, Right, Top, Bottom]
    rosservice call /camera/set_depth_ae_roi '{data_param:[0,1279,0,719]}'
    ```
*   `/camera/set_depth_mirror`
    ```bash
    rosservice call /camera/set_depth_mirror 1
    ```

#### IR Stream

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

### Sensor & Emitter Control

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

### Device Information & Management

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
    ```*   `/camera/get_device_type`
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

### Synchronization

*   `/camera/get_ptp_config` & `/camera/set_ptp_config`
    ```bash
    rosservice call /camera/get_ptp_config
    rosservice call /camera/set_ptp_config true
    ```

### Depth Filter Configuration

*   `/camera/set_filter`
    ```bash
    # filter_name, filter_enable, filter_param
    
    # Set DecimationFilter
    rosservice call /camera/set_filter '{filter_name: DecimationFilter, filter_enable: false, filter_param: [5]}'
    
    # Set SpatialAdvancedFilter
    rosservice call /camera/set_filter '{filter_name: SpatialAdvancedFilter, filter_enable: true, filter_param: [0.5,160,1,8]}'
    
    # Set SequenceIdFilter
    rosservice call /camera/set_filter '{filter_name: SequenceIdFilter, filter_enable: true, filter_param: [1]}'
    
    # Set ThresholdFilter
    rosservice call /camera/set_filter '{filter_name: ThresholdFilter, filter_enable: true, filter_param: [0,15999]}'
    
    # Set NoiseRemovalFilter
    rosservice call /camera/set_filter '{filter_name: NoiseRemovalFilter, filter_enable: true, filter_param: [256,80]}'
    
    # Set HardwareNoiseRemoval
    rosservice call /camera/set_filter '{filter_name: HardwareNoiseRemoval, filter_enable: true, filter_param: []}'
    
    # Set SpatialFastFilter: [radius]
    rosservice call /camera/set_filter '{filter_name: SpatialFastFilter, filter_enable: true, filter_param: [4]}'
    
    # Set SpatialModerateFilter: [disp_diff, magnitude, radius]
    rosservice call /camera/set_filter '{filter_name: SpatialModerateFilter, filter_enable: true, filter_param: [160,1,3]}'
    ```

### Data Capture & Calibration Management

*   `/camera/get_camera_params`
    
    ```bash
    rosservice call /camera/get_camera_params
    ```
*   `/camera/get_color_camera_info`, `/camera/get_depth_camera_info`, `/camera/get_ir_camera_info`
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
