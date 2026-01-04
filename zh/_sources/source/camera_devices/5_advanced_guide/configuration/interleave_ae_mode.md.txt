# 在Gemini330系列相机中使用interleave_ae

> 本节描述如何在Gemini 330系列相机中使用interleave_ae（最低相机固件版本[1.4.00](https://www.orbbec.com/docs/g330-firmware-release/)）

## 参数介绍

interleave_ae相关参数在[gemini_330_series.launch](https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/launch/gemini_330_series.launch)中设置。

* `interleave_ae_mode` : 设置激光或hdr交替。
* `interleave_frame_enable` : 启用交替帧模式。
* `interleave_skip_enable` : 启用跳帧模式。
* `interleave_skip_index` : 设置为0跳过模式红外，设置为1跳过泛光红外。

**交替hdr**

当`interleave_ae_mode`参数设置为`hdr`且`interleave_frame_enable`设置为`true`时，将启用交替hdr

* `hdr_index1_laser_control` : 帧1激光开关设置。
* `hdr_index1_depth_exposure` : 帧1深度曝光值设置，非AE模式。
* `hdr_index1_depth_gain` : 帧1深度增益值设置，非AE模式。
* `hdr_index1_ir_brightness` : 帧1红外增益值设置。
* `hdr_index1_ir_ae_max_exposure` : 帧1红外在AE（自动曝光）中的最大曝光值设置。
* `hdr_index0_laser_control`: 帧0激光开关设置。
* `hdr_index0_depth_exposure`: 帧0深度曝光值设置，非AE模式。
* `hdr_index0_depth_gain` : 帧0深度增益值设置，非AE模式。
* `hdr_index0_ir_brightness` : 帧0红外增益值设置。
* `hdr_index0_ir_ae_max_exposure` : 帧0红外在AE（自动曝光）中的最大曝光值设置。

**交替激光**

当`interleave_ae_mode`参数设置为`laser`且`interleave_frame_enable`设置为`true`时，将启用交替激光

* `laser_index1_laser_control` : 帧1激光开关设置。
* `laser_index1_depth_exposure` : 帧1深度曝光值设置，非AE模式。
* `laser_index1_depth_gain` : 帧1深度增益值设置，非AE模式。
* `laser_index1_ir_brightness` : 帧1红外增益值设置。
* `laser_index1_ir_ae_max_exposure` : 帧1红外在AE（自动曝光）中的最大曝光值设置。
* `laser_index0_laser_control` : 帧0激光开关设置。
* `laser_index0_depth_exposure` : 帧0深度曝光值设置，非AE模式。
* `laser_index0_depth_gain` : 帧0深度增益值设置，非AE模式。
* `laser_index0_ir_brightness` : 帧0红外增益值设置。
* `laser_index0_ir_ae_max_exposure` : 帧0红外在AE（自动曝光）中的最大曝光值设置。

## 运行启动文件

设置interleave_ae参数，再次`colcon build`并运行启动文件

```bash
roslaunch orbbec_camera gemini_330_series.launch
```

**Example Visualization**

![Depth Point Cloud Visualization](../../image/interleave_ae_mode/interleave_ae0.jpeg)

![Depth Point Cloud Visualization](../../image/interleave_ae_mode/interleave_ae1.jpeg)

## Multi_camera_synced + Interleave_ae

Please refer to [multi_camera_synced](../multi_camera/multi_camera_synced.md) and [Parameter Introduction](#parameter-introduction)
