# Orbbec ROS SDK
[![stable](http://badges.github.io/stability-badges/dist/stable.svg)](http://github.com/badges/stability-badges) ![version](https://img.shields.io/badge/version-1.4.5-green)

Orbbec ROS SDK is a wrapper for OrbbecSDK that supports ROS Kinetic, Melodic, and Noetic distributions.
It enables smooth integration of Orbbec 3D cameras into ROS projects.

## Install dependencies

### ROS

- Please refer directly to ROS [wiki](http://wiki.ros.org/ROS/Installation).

### other dependencies

- Install dependencies (be careful with your ROS distribution)

  ```bash
  # Assuming you have sourced the ros environment, same below
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
  ```

Create ROS workspace(if you don't have one)

```bash
mkdir -p ~/ros_ws/src
```

Get Source code

```bash
cd ~/ros_ws/src/
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
```

Or you can download from [Orbbec developer community](https://developer.orbbec.com.cn/download.html)

Build

```bash
cd ~/ros_ws
catkin_make
```

- Install udev rules.

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
cd scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules 
sudo udevadm control --reload && sudo  udevadm trigger
```

Start the camera

- In terminal 1

```bash
source ./devel/setup.bash 
roslaunch orbbec_camera astra.launch
```

- In terminal 2

```bash
source ./devel/setup.bash
rviz
```

Select the topic you want to display

- Check topics / services/ parameters (open a new terminal)

```bash
rostopic list
rosservice list
rosparam list
```

- Get camera parameter, *MUST* start stream first.

```bash
rosservice call /camera/get_camera_params "{}"
```

- Check camera parameter, please refer to the ROS documentation for the meaning of the specific fields of the camera
  parameter [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

- Check device information

```bash
rosservice call /camera/get_device_info "{}" 
```

- Get the SDK version (Include firmware and Orbbec SDK version )

```bash
rosservice call /camera/get_version "{}"
```

- Set/get (auto) exposure

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}' 
rosservice call /camera/set_ir_auto_exposure  "{data: false}"
    
# Setting exposure values (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_ir_exposure  "{data: 2000}"
roservice call /camera/set_color_exposure  "{data: 2000}"
 # Get exposure
 rosservice call /camera/get_ir_exposure  "{}"
 rosservice call /camera/get_color_exposure "{}"
```

- Set/get gain

```bash
# Get Gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_ir_gain '{}' 
# Setting the gain (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_color_gain  "{data: 200}"
rosservice call /camera/set_ir_gain "{data: 200}"
```

- Set/get mirror

```bash
rosservice call /camera/set_color_mirror  "{data: true}"
rosservice call /camera/set_depth_mirror  "{data: true}"
rosservice call /camera/set_ir_mirror  "{data: true}"
```

- Set/get (auto) white balance

```bash
rosservice call /camera/set_auto_white_balance  "{data: false}"
rosservice call /camera/get_auto_white_balance  "{data: false}"
```

- Turn on/off laser

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

- Turn on/off fans

```bash
 rosservice call /camera/set_fan  '{data: true}' # Turn on
 rosservice call /camera/set_fan  '{data: false}' # Turn off
```

- Turn on/off LDP

```bash
rosservice  call /camera/set_ldp '{data: true}'  # Turn on
rosservice  call /camera/set_ldp '{data: false}' # Turn off

```

- Turn on/off sensors

```bash
rosservice call  /camera/toggle_ir "{data: true}"
rosservice call  /camera/toggle_color "{data: true}"
rosservice call  /camera/toggle_depth "{data: true}"
```

- Save image

```bash
rosservice call /camera/save_images "{}"
```

- save point cloud

```bash
rosservice call /camera/save_point_cloud "{}"
```

NOTE: The images are saved under ~/.ros/image and are only available when the sensor is on.

### All available service for camera control

The name of the following service already expresses its function.
However, it should be noted that the corresponding `set_[ir|depth|color]*`
and `get[ir|depth|color]*` **services are only available if you set** `enable[ir|depth|color]`
to `true` in the stream that corresponds to the argument of the launch file.

- `/camera/get_auto_white_balance`
- `/camera/get_camera_params`
- `/camera/get_color_auto_exposure`
- `/camera/get_color_camera_info`
- `/camera/get_color_exposure`
- `/camera/get_color_gain`
- `/camera/get_depth_auto_exposure`
- `/camera/get_depth_camera_info`
- `/camera/get_depth_exposure`
- `/camera/get_depth_gain`
- `/camera/get_device_info`
- `/camera/get_device_type`
- `/camera/get_ir_auto_exposure`
- `/camera/get_ir_camera_info`
- `/camera/get_ir_exposure`
- `/camera/get_ir_gain`
- `/camera/get_serial`
- `/camera/get_sdk_version`
- `/camera/get_white_balance`
- `/camera/reset_color_exposure`
- `/camera/reset_color_gain`
- `/camera/reset_depth_exposure`
- `/camera/reset_depth_gain`
- `/camera/reset_ir_exposure`
- `/camera/reset_ir_gain`
- `/camera/reset_white_balance`
- `/camera/save_images`
- `/camera/save_point_cloud`
- `/camera/set_auto_white_balance`
- `/camera/set_color_auto_exposure`
- `/camera/set_color_exposure`
- `/camera/set_color_gain`
- `/camera/set_color_mirror`
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_depth_mirror`
- `/camera/set_fan_work_mode`
- `/camera/set_floor`
- `/camera/set_ir_auto_exposure`
- `/camera/set_ir_exposure`
- `/camera/set_ir_gain`
- `/camera/set_ir_mirror`
- `/camera/set_laser`
- `/camera/set_ldp`
- `/camera/set_white_balance`
- `/camera/toggle_color`
- `/camera/toggle_depth`
- `/camera/toggle_ir`

### All available topics

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream image.
- `/camera/depth/image_raw`: The depth stream image
- `/camera/depth/points` : The point cloud, only available when  `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when  `enable_colored_point_cloud`
  is `true`.
- `/camera/ir/camera_info`:  The IR camera info.
- `/camera/ir/image_raw`: The IR stream image
- `/camera/accel/sample`: Acceleration data stream `enable_sync_output_accel_gyro`turned off，`enable_accel`turned on
- `/camera/gyro/sample`: Gyroscope data stream，enable_sync_output_accel_gyro`turned off，`enable_gyro`turned on
- `camera/gyro_accel/sample`: Synchronized data stream of acceleration and gyroscope，`enable_sync_output_accel_gyro`turned on

### Multiple cameras

- First, you need to enumerate the `usb_port` of the cameras. Plug them in and run the following command in the
  terminal, please make sure no camera is open before running:

  ```bash
  rosrun orbbec_camera list_devices_node 
  ```

or you can use the following script to get the serial number and usb port number, save the following script
as `list_ob_devices.sh`, `chmod +x list_ob_devices.sh`, and then run `./list_ob_devices.sh`.

```bash
VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null) # product name
      serial=$(cat "$dev/serial" 2>/dev/null) # serial number
      echo "Found Orbbec device $product, usb port $port, serial number $serial"
    fi
  fi
done
```

- Set the `device_num` parameter to the number of cameras you have.
- Go to the `orbbec-ros-sdk/launch/multi_xxx.launch` file and modify the `usb_port`.

```XML

<launch>
    <!-- unique camera name-->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <!-- TODO: change to your usb port -->
    <arg name="camera1_usb_port" default="1-4.2.1"/>

    <arg name="camera2_usb_port" default="1-4.2.2"/>
    -->Change serial number here -->
    <arg name="device_num" default="2"/>
    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="usb_port" value="$(arg camera1_usb_port)"/>
        <arg name="device_num" default="$(arg device_num)"/>

    </include>

    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="usb_port" value="$(arg camera2_usb_port)"/>
        <arg name="device_num" default="$(arg device_num)"/>
    </include>
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

 ```

- Launch

``` bash
roslaunch orbbec_camera multi_camera.launch
```

## Use hardware decoder to decode JPEG

### rockchip and Amlogic

Depends on `rockchip-mpp-dev` and `rockchip-rga-dev`, not all systems have these two packages, the names may be
different, please search by yourself.
Open `CMakeLists.txt` and set `USE_RK_HW_DECODER` to `ON`.

### Nvidia Jetson

Depends on: `jetson_multimedia_api`,`libyuv`.
Open `CMakeLists.txt` and set `USE_NV_HW_DECODER` to `ON`.

## Launch parameters

The following are the launch parameters available:

- `connection_delay`: The delay time in milliseconds for reopening the device.
  Some devices, such as Astra mini, require a longer time to initialize and
  reopening the device immediately can cause firmware crashes when hot plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
- `device_num`: The number of devices. This must be filled in if multiple cameras are required.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_ir`: Enables the IR camera.
- `depth_registration`: Enables hardware alignment the depth frame to color frame. This field is required when
  the `enable_colored_point_cloud` is set to `true`.
- `enable_publish_extrinsic`: Enables the publishing of camera extrinsic information.
- `log_level`: The log level for OrbbecSDK, with optional values of `none`, `info`, `debug`, `warn`, and `fatal`.
  The log file can be found in the ROS runtime directory, and the default location is `~/.ros/Log`.
- `enable_accel` : Enables the accelerometer.
- `accel_rate`: The frequency of the accelerometer, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
- `accel_range` : The range of the accelerometer, the optional values are `2g`,`4g`,`8g`,`16g`. The specific value
  depends on the current camera.
- `enable_gyro`: Whether to enable the gyroscope.
- `gyro_rate` : The frequency of the gyroscope, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
- `gyro_range` : The range of the gyroscope, the optional values
  are `16dps`,`31dps`,`62dps`,`125dps`,`250dps`,`500dps`,`1000dps`,`2000dps`. The specific value depends on the current
  camera.
- `enumerate_net_device` : Whether to enable the function of enumerating network devices. True means enabled, false means disabled.
  This feature is only supported by Femto Mega and Gemini 2 XL devices. When accessing these devices through the network, the IP address of the device needs to be configured in advance. The enable switch needs to be set to true.
- `depth_filter_config` : Configure the loading path for the depth filtering configuration file. By default, the depth filtering configuration file is located in the /config/depthfilter directory，Supported only on Gemini2.

## Depth work mode switch:
- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch is the camera's default configuration. If you need to modify it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```XML
  <!-- Depth work mode support is as follows： -->
  <!-- Unbinned Dense Default -->
  <!-- Unbinned Sparse Default -->
  <!-- Binned Sparse Default -->
  <!-- Obstacle Avoidance -->
  <arg name="depth_work_mode" default=""/>
```

- View depth work modes:

```bash
rosrun orbbec_camera list_depth_work_mode_node
```
## Configuration of depth NFOV and WFOV modes
For the Femto Mega and Femto Bolt devices, the NFOV and WFOV modes are implemented by configuring the resolution of Depth and IR in the launch file. 
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth  and the resolution of the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and resolutions is as follows:
- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## Check which profiles the camera supports

  ```bash
  rosrun orbbec_camera list_camera_profile_mode_node
  ```

## Launch files

| product serials                           | launch file          |
|-------------------------------------------|----------------------|
| astra+                                    | astra_adv.launch     |
| astra mini/astra mini pro/astra pro       | astra.launch         |
| astra mini s pro                          | astra.launch         |
| astra2                                    | astra2.launch        |
| astra stereo s                            | stereo_s_u3.launch   |
| astra pro2                                | astra_pro2.launch    |
| dabai                                     | dabai.launch         |
| dabai d1                                  | dabai_d1.launch      |
| dabai dcw                                 | dabai_dcw.launch     |
| dabai dw                                  | dabai_dw.launch      |
| dabai pro                                 | dabai_pro.launch     |
| deeya                                     | deeya.launch         |
| femto / femto w                           | femto.launch         |
| femto mega                                | femto_mega.launch    |
| femto bolt                                | femto_bolt.launch    |
| gemini                                    | gemini.launch        |
| gemini                                    | gemini.launch        |
| gemini2 / dabai DCL                       | gemini2.launch       |
| gemini2L                                  | gemini2L.launch      |
| gemini e                                  | gemini_e.launch      |
| gemini e lite                             | gemini_e_lite.launch |
| dabai max                                 | dabai_max.launch     |
| dabai max pro                             | dabai_max_pro.launch |
| gemini uw                                 | gemini_uw.launch     |
| dabai dcw2                                | dabai_dcw2.launch    |
| dabai dw2                                 | dabai_dw2.launch     |
| gemini ew                                 | gemini_ew.launch     |
| gemini ew lite                            | gemini_ew_lite.launch|

Actually, All launch files all most the same, the only difference is the default value of the parameters.

## Use Nodelet

For users who need to use nodelet, please refer to `gemini2_nodelet.launch`

## Supported hardware products

| **products list** | **firmware version**                      |
|-------------------|-------------------------------------------|
| Gemini 2 XL       | Obox: V1.2.5  VL:1.4.54                   |
| Astra 2           | 2.8.20                                    |
| Gemini 2 L        | 1.4.32                                    |
| Gemini 2          | 1.4.60 /1.4.76                            |
| Femto Mega        | 1.1.7  (window10、ubuntu20.04、ubuntu22.04) |
| Astra+            | 1.0.22/1.0.21/1.0.20/1.0.19               |
| Femto             | 1.6.7                                     |
| Femto W           | 1.1.8                                     |
| Femto Bolt        | 1.0.6 (unsupported ARNM32)                |
| DaBai             | 2436                                      |
| DaBai DCW         | 2460                                      |
| DaBai DW          | 2606                                      |
| Astra Mini Pro    | 1007                                      |
| Gemini E          | 3460                                      |
| Gemini E Lite     | 3606                                      |
| Gemini            | 3.0.18                                    |
| Astra Mini S Pro  | 1.0.05                                    |
| DaBai Max Pro     | 1.0.06                                    |
| Gemini UW         | 1.0.060                                   |

## Frequently Asked Questions

No Picture from Multiple Cameras

- it's possible that the power supply is insufficient.
  To avoid this, do not connect all cameras to the same hub and use a powered hub instead.

- It's also possible that the resolution is too high.
  To resolve this, try lowering the resolution.

Why are there so many launch files here

- The reason for the presence of multiple launch
  files is due to the fact that the default resolutions and image formats of different cameras vary.
  To make it easier to use, the launch files have been separated for each camera.

Why IR stream No data output

- If you are using Astra/Astra mini / astra mini S camera, you cannot turn on IR and RGB at the same time, please
  change `enable_color` to false in the corresponding launch file to turn off color camera.

Why do I get the following error when running the launch file

```text
Failed to start device: usbEnumerator createUsbDevice failed!
```

- The current device does not have permission to access the device， check the PID of the current device

```bash
lsusb | grep 2bc5
```

Your output should look like this

```text
Bus 002 Device 007: ID 2bc5:your_pid_here
```

Edit`/etc/udev/rules.d/99-obsensor-libusb.rules`，add the following line

```bash
SUBSYSTEM=="usb", ATTR{idProduct}=="your_pid_here", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root"  GROUP:="video", SYMLINK+="you_device_name_here"
```

`you_device_name_here` is the name of the device you want to create, for example `Astra`.

Then restart the udev service

```bash
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
