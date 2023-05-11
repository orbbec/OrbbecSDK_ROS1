# Orbbec ROS SDK

Orbbec ROS SDK 是 OrbbecSDK 的 ROS wrapper，支持 Kinetic, Melodic, 和 Noetic.

## 安装依赖

### ROS

- 请直接参考ROS官方[文档](http://wiki.ros.org/ROS/Installation).

### 其他依赖

- 安装其他依赖 (注意你的ROS发行版)

```bash
  # 假定你已经source了你的ROS环境，下同
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

创建ROS的工作空间（如果你事先没有）

```bash
mkdir -p ~/ros_ws/src
```

把代码放到 ROS 工作空间

```bash
cd ~/ros_ws/src/
git clone https://github.com/OrbbecDeveloper/OrbbecSDK_ROS1.git
```

或者您可以从[Orbbec 开发者社区](https://developer.orbbec.com.cn/download.html)下载

构建

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

启动相机

- 在终端 1

```bash
source ./devel/setup.bash 
roslaunch orbbec_camera astra.launch
```

- 在终端 2

```bash
source ./devel/setup.bash
rviz
```

选择你想要显示的topic

- 查看 topics / services/ parameters (open a new terminal)

```bash
rostopic list
rosservice list
rosparam list
```

- 获取相机参数，**必须** 先开流

```bash
rosservice call /camera/get_camera_params "{}"
```

- 检查相机参数，请参考 ROS 文档以了解相机参数中具体字段的含义 [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

- 检查设备信息。

```bash
rosservice call /camera/get_device_info "{}" 
```

- 获取SDK版本（包括固件和Orbbec SDK版本）

```bash
rosservice call /camera/get_version "{}"
```

- 设置/获取（自动）曝光

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}' 
rosservice call /camera/set_ir_auto_exposure  "{data: false}"
    
# 设置曝光值（注意数据范围，以下示例可能不正确）
rosservice call /camera/set_ir_exposure  "{data: 2000}"
roservice call /camera/set_color_exposure  "{data: 2000}"
 # 获取曝光值。
 rosservice call /camera/get_ir_exposure  "{}"
 rosservice call /camera/get_color_exposure "{}"
```

- 设置/获取增益。

```bash
# 获取增益。
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_ir_gain '{}' 
# 设置增益（注意数据范围，以下示例可能不正确）。
rosservice call /camera/set_color_gain  "{data: 200}"
rosservice call /camera/set_ir_gain "{data: 200}"
```

- 设置/获取镜像。

```bash
rosservice call /camera/set_color_mirror  "{data: true}"
rosservice call /camera/set_depth_mirror  "{data: true}"
rosservice call /camera/set_ir_mirror  "{data: true}"
```

- 设置/获取（自动）白平衡。

```bash
rosservice call /camera/set_auto_white_balance  "{data: false}"
rosservice call /camera/get_auto_white_balance  "{data: false}"
```

- 打开/关闭激光。

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

- 打开/关闭风扇。

```bash
 rosservice call /camera/set_fan  '{data: true}' # Turn on
 rosservice call /camera/set_fan  '{data: false}' # Turn off
```

- 打开/关闭 LDP

```bash
ros2 service call /camera/set_ldp '{data: true}'  # Turn on
ros2 service call /camera/set_ldp '{data: false}' # Turn off

```

- T打开/关闭传感器

```bash
rosservice call  /camera/toggle_ir "{data: true}"
rosservice call  /camera/toggle_color "{data: true}"
rosservice call  /camera/toggle_depth "{data: true}"
```

- 保存图像。

```bash
rosservice call /camera/save_images "{}"
```

注意：图像保存在 `~/.ros/image` 下，仅在传感器打开时才可用

### 相机控制的所有可用service

以下服务的名称已经表达了它们的功能。
但需要注意的是，相应的 `set_[ir|depth|color]` 和 `get[ir|depth|color]`,
服务仅在你在对应的数据流中设置`enable[ir|depth|color]` 为 `true` 时才可用。

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

### 所有可用的话题（topics）

- `/camera/color/camera_info` : 彩色相机信息（CameraInfo）话题。
- `/camera/color/image_raw`: 彩色数据流图像话题。
- `/camera/depth/camera_info`: 深度数据流图像话题。
- `/camera/depth/image_raw`: 红外数据流图像话题。
- `/camera/depth/points` : 点云话题，仅当 `enable_point_cloud` 为 `true` 时才可用`.
- `/camera/depth_registered/points`: 彩色点云话题，仅当 `enable_colored_point_cloud` 为 `true` 时才可用。
- `/camera/ir/camera_info`:  红外相机信息（CameraInfo）话题。
- `/camera/ir/image_raw`: 红外数据流图像话题。

### 多相机

- 首先，您需要枚举相机的序列号。将相机连接并在终端中运行以下命令:

```bash
  rosrun orbbec_camera list_devices_node 
```

- 将 `device_num` 参数设置为您拥有的相机数量。
- 打开 `orbbec-ros-sdk/launch/multi_xxx.launch` 文件并修改序列号。目前，不同的相机只能通过其序列号进行区分。

```XML
<launch>
    <!-- unique camera name-->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_serila_number" default="AU094930073"/>
    -->Change serial number here -->
    <arg name="camera2_serila_number" default="AU1D41100NH"/>
    -->Change serial number here -->
    <arg name="device_num" default="2"/>
    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="serial_number" value="$(arg camera1_serila_number)"/>
        <arg name="device_num" default="$(arg device_num)"/>

    </include>

    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="serial_number" value="$(arg camera2_serila_number)"/>
        <arg name="device_num" default="$(arg device_num)"/>
    </include>
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

 ```

- 启动

``` bash
roslaunch orbbec_camera multi_camera.launch
```

## 启动参数

以下是可用的启动参数:

- `connection_delay`: 重新打开设备的延迟时间，单位为毫秒。某些设备，例如 Astra mini，需要更长的时间来初始化，立即重新打开设备可能会在热插拔时导致固件崩溃。
- `enable_point_cloud`: 启用点云。
- `enable_colored_point_cloud`: 启用彩色点云。
- `enable_d2c_viewer`: 发布 D2C 叠加图像（仅用于测试）。
- `device_num`: 设备数量。如果需要多个相机，则必须填写此项。
- `color_width`, `color_height`, `color_fps`: 彩色数据流的分辨率和帧率。
- `ir_width`, `ir_height`, `ir_fps`: 红外数据流的分辨率和帧率。
- `depth_width`, `depth_height`, `depth_fps`: 深度数据流的分辨率和帧率。
- `enable_color`: 启用彩色相机。
- `enable_depth`: 启用深度相机。
- `enable_ir`: 启用红外相机。
- `depth_registration`: 启用硬件对准将深度帧对准到彩色帧。当 `enable_colored_point_cloud` 设置为 `true` 时需要此字段。
- `enable_publish_extrinsic`: 启用相机外参信息的publisher。
- `log_level`: `OrbbecSDK` 的日志级别，可选值为 `none`、`info`、`debug`、`warn` 和 `fatal`。日志文件可以在 ROS 运行时目录中找到，其默认位置为`~/.ros/Log`。

## 常见问题

多个相机没有图像

- 可能是USB供电不足导致的。为避免此问题，请不要将所有相机连接到同一个HUB上，还要使用供电的HUB。

- 也有可能是分辨率太高导致的。要解决此问题，请尝试降低分辨率。

为什么这里有这么多启动文件？

- 存在多个启动文件的原因是因为不同相机的默认分辨率和图像格式不同。
为了方便使用，启动文件已针对每个相机进行了分离

为什么IR数据没有输出

- 如果你用的是Astra/Astra mini相机，是不能同时打开IR和RGB的， 请在对应的launch文件把`enable_color`改成false关闭彩色相机

## License

Copyright 2023 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
