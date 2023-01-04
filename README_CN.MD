# Orbbec ROS SDK

Orbbec Sensor SDK 的ROS封装，支持ROS Kinetic, Melodic 和 Noetic 版本

## 安装依赖

### ROS

- 请直接参考ROS [wiki](http://wiki.ros.org/ROS/Installation)。

### 其他依赖

- Install dependencies (be careful with your ROS distribution)
- 安装依赖（注意ROS版本）

  ```bash
  # 假定你已经source了ros环境，下同
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
  ```

- 获取源码, 在[奥比中光3D开发者社区](https://developer.orbbec.com.cn/download.html)下载
- Build

```bash
cd ~/ros_ws
catkin_make
```

- 安装udev rules

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
cd scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules 
sudo udevadm control --reload && sudo  udevadm trigger
```

- 启动相机
-在终端1

```bash
source ./devel/setup.bash 
roslaunch orbbec_camera astra.launch
```

- 在终端2

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

- 查看 camera extrinsic parameter(from depth to color)

```bash
rostopic echo /camera/extrinsic/depth_to_color
```

- 查看相机内参， 需要先启动相机

```bash
rosservice call /camera/get_camera_params "{}"
```

- 查看相机参数，具体含义请参考ROS文档 [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

- 查看设备信息

```bash
rosservice call /camera/get_device_info "{}" 
```

- 查看SDK版本

```bash
rosservice call /camera/get_version "{}"
```

- 设置/获取自动曝光

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

- 设置/获取增益

```bash
# Get Gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_ir_gain '{}' 
# Setting the gain (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_color_gain  "{data: 200}"
rosservice call /camera/set_ir_gain "{data: 200}"
```

- 设置/获取镜像

```bash
rosservice call /camera/set_color_mirror  "{data: true}"
rosservice call /camera/set_depth_mirror  "{data: true}"
rosservice call /camera/set_ir_mirror  "{data: true}"
```

- 设置/获取（自动）白平衡

```bash
rosservice call /camera/set_auto_white_balance  "{data: false}"
rosservice call /camera/get_auto_white_balance  "{data: false}"
```

- 打开/关闭激光

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

- 打开/关闭风扇

```bash
 rosservice call /camera/set_fan  '{data: true}' # Turn on
 rosservice call /camera/set_fan  '{data: false}' # Turn off
```

- 打开/关闭 LDP

```bash
ros2 service call /camera/set_ldp '{data: true}'
ros2 service call /camera/set_ldp '{data: false}'
```

- 打开/关闭传感器

```bash
rosservice call  /camera/toggle_ir "{data: true}"
rosservice call  /camera/toggle_color "{data: true}"
rosservice call  /camera/toggle_depth "{data: true}"
```

- 保存图像

```bash
rosservice call /camera/save_images "{}"
```

NOTE: T图像保存在 ~/.ros/image， 只有对应传感器启动才会保存。

### 多相机

- 首先，需要枚举相机的序列号，插入相机后运行

  ```bash
  rosrun orbbec_camera list_devices_node 
  ```

**Set the parameter `device_num` to the number of cameras**

- cd 到 `orbbec-ros-sdk/launch/multi_xxx.launch`，并修改对应的serial number.

```xml
  <launch>
      <!-- unique camera name-->
      <arg name="camera_name" default="camera"/>
      <!-- Hardware depth registration -->
      <arg name="3d_sensor" default="astra"/>
      <!-- stereo_s_u3, astrapro, astra -->
      <arg name="camera1_prefix" default="01"/>
      <arg name="camera2_prefix" default="02"/>
      <arg name="camera1_serila_number" default="AU094930073"/> <-->Change serial number here </-->
      <arg name="camera2_serila_number" default="AU1D41100NH"/> <-->Change serial number here </-->
      <arg name="device_num" default="2"/
      <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
          <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
          <arg name="serial_number" value="$(arg camera1_serila_number)"/>
          <arg name="device_num" default="$(arg device_num)"/

      </include>
  
      <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
          <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
          <arg name="serial_number" value="$(arg camera2_serila_number)"/>
          <arg name="device_num" default="$(arg device_num)"/>
      </include>
      <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf" args="0 0 0 0 0 0 camera01_link camera02_link"/>
  </launch>
  ```

- 启动

``` bash
roslaunch orbbec_camera multi_camera.launch
```

## 启动参数

- `depth_align`, 开启硬件对齐，如果需要彩色点云和深度对齐，需要打开，默认关闭。
- `enable_point_cloud`, 是否开启点云.
- `enable_point_cloud_xyzrgb`, 是否开启RGB点云
- `enable_d2c_viewer`, 发布彩色对齐到深度的图像.
- `device_num`,相机数量，如果是多相机，需要设置为相机数量.
- `color_width`， `color_height`， `color_fps`， 彩色图像分辨率和帧率.
- `ir_width`， `ir_height`， `ir_fps`， 红外图像分辨率和帧率.
- `depth_width`， `depth_height`， `depth_fps` ，深度图像分辨率和帧率.
- `enable_color`， `enable_ir`， `enable_depth`， 是否开启对应传感器.
- `flip_[color|ir|depth]` ，是否翻转图像.

## 常见问题

- 多像机时没有图像
  - 可能是电源不足，考虑用供电的USB集线器连接摄像机。
  - 可能是分辨率太高，降低分辨率进行测试
- 热拔插图像异常
  - 可以把`connection_delay`参数设置得大一些，因为有些设备的初始化时间比较长，可能还没有完成设备的初始化。
- 热插拔时没有图像
  - 检查数据线是否插好了
  - 尝试连接到一个有电源的USB集线器，ARM开发板可能有不稳定的电源导致设备无法重新启动。
- 点云的帧率很低，可以考虑通过添加一个udp缓冲器来提高帧率

   ```bash
    sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
   ```

## License

Copyright 2022 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
