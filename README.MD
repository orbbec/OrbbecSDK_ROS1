# Orbbec ROS SDK

Orbbec Sensor SDK ROS wrapper This package supports ROS Kinetic, Melodic and Noetic distributions

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

- Check camera extrinsic parameter(from depth to color)

```bash
rostopic echo /camera/extrinsic/depth_to_color
```

- Get camera parameter, *MUST* start stream first.

```bash
rosservice call /camera/get_camera_params "{}"
```

- Check camera parameter, please refer to the ROS documentation for the meaning of the specific fields of the camera parameter [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)

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
ros2 service call /camera/set_ldp '{data: true}'
ros2 service call /camera/set_ldp '{data: false}'
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
NOTE: The images are saved under ~/.ros/image and are only available when the sensor is on.

### **Multiple cameras**

- First, you need to enumerate the serial number of the camera, plug in the cameras and run

  ```bash
  rosrun orbbec_camera list_devices_node 
  ```
**Set the parameter `number_of_devices` to the number of cameras**

- Go to the `ros_astra_camera/launch/multi_xxx.launch`   and  change the serial number. Currently, different cameras can only be distinguished by the serial number,

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
      <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
          <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
          <arg name="serial_number" value="$(arg camera1_serila_number)"/>
      </include>
  
      <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
          <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
          <arg name="serial_number" value="$(arg camera2_serila_number)"/>
      </include>
      <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf" args="0 0 0 0 0 0 camera01_link camera02_link"/>
  </launch>
  ```
- Launch

``` bash
roslaunch orbbec_camera multi_camera.launch
```
## Launch parameters

`reconnection_delay`, The delay time for reopening the device in seconds. Some devices would take longer time to initialize, such as Astra mini, so reopening the device immediately would causes firmware crashes when hot plug.
- `depth_align`, Enables hardware depth to color alignment, requires RGB point cloud to open
- `enable_pointcloud`, Whether to enable point cloud.
- `enable_pointcloud_xyzrgb`, Whether to enable RGB point cloud.
- `enable_d2c_viewer`, Publish D2C overlay image.
- `device_num`,The number of devices, You need to fill in the number of devices when you need multiple cameras.
- `color_width`， `color_height`， `color_fps`， color stream resolution and frame rate.
- `ir_width`， `ir_height`， `ir_fps`，IR stream resolution and frame rate
- `depth_width`， `depth_height`， `depth_fps` depth stream resolution and frame rate
- `enable_color`， Whether to enable  RGB camera, this parameter has no effect when the RGB camera is UVC protocol
- `enable_depth` , Whether to enable depth camera
- `enable_ir`, Whether to enable IR camera
- `flip_[color|ir|depth]` Publish Mirror of origin image

## Frequently Asked Questions

- No image when  multiple cameras
  - Maybe the power supply is not sufficient, consider to connect the camera with a powered USB hub.
  - Maybe the resolution is too high, lower the resolution to test
- Hot-plug image anomaly
  - The `reconnection_delay` parameter can be set to bigger, as some devices take longer time to initialize and may not have completed the initialization of the device.
- No image when hot-plugging
  - Check if the data cable is plugged in well
  - Try to connect to a powered usb hub, the ARM development board may have unstable power supply causing the device fail to repoen.
- The frame rate of a point cloud is very low, Consideration increased by adding an udp buffer

   ```bash
    sudo sysctl -w net.core.rmem_max=8388608 net.core.rmem_default=8388608
   ```
## License

Copyright 2022 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific language governing permissions and limitations under the License.

*Other names and brands may be claimed as the property of others*
