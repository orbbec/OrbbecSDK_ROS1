1. 安装ROS，参考 http://wiki.ros.org/ROS/Installation

2. 编译Orbbec ROS SDK

   ```
   mkdir -p catkin_ws/src

   catkin_make

   source devel/setup.bash

   cd catkin_src/src

   git clone https://code.orbbec.com.cn/OrbbecSDK/orbbec-ros-sdk.git

   mv orbbec-ros-sdk orbbec_camera

   cd .. && catkin_make

   roscd orbbec_camera

   cd scripts && ./install

   cd ~/catkin_ws

   catkin_make
   ```

3. 运行Orbbec ROS SDK

   roslaunch orbbec_camera device.launch

4. 运行多设备launch

   roslaunch orbbec_camera multidevice.launch

5. launch说明

   ```
   <arg name="camera" default="camera" />
   <arg name="device_name" default="" />
   <arg name="sn" default="" />
   <arg name="vid" default="0" />
   <arg name="pid" default="0" />
   ```
   
   - device_name设置要启动的设备名称
   - sn设置要启动的设备sn
   - vid设置要启动的设备vid（十进制）
   - pid设置要启动的设备pid（十进制）

   可以只设置其中一项或者几项的组合来启动特定的设备，如只设置pid就可以启动任意pid为设置的值的设备，多设备情况下如果两个设备是相同类型的设备则必须设置sn才能区分

6. 查看可用topic

   ```
   rostopic list
   ```

   在没有修改launch中的camera参数的情况下，单设备topic为所有以/camera开头的topic，多设备topic为所有以/camera1和/camera2开头的topic

7. 查看可用service

   ```
   rosservice list
   ```

   在没有修改launch中的camera参数的情况下，单设备service为所有以/camera开头的service，多设备service为所有以/camera1和/camera2开头的service

8. 动态参数调节

   ```
   rosrun rqt_reconfigure rqt_reconfigure
   ```

   目前可供调节的参数只有彩色、深度、IR相机的分辨率及帧率
