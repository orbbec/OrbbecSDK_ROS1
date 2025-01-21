## 使用Orbbec ROS包配置多个摄像头(Nodelet)

本节描述了如何在ROS 1环境将多个摄像头的Node节点放进同一个Nodelet_manager，请确保你已查看过[multi_camera.MD](multi_camera.MD)

本节所使用的案例launch文件为`multi_camera_nodelet.launch`与`gemini_330_series_nodelet.launch`

### 配置Nodelet_manager

你可以通过设置`manager`参数启动具体名称的**Nodelet_manager**。以下是一个使用ROS1启动一个名为`orbbec_camera_manager`的**Nodelet_manager**。

```bash
<launch>
    <arg name="manager" default="orbbec_camera_manager"/>

    <node unless="false" pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager"
              output="screen" required="false" respawn="false"/>
</launch>
```

### 配置相机Nodelet

在这个示例中，我们启动了一个名为 `orbbec_camera_manager` 的  **Nodelet Manager** ，并且通过传递 `manager` 参数来启动两个 `gemini_330_series_nodelet.launch` 文件。通过这种方式，两个相机的 `nodelet` 都将由同一个 `orbbec_camera_manager` 来管理。

```bash
<launch>
    <arg name="camera_name" default="ob_camera"/>
    <arg name="3d_sensor" default="gemini_330_series_nodelet"/>
    <arg name="external_manager" default="true"/>
    <arg name="manager" default="orbbec_camera_manager"/>
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_usb_port" default="4-1"/>
    <arg name="camera2_usb_port" default="4-2"/>
    <arg name="device_num" default="2"/>

    <node unless="false" pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager"
              output="screen" required="false" respawn="false"/>

    <include file="$(find orbbec_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="manager" value="$(arg manager)"/>
        <arg name="external_manager" value="$(arg external_manager)"/>
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="usb_port" value="$(arg camera1_usb_port)"/>
        <arg name="device_num" value="$(arg device_num)"/>
    </include>

    <include file="$(find orbbec_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="manager" value="$(arg manager)"/>
        <arg name="external_manager" value="$(arg external_manager)"/>
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="usb_port" value="$(arg camera2_usb_port)"/>
        <arg name="device_num" value="$(arg device_num)"/>
    </include>
</launch>
```

#### 运行启动文件

```bash
roslaunch orbbec_camera multi_camera_nodelet.launch
```

### 配置相机参数

#### 多台相机相同参数

当你需要将所有相机的配置修改为相同时，由于 `multi_camera_nodelet.launch` 中启动的相机节点是基于 `gemini_330_series_nodelet.launch` 文件的，因此你只需要修改 `gemini_330_series_nodelet.launch` 中的相关参数即可。

#### 多台相机不同参数

当你需要为多个相机配置不同的参数时，例如修改 `enable_hardware_noise_removal_filter` 参数，你可以在 `multi_camera_nodelet.launch` 文件中为每个相机单独配置不同的值，从而实现每个相机的个性化配置。

```bash
<launch>
    <arg name="camera_name" default="ob_camera"/>
    <arg name="3d_sensor" default="gemini_330_series_nodelet"/>
    <arg name="external_manager" default="true"/>
    <arg name="manager" default="orbbec_camera_manager"/>
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <arg name="camera1_usb_port" default="4-1"/>
    <arg name="camera2_usb_port" default="4-2"/>
    <arg name="device_num" default="2"/>

    <node unless="false" pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager"
              output="screen" required="false" respawn="false"/>

    <include file="$(find orbbec_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="manager" value="$(arg manager)"/>
        <arg name="external_manager" value="$(arg external_manager)"/>
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="usb_port" value="$(arg camera1_usb_port)"/>
        <arg name="device_num" value="$(arg device_num)"/>
        <arg name="enable_hardware_noise_removal_filter" default="true"/>
    </include>

    <include file="$(find orbbec_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="manager" value="$(arg manager)"/>
        <arg name="external_manager" value="$(arg external_manager)"/>
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="usb_port" value="$(arg camera2_usb_port)"/>
        <arg name="device_num" value="$(arg device_num)"/>
        <arg name="enable_hardware_noise_removal_filter" default="false"/>
    </include>
</launch>

```
