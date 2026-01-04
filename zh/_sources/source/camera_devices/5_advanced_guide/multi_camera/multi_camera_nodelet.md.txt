## 使用Orbbec ROS包配置多相机（Nodelet）

本节描述如何在ROS 1环境中在同一个Nodelet Manager中设置多个相机节点。请确保您已经阅读了[多相机文档](./multi_camera.md)。

本节使用的示例启动文件是`multi_camera_nodelet.launch`和`gemini_330_series_nodelet.launch`。

### 配置Nodelet Manager

您可以通过设置`manager`参数来启动特定的**Nodelet Manager**。以下是在ROS 1中启动名为`orbbec_camera_manager`的**Nodelet Manager**的示例。

```bash
<launch>
    <arg name="manager" default="orbbec_camera_manager"/>

    <node unless="false" pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager"
              output="screen" required="false" respawn="false"/>
</launch>

```

### 配置相机Nodelets

在此示例中，我们启动一个名为`orbbec_camera_manager`的**Nodelet Manager**，然后使用`manager`参数启动两个`gemini_330_series_nodelet.launch`文件。通过这样做，两个相机nodelets都将由同一个`orbbec_camera_manager`管理。

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

**运行启动文件**

```bash
roslaunch orbbec_camera multi_camera_nodelet.launch
```

### 配置相机参数

**所有相机的相同参数**

当您需要对所有相机应用相同的配置时，由于在`multi_camera_nodelet.launch`中启动的相机节点基于`gemini_330_series_nodelet.launch`文件，您只需在`gemini_330_series_nodelet.launch`中修改相关参数即可。

**每个相机的不同参数**

当您需要为每个相机配置不同的参数时（例如，修改`enable_hardware_noise_removal_filter`参数），您可以在`multi_camera_nodelet.launch`文件中为每个相机设置不同的值。这允许您自定义每个相机的配置。

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
