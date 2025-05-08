## Configuring Multiple Cameras with Orbbec ROS Package (Nodelet)

This section describes how to set up multiple camera Node nodes in the same Nodelet Manager in a ROS 1 environment. Make sure you've already reviewed the [multi_camera document](../multi_camera/).

The example launch files used in this section are `multi_camera_nodelet.launch` and `gemini_330_series_nodelet.launch`.

### Configuring the Nodelet Manager

You can start a specific **Nodelet Manager** by setting the `manager` parameter. Here's an example of launching a **Nodelet Manager** named `orbbec_camera_manager` in ROS 1.

```bash
<launch>
    <arg name="manager" default="orbbec_camera_manager"/>

    <node unless="false" pkg="nodelet" type="nodelet" name="$(arg manager)" args="manager"
              output="screen" required="false" respawn="false"/>
</launch>

```

### Configuring Camera Nodelets

In this example, we launch a **Nodelet Manager** called `orbbec_camera_manager`, and then use the `manager` parameter to launch two `gemini_330_series_nodelet.launch` files. By doing this, both camera nodelets will be managed by the same `orbbec_camera_manager`.

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

#### Running the Launch File

```bash
roslaunch orbbec_camera multi_camera_nodelet.launch
```

### Configuring Camera Parameters

#### Identical Parameters for All Cameras

When you need to apply the same configuration to all cameras, since the camera nodes launched in `multi_camera_nodelet.launch` are based on the `gemini_330_series_nodelet.launch` file, you only need to modify the relevant parameters in `gemini_330_series_nodelet.launch`.

#### Different Parameters for Each Camera

When you need to configure different parameters for each camera (for example, modifying the `enable_hardware_noise_removal_filter` parameter), you can set different values for each camera in the `multi_camera_nodelet.launch` file. This allows you to customize each camera's configuration.

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
