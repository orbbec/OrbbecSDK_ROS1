# Multi-Camera Synchronization Verification Node

**File path:** [image_sync_example_node.cpp](https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/examples/multi_camera_time_sync/image_sync_example_node.cpp)

This example node performs synchronized capture and timestamp verification across multiple Orbbec cameras and supports **1 to 8 image topics** at a time.
At startup, it automatically discovers published color/depth image topics and can be used to validate frame alignment accuracy under the multi-camera **Primary / Secondary Synced** mode.

---

## Usage Guide

1. **Modify `multi_camera_synced.launch` as follows**

   - Adjust the include blocks according to the number of cameras participating in synchronization.

   - Select the appropriate launch file based on your camera model.
     For example, use `gemini_330_series.launch` for the Gemini 330 series.

   - Naming convention:
     `camera_name` should follow the format `camera_01`, `camera_02`, `camera_03`, ...

   - Configure the USB ports using:

     `rosrun orbbec_camera list_devices_node`

     to view and bind the correct ports.

   - Set synchronization mode to **Primary/Secondary Synced**,
     with one camera as **primary** and the others as **secondary_synced**.

   - **Startup sequence:**
     Start secondary cameras first, and start the **Primary camera last** to ensure the synchronization signal is established correctly.

---

2. **Modify configuration files**

   Edit the following two files under the `config` directory:

   - `camera_params.yaml`

   - `camera_secondary_params.yaml`

   Ensure the following settings are consistent across all cameras:

   - Enable both **depth** and **color** streams.

   - Use the same **frame rate (fps)** for all cameras.

---

3. **Launch and Verification**

   - Start the multi-camera synchronization launch file:

     `roslaunch orbbec_camera multi_camera_synced.launch`

   - In a new terminal, run the synchronization verification node:

     `rosrun orbbec_camera image_sync_example_node`

     The node automatically discovers color/depth image topics from the running cameras, displays synchronized images, and outputs timestamp differences and FPS statistics for synchronization validation.

---

4. **Four-Camera Reference Launch File**

```xml
<launch>
  <arg name="camera1_name" default="camera_01"/>
  <arg name="camera2_name" default="camera_02"/>
  <arg name="camera3_name" default="camera_03"/>
  <arg name="camera4_name" default="camera_04"/>

  <arg name="camera1_usb_port" default="2-2.3"/>
  <arg name="camera2_usb_port" default="2-1"/>
  <arg name="camera3_usb_port" default="2-3"/>
  <arg name="camera4_usb_port" default="2-4"/>

  <arg name="primary_config" default="$(find orbbec_camera)/config/camera_params.yaml"/>
  <arg name="secondary_config" default="$(find orbbec_camera)/config/camera_secondary_params.yaml"/>

  <!-- secondary cameras -->
  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera2_name)"/>
    <arg name="usb_port" value="$(arg camera2_usb_port)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera3_name)"/>
    <arg name="usb_port" value="$(arg camera3_usb_port)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera4_name)"/>
    <arg name="usb_port" value="$(arg camera4_usb_port)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <!-- primary camera -->
  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera1_name)"/>
    <arg name="usb_port" value="$(arg camera1_usb_port)"/>
    <arg name="sync_mode" value="primary"/>
    <arg name="config_file_path" value="$(arg primary_config)"/>
    <arg name="trigger_out_enabled" value="true"/>
  </include>
</launch>
```

---

## System Configuration Requirements

1. **Increase USB Buffer Memory**

   Prevent frame drops caused by concurrent data transmission:

   `echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`

2. **Tune ROS1 Runtime Performance**

   Optimize CPU and USB usage to reduce transmission delay and instability.
   See [this section](../performance/lower_cpu_usage.md) for detailed tuning instructions.
