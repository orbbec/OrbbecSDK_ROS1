# Using Multiple Cameras with the Orbbec ROS Package

This guide describes how to configure and use multiple Orbbec cameras simultaneously in a ROS 1 environment.

## Prerequisites
First asume that you have already installed the Orbbec ROS package. If not, please refer to the [Orbbec ROS Package Installation Guide](../README.MD).

## 1. Increase usbfs_memory_mb Value (CRITICAL STEP)

**IMPORTANT: This step is crucial for multi-camera setups. Without increasing the usbfs_memory_mb value, you may not receive any data from your cameras.**

For multi-camera setups, it's essential to increase the `usbfs_memory_mb` value. Set it to 128MB (adjustable based on your system's needs) by running:

```bash
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

To make this change permanent, refer to [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

If you skip this step or set the value too low, your cameras may not function properly or may not provide any data at all.

## 2. Identifying Camera USB Ports

### Script to List Connected Cameras

Use this bash script to list all connected Orbbec devices with their USB ports and serial numbers:

```bash
#!/bin/bash

VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null)
      serial=$(cat "$dev/serial" 2>/dev/null)
      echo "Found Orbbec device $product, usb port $port, serial number $serial"
    fi
  fi
done
```

Save and execute this script, or use the ROS command:

```bash
rosrun orbbec_camera list_ob_devices.sh
```

## 3. Launching Multiple Cameras

### Setup for Multiple Camera Launch

Create a launch file (e.g., `multi_camera.launch`) with individual configurations for each camera:

```xml
<launch>
    <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
        <arg name="camera_name" value="camera_01"/>
        <arg name="usb_port" value="2-3.4.4.4.1"/>
        <arg name="device_num" value="2"/>
        <arg name="sync_mode" value="free_run"/>
    </include>

    <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
        <arg name="camera_name" value="camera_02"/>
        <arg name="usb_port" value="2-3.4.4.4.3"/>
        <arg name="device_num" value="2"/>
        <arg name="sync_mode" value="free_run"/>
    </include>
</launch>
```

### Running the Launch File

Execute the launch configuration with:

```bash
roslaunch orbbec_camera multi_camera.launch
```

## 4. Configuring the TF Tree for Multiple Cameras

Create a TF configuration file (e.g., `multi_camera_tf.launch`) for your calibrated camera setup:

```xml
<launch>
    <node pkg="tf" type="static_transform_publisher" name="camera_01_tf" args="0 0 0 0 0 0 base_link camera_01_link" />
    <node pkg="tf" type="static_transform_publisher" name="camera_02_tf" args="0 0 0 0 0 0 base_link camera_02_link" />
</launch>
```

Run the TF configuration:

```bash
roslaunch orbbec_camera multi_camera_tf.launch
```

This setup allows you to use multiple Orbbec cameras simultaneously in your ROS environment. Remember, 
the first step of increasing the usbfs_memory_mb value is critical for ensuring your cameras function correctly in a multi-camera setup.
