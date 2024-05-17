## Using Multiple Cameras with the Orbbec ROS Package

This section describes how to configure and use multiple Orbbec cameras simultaneously in a ROS 1 environment.

### Identifying Camera USB Ports

#### Script to List Connected Cameras

To determine which USB ports the cameras are connected to, you can use the following bash script. This script lists all Orbbec devices attached to the system along with their USB port and serial number.

```bash
#!/bin/bash

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

Save this script to a file and execute it in your terminal to output a list of connected cameras.
Or you can run the following command to list all connected Orbbec cameras:

```bash
rosrun orbbec_camera list_ob_devices.sh
```

### Launching Multiple Cameras

#### Setup for Multiple Camera Launch

You can launch multiple cameras by specifying different USB ports for each camera. Below is an example XML file that uses the ROS 1 launch system to start two cameras with individual configurations.

```XML
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

#### Running the Launch File

To execute the launch configuration for multiple cameras, use the command:

```bash
roslaunch orbbec_camera multi_camera.launch
```

### Configuring the TF Tree for Multiple Cameras

#### Example TF Configuration for Two Cameras

When using multiple cameras, it's essential to calibrate them and publish a static TF tree for each camera. The following ROS 1 XML launch configuration sets up the TF tree based on your calibration results:

```XML
<launch>
    <node pkg="tf" type="static_transform_publisher" name="camera_01_tf" args="0 0 0 0 0 0 base_link camera_01_link" />
    <node pkg="tf" type="static_transform_publisher" name="camera_02_tf" args="0 0 0 0 0 0 base_link camera_02_link" />
</launch>
```

Save this configuration as `multi_camera_tf.launch` in the launch directory of the Orbbec camera package. To run it, use:

```bash
roslaunch orbbec_camera multi_camera_tf.launch
```