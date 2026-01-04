# 在Orbbec ROS包中使用多个相机

本指南描述了如何在ROS 1环境中同时配置和使用多个Orbbec相机。

## 前提条件

首先假设您已经安装了Orbbec ROS包。如果没有，请参考[Orbbec ROS包安装指南](../../2_installation/build_the_package.md)。

## 增加usbfs_memory_mb值（关键步骤）

**重要提示：这个步骤对于多相机设置至关重要。如果不增加usbfs_memory_mb值，您可能无法从相机接收任何数据。**

对于多相机设置，增加`usbfs_memory_mb`值是必要的。将其设置为128MB（可根据系统需要调整）：

```bash
echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

要使此更改永久生效，请参考[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

如果您跳过此步骤或将值设置得太低，您的相机可能无法正常工作或根本无法提供任何数据。

## 识别相机USB端口

**列出已连接相机的脚本**

使用此bash脚本列出所有已连接的Orbbec设备及其USB端口和序列号：

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

保存并执行此脚本，或使用ROS命令：

```bash
rosrun orbbec_camera list_ob_devices.sh
```

## 启动多相机

**多相机启动设置**

创建一个启动文件（例如，`multi_camera.launch`），为每个相机配置单独的设置：

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

**运行启动文件**

使用以下命令执行启动配置：

```bash
roslaunch orbbec_camera multi_camera.launch
```

## 为多相机配置TF树

为您的标定相机设置创建TF配置文件（例如，`multi_camera_tf.launch`）：

```xml
<launch>
    <node pkg="tf" type="static_transform_publisher" name="camera_01_tf" args="0 0 0 0 0 0 base_link camera_01_link" />
    <node pkg="tf" type="static_transform_publisher" name="camera_02_tf" args="0 0 0 0 0 0 base_link camera_02_link" />
</launch>
```

运行TF配置：

```bash
roslaunch orbbec_camera multi_camera_tf.launch
```

此设置允许您在ROS环境中同时使用多个Orbbec相机。请记住，
增加usbfs_memory_mb值的第一步对于确保您的相机在多相机设置中正常工作至关重要。
