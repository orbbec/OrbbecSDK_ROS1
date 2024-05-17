## 使用Orbbec ROS包配置多个摄像头

本节描述了如何在ROS 1环境中配置和使用多个Orbbec摄像头。

### 识别摄像头USB端口

#### 列出已连接摄像头的脚本

为了确定摄像头连接到哪些USB端口，您可以使用以下bash脚本。此脚本列出了连接到系统的所有Orbbec设备及其USB端口和序列号。

```bash
#!/bin/bash

VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null) # 产品名称
      serial=$(cat "$dev/serial" 2>/dev/null) # 序列号
      echo "找到Orbbec设备 $product，usb端口 $port，序列号 $serial"
    fi
  fi
done
```

将此脚本保存到文件中，并在终端执行它以输出连接的摄像头列表。
或者您可以运行以下命令来列出所有连接的Orbbec摄像头：

```bash
rosrun orbbec_camera list_ob_devices.sh
```

### 启动多个摄像头

#### 多摄像头启动设置

您可以通过为每个摄像头指定不同的USB端口来启动多个摄像头。以下是一个使用ROS 1启动系统启动两个具有个别配置的摄像头的示例XML文件。

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

#### 运行启动文件

要执行多摄像头的启动配置，请使用命令：

```bash
roslaunch orbbec_camera multi_camera.launch
```

### 配置多摄像头的TF树

#### 两个摄像头的TF配置示例

使用多个摄像头时，校准它们并为每个摄像头发布一个静态TF树是必不可少的。以下ROS 1 XML启动配置基于您的校准结果设置TF树：

```XML
<launch>
    <node pkg="tf" type="static_transform_publisher" name="camera_01_tf" args="0 0 0 0 0 0 base_link camera_01_link" />
    <node pkg="tf" type="static_transform_publisher" name="camera_02_tf" args="0 0 0 0 0 0 base_link camera_02_link" />
</launch>
```

将此配置保存为Orbbec摄像头包的启动目录中的`multi_camera_tf.launch`。要运行它，使用：

```bash
roslaunch orbbec_camera multi_camera_tf.launch
```