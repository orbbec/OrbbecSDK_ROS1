# 多相机同步验证节点

文件路径：[image_sync_example_node.cpp](https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/examples/multi_camera_time_sync/image_sync_example_node.cpp)

此示例节点用于 **4 台 Orbbec 相机** 的同步采集与时间戳验证。
它可用于验证多相机主从同步（Primary / Secondary Synced）模式下的帧对齐精度。

---

## 使用教程

1. **修改 `multi_camera_synced.launch`**

   - 增加 include 配置以适配 4 台相机。

   - 根据设备型号选择对应的 launch 文件。
     例如，330 系列使用 `gemini_330_series.launch`。

   - 命名规范：
     `camera_name` 按 `camera_01`、`camera_02`、`camera_03` ... 命名。

   - 将 `device_num` 设置为 **4**。

   - 通过以下命令查看并绑定正确的 USB 端口：

     `rosrun orbbec_camera list_devices_node`

   - 同步模式设置为主从模式：
     1 台相机为 **primary**，其余相机设置为 **secondary_synced**。

   - **启动顺序：**
     建议先启动 secondary 相机，最后启动 **primary** 相机，以确保同步信号建立正确。

---

2. **参数文件修改**

   修改 `config` 目录下的两个配置文件：

   - `camera_params.yaml`

   - `camera_secondary_params.yaml`

   并确保以下配置一致：

   - 全部相机均开启 **depth** 和 **color** 数据流；

   - 全部相机使用一致的 **帧率（fps）**。

---

3. **启动与验证**

   - 启动多机同步：

     `roslaunch orbbec_camera multi_camera_synced.launch`

   - 新开终端运行同步验证节点：

     `rosrun orbbec_camera image_sync_example_node`

     该节点会输出多相机图像的时间戳差异信息，用于验证同步效果。

---

4. **参考 launch 文件**

```xml
<launch>
  <arg name="device_num" default="4"/>

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
    <arg name="device_num" value="$(arg device_num)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera3_name)"/>
    <arg name="usb_port" value="$(arg camera3_usb_port)"/>
    <arg name="device_num" value="$(arg device_num)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera4_name)"/>
    <arg name="usb_port" value="$(arg camera4_usb_port)"/>
    <arg name="device_num" value="$(arg device_num)"/>
    <arg name="sync_mode" value="secondary_synced"/>
    <arg name="config_file_path" value="$(arg secondary_config)"/>
    <arg name="trigger_out_enabled" value="false"/>
  </include>

  <!-- primary camera -->
  <include file="$(find orbbec_camera)/launch/gemini_330_series.launch">
    <arg name="camera_name" value="$(arg camera1_name)"/>
    <arg name="usb_port" value="$(arg camera1_usb_port)"/>
    <arg name="device_num" value="$(arg device_num)"/>
    <arg name="sync_mode" value="primary"/>
    <arg name="config_file_path" value="$(arg primary_config)"/>
    <arg name="trigger_out_enabled" value="true"/>
  </include>
</launch>
```

---

## 必要系统配置

1. **提升 USB 缓冲区容量**

   防止多路数据并发传输时发生丢帧：

   `echo 512 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb`

2. **ROS1 运行性能优化**

   建议优化 CPU 与 USB 资源占用，降低传输时延和不稳定问题。
   详细配置请参考[此章节](../performance/lower_cpu_usage.md)。
