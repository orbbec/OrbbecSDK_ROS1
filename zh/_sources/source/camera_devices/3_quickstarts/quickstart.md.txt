## ROS包快速开始

### 介绍

本节提供使用Orbbec ROS 1包装器的快速入门指南。
您将学习如何：

- 启动相机节点。
- 在**RViz**中可视化深度/彩色流。
- 使用**ROS 1 CLI工具**与话题和服务进行交互。

------

### 构建您的第一个相机应用程序

#### 步骤1：配置ROS 1和工作空间环境

确保ROS 1和您的工作空间环境已正确配置：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros_ws/devel/setup.bash
```

#### 步骤2：启动相机节点

- 在终端1中

```bash
roscd orbbec_camera
rosrun orbbec_camera list_devices_node   # 检查相机是否已连接
roslaunch orbbec_camera gemini_330_series.launch   # 或其他启动文件
```

如果您连接了多个相机，可以指定**序列号**：

```bash
roslaunch orbbec_camera gemini_330_series.launch serial_number:=<您的相机序列号>
```

#### 步骤3：在RViz中可视化

启动RViz并加载默认配置：

- 在终端2中

```bash
rviz
```

- 添加一个**Image**显示器，将话题设置为`/camera/color/image_raw`。
- 为`/camera/depth/image_raw`添加另一个**Image**显示器。
- 可选择性地为`/camera/depth/points`添加一个**PointCloud2**显示器。

现在您应该能在RViz中看到彩色流、深度流和3D点云。

------

### 示例功能

节点运行后，尝试一些ROS 1 CLI命令：

#### 列出可用的话题/服务/参数

```bash
rostopic list
rosservice list
rosparam list
```

#### 回显话题

查看深度相机数据：

```bash
rostopic echo /camera/depth/camera_info
```

#### 调用服务

例如，获取设备信息：

```bash
rosservice call /camera/get_device_info "{}"
```

#### 使用rosbag录制

```bash
rosbag record /camera/color/image_raw /camera/depth/image_raw
```
