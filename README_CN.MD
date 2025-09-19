# OrbbecSDK ROS v2

> [!IMPORTANT]
>
> 欢迎使用 OrbbecSDK ROS Wrapper v2。在开始使用此版本的 ROS Wrapper 之前，请务必查看以下 [设备支持列表](#supported-devices)，以确认兼容性。

[English](./README.MD) | [中文](./README_CN.MD)

OrbbecSDK ROS Wrapper 提供了 Orbbec 相机与 ROS 环境的无缝集成。支持 ROS Kinetic、Melodic 和 Noetic 发行版。

默认推荐使用 **v2-main** 分支。对于 **v2-main** 不支持的旧 OpenNI 设备，请使用 **main** 分支。仅由 main 分支支持的设备型号在下表中列出。

如果您是中国区用户，推荐使用 [gitee 仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS1)。

以下是 main 分支 (v1.x) 和 v2-main 分支 (v2.x) 的设备支持列表：

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>产品系列</th>
      <th>产品</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/main" style="color: black; text-decoration: none;">main分支</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main" style="color: black; text-decoration: none;">v2-main分支</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>不支持</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td>不支持</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>推荐用于新设计</td>
      <td>即将支持</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>全面维护</td>
      <td>即将支持</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>全面维护</td>
      <td>推荐用于新设计</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>有限的维护</td>
      <td>不支持</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>有限的维护</td>
      <td>不支持</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>全面维护</td>
      <td>不支持</td>
    </tr>
  </tbody>
</table>
**注意**: 如果没有找到您的设备，请联系 FAE 或销售人员获取帮助。

**定义**:

1. 推荐用于新项目：我们将提供完整支持，包括新功能、bug 修复和性能优化；
2. 完整维护：我们将提供 bug 修复支持；
3. 有限维护：我们将提供关键 bug 修复支持；
4. 不支持：此版本不支持该设备；
5. 即将支持：我们将在不久的将来添加支持。

## 安装依赖

### ROS

- 请直接参考 ROS [wiki](http://wiki.ros.org/ROS/Installation) 进行安装。

### 其他依赖

- 安装依赖（请注意 ROS 发行版）：

  ```
  # 假设您已经加载了 ROS 环境，以下相同
  sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
  libdw-dev
  ```

## 支持的设备

目前，以下设备受 OrbbecSDK ROS Wrapper v2-main 分支支持。后续会添加更多设备支持。如果您在下表中找不到设备，请尝试 [main](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main) 分支。

为了获得最佳性能，强烈建议升级到最新固件版本。这将确保您能够获得最新的改进和 bug 修复。

以下设备受 OrbbecSDK ROS Wrapper 支持：

| 产品列表       | 最低固件版本 | **启动文件**             |
| -------------- | ------------ | ------------------------ |
| Gemini 435Le   | 1.2.04       | gemini435_le.launch      |
| Gemini 335     | 1.2.20       | gemini_330_series.launch |
| Gemini 336     | 1.2.20       | gemini_330_series.launch |
| Gemini 335L    | 1.2.20       | gemini_330_series.launch |
| Gemini 336L    | 1.2.20       | gemini_330_series.launch |
| Gemini 335Lg   | 1.3.46       | gemini_330_series.launch |
| Gemini 335Le   | 1.5.31       | gemini_330_series.launch |
| Gemini 330     | 1.2.20       | gemini_330_series.launch |
| Gemini 330L    | 1.2.20       | gemini_330_series.launch |
| Gemini 2       | 1.4.92       | gemini2.launch           |
| Gemini 2 L     | 1.4.53       | gemini2L.launch          |
| Femto Bolt     | 1.1.2        | femto_bolt.launch        |
| Femto Mega     | 1.3.0        | femto_mega.launch        |
| Astra 2        | 2.8.20       | astra2.launch            |
| Astra Mini Pro | 2.0.01       | astra.launch             |

所有的 launch 文件基本相同，主要区别在于针对同系列不同型号设定的参数默认值不同。USB 2.0 和 USB 3.0 等标准差异，可能需要对参数进行调整。如果遇到启动失败，请仔细检查规格手册，尤其是 launch 文件中的分辨率设置及其他参数，确保兼容性和最佳性能。

## 创建 ROS 工作空间并编译

创建 ROS 工作空间（如果尚未创建）：

```
mkdir -p ~/ros_ws/src
```

获取源码：

```
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
roscd orbbec_camera
git checkout v2-main
git branch  #检查分支是否切换成功
```

编译包：

```
cd ~/ros_ws
catkin_make
```

安装 udev 规则：

```
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

## 启动相机

终端 1：

```
source ./devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

终端 2：

```
source ./devel/setup.bash
rviz
```

## 选择话题并控制相机

查看话题、服务和参数（新开终端）：

```
rostopic list
rosservice list
rosparam list
```

获取相机参数（必须先启动数据流）：

```
rosservice call /camera/get_camera_params "{}"
```

查看相机参数（具体字段含义请参考 [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)）：

```
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

查看设备信息：

```
rosservice call /camera/get_device_info "{}"
```

获取 SDK 版本（包括固件和 Orbbec SDK 版本）：

```
rosservice call /camera/get_sdk_version "{}"
```

保存图像：

```
rosservice call /camera/save_images "{}"
```

保存点云：

```
rosservice call /camera/save_point_cloud "{}"
```

**注意**: 图像会保存到 `~/.ros/image` 下，仅在传感器开启时可用。

> 更多使用细节，请参考官方 [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/source/4_application_guide/application_guide.html)

## 使用方法

### 检查相机支持的 profile

```
rosrun orbbec_camera list_camera_profile_mode_node
```

### 可选深度预设

> 您可以通过 `preset_firmware_path` 启动参数传入可选预设固件路径。用户可以根据应用场景，从 Orbbec 提供的 [Depth Preset Release](https://www.orbbec.com/docs/depth-preset-release/) 页面下载合适的预设，并更新到相机中，以在对应场景下获得更好的深度性能。

```
roslaunch orbbec_camera gemini_330_series.launch preset_firmware_path:=/home/orbbec/G336X_AMR_Default_v0.0.5.bin
```

### 使用 V4L2 后端

注意：V4L2 后端默认未启用。

- 示例：

```
roslaunch orbbec_camera gemini_330_series.launch uvc_backend:=v4l2
```

### 使用 Nodelet

需要使用 nodelet 的用户请参考 `gemini_330_series_nodelet.launch`

> 更多使用细节，请参考官方 [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/source/4_application_guide/application_guide.html)

## 示例

要了解实际示例并掌握如何在 ROS 中使用相机，请前往 [Examples](./examples/) 目录查看更多信息。

## 其他有用的链接

- [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/index.html)
- [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 深度预设](https://www.orbbec.com/docs/g330-use-depth-presets/)

## 许可证

Copyright 2024 Orbbec Ltd.

根据 Apache License 2.0（以下简称“许可”）授权；除非遵循本许可，否则您不能使用此项目。您可以在以下地址获取许可副本：

http://www.apache.org/licenses/LICENSE-2.0

除非适用法律要求或书面同意，本软件按“原样”分发，不附带任何明示或暗示的担保或条件。请参阅许可中有关特定语言的权限和限制的内容。

其他名称和品牌可能归他人所有。
