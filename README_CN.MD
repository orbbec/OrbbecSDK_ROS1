# OrbbecSDK ROS v2

[English](./README.MD) | [中文](./README_CN.MD)

## 简介

**OrbbecSDK ROS1 Wrapper v2** 为 Orbbec 相机与 ROS 1 生态系统提供无缝集成。
 它支持 ROS1 的 **Kinetic**、**Melodic** 和 **Noetic** 发行版。

- 默认分支：**v2-main**
- 对于传统 **OpenNI** 设备：请使用 **main** 分支
- 中国用户：请使用 [Gitee 仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS)

## 分支选择

下面是 **main 分支 (v1.x)** 和 **v2-main 分支 (v2.x)** 的设备支持列表：

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
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="5" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>recommended for new designs</td>
      <td>to be supported</td>
    </tr>
    <tr>
      <td>Gemini 215</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini (S) Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
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

**迁移指南**

如果您需要将现有项目从 main (v1.x) 分支迁移到 v2-main (v2.x) 分支，请参考官方[迁移指南](https://orbbec.github.io/OrbbecSDK_ROS1/zh/source/7_developer_guide/migration_guide.html)

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

查看话题、服务和参数（终端3）：

```
rostopic list
rosservice list
rosparam list
```

输出话题：

```
rostopic echo /camera/depth/camera_info
```

调用服务：

```
rosservice call /camera/get_sdk_version "{}"
```

> 更多使用细节，请参考官方 [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/zh/source/4_application_guide/application_guide.html)

## 支持的设备

目前，以下设备受 OrbbecSDK ROS Wrapper v2-main 分支支持。后续会添加更多设备支持。如果您在下表中找不到设备，请尝试 [main](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main) 分支。

为了获得最佳性能，强烈建议升级到最新固件版本。这将确保您能够获得最新的改进和 bug 修复。

以下设备受 OrbbecSDK ROS Wrapper 支持：

| **Products List** | **Recommended FW Version**                                                             |  Launch File                                          |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Astra Mini Pro           | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro)             |                                               astra.launch   |
| Astra Mini S Pro            | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro)             |                                               astra.launch   |
| Gemini 435Le            | [1.3.6](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware)             |                                               gemini435_le.launch   |
| Gemini 330 series       | [1.6.00](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip) | gemini_330_series.launch |
| Gemini 215              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware)               |                                               gemini210.launch   |
| Gemini 210              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware)               |                                               gemini210.launch    |
| Gemini 2                | [1.4.98](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware)                |                                               gemini2.launch     |
| Gemini 2 L              | [1.5.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware)                |                                               gemini2L.launch  |
| Femto Bolt              | [1.1.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware)              |                               femto_bolt.launch |
| Femto Mega              | [1.3.1](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware)              |                                               femto_mega.launch  |
| Femto Mega I             | [2.0.4](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware)              |                                               femto_mega.launch  |
| Astra 2                 | [2.8.20](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip)    |                                               astra2.launch    |

所有的 launch 文件基本相同，主要区别在于针对同系列不同型号设定的参数默认值不同。USB 2.0 和 USB 3.0 等标准差异，可能需要对参数进行调整。如果遇到启动失败，请仔细检查规格手册，尤其是 launch 文件中的分辨率设置及其他参数，确保兼容性和最佳性能。

## 教程

请参考官方 [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/zh/source/4_application_guide/application_guide.html)

## 示例

要了解实际示例并掌握如何在 ROS 中使用相机，请前往 [Examples](./examples/) 目录查看更多信息。

## 其他有用的链接

- [OrbbecSDK ROS 文档](https://orbbec.github.io/OrbbecSDK_ROS1/zh/index.html)
- [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 深度预设](https://www.orbbec.com/docs/g330-use-depth-presets/)

## 许可证

Copyright 2024 Orbbec Ltd.

根据 Apache License 2.0（以下简称“许可”）授权；除非遵循本许可，否则您不能使用此项目。您可以在以下地址获取许可副本：

http://www.apache.org/licenses/LICENSE-2.0

除非适用法律要求或书面同意，本软件按“原样”分发，不附带任何明示或暗示的担保或条件。请参阅许可中有关特定语言的权限和限制的内容。

其他名称和品牌可能归他人所有。
