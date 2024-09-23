# Orbbec ROS SDK

![稳定](http://badges.github.io/stability-badges/dist/stable.svg)(http://github.com/badges/stability-badges) ![版本](https://img.shields.io/badge/version-1.5.3-green)

**注意：此文档是通过机器翻译生成且不会及时更新，仅供参考。具体内容请以英文原文为准。**

Orbbec ROS SDK 是 OrbbecSDK 的一个封装，支持 ROS Kinetic、Melodic 和 Noetic 发行版。此 SDK 使 Orbbec 3D 摄像机能够平滑地集成到
ROS 项目中。

## 目录

<!-- TOC -->

- [Orbbec ROS SDK](#orbbec-ros-sdk)
    - [目录](#目录)
    - [安装依赖](#安装依赖)
        - [ROS](#ros)
        - [其他依赖](#其他依赖)
    - [创建ROS工作区并构建](#创建ROS工作区并构建)
    - [启动摄像头](#启动摄像头)
    - [选择主题并控制摄像头](#选择主题并控制摄像头)
        - [可用的摄像头控制服务](#可用的摄像头控制服务)
        - [可用的主题](#可用的主题)
    - [启动参数](#启动参数)
    - [深度工作模式切换：](#深度工作模式切换)
    - [配置深度NFOV和WFOV模式](#配置深度NFOV和WFOV模式)
    - [检查摄像头支持哪些配置文件](#检查摄像头支持哪些配置文件)
    - [构建Debian包](#构建Debian包)
    - [启动文件](#启动文件)
    - [使用Nodelet](#使用Nodelet)
    - [支持的硬件产品](#支持的硬件产品)
    - [常见问题](#常见问题)
        - [多摄像头无画面](#多摄像头无画面)
        - [错误："设备启动失败：usbEnumerator createUsbDevice失败！"](#错误："设备启动失败：usbEnumerator-createUsbDevice失败！")
    - [许可证](#许可证)
  <!-- TOC -->

## 安装依赖

### ROS

- 请直接参阅ROS [wiki](http://wiki.ros.org/ROS/Installation)获取安装指南。

### 其他依赖

- 安装依赖（注意您的ROS发行版）：

  ```bash
  # 假设您已经源设置了ROS环境，同下
  sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs libdw-dev
  ```

## 创建ROS工作区并构建

创建一个ROS工作区（如果您还没有）：

```bash
mkdir -p ~/ros_ws/src
```

获取源代码：

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
```

构建包：

```bash
cd ~/ros_ws
catkin_make
```

安装udev规则：

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
cd scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules
sudo udevadm control --reload && sudo udevadm trigger
```

## 启动摄像头

在终端1：

```bash
source ./devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

在终端2：

```bash
source ./devel/setup.bash
rviz
```

## 选择主题并控制摄像头

检查主题、服务和参数（开启一个新终端）：

```bash
rostopic list
rosservice list
rosparam list
```

获取摄像头参数（必须先启动流）：

```bash
rosservice call /camera/get_camera_params "{}"
```

检查摄像头参数（请参考ROS文档了解
[camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)中的具体字段含义）：

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

检查设备信息：

```bash
rosservice call /camera/get_device_info "{}"
```

获取SDK版本（包括固件和Orbbec SDK版本）：

```bash
rosservice call /camera/get_sdk_version "{}"
```

设置/获取（自动）曝光：

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}'
rosservice call /camera/set_left_ir_auto_exposure "{data: false}"

# 设置曝光值（小心数据范围；下面的示例可能不正确）
rosservice call /camera/set_left_ir_exposure "{data: 2000}"
rosservice call /camera/set_color_exposure "{data: 2000}"

# 获取曝光
rosservice call /camera/get_left_ir_exposure "{}"
rosservice call /camera/get_color_exposure "{}"
```

设置/获取增益：

```bash
# 获取增益
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_left_ir_gain '{}'

# 设置增益（小心数据范围；下面的示例可能不正确）
rosservice call /camera/set_color_gain "{data: 200}"
rosservice call /camera/set_left_ir_gain "{data: 200}"
```

设置/获取（自动）白平衡：

```bash
rosservice call /camera/set_auto_white_balance "{data: false}"
rosservice call /camera/get_auto_white_balance "{data: false}"
```

开启/关闭激光：

```bash
rosservice call /camera/set_laser '{data: true}' # 开启
rosservice call /camera/set_laser '{data: false}' # 关闭
```

保存图片：

```bash
rosservice call /camera/save_images "{}"
```

保存点云：

```bash
rosservice call /camera/save_point_cloud "{}"
```

注意：图片保存在`~/.ros/image`目录下，只有当传感器开启时才可用。

### 可用的摄像头控制服务

服务名称直观反映了其用途。至关重要的是要理解，与设置或获取参数相关的服务——标记为`set_*`和`get_*`——仅在相应的`enable_*`
参数激活时可用。例如，激活左红外（IR）功能需要`enable_left_ir`，右红外功能需要`enable_right_ir`
，深度感知功能需要`enable_depth`，或颜色处理功能需要`enable_color`（参考[启动参数](#启动参数)
）是这些对应服务运行的先决条件。这种配置确保仅当在启动文件的流参数中启用了特定流时，服务才可访问。

- `/camera/get_auto_white_balance`
- `/camera/get_camera_params`
- `/camera/get_color_auto_exposure`
- `/camera/get_color_camera_info`
- `/camera/get_color_exposure`
- `/camera/get_color_gain`
- `/camera/get_depth_auto_exposure`
- `/camera/get_depth_camera_info`
- `/camera/get_depth_exposure`
- `/camera/get_depth_gain`
- `/camera/get_device_info`
- `/camera/get_device_type`
- `/camera/get_left_ir_auto_exposure`
- `/camera/get_left_ir_camera_info`
- `/camera/get_left_ir_exposure`
- `/camera/get_left_ir_gain`
- `/camera/get_serial`
- `/camera/get_sdk_version`
- `/camera/get_white_balance`
- `/camera/reset_color_exposure`
- `/camera/reset_color_gain`
- `/camera/reset_depth_exposure`
- `/camera/reset_depth_gain`
- `/camera/reset_left_ir_exposure`
- `/camera/reset_left_ir_gain`
- `/camera/reset_white_balance`
- `/camera/save_images`
- `/camera/save_point_cloud`
- `/camera/set_auto_white_balance`
- `/camera/set_color_auto_exposure`
- `/camera/set_color_exposure`
- `/camera/set_color_gain`
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_flood`
- `/camera/set

_left_ir_auto_exposure`

- `/camera/set_left_ir_exposure`
- `/camera/set_left_ir_gain`
- `/camera/set_laser`
- `/camera/set_white_balance`

### 可用的主题

- `/camera/color/camera_info`：彩色摄像头信息。
- `/camera/color/image_raw`：彩色流图像。
- `/camera/depth/camera_info`：深度摄像头信息。
- `/camera/depth/image_raw`：深度流图像。
- `/camera/depth/points`：点云，仅在`enable_point_cloud`为`true`时可用。
- `/camera/depth_registered/points`：彩色点云，仅在`enable_colored_point_cloud`为`true`时可用。
- `/camera/left_ir/camera_info`：左IR摄像头信息。
- `/camera/left_ir/image_raw`：左IR流图像。
- `/camera/right_ir/camera_info`：右IR摄像头信息。
- `/camera/right_ir/image_raw`：右IR流图像。

## 启动参数

以下启动参数可用：

- `connection_delay`：重新打开设备的延迟时间（毫秒）。某些设备需要较长时间初始化，立即重新打开设备可能会导致固件崩溃，特别是在热插拔时。
- `enable_point_cloud`：启用点云。
- `enable_colored_point_cloud`：启用RGB点云。
- `color_width`、`color_height`、`color_fps`：彩色流的分辨率和帧率。
- `left_ir_width`、`left_ir_height`、`left_ir_fps`：左IR流的分辨率和帧率。
- `right_ir_width`、`right_ir_height`、`right_ir_fps`：右IR流的分辨率和帧率。
- `depth_width`、`depth_height`、`depth_fps`：深度流的分辨率和帧率。
- `enable_color`：启用RGB摄像头。
- `enable_depth`：启用深度摄像头。
- `enable_left_ir`：启用左IR摄像头。
- `enable_right_ir`：启用右IR摄像头。
- `depth_registration`：启用深度帧到彩色帧的硬件对齐。当`enable_colored_point_cloud`设置为`true`时，此字段是必需的。
- `log_level`用于OrbbecSDK控制台日志详细程度，级别为`none`、`info`、`debug`、`warn`、`fatal`。日志保存在`~/.ros/Log`
  中。要进行文件日志记录，请调整`config/OrbbecSDKConfig_v1.0.xml`中的`<FileLogLevel>`。
- `oredered_pc`：点云是否应组织为有序网格（`true`）或作为无序点集（`false`）。
- `device_preset`：设备预设选项为`Default`或`High Accuracy`。
- `enable_decimation_filter`：此滤波有效减少深度场景复杂性。滤波在[2x2]到[8x8]像素的核心大小上运行。图像大小在两个维度上按比例缩小，以保持宽高比。
- `enable_hdr_merge`：此滤波与深度HDR功能联用。通过合并交替曝光值的连续深度图像，我们可以同时克服获取过度照明和光照不足物体的深度值的挑战。
- `sequence_id_filter_id`：此滤波与深度HDR功能联用，并仅输出指定序列ID的序列。
- `enable_threshold_filter`：此滤波保留感兴趣的深度值，并省略超出范围的深度值。
- `enable_noise_removal_filter`：此滤波去除簇状斑点噪声，并导致深度图更少填充。
- `enable_spatial_filter`：此滤

镜根据大小参数执行多次迭代处理，以增强深度数据的平滑性。它还能填充深度图中的小孔。

- `enable_temporal_filter`：此滤波旨在通过操纵基于先前帧的每个像素值来提高深度数据的持久性。滤波对数据进行单次处理，调整深度值的同时也更新跟踪历史。
- `enable_hole_filling_filter`：此滤波使用指定的模式填充深度图中的所有孔洞。
- `depth_precision`：深度精度，应为`1mm`格式。默认值为`1mm`。
- `enable_laser`：是否启用激光。默认值为`true`。
- `laser_on_off_mode`：激光开/关交替模式，0：关闭，1：开-关交替，2：关-开交替。默认值为`0`。
- `retry_on_usb3_detection_failure`：如果摄像头连接到USB 2.0端口未被检测到，系统将尝试重置摄像头最多三次。此设置旨在防止USB
  3.0设备被错误地识别为USB 2.0。建议在使用USB 2.0连接时将此参数设置为`false`，以避免不必要的重置。

## 深度工作模式切换：

- 在启动摄像头之前，可以为相应的xxx.launch文件配置深度工作模式（depth_work_mode）。
- 深度工作模式切换由Gemini 2、Gemini 2 L和Gemini 2 XL摄像头支持。
- xxx.launch的默认深度工作模式配置是摄像头的默认配置。如果需要修改，请根据需要切换到相应模式。
- 具体的摄像头深度工作模式支持类型可以在深度模式的注释中找到。

```XML
<!-- Depth work mode support is as follows： -->
<!-- Unbinned Dense Default -->
<!-- Unbinned Sparse Default -->
<!-- Binned Sparse Default -->
<!-- Obstacle Avoidance -->
<arg name="depth_work_mode" default=""/>
```

- 查看深度工作模式：

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

## 配置深度NFOV和WFOV模式

对于Femto Mega和Femto
Bolt设备，通过在启动文件中配置深度和IR的分辨率实现NFOV和WFOV模式。在启动文件中，depth_width、depth_height、ir_width、ir_height代表深度和IR的分辨率。IR的帧速率和分辨率必须与深度一致。不同模式与分辨率之间的对应关系如下：

- NFOV未合并：640 x 576。
- NFOV合并：320 x 288。
- WFOV未合并：1024 x 1024。
- WFOV合并：512 x 512。

## 检查摄像头支持哪些配置文件

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

## 构建Debian包

首先，确保安装了必要的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

要创建Debian包，请执行以下命令：

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```

## 启动文件

| 产品系列                                | 启动文件                     |
|-------------------------------------|--------------------------|
| astra+                              | astra_adv.launch         |
| astra mini/astra mini pro/astra pro | astra.launch             |
| astra mini s pro                    | astra.launch             |
| astra2                              | astra2.launch            |
| astra stereo s                      | stereo_s_u3.launch       |
| astra pro2                          | astra_pro2.launch        |
| dabai                               | dabai.launch             |
| dabai d1                            | dabai_d1.launch          |
| dabai dcw                           | dabai_dcw.launch         |
| dabai dw                            | dabai_dw.launch          |
| dabai pro                           | dabai_pro.launch         |
| deeya                               | deeya.launch             |
| femto / femto w                     | femto.launch             |
| femto mega                          | femto_mega.launch        |
| femto bolt                          | femto_bolt.launch        |
| gemini                              | gemini.launch            |
| gemini                              | gemini.launch            |
| gemini2 / dabai DCL                 | gemini2.launch           |
| gemini2L                            | gemini2L.launch          |
| gemini e                            | gemini_e.launch          |
| gemini e lite                       | gemini_e_lite.launch     |
| dabai max                           | dabai_max.launch         |
| dabai max pro                       | dabai_max_pro.launch     |
| gemini uw                           | gemini_uw.launch         |
| dabai dcw2                          | dabai_dcw2.launch        |
| dabai dw2                           | dabai_dw2.launch         |
| gemini ew                           | gemini_ew.launch         |
| gemini ew lite                      | gemini_ew_lite.launch    |
| gemini 335 /335 L                   | gemini_330_series.launch |

**所有启动文件基本相似，主要区别在于为不同模型设置的参数默认值不同
在同一系列中。USB标准的差异，例如USB 2.0与USB 3.0，可能需要调整这些参数。如果您
遇到启动失败，请仔细查看规格手册。特别注意启动文件中的分辨率设置，以及其他参数，以确保兼容性和最佳性能。**

## 使用Nodelet

对于需要使用nodelet的用户，请参考`gemini2_nodelet.launch`

## 支持的硬件产品

## 产品支持

| **产品列表**         | **固件版本**                    |
|------------------|-----------------------------|
| Gemini 335       | 1.2.20                      |
| Gemini 335L      | 1.2.20                      |
| Femto Bolt       | 1.0.6/1.0.9                 |
| Femto Mega       | 1.1.7/1.2.7                 |
| Gemini 2 XL      | Obox: V1.2.5 VL:1.4.54      |
| Astra 2          | 2.8.20                      |
| Gemini 2 L       | 1.4.32                      |
| Gemini 2         | 1.4.60 /1.4.76              |
| Astra+           | 1.0.22/1.0.21/1.0.20/1.0.19 |
| Femto            | 1.6.7                       |
| Femto W          | 1.1.8                       |
| DaBai            | 2436                        |
| DaBai DCW        | 2460                        |
| DaBai DW         | 2606                        |
| Astra Mini Pro   | 1007                        |
| Gemini E         | 3460                        |
| Gemini E Lite    | 3606                        |
| Gemini           | 3.0.18                      |
| Astra Mini S Pro | 1.0.05                      |

## 常见问题

### 多相机无画面

- 可能是电源供应不足。为避免这种情况，不要将所有摄像头连接到同一个集线器上，而应使用带电源的集线器。

- 还可能是分辨率过高。解决这个问题，尝试降低分辨率。

### 错误："设备启动失败：usbEnumerator createUsbDevice失败！"

- 当前设备没有权限访问设备。检查当前设备的PID：

  ```bash
  lsusb | grep 2bc5
  ```

  您的输出应该看起来像这样：

  ```text
  Bus 002 Device 007: ID 2bc5:your_pid_here
  ```

  编辑`/etc/udev/rules.d/99-obsensor-libusb.rules`并添加以下行：

  ```bash
  SUBSYSTEM=="usb", ATTR{idProduct}=="

your_pid_here", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="your_device_name_here"

  ```

  将`your_pid_here`替换为您设备的PID，将`your_device_name_here`替换为您想为设备创建的名称（例如，`gemini_camera`）。

  然后重启udev服务：

  ```bash
  sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
  ```

## 许可证

版权所有 2024 Orbbec Ltd.

根据Apache许可证，版本2.0（“许可证”）授权；除非符合许可证，否则您不得使用此项目。您可以在

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

获得许可证副本。

除非适用法律要求或书面同意，根据许可证分发的软件将按“原样”基础提供，不附带任何形式的明示或暗示的保证或条件。请参阅许可证了解权限和限制的具体语言。

其他名称和品牌可能是其他所有者的财产。
