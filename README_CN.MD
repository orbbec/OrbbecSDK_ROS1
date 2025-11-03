# OrbbecSDK ROS

> [!重要]
>
> 欢迎使用 OrbbecSDK ROS Wrapper。在开始使用本版本的 ROS 封装前，请务必查看以下的 [设备支持列表](#支持的设备)，以确认您的设备是否兼容。

OrbbecSDK ROS 封装包为 Orbbec 相机与 ROS 环境之间提供了无缝集成，支持 ROS的 Kinetic, Melodic, 和 Noetic发行版。

2024 年 10 月，我们发布了重大更新版本 —— [OrbbecSDK ROS1 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main)，该版本基于开源的[OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases)，具有更强的灵活性与可扩展性。

此更新确保了对所有符合 UVC 标准的 Orbbec USB 产品的兼容性，但不再支持使用 Orbbec 传统 OpenNI 协议的设备。如果您的设备在支持列表中，我们强烈建议您使用 v2-main 分支。

以下是 main 分支（v1.x）与 v2-main 分支（v2.x）对应的设备支持列表：

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

注意：如果您未在列表中找到您的设备，请联系我们的 FAE 或销售代表获取支持。

**定义说明**:

1. 推荐用于新设计: 我们将提供全面支持，包括新功能、漏洞修复以及性能优化。
2. 全面维护: 我们将提供漏洞修复支持。
3. 有限的维护: 我们仅提供关键性问题的修复支持。
4. 不支持: 本版本中不支持该设备。
5. 即将支持: 我们将在近期添加对该设备的支持。

## 目录

- [OrbbecSDK ROS](#orbbecsdk-ros)
  - [目录](#目录)
  - [安装说明](#安装说明)
  - [创建 ROS 工作区并构建](#创建-ros-工作区并构建)
  - [启动相机](#启动相机)
  - [选择您想要显示的topic](#选择您想要显示的topic)
  - [Network camera](#network-camera)
  - [Launch参数](#launch参数)
  - [深度工作模式开关](#深度工作模式开关)
  - [深度 NFOV 和 WFOV 模式的配置](#深度-nfov-和-wfov-模式的配置)
  - [检查相机支持哪些配置文件](#检查相机支持哪些配置文件)
  - [多相机](#多相机)
  - [所有可用的摄像机控制服务](#所有可用的摄像机控制服务)
  - [所有可用主题](#所有可用主题)
  - [构建Debian软件包](#构建debian软件包)
  - [使用 Nodelet](#使用-nodelet)
  - [支持的设备](#支持的设备)
  - [常见问题](#常见问题)
    - [意外崩溃](#意外崩溃)
    - [没有来自多个摄像机的数据流](#没有来自多个摄像机的数据流)
    - [由于 OpenCV 版本问题导致编译失败](#由于-opencv-版本问题导致编译失败)
    - [其他故障排除](#其他故障排除)
    - [为什么有这么多启动文件？](#为什么有这么多启动文件)
  - [其他有用链接](#其他有用链接)
  - [License](#license)

## 安装说明

下载ROS

* 请直接参考 ROS [wiki](http://wiki.ros.org/ROS/Installation) 获取安装说明。

安装 deb 依赖项

```bash
# Assuming you have sourced the ROS environment, same below
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
libdw-dev
```

## 创建 ROS 工作区并构建

创建 ROS 工作区（如果您没有）：

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

安装 udev 规则：

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

## 启动相机

- 第一个终端

```bash
source ./devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

- 第二个终端

```bash
source ./devel/setup.bash
rviz
```

## 选择您想要显示的topic

- 获取topics / services/ parameters列表 (第3个终端)

```bash
rostopic list
rosservice list
rosparam list
```

* 获取相机参数（必须先启动流）：

```bash
rosservice call /camera/get_camera_params "{}"
```

检查相机参数（相机信息中具体字段的含义，请参阅 [ROS 文档](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)）：

- 获取设备信息

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

检查设备信息：

```bash
rosservice call /camera/get_device_info "{}"
```

获取SDK版本（包含固件和Orbbec SDK版本）：

```bash
rosservice call /camera/get_sdk_version "{}"
```

设置/获取（自动）曝光：

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}'
rosservice call /camera/set_left_ir_auto_exposure "{data: false}"

# Setting exposure values (be careful with the data range; the following example may not be correct)
rosservice call /camera/set_left_ir_exposure "{data: 2000}"
rosservice call /camera/set_color_exposure "{data: 2000}"

# Get exposure
rosservice call /camera/get_left_ir_exposure "{}"
rosservice call /camera/get_color_exposure "{}"
```

设置自动曝光ROI：

```bash
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.

#Set color ROI
rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'

#Set depth ROI
rosservice call /camera/set_depth_ae_roi '{data_param:[0,847,0,479]}'
```

设置/获取增益：

```bash
# Get gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_left_ir_gain '{}'

# Setting the gain (be careful with the data range; the following example may not be correct)
rosservice call /camera/set_color_gain "{data: 200}"
rosservice call /camera/set_left_ir_gain "{data: 200}"
```

设置/获取（自动）白平衡：

```bash
rosservice call /camera/set_auto_white_balance "{data: false}"
rosservice call /camera/get_auto_white_balance "{data: false}"
```

打开/关闭激光：

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

保存图像：

```bash
rosservice call /camera/save_images "{}"
```

保存点云：

```bash
rosservice call /camera/save_point_cloud "{}"
```

注意：图像保存在 `~/.ros/image`下，并且仅在传感器打开时可用。

## Network camera

For Femto Mega and Femto Mega I, the network camera feature is supported.

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

## Launch参数

以下是可用的launch启动参数：

- `connection_delay`: 重新打开设备的延迟时间（以毫秒为单位）。
- `enable_point_cloud`: 深度点云开关。
- `enable_colored_point_cloud`: RGB点云开关。
- `color_width`, `color_height`, `color_fps`: 彩色流的分辨率和帧率。
- `left_ir_width`,`left_ir_height` ,`left_ir_fps` : 左红外流的分辨率和帧速率。
- `right_ir_width`,`right_ir_height` ,`right_ir_fps` : 右红外流的分辨率和帧速率。
- `depth_width`, `depth_height`, `depth_fps`: 深度流的分辨率和帧速率。
- `enable_color`: 启用彩色流。
- `enable_depth`: 启用深度流。
- `enable_left_ir`: 启动左红外流。
- `enable_right_ir`: 启动右红外流。
- `depth_registration`: 启用深度帧与颜色帧的对齐。当 `enable_colored_point_cloud` 设置为 `true` 时，此字段为必填项。
- `color_ae_roi_[left|right|top|bottom]`,`depth_ae_roi_[left|right|top|bottom]`:设置颜色和深度自动曝光 ROI。
- `log_level` : SDK 日志级别，默认值为 `info`，可选值为 `debug`, `info`, `warn`, `error`, `fatal`。保存在 `~/.ros/Log` 中。对于文件日志记录，请调整 `config/OrbbecSDKConfig_v1.0.xml` 中的 `<FileLogLevel>`。
- `ordered_pc`: 点云是否应按有序网格组织（`true`）或按无序点集组织（`false`）。
- `preset_firmware_path` : 输入参数为预设的固件路径，若输入多个路径，每个路径之间需用 `,`分隔，最多可输入3个固件路径
- `device_preset`: 默认值为 `Default`。仅支持 G330 系列。更多信息，请参阅 [G330 文档](https://www.orbbec.com/docs/g330-use-depth-presets/)。请参考下表，根据您的使用情况设置 `device_preset` 值。该值应为[表格](#预设presets) 中列出的预设名称之一。
- `enable_decimation_filter`: 此滤波器有效降低了深度场景的复杂度。滤波器在核尺寸 [2x2] 至 [8x8] 像素上运行。图像尺寸在两个维度上均按比例缩小，以保持纵横比。
- `enable_hdr_merge`: 此滤镜与深度 HDR 功能配合使用。通过合并交替曝光值的连续深度图像，我们可以克服同时获取欠照和过照物体深度值的挑战。
- `enable_sequence_id_filter`: 此过滤器与深度 HDR 函数联合使用，仅输出具有指定序列 ID 的序列。
- `enable_threshold_filter`: 该过滤器保留感兴趣的深度值并省略范围之外的深度值。
- `enable_noise_removal_filter`: 该滤波器可消除簇状斑点噪声，并生成填充程度较低的深度图。
- `enable_spatial_filter`: 此滤波器根据幅值参数的指定执行多次迭代处理，以增强深度数据的平滑度。它还能够填充深度图中的小孔。
- `enable_temporal_filter`: 此滤镜旨在通过基于先前帧操纵每个像素的值来提高深度数据的持久性。该滤镜对数据执行单次传递，在调整深度值的同时更新追踪历史记录。
- `enable_hole_filling_filter`: 此过滤器使用指定的模式填充深度图中的所有孔洞。
- `depth_precision`: 深度精度应采用 `1mm`格式。默认值为 `1mm`。
- `enable_laser` : 是否启用激光，默认值为 `true`。
- `retry_on_usb3_detection_failure`:如果摄像头连接到 USB 2.0 端口但未被检测到，系统将尝试重置摄像头最多三次。此设置旨在防止 USB 3.0 设备被错误地识别为 USB 2.0。建议在使用 USB 2.0 连接时将此参数设置为 `false`，以避免不必要的重置。
- `enable_heartbeat`: 此选项启用心跳功能。默认情况下，设置为 `false`。
  如果设置为 `true`，摄像头将发送心跳消息并生成固件日志。
- `enable_hardware_reset`: 此选项启用硬件重置功能。默认情况下，设置为 `false`。如果设置为 `true`，则摄像头将在首次连接时重新启动。
- `frame_aggregate_mode`:支持 `full_frame`, `color_frame`, `ANY` 和 `disable `

  `full_frame`:当所有启用的视频流的帧数据都有效时，才会输出该视频流主题数据。如果任何一个视频流的帧数据无效，则其他视频流的帧数据无法输出。
  `color_frame`:当彩色流的帧数据有效时，才会输出视频流主题数据，适合彩色采用H264、H265格式，保证彩色帧不丢，因为丢彩色帧会影响H264、H265的解码。
  `ANY`:当视频流的帧数据有效或者无效时，都会输出视频流，但是无效的视频流帧数据会被过滤掉，并通过主题输出。
  `disable`:禁用frame_aggregate_mode，所有类型的数据帧将独立输出。主要解决用户为不同视频流配置不同帧率时，帧输出延迟的问题

**重要提示**：*请仔细阅读[此链接](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/)中关于软件过滤设置的说明。如果您不确定，请勿修改这些设置。*

## 深度工作模式开关

- 在启动相机之前，可以配置深度工作模式（depth_work_mode），以供相应的xxx.launch文件支持。
- 深度工作模式切换支持 Gemini 2、Gemini 2 L 和 Gemini 2 XL 相机。
- xxx.launch 默认的深度工作模式配置是相机的默认配置，如需修改，可根据需要切换到相应模式。
- 具体的相机深度工作模式支持类型可以参见深度模式的注释。

```python
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

## 深度 NFOV 和 WFOV 模式的配置

对于 Femto Mega 和 Femto Bolt 设备，NFOV 和 WFOV 模式是通过在启动文件中配置深度和红外分辨率来实现的。
在启动文件中，depth_width、depth_height、ir_width、ir_height 分别表示深度分辨率和红外分辨率。
红外的帧率和分辨率必须与深度分辨率一致。不同模式与分辨率的对应关系如下：

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## 检查相机支持哪些配置文件

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

## 多相机

请参阅[多摄像头支持](./docs/multi_camera.MD)文档以了解更多信息。

## 所有可用的摄像机控制服务

服务名称直观地反映了它们的用途。务必理解，与设置或获取参数相关的服务（分别表示为 `set_`和 `get_`）只有在激活相应的 `enable_`参数后才可用。例如，使用 `enable_left_ir` 启用左红外 (IR)、使用 `enable_right_ir` 启用右红外、使用 `enable_depth` 启用深度感知或使用 `enable_color` 启用色彩处理等功能（请参阅[启动参数](#launch参数)）是相应服务正常运行的先决条件。此配置确保只有在启动文件的流参数中启用其特定流时，服务才可用。

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
- `/camera/set_left_ir_auto_exposure`
- `/camera/set_left_ir_exposure`
- `/camera/set_left_ir_gain`
- `/camera/set_laser`
- `/camera/set_white_balance`

## 所有可用主题

- `/camera/color/camera_info` : 彩色相机信息。
- `/camera/color/image_raw`: 彩色流图像。
- `/camera/depth/camera_info`: 深度相机信息。
- `/camera/depth/image_raw`: 深度流图像。
- `/camera/depth/points` :点云，仅当 `enable_point_cloud`为 `true`时可用。
- `/camera/depth_registered/points`: 彩色点云，仅当 `enable_colored_point_cloud` 为 `true` 时可用。
- `/camera/left_ir/camera_info`: 左红外相机信息。
- `/camera/left_ir/image_raw`: 左红外流图像。
- `/camera/right_ir/camera_info`: 右红外相机信息。
- `/camera/right_ir/image_raw`: 右红外流图像。。
- `/diagnostics`: 摄像机的诊断信息。目前，诊断信息仅包括摄像机的温度。

## 构建Debian软件包

开始之前，请安装所需的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

要创建 Debian 包，请执行以下命令：

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```

## 使用 Nodelet

对于需要使用nodelet的用户，请参考 `gemini2_nodelet.launch`

## 支持的设备

OrbbecSDK ROS Wrapper 支持以下设备。

| 产品列表          | **Launch File**             |
| ---------------- | --------------------------- |
| Gemini 330       | gemini_330_series.launch |
| Gemini 330L      | gemini_330_series.launch |
| Gemini 335       | gemini_330_series.launch |
| Gemini 335L      | gemini_330_series.launch |
| Gemini 336       | gemini_330_series.launch |
| Gemini 336L      | gemini_330_series.launch |
| Femto Bolt       | femto_bolt.launch        |
| Femto Mega       | femto_mega.launch        |
| Femto Mega I     | femto_mega.launch        |
| Gemini 2 XL      | gemini2XL.launch         |
| Astra 2          | astra2.launch            |
| Gemini 2 L       | gemini2L.launch          |
| Gemini 2         | gemini2.launch           |
| Astra+           | astra_adv.launch         |
| Femto            | femto.launch             |
| DaBai            | dabai.launch             |
| DaBai DCW        | dabai_dcw.launch         |
| DaBai DW         | dabai_dw.launch          |
| Astra Mini Pro   | astra.launch             |
| Gemini E         | gemini_e.launch          |
| Gemini E Lite    | gemini_e_lite.launch     |
| Gemini           | gemini.launch            |
| Astra Mini S Pro | astra.launch             |
| Astra Pro Plus   | astra_pro_plus.launch    |

> [!重要]
>
> 我们建议使用 [OrbbecSDK ROS1 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main)，以便为以下设备提供更好的灵活性和可扩展性：Astra 2、Femto Mega、Femto Bolt、Gemini 2、Gemini 2 L、Gemini 335、Gemini 335L、Gemini 336、Gemini 336L、Gemini 335Lg（仅支持 v2-main 分支）。

所有启动文件本质上都类似，主要区别在于同一系列不同型号的参数默认值。USB 标准的差异（例如 USB 2.0 和 USB 3.0）可能需要调整这些参数。如果您遇到启动失败，请仔细阅读规格手册。请特别注意启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。

## 常见问题

### 意外崩溃

如果摄像头节点意外崩溃，它将在当前运行目录中生成崩溃日志：`~/.ros/Log/camera_crash_stack_trace_xx.log`。请将此日志发送给支持团队或将其提交至 GitHub 问题以获取进一步帮助。

### 没有来自多个摄像机的数据流

**电源不足**:

- 确保每个摄像头都连接到单独的集线器。
- 使用有源集线器为每个摄像头提供足够的电力。

**高分辨率**:

- 尝试降低分辨率来解决数据流问题。

**增加 usbfs_memory_mb 值**:

- 运行以下命令，将 `usbfs_memory_mb` 值增加到 128MB（这是一个参考值，可以根据系统需求进行调整）：

```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- 要使此更改永久生效，请检查[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

### 由于 OpenCV 版本问题导致编译失败

在某些情况下，您的主机上可能有多个版本的 OpenCV，这可能会导致编译失败。您可以通过指定 OpenCV 版本来解决此问题。在 cmake 文件夹中找到 CMakeLists.txt 文件，并找到以下代码：

```bash
find_package(OpenCV REQUIRED)
```

添加 OpenCV_dir 或在其之前指定版本：

```bash
find_package(OpenCV 4.2.0 REQUIRED)
```

Or:

```bash
set(OpenCV_DIR "/path_to_your_opencv_dir")
find_package(OpenCV REQUIRED)
```

### 其他故障排除

- 如果您遇到其他问题，请将 `log_level` 参数设置为 `debug`。这将在运行目录中生成一个 SDK 日志文件：`~/.ros/Log/OrbbecSDK.log.txt`。
  请将此文件提供给支持团队以获取进一步帮助。
- 如果需要固件日志，请将 `enable_heartbeat`设置为 `true`以激活此功能。

### 为什么有这么多启动文件？

- 不同的相机有不同的默认分辨率和图像格式。
- 为了简化使用，每个相机都有自己的启动文件。

## 其他有用链接

- [OrbbecSDK_v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
- [Gemini 330 depth presets](https://www.orbbec.com/docs/g330-use-depth-presets/)

## License

版权所有 2024 Orbbec Ltd。

根据 Apache 许可证 2.0 版（"许可证"）获得许可；除非遵守许可证，否则您不得使用此项目。您可以在以下网址获取许可证副本：

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

除非适用法律要求或书面同意，否则根据本许可证分发的软件均按“原样”分发，不附带任何明示或暗示的保证或条件。请参阅许可证，了解本许可证下特定语言的权限和限制规定。

**Other names and brands may be claimed as the property of others**
