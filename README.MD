# OrbbecSDK ROS

> [!IMPORTANT]
>
> Welcome to the OrbbecSDK ROS Wrapper. Before you begin using this version of ROS wrapper, it's crucial to check the following [device support list ](#supported-devices)to verify the compatibility.

OrbbecSDK ROS Wrapper provides seamless integration of Orbbec cameras with ROS environment. It supports ROS Kinetic, Melodic, and Noetic distributions.

With a major update in October 2024, we release the [OrbbecSDK ROS1 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main) connected to the open source [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases) with enhanced flexibility and extensibility. This update ensures compatibility with all Orbbec USB products adhering to UVC standard. However, it no longer supports Orbbec's traditional OpenNI protocol devices. We strongly encourage you to use the v2-main branch if your device is supported.

Here is the device support list of main branch (v1.x) and v2-main branch (v2.x):

<table border="1" style="border-collapse: collapse; text-align: center; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/main" style="color: black; text-decoration: none;">Branch main</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main" style="color: black; text-decoration: none;">Branch v2-main</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
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
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 2</td>
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
      <td>to be supported</td>
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
      <td>Astra Mini Pro</td>
      <td>full maintenance</td>
      <td>not supported</td>
    </tr>
  </tbody>
</table>

**Note**: If you do not find your device, please contact our FAE or sales representative for help.

**Definition**:

1. recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. full maintenance: we will provide bug fix support;
3. limited maintenance: we will provide critical bug fix support;
4. not supported: we will not support specific device in this version;
5. to be supported: we will add support in the near future.

## Table of Contents

* [OrbbecSDK ROS](#orbbecsdk-ros)
  * [Table of Contents](#table-of-contents)
  * [Install Dependencies](#install-dependencies)
    * [ROS](#ros)
    * [Other Dependencies](#other-dependencies)
  * [Create ROS Workspace and Build](#create-ros-workspace-and-build)
  * [Start the Camera](#start-the-camera)
  * [Select Topics and Control the Camera](#select-topics-and-control-the-camera)
  * [Launch Parameters](#launch-parameters)
  * [Depth work mode switch:](#depth-work-mode-switch)
  * [Configuration of depth NFOV and WFOV modes](#configuration-of-depth-nfov-and-wfov-modes)
  * [Check which profiles the camera supports](#check-which-profiles-the-camera-supports)
  * [Multiple Camera Support](#multiple-camera-support)
  * [Available Services for Camera Control](#available-services-for-camera-control)
  * [Available Topics](#available-topics)
  * [Building a Debian Package](#building-a-debian-package)
  * [Launch files](#launch-files)
  * [Use Nodelet](#use-nodelet)
  * [Supported hardware products](#supported-hardware-products)
  * [Frequently Asked Questions](#frequently-asked-questions)
    * [Unexpected Crash](#unexpected-crash)
    * [No Data Stream from Multiple Cameras](#no-data-stream-from-multiple-cameras)
    * [Compilation Failure Due to OpenCV Version Issues](#compilation-failure-due-to-opencv-version-issues)
    * [Additional Troubleshooting](#additional-troubleshooting)
    * [Why Are There So Many Launch Files?](#why-are-there-so-many-launch-files)
  * [Other useful links](#other-useful-links)
  * [License](#license)

## Install Dependencies

### ROS

- Please refer directly to the ROS [wiki](http://wiki.ros.org/ROS/Installation) for installation instructions.

### Other Dependencies

- Install dependencies (be careful with your ROS distribution):

  ```bash
  # Assuming you have sourced the ROS environment, same below
  sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
  ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
  libdw-dev
  ```

## Create ROS Workspace and Build

Create a ROS workspace (if you don't have one):

```bash
mkdir -p ~/ros_ws/src
```

Get the source code:

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
```

Build the package:

```bash
cd ~/ros_ws
catkin_make
```

Install udev rules:

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

## Start the Camera

In terminal 1:

```bash
source ./devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

In terminal 2:

```bash
source ./devel/setup.bash
rviz
```

## Select Topics and Control the Camera

Check topics, services, and parameters (open a new terminal):

```bash
rostopic list
rosservice list
rosparam list
```

Get camera parameters (_MUST_ start stream first):

```bash
rosservice call /camera/get_camera_params "{}"
```

Check camera parameters (please refer to the ROS documentation for the meaning of specific fields in
the [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)):

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

Check device information:

```bash
rosservice call /camera/get_device_info "{}"
```

Get the SDK version (includes firmware and Orbbec SDK versions):

```bash
rosservice call /camera/get_sdk_version "{}"
```

Set/get (auto) exposure:

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

Set/get gain:

```bash
# Get gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_left_ir_gain '{}'

# Setting the gain (be careful with the data range; the following example may not be correct)
rosservice call /camera/set_color_gain "{data: 200}"
rosservice call /camera/set_left_ir_gain "{data: 200}"
```

Set/get (auto) white balance:

```bash
rosservice call /camera/set_auto_white_balance "{data: false}"
rosservice call /camera/get_auto_white_balance "{data: false}"
```

Turn on/off laser:

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

Save images:

```bash
rosservice call /camera/save_images "{}"
```

Save point cloud:

```bash
rosservice call /camera/save_point_cloud "{}"
```

NOTE: The images are saved under `~/.ros/image` and are only available when the sensor is on.

## Network camera

For Femto Mega and Femto Mega I, the network camera feature is supported.

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

## Launch Parameters

The following launch parameters are available:

- `connection_delay`: The delay time in milliseconds for reopening the device. Some devices require
  a longer time to initialize, and reopening the device immediately can cause firmware crashes when hot-plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `left_ir_width`, `left_ir_height`, `left_ir_fps`: The resolution and frame rate of the left IR stream.
- `right_ir_width`, `right_ir_height`, `right_ir_fps`: The resolution and frame rate of the right IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_left_ir`: Enables the left IR camera.
- `enable_right_ir`: Enables the right IR camera.
- `depth_registration`: Enables hardware alignment of the depth frame to the color frame. This field is required
  when `enable_colored_point_cloud` is set to `true`.
- `log_level` for OrbbecSDK controls console log verbosity, with levels `none`, `info`, `debug`, `warn`, `fatal`. Logs
  save in `~/.ros/Log`. For file logging, adjust `<FileLogLevel>` in `config/OrbbecSDKConfig_v1.0.xml`.
- `ordered_pc`: Whether the point cloud should be organized in an ordered grid (`true`) or as an unordered set of
  points (`false`).
- `device_preset`: The default value is `Default`. Only the G330 series is supported. For more information, refer to
  the [G330 documentation](https://www.orbbec.com/docs/g330-use-depth-presets/). Please refer to the table below to set
  the `device_preset` value based on your use case. The value should be one of the preset names
  listed [in the table](#predefined-presets).
- `enable_decimation_filter`: Only the G330 series is supported. This filter effectively reduces the depth scene complexity. The filter runs on kernel
  sizes [2x2] to [8x8] pixels. The image size is scaled down proportionally in both dimensions to preserve the aspect
  ratio.
- `enable_hdr_merge`: This filter is used jointly with depth HDR function. By merging consecutive depth images of
  alternating exposure values, we can over come challenges in acquiring depth values for under-illuminated and
  over-illuminated objects simultaneously.
- `sequence_id_filter_id`: This filter is used jointly with depth HDR function and outputs only the sequence with
  specified sequence ID.
- `enable_threshold_filter`: This filter preserves depth values of interest and omits depth values out of scope.
- `enable_noise_removal_filter`: This filter removes speckle noise in clusters and gives rise to a less-filled depth
  map.
- `enable_spatial_filter`: This filter performs multiple iterations of processing as specified by the magnitude
  parameter, to enhance the smoothness of depth data. It is also capable of filling small holes in depth maps.
- `enable_temporal_filter`:This filter is intended to improve the depth data persistency by manipulating per-pixel
  values based on previous frames. The filter performs a single pass on the data, adjusting the depth values while also
  updating the tracking history.
- `enable_hole_filling_filter`: This filter fills all holes in the depth map using the specified mode.
- `depth_precision` : The depth precision, should be like `1mm` format. The default value is `1mm`.
- `enable_laser` : Whether to enable the laser. The default value is `true`.
- `laser_on_off_mode`: Laser on/off alternate mode, 0: off, 1: on-off alternate, 2: off-on alternate. The default value
  is `0`.
- `retry_on_usb3_detection_failure`: If the camera is connected to a USB 2.0 port and is not detected, the system will
  attempt to reset the camera up to three times. This setting aims to prevent USB 3.0 devices from being incorrectly
  recognized as USB 2.0.
  It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.
  Here is a refined version of the two sentences:
- `enable_heartbeat`: This option enables the heartbeat function. By default, it is set to `false`.
  If set to `true`, the camera will send heartbeat messages and generate firmware logs.
- `enable_hardware_reset`: This option enables the hardware reset function. By default, it is set to `false`.
  If set to `true`, the camera will reboot upon its first connection.
- `frame_aggregate_mode`:Support full_frame, color_frame, ANY or disable mode
  `full_frame`:When the frame data of all enabled video stream is valid, the video stream topic data will be output.If the frame data of any video stream is invalid, the frame data of other video streams cannot be output.
  `color_frame`:When the frame data of the color stream is valid, the video stream topic data will be output,suitable for color using H264, H265 format,ensure that color frames are not dropped, because dropping color frames affects the decoding of H264 and H265.
  `ANY`:When the frame data of the video stream is valid or invalid, the video stream will be output, but the invalid video stream frame data will be filtered out and output through the topic.
  `disable`:Disable frame_aggregate_mode, all types of data frames will output independently.Primarily addresses the issue of frame output delay when users configure different frame rates for different video streams

**IMPORTANT**: *Pleframe_aggregate_modease carefully read the instructions regarding software filtering settings at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify these settings.*

## Depth work mode switch:

- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch
  file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch is the camera's default configuration. If you need to modify
  it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```XML
<!-- Depth work mode support is as follows： -->
<!-- Unbinned Dense Default -->
<!-- Unbinned Sparse Default -->
<!-- Binned Sparse Default -->
<!-- Obstacle Avoidance -->
<arg name="depth_work_mode" default=""/>
```

- View depth work modes:

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

## Configuration of depth NFOV and WFOV modes

For the **Femto Mega** and **Femto Bolt** devices, the NFOV and WFOV modes are implemented by configuring the resolution of
Depth and IR in the launch file.
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth and the resolution of
the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and
resolutions is as follows:

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## Check which profiles the camera supports

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

## Multiple Camera Support

Please refer to the [Multiple Camera Support](./docs/multi_camera.MD) document for more information.

## Available Services for Camera Control

The service names intuitively reflect their purposes. It's crucial to understand that services related to setting or
getting parameters—denoted as `set_*` and `get_*`—become available only when the respective `enable_*` parameters are
activated. For instance, enabling features such as left infrared (IR) with `enable_left_ir`, right IR
with `enable_right_ir`, depth sensing with `enable_depth`, or color processing with `enable_color` (refer
to [Launch Parameters](#Launch-Parameters)) is a prerequisite for their corresponding services to be operational. This
configuration ensures that services are accessible only when their specific stream is enabled in the
launch file's stream argument.

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

## Available Topics

- `/camera/color/camera_info`: The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth camera info.
- `/camera/depth/image_raw`: The depth stream image.
- `/camera/depth/points`: The point cloud, only available when `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when `enable_colored_point_cloud`
  is `true`.
- `/camera/left_ir/camera_info`: The left IR camera info.
- `/camera/left_ir/image_raw`: The left IR stream image.
- `/camera/right_ir/camera_info`: The right IR camera info.
- `/camera/right_ir/image_raw`: The right IR stream image.
- `/diagnostics`: The diagnostic information of the camera, Currently, the diagnostic information only includes the temperature of the camera.

## Building a Debian Package

First, ensure the necessary tools are installed:

```bash
sudo apt install debhelper fakeroot python3-bloom
```

To create the Debian package, execute these commands:

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```

## Use Nodelet

For users who need to use nodelet, please refer to `gemini2_nodelet.launch`

## Supported Devices

The following devices are supported by the OrbbecSDK ROS1 Wrapper.

| Product          | **Minimal Firmware Version** | launch file              |
| ---------------- | ---------------------------------- | ------------------------ |
| Gemini 330       | 1.2.20                             | gemini_330_series.launch |
| Gemini 330L      | 1.2.20                             | gemini_330_series.launch |
| Gemini 335       | 1.2.20                             | gemini_330_series.launch |
| Gemini 335L      | 1.2.20                             | gemini_330_series.launch |
| Gemini 336       | 1.2.20                             | gemini_330_series.launch |
| Gemini 336L      | 1.2.20                             | gemini_330_series.launch |
| Femto Bolt       | 1.0.6                              | femto_bolt.launch        |
| Femto Mega       | 1.1.7                              | femto_mega.launch        |
| Gemini 2 XL      | Obox: V1.2.5 VL:1.4.54             | gemini2XL.launch         |
| Astra 2          | 2.8.20                             | astra2.launch            |
| Gemini 2 L       | 1.4.32                             | gemini2L.launch          |
| Gemini 2         | 1.4.60                             | gemini2.launch           |
| Astra+           | 1.0.19                             | astra_adv.launch         |
| Femto            | 1.6.7                              | femto.launch             |
| DaBai            | 2436                               | dabai.launch             |
| DaBai DCW        | 2460                               | dabai_dcw.launch         |
| DaBai DW         | 2606                               | dabai_dw.launch          |
| Astra Mini Pro   | 1007                               | astra.launch             |
| Gemini E         | 3460                               | gemini_e.launch          |
| Gemini E Lite    | 3606                               | gemini_e_lite.launch     |
| Gemini           | 3018                               | gemini.launch            |
| Astra Mini S Pro | 1005                               | astra.launch             |

> [!IMPORTANT]
>
> We recommend using the [OrbbecSDK ROS1 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main) to get better flexibility and extensibility for the following devices: Astra 2, Femto Mega, Femto Bolt, Gemini 2, Gemini 2 L, Gemini 335, Gemini 335L, Gemini 336, Gemini 336L, Gemini 335Lg(only supported in v2-main branch).

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Frequently Asked Questions

### Unexpected Crash

If the camera node crashes unexpectedly, it will generate a crash log in the current running directory: `~/.ros/Log/camera_crash_stack_trace_xx.log`. Additionally, **regardless of whether the camera node crashes or not**, the OrbbecSDK will always generate a log file: `~/.ros/Log/OrbbecSDK.log.txt`, which contains detailed records of the SDK's operations.

Please send these log files to the support team or submit them to a GitHub issue for further assistance.

### No Data Stream from Multiple Cameras

**Insufficient Power Supply**:

- Ensure that each camera is connected to a separate hub.
- Use a powered hub to provide sufficient power to each camera.

**High Resolution**:

- Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system’s needs)
  by running the following command:

```bash
    echo 128 | sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb
```

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

### Compilation Failure Due to OpenCV Version Issues

In some cases, you may have multiple versions of OpenCV on your host, which can lead to compilation failures. You can resolve this by specifying the OpenCV version. Find the CMakeLists.txt file in the cmake folder and locate the following code:

```cmake
find_package(OpenCV REQUIRED)
```

Either add OpenCV_dir or specify the version before it:

```cmake
find_package(OpenCV 4.2.0 REQUIRED)
```

Or:

```cmake
set(OpenCV_DIR "/path_to_your_opencv_dir")
find_package(OpenCV REQUIRED)
```

### Additional Troubleshooting

- If you encounter other issues, set the `log_level` parameter to `debug`. This will generate an SDK log file in the running directory: `~/.ros/Log/OrbbecSDK.log.txt`.
  Please provide this file to the support team for further assistance.
- If firmware logs are required, set `enable_heartbeat` to `true` to activate this feature.

### Why Are There So Many Launch Files?

- Different cameras have varying default resolutions and image formats.
- To simplify usage, each camera has its own launch file.

## Other useful links

- [Orbbec SDK](https://orbbec.github.io/OrbbecSDK/)
- [Gemini 330 depth presets](https://www.orbbec.com/docs/g330-use-depth-presets/)

## License

Copyright 2024 Orbbec Ltd.

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

Other names and brands may be claimed as the property of others.
