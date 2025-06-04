# OrbbecSDK ROS v2

> [!IMPORTANT]
>
> Welcome to the OrbbecSDK ROS Wrapper v2. Before you begin using this version of ROS Wrapper, it's crucial to check the following [device support list](#supported-devices) to verify the compatibility.

OrbbecSDK ROS Wrapper provides seamless integration of Orbbec cameras with ROS environment. It supports ROS Kinetic, Melodic, and Noetic distributions.
With a major update in October 2024, we release the [OrbbecSDK ROS1 Wrapper v2](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main) connected to the open source [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases) with enhanced flexibility and extensibility. This update ensures compatibility with all Orbbec USB products adhering to UVC standard. However, it no longer supports Orbbec's traditional OpenNI protocol devices. We strongly encourage you to use the v2-main branch if your device is supported.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS1).

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
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td>Not supported</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>Recommended for new designs</td>
      <td>To be supported</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>Full maintenance</td>
      <td>To be supported</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>Full maintenance</td>
      <td>Recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>Limited maintenance</td>
      <td>Not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>Limited maintenance</td>
      <td>Not supported</td>
    </tr>    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>Full maintenance</td>
      <td>Not supported</td>
    </tr>
  </tbody>
</table>

   **Note**: If you do not find your device, please contact our FAE or sales representative for help.

   **Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

## Table of Contents

- [OrbbecSDK ROS v2](#orbbecsdk-ros-v2)
  - [Table of Contents](#table-of-contents)
  - [Install Dependencies](#install-dependencies)
    - [ROS](#ros)
    - [Other Dependencies](#other-dependencies)
  - [Supported Devices](#supported-devices)
  - [Create ROS Workspace and Build](#create-ros-workspace-and-build)
  - [Start the Camera](#start-the-camera)
  - [Select Topics and Control the Camera](#select-topics-and-control-the-camera)
  - [Usage](#usage)
    - [Launch parameters](#launch-parameters)
    - [All available service for camera control](#all-available-service-for-camera-control)
    - [All available topics](#all-available-topics)
    - [Network device enumeration](#network-device-enumeration)
    - [Multi-Camera](#multi-camera)
    - [Check which profiles the camera supports](#check-which-profiles-the-camera-supports)
    - [Predefined presets](#predefined-presets)
    - [Optional depth presets](#optional-depth-presets)
    - [Depth work mode switch](#depth-work-mode-switch)
    - [Configuration of depth NFOV and WFOV modes](#configuration-of-depth-nfov-and-wfov-modes)
  - [Advanced Usage](#advanced-usage)
    - [Use V4L2 backend](#use-v4l2-backend)
    - [Building a Debian Package](#building-a-debian-package)
    - [Use Nodelet](#use-nodelet)
  - [Examples](#examples)
  - [Frequently Asked Questions](#frequently-asked-questions)
    - [Unexpected Crash](#unexpected-crash)
    - [No Data Stream from Multiple Cameras](#no-data-stream-from-multiple-cameras)
    - [Compilation Failure Due to OpenCV Version Issues](#compilation-failure-due-to-opencv-version-issues)
    - [Additional Troubleshooting](#additional-troubleshooting)
    - [Why Are There So Many Launch Files?](#why-are-there-so-many-launch-files)
  - [Other useful links](#other-useful-links)
  - [License](#license)

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

## Supported Devices

Currently, the following devices are supported by the OrbbecSDK ROS Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

The following devices are supported by the OrbbecSDK ROS Wrapper.

| Product List | Minimal Firmware Version | **Launch File**    |
| :----------- | :----------------------- | :----------------------- |
| Gemini 435Le | 1.2.04                   | gemini435_le.launch      |
| Gemini 335   | 1.2.20                   | gemini_330_series.launch |
| Gemini 336   | 1.2.20                   | gemini_330_series.launch |
| Gemini 335L  | 1.2.20                   | gemini_330_series.launch |
| Gemini 336L  | 1.2.20                   | gemini_330_series.launch |
| Gemini 335Lg | 1.3.46                   | gemini_330_series.launch |
| Gemini 335Le | 1.5.31                   | gemini_330_series.launch |
| Gemini 330   | 1.2.20                   | gemini_330_series.launch |
| Gemini 330L  | 1.2.20                   | gemini_330_series.launch |
| Gemini 2     | 1.4.92                   | gemini2.launch           |
| Gemini 2 L   | 1.4.53                   | gemini2L.launch          |
| Femto Bolt   | 1.1.2                    | femto_bolt.launch        |
| Femto Mega   | 1.3.0                    | femto_mega.launch        |
| Astra 2      | 2.8.20                   | astra2.launch            |

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Create ROS Workspace and Build

Create a ROS workspace (if you don't have one):

```bash
mkdir -p ~/ros_ws/src
```

Get the source code:

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
roscd orbbec_camera
git checkout v2-main
git branch  #Check whether the branch switch is successful
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

Check topics, services, and parameters (Open a new terminal):

```bash
rostopic list
rosservice list
rosparam list
```

Get camera parameters(Must start stream first):

```bash
rosservice call /camera/get_camera_params "{}"
```

Check camera parameters (Please refer to the ROS documentation for the meaning of specific fields in
the [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)):

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

Check device information:

```bash
rosservice call /camera/get_device_info "{}"
```

Get the SDK version (Includes firmware and Orbbec SDK versions):

```bash
rosservice call /camera/get_sdk_version "{}"
```

Set/get (Auto) exposure:

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

Set auto exposure ROI:

```bash
#In data_param, the first value is the Left setting, the second value is the Right setting, the third value is the Top setting, and the fourth value is the Bottom setting.

#Set color ROI
rosservice call /camera/set_color_ae_roi '{data_param:[0,1279,0,719]}'

#Set depth ROI
rosservice call /camera/set_depth_ae_roi '{data_param:[0,847,0,479]}'
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

Set/get (Auto) white balance:

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

## Usage

### Launch Parameters

For the entire list of parameters type `rosparam list`.

**Modify parameters when launching launch**:

```bash
roslaunch orbbec_camera gemini_330_series.launch enable_color:=true color_width:=640 color_height:=480
```

* **[color|depth|left_ir|right_ir|ir]_width,[color|depth|left_ir|right_ir|ir]_height,[color|depth|left_ir|right_ir|ir]_fps,[color|depth|left_ir|right_ir|ir]_format**
  * The resolution and frame rate of the sensor stream
  * Run `rosrun orbbec_camera list_camera_profile_mode_node` to get the list of supported profiles
  * For example:`color_width:=640 color_height:=480 color_fps:=30 color_format:=MJPG depth_width:=640 depth_height:=480 depth_fps:=30 depth_format:=Y16 ir_width:=640 ir_height:=480 ir_fps:=30 ir_format:=Y8`
* **enable_color_auto_exposure_priority**
  * Enable the Color auto exposure priority
  * For example:`enable_color_auto_exposure_priority:=true`
* **enable_color_auto_exposure**
  * Enable the Color auto exposure
  * For example:`enable_color_auto_exposure:=true`
* **color_exposure**
  * Set the Color exposure
  * For example:`color_exposure:=30`
  * Note:To ensure that the `color_exposure` setting takes effect, make sure to set `enable_color_auto_exposure:=false`.
* **color_gain**
  * Set the Color gain
  * For example:`color_gain:=16`
  * Note:To ensure that the `color_gain` setting takes effect, make sure to set `enable_color_auto_exposure:=false`.
* **enable_depth_auto_exposure_priority**
  * Enable the Depth auto exposure priority
  * For example:`enable_depth_auto_exposure_priority:=true`
* **depth_brightness**
  * Set the Depth brightness
  * For example:`depth_brightness:=100`

[......](./docs/launch_parameters.md)

Explanation of parameters.Reference:[Launch parameters documentation](./docs/launch_parameters.md)

### All available service for camera control

For the entire list of service `rosservice list`.

* `/camera/get_device_info`

```bash
rosservice call /camera/get_device_info
```

* `/camera/get_sdk_version`

```bash
rosservice call /camera/get_sdk_version
```

* `/camera/reboot_device`

```bash
rosservice call /camera/reboot_device
```

[......](./docs/all_available_service_for_camera_control.md)

Explanation of service.Reference:[All available service fo camera control documentation](./docs/all_available_service_for_camera_control.md)

### All available topics

For the entire list of topic `rostopic list`.

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream info.
- `/camera/depth/image_raw`: The depth stream image.

[......](./docs/all_available_topics.md)

Explanation of topic.Reference:[All available topics documentation](./docs/all_available_topics.md)

### Network device enumeration

When using Femto Mega and Gemini 335Le, you can use the network to connect the device.

**Femto Mega:**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**Gemini 335Le:**

```bash
roslaunch orbbec_camera gemini_330_series.launch enumerate_net_device:=true
```

**Gemini 435Le:**

```bash
roslaunch orbbec_camera gemini435_le.launch enumerate_net_device:=true
```

For more information about network device enumeration.Reference:[Network device documentation](./examples/net_camera/)

### Multi-Camera

When you have multiple cameras, you can activate all of them at the same time.

For more information about Multi camera.Reference:[Multi camera documentation](./examples/multi_camera/)

### Check which profiles the camera supports

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

### Predefined presets

About the mode settings of Predefined presets.Reference:[Predefined presets documentation](./docs/predefined_presets.md)

### Optional depth presets

> You can pass the firmware path of the Optional preset into the `preset_firmware_path` launch param.Users can download the appropriate Preset from the [Depth Preset Release](https://www.orbbec.com/docs/depth-preset-release/) page provided by Orbbec based on their specific application scenarios and update it to the camera for better depth performance in corresponding scenarios.

```bash
roslaunch orbbec_camera gemini_330_series.launch preset_firmware_path:=/home/orbbec/G336X_AMR_Default_v0.0.5.bin
```

### Depth work mode switch

Gemini 2, Gemini 2 L,and Femto and Femto Bolt cameras setting depth work mode.Reference:[Depth work mode switch documentation](./docs/depth_work_mode_switch.md)

### Configuration of depth NFOV and WFOV modes

For the Femto Mega and Femto Bolt devices setting NFOV and WFOV modes.Reference:[Configuration of depth NFOV and WFOV modes documentation](./docs/configuration_of_depth_NFOV_and_WFOV_modes.md)

## Advanced Usage

### Use V4L2 backend

[Setting uvc_backend](./docs/launch_parameters.md)

Note: The V4L2 backend is not enabled by default.

* Example:

```bash
roslaunch orbbec_camera gemini_330_series.launch uvc_backend:=v4l2
```

### Building a Debian Package

If you want to build the Debian package of OrbbecSDK_ROS.Reference:[Building a Debian Package documentation](./docs/building_a_Debian_Package.md)

### Use Nodelet

For users who need to use nodelet, please refer to `gemini_330_series_nodelet.launch`

## Examples

To explore practical examples and gain insight into how to use the camera in ROS, please navigate to the [Examples](./examples/) section for more information.

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

- [OrbbecSDK v2](https://github.com/orbbec/OrbbecSDK_v2/releases)
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
