# OrbbecSDK ROS1 Wrapper v2

[English](./README.MD) | [中文](./README_CN.MD)

## Introduction

The **OrbbecSDK ROS1 Wrapper v2** provides seamless integration of Orbbec cameras with the ROS 1 ecosystem.
 It supports ROS1 **Kinetic**, **Melodic**, and **Noetic** distributions.

- Default branch: **v2-main**
- For legacy **OpenNI** devices: use the **main** branch
- For users in China: use the [Gitee repository](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS)

## branch selection

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

   **Note**: If you do not find your device, please contact our FAE or sales representative for help.

   **Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

**Migration Guide**

If you need to migrate existing projects from the main (v1.x) branch to the v2-main (v2.x) branch, please refer to the official [migration guide](https://orbbec.github.io/OrbbecSDK_ROS1/en/source/7_developer_guide/migration_guide.html)

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
cd OrbbecSDK_ROS1
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

Select the topic you want to display

- List topics / services/ parameters ( On terminal 3)

```bash
rostopic list
rosservice list
rosparam list
```

Echo a topic

```bash
rostopic echo /camera/depth/camera_info
```

Call a service

```bash
rosservice call /camera/get_sdk_version "{}"
```

> For more usage details, please refer to the official [OrbbecSDK ROS documentation](https://orbbec.github.io/OrbbecSDK_ROS1/en/source/4_application_guide/application_guide.html)

## Supported Devices

Currently, the following devices are supported by the OrbbecSDK ROS Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS1/tree/main)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

The following devices are supported by the OrbbecSDK ROS Wrapper.

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

All launch files are essentially similar, with the primary difference being the default values of the parameters set
for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.

## Tutorial

Please refer to the official [OrbbecSDK ROS documentation](https://orbbec.github.io/OrbbecSDK_ROS1/en/source/4_application_guide/application_guide.html).

## Examples

To explore practical examples and gain insight into how to use the camera in ROS, please navigate to the [Examples](./examples/) section for more information.

## Other useful links

- [OrbbecSDK ROS documentation](https://orbbec.github.io/OrbbecSDK_ROS1/en/index.html)
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
