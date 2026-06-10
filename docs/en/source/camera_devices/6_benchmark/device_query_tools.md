# Device Query and Basic Maintenance Tools

This section describes device query tools that can run without the camera node, plus basic USB permission maintenance scripts.

## list_devices_node

`list_devices_node` enumerates connected Orbbec devices. It prints the device name, serial number, USB/network information, firmware version, preset list, preset version, and network device IP configuration status.

The tool does not require the camera node to be running and is useful before launching a camera node.

Since v2.8.x, the tool also prints firmware version, preset list, preset version, local network interface name for Ethernet devices, and IP source type (`NONE`, `LLA`, `DHCP`, `PERSISTENT`). If one device fails during enumeration, the tool continues with the remaining devices.

```bash
rosrun orbbec_camera list_devices_node
```

To enable SDK file logs and attempt to enable firmware logs:

```bash
rosrun orbbec_camera list_devices_node --sdk_log_level debug
```

## list_depth_work_mode_node

`list_depth_work_mode_node` queries the current depth work mode and the supported depth work mode list. It does not require the camera node to be running.

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

## list_camera_profile_mode_node

`list_camera_profile_mode_node` queries supported color, depth, IR, IMU, and LiDAR stream profiles, and prints depth work mode and preset information. It does not require the camera node to be running.

Query the default device:

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

Query by serial number:

```bash
rosrun orbbec_camera list_camera_profile_mode_node --serial_number <SN>
```

To enable SDK file logs:

```bash
rosrun orbbec_camera list_camera_profile_mode_node --sdk_log_level debug
```

## list_ob_devices.sh

`list_ob_devices.sh` scans Orbbec USB devices from Linux `/sys/bus/usb` and prints USB port, product name, and serial number. It does not depend on the SDK or camera node.

```bash
cd src/OrbbecSDK_ROS1/scripts
./list_ob_devices.sh
```

## install_udev_rules.sh

`install_udev_rules.sh` installs USB device udev rules to allow normal users to access the device. The script requires `sudo`.

```bash
cd src/OrbbecSDK_ROS1/scripts
sudo bash install_udev_rules.sh
```

The script reloads udev rules after installation.
