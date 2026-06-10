# 设备查询与基础维护工具

本节介绍不依赖相机节点即可使用的设备查询工具，以及基础 USB 权限维护脚本。

## list_devices_node

`list_devices_node` 用于枚举当前连接的 Orbbec 设备。它会输出设备名称、序列号、USB/网络信息、固件版本、preset 列表、preset 版本以及网络设备 IP 配置状态。

该工具不需要先启动相机节点，适合在启动 launch 前确认设备连接状态。

v2.8.x 之后，该工具会额外输出固件版本、preset 列表、preset 版本、网口设备本地网卡名和 IP 来源类型（`NONE`、`LLA`、`DHCP`、`PERSISTENT`）。当某个设备枚举失败时，工具会继续枚举其他设备。

```bash
rosrun orbbec_camera list_devices_node
```

如需开启 SDK 文件日志并尝试开启固件日志：

```bash
rosrun orbbec_camera list_devices_node --sdk_log_level debug
```

## list_depth_work_mode_node

`list_depth_work_mode_node` 用于查询当前设备的 depth work mode 和支持的 depth work mode 列表。它不需要先启动相机节点。

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

## list_camera_profile_mode_node

`list_camera_profile_mode_node` 用于查询设备支持的彩色、深度、IR、IMU、LiDAR stream profile，并输出 depth work mode 和 preset 信息。它不需要先启动相机节点。

查询默认设备：

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

指定序列号查询：

```bash
rosrun orbbec_camera list_camera_profile_mode_node --serial_number <SN>
```

如需开启 SDK 文件日志：

```bash
rosrun orbbec_camera list_camera_profile_mode_node --sdk_log_level debug
```

## list_ob_devices.sh

`list_ob_devices.sh` 是源码脚本，会从 Linux `/sys/bus/usb` 扫描 Orbbec USB 设备，输出 USB port、产品名和序列号。它不依赖 SDK，也不需要相机节点。

```bash
cd src/OrbbecSDK_ROS1/scripts
./list_ob_devices.sh
```

## install_udev_rules.sh

`install_udev_rules.sh` 用于安装 USB 设备 udev 规则，解决普通用户访问设备的权限问题。该脚本需要 `sudo`。

```bash
cd src/OrbbecSDK_ROS1/scripts
sudo bash install_udev_rules.sh
```

安装后脚本会重新加载 udev 规则。
