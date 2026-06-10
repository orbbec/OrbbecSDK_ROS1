# Tool Index

This page is the OrbbecSDK_ROS tool index. It lists each tool, its use case, and the full guide entry. Before running `rosrun` or `roslaunch`, source the ROS 1 and workspace environments:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source devel/setup.bash
```

## Tool Overview

| Tool | Scenario / Full Guide | Summary |
| --- | --- | --- |
| `list_devices_node` (Recommended) | <a href="device_query_tools.html">Device Query and Basic Maintenance</a> | Enumerates Orbbec devices and prints device, firmware, preset, and IP status. |
| `list_depth_work_mode_node` | <a href="device_query_tools.html">Device Query and Basic Maintenance</a> | Queries supported depth work modes for the current device. |
| `list_camera_profile_mode_node` (Recommended) | <a href="device_query_tools.html">Device Query and Basic Maintenance</a> | Queries supported stream profiles, depth work modes, and presets. |
| `list_ob_devices.sh` | <a href="device_query_tools.html">Device Query and Basic Maintenance</a> | Scans Orbbec devices from Linux USB sysfs. |
| `install_udev_rules.sh` | <a href="device_query_tools.html">Device Query and Basic Maintenance</a> | Installs USB device udev rules. |
| `firmware_update_tool` (Recommended) | <a href="firmware_update_tool.html">Device Maintenance</a> | Updates firmware or burns preset files. |
| `ip_config_tool` (Recommended) | <a href="network_config_tools.html">Network Configuration</a> | Configures DHCP, persistent IP, and Force IP for network cameras. |
| `common_benchmark_node.py` (Recommended) | <a href="benchmark_tools.html">Performance Benchmark</a> | Measures FPS, latency, CPU, memory, frame drops, and packet loss. |
| `service_benchmark_node.py` | <a href="benchmark_tools.html">Performance Benchmark</a> | Measures service call latency and success rate. |
| `service_benchmark_node` | <a href="benchmark_tools.html">Performance Benchmark</a> | C++ service call benchmark. |
| `enable_frame_drop_log` / `frame_timestamp_csv_file` (Recommended) | <a href="diagnostic_tools.html">Performance Diagnostics</a> | Built-in camera-node frame drop logging and timestamp CSV recording. |
| `monitor_fd.sh` | <a href="diagnostic_tools.html">Performance Diagnostics</a> | Monitors file descriptor count for the camera process. |
| `image_sync_example_node` (Recommended) | <a href="multi_camera_tools.html">Multi-Camera</a> | Displays and reports timestamp synchronization for multiple image streams. |
| `group_images.sh` | <a href="multi_camera_tools.html">Multi-Camera</a> | Groups saved multi-camera images by timestamp. |
| `open_stream.sh` | <a href="helper_scripts.html">Debug Helpers</a> | Runs `rostopic hz` on fixed multi-camera topics. |
| `save_image.sh` | <a href="helper_scripts.html">Debug Helpers</a> | Calls fixed multi-camera `/save_images` services. |
| `set_laser.py` | <a href="helper_scripts.html">Debug Helpers</a> | Calls `/camera/set_laser` to enable the laser. |
