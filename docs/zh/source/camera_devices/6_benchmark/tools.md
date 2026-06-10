# 工具索引

本页作为 OrbbecSDK_ROS 工具索引，只列出每个工具的用途和完整说明入口。运行 `rosrun` 或 `roslaunch` 命令前，请先加载 ROS 1 和工作空间环境：

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source devel/setup.bash
```

## 工具总览

| 工具 | 使用场景 / 完整说明 | 简介 |
| --- | --- | --- |
| `list_devices_node`（推荐） | <a href="device_query_tools.html">设备查询与基础维护</a> | 枚举 Orbbec 设备并输出设备、固件、preset 和 IP 状态。 |
| `list_depth_work_mode_node` | <a href="device_query_tools.html">设备查询与基础维护</a> | 查询当前设备支持的 depth work mode。 |
| `list_camera_profile_mode_node`（推荐） | <a href="device_query_tools.html">设备查询与基础维护</a> | 查询设备支持的 stream profile、depth work mode 和 preset。 |
| `list_ob_devices.sh` | <a href="device_query_tools.html">设备查询与基础维护</a> | 从 Linux USB sysfs 扫描 Orbbec 设备。 |
| `install_udev_rules.sh` | <a href="device_query_tools.html">设备查询与基础维护</a> | 安装 USB 设备 udev 规则。 |
| `firmware_update_tool`（推荐） | <a href="firmware_update_tool.html">设备维护</a> | 升级固件或烧录 preset 文件。 |
| `ip_config_tool`（推荐） | <a href="network_config_tools.html">网络配置</a> | 配置网络相机 DHCP、persistent IP、Force IP。 |
| `common_benchmark_node.py`（推荐） | <a href="benchmark_tools.html">性能基准测试</a> | 统计帧率、延迟、CPU、内存和丢帧/丢包。 |
| `service_benchmark_node.py` | <a href="benchmark_tools.html">性能基准测试</a> | 统计 service 调用延迟和成功率。 |
| `service_benchmark_node` | <a href="benchmark_tools.html">性能基准测试</a> | C++ service 调用 benchmark。 |
| `enable_frame_drop_log` / `frame_timestamp_csv_file`（推荐） | <a href="diagnostic_tools.html">性能诊断</a> | 相机节点内置的丢帧日志和时间戳 CSV 记录功能。 |
| `monitor_fd.sh` | <a href="diagnostic_tools.html">性能诊断</a> | 监控相机进程文件描述符数量。 |
| `image_sync_example_node`（推荐） | <a href="multi_camera_tools.html">多相机</a> | 在线显示并统计多路图像时间戳同步情况。 |
| `group_images.sh` | <a href="multi_camera_tools.html">多相机</a> | 按时间戳对多相机保存图片分组。 |
| `open_stream.sh` | <a href="helper_scripts.html">调试辅助</a> | 对固定多相机 topic 执行 `rostopic hz`。 |
| `save_image.sh` | <a href="helper_scripts.html">调试辅助</a> | 调用固定多相机 `/save_images` 服务。 |
| `set_laser.py` | <a href="helper_scripts.html">调试辅助</a> | 调用 `/camera/set_laser` 服务打开激光。 |
