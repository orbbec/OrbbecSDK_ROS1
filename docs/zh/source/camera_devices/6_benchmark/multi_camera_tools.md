# 多相机辅助工具

本节介绍 ROS1 项目中提供的多相机同步验证和图片分组辅助工具。

## image_sync_example_node

`image_sync_example_node` 用于在线验证多相机图像时间戳同步情况。它会订阅多路图像 topic，显示同步图像，并输出时间戳差异信息，用于验证多相机主从同步模式下的帧对齐效果。

启动多相机同步：

```bash
roslaunch orbbec_camera multi_camera_synced.launch
```

新开终端运行同步验证节点：

```bash
rosrun orbbec_camera image_sync_example_node
```

更完整的多相机同步配置流程见 [多相机同步验证节点](../5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.md)。

## group_images.sh

`group_images.sh` 是多相机保存图片后的离线分组脚本。脚本默认读取 `/home/orbbec/image/`，按时间戳将多相机图像分组，并把结果复制到脚本目录下的 `grouped_images`。

使用前请根据环境修改脚本内的 `image_directory`、`master_camera_serial_no`、`use_device_time` 等变量。

```bash
cd src/OrbbecSDK_ROS1/scripts
python3 group_images.sh
```

> 说明：该文件扩展名为 `.sh`，但内容是 Python 脚本。
