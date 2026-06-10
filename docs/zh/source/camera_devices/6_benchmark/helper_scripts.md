# 调试辅助脚本

本节介绍 ROS1 项目中保留的轻量调试脚本。这些脚本主要面向固定命名的多相机场景，使用前通常需要根据自己的 `camera_name` 或 service 名称修改脚本内容。

## open_stream.sh

`open_stream.sh` 会对固定的多相机 depth / IR topic 启动多个 `rostopic hz`，用于快速观察图像流是否持续发布。

脚本内默认 topic 为 `/ob_camera_01` 到 `/ob_camera_05` 的 depth 和 IR 图像。若实际相机名不同，请先修改脚本。

```bash
cd src/OrbbecSDK_ROS1/scripts
./open_stream.sh
```

## save_image.sh

`save_image.sh` 会调用固定多相机的 `/save_images` service，触发相机节点保存图像。脚本内默认 service 为 `/ob_camera_01/save_images` 到 `/ob_camera_05/save_images`。

```bash
cd src/OrbbecSDK_ROS1/scripts
./save_image.sh
```

如使用默认单相机名称，也可以直接调用：

```bash
rosservice call /camera/save_images "{}"
```

## set_laser.py

`set_laser.py` 会调用 `/camera/set_laser` service 并传入 `true`，用于快速打开激光。该脚本默认 service 名称固定为 `/camera/set_laser`，多相机或非默认相机名场景需要先修改脚本。

```bash
cd src/OrbbecSDK_ROS1/scripts
python3 set_laser.py
```
