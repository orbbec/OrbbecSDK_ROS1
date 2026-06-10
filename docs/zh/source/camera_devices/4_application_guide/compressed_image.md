# 压缩图像

OrbbecSDK ROS1 支持通过 `image_transport` 发布压缩图像话题。常用场景包括降低网络传输带宽、远程查看图像，以及在 `color_format:=MJPG` 时直接订阅压缩彩色图像以降低主机端解码开销。

## 发布行为

普通图像流的压缩传输话题由系统中已安装的 ROS `image_transport` 插件提供。

对于 MJPG 彩色流，OrbbecSDK ROS1 会额外直接发布 `/camera/color/image_raw/compressed` 压缩彩色图像。如果只需要压缩彩色图像，订阅该话题可以避免 ROS wrapper 对 MJPG 帧进行解码。

## 常用压缩图像话题

默认相机命名空间为 `/camera`。如果启动时修改了 `camera_name`，请将下面话题中的 `/camera` 替换为实际命名空间。

| 数据流 | 压缩图像话题 |
| --- | --- |
| 彩色图像 | `/camera/color/image_raw/compressed` |
| 深度图像 | `/camera/depth/image_raw/compressedDepth` |
| 左红外图像 | `/camera/left_ir/image_raw/compressed` |
| 右红外图像 | `/camera/right_ir/image_raw/compressed` |

查看压缩彩色图像消息：

```bash
rostopic echo /camera/color/image_raw/compressed
```

查看压缩深度图像消息：

```bash
rostopic echo /camera/depth/image_raw/compressedDepth
```

## `color_format:=MJPG` 场景

当彩色流使用 `color_format:=MJPG` 时，ROS wrapper 会直接发布 `/camera/color/image_raw/compressed`，订阅该话题可以避免在主机侧额外解码 MJPG 图像，通常能降低 CPU 占用。

```bash
roslaunch orbbec_camera gemini_330_series.launch color_format:=MJPG
```

此时建议订阅：

```bash
rostopic echo /camera/color/image_raw/compressed
```

如果订阅 `/camera/color/image_raw`，MJPG 图像仍需要在主机侧解码，CPU 占用会更高。更多低 CPU 配置建议请参考 [降低 CPU 使用率](../5_advanced_guide/performance/lower_cpu_usage.md)。

## 排查方法

如果没有看到压缩图像话题，请先检查话题列表：

```bash
rostopic list | grep image_raw
```

然后确认系统已安装 `image_transport` 相关插件。对于 MJPG 彩色流，还需要确认启动时使用了 `color_format:=MJPG`，并且订阅端已连接到 `/camera/color/image_raw/compressed`。
