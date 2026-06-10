# Compressed Image

OrbbecSDK ROS1 supports publishing compressed image topics through `image_transport`. This is useful for reducing network bandwidth, viewing images remotely, and subscribing to the compressed color image directly when `color_format:=MJPG` is used to reduce host-side decoding overhead.

## Publishing Behavior

For regular image streams, compressed transport topics are provided by the installed ROS `image_transport` plugins.

For MJPG color streams, OrbbecSDK ROS1 additionally publishes the compressed color image directly on `/camera/color/image_raw/compressed`. This path avoids decoding MJPG frames in the ROS wrapper when you only need compressed color images.

## Common Compressed Image Topics

The default camera namespace is `/camera`. If you change `camera_name` when launching the camera, replace `/camera` in the following topics with the actual namespace.

| Stream | Compressed Image Topic |
| --- | --- |
| Color image | `/camera/color/image_raw/compressed` |
| Depth image | `/camera/depth/image_raw/compressedDepth` |
| Left IR image | `/camera/left_ir/image_raw/compressed` |
| Right IR image | `/camera/right_ir/image_raw/compressed` |

View compressed color image messages:

```bash
rostopic echo /camera/color/image_raw/compressed
```

View compressed depth image messages:

```bash
rostopic echo /camera/depth/image_raw/compressedDepth
```

## `color_format:=MJPG` Scenario

When the color stream uses `color_format:=MJPG`, the ROS wrapper directly publishes `/camera/color/image_raw/compressed`. Subscribing to this topic avoids extra host-side MJPG decoding and usually reduces CPU usage.

```bash
roslaunch orbbec_camera gemini_330_series.launch color_format:=MJPG
```

In this case, subscribe to:

```bash
rostopic echo /camera/color/image_raw/compressed
```

If you subscribe to `/camera/color/image_raw`, the MJPG image still needs to be decoded on the host, which increases CPU usage. For more low-CPU configuration suggestions, see [Reducing CPU Usage](../5_advanced_guide/performance/lower_cpu_usage.md).

## Troubleshooting

If you do not see compressed image topics, check the topic list first:

```bash
rostopic list | grep image_raw
```

Then make sure the related `image_transport` plugins are installed. For the MJPG color stream, also confirm that `color_format:=MJPG` is used and that a subscriber is connected to `/camera/color/image_raw/compressed`.
