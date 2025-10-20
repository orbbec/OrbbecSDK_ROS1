## 常见问题

#**电源供应不足**：

- 确保每个相机连接到单独的集线器。
- 使用有源集线器为每个相机提供充足的电源。

**高分辨率**：

- 尝试降低分辨率来解决数据流问题。

**增加usbfs_memory_mb值**：

- 将`usbfs_memory_mb`值增加到128MB（这是一个参考值，可以根据您系统的需要进行调整）
  通过运行以下命令：节点意外崩溃，它会在当前运行目录中生成崩溃日志：`~/.ros/Log/camera_crash_stack_trace_xx.log`。此外，**无论相机节点是否崩溃**，OrbbecSDK都会始终生成一个日志文件：`~/.ros/Log/OrbbecSDK.log.txt`，其中包含SDK操作的详细记录。

请将这些日志文件发送给支持团队或提交到GitHub问题以获得进一步帮助。

### 多相机无数据流

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

- 要使此更改永久生效，请查看[此链接](https://github.com/OpenKinect/libfreenect2/issues/807)。

### 由于OpenCV版本问题导致编译失败

在某些情况下，您的主机上可能有多个版本的OpenCV，这可能导致编译失败。您可以通过指定OpenCV版本来解决此问题。在cmake文件夹中找到CMakeLists.txt文件并找到以下代码：

```cmake
find_package(OpenCV REQUIRED)
```

在其之前添加OpenCV_dir或指定版本：

```cmake
find_package(OpenCV 4.2.0 REQUIRED)
```

或者：

```cmake
set(OpenCV_DIR "/path_to_your_opencv_dir")
find_package(OpenCV REQUIRED)
```

### 其他故障排除

- 如果您遇到其他问题，请将`log_level`参数设置为`debug`。这将在运行目录中生成SDK日志文件：`~/.ros/Log/OrbbecSDK.log.txt`。
  请将此文件提供给支持团队以获得进一步帮助。
- 如果需要固件日志，请将`enable_heartbeat`设置为`true`以激活此功能。

### 为什么有这么多启动文件？

- 不同的相机具有不同的默认分辨率和图像格式。
- 为了简化使用，每个相机都有自己的启动文件。