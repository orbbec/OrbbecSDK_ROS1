## 常见问题

**电源供应不足**：

- 确保每个相机连接到单独的集线器。
- 使用有源集线器为每个相机提供充足的电源。

**高分辨率**：

- 尝试降低分辨率来解决数据流问题。

**增加usbfs_memory_mb值**：

- 将`usbfs_memory_mb`值增加到128MB（这是一个参考值，可以根据您系统的需要进行调整）
  通过运行以下命令：节点意外崩溃，它会在当前运行目录中生成崩溃日志：`~/.ros/Log/camera_crash_stack_trace_xx.log`。此外，**无论相机节点是否崩溃**，OrbbecSDK都会始终生成一个日志文件：`~/.ros/Log/OrbbecSDK.log.txt`，其中包含SDK操作的详细记录。

请将这些日志文件发送给支持团队或提交到GitHub问题以获得进一步帮助。

### 多相机无数据流

**电源供应不足**：

- 确保每个相机连接到单独的集线器。
- 使用有源集线器为每个相机提供足够的电力。

**高分辨率**：

- 尝试降低分辨率以解决数据流问题。

**增加usbfs_memory_mb值**：

- 通过运行以下命令将 `usbfs_memory_mb` 值增加到128MB（这是参考值，可根据系统需求调整）：

```
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

### 如何采集和保存日志

请按以下步骤采集日志：

1. 将 launch 参数 `log_level` 设为 `debug`。如需保存 ROS1 日志，再将节点的 `output="screen"` 改为 "log" 或 "both"。
2. 启动相机并重新复现问题，同时记下问题发生时间。
3. 将以下两类日志一起提供给技术支持：

   - **SDK 日志**：`~/.ros/Log/<camera_name>/` 中与问题发生时间对应的日志文件。
   - **ROS1 日志**：`~/.ros/log/<run_id>/` 中本次运行生成的整个目录。

SDK 日志默认以启动时间命名。多相机或无法确定具体文件时，可提供本次测试新生成的全部日志。节点崩溃时，请同时提供相机目录中的崩溃堆栈文件；仅在技术支持要求采集固件日志时，再将 `enable_heartbeat` 设为 `true`。

#### 开发者排查说明

**SDK 日志**

- SDK 日志保存在 `~/.ros/Log/<camera_name>/`。
- `log_file_name` 为空时，日志默认以节点启动时间命名，格式为 `<YYYYMMDD_HHMMSS>.log`，例如 `~/.ros/Log/camera/20260713_143025.log`。
- 指定 `log_file_name` 后，实际路径为 `~/.ros/Log/<camera_name>/<log_file_name>`。
- 多相机场景下，SDK 日志按 `camera_name` 分目录保存。
- 节点异常崩溃时，崩溃堆栈文件也会保存在对应的相机目录下。
- 如需固件日志，可在将 `log_level` 设为 `debug` 的同时，将 `enable_heartbeat` 设为 `true`。

**ROS1 日志**

- 将节点的 `output="screen"` 改为 `output="log"` 后，日志保存在 `~/.ros/log/<run_id>/`。
- `roslaunch-*.log` 记录本次 `roslaunch` 的启动流程和节点启动信息。
- `master.log` 是 ROS master 日志。
- `rosout.log` 汇总所有节点输出。多相机场景下，可根据命名空间或节点名区分相机，例如 `/ob_camera_01/camera`、`/ob_camera_02/camera`。

### 为什么有这么多启动文件？

- 不同的相机具有不同的默认分辨率和图像格式。
- 为了简化使用，每个相机都有自己的启动文件。

### 多相机连接时如何指定启动某一个相机

如果启动文件未显式指定要使用的设备，在同时连接多台相机时，驱动会默认连接到其中一个（默认设备）。

可以先通过以下命令查看设备序列号：
```bash
rosrun orbbec_camera list_devices_node
```

然后在启动时显式指定序列号，例如：
```bash
roslaunch orbbec_camera femto_bolt.launch serial_number:=CL8H741005J
```

### 多相机启动或切换流时为什么需要设置延迟？

多相机系统对带宽和设备初始化时序要求较高。如果在同一时间启动或切换多个相机流，可能会引发带宽瞬时拥塞，进而导致设备初始化失败、流启动异常或丢帧等问题。为确保系统稳定性，建议注意以下几点：

- **多相机启动阶段**

  在启动多个相机时，建议在每个相机启动之间增加适当的延迟（例如 **2s**），以避免瞬时带宽过载或底层设备初始化冲突。

- **流开关与模式切换阶段**

  在调用开关流相关服务（如 `set_streams_enable`、`toggle_depth`、`toggle_color`）时，不建议同时触发多个接口调用，应在各操作之间设置合理的时间间隔（例如 **20 ms**），以保证流状态切换的可靠性。

遵循上述时序控制原则，有助于提升多相机系统在启动和运行过程中的稳定性，减少异常和不可预期行为的发生。

### 图像未达到预设帧率

首先需要确认图像是否确实未达到预设帧率。在 ROS1 中可通过多种方式查看帧率，例如：

* `rostopic hz`
* `rqt`
* 自定义工具（如本 ROS 包提供的 `benchmark` 工具）

需要注意的是，不同工具的统计方式和 QoS 配置不同，因此得到的帧率结果可能存在差异。当发现帧率低于预期时，请优先排查是否为帧率统计工具本身导致的误差。

若确认图像帧率确实未达到预设值，可尝试以下排查步骤：

1. **降低分辨率或帧率**，判断是否由于 USB / 网络带宽受限导致帧率下降；
2. **确认相机固件版本及 ROS 包版本是否为最新**，旧版本可能存在性能或兼容性问题。

若以上方法仍无法解决问题，请联系我司 **FAE**，或在 **GitHub Issue** 中提交问题以获得进一步支持。


### 软触发模式相关问题

* **信号触发时各传感器未同时出流**
  请开启帧汇聚功能，将参数`frame_aggregate_mode`设置为`full_frame`，以保证多传感器数据在同一次触发下同步输出。

* **自动触发模式下无法达到预设帧率**
  设置 `software_trigger_period` 时，需要综合考虑实际开流帧率与曝光时间。例如，当 `color_fps` 设置为 10 FPS 时，`software_trigger_period` 不能低于以下计算值：

  ```
  software_trigger_period ≥ 1000000 / fps × N + 2 × expo
  ```

  其中：

  * `fps`：传感器帧率
  * `N`：单次触发采集的帧数量
  * `expo`：曝光时间
  * `单位`：µs

  若 `software_trigger_period` 设置过小，将导致触发频率受限，从而丢帧。
