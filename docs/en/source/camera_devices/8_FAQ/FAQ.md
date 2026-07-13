## Frequently Asked Questions

**Insufficient Power Supply**:

- Ensure that each camera is connected to a separate hub.
- Use a powered hub to provide sufficient power to each camera.

**High Resolution**:

- Try lowering the resolution to resolve data stream issues.

**Increase usbfs_memory_mb Value**:

- Increase the `usbfs_memory_mb` value to 128MB (this is a reference value and can be adjusted based on your system's needs).
  By running the following command: if the node crashes unexpectedly, it will generate a crash log in the current run directory: `~/.ros/Log/camera_crash_stack_trace_xx.log`. In addition, **regardless of whether the camera node crashes**, OrbbecSDK will always generate a log file: `~/.ros/Log/OrbbecSDK.log.txt`, which contains detailed records of SDK operations.

Please send these log files to the support team or submit them in a GitHub issue for further assistance.

### No Data Stream from Multiple Cameras

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

- To make this change permanent, check [this link](https://github.com/OpenKinect/libfreenect2/issues/807).

### Compilation Failure Due to OpenCV Version Issues

In some cases, you may have multiple versions of OpenCV on your host, which can lead to compilation failures. You can resolve this by specifying the OpenCV version. Find the CMakeLists.txt file in the cmake folder and locate the following code:

```cmake
find_package(OpenCV REQUIRED)
```

Either add OpenCV_dir or specify the version before it:

```cmake
find_package(OpenCV 4.2.0 REQUIRED)
```

Or:

```cmake
set(OpenCV_DIR "/path_to_your_opencv_dir")
find_package(OpenCV REQUIRED)
```

### How to Collect and Save Logs

#### SDK Logs

- Set the launch parameter `log_level` to `debug`. After running, the SDK will generate log files under `~/.ros/Log/<camera_name>/`.
- When `log_file_name` is empty, the SDK log file is named after the node startup time in the format `<YYYYMMDD_HHMMSS>.log`, for example `~/.ros/Log/camera/20260713_143025.log`. You can also use `log_file_name` to specify the file name; the resulting path is `~/.ros/Log/<camera_name>/<log_file_name>`.
- If the node crashes unexpectedly, the crash stack trace file is also saved under the corresponding camera directory.
- If firmware logs are required, set `log_level` to `debug` and set `enable_heartbeat` to `true`.
- In multi-camera setups, SDK logs are stored in separate directories by `camera_name`. Each launch creates a new log file named with that launch's startup time.
- When submitting an issue, provide the SDK log file that corresponds to the time when the issue occurred.

#### ROS Logs

- If you do not want too much terminal output, change `output="screen"` to `output="log"` in the launch file, then check the corresponding ROS1 logs under `~/.ros/log/<run_id>/`.
- `roslaunch-*.log` records the full `roslaunch` startup flow and node launch information, and usually includes all cameras.
- `master.log` is the ROS master log.
- `rosout.log` aggregates output from all nodes. In multi-camera setups, logs from multiple cameras are mixed in the same file, so distinguish them by namespace or node name, for example `/ob_camera_01/camera` and `/ob_camera_02/camera`.
- When submitting an issue, it is recommended to provide both the SDK logs under `~/.ros/Log/` and the ROS1 logs under `~/.ros/log/<run_id>/` for the same time period.

### Why Are There So Many Launch Files?

- Different cameras have varying default resolutions and image formats.
- To simplify usage, each camera has its own launch file.

### How to Launch a Specific Camera When Multiple Cameras Are Connected

While the launch file did not explicitly specify which device to use. In that case, the driver will connect to the default device.

You can check the serial number of your device by running:
```bash
rosrun orbbec_camera list_devices_node
```

Then launch with the serial number explicitly set, for example:
```bash
roslaunch orbbec_camera femto_bolt.launch serial_number:=CL8H741005J
```

### Why Is It Necessary to Add Delays When Starting Multiple Cameras or Switching Streams?

Multi-camera systems place high demands on USB bandwidth and device initialization timing. If multiple camera streams are started or switched simultaneously, it may cause temporary bandwidth congestion, leading to device initialization failures, stream startup errors, or frame drops. To ensure system stability, the following practices are recommended:

- **Multi-camera startup phase**

  When starting multiple cameras, it is recommended to introduce an appropriate delay between each camera startup (e.g., **2 seconds**) to avoid instantaneous bandwidth overload or low-level device initialization conflicts.

- **Stream enable/disable and mode switching phase**

  When invoking stream control services (such as `set_streams_enable`, `toggle_depth`, and `toggle_color`), avoid triggering multiple service calls at the same time. Instead, introduce a reasonable interval between operations (e.g., **20 ms**) to ensure reliable stream state transitions.

Following these timing control guidelines can significantly improve the stability of multi-camera systems during startup and runtime, reducing errors and unexpected behavior.

### The image does not reach the preset frame rate

First you need to confirm whether the image does not reach the preset frame rate. There are several ways to view framerate in ROS1, such as:

* `rostopic hz`
* `rqt`
* Custom tools (such as the `benchmark` tool provided by this ROS package)

It should be noted that different tools have different statistical methods and QoS configurations, so the frame rate results obtained may be different. When you find that the frame rate is lower than expected, please prioritize whether the error is caused by the frame rate statistics tool itself.

If you confirm that the image frame rate does not reach the preset value, you can try the following troubleshooting steps:

1. **Reduce the resolution or frame rate** to determine whether the frame rate is reduced due to USB/network bandwidth limitations;
2. **Confirm whether the camera firmware version and ROS package version are the latest**. Older versions may have performance or compatibility issues.

If the above methods still cannot solve the problem, please contact our company **FAE**, or submit an issue in **GitHub Issue** for further support.


### Issues related to soft trigger mode

* **Each sensor does not flow out at the same time when the signal is triggered**
  Please enable the frame aggregation function and set the parameter `frame_aggregate_mode` to `full_frame` to ensure that multiple sensor data are output synchronously under the same trigger.

* **The preset frame rate cannot be reached in auto trigger mode**
  When setting `software_trigger_period`, you need to consider the actual open stream frame rate and exposure time. For example, when `color_fps` is set to 10 FPS, `software_trigger_period` cannot be lower than the following calculated value:

  ```
  software_trigger_period ≥ 1000000 / fps × N + 2 × expo
  ```

  Among them:

  * `fps`: sensor frame rate
  * `N`: The number of frames collected in a single trigger
  * `expo`: exposure time
  * `Unit`: µs

  If `software_trigger_period` is set too small, the trigger frequency will be limited, resulting in frame loss.
