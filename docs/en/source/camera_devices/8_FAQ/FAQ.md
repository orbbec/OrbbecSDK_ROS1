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

Follow these steps to collect logs:

1. Set the launch parameter `log_level` to `debug`. To save ROS 1 logs, also change the node's `output="screen"` to "log" or "both".
2. Start the camera, reproduce the issue, and note when the issue occurred.
3. Send both of the following to technical support:

   - **SDK logs**: the log files under `~/.ros/Log/<camera_name>/` that match the time of the issue.
   - **ROS 1 logs**: the entire `~/.ros/log/<run_id>/` directory created by this run.

SDK log files are named with the startup time by default. For multiple cameras, or if you are unsure which files are relevant, send all logs newly generated during the test. If the node crashed, also send the crash stack trace from the camera's log directory. Set `enable_heartbeat` to `true` only when technical support asks for firmware logs.

#### Details for Developers

**SDK logs**

- SDK logs are stored under `~/.ros/Log/<camera_name>/`.
- When `log_file_name` is empty, the log file is named after the node startup time in the format `<YYYYMMDD_HHMMSS>.log`, for example `~/.ros/Log/camera/20260713_143025.log`.
- When `log_file_name` is specified, the resulting path is `~/.ros/Log/<camera_name>/<log_file_name>`.
- In multi-camera setups, SDK logs are stored in separate directories by `camera_name`.
- If the node crashes unexpectedly, the crash stack trace is also saved in the corresponding camera directory.
- To collect firmware logs, set `log_level` to `debug` and `enable_heartbeat` to `true`.

**ROS 1 logs**

- After changing the node's `output="screen"` to `output="log"`, logs are stored under `~/.ros/log/<run_id>/`.
- `roslaunch-*.log` records the `roslaunch` startup flow and node launch information.
- `master.log` is the ROS master log.
- `rosout.log` aggregates output from all nodes. In multi-camera setups, identify each camera by its namespace or node name, for example `/ob_camera_01/camera` and `/ob_camera_02/camera`.

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
