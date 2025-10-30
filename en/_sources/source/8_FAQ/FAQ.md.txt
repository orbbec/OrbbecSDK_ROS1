## Frequently Asked Questions

### Unexpected Crash

If the camera node crashes unexpectedly, it will generate a crash log in the current running directory: `~/.ros/Log/camera_crash_stack_trace_xx.log`. Additionally, **regardless of whether the camera node crashes or not**, the OrbbecSDK will always generate a log file: `~/.ros/Log/OrbbecSDK.log.txt`, which contains detailed records of the SDK's operations.

Please send these log files to the support team or submit them to a GitHub issue for further assistance.

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

### Additional Troubleshooting

- If you encounter other issues, set the `log_level` parameter to `debug`. This will generate an SDK log file in the running directory: `~/.ros/Log/OrbbecSDK.log.txt`.
  Please provide this file to the support team for further assistance.
- If firmware logs are required, set `enable_heartbeat` to `true` to activate this feature.

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