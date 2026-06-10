# Diagnostic and Latency Analysis Tools

This section describes frame continuity, timestamp, and runtime status diagnostics.

## Diagnostic Tool: Frame Drop Log and Timestamp CSV Recording

After `enable_frame_drop_log` is enabled, the camera node prints color and depth frame drop statistics in the log. This helps locate frame drops at the SDK receive stage and ROS publish stage. When `frame_timestamp_csv_file` is set, the camera node also records color and depth frame timestamp data to a CSV file for analyzing frame continuity, publish latency, and timestamp anomalies.

```bash
roslaunch orbbec_camera gemini_330_series.launch \
enable_frame_drop_log:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

The CSV contains SDK frame index, hardware frame number, sensor timestamp, device/global/system timestamp, steady arrival/publish delta, ROS publish duration, and SDK delay fields.

### Timestamp CSV Field Description

The current CSV contains two groups of fields with the same structure, prefixed by `color_` and `depth_`, for example `color_sdk_frame_index` and `depth_sdk_frame_index`. The two groups have identical definitions and differ only in data source.

| Field suffix | Description | Unit / Notes |
| --- | --- | --- |
| `_sdk_frame_index` | SDK frame index | `frame->index()` |
| `_hardware_frame_number` | Hardware frame number | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)` |
| `_sensor_ts_sec` | Sensor timestamp | Seconds, usually exposure midpoint |
| `_sensor_ts_delta_us` | Delta between adjacent sensor timestamps | us |
| `_device_ts_sec` | Device clock timestamp | Seconds |
| `_device_ts_delta_us` | Delta between adjacent device timestamps | us |
| `_global_ts_sec` | Global timestamp | Seconds |
| `_global_ts_delta_us` | Delta between adjacent global timestamps | us |
| `_system_ts_sec` | SDK system timestamp | Seconds |
| `_system_ts_delta_us` | Delta between adjacent SDK system timestamps | us |
| `_arrival_steady_delta_us` | Delta between adjacent host steady arrival times | us |
| `_publish_steady_delta_us` | Delta between adjacent host steady times before publishing | us |
| `_arrival_to_publish_steady_us` | Time from ROS receiving the frame to publishing it (steady) | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK publish delay (global reference) | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK publish delay (system reference) | `arrival_system - sdk_system_ts` |

### Timestamp CSV Analysis Methods

#### Detecting Hardware Frame Drops from CSV

- Check whether `_hardware_frame_number` is continuous.
- Plot `_sensor_ts_delta_us` as a line chart or scatter plot and check for obvious jumps.
- For example, at 30 fps, the adjacent frame interval should usually be close to 33333 us.

#### Detecting SDK / ROS Frame Drops from CSV

- Check whether `_sdk_frame_index` is continuous.
- Plot `_device_ts_delta_us`, `_global_ts_delta_us`, and `_system_ts_delta_us` as line charts or scatter plots and check for jumps.
- After `enable_frame_drop_log` is enabled, `stage=SDK_RECEIVE` in the log indicates frame drops detected at the SDK receive stage, and `stage=ROS_PUBLISH` indicates frame drops detected at the ROS publish stage.

#### Latency Analysis from CSV

- SDK latency: check `_sdk_delay_from_global_us` and `_sdk_delay_from_system_us` as line charts or scatter plots to observe delay changes from the low-level timestamp to arrival at the ROS node.
- ROS latency: check `_arrival_to_publish_steady_us` as a line chart or scatter plot to measure the time from the SDK callback receiving a frame to publishing the image on the ROS side.
- If you need a value closer to real processing time, prefer fields related to the steady clock.

#### Timestamp CSV Synchronization Note

This CSV is mainly used to analyze continuity and latency of a single color or depth stream. It cannot directly measure synchronization between color and depth.

## monitor_fd.sh

`monitor_fd.sh` prints the file descriptor count of camera-related processes once per second. It is useful for checking file descriptor leaks during long-running tests.

```bash
cd src/OrbbecSDK_ROS1/scripts
./monitor_fd.sh
```
