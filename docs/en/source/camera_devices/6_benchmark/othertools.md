# Other Tools

## Frame Drop Logging and Frame Timestamp CSV Logging

When `enable_frame_drop_log` is enabled, the camera node prints Color and Depth frame drop statistics to the log. The log helps distinguish drops detected at the SDK receive stage from drops detected at the ROS publish stage. When `frame_timestamp_csv_file` is set, the camera node also records Color and Depth frame timestamp data to a CSV file for frame continuity, publish latency, and timestamp debugging.

```bash
roslaunch orbbec_camera gemini_330_series.launch \
enable_frame_drop_log:=true \
frame_timestamp_csv_file:=/tmp/frame_timestamp.csv
```

The CSV includes SDK frame index, hardware frame number, sensor timestamp, device/global/system timestamp, steady arrival/publish delta values, ROS publish latency, and SDK delay fields.

### Field Description

The current CSV contains two sets of homogeneous fields with the prefixes `color_` and `depth_`, for example `color_sdk_frame_index` and `depth_sdk_frame_index`. The definitions are identical for both sets; only the data source differs.

| Field suffix | Description | Unit / Notes |
| --- | --- | --- |
| `_sdk_frame_index` | SDK frame index | `frame->index()` |
| `_hardware_frame_number` | Hardware frame number | `frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)` |
| `_sensor_ts_sec` | Sensor timestamp | Seconds, usually the midpoint of the exposure time |
| `_sensor_ts_delta_us` | Delta between adjacent sensor timestamps | us |
| `_device_ts_sec` | Device clock timestamp | Seconds |
| `_device_ts_delta_us` | Delta between adjacent device timestamps | us |
| `_global_ts_sec` | Global timestamp | Seconds |
| `_global_ts_delta_us` | Delta between adjacent global timestamps | us |
| `_system_ts_sec` | SDK system timestamp | Seconds |
| `_system_ts_delta_us` | Delta between adjacent SDK system timestamps | us |
| `_arrival_steady_delta_us` | Delta between adjacent arrival steady timestamps | us |
| `_publish_steady_delta_us` | Delta between adjacent publish steady timestamps | us |
| `_arrival_to_publish_steady_us` | Time from frame arrival to publish on the ROS side (steady) | `publish_steady - arrival_steady` |
| `_sdk_delay_from_global_us` | SDK publish delay referenced to global time | `arrival_system - global_ts` |
| `_sdk_delay_from_system_us` | SDK publish delay referenced to system time | `arrival_system - sdk_system_ts` |

### Analysis Method

#### Hardware Frame Drop Detection

- Check whether `_hardware_frame_number` is continuous.
- Plot `_sensor_ts_delta_us` as a line chart or scatter plot and look for obvious jumps.
- For example, at 30 fps, the interval between adjacent frames should usually be close to 33333 us.

#### SDK / ROS Frame Drop Detection

- Check whether `_sdk_frame_index` is continuous.
- Plot `_device_ts_delta_us`, `_global_ts_delta_us`, and `_system_ts_delta_us` to see whether any of them show abnormal jumps.
- When `enable_frame_drop_log` is enabled, `stage=SDK_RECEIVE` means drops were detected at the SDK receive stage, and `stage=ROS_PUBLISH` means drops were detected at the ROS publish stage.

#### Latency Analysis

- SDK latency: inspect `_sdk_delay_from_global_us` and `_sdk_delay_from_system_us` with line charts or scatter plots to observe the delay from the underlying timestamp to frame arrival at the ROS node.
- ROS latency: inspect `_arrival_to_publish_steady_us` to measure the time from receiving a frame in the SDK callback to publishing the image on the ROS side.
- If you want a metric closer to actual processing time, prefer fields based on the steady clock.

#### Synchronization Note

This CSV is mainly intended for analyzing continuity and latency of a single Color or Depth stream. It cannot be used directly to evaluate synchronization between Color and Depth.
