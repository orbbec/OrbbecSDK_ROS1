# Gemini 330 系列 SDK JSON 配置导入与导出

本文档说明如何在 ROS1 中为 Gemini 330 系列相机导入和导出 SDK JSON 配置文件。

SDK JSON 可用于恢复或迁移相机配置。JSON 文件可以来自 SDK、OrbbecViewer 或 ROS 导出的配置。ROS 节点负责传入文件路径、触发导入导出，并在初始化后回读相机最终配置。

## 适用范围

本文仅描述 Gemini 330 系列的 SDK JSON 导入导出。建议优先使用专用启动文件：

```bash
gemini_330_series_sdk_json.launch
```

该启动文件面向 SDK JSON 使用场景，只保留 ROS 运行和设备管理所需参数，尽量避免 launch 默认相机参数覆盖 JSON 中的配置。

`gemini_330_series.launch`、`gemini_330_series_low_cpu.launch`、`gemini_330_series_nodelet.launch` 等完整启动文件也提供 `load_config_json_file_path` 参数，但这些启动文件包含较多相机参数。参数冲突时，launch 或 YAML 中已经传入节点的参数会优先于 JSON 中的对应配置。

## 参数优先级

当 SDK JSON 与 ROS launch / YAML 参数同时配置同一项时，可以按下面的规则理解最终生效值：

```text
已传入节点的 launch / YAML 参数 > SDK JSON 中的同名配置
```

这里的“已传入节点”包括启动文件中通过 `<param>` 写入的默认参数。因此，使用 `gemini_330_series.launch` 等完整启动文件时，即使用户没有在命令行显式设置某些相机参数，启动文件默认值也可能已经传入节点，并覆盖 JSON 中的对应配置。

SDK JSON 中的 `application_config`、`parameters.sensor_depth`、`parameters.sensor_color` 等模块都包含与 ROS 参数对应的配置。无论字段属于哪个模块，只要它和某个 launch / YAML 参数控制同一项配置，就需要按上述优先级判断最终生效值。

例如，JSON 中配置了 `color_brightness=10`，但 launch 或 YAML 中传入了 `color_brightness=0`，最终相机会使用 `0`。最终生效值以启动日志中的 `Config final readback ...` 为准。

因此：

* 如果希望尽量按 JSON 恢复配置，推荐使用 `gemini_330_series_sdk_json.launch`。
* 如果必须使用完整启动文件，请将会与 JSON 冲突的参数置空、不设置，或改为期望值。

可用 depth preset 名称请参考[设备预设](predefined_presets.md)。如果需要烧录或升级 preset 文件，请参考 [firmware_update_tool 设备维护工具](../../6_benchmark/firmware_update_tool.md)。

## 导入 SDK JSON

使用 `load_config_json_file_path` 指定需要导入的 SDK JSON 文件路径：

```bash
roslaunch orbbec_camera gemini_330_series_sdk_json.launch \
  load_config_json_file_path:=/path/to/camera_config.json
```

文件路径支持绝对路径、相对路径和 `~`。相对路径会按当前工作目录解析为绝对路径。

导入成功时，日志中会看到：

```text
Config JSON loaded file=/path/to/camera_config.json
```

如果文件不存在，日志中会看到：

```text
Config JSON load skip file=/path/to/camera_config.json reason=file_not_found
```

如果 SDK 加载 JSON 失败，日志中会看到：

```text
Config JSON load failed file=/path/to/camera_config.json error="..."
```

## 确认最终生效配置

JSON 导入后，节点会在初始化过程中回读相机最终配置，并输出类似日志：

```text
Config final readback [depth] device_preset=Default
Config final readback [color] color_brightness=0
Config final readback [filter.depth.DecimationFilter] scale=2
```

这些日志表示相机最终生效的配置。若导入的 JSON 与 launch / YAML 参数冲突，回读日志会反映覆盖后的最终值。

对于深度滤波器，也可以通过状态话题确认当前使能状态和参数：

```bash
rostopic echo /camera/depth_filters/status
```

## 导出 SDK JSON

相机节点运行后，可以通过 `/camera/export_config_json` 服务导出当前配置：

```bash
rosservice call /camera/export_config_json "data: '/tmp/orbbec_camera_config.json'"
```

调用成功后，指定路径会生成 SDK JSON 配置文件。该文件可在后续启动时通过 `load_config_json_file_path` 导入。

导出路径支持绝对路径、相对路径和 `~`。如果父目录不存在，节点会自动创建。

导出成功时，服务返回和日志中会包含：

```text
Exported config json file path: /tmp/orbbec_camera_config.json
```

如果路径为空或导出失败，服务会返回失败信息，并在日志中输出错误原因。

## 精简字段映射

下表列出常见 SDK JSON 字段与 ROS 参数的关系。它用于理解冲突和覆盖关系，不表示所有字段都需要通过 launch 配置。

| SDK JSON 字段 | 相关 ROS 参数 | 说明 |
| --- | --- | --- |
| `application_config.sensors.Color.profile.*` | `enable_color`、`color_width`、`color_height`、`color_fps`、`color_format`、`enable_color_undistortion` | 彩色流开关、分辨率、帧率、格式和去畸变。 |
| `application_config.sensors.Depth.profile.*` | `enable_depth`、`depth_width`、`depth_height`、`depth_fps`、`depth_format`、`enable_depth_undistortion` | 深度流开关、分辨率、帧率、格式和去畸变。 |
| `application_config.sensors.LeftIR.profile.*` | `enable_left_ir`、`left_ir_width`、`left_ir_height`、`left_ir_fps`、`left_ir_format`、`enable_left_ir_undistortion` | 左 IR 流配置。 |
| `application_config.sensors.RightIR.profile.*` | `enable_right_ir`、`right_ir_width`、`right_ir_height`、`right_ir_fps`、`right_ir_format`、`enable_right_ir_undistortion` | 右 IR 流配置。 |
| `application_config.sensors.Accel.profile.*` | `enable_accel`、`accel_rate`、`accel_range` | 加速度计开关、采样率和量程。 |
| `application_config.sensors.Gyro.profile.*` | `enable_gyro`、`gyro_rate`、`gyro_range` | 陀螺仪开关、采样率和量程。 |
| `application_config.point_cloud.*` | `enable_point_cloud`、`enable_colored_point_cloud`、`point_cloud_decimation_filter_factor`、`depth_registration`、`align_mode`、`align_target_stream`、`enable_frame_sync`、`frame_aggregate_mode` | 点云、彩色点云、对齐、帧同步和帧聚合配置。 |
| `application_config.hdr_merge.*` | `enable_hdr_merge` | HDR merge 配置。 |
| `application_config.device_decimation.*` | `preset_resolution_config` | 设备级下采样配置。 |
| `parameters.sensor_depth.depth_preset` | `device_preset` | 深度 preset。使用完整启动文件导入 JSON 时，避免用 `device_preset` 覆盖 JSON。 |
| `parameters.sensor_depth` 中的曝光、增益、AE ROI、深度单位、激光、视差、图像方向等字段 | `depth_exposure`、`depth_gain`、`enable_ir_auto_exposure`、`ir_ae_max_exposure`、`depth_ae_roi_*`、`depth_precision`、`enable_laser`、`laser_energy_level`、`disparity_to_depth_mode`、`disparity_range_mode`、`disparity_search_offset`、`depth_rotation`、`depth_flip`、`depth_mirror` 等 | 深度相关设备配置。 |
| `parameters.sensor_depth.frame_interleave.*` | `interleave_frame_enable`、`interleave_ae_mode`、`interleave_skip_index`、`hdr_index*_*`、`laser_index*_*` | HDR / laser interleave 配置。 |
| `parameters.sensor_depth.post_processing_filter.*` | `enable_*_filter` 和对应滤波器参数 | 深度后处理滤波配置。`FalsePositiveFilter` 的详细参数通过 JSON 或 `/camera/set_filter` 的 `filter_config` 配置。 |
| `parameters.sensor_color` 中的曝光、白平衡、亮度、锐度、防频闪、AE ROI、图像方向等字段 | `enable_color_auto_exposure`、`color_exposure`、`color_gain`、`enable_color_auto_white_balance`、`color_white_balance`、`color_brightness`、`color_sharpness`、`color_powerline_freq`、`color_ae_roi_*`、`color_rotation`、`color_flip`、`color_mirror` 等 | 彩色相关设备配置。 |
| `parameters.sensor_color.post_processing_filter.DecimationFilter` | `enable_color_decimation_filter`、`color_decimation_filter_scale` | 彩色降采样滤波配置。 |
| `parameters.sensor_left_ir` / `parameters.sensor_right_ir` | `left_ir_rotation`、`right_ir_rotation`、`enable_left_ir_sequence_id_filter`、`enable_right_ir_sequence_id_filter`、`left_ir_sequence_id_filter_id`、`right_ir_sequence_id_filter_id` 等 | 左右 IR 图像方向和 sequence id filter 配置。 |

## 常见问题

### JSON 导入后没有生效

请检查：

* `load_config_json_file_path` 指向的文件是否存在。
* 日志中是否出现 `Config JSON loaded file=...`。
* 是否使用完整启动文件传入了同名 launch / YAML 参数，导致 JSON 中的 `application_config` 被覆盖。
* 当前设备和固件是否支持 JSON 中的配置字段。

### 应该使用专用 launch 还是完整 launch

如果目标是尽量恢复 JSON 中的配置，推荐使用 `gemini_330_series_sdk_json.launch`。

如果想保留启动文件中的大量参数控制，可以继续使用 `gemini_330_series.launch` 等完整启动文件，但需要明确哪些参数会覆盖 JSON。

### 导出的 JSON 是否包含最终配置

导出的 JSON 基于当前相机最终状态。导出前，节点会把当前 ROS 侧的 sensor、point cloud 和 HDR merge 配置同步到 SDK `application_config`，再调用 SDK 导出 JSON。

如果启动时导入了 JSON，同时 launch / YAML 覆盖了部分配置，导出的文件会包含覆盖后的最终配置。
