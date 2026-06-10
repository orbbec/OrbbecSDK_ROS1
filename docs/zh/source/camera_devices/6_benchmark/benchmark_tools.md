# 性能基准测试工具

本节介绍性能基准测试工具，解释其目的、功能以及它可以帮助您测量的内容。

## common_benchmark_node.py

`common_benchmark_node.py` 是一个用于监控在 ROS 环境中运行的 Orbbec 相机性能的工具。它实时收集和记录关键相机指标，如帧率、延迟、系统资源使用和丢包率，帮助用户评估相机节点的稳定性和性能（每秒更新一次）。

功能：

- 测量发布的图像帧率和延迟（当前、最小、最大、平均）。
- 监控相机节点的 CPU/ARM 使用率（当前、最小、最大、平均）。
- 跟踪丢帧率（发布者）和丢包率（订阅者）。
- 将实时统计信息（1 Hz）打印到终端并将结果保存到 CSV 文件。
- 支持可配置的运行时长、CSV 输出路径和多相机名称。

在 ROS1 中，由于消息头包含 `seq` 字段，可以同时统计丢帧率和丢包率。

![common_benchmark_ros1](../image/benchmark_images/common_benchmark_ros1.png "ROS1")

默认参数运行：

```bash
rosrun orbbec_camera common_benchmark_node.py --run_time 30s
```

指定自定义 CSV 输出路径：

```bash
rosrun orbbec_camera common_benchmark_node.py \
  --run_time 2h \
  --csv_file /path/to/log.csv
```

多相机监控示例：

```bash
rosrun orbbec_camera common_benchmark_node.py \
  --run_time 1h \
  --csv_file /tmp/cam_log.csv \
  --camera_names camera,camera01
```

参数：

- `--run_time` / `_run_time`：监控持续时间。支持时间字符串格式：`10s`（秒）、`5m`（分钟）、`1h`（小时）、`2d`（天）。默认值：`10s`。
- `--csv_file` / `_csv_file`：CSV 输出文件路径。
- `--camera_names` / `_camera_names`：相机名称列表，可用逗号分隔。

## service_benchmark_node.py

`service_benchmark_node.py` 用于监控 service 调用性能。它会根据 YAML 配置文件批量调用多个 service，并统计调用次数、成功率、平均耗时、最小耗时和最大耗时。

功能：

- 对 YAML 配置文件中定义的多个服务进行基准测试。
- 自动根据当前 ROS service 类型构造请求。
- 可将基准测试结果保存到 CSV 文件。

![service benchmark](../image/benchmark_images/service_benchmark.png)

多个服务压测（YAML 配置 + CSV 输出）：

```bash
rosrun orbbec_camera service_benchmark_node.py \
  --yaml_file /path/to/default_service.yaml \
  --csv_file /path/to/results.csv
```

示例 YAML 配置文件位于 `scripts/default_service.yaml`，`default_count` 为未在单个条目指定调用次数时的默认调用次数。

## service_benchmark_node

`service_benchmark_node` 是 C++ service 调用 benchmark 工具。当前推荐优先使用 Python 版 `service_benchmark_node.py`，因为 Python 版可以直接通过 YAML 批量测试多个 service 并输出 CSV。

单个服务压测示例：

```bash
rosrun orbbec_camera service_benchmark_node \
  _service_name:=/camera/set_color_ae_roi \
  _service_type:=orbbec_camera/SetArrays \
  _request_data:='{data_param: [0, 1279, 0, 719]}'
```

多个服务压测（使用 YAML 配置）：

```bash
rosrun orbbec_camera service_benchmark_node \
  _yaml_file:=/path/to/default_service_cpp.yaml
```

## 基准测试使用建议

1. 先在少量服务上测试以验证网络与设备稳定，再扩展到全部服务列表。
2. 长时间运行时建议指定 `--csv_file` 到持久化目录，避免临时环境清理。
3. 可将不同设备的结果按日期归档，便于性能趋势分析。
4. 出现异常延迟可结合 `/camera/device_status` 话题排查设备状态。

完整 service YAML 示例可参考源码中的 `scripts/default_service.yaml`。
