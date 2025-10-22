# 基准测试使用指南

本节介绍如何在 C++ 与 Python 中使用基准测试工具，并给出示例 YAML 配置文件。

## 使用通用基准测试节点（common_benchmark_node）

* **默认参数运行**

```bash
rosrun orbbec_camera common_benchmark_node.py --run_time 30s
```

默认情况下生成的 CSV 文件保存在当前工作空间目录下（文件名 camera_monitor_log.csv）。

* **指定自定义 CSV 输出路径**

```bash
rosrun orbbec_camera common_benchmark_node.py \
  --run_time 2h \
  --csv_file /path/to/log.csv
```

* **参数说明**
  * `--run_time`：监控持续时间。支持时间字符串格式：`10s`（秒）、`5m`（分钟）、`1h`（小时）、`2d`（天）。默认值：`10s`。
  * `--csv_file`：CSV 输出文件路径。未指定时使用工作空间目录并命名为 `camera_monitor_log.csv`。

## 使用服务基准测试节点（service_benchmark_node）

### ROS1 C++

* **单个服务压测示例**

```bash
rosrun orbbec_camera service_benchmark_node \
  _service_name:=/camera/set_color_ae_roi \
  _service_type:=orbbec_camera/SetArrays \
  _request_data:='{data_param: [0, 1279, 0, 719]}'
```

* **多个服务压测（使用 YAML 配置）**

```bash
rosrun orbbec_camera service_benchmark_node \
  _yaml_file:=/path/to/default_service_cpp.yaml
```

### ROS1 Python

* **单个服务压测示例**

```bash
rosrun orbbec_camera service_benchmark_node.py \
  --service_name /camera/get_depth_exposure \
  --count 10
```

* **多个服务压测（YAML 配置 + CSV 输出）**

```bash
rosrun orbbec_camera service_benchmark_node.py \
  --yaml_file /path/to/default_service.yaml \
  --csv_file /path/to/results.csv
```

### 示例 YAML 配置文件

我们提供一个示例 YAML 配置文件位于 `scripts` 目录下，文件名为 `service_default.yaml`，用于批量对多个服务进行调用与统计。`default_count` 为未在单个条目指定调用次数时的默认调用次数。

```yaml
default_count: 50

services:
- name: /camera/set_filter
  type: orbbec_camera/SetFilter
  request: {filter_name: DecimationFilter,filter_enable: false,filter_param: [5]}
- name: /camera/set_depth_exposure
  type: orbbec_camera/SetInt32
  request: {data: 3000}
- name: /camera/get_depth_exposure
  type: orbbec_camera/GetInt32
- name: /camera/set_depth_ae_roi
  type: orbbec_camera/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/reset_depth_exposure
  type: std_srvs/Empty
- name: /camera/get_depth_gain
  type: orbbec_camera/GetInt32
- name: /camera/set_depth_gain
  type: orbbec_camera/SetInt32
  request: {data: 200}
- name: /camera/reset_depth_gain
  type: std_srvs/Empty
- name: /camera/set_depth_mirror
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/set_depth_flip
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/set_depth_rotation
  type: orbbec_camera/SetInt32
  request: {data: 90}
- name: /camera/set_depth_auto_exposure
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/get_depth_auto_exposure
  type: orbbec_camera/GetBool
- name: /camera/get_depth_camera_info
  type: orbbec_camera/GetCameraInfo
- name: /camera/get_color_exposure
  type: orbbec_camera/GetInt32
- name: /camera/set_color_exposure
  type: orbbec_camera/SetInt32
  request: {data: 30}
- name: /camera/set_color_ae_roi
  type: orbbec_camera/SetArrays
  request: {data_param: [0,1279,0,719]}
- name: /camera/reset_color_exposure
  type: std_srvs/Empty
- name: /camera/get_color_gain
  type: orbbec_camera/GetInt32
- name: /camera/set_color_gain
  type: orbbec_camera/SetInt32
  request: {data: 20}
- name: /camera/reset_color_gain
  type: std_srvs/Empty
- name: /camera/set_color_mirror
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/set_color_flip
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/set_color_rotation
  type: orbbec_camera/SetInt32
  request: {data: 90}
- name: /camera/set_color_auto_exposure
  type: std_srvs/SetBool
  request: {data: false}
- name: /camera/get_color_auto_exposure
  type: orbbec_camera/GetBool
- name: /camera/get_color_camera_info
  type: orbbec_camera/GetCameraInfo
- name: /camera/get_auto_white_balance
  type: orbbec_camera/GetInt32
- name: /camera/set_auto_white_balance
  type: orbbec_camera/SetInt32
  request: {data: 0}
- name: /camera/get_white_balance
  type: orbbec_camera/GetInt32
- name: /camera/set_white_balance
  type: orbbec_camera/SetInt32
  request: {data: 3000}
- name: /camera/reset_white_balance
  type: std_srvs/Empty
- name: /camera/set_laser
  type: std_srvs/SetBool
  request: {data: true}
- name: /camera/set_ldp
  type: std_srvs/SetBool
  request: {data: false}
```

## 使用建议

1. 先在少量服务上测试以验证网络与设备稳定，再扩展到全部服务列表。
2. 长时间运行时建议指定 `--csv_file` 到持久化目录，避免临时环境清理。
3. 可将不同设备的结果按日期归档，便于性能趋势分析。
4. 出现异常延迟可结合 `/camera/device_status` 话题排查设备状态。
