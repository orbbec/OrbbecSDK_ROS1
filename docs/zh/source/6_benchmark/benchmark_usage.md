# 基准测试使用指南

本节介绍如何在C++和Python中使用基准测试工具，并提供示例YAML配置文件。

## Using common benchmark node

* **Run with default settings**

```
rosrun orbbec_camera common_benchmark_node.py --run_time 30s
```

By default, the CSV file is saved in the current workspace.

* **Run with custom CSV output path**

```
rosrun orbbec_camera common_benchmark_node.py \
    --run_time 2h  \
    --csv_file /path/to/log.csv
```

* **Parameters**
  * **--run_time**: Duration for monitoring, specified as time strings like `"10s"`, `"5m"`, `"1h"`, `"2d"`. Default is 10 seconds.
  *  **--csv_file**: Path to the output CSV file. By default, it is saved in the workspace directory with the name "camera_monitor_log.csv".

## Using service benchmark node

### ROS1 C++

* **Single service benchmark**

```
rosrun orbbec_camera service_benchmark_node \
    _service_name:=/camera/set_color_ae_roi \
    _service_type:=orbbec_camera/SetArrays \
    _request_data:='{data_param: [0, 1279, 0, 719]}'
```

* **Multiple services benchmark (YAML config)**

```
rosrun orbbec_camera service_benchmark_node \
    _yaml_file:=/path/to/default_service_cpp.yaml
```

### ROS1 Python

* **Single service benchmark**

```
rosrun orbbec_camera service_benchmark_node.py \
    --service_name /camera/get_depth_exposure \
    --count 10
```

* **Multiple services benchmark (YAML config + CSV output)**

```
rosrun orbbec_camera service_benchmark_node.py \
    --yaml_file /path/to/default_service.yaml \
    --csv_file /path/to/results.csv
```

### **Example YAML configuration**

We provide an example YAML configuration, located in the `scripts` directory as `service_default.yaml`.

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
