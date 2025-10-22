# 简介

本节介绍基准测试工具，说明其目的、功能以及它可以帮助您测量的内容。

## 通用基准测试节点

`common_benchmark_node.py`是一个用于监控在ROS环境中运行的Orbbec相机性能的工具。它实时收集和记录关键的相机指标，如帧率、延迟、系统资源使用情况和丢包率，帮助用户评估相机节点的稳定性和性能（每秒更新一次）。

**功能特性**

- 测量发布的图像帧率和延迟（当前、最小、最大、平均值）

- 监控相机节点的CPU/ARM使用率（当前、最小、最大、平均值）

- 跟踪丢帧率（发布者）和丢包率（订阅者）

- 将实时统计信息（1 Hz）打印到终端并保存结果到CSV文件

- 支持可配置的运行时长和CSV输出路径

**示例**

在ROS1中，可以测量丢帧率和丢包率，而在ROS2中，由于头部缺少`seq`字段，因此只计算发布者端的丢帧率。

![common_benchmark_ros1](../image/benchmark_images/common_benchmark_ros1.png "ROS1")

![common_benchmark_ros2](../image/benchmark_images/common_benchmark_ros2.png "ROS2")

## 服务基准测试节点

`service_benchmark_node`工具用于监控服务调用的性能。它可以测量服务调用的成功率和执行服务所需的时间。

**功能特性**

- 对单个服务调用进行基准测试，测量延迟和成功率

- 根据YAML配置文件中定义的多个服务进行基准测试

- 可选择将基准测试结果保存到CSV文件

**示例**

![service benchmark](../image/benchmark_images/service_benchmark.png)

当您需要为多个服务收集数据时，建议使用CSV文件进行分析。