## 使用Orbbec ROS包降低CPU使用率

本文档概述了在使用**Gemini 330系列相机**的**OrbbecSDK_RO### 滤波器配置影响

| 应用的滤波### 进一步优化

|           参数           |                  推荐值                  |                      说明                      |
| :----------------------------: | :----------------------------------------------: | :---------------------------------------------: |
|     `depth_registration`     | `false`或使用`align_mode=HW`的`true` |      软件对齐消耗更多CPU      |
|     `enable_point_cloud`     |                    `false`                    |     禁用点云减少CPU使用率     |
| `enable_colored_point_cloud` |                    `false`                    | 禁用彩色点云减少CPU使用率 |                                 | libuvc CPU使用率 | CPU使用率增加 | v4l2 CPU使用率 | CPU使用率增加 |
| ----------------------------------------------------- | ---------------- | ------------------ | -------------- | ------------------ |
| 无滤波器（基准）                                 | 116.0%           | 0.0%（基准）    | 45.7%          | 0.0%（基准）    |
| `(软件)noise_removal_filter`                    | 148.2%           | +32.2%             | 73.4%          | +27.7%             |
| `(软件)noise_removal_filter + spatial_filter` | 169.3%           | +53.3%             | 93.3%          | +47.6%             |
| `hardware_noise_removal_filter`                     | 115.7%           | -0.3%              | 45.6%          | -0.1%              |
| `hardware_noise_removal_filter + spatial_filter`  | 124.5%           | +8.5%              | 61.7%          | +16.0%             |中最小化CPU使用的策略。固件版本必须**不低于1.4.10**，并且`device`应设置为**Default**。

您可以在[示例](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples)中找到示例使用代码。

### 降低CPU使用率的推荐设置

为了在OrbbecSDK_ROS1中实现尽可能低的CPU使用率，建议配置以下参数。

|    参数    |             推荐值             |                      说明                      |
| :--------------: | :------------------------------------: | :--------------------------------------------: |
| `uvc_backend` |                `v4l2`                |     与`libuvc`相比CPU使用率更低     |
| `color_format` |                `RGB`                |         比`MJPG`格式CPU使用率更低         |
|    `filter`    | 仅使用`hardware_noise_removal_filter` | 其他滤波器会显著增加CPU使用率 |

### 用于测试的启动文件

* `gemini_330_lower_cpu_usage.launch`
* `multi_camera_lower_cpu_usage.launch`

### 测试环境

**硬件配置**

* **CPU**: Intel i7-8700 @ 3.20GHz
* **内存**: 24 GB
* **存储**: Micron 2200S NVMe 256GB
* **GPU**: NVIDIA GeForce GTX 1660Ti
* **操作系统**: Ubuntu20.04（虚拟机）

**ROS配置**

* **ROS版本**: ROS1 Noetic
* **SDK版本**: OrbbecSDK_ROS1 v2.2.1

**相机设置**

* 设备: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L
* 固件版本: 1.4.10

### 测试设置

* **流设置：**
  * 深度 / 左红外 / 右红外: 848×480 @ 30fps
  * 彩色: 848×480 @ 15fps

注意：以下CPU使用率数据主要关注`uvc_backend`、`color_format`和各种滤波器组合。

### 测试结果

### uvc_backend比较（RGB格式）

| libuvc CPU使用率 | v4l2 CPU使用率 | 绝对变化 |
| :--------------: | :------------: | :-------------: |
|      116.0%      |     45.7%     |     -70.3%     |

使用v4l2后端可以显著降低CPU使用率。在我们的实现中，v4l2无需对Linux内核进行任何补丁即可工作，允许用户轻松在v4l2和libuvc之间切换，并保持与标准Linux发行版的完全兼容性。

### color_format比较（MJPG vs RGB）

| 后端 | MJPG CPU使用率 | RGB CPU使用率 | 绝对变化 |
| :-----: | :------------: | :-----------: | :-------------: |
| libuvc |     132.6%     |    116.0%    |     -16.6%     |
|  v4l2  |     56.0%     |     45.7%     |     -10.3%     |

选择RGB格式而不是MJPG可以降低CPU使用率，因为MJPG图像的解码会消耗主机CPU资源。

### Filter Configuration Impact

| Filters Applied                                       | libuvc CPU Usage | CPU Usage Increase | v4l2 CPU Usage | CPU Usage Increase |
| ----------------------------------------------------- | ---------------- | ------------------ | -------------- | ------------------ |
| No Filter (benchmark)                                 | 116.0%           | 0.0%(benchmark)    | 45.7%          | 0.0%(benchmark)    |
| `(software)noise_removal_filter`                    | 148.2%           | +32.2%             | 73.4%          | +27.7%             |
| `(software)noise_removal_filter + spatial_filter` | 169.3%           | +53.3%             | 93.3%          | +47.6%             |
| `hardware_noise_removal_filter`                     | 115.7%           | -0.3%              | 45.6%          | -0.1%              |
| `hardware_noise_removal_filter + spatial_filter`  | 124.5%           | +8.5%              | 61.7%          | +16.0%             |

根据测试结果，仅使用`hardware_noise_removal_filter`对于`libuvc`（-0.3%）和`v4l2`（-0.1%）相比无滤波器基准，CPU使用率变化可忽略不计，因为此滤波器在相机硬件内部运行。相反，其他滤波器在主机系统上执行。将`spatial_filter`添加到硬件滤波器会导致CPU使用率适度增加，而应用基于软件的`noise_removal_filter`（无论是单独使用还是与`spatial_filter`结合使用）都会显著增加CPU负载。为了保持低CPU使用率，建议避免使用基于软件的滤波器，仅依赖`hardware_noise_removal_filter`。

### Further Optimization

|           Parameter           |                  Recommendation                  |                      Note                      |
| :----------------------------: | :----------------------------------------------: | :---------------------------------------------: |
|     `depth_registration`     | `false` or `true` with `align_mode=HW` |      Software alignment consumes more CPU      |
|     `enable_point_cloud`     |                    `false`                    |     Disabling point cloud reduces CPU usage     |
| `enable_colored_point_cloud` |                    `false`                    | Disabling colored point cloud reduces CPU usage |
