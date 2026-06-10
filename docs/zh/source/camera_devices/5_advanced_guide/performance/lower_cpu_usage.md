## 使用 Orbbec ROS 包降低 CPU 使用率

本文档概述了在使用 **Gemini 330 系列相机** 的 **OrbbecSDK_ROS1 v2** 环境中最小化 CPU 使用率的策略。固件版本必须 **不低于 1.4.10**，并且 `device` 应设置为 **Default**。

您可以参考 [gemini_330_series_low_cpu.launch](https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/launch/gemini_330_series_low_cpu.launch) 启动文件中的低 CPU 占用配置。

### 降低 CPU 使用率的推荐设置

为了在 OrbbecSDK_ROS1 中实现尽可能低的 CPU 使用率，建议配置以下参数。

| 参数 | 推荐值 | 说明 |
| :---: | :---: | :---: |
| `uvc_backend` | `v4l2` | 与 `libuvc` 相比 CPU 使用率更低 |
| `color_format` | `RGB` | 比 `MJPG` 格式 CPU 使用率更低 |
| `filter` | 仅使用 `hardware_noise_removal_filter` | 其他滤波器会显著增加 CPU 使用率 |
| `depth_registration` | `false` 或 `true` 配合 `align_mode=HW` | 软件对齐消耗更多 CPU |
| `enable_point_cloud` | `false` | 禁用点云可降低 CPU 使用率 |
| `enable_colored_point_cloud` | `false` | 禁用彩色点云可降低 CPU 使用率 |

### 彩色流格式与订阅方式

v2.8.8 优化了彩色流图像发布流程：

- 当 `color_format` 为 RGB/YUYV 等非 MJPG 格式时，订阅 `/camera/color/image_raw`。
- 当 `color_format:=MJPG` 时，建议订阅 `/camera/color/image_raw/compressed`。ROS wrapper 会直接发布压缩图像，避免额外解码，从而显著降低 MJPG 场景下的 CPU 占用，甚至比 RGB 格式更低。
- 如果订阅 `/camera/color/image_raw`，MJPG 仍需要在主机侧解码，CPU 占用会更高。

### 用于测试的启动文件

* [gemini_330_series_low_cpu.launch](https://github.com/orbbec/OrbbecSDK_ROS1/blob/v2-main/launch/gemini_330_series_low_cpu.launch)

### 测试环境

**硬件配置**

* **CPU**: Intel i7-8700 @ 3.20GHz
* **内存**: 24 GB
* **存储**: Micron 2200S NVMe 256GB
* **GPU**: NVIDIA GeForce GTX 1660Ti
* **操作系统**: Ubuntu20.04（虚拟机）

**ROS 配置**

* **ROS 版本**: ROS1 Noetic
* **SDK 版本**: OrbbecSDK_ROS1 v2.2.1

**相机设置**

* 设备: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L
* 固件版本: 1.4.10

### 测试设置

* **流设置：**
  * 深度 / 左红外 / 右红外: 848×480 @ 30fps
  * 彩色: 848×480 @ 15fps

注意：以下 CPU 使用率数据主要关注 `uvc_backend`、`color_format` 和各种滤波器组合。

### 测试结果

### uvc_backend 比较（RGB 格式）

| libuvc CPU 使用率 | v4l2 CPU 使用率 | 绝对变化 |
| :---: | :---: | :---: |
| 116.0% | 45.7% | -70.3% |

使用 v4l2 后端可以显著降低 CPU 使用率。在我们的实现中，v4l2 无需对 Linux 内核进行任何补丁即可工作，允许用户轻松在 v4l2 和 libuvc 之间切换，并保持与标准 Linux 发行版的完全兼容性。

### color_format 比较（MJPG vs RGB）

| 后端 | MJPG CPU 使用率 | RGB CPU 使用率 | 绝对变化 |
| :---: | :---: | :---: | :---: |
| libuvc | 132.6% | 116.0% | -16.6% |
| v4l2 | 56.0% | 45.7% | -10.3% |

选择 RGB 格式而不是 MJPG 可以降低 CPU 使用率，因为 MJPG 图像的解码会消耗主机 CPU 资源。

### 滤波器配置影响

| 应用的滤波器 | libuvc CPU 使用率 | CPU 使用率增加 | v4l2 CPU 使用率 | CPU 使用率增加 |
| --- | --- | --- | --- | --- |
| 无滤波器（基准） | 116.0% | 0.0%（基准） | 45.7% | 0.0%（基准） |
| `（软件）noise_removal_filter` | 148.2% | +32.2% | 73.4% | +27.7% |
| `（软件）noise_removal_filter + spatial_filter` | 169.3% | +53.3% | 93.3% | +47.6% |
| `hardware_noise_removal_filter` | 115.7% | -0.3% | 45.6% | -0.1% |
| `hardware_noise_removal_filter + spatial_filter` | 124.5% | +8.5% | 61.7% | +16.0% |

根据测试结果，仅使用 `hardware_noise_removal_filter` 对于 `libuvc`（-0.3%）和 `v4l2`（-0.1%）相比无滤波器基准，CPU 使用率变化可忽略不计，因为此滤波器在相机硬件内部运行。相反，其他滤波器在主机系统上执行。将 `spatial_filter` 添加到硬件滤波器会导致 CPU 使用率适度增加，而应用基于软件的 `noise_removal_filter`（无论是单独使用还是与 `spatial_filter` 结合使用）都会显著增加 CPU 负载。为了保持低 CPU 使用率，建议避免使用基于软件的滤波器，仅依赖 `hardware_noise_removal_filter`。
