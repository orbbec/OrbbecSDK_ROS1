## Reducing CPU Usage with Orbbec ROS Package

This document outlines strategies for minimizing CPU usage in the **OrbbecSDK_ROS1 v2** environment when using **Gemini 330 series cameras**. The firmware version must be **no lower than 1.4.10**, and `device` should be set to **Default**.

### Recommended Settings for Lower CPU Usage

To achieve the lowest possible CPU usage in OrbbecSDK_ROS1, it is recommended to configure the following parameters.

|    Parameter    |             Recommendation             |                      Note                      |
| :--------------: | :------------------------------------: | :--------------------------------------------: |
| `uvc_backend` |                `v4l2`                |     Lower CPU usage compared to `libuvc`     |
| `color_format` |                `RGB`                |         Lower CPU usage than `MJPG`         |
|    `filter`    | Only `hardware_noise_removal_filter` | Other filters significantly increase CPU usage |

### Launch Files Used for Testing

* `gemini_330_lower_cpu_usage.launch`
* `multi_camera_lower_cpu_usage.launch`

### Test environment

#### Hardware Configuration

* **CPU**: Intel i7-8700 @ 3.20GHz
* **Memory**: 24 GB
* **Storage**: Micron 2200S NVMe 256GB
* **GPU**: NVIDIA GeForce GTX 1660Ti
* **OS**: Ubuntu20.04(Virtual Machines)

#### ROS Configuration

* **ROS Version**: ROS1 Noetic
* **SDK Version**: OrbbecSDK_ROS1 v2.2.1

#### Camera Setup

* Devices: 2x Gemini 335, 1x Gemini 336, 1x Gemini 336L
* Firmware Version: 1.4.10

### Test Setup

* **Stream Settings:**
  * Depth / IR Left / IR Right: 848×480 @ 30fps
  * Color: 848×480 @ 15fps

Note: The following CPU usage data focuses on `uvc_backend`, `color_format` and various filter combinations.

### Test Results

### 1.  `uvc_backend` Comparison (RGB format)

| libuvc CPU Usage | v4l2 CPU Usage | Absolute Change |
| :--------------: | :------------: | :-------------: |
|      116.0%      |     45.7%     |     -70.3%     |

The CPU usage can be significantly reduced with v4l2 backend. In our implementation, v4l2 works without requiring any patches to the Linux kernel, allowing users to easily switch between v4l2 and libuvc and maintaining full compatibility with standard Linux distributions.

### 2. `color_format` Comparison (MJPG vs RGB)

| Backend | MJPG CPU Usage | RGB CPU Usage | Absolute Change |
| :-----: | :------------: | :-----------: | :-------------: |
| libuvc |     132.6%     |    116.0%    |     -16.6%     |
|  v4l2  |     56.0%     |     45.7%     |     -10.3%     |

The CPU usage can be reduced if the RGB format is selected instead of MJPG, since the decoding of MJPG image will consume the host CPU resource.

### 3. Filter Configuration Impact

| Filters Applied                                       | libuvc CPU Usage | CPU Usage Increase | v4l2 CPU Usage | CPU Usage Increase |
| ----------------------------------------------------- | ---------------- | ------------------ | -------------- | ------------------ |
| No Filter (benchmark)                                 | 116.0%           | 0.0%(benchmark)    | 45.7%          | 0.0%(benchmark)    |
| `(software)noise_removal_filter`                    | 148.2%           | +32.2%             | 73.4%          | +27.7%             |
| `(software)noise_removal_filter + spatial_filter` | 169.3%           | +53.3%             | 93.3%          | +47.6%             |
| `hardware_noise_removal_filter`                     | 115.7%           | -0.3%              | 45.6%          | -0.1%              |
| `hardware_noise_removal_filter + spatial_filter`  | 124.5%           | +8.5%              | 61.7%          | +16.0%             |

Based on the test results, using only the `hardware_noise_removal_filter` results in a negligible change in CPU usage for both `libuvc` (-0.3%) and `v4l2` (-0.1%) compared to the no-filter benchmark, as this filter runs internally on the camera hardware. In contrast, other filters execute on the host system. Adding the `spatial_filter` to the hardware filter leads to a moderate increase in CPU usage, while applying the software-based `noise_removal_filter` —either alone or combined with `spatial_filter` —significantly increases CPU load. To maintain low CPU usage, it is recommended to avoid software-based filters and rely solely on the `hardware_noise_removal_filter`.

## Further Optimization

|           Parameter           |                  Recommendation                  |                      Note                      |
| :----------------------------: | :----------------------------------------------: | :---------------------------------------------: |
|     `depth_registration`     | `false` or `true` with `align_mode=HW` |      Software alignment consumes more CPU      |
|     `enable_point_cloud`     |                    `false`                    |     Disabling point cloud reduces CPU usage     |
| `enable_colored_point_cloud` |                    `false`                    | Disabling colored point cloud reduces CPU usage |
