## Using the Orbbec ROS package to get lower CPU usage

This section shows you how to reduce CPU usage for one or more cameras in a ROS 1 environment. This method only works with Gemini 330 series cameras, and the firmware version needs to be above 1.4.10.

The example launch files used in this section are `gemini_330_lower_cpu_usage.launch` and `multi_camera_lower_cpu_usage.launch`.

### Parameters that affect CPU usage

`uvc_backend`:Global UVC Backend select on Linux,optional values: libuvc, v4l2.
`color_format`:Color stream coding format.
`depth_registration`: Enables alignment of the depth frame to the color frame.
`enable_point_cloud`:Enables the point cloud.
`enable_colored_point_cloud`:Enables the RGB point cloud.
`filter`:decimation_filter,hdr_merge,sequenced_filter,threshold_filter,hardware_noise_removal_filter,noise_removal_filter,spatial_filter,temporal_filter,hole_filling_filter.
`Video stream resolution and frame rate`

### Data comparison

> Since different machines and different software environments will result in different CPU usage, the CPU improvement rate is calculated based on the device's CPU benchmark.
> A comparing B:
> `CPU improvement rate=（CPUA-CPUB）/CPUB*100`

In this example, we used two different video stream resolutions as test benchmarks to provide a performance comparison between lower and higher resolutions.(Currently, data is only provided for `uvc_backend`, `color_format`, and some `filter`)

#### Depth&Left_ir&Right_ir 424 * 266 15fps,Color 848 * 480 15fps

##### uvc_backend

libuvc comparing v4l2 In the same environment, CPU improvement rate is ***189.6%***

* libuvc is ***43.1%***
* v4l2 is ***21%***

##### color_format

ROS1 color stream sets MJPG encoding format and decodes it to RGB comparing RGB encoding format (including YUYV format to RGB conversion operation cost)，The libuvc protocol is basically the same, and the CPU improvement rate under the v4l2 protocol is ***51.4%***

* libuvc:RGB encoding format is ***43.1%***，MJPG encoding format and decoded into RGB is ***42.7%***
* v4l2:RGB encoding format is ***21%***，MJPG encoding format and decoded into RGB is ***31.8%***

##### filter

The color stream setting RGB encoding and MJPG have no effect on filtering and are basically the same. The CPU usage of the depth stream is basically the same under the configuration of 424*266 15fps.

|             Filter Configuration             | libuvc(CPU improvement rate) | v4l2(CPU improvement rate) |
| :------------------------------------------: | :--------------------------: | :------------------------: |
|                  No filter                  |          43.1%(0%)          |          21%(0%)          |
|        hardware_noise_removal_filter        |         42.7%(-0.9%)         |          21%(0%)          |
| hardware_noise_removal_filter+spatial_filter |          43%(-0.2%)          |        21.7%(3.3%)        |
|             noise_removal_filter             |          43.1%(0%)          |        22.1%(5.2%)        |
|     noise_removal_filter+spatial_filter     |         43.2%(0.2%)         |         22.9%(9%)         |

#### Depth&Left_ir&Right_ir 848 * 480 30fps,Color 848 * 480 15fps

##### uvc_backend

libuvc comparing v4l2 In the same environment, CPU improvement rate is ***189.6%***

* libuvc is ***117.28%***
* v4l2 is ***40.5%***

##### color_format

ROS1 color stream sets MJPG encoding format and decodes it to RGB comparing RGB encoding format (including YUYV format to RGB conversion operation cost).The CPU improvement rate under the libuvc protocol is ***14.3%*** the CPU improvement rate under the v4l2 protocol is ***22.5%***

* libuvc:RGB encoding format is ***116%***，MJPG encoding format and decoded into RGB is ***132.6%***
* v4l2:RGB encoding format is ***45.7%***，MJPG encoding format and decoded into RGB is ***56%***

##### filter

When the depth stream is configured at 848*480 30fps, the CPU usage is significantly different.

|             Filter Configuration             | libuvc(CPU improvement rate) | v4l2(CPU improvement rate) |
| :------------------------------------------: | :--------------------------: | :------------------------: |
|                  No filter                  |           116%(0%)           |         45.7%(0%)         |
|        hardware_noise_removal_filter        |        115.7%(-0.3%)        |        45.6%(-0.2%)        |
| hardware_noise_removal_filter+spatial_filter |         124.5%(7.3%)         |         61.7%(35%)         |
|             noise_removal_filter             |        148.2%(27.8%)        |        73.4%(60.6%)        |
|     noise_removal_filter+spatial_filter     |        169.3%(45.9%)        |       93.3%(104.2%)       |

### Example launch

This is the current example file for running the camera to achieve the lowest CPU usage, excluding the impact of filters.

Use the following command to start the single-camera configuration:

```bash
roslaunch orbbec_camera gemini_330_lower_cpu_usage.launch
```

Use the following command to start the multi-camera configuration:

```bash
roslaunch orbbec_camera multi_camera_lower_cpu_usage.launch
```
