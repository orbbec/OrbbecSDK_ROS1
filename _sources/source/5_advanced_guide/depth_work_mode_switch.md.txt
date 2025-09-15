# Depth work mode switch

- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch
  file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch is the camera's default configuration. If you need to modify
  it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```XML
<!-- Depth work mode support is as follows： -->
<!-- Unbinned Dense Default -->
<!-- Unbinned Sparse Default -->
<!-- Binned Sparse Default -->
<!-- Obstacle Avoidance -->
<arg name="depth_work_mode" default=""/>
```

- View depth work modes:

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

* Example:

```bash
roslaunch orbbec_camera gemini2L.launch depth_work_mode:="Unbinned Dense Default"
```
