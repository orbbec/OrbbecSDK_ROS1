# 深度工作模式切换

- 在启动相机之前，可以为相应的xxx.launch文件配置深度工作模式（depth_work_mode）支持。
- 深度工作模式切换由Gemini 2、Gemini 2 L和Gemini 2 XL相机支持。
- xxx.launch的默认深度工作模式配置是相机的默认配置。如果您需要修改它，可以根据需要切换到相应的模式。
- 具体相机深度工作模式支持类型可以在深度模式的注释中找到。

```XML
<!-- 深度工作模式支持如下： -->
<!-- Unbinned Dense Default -->
<!-- Unbinned Sparse Default -->
<!-- Binned Sparse Default -->
<!-- Obstacle Avoidance -->
<arg name="depth_work_mode" default=""/>
```

- 查看深度工作模式：

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

* 示例：

```bash
roslaunch orbbec_camera gemini2L.launch depth_work_mode:="Unbinned Dense Default"
```
