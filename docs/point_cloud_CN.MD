## 在ROS 1中启用和可视化点云

本节演示如何从摄像头节点启用点云数据输出，并使用RViz进行可视化，类似于在[启动摄像头节点](./start_camera_node.MD)文档中讨论的初始摄像头节点设置。

### 启用深度点云

#### 启用深度点云的命令

要激活深度信息的点云数据流，请使用以下命令：

```bash
roslaunch orbbec_camera gemini_330_series.launch enable_point_cloud:=true
```

#### 在RViz中可视化深度点云

运行上述命令后，执行以下步骤以可视化深度点云：

1. 打开RViz。
2. 添加一个`PointCloud2`显示。
3. 选择`/camera/depth/points`主题进行可视化。
4. 将固定框架设置为`camera_link`以正确对齐数据。

##### 示例可视化

这是在RViz中深度点云可能的样子：

![深度点云可视化](./images/image5.jpg)

### 启用彩色点云

#### 启用彩色点云的命令

要启用彩色点云功能，请输入以下命令：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_colored_point_cloud:=true
```

#### 在RViz中可视化彩色点云

要可视化彩色点云数据：

1. 执行命令后启动RViz。
2. 添加一个`PointCloud2`显示面板。
3. 从列表中选择`/camera/depth_registered/points`主题。
4. 确保固定框架设置为`camera_link`。

##### 示例可视化

RViz中彩色点云的结果应该类似于此：

![彩色点云可视化](./images/image6.jpg)