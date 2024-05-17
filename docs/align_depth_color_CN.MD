## 深度图像与彩色图像的对齐

本节介绍如何在ROS中将深度图像与彩色图像对齐，以创建叠加图像。这对于需要同步不同传感器模态的视觉信息的应用来说非常有用。

### 深度与彩色图像对齐和查看的命令

1. **基础的深度到彩色对齐：**

   要将深度图像简单对齐到彩色图像，请使用以下命令：

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
   ```

   这个命令启用了深度注册功能，但不会打开视图窗口。

2. **查看深度与彩色叠加：**

   如果你想查看深度图像和彩色图像的叠加效果，可以使用以下命令来启用视图窗口：

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true enable_d2c_viewer:=true
   ```

   该命令会启动带有深度到彩色注册的相机节点，并打开一个视图窗口显示叠加图像。

### 在RViz中选择主题

为了在RViz中可视化对齐后的图像：

1. 在运行以上任一命令后启动RViz。
2. 选择显示深度到彩色叠加图像的主题。下图显示了如何选择主题：
   
   ![选择深度到彩色叠加的主题](./images/image3.png)

### 深度到彩色叠加的示例图

选择了正确的主题后，在RViz中你将能看到深度与彩色叠加的图像。下面是一个可能的示例图：

![深度到彩色叠加图像示例](./images/image4.jpg)