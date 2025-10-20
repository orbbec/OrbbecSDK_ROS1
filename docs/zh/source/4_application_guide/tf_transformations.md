### 从坐标A到坐标B的TF变换：

在奥比中光相机中，原点(0,0,0)取自camera_link位置

我们的包装器提供从每个传感器坐标到相机基座(camera_link)的静态TF变换

同时，它还提供从每个传感器ROS坐标到其对应光学坐标的TF变换。

以下是Gemini335模块的RGB传感器和右红外传感器在rviz中显示的静态TF变换示例：

```bash
roslaunch orbbec_description view_model.launch model:=gemini_335_336.urdf.xacro
```

![rviz中的模块](../image/application_guide/image2.png)
