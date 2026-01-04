## ROS Package QuickStarts

### Introduction

This section provides a quick start to using the Orbbec ROS 1 wrapper.
 You will learn how to:

- Launch a camera node.
- Visualize depth/color streams in **RViz**.
- Interact with topics and services using **ROS 1 CLI tools**.

------

### Build your First Camera Application

#### Step 1: Source ROS 1 and Workspace

Make sure ROS 1 and your workspace environment are sourced:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros_ws/devel/setup.bash
```

#### Step 2: Launch the Camera Node

- On terminal 1

```bash
roscd orbbec_camera
rosrun orbbec_camera list_devices_node   # Check if the camera is connected
roslaunch orbbec_camera gemini_330_series.launch   # Or other launch file
```

If you have multiple cameras connected, you can specify the **serial number**:

```bash
roslaunch orbbec_camera gemini_330_series.launch serial_number:=<YourCameraSN>
```

#### Step 3: Visualize in RViz

Launch RViz and load the default config:

- On terminal 2

```bash
rviz
```

- Add an **Image** display, set topic to `/camera/color/image_raw`.
- Add another **Image** display for `/camera/depth/image_raw`.
- Optionally, add a **PointCloud2** display for `/camera/depth/points`.

You should now see the color stream, depth stream, and 3D point cloud in RViz.

------

### Sample Features

After the node is running, try some ROS 1 CLI commands:

#### List available topics / services / parameters

```bash
rostopic list
rosservice list
rosparam list
```

#### Echo a topic

View depth camera data:

```bash
rostopic echo /camera/depth/camera_info
```

#### Call a service

For example, get device Information:

```bash
rosservice call /camera/get_device_info "{}"
```

#### Record with rosbag

```bash
rosbag record /camera/color/image_raw /camera/depth/image_raw
```
