### 从源代码构建

#### 环境

- **ROS**

  请直接参考ROS [wiki](http://wiki.ros.org/ROS/Installation)获取安装说明。

- **其他依赖项**

​		安装依赖项（请注意您的ROS发行版）：

```bash
# 假设您已经配置了ROS环境，下同
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
libdw-dev
```

#### 创建ROS工作空间并构建

创建ROS工作空间（如果您还没有）：

```bash
mkdir -p ~/ros_ws/src
```

获取源代码：

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
roscd orbbec_camera
git checkout v2-main
git branch  #检查分支切换是否成功
```

构建包：

```bash
cd ~/ros_ws
catkin_make
```

