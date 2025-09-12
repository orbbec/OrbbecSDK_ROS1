### Build from Source

#### Environment

- **ROS**

  Please refer directly to the ROS [wiki](http://wiki.ros.org/ROS/Installation) for installation instructions.

- **Other Dependencies**

​		Install dependencies (be careful with your ROS distribution):

```bash
# Assuming you have sourced the ROS environment, same below
sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
ros-${ROS_DISTRO}-image-transport-plugins ros-${ROS_DISTRO}-compressed-image-transport \
ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs \
libdw-dev
```

#### Create ROS Workspace and Build

Create a ROS workspace (if you don't have one):

```bash
mkdir -p ~/ros_ws/src
```

Get the source code:

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
roscd orbbec_camera
git checkout v2-main
git branch  #Check whether the branch switch is successful
```

Build the package:

```bash
cd ~/ros_ws
catkin_make
```

