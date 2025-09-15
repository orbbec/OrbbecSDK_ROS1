# Building a Debian Package

First, ensure the necessary tools are installed:

```bash
sudo apt install debhelper fakeroot python3-bloom
```

To create the Debian package, execute these commands:

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```
