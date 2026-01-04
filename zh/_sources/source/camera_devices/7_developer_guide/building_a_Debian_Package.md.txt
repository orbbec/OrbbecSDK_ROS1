# 构建Debian包

首先，确保安装了必要的工具：

```bash
sudo apt install debhelper fakeroot python3-bloom
```

要创建Debian包，请执行以下命令：

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```
