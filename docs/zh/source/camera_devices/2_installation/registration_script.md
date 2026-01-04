## 注册脚本（必需）

为了让奥比中光相机在Linux上被正确识别，请安装udev规则：

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

此步骤对于Linux用户是**强制性的**。

`注意：` 如果不执行此脚本，由于权限问题，打开设备将失败。您需要使用sudo（管理员权限）运行示例。

安装udev规则：

