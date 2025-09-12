## Registration script (required)

To allow the Orbbec cameras to be recognized correctly on Linux, install the udev rules:

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
sudo bash ./scripts/install_udev_rules.sh
```

This step is **mandatory** for Linux users.

`Notes:` If this script is not executed, open the device will fail due to permission issues. You need to run the sample with sudo (administrator privileges).

Install udev rules:

