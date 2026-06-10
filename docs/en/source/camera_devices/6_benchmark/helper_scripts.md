# Debug Helper Scripts

This section describes lightweight debug scripts kept in the ROS1 project. These scripts mainly target fixed-name multi-camera scenarios, so edit the camera names or service names before use if your setup differs.

## open_stream.sh

`open_stream.sh` starts multiple `rostopic hz` commands for fixed multi-camera depth / IR topics. It is useful for quickly checking whether image streams are being published continuously.

The default topics are depth and IR image topics from `/ob_camera_01` to `/ob_camera_05`. Edit the script first if your camera names are different.

```bash
cd src/OrbbecSDK_ROS1/scripts
./open_stream.sh
```

## save_image.sh

`save_image.sh` calls fixed multi-camera `/save_images` services to trigger image saving in camera nodes. The default services are `/ob_camera_01/save_images` through `/ob_camera_05/save_images`.

```bash
cd src/OrbbecSDK_ROS1/scripts
./save_image.sh
```

For the default single-camera name, you can call the service directly:

```bash
rosservice call /camera/save_images "{}"
```

## set_laser.py

`set_laser.py` calls `/camera/set_laser` with `true` to quickly enable the laser. The default service name is fixed to `/camera/set_laser`; edit the script first for multi-camera or non-default camera names.

```bash
cd src/OrbbecSDK_ROS1/scripts
python3 set_laser.py
```
