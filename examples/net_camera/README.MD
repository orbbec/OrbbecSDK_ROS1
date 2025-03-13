# Net_camera

> This section describes how to use Net camera in OrbbecSDK_ROS2.Currently, only Femto_Mega devices are supported, and other Net devices will be supported in the near future.

## Parameter Introduction

Network device settings: default `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices,you can set `enumerate_net_device` to false, `net_device_ip` to the device's IP address, and `net_device_port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `ip_address` : Setting net device's IP address.
* `port` : Setting net device's port.Usually, you can set it to 8090.

## Single Net camera

For [femto_mega.launch](../../launch/femto_mega.launch) as an example:

### automatically enumerate network devices:

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

### Specify IP address to start the device:

Note:`ip_address` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false ip_address:=192.168.1.10 port:=8090
```

## Multi Net camera

For [multi_net_camera.launch](./multi_net_camera.launch) as an example:

```bash
roslaunch orbbec_camera multi_net_camera.launch
```
