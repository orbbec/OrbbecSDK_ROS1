# Net_camera

> This section describes how to use Net camera in OrbbecSDK_ROS.Currently, only Femto_Mega and Gemini 335Le devices are supported, and other Net devices will be supported in the near future.

## Femto Mega

### Parameter Introduction

Network device settings: default `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices,you can set `enumerate_net_device` to false, `ip_address` to the device's IP address, and `port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `ip_address` : Setting net device's IP address.
* `port` : Setting net device's port.Usually, you can set it to 8090.

## Single Net camera

If you need to run Gemini 435Le, you only need to replace [femto_mega.launch](../../launch/femto_mega.launch) in the run command with [gemini435_le.launch](../../launch/gemini435_le.launch)

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

## Gemini 335Le

Network device settings: `enumerate_net_device` must be set to true, set `ip_address` to the IP address of the device, and set `port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `ip_address` : Setting net device's IP address.
* `port` : Setting net device's port.Usually, you can set it to 8090.

## Single Net camera

For [gemini_330_series.launch](../../launch/gemini_330_series.launch) as an example:

### automatically enumerate network devices:

```bash
roslaunch orbbec_camera gemini_330_series.launch enumerate_net_device:=true
```

### Specify IP address to start the device:

Note:`ip_address` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
roslaunch orbbec_camera gemini_330_series.launch enumerate_net_device:=true ip_address:=192.168.1.10 port:=8090
```

## Multi Net camera

For [multi_net_camera.launch](./multi_net_camera.launch) as an example:

```bash
roslaunch orbbec_camera multi_net_camera.launch
```
