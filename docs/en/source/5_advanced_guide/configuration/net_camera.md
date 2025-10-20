# Net_camera

> This section describes how to use Net camera in OrbbecSDK_ROS.Currently, only Femto_Mega and Gemini 335Le devices are supported, and other Net devices will be supported in the near future.

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples).

## Femto Mega

**Parameter Introduction**

Network device settings: default `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices,you can set `enumerate_net_device` to false, `ip_address` to the device's IP address, and `port` to the default value of 8090.

* `enumerate_net_device` : Enable automatically enumerate network devices.
* `ip_address` : Setting net device's IP address.
* `port` : Setting net device's port.Usually, you can set it to 8090.

**Single Net camera**

If you need to run Gemini 435Le/Gemini 335Le, you only need to replace femto_mega.launch in the run command with gemini435_le.launch/gemini335_le.launch.

For femto_mega.launchas an example:

**automatically enumerate network devices:**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**Specify IP address to start the device:**

Note:`ip_address` needs to be changed to the IP address of the device, here it is 192.168.1.10

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false ip_address:=192.168.1.10 port:=8090
```

**Multi Net camera**

For multi_net_camera.launchas an example:

```bash
roslaunch orbbec_camera multi_net_camera.launch
```

## set_device_ip Utility

The **`set_device_ip`** executable allows you to configure the IP settings of a network camera directly from ROS 2, including switching between DHCP and static IP, and setting subnet mask and gateway. This is useful for quickly assigning or updating IP addresses without modifying launch files.

> **Note:** The IP settings applied with `set_device_ip` are **permanent** and **will not be reset** if the device is powered off or restarted.

**Example Usage**

```bash
rosrun orbbec_camera set_device_ip \
_old_ip:=192.168.1.10 \
_dhcp:=false \
_new_ip:=192.168.1.11 \
_mask:=255.255.255.0 \
_gateway:=192.168.1.1
```

**Parameters**

- **`old_ip`** – Current IP address of the device.
- **`dhcp`** – Set to `true` to use DHCP or `false` for static IP.
- **`new_ip`** – Static IP address to assign when DHCP is disabled.
- **`mask`** – Subnet mask for the new IP.
- **`gateway`** – Gateway address for the new IP.

## Force IP Function

The **Force IP** feature allows you to assign a **static IP address** to a network camera, overriding DHCP settings. This is useful when multiple network cameras are connected, and you need each device to have a fixed IP for reliable communication.

> **Note:** The Force IP configuration **will be reset if the device is powered off or restarted**. You need to reapply the settings after reboot.

**Parameters**

- **`force_ip_enable`** – Enable the Force IP function. **Default:** `false`
- **`force_ip_mac`** – Target device MAC address when multiple cameras are connected (e.g., `"54:14:FD:06:07:DA"`). You can use the `list_devices_node` to find the MAC of each device. **Default:** `""`
- **`force_ip_address`** – Static IP address to assign . **Default:** `192.168.1.10`
- **`force_ip_subnet_mask`** – Subnet mask for the static IP. **Default:** `255.255.255.0`
- **`force_ip_gateway`** – Gateway address for the static IP. **Default:** `192.168.1.1`

**Example Usage**

- **Enable Force IP for a specific device:**

```bash
roslaunch orbbec_camera gemini_330_series.launch \
force_ip_enable:=true \
force_ip_mac:=54:14:FD:06:07:DA \
force_ip_address:=192.168.1.50 \
force_ip_subnet_mask:=255.255.255.0 \
force_ip_gateway:=192.168.1.1 \
net_device_ip:=192.168.1.50 \
net_device_port:=8090
```

> Tip: Make sure the camera is connected and its MAC address is correct before enabling Force IP. Use `list_devices_node` to check the MAC address of all connected cameras.
