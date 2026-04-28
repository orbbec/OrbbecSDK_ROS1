# Net Camera

> This section describes how to use Net camera in OrbbecSDK_ROS. Currently, only Femto Mega, Gemini 335Le and Gemini 435Le devices are supported, and other Net devices will be supported in the near future.

You can find example usage code in the [example](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples).

## Femto Mega & Gemini 435Le & Gemini 335Le

**Parameter Introduction**

Network device settings: `enumerate_net_device` is set to true, which will automatically enumerate network devices.

If you do not want to automatically enumerate network devices, you can set `enumerate_net_device` to false, `net_device_ip` to the device's IP address, and `net_device_port` to the default value of 8090.

* `enumerate_net_device`: Enable automatically enumerate network devices.
* `net_device_ip`: Set net device's IP address.
* `net_device_port`: Set net device's port. Usually, you can set it to 8090.

**Single Net camera**

If you need to run Gemini 435Le/Gemini 335Le, replace `femto_mega.launch` in the run command with `gemini435_le.launch` or `gemini_330_series.launch`.

For `femto_mega.launch` as an example:

**Automatically enumerate network devices:**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**Specify IP address to start the device:**

Note: `net_device_ip` needs to be changed to the IP address of the device, here it is 192.168.1.10.

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false net_device_ip:=192.168.1.10 net_device_port:=8090
```

**Multi Net camera**

For `multi_net_camera.launch` as an example:

```bash
roslaunch orbbec_camera multi_net_camera.launch
```

Use `list_devices_node` to inspect connected devices. Since v2.8.x, this tool also prints firmware version, local network interface name for Ethernet devices, and IP source type (`NONE`, `LLA`, `DHCP`, `PERSISTENT`).

## ip_config_tool Utility

The **`ip_config_tool`** executable allows you to configure network camera IP settings directly from ROS1, including DHCP, static IP, Force IP, and DHCP address assignment timeout. This is useful for quickly assigning or updating IP addresses without modifying launch files.

> **Note:** Configuration applied with `dhcp` or `set_ip` is written to the device. `force_ip` is temporary and must be applied again after the device is powered off or restarted.
> **Compatibility:** `set_device_ip` has been replaced by `ip_config_tool`. The old `old_ip` argument has been renamed to `current_ip`.

**Example Usage**

Show help:

```bash
rosrun orbbec_camera ip_config_tool --help
```

Enable DHCP:

```bash
rosrun orbbec_camera ip_config_tool dhcp \
--current_ip 192.168.1.10 \
--enable_dhcp true
```

Disable DHCP and set a static IP:

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--new_ip 192.168.1.11 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

Force IP by MAC address:

```bash
rosrun orbbec_camera ip_config_tool force_ip \
--force_ip_mac 54:14:FD:06:07:DA \
--new_ip 192.168.1.50 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

Set DHCP address assignment timeout:

```bash
rosrun orbbec_camera ip_config_tool set_dhcp_timeout \
--current_ip 192.168.1.10 \
--timeout 10
```

**Parameters**

- **`current_ip`** - Current IP address of the device.
- **`enable_dhcp`** - Enable or disable DHCP for the `dhcp` or `force_ip` subcommand.
- **`new_ip`** - Static IP address to assign.
- **`mask`** - Subnet mask for the new IP.
- **`gateway`** - Gateway address for the new IP.
- **`force_ip_mac`** - Target MAC address for Force IP.
- **`timeout`** / **`dhcp_assign_ip_timeout`** - DHCP address assignment timeout in seconds.

> **Version notes:** The `LLA` switch was supported only by Gemini 335Le firmware `1.7.05` and above and Gemini 435Le firmware `1.3.17` and above.

## Force IP Function

The **Force IP** feature allows you to assign a **static IP address** to a network camera, overriding DHCP settings. This is useful when multiple network cameras are connected, and you need each device to have a fixed IP for reliable communication.

> **Note:** The Force IP configuration **will be reset if the device is powered off or restarted**. You need to reapply the settings after reboot.

**Parameters**

- **`force_ip_enable`** - Enable the Force IP function. **Default:** `false`
- **`force_ip_mac`** - Target device MAC address when multiple cameras are connected (e.g., `"54:14:FD:06:07:DA"`). You can use the `list_devices_node` to find the MAC of each device. **Default:** `""`
- **`force_ip_address`** - Static IP address to assign. **Default:** `192.168.1.10`
- **`force_ip_subnet_mask`** - Subnet mask for the static IP. **Default:** `255.255.255.0`
- **`force_ip_gateway`** - Gateway address for the static IP. **Default:** `192.168.1.1`

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
