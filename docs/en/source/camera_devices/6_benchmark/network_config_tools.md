# Network Configuration Tools

This section describes the network camera IP configuration tool. For network camera startup, automatic enumeration, launching with a specified IP address, and camera-node Force IP parameters, see [Network Camera](../5_advanced_guide/configuration/net_camera.md).

## ip_config_tool

`ip_config_tool` configures network camera IP settings directly from ROS1, including DHCP, persistent IP, Force IP, and DHCP address assignment timeout. It does not require the camera node to be running and is useful for quickly assigning or updating IP addresses.

> **Note:** DHCP / persistent IP configuration applied with `set_ip` is written to the device. `force_ip` is temporary and must be applied again after the device is powered off or restarted.
> **Compatibility:** `set_device_ip` is kept as a legacy alias and calls `ip_config_tool`. The old `old_ip` argument has been renamed to `current_ip`.

Show help:

```bash
rosrun orbbec_camera ip_config_tool --help
```

Enable DHCP:

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp true \
--enable_persistent_ip false
```

Disable DHCP and set a persistent IP:

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp false \
--enable_persistent_ip true \
--new_ip 192.168.1.11 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

Enable both DHCP and persistent IP (requires a device/firmware that supports IP config V2):

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp true \
--enable_persistent_ip true \
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

## Parameters

- **`current_ip`**: Current IP address of the device.
- **`enable_dhcp`**: Enable or disable DHCP for the `set_ip` or `force_ip` subcommand.
- **`enable_persistent_ip`**: Enable or disable persistent IP for the `set_ip` subcommand.
- **`new_ip`**: Persistent IP or Force IP address to assign.
- **`mask`**: Subnet mask for the new IP.
- **`gateway`**: Gateway address for the new IP.
- **`force_ip_mac`**: Target MAC address for Force IP.
- **`timeout`** / **`dhcp_assign_ip_timeout`**: DHCP address assignment timeout in seconds.
- **`sdk_log_level`**: SDK file log level. Optional values: `debug`, `info`, `warn`, `error`, `fatal`, `off`. Any non-`off` value also attempts to enable firmware logs.

> **Version notes:** The `LLA` switch is supported only by Gemini 335Le firmware `1.7.05` and above and Gemini 435Le firmware `1.3.17` and above.
