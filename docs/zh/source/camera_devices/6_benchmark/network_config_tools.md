# 网络配置工具

本节介绍网络相机 IP 配置工具。网络相机的启动、自动枚举、指定 IP 启动和相机节点 Force IP 参数见 [网络相机](../5_advanced_guide/configuration/net_camera.md)。

## ip_config_tool

`ip_config_tool` 用于直接从 ROS1 配置网络相机的 IP 设置，包括 DHCP、persistent IP、Force IP 和 DHCP 分配 IP 超时时间。它不需要先启动相机节点，适合快速分配或更新 IP 地址。

> **注意：** 通过 `set_ip` 应用的 DHCP / persistent IP 配置会写入设备。`force_ip` 是临时强制 IP，设备断电或重启后需要重新应用。
> **兼容说明**：`set_device_ip` 仍作为兼容别名保留，实际调用 `ip_config_tool`。旧参数 `old_ip` 已更名为 `current_ip`。

查看帮助：

```bash
rosrun orbbec_camera ip_config_tool --help
```

开启 DHCP：

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp true \
--enable_persistent_ip false
```

关闭 DHCP 并设置 persistent IP：

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp false \
--enable_persistent_ip true \
--new_ip 192.168.1.11 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

同时开启 DHCP 和 persistent IP（仅支持 IP 配置 V2 的设备/固件）：

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
--enable_dhcp true \
--enable_persistent_ip true \
--new_ip 192.168.1.11 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

通过 MAC 地址 Force IP：

```bash
rosrun orbbec_camera ip_config_tool force_ip \
--force_ip_mac 54:14:FD:06:07:DA \
--new_ip 192.168.1.50 \
--mask 255.255.255.0 \
--gateway 192.168.1.1
```

设置 DHCP 分配 IP 超时时间：

```bash
rosrun orbbec_camera ip_config_tool set_dhcp_timeout \
--current_ip 192.168.1.10 \
--timeout 10
```

## 参数

- **`current_ip`**：设备的当前 IP 地址。
- **`enable_dhcp`**：在 `set_ip` 或 `force_ip` 子命令中设置是否启用 DHCP。
- **`enable_persistent_ip`**：在 `set_ip` 子命令中设置是否启用 persistent IP。
- **`new_ip`**：要分配的 persistent IP 或 Force IP 地址。
- **`mask`**：新 IP 的子网掩码。
- **`gateway`**：新 IP 的网关地址。
- **`force_ip_mac`**：Force IP 目标设备 MAC 地址。
- **`timeout`** / **`dhcp_assign_ip_timeout`**：DHCP 分配 IP 超时时间，单位为秒。
- **`sdk_log_level`**：SDK 文件日志级别，可选值：`debug`、`info`、`warn`、`error`、`fatal`、`off`。非 `off` 时会同时尝试开启固件日志。

> **版本说明**：`LLA` 开关仅 Gemini 335Le 固件 `1.7.05` 及以上、Gemini 435Le 固件 `1.3.17` 及以上支持。
