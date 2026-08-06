# 网络相机

> 本节介绍如何在 OrbbecSDK_ROS 中使用网络相机。

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples) 中找到示例使用代码。

## Femto Mega & Gemini 435Le & Gemini 335Le & Gemini 338Le

**参数介绍**

网络设备设置：`enumerate_net_device` 设置为 true，将自动枚举网络设备。

如果您不想自动枚举网络设备，可以将 `enumerate_net_device` 设置为 false，将 `ip_address` 设置为设备的 IP 地址，并将 `port` 设置为默认值 8090。

* `enumerate_net_device`：启用自动枚举网络设备。
* `ip_address`：设置网络设备的 IP 地址。
* `port`：设置网络设备的端口。通常可以设置为 8090。

**单个网络相机**

如果您需要运行 Gemini 435Le、Gemini 335Le 或 Gemini 338Le，只需在运行命令中将 `femto_mega.launch` 替换为 `gemini435_le.launch` 或 `gemini_330_series.launch`。

以 `femto_mega.launch` 为例：

**自动枚举网络设备：**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**指定 IP 地址启动设备：**

注意：`ip_address` 需要更改为设备的 IP 地址，这里是 192.168.1.10。

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false ip_address:=192.168.1.10 port:=8090
```

**多个网络相机**

以 `multi_net_camera.launch` 为例：

```bash
roslaunch orbbec_camera multi_net_camera.launch
```

可以使用 `list_devices_node` 查看当前连接设备。v2.8.x 之后，该工具会额外输出固件版本、preset 列表、preset 版本、网口设备本地网卡名和 IP 来源类型（`NONE`、`LLA`、`DHCP`、`PERSISTENT`）。当某个设备枚举失败时，工具会继续枚举其他设备。

如需开启 SDK 和固件日志，可添加 `--sdk_log_level debug`：

```bash
rosrun orbbec_camera list_devices_node --sdk_log_level debug
```

## IP 配置工具

如需直接配置网络相机的 DHCP、persistent IP、Force IP 或 DHCP 分配 IP 超时时间，请使用 `ip_config_tool`。完整命令示例和参数说明见 [网络配置工具](../../6_benchmark/network_config_tools.md)。

## 强制 IP 功能

**强制 IP** 功能允许您为网络相机分配**静态 IP 地址**，覆盖 DHCP 设置。当连接多个网络相机时，这非常有用，您需要每个设备具有固定的 IP 以实现可靠的通信。

> **注意：** 如果设备断电或重启，强制 IP 配置**将被重置**。您需要在重启后重新应用设置。

**参数**

- **`force_ip_enable`** - 启用强制 IP 功能。**默认值：** `false`
- **`force_ip_mac`** - 连接多个相机时的目标设备 MAC 地址（例如 `"54:14:FD:06:07:DA"`）。您可以使用 `list_devices_node` 查找每个设备的 MAC。**默认值：** `""`
- **`force_ip_address`** - 要分配的静态 IP 地址。**默认值：** `192.168.1.10`
- **`force_ip_subnet_mask`** - 静态 IP 的子网掩码。**默认值：** `255.255.255.0`
- **`force_ip_gateway`** - 静态 IP 的网关地址。**默认值：** `192.168.1.1`

**示例用法**

- **为特定设备启用强制 IP：**

```bash
roslaunch orbbec_camera gemini_330_series.launch \
force_ip_enable:=true \
force_ip_mac:=54:14:FD:06:07:DA \
force_ip_address:=192.168.1.50 \
force_ip_subnet_mask:=255.255.255.0 \
force_ip_gateway:=192.168.1.1 \
ip_address:=192.168.1.50 \
port:=8090
```

> 提示：在启用 Force IP 之前，请确认相机已连接且 MAC 地址正确。可使用 `list_devices_node` 查看所有已连接相机的 MAC 地址。
