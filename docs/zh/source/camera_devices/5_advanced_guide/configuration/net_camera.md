# 网络相机

> 本节介绍如何在 OrbbecSDK_ROS 中使用网络相机。目前仅支持 Femto Mega、Gemini 335Le 和 Gemini 435Le 设备，其他网络设备将在不久的将来得到支持。

您可以在 [example](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples) 中找到示例使用代码。

## Femto Mega & Gemini 435Le & Gemini 335Le

**参数介绍**

网络设备设置：`enumerate_net_device` 设置为 true，将自动枚举网络设备。

如果您不想自动枚举网络设备，可以将 `enumerate_net_device` 设置为 false，将 `net_device_ip` 设置为设备的 IP 地址，并将 `net_device_port` 设置为默认值 8090。

* `enumerate_net_device`：启用自动枚举网络设备。
* `net_device_ip`：设置网络设备的 IP 地址。
* `net_device_port`：设置网络设备的端口。通常可以设置为 8090。

**单个网络相机**

如果您需要运行 Gemini 435Le/Gemini 335Le，只需在运行命令中将 `femto_mega.launch` 替换为 `gemini435_le.launch` 或 `gemini_330_series.launch`。

以 `femto_mega.launch` 为例：

**自动枚举网络设备：**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**指定 IP 地址启动设备：**

注意：`net_device_ip` 需要更改为设备的 IP 地址，这里是 192.168.1.10。

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false net_device_ip:=192.168.1.10 net_device_port:=8090
```

**多个网络相机**

以 `multi_net_camera.launch` 为例：

```bash
roslaunch orbbec_camera multi_net_camera.launch
```

可以使用 `list_devices_node` 查看当前连接设备。v2.8.x 之后，该工具会额外输出固件版本、网口设备本地网卡名和 IP 来源类型（`NONE`、`LLA`、`DHCP`、`PERSISTENT`）。

## ip_config_tool 工具

**`ip_config_tool`** 可执行文件允许您直接从 ROS1 配置网络相机的 IP 设置，包括 DHCP、静态 IP、Force IP 和 DHCP 分配 IP 超时时间。这对于快速分配或更新 IP 地址而无需修改启动文件非常有用。

> **注意：** 通过 `dhcp` 或 `set_ip` 应用的配置会写入设备。`force_ip` 是临时强制 IP，设备断电或重启后需要重新应用。
> **兼容说明**：`set_device_ip` 已由 `ip_config_tool` 替代。旧参数 `old_ip` 已更名为 `current_ip`。

**示例用法**

查看帮助：

```bash
rosrun orbbec_camera ip_config_tool --help
```

开启 DHCP：

```bash
rosrun orbbec_camera ip_config_tool dhcp \
--current_ip 192.168.1.10 \
--enable_dhcp true
```

关闭 DHCP 并设置静态 IP：

```bash
rosrun orbbec_camera ip_config_tool set_ip \
--current_ip 192.168.1.10 \
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

**参数**

- **`current_ip`** - 设备的当前 IP 地址。
- **`enable_dhcp`** - 在 `dhcp` 或 `force_ip` 子命令中设置是否启用 DHCP。
- **`new_ip`** - 要分配的静态 IP 地址。
- **`mask`** - 新 IP 的子网掩码。
- **`gateway`** - 新 IP 的网关地址。
- **`force_ip_mac`** - Force IP 目标设备 MAC 地址。
- **`timeout`** / **`dhcp_assign_ip_timeout`** - DHCP 分配 IP 超时时间，单位为秒。

> **版本说明**：`LLA` 开关仅 Gemini 335Le 固件 `1.7.05` 及以上、Gemini 435Le 固件 `1.3.17` 及以上支持。

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
net_device_ip:=192.168.1.50 \
net_device_port:=8090
```

> 提示：在启用 Force IP 之前，请确认相机已连接且 MAC 地址正确。可使用 `list_devices_node` 查看所有已连接相机的 MAC 地址。
