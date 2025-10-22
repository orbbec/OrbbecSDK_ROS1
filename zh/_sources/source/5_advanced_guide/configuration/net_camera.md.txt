# 网络相机

> 本节描述如何在OrbbecSDK_ROS中使用网络相机。目前仅支持Femto_Mega和Gemini 335Le设备，其他网络设备将在不久的将来得到支持。

您可以在[示例](https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main/examples)中找到示例使用代码。

## Femto Mega

**参数介绍**

网络设备设置：默认`enumerate_net_device`设置为true，这将自动枚举网络设备。

如果您不想自动枚举网络设备，可以将`enumerate_net_device`设置为false，将`ip_address`设置为设备的IP地址，将`port`设置为默认值8090。

* `enumerate_net_device` : 启用自动枚举网络设备。
* `ip_address` : 设置网络设备的IP地址。
* `port` : 设置网络设备的端口。通常，您可以将其设置为8090。

**单个网络相机**

如果您需要运行Gemini 435Le/Gemini 335Le，只需在运行命令中将femto_mega.launch替换为gemini435_le.launch/gemini335_le.launch。

以femto_mega.launch为例：

**自动枚举网络设备：**

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=true
```

**指定IP地址启动设备：**

注意：`ip_address`需要更改为设备的IP地址，这里是192.168.1.10

```bash
roslaunch orbbec_camera femto_mega.launch enumerate_net_device:=false ip_address:=192.168.1.10 port:=8090
```

**多网络相机**

以multi_net_camera.launch为例：

```bash
roslaunch orbbec_camera multi_net_camera.launch
```

## set_device_ip实用程序

**`set_device_ip`**可执行文件允许您直接从ROS 2配置网络相机的IP设置，包括在DHCP和静态IP之间切换，以及设置子网掩码和网关。这对于快速分配或更新IP地址而无需修改启动文件很有用。

> **注意：** 使用`set_device_ip`应用的IP设置是**永久的**，**不会重置**，即使设备断电或重启。

**使用示例**

```bash
rosrun orbbec_camera set_device_ip \
_old_ip:=192.168.1.10 \
_dhcp:=false \
_new_ip:=192.168.1.11 \
_mask:=255.255.255.0 \
_gateway:=192.168.1.1
```

**参数**

- **`old_ip`** – 设备的当前IP地址。
- **`dhcp`** – 设置为`true`使用DHCP或`false`使用静态IP。
- **`new_ip`** – 禁用DHCP时分配的静态IP地址。
- **`mask`** – 新IP的子网掩码。
- **`gateway`** – 新IP的网关地址。

## 强制IP功能

**强制IP**功能允许您为网络相机分配**静态IP地址**，覆盖DHCP设置。当连接多个网络相机时，这很有用，您需要每个设备都有固定的IP以实现可靠的通信。

> **注意：** 强制IP配置**如果设备断电或重启将会重置**。您需要在重启后重新应用设置。

**参数**

- **`force_ip_enable`** – 启用强制IP功能。**默认：** `false`
- **`force_ip_mac`** – 连接多个相机时的目标设备MAC地址（例如，`"54:14:FD:06:07:DA"`）。您可以使用`list_devices_node`找到每个设备的MAC。**默认：** `""`
- **`force_ip_address`** – 要分配的静态IP地址。**默认：** `192.168.1.10`
- **`force_ip_subnet_mask`** – 静态IP的子网掩码。**默认：** `255.255.255.0`
- **`force_ip_gateway`** – 静态IP的网关地址。**默认：** `192.168.1.1`

**使用示例**

- **为特定设备启用强制IP：**

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

> 提示：在启用强制IP之前，确保相机已连接且其MAC地址正确。使用`list_devices_node`检查所有已连接相机的MAC地址。
