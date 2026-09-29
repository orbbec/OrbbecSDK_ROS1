# Action Command

本示例启动两台 Gemini 335Le 相机和一个主机侧 Action Command 发送节点，通过 Action Command 触发多台相机同步采集。

## 环境要求

- Gemini 335Le 固件版本 `1.8.24` 或更高；
- Orbbec SDK 版本 `2.10.2` 或更高；
- 两台相机和主机位于同一网络。

## 启动

指定两台相机的 IP 地址：

```bash
roslaunch orbbec_camera multi_action_command.launch \
  camera1_ip:=192.168.1.10 camera2_ip:=192.168.1.11
```

启动后提供以下服务：

```text
/camera_01/get_action_config
/camera_01/set_action_config
/camera_02/get_action_config
/camera_02/set_action_config
/action_command_node/send_action_command
```

## 配置 Action Signal

在两台相机上为 Action Signal block 0 配置相同的 device key、group key 和 group mask：

```bash
rosservice call /camera_01/set_action_config \
  "device_key: 1
selector: 0
group_key: 1
group_mask: 1"

rosservice call /camera_02/set_action_config \
  "device_key: 1
selector: 0
group_key: 1
group_mask: 1"
```

使用以下命令读取配置：

```bash
rosservice call /camera_01/get_action_config "selector: 0"
```

发送请求中的 device key、group key 和 group mask 与相机配置匹配时，对应相机会被触发。

## 发送 Action Command

`/action_command_node/send_action_command` 服务使用 `orbbec_camera/SendActionCommand` 类型。触发模式如下：

| `trigger_mode` | 模式 | 参数要求 |
| --- | --- | --- |
| `0` | 立即触发 | `delay_ms` 和 `scheduled_time` 必须为 `0`。 |
| `1` | 相对延迟触发 | `delay_ms` 必须大于 `0`，节点根据主机时间计算目标 PTP 时间。 |
| `2` | 绝对 PTP 时间触发 | `delay_ms` 必须为 `0`，并提供未来的 `scheduled_time`。 |

`broadcast_ip` 为空时使用 `255.255.255.255`，也可以指定 IPv4 广播地址。

### 立即触发

```bash
rosservice call /action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 0
delay_ms: 0
scheduled_time: 0"
```

### 相对延迟触发

设置 `trigger_mode` 为 `1`，例如延迟 1 秒触发：

```bash
rosservice call /action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 1
delay_ms: 1000
scheduled_time: 0"
```

节点读取主机 `CLOCK_REALTIME`，加上 `delay_ms` 后转换为 SDK 使用的绝对 GVCP/PTP 时间。主机时钟必须与相机处于同一 PTP 域，例如使用 `phc2sys` 进行同步。启动文件只配置相机 PTP，不会配置主机 PTP 服务；应设置足够大的延迟，确保命令在目标时间前到达相机。

### 绝对 PTP 时间触发

设置 `trigger_mode` 为 `2`，`delay_ms` 保持为 `0`，并传入未来的编码 PTP 时间戳。`scheduled_time` 的高 32 位为秒，低 32 位为纳秒：

```bash
rosservice call /action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 2
delay_ms: 0
scheduled_time: <PTP_TIMESTAMP>"
```

响应中的 `encoded_scheduled_time` 是实际发送给 SDK 的 64 位时间值；相对延迟触发时，该字段为节点计算出的目标时间。`success: true` 只表示主机已将 GVCP 命令发送给 SDK，协议不会返回设备确认。
