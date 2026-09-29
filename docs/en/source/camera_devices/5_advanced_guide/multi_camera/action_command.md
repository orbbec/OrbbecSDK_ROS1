# Action Command

This example starts two Gemini 335Le cameras and one host-side Action Command sender to trigger multiple cameras through an Action Command.

## Requirements

- Gemini 335Le firmware `1.8.24` or later;
- Orbbec SDK `2.10.2` or later;
- both cameras and the host on the same network.

## Launch

Specify the IP addresses of the two cameras:

```bash
roslaunch orbbec_camera multi_action_command.launch \
  camera1_ip:=192.168.1.10 camera2_ip:=192.168.1.11
```

The launch file provides these services:

```text
/camera_01/get_action_config
/camera_01/set_action_config
/camera_02/get_action_config
/camera_02/set_action_config
/action_command_node/send_action_command
```

## Configure the Action Signal

Configure Action Signal block 0 on both cameras with the same device key, group key, and group mask:

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

Read the configuration back with:

```bash
rosservice call /camera_01/get_action_config "selector: 0"
```

Every camera whose device key, group key, and group mask match the request will be triggered.

## Send an Action Command

The `/action_command_node/send_action_command` service uses `orbbec_camera/SendActionCommand`. The trigger modes are:

| `trigger_mode` | Mode | Requirements |
| --- | --- | --- |
| `0` | Immediate | `delay_ms` and `scheduled_time` must be `0`. |
| `1` | Relative delay | `delay_ms` must be greater than `0`; the node calculates the target PTP time from the host clock. |
| `2` | Absolute PTP time | `delay_ms` must be `0`, and a future `scheduled_time` must be provided. |

When `broadcast_ip` is empty, the node uses `255.255.255.255`; an IPv4 broadcast address can also be specified.

### Immediate trigger

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

### Relative-delay trigger

Set `trigger_mode` to `1`. This example triggers the cameras one second later:

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

The node reads the host `CLOCK_REALTIME`, adds `delay_ms`, and converts the result to the absolute GVCP/PTP time used by the SDK. The host clock must be synchronized to the same PTP domain as the cameras, for example with `phc2sys`. The launch file configures camera PTP but does not configure host PTP services; use a delay long enough for the command to reach the cameras before the target time.

### Absolute PTP-time trigger

Set `trigger_mode` to `2`, keep `delay_ms` at `0`, and provide a future encoded PTP timestamp. The upper 32 bits of `scheduled_time` contain seconds and the lower 32 bits contain nanoseconds:

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

The response field `encoded_scheduled_time` is the exact 64-bit value sent to the SDK; for relative-delay triggering, it is the target time calculated by the node. `success: true` means that the host dispatched the GVCP command to the SDK; the protocol does not return a device acknowledgment.
