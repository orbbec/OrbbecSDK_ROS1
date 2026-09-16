# GigE Action Command

This example starts two Gemini 335Le cameras in Group Actions synchronization mode and one
host-side Action Command sender. The sender is created once because one GVCP Action Command can
trigger multiple cameras.

## Requirements

- Gemini 335Le firmware 1.8.24 or later
- Orbbec SDK 2.10.2 or later
- Both cameras and the host on the same network

Pass the camera addresses on launch, or change the defaults in
`multi_gige_action_command.launch`:

```bash
roslaunch orbbec_camera multi_gige_action_command.launch \
  camera1_ip:=192.168.1.10 camera2_ip:=192.168.1.11
```

The launch file creates these services:

```text
/camera_01/get_action_config
/camera_01/set_action_config
/camera_02/get_action_config
/camera_02/set_action_config
/gige_action_command_node/send_action_command
```

Configure Action Signal block 0 on both cameras with matching keys and masks:

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

Read the configuration back when needed:

```bash
rosservice call /camera_01/get_action_config "selector: 0"
```

The service exposes three trigger modes. Every camera whose device key, group key, and group mask
match the request will be triggered.

### Immediate trigger

Set `trigger_mode` to `0`. The delay and scheduled time fields must be zero:

```bash
rosservice call /gige_action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 0
delay_ms: 0
scheduled_time: 0"
```

### Relative-delay trigger

Set `trigger_mode` to `1` and provide a positive delay in milliseconds. The node reads the host
system clock, adds the delay, and converts the result to the absolute GVCP/PTP timestamp expected by
the SDK. This example schedules the command one second in the future:

```bash
rosservice call /gige_action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 1
delay_ms: 1000
scheduled_time: 0"
```

The host `CLOCK_REALTIME` must be synchronized to the same PTP domain as the cameras, for example
by using `phc2sys`. The launch file enables camera PTP synchronization, but it does not configure
the host PTP services. Choose a delay long enough for the command to reach the cameras before its
target time.

### Absolute PTP-time trigger

Set `trigger_mode` to `2`, leave `delay_ms` at zero, and provide a future encoded PTP timestamp. The
upper 32 bits contain seconds and the lower 32 bits contain nanoseconds:

```bash
rosservice call /gige_action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
broadcast_ip: '255.255.255.255'
trigger_mode: 2
delay_ms: 0
scheduled_time: <PTP_TIMESTAMP>"
```

The response returns `encoded_scheduled_time`, the exact 64-bit value sent to the SDK. For delayed
triggering this is the timestamp calculated by the node. `success: true` means the host dispatched
the GVCP command; the protocol does not return a device acknowledgment.
