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

Send an immediate broadcast command. Every camera whose device key, group key, and group mask
match the request will be triggered:

```bash
rosservice call /gige_action_command_node/send_action_command \
  "device_key: 1
group_key: 1
group_mask: 1
destination_ip: '255.255.255.255'
scheduled_time: 0"
```

`success: true` means the host dispatched the GVCP command; the protocol does not return a device
acknowledgment. A nonzero `scheduled_time` uses a GVCP/PTP timestamp, with seconds in the upper
32 bits and nanoseconds in the lower 32 bits. Scheduled triggering requires synchronized camera
and host clocks.
