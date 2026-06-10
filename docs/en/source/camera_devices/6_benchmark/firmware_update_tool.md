# firmware_update_tool Device Maintenance Tool

`firmware_update_tool` updates device firmware or burns preset files from the ROS1 command line. Before updating, make sure the device connection is stable. When multiple devices are connected, specify serial numbers to avoid updating the wrong device.

Show help:

```bash
rosrun orbbec_camera firmware_update_tool --help
```

Update firmware for one device:

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

Burn a preset file:

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

For batch updates, `--serial_number` accepts comma-separated serial numbers. Add `--continue_on_error` if later devices should still be processed after one device fails.

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```

To enable SDK file logs and also attempt to enable firmware logs, add `--sdk_log_level`. Optional values are `debug`, `info`, `warn`, `error`, `fatal`, and `off`; the default is `off`.

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--preset_path /path/to/preset.bin \
--sdk_log_level debug
```

The old launch arguments `upgrade_firmware` and `preset_firmware_path` are still parsed for compatibility, but `--firmware_path` and `--preset_path` are recommended.
