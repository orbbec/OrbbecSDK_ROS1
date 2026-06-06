# firmware_update_tool

`firmware_update_tool` upgrades device firmware or writes preset files from the ROS1 command line. Before upgrading, make sure the device connection is stable. When multiple devices are connected, specify the serial number to avoid updating the wrong device.

Show help:

```bash
rosrun orbbec_camera firmware_update_tool --help
```

Upgrade firmware for one device:

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

Write a preset file:

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

For batch updates, `--serial_number` accepts comma-separated values. Add `--continue_on_error` if later devices should still be processed after one device fails.

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
