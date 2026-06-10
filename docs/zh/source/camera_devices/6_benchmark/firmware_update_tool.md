# firmware_update_tool 设备维护工具

`firmware_update_tool` 用于从 ROS1 命令行升级设备固件或烧录 preset 文件。升级前请确认设备连接稳定；多设备连接时建议指定序列号，避免升级到错误设备。

查看帮助：

```bash
rosrun orbbec_camera firmware_update_tool --help
```

升级单个设备固件：

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--firmware_path /path/to/firmware.bin
```

烧录 preset 文件：

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--preset_path /path/to/preset.bin
```

批量升级多个设备时，`--serial_number` 支持逗号分隔；如希望某个设备失败后继续处理后续设备，可增加 `--continue_on_error`。

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number SN1,SN2 \
--firmware_path /path/to/firmware.bin \
--continue_on_error
```

如需开启 SDK 文件日志并同时尝试开启固件日志，可增加 `--sdk_log_level`。可选值为 `debug`、`info`、`warn`、`error`、`fatal`、`off`，默认 `off`。

```bash
rosrun orbbec_camera firmware_update_tool \
--serial_number <SN> \
--preset_path /path/to/preset.bin \
--sdk_log_level debug
```

旧启动参数 `upgrade_firmware` 和 `preset_firmware_path` 仍会兼容解析，但推荐使用 `--firmware_path` 和 `--preset_path`。
