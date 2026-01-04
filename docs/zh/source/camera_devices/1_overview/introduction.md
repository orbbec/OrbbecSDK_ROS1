# 介绍

OrbbecSDK ROS包装器提供了奥比中光相机与ROS环境的无缝集成。它支持ROS Kinetic、Melodic和Noetic发行版。
默认情况下，我们推荐使用**v2-main**分支。对于v2-main不支持的较旧OpenNI设备，请使用**main**分支。仅支持main分支的设备型号列在下表中。

如果您是中国用户，建议使用[gitee仓库](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS1)。

以下是main分支（v1.x）和v2-main分支（v2.x）的设备支持列表：

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: white; text-align: center;">
      <th>产品系列</th>
      <th>产品</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/main" style="color: black; text-decoration: none;">main分支</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS2/tree/v2-main" style="color: black; text-decoration: none;">v2-main分支</a></th>
    </tr>
  </thead>
  <tbody>
      <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 336L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 330L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="5" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td>recommended for new designs</td>
      <td>to be supported</td>
    </tr>
    <tr>
      <td>Gemini 215</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td>Astra Pro Plus</td>
      <td>limited maintenance</td>
      <td>not supported</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini (S) Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
  </tbody>
</table>

**注意**：如果您没有找到您的设备，请联系我们的现场应用工程师（FAE）或销售代表寻求帮助。

**定义**：

1. 推荐用于新设计：我们将提供全面支持，包括新功能、错误修复和性能优化；
2. 完全维护：我们将提供错误修复支持；
3. 有限维护：我们将提供关键错误修复支持；
4. 不支持：我们不会在此版本中支持特定设备；
5. 待支持：我们将在不久的将来添加支持。

## 支持的硬件产品

以下设备由OrbbecSDK ROS包装器v2-main分支支持。更多设备支持将在不久的将来添加。如果您在下表中找不到您的设备，请尝试使用[main](https://github.com/orbbec/OrbbecSDK_ROS1)分支。

为了获得最佳性能，我们强烈建议更新到最新的固件版本。这确保您能受益于最新的增强功能和错误修复。

以下设备由OrbbecSDK ROS包装器支持。

要了解如何获取和升级最新固件，[请点击这里](../3_quickstarts/orbbecviewer.md)。

| **Products List** | **Recommended FW Version**                                                             |  Launch File                                          |
| ----------------------- | -------------------------------------------------------------------------------------------- | --------------------------------------------- |
| Astra Mini Pro           | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro)             |                                               astra.launch   |
| Astra Mini S Pro            | [2.0.03](https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro)             |                                               astra.launch   |
| Gemini 435Le            | [1.3.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware)             |                                               gemini435_le.launch   |
| Gemini 330 series       | [1.6.00](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip) | gemini_330_series.launch |
| Gemini 215              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware)               |                                               gemini210.launch   |
| Gemini 210              | [1.0.9](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware)               |                                               gemini210.launch    |
| Gemini 2                | [1.4.98](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware)                |                                               gemini2.launch     |
| Gemini 2 L              | [1.5.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware)                |                                               gemini2L.launch  |
| Femto Bolt              | [1.1.2](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware)              |                               femto_bolt.launch |
| Femto Mega              | [1.3.1](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware)              |                                               femto_mega.launch  |
| Femto Mega I             | [2.0.4](https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware)              |                                               femto_mega.launch  |
| Astra 2                 | [2.8.20](https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip)    |                                               astra2.launch    |

所有启动文件本质上都是相似的，主要区别在于为同一系列中的不同型号设置的参数默认值。USB标准的差异（如USB 2.0与USB 3.0）可能需要调整这些参数。如果遇到启动失败，请仔细查阅规格说明书。特别注意启动文件中的分辨率设置以及其他参数，以确保兼容性和最佳性能。



## 奥比中光相机数据手册

请参考相机数据手册获取更多信息。

<style>
table {
  border-collapse: collapse;
  width: 100%;
}
th, td {
  border: 1px solid #ccc;
  padding: 8px;
  text-align: left;
  vertical-align: middle;
}
thead th {
  background-color: #1f4e78;
  color: white;
  text-align: center;
  vertical-align: middle;
}
</style>

<table>
  <thead>
    <tr>
      <th>Product Series</th>
      <th>Product</th>
      <th>Datasheet</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center;">Gemini 435Le</td>
      <td>Gemini 435Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/06/04011158/Orbbec-Gemini-435Le-Datasheet-V1.pdf">Orbbec Gemini 435Le数据手册</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="6">Gemini 330</td>
      <td>Gemini 335</td>
      <td rowspan="4"><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/04/22062452/Gemini-330-series-Datasheet-V1.6.pdf">Gemini 330系列USB设备数据手册</a></td>
    </tr>
    <tr><td>Gemini 336</td></tr>
    <tr><td>Gemini 335L</td></tr>
    <tr><td>Gemini 336L</td></tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2024/10/22030914/Gemini-335Lg-Datasheet-V1.0-241022.pdf">Gemini 330系列GMSL设备数据手册</a></td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/03/24023151/Orbbec-Gemini-335Le-Datasheet-V1-2.pdf">Gemini 330系列以太网设备数据手册</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Gemini 2</td>
      <td>Gemini 2</td>
      <td rowspan="2"><a href="https://xm917ch2uk.feishu.cn/file/Khxfb2vdioUghexIMqJcAyL3nXf">Orbbec Gemini 2系列数据手册</a></td>
    </tr>
    <tr><td>Gemini 2 L</td></tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/QW2vbNvwxoocRIxSL6Zcvut2npS">Orbbec Gemini 2 XL数据手册</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/08/ORBBEC_Datasheet_Femto-Bolt-v1.0.pdf">Orbbec Femto Bolt数据手册</a></td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Femto-Mega1.pdf">Orbbec Femto Mega数据手册</a></td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/08/ORBBEC_Datasheet_Femto-Mega-I.pdf">Orbbec Femto Mega I数据手册</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-2_V1.2.pdf">Orbbec Astra 2数据手册</a></td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/Qk0zbx26Doh8XMxw0rIcOgQYnff">Orbbec Astra+数据手册</a></td>
    </tr>
    <tr>
      <td>Astra Mini Pro</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-Mini-Pro-1.pdf">Orbbec Astra Mini Pro数据手册</a></td>
    </tr>
  </tbody>
</table>

---


## 支持平台

- Linux x64：在Ubuntu 20.04上测试
- Linux ARM64：在NVIDIA Jetson AGX Orin、NVIDIA Jetson Orin NX、NVIDIA Jetson Orin Nano、NVIDIA Jetson AGX Xavier、NVIDIA Jetson Xavier NX上测试
