# Introduction

OrbbecSDK ROS Wrapper provides seamless integration of Orbbec cameras with ROS environment. It supports ROS Kinetic, Melodic, and Noetic distributions.
By default, we recommend using the **v2-main** branch. For older OpenNI devices not supported by v2-main, please use the **main** branch. Device models that are only supported by the main branch are listed in the table below.

If you are a user in China, it is recommended to use [gitee Repo](https://gitee.com/orbbecdeveloper/OrbbecSDK_ROS1).

Here is the device support list of main branch (v1.x) and v2-main branch (v2.x):

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr style="background-color: #1f4e78; color: black; text-align: center;">
      <th>Product Series</th>
      <th>Product</th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/main" style="color: black; text-decoration: none;">Orbbec SDK v1.x</a></th>
      <th><a href="https://github.com/orbbec/OrbbecSDK_ROS1/tree/v2-main" style="color: black; text-decoration: none;">Orbbec SDK v2.x</a></th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 430</td>
      <td>Gemini 435Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 301</td>
      <td>Gemini 305</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Gemini 305g</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
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
      <td>Gemini 335Le</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 340</td>
      <td>Gemini 345Lg</td>
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
      <td>not supported</td>
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
      <td rowspan="2" style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Astra Mini S Pro</td>
      <td>full maintenance</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td>
      <td>Pulsar ME450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
    <tr>
      <td>Pulsar SL450</td>
      <td>not supported</td>
      <td>recommended for new designs</td>
    </tr>
  </tbody>
</table>

**Note**: If you do not find your device, please contact our FAE or sales representative for help.

**Definition**:

1. Recommended for new designs: we will provide full supports with new features,  bug fix and performance optimization;
2. Full maintenance: we will provide bug fix support;
3. Limited maintenance: we will provide critical bug fix support;
4. Not supported: we will not support specific device in this version;
5. To be supported: we will add support in the near future.

## Support Hardware Products

The following devices are supported by the OrbbecSDK ROS Wrapper v2-main branch.  More devices support will be added in the near future. If you can not find your device in the table below, try the [main](https://github.com/orbbec/OrbbecSDK_ROS1)  branch.

For optimal performance, we strongly recommend updating to the latest firmware version. This ensures that you benefit from the most recent enhancements and bug fixes.

The following devices are supported by the OrbbecSDK ROS Wrapper.

To learn how to obtain and upgrade the latest firmware, [please click here](../3_quickstarts/orbbecviewer.md).

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr>
      <th>Product Series</th>
      <th>Products List</th>
      <th>Recommended FW Version</th>
      <th>Launch File</th>
    </tr>
  </thead>
  <tbody>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 430</td>
      <td>Gemini 435Le</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemin435Le-Firmware">1.3.19</a></td>
      <td>gemini435_le.launch</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 301</td>
      <td>Gemini 305</td>
      <td><a href="https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Firmware%20Release%20of%20Gemini%20305">1.0.70</a></td>
      <td>gemini_301_series.launch</td>
    </tr>
    <tr>
      <td>Gemini 305g</td>
      <td><a href="https://doc.orbbec.com/documentation/Gemini%20305%20Documentation/Firmware%20Release%20of%20Gemini%20305">1.0.70</a></td>
      <td>gemini_301_series.launch</td>
    </tr>
    <tr>
      <td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td>
      <td>Gemini 335</td>
      <td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td>
      <td>gemini_330_series.launch</td>
    </tr>
    <tr><td>Gemini 336</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 330</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 335L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 336L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 330L</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 335Lg</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr><td>Gemini 335Le</td><td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Gemini330_Release_1.6.00.zip">1.6.00</a></td><td>gemini_330_series.launch</td></tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Gemini 340</td>
      <td>Gemini 345Lg</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini340-Firmware">1.9.03</a></td>
      <td>gemini345_lg.launch</td>
    </tr>
    <tr>
      <td rowspan="4" style="text-align: center; font-weight: bold;">Gemini 2</td>
      <td>Gemini 2</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2-Firmware">1.4.98</a></td>
      <td>gemini2.launch</td>
    </tr>
    <tr>
      <td>Gemini 2 L</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini2L-Firmware">1.5.2</a></td>
      <td>gemini2L.launch</td>
    </tr>
    <tr>
      <td>Gemini 215</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini215-Firmware">1.0.9</a></td>
      <td>gemini210.launch</td>
    </tr>
    <tr>
      <td>Gemini 210</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Gemini210-Firmware">1.0.9</a></td>
      <td>gemini210.launch</td>
    </tr>
    <tr>
      <td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Bolt-Firmware">1.1.2</a></td>
      <td>femto_bolt.launch</td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-Firmware">1.3.1</a></td>
      <td>femto_mega.launch</td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Femto-Mega-I-Firmware">2.0.4</a></td>
      <td>femto_mega.launch</td>
    </tr>
    <tr>
      <td style="text-align: center; font-weight: bold;">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Astra2_Release_2.8.20.zip">2.8.20</a></td>
      <td>astra2.launch</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">Astra Mini</td>
      <td>Astra Mini Pro</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-Pro">2.0.03</a></td>
      <td>astra.launch</td>
    </tr>
    <tr>
      <td>Astra Mini S Pro</td>
      <td><a href="https://github.com/orbbec/OrbbecFirmware/releases/tag/Astra-Mini-S-Pro">2.0.03</a></td>
      <td>astra.launch</td>
    </tr>
    <tr>
      <td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td>
      <td>Pulsar ME450</td>
      <td>1.0.0.6</td>
      <td>lidar.launch</td>
    </tr>
    <tr>
      <td>Pulsar SL450</td>
      <td>2.2.4.5</td>
      <td>lidar.launch</td>
    </tr>
  </tbody>
</table>

## URDF Model Support

<table border="1" style="border-collapse: collapse; text-align: left; width: 100%;">
  <thead>
    <tr>
      <th>Product Series</th>
      <th>Product</th>
      <th>URDF</th>
    </tr>
  </thead>
  <tbody>
    <tr><td style="text-align: center; font-weight: bold;">Gemini 430</td><td>Gemini 435Le</td><td>-</td></tr>
    <tr><td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 301</td><td>Gemini 305</td><td>gemini_305.urdf.xacro</td></tr>
    <tr><td>Gemini 305g</td><td>gemini_305_g.urdf.xacro</td></tr>
    <tr><td rowspan="8" style="text-align: center; font-weight: bold;">Gemini 330</td><td>Gemini 335</td><td>gemini_335_336.urdf.xacro</td></tr>
    <tr><td>Gemini 336</td><td>gemini_335_336.urdf.xacro</td></tr>
    <tr><td>Gemini 330</td><td>-</td></tr>
    <tr><td>Gemini 335L</td><td>gemini_335_L_336_L.urdf.xacro</td></tr>
    <tr><td>Gemini 336L</td><td>gemini_335_L_336_L.urdf.xacro</td></tr>
    <tr><td>Gemini 330L</td><td>-</td></tr>
    <tr><td>Gemini 335Lg</td><td>gemini_335_Lg.urdf.xacro</td></tr>
    <tr><td>Gemini 335Le</td><td>gemini_335_Le.urdf.xacro</td></tr>
    <tr><td rowspan="2" style="text-align: center; font-weight: bold;">Gemini 340</td><td>Gemini 345</td><td>gemini_345.urdf.xacro</td></tr>
    <tr><td>Gemini 345Lg</td><td>gemini_345_Lg.urdf.xacro</td></tr>
    <tr><td rowspan="4" style="text-align: center; font-weight: bold;">Gemini 2</td><td>Gemini 2</td><td>gemini_2.urdf.xacro</td></tr>
    <tr><td>Gemini 2 L</td><td>gemini_2_L.urdf.xacro</td></tr>
    <tr><td>Gemini 215</td><td>-</td></tr>
    <tr><td>Gemini 210</td><td>-</td></tr>
    <tr><td rowspan="3" style="text-align: center; font-weight: bold;">Femto</td><td>Femto Bolt</td><td>femto_bolt.urdf.xacro</td></tr>
    <tr><td>Femto Mega</td><td>-</td></tr>
    <tr><td>Femto Mega I</td><td>-</td></tr>
    <tr><td style="text-align: center; font-weight: bold;">Astra</td><td>Astra 2</td><td>astra_2.urdf.xacro</td></tr>
    <tr><td rowspan="2" style="text-align: center; font-weight: bold;">Astra Mini</td><td>Astra Mini Pro</td><td>-</td></tr>
    <tr><td>Astra Mini S Pro</td><td>-</td></tr>
    <tr><td rowspan="2" style="text-align: center; font-weight: bold;">LiDAR</td><td>Pulsar ME450</td><td>-</td></tr>
    <tr><td>Pulsar SL450</td><td>-</td></tr>
  </tbody>
</table>

All launch files are essentially similar, with the primary difference being the default values of the parameters set for different models within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch file, as well as other parameters, to ensure compatibility and optimal performance.



## Orbbec camera datasheet

Refer to the camera datasheet for more information.

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
  color: black;
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
      <td style="text-align: center;">Gemini 430</td>
      <td>Gemini 435Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/06/04011158/Orbbec-Gemini-435Le-Datasheet-V1.pdf">Orbbec Gemini 435Le Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;">Gemini 301</td>
      <td>Gemini 305</td>
      <td><a href="https://orbbec-debian-repos-aws.s3.amazonaws.com/product/Orbbec_Gemini%20305%20Datasheet%20V1.0_20260105.pdf">Orbbec Gemini 305 Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="6">Gemini 330</td>
      <td>Gemini 335</td>
      <td rowspan="4"><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/04/22062452/Gemini-330-series-Datasheet-V1.6.pdf">Gemini 330 Series Datasheet for USB Devices</a></td>
    </tr>
    <tr><td>Gemini 336</td></tr>
    <tr><td>Gemini 335L</td></tr>
    <tr><td>Gemini 336L</td></tr>
    <tr>
      <td>Gemini 335Lg</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2024/10/22030914/Gemini-335Lg-Datasheet-V1.0-241022.pdf">Gemini 330 Series Datasheet for GMSL Devices</a></td>
    </tr>
    <tr>
      <td>Gemini 335Le</td>
      <td><a href="https://new-orbbec3d-s3.s3.amazonaws.com/wp-content/uploads/2025/03/24023151/Orbbec-Gemini-335Le-Datasheet-V1-2.pdf">Gemini 330 Series Datasheet for Ethernet Devices</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Gemini 2</td>
      <td>Gemini 2</td>
      <td rowspan="2"><a href="https://xm917ch2uk.feishu.cn/file/Khxfb2vdioUghexIMqJcAyL3nXf">Orbbec Gemini 2 Series Datasheet</a></td>
    </tr>
    <tr><td>Gemini 2 L</td></tr>
    <tr>
      <td>Gemini 2 XL</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/QW2vbNvwxoocRIxSL6Zcvut2npS">Orbbec Gemini 2 XL Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Femto</td>
      <td>Femto Bolt</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2024/08/ORBBEC_Datasheet_Femto-Bolt-v1.0.pdf">Orbbec Femto Bolt Datasheet</a></td>
    </tr>
    <tr>
      <td>Femto Mega</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Femto-Mega1.pdf">Orbbec Femto Mega Datasheet</a></td>
    </tr>
    <tr>
      <td>Femto Mega I</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/08/ORBBEC_Datasheet_Femto-Mega-I.pdf">Orbbec Femto Mega I Datasheet</a></td>
    </tr>
    <tr>
      <td style="text-align: center;" rowspan="3">Astra</td>
      <td>Astra 2</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-2_V1.2.pdf">Orbbec Astra 2 Datasheet</a></td>
    </tr>
    <tr>
      <td>Astra+</td>
      <td><a href="https://xm917ch2uk.feishu.cn/file/Qk0zbx26Doh8XMxw0rIcOgQYnff">Orbbec Astra+ Datasheet</a></td>
    </tr>
    <tr>
      <td>Astra Mini Pro</td>
      <td><a href="https://d1cd332k3pgc17.cloudfront.net/wp-content/uploads/2023/04/ORBBEC_Datasheet_Astra-Mini-Pro-1.pdf">Orbbec Astra Mini Pro Datasheet</a></td>
    </tr>
  </tbody>
</table>

---


## Support Platforms

- Linux x64: tested on Ubuntu 20.04
- Linux ARM64: tested on NVIDIA Jetson AGX Orin , NVIDIA Jetson Orin NX , NVIDIA Jetson Orin Nano , NVIDIA Jetson AGX Xavier , NVIDIA Jetson Xavier NX
