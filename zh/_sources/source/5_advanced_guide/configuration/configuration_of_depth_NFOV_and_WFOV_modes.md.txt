# 深度NFOV和WFOV模式配置

对于**Femto Mega**和**Femto Bolt**设备，NFOV和WFOV模式通过在启动文件中配置深度和红外的分辨率来实现。

在启动文件中，depth_width、depth_height、ir_width、ir_height分别表示深度的分辨率和红外的分辨率。

红外的帧率和分辨率必须与深度保持一致。不同模式与分辨率的对应关系如下：

- NFOV unbinned（未装箱）: 640 x 576
- NFOV binned（装箱）: 320 x 288
- WFOV unbinned（未装箱）: 1024 x 1024
- WFOV binned（装箱）: 512 x 512
