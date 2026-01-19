# 设备预设

本文档按产品列出可用预设、特点与推荐场景。请选择匹配产品型号的预设名称，并将其设置为`device_preset`参数的值。

## Gemini 330 / Gemini 330L / Gemini 335 / Gemini 335L

| 预设 | 特点 | 推荐使用场景 |
| --- | --- | --- |
| Default |  最佳视觉效果<br> 整体性能较好，包括精度、填充率、小物体检测能力等典型指标 |  通用场景<br> 机器人应用 |
| Hand |  清晰的手和手指边缘 |  手势识别 |
| High Accuracy |  高可靠的深度信息<br> 极少的深度噪声<br> 相对较低的深度填充率 |  避障<br> 物体扫描 |
| High Density |  更高的深度填充率<br> 可检测到更多的细小物体<br> 更容易受到深度噪声影响 |  物体识别<br> 抓取<br> 前景或背景处理，如抠图等 |
| Medium Density |  平衡的深度填充率和精度表现<br> 与默认设置相比：填充率相对较低，边缘质量更好 |  通用场景，可替代Default |
| Custom |  自定义修改，例如后处理管道的新配置、深度AE功能的修改 |  通过自行调整深度配置，可获得比预定义深度预置配置更好的效果<br> 经过充分验证的定制化深度配置 |

## Gemini 336 / Gemini 336L

| 预设 | 特点 |
| --- | --- |
| Default |  最佳视觉效果<br> 整体性能较好，包括精度、填充率、小物体检测能力等典型指标 |
| High Accuracy |  高可靠的深度信息<br> 极少的深度噪声<br> 相对较低的深度填充率 |
| Custom |  自定义修改，例如后处理管道的新配置、深度AE功能的修改 |

## Gemini 2

| 预设 | 特点 |
| --- | --- |
| Unbinned Dense Default |  深度质量优先 |
| Binned Sparse Default |  小盲区、低功耗、高帧率 |
| Obstacle Avoidance |  机器人避障模式 |

## Gemini 2L

| 预设 | 特点 |
| --- | --- |
| Unbinned Dense Default |  深度质量优先 |
| Dimensioning |  测量精度优先 |
| Binned Sparse Default |  小盲区、低功耗、高帧率 |
| Unbinned Sparse Default |  平衡质量和功耗，提升低反和半室外效果 |

## Gemini 305

| 预设 | 特点 |
| --- | --- |
| Default |  最佳视觉感知<br> 在精度、填充率、微小物体等方面整体性能良好 |
| High Accuracy |  高置信度深度值<br> 几乎无噪声深度值<br> 填充率较低 |
| Close Range High Accuracy |  可调更大视差搜索范围，Mini-Z减小<br> 其余表现与High Accuracy保持一致 |
| Dual Color Streams |  支持左右彩色同时出流，无深度IR信息<br> 左右路输出效果一致 |
| Custom |  自定义修改，例如后处理管道的新配置、深度AE功能的修改 |
> 由于 `Dual Color Streams` 模式的参数配置与 `Default` 模式差异较大，我们提供了对应的 YAML 配置文件。
请将 `config_file_path` 设置为 `gemini305_dual_color`，该配置文件位于 config 目录下。

## 扩展预设

### G33X Close Range High Accuracy（Gemini 330 / Gemini 335 / Gemini 336）

| 预设 | 特点 | 推荐使用场景 |
| --- | --- | --- |
| G33X Close Range High Accuracy |  对于选定的深度分辨率，最近工作距离的深度值将减小50%<br> 启用近距应用时新的最近工作距离为0.13m<br> 支持的深度分辨率：仅限1280x800/1280x720/640x400/424x266 |  眼在手上和近距离操作任务<br> 近距离测量 |

### G336X AMR Default（Gemini 336 / Gemini 336L）

| 预设 | 特点 | 推荐使用场景 |
| --- | --- | --- |
| G336X AMR Default |  优化其在阳光照射和重复纹理的仓库场景中具有可靠的深度性能<br> 推荐深度分辨率：1280x800 / 640x400 / 424x266 |  仓库AMRs |


