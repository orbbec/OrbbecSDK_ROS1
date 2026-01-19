#Device default

This document lists available presets, features, and recommended scenarios by product. Please select a preset name that matches the product model and set it as the value of the `device_preset` parameter.

## Gemini 330 / Gemini 330L / Gemini 335 / Gemini 335L

| Defaults | Features | Recommended usage scenarios |
| --- | --- | --- |
| Default | Best visual effect<br> Overall performance is good, including typical indicators such as accuracy, filling rate, small object detection capability | General scenarios<br> Robot application |
| Hand | Clear hand and finger edges | Gesture recognition |
| High Accuracy | Highly reliable depth information<br> Very little depth noise<br> Relatively low depth fill rate | Obstacle avoidance<br> Object scanning |
| High Density | Higher depth filling rate<br> Can detect more small objects<br> More susceptible to depth noise | Object recognition<br> Grabbing<br> Foreground or background processing, such as cutout, etc. |
| Medium Density | Balanced depth fill rate and precision performance<br> Compared with default settings: relatively low fill rate, better edge quality | Universal scene, alternative to Default |
| Custom | Custom modifications, such as new configurations of post-processing pipelines, modifications to depth AE functions | By adjusting the depth configuration yourself, you can achieve better results than the predefined depth preset configuration<br> Fully proven customized depth configuration |

## Gemini 336 / Gemini 336L

| Defaults | Features |
| --- | --- |
| Default | The best visual effect<br> The overall performance is good, including typical indicators such as accuracy, filling rate, and small object detection capabilities |
| High Accuracy | Highly reliable depth information<br> Very little depth noise<br> Relatively low depth fill rate |
| Custom | Custom modifications, such as new configurations of post-processing pipelines and modifications to deep AE functions |

## Gemini 2

| Defaults | Features |
| --- | --- |
| Unbinned Dense Default | Depth Quality Priority |
| Binned Sparse Default | Small dead zone, low power consumption, high frame rate |
| Obstacle Avoidance | Robot obstacle avoidance mode |

## Gemini 2L

| Defaults | Features |
| --- | --- |
| Unbinned Dense Default | Depth Quality Priority |
| Dimensioning | Measurement accuracy is priority |
| Binned Sparse Default | Small dead zone, low power consumption, high frame rate |
| Unbinned Sparse Default | Balance quality and power consumption, improve low-reflection and semi-outdoor effects |

## Gemini 305

| Defaults | Features |
| --- | --- |
| Default | Best visual perception<br> Good overall performance in terms of accuracy, filling rate, small objects, etc. |
| High Accuracy | High confidence depth values<br> Virtually noiseless depth values<br> Low fill rate |
| Close Range High Accuracy | Adjustable larger parallax search range, Mini-Z reduction<br> The rest of the performance remains consistent with High Accuracy |
| Dual Color Streams | Supports left and right color streams at the same time, no depth IR information<br> The left and right output effects are consistent |
| Custom | Custom modifications, such as new configurations of post-processing pipelines and modifications to deep AE functions |
> Since the parameter configuration of `Dual Color Streams` mode is quite different from that of `Default` mode, we provide the corresponding YAML configuration file.
Please set `config_file_path` to `gemini305_dual_color.yaml`, the configuration file is located in the config directory.

## Extended presets

### G33X Close Range High Accuracy (Gemini 330 / Gemini 335 / Gemini 336)

| Defaults | Features | Recommended usage scenarios |
| --- | --- | --- |
| G33X Close Range High Accuracy | For the selected depth resolution, the depth value at the closest working distance will be reduced by 50%<br> The new closest working distance is 0.13m when close application is enabled<br> Supported depth resolutions: 1280x800/1280x720/640x400/424x266 only | Eye in hand and close range operation tasks<br> Close range measurement |

### G336X AMR Default (Gemini 336 / Gemini 336L)

| Defaults | Features | Recommended usage scenarios |
| --- | --- | --- |
| G336X AMR Default | Optimized for reliable depth performance in sunlit and repetitively textured warehouse scenes<br> Recommended depth resolutions: 1280x800 / 640x400 / 424x266 | Warehouse AMRs |