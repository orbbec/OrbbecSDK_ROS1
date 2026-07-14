## Aligning Depth to Color

This section explains how to align depth images with color images to create an overlay image using ROS. This is particularly useful for applications requiring synchronized visual information from different sensor modalities.

### Commands to Align and View Depth and Color Images

1. **Basic Depth to Color Alignment:**

   To simply align the depth image to the color image, use the following command:

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true
   ```

   This command activates the depth registration feature without opening a viewer.
2. **Viewing Depth to Color Overlay:**

   If you wish to view the depth to color overlay, you need to enable the viewer by using the command below:

   ```bash
   roslaunch orbbec_camera gemini_330_series.launch depth_registration:=true enable_d2c_viewer:=true
   ```

   This launches the camera node with depth to color registration and opens a viewer to display the overlay image.

### Switching Registration Mode at Runtime

After the camera node starts, use `/camera/set_image_registration_mode` to switch registration mode without restarting the node. The supported modes are:

* `OFF`: Disable image registration.
* `HW_D2C`: Align depth to color in hardware.
* `SW_D2C`: Align depth to color in software.
* `SW_C2D`: Align color to depth in software.

Both color and depth streams must be enabled for every mode except `OFF`. The service stops and restarts streams automatically during the switch and restores the previous mode if the operation fails.

```bash
rosservice call /camera/set_image_registration_mode "{data: 'SW_D2C'}"
```

### Selecting Topics in RViz

To visualize the aligned images in RViz:

1. Launch RViz after running one of the above commands.
2. Select the topic for the depth to color overlay image. An example topic selection is shown here:

   ![Topic Selection for Depth to Color Overlay](../../image/align_depth_color/image3.png)

### Example of Depth to Color Overlay

After selecting the appropriate topic in RViz, you will be able to see the depth to color overlay image. Here's what it might look like:

![Depth to Color Overlay Image](../../image/align_depth_color/image4.jpg)
