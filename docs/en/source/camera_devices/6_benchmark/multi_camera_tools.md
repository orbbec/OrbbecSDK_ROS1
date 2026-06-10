# Multi-Camera Helper Tools

This section describes multi-camera synchronization verification and image grouping helper tools provided by the ROS1 project.

## image_sync_example_node

`image_sync_example_node` verifies multi-camera image timestamp synchronization online. It subscribes to multiple image topics, displays synchronized images, and prints timestamp difference information for validating frame alignment in Primary / Secondary Synced mode.

Start multi-camera synchronization:

```bash
roslaunch orbbec_camera multi_camera_synced.launch
```

In a new terminal, run the synchronization verification node:

```bash
rosrun orbbec_camera image_sync_example_node
```

For the complete multi-camera synchronization setup, see [Multi-Camera Synchronization Verification Node](../5_advanced_guide/multi_camera/multi_camera_synced_verification_tool.md).

## group_images.sh

`group_images.sh` is an offline grouping script for saved multi-camera images. By default, it reads `/home/orbbec/image/`, groups images by timestamp, and copies the grouped result to `grouped_images` under the script directory.

Before using it, edit `image_directory`, `master_camera_serial_no`, `use_device_time`, and related variables in the script for your environment.

```bash
cd src/OrbbecSDK_ROS1/scripts
python3 group_images.sh
```

> Note: The file extension is `.sh`, but the content is a Python script.
