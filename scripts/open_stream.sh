#!/bin/bash

source /opt/ros/noetic/setup.bash

rostopic hz /ob_camera_01/depth/image_raw &
rostopic hz /ob_camera_01/ir/image_raw &

rostopic hz /ob_camera_02/depth/image_raw &
rostopic hz /ob_camera_02/ir/image_raw &

rostopic hz /ob_camera_03/depth/image_raw &
rostopic hz /ob_camera_03/ir/image_raw &

rostopic hz /ob_camera_04/depth/image_raw &
rostopic hz /ob_camera_04/ir/image_raw &

rostopic hz /ob_camera_05/depth/image_raw &
rostopic hz /ob_camera_05/ir/image_raw &

wait
