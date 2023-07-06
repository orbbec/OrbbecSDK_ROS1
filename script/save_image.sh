#!/bin/bash

source /opt/ros/noetic/setup.bash

rosservice call /ob_camera_01/save_images "{}"

rosservice call /ob_camera_02/save_images "{}"

rosservice call /ob_camera_03/save_images "{}"

rosservice call /ob_camera_04/save_images "{}"

rosservice call /ob_camera_05/save_images "{}"

wait
