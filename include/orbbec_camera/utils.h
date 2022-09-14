/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2021 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#pragma once

#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/distortion_models.h"
#include "libobsensor/ObSensor.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "types.h"
#include "sensor_msgs/PointCloud2.h"
#include "orbbec_camera/Extrinsics.h"

namespace orbbec_camera {
OBFormat OBFormatFromString(const std::string &format);

sensor_msgs::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                            OBCameraDistortion distortion, int width);

void savePointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string& filename);

void saveRGBPointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string& filename);


tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs);

Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics, const std::string &frame_id);

ros::Time frameTimeStampToROSTime(uint64_t ms);

}  // namespace orbbec_camera
