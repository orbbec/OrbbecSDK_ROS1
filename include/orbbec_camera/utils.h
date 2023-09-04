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

std::string OBFormatToString(const OBFormat &format);

std::string ObDeviceTypeToString(const OBDeviceType &type);

sensor_msgs::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                            OBCameraDistortion distortion, int width);

void savePointsToPly(std::shared_ptr<ob::Frame> frame, const std::string &fileName);

void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, const std::string &fileName);

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs);

Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics, const std::string &frame_id);

ros::Time frameTimeStampToROSTime(uint64_t ms);

bool isOpenNIDevice(int pid);

OBSyncMode OBSyncModeFromString(const std::string &mode);

OB_SAMPLE_RATE sampleRateFromString(std::string &sample_rate);

std::string sampleRateToString(const OB_SAMPLE_RATE &sample_rate);

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string &full_scale_range);

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE &full_scale_range);

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string &full_scale_range);

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange &full_scale_range);

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame);

std::string fourccToString(const uint32_t fourcc);

}  // namespace orbbec_camera
