/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/

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

void saveRGBPointCloudMsgToPly(const sensor_msgs::PointCloud2 &msg, const std::string &fileName);

void saveDepthPointCloudMsgToPly(const sensor_msgs::PointCloud2 &msg, const std::string &fileName);


tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]);

std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs);

Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics, const std::string &frame_id);

ros::Time frameTimeStampToROSTime(uint64_t ms);

bool isOpenNIDevice(int pid);

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string &mode);

OB_SAMPLE_RATE sampleRateFromString(std::string &sample_rate);

std::string sampleRateToString(const OB_SAMPLE_RATE &sample_rate);

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string &full_scale_range);

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE &full_scale_range);

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string &full_scale_range);

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange &full_scale_range);

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame);

std::string fourccToString(const uint32_t fourcc);

}  // namespace orbbec_camera
