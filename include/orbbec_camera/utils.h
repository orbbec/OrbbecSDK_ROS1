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
#include <opencv2/opencv.hpp>

// Utility function for failure messages
namespace orbbec_camera {
inline void LogFatal(const char *file, int line, const std::string &message) {
  std::cerr << "Check failed at " << file << ":" << line << ": " << message << std::endl;
  std::abort();
}
}  // namespace orbbec_camera

// Macros for checking conditions and comparing values
#define CHECK(condition) \
  (!(condition) ? LogFatal(__FILE__, __LINE__, "Check failed: " #condition) : (void)0)

template <typename T1, typename T2>
void CheckOp(const char *expr, const char *file, int line, T1 val1, T2 val2, bool result) {
  if (!result) {
    std::ostringstream os;
    os << "Check failed: " << expr << " (" << val1 << " vs. " << val2 << ")";
    orbbec_camera::LogFatal(file, line, os.str());
  }
}

#define CHECK_OP(opname, op, val1, val2) \
  CheckOp(#val1 " " #op " " #val2, __FILE__, __LINE__, val1, val2, (val1)op(val2))

#define CHECK_EQ(val1, val2) CHECK_OP(_EQ, ==, val1, val2)
#define CHECK_NE(val1, val2) CHECK_OP(_NE, !=, val1, val2)
#define CHECK_LE(val1, val2) CHECK_OP(_LE, <=, val1, val2)
#define CHECK_LT(val1, val2) CHECK_OP(_LT, <, val1, val2)
#define CHECK_GE(val1, val2) CHECK_OP(_GE, >=, val1, val2)
#define CHECK_GT(val1, val2) CHECK_OP(_GT, >, val1, val2)

// Overload for raw pointers
template <typename T>
T *CheckNotNull(T *ptr, const char *file, int line) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    orbbec_camera::LogFatal(file, line, os.str());
  }
  return ptr;
}

// Template for smart pointers like std::shared_ptr, std::unique_ptr
template <typename T>
T &CheckNotNull(T &ptr, const char *file, int line) {
  if (ptr == nullptr) {
    std::ostringstream os;
    os << "Null pointer passed to CheckNotNull at " << file << ":" << line;
    orbbec_camera::LogFatal(file, line, os.str());
  }
  return ptr;
}

#if defined(CHECK_NOTNULL)
#undef CHECK_NOTNULL
#endif
#define CHECK_NOTNULL(val) CheckNotNull(val, __FILE__, __LINE__)

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

ros::Time fromMsToROSTime(uint64_t ms);

ros::Time fromUsToROSTime(uint64_t us);

bool isOpenNIDevice(int pid);

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string &mode);

std::string OBSyncModeToString(const OBMultiDeviceSyncMode &mode);

std::ostream &operator<<(std::ostream &os, const OBMultiDeviceSyncMode &rhs);

OB_SAMPLE_RATE sampleRateFromString(std::string &sample_rate);

std::string sampleRateToString(const OB_SAMPLE_RATE &sample_rate);

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string &full_scale_range);

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE &full_scale_range);

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string &full_scale_range);

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange &full_scale_range);

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame);

std::string fourccToString(uint32_t fourcc);

std::string metaDataTypeToString(const OBFrameMetadataType &meta_data_type);

OBHoleFillingMode holeFillingModeFromString(const std::string &hole_filling_mode);

float depthPrecisionFromString(const std::string &depth_precision_level_str);

std::ostream &operator<<(std::ostream &os, const OBFormat &rhs);

std::string OBSensorTypeToString(const OBSensorType &type);

std::ostream &operator<<(std::ostream &os, const OBSensorType &rhs);

}  // namespace orbbec_camera
