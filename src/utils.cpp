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

#include "orbbec_camera/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"

#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/ros.h"

namespace orbbec_camera {
OBFormat OBFormatFromString(const std::string &format) {
  std::string fixed_format;
  std::transform(format.begin(), format.end(), std::back_inserter(fixed_format),
                 [](const char ch) { return std::isalpha(ch) ? toupper(ch) : ch; });
  if (fixed_format == "MJPG") {
    return OB_FORMAT_MJPG;
  } else if (fixed_format == "YUYV") {
    return OB_FORMAT_YUYV;
  } else if (fixed_format == "YUYV2") {
    return OB_FORMAT_YUY2;
  } else if (fixed_format == "UYVY") {
    return OB_FORMAT_UYVY;
  } else if (fixed_format == "NV12") {
    return OB_FORMAT_NV12;
  } else if (fixed_format == "NV21") {
    return OB_FORMAT_NV21;
  } else if (fixed_format == "H264") {
    return OB_FORMAT_H264;
  } else if (fixed_format == "H265") {
    return OB_FORMAT_H265;
  } else if (fixed_format == "Y16") {
    return OB_FORMAT_Y16;
  } else if (fixed_format == "Y8") {
    return OB_FORMAT_Y8;
  } else if (fixed_format == "Y10") {
    return OB_FORMAT_Y10;
  } else if (fixed_format == "Y11") {
    return OB_FORMAT_Y11;
  } else if (fixed_format == "Y12") {
    return OB_FORMAT_Y12;
  } else if (fixed_format == "GRAY") {
    return OB_FORMAT_GRAY;
  } else if (fixed_format == "HEVC") {
    return OB_FORMAT_HEVC;
  } else if (fixed_format == "I420") {
    return OB_FORMAT_I420;
  } else if (fixed_format == "ACCEL") {
    return OB_FORMAT_ACCEL;
  } else if (fixed_format == "GYRO") {
    return OB_FORMAT_GYRO;
  } else if (fixed_format == "POINT") {
    return OB_FORMAT_POINT;
  } else if (fixed_format == "RGB_POINT") {
    return OB_FORMAT_RGB_POINT;
  } else if (fixed_format == "RLE") {
    return OB_FORMAT_RLE;
  } else if (fixed_format == "RGB888" || fixed_format == "RGB") {
    return OB_FORMAT_RGB888;
  } else if (fixed_format == "BGR") {
    return OB_FORMAT_BGR;
  } else if (fixed_format == "Y14") {
    return OB_FORMAT_Y14;
  } else if (fixed_format == "BGRA") {
    return OB_FORMAT_BGRA;
  } else if (fixed_format == "COMPRESSED") {
    return OB_FORMAT_COMPRESSED;
  } else if (fixed_format == "RVL") {
    return OB_FORMAT_RVL;
  } else if (fixed_format == "Z16") {
    return OB_FORMAT_Z16;
  } else if (fixed_format == "YV12") {
    return OB_FORMAT_YV12;
  } else if (fixed_format == "BA81") {
    return OB_FORMAT_BA81;
  } else if (fixed_format == "RGBA") {
    return OB_FORMAT_RGBA;
  } else if (fixed_format == "BYR2") {
    return OB_FORMAT_BYR2;
  } else if (fixed_format == "RW16") {
    return OB_FORMAT_RW16;
  } else if (fixed_format == "DISP16") {
    return OB_FORMAT_DISP16;
  } else {
    return OB_FORMAT_UNKNOWN;
  }
}

std::string OBFormatToString(const OBFormat &format) {
  switch (format) {
    case OB_FORMAT_MJPG:
      return "MJPG";
    case OB_FORMAT_YUYV:
      return "YUYV";
    case OB_FORMAT_YUY2:
      return "YUYV2";
    case OB_FORMAT_UYVY:
      return "UYVY";
    case OB_FORMAT_NV12:
      return "NV12";
    case OB_FORMAT_NV21:
      return "NV21";
    case OB_FORMAT_H264:
      return "H264";
    case OB_FORMAT_H265:
      return "H265";
    case OB_FORMAT_Y16:
      return "Y16";
    case OB_FORMAT_Y8:
      return "Y8";
    case OB_FORMAT_Y10:
      return "Y10";
    case OB_FORMAT_Y11:
      return "Y11";
    case OB_FORMAT_Y12:
      return "Y12";
    case OB_FORMAT_GRAY:
      return "GRAY";
    case OB_FORMAT_HEVC:
      return "HEVC";
    case OB_FORMAT_I420:
      return "I420";
    case OB_FORMAT_ACCEL:
      return "ACCEL";
    case OB_FORMAT_GYRO:
      return "GYRO";
    case OB_FORMAT_POINT:
      return "POINT";
    case OB_FORMAT_RGB_POINT:
      return "RGB_POINT";
    case OB_FORMAT_RLE:
      return "RLE";
    case OB_FORMAT_RGB888:
      return "RGB888";
    case OB_FORMAT_BGR:
      return "BGR";
    case OB_FORMAT_Y14:
      return "Y14";
    case OB_FORMAT_BGRA:
      return "BGRA";
    case OB_FORMAT_COMPRESSED:
      return "COMPRESSED";
    case OB_FORMAT_RVL:
      return "RVL";
    case OB_FORMAT_Z16:
      return "Z16";
    case OB_FORMAT_YV12:
      return "YV12";
    case OB_FORMAT_BA81:
      return "BA81";
    case OB_FORMAT_RGBA:
      return "RGBA";
    case OB_FORMAT_BYR2:
      return "BYR2";
    case OB_FORMAT_RW16:
      return "RW16";
    case OB_FORMAT_DISP16:
      return "DISP16";
    default:
      return "UNKNOWN";
  }
}

std::string ObDeviceTypeToString(const OBDeviceType &type) {
  switch (type) {
    case OBDeviceType::OB_STRUCTURED_LIGHT_BINOCULAR_CAMERA:
      return "structured light binocular camera";
    case OBDeviceType::OB_STRUCTURED_LIGHT_MONOCULAR_CAMERA:
      return "structured light monocular camera";
    case OBDeviceType::OB_TOF_CAMERA:
      return "tof camera";
  }
  return "unknown technology camera";
}

sensor_msgs::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                            OBCameraDistortion distortion, int width) {
  (void)width;
  sensor_msgs::CameraInfo info;
  info.distortion_model = sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL;
  info.width = intrinsic.width;
  info.height = intrinsic.height;
  info.D.resize(8, 0.0);
  info.D[0] = distortion.k1;
  info.D[1] = distortion.k2;
  info.D[2] = distortion.p1;
  info.D[3] = distortion.p2;
  info.D[4] = distortion.k3;
  info.D[5] = distortion.k4;
  info.D[6] = distortion.k5;
  info.D[7] = distortion.k6;

  info.K.fill(0.0);
  info.K[0] = intrinsic.fx;
  info.K[2] = intrinsic.cx;
  info.K[4] = intrinsic.fy;
  info.K[5] = intrinsic.cy;
  info.K[8] = 1.0;

  info.R.fill(0.0);
  info.R[0] = 1;
  info.R[4] = 1;
  info.R[8] = 1;

  info.P.fill(0.0);
  info.P[0] = info.K[0];
  info.P[2] = info.K[2];
  info.P[5] = info.K[4];
  info.P[6] = info.K[5];
  info.P[10] = 1.0;
  return info;
}

void saveRGBPointsToPly(std::shared_ptr<ob::Frame> frame, const std::string &fileName) {
  CHECK_NOTNULL(frame.get());
  size_t point_size = frame->dataSize() / sizeof(OBColorPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  fprintf(fp, "ply\n");
  CHECK_NOTNULL(fp);
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  auto *point = (OBColorPoint *)frame->data();
  CHECK_NOTNULL(point);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", point->x, point->y, point->z, (int)point->r,
            (int)point->g, (int)point->b);
    point++;
  }

  fflush(fp);
  fclose(fp);
}

void saveRGBPointCloudMsgToPly(const sensor_msgs::PointCloud2 &msg, const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);

  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_r(msg, "r");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_g(msg, "g");
  sensor_msgs::PointCloud2ConstIterator<uint8_t> iter_b(msg, "b");

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", valid_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", *iter_x, *iter_y, *iter_z, (int)*iter_r,
              (int)*iter_g, (int)*iter_b);
    }
  }

  fflush(fp);
  fclose(fp);
}

void saveDepthPointCloudMsgToPly(const sensor_msgs::PointCloud2 &msg, const std::string &fileName) {
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);
  sensor_msgs::PointCloud2ConstIterator<float> iter_x(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float> iter_y(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float> iter_z(msg, "z");

  // First, count the actual number of valid points
  size_t valid_points = 0;
  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      ++valid_points;
    }
  }

  // Reset the iterators
  iter_x = sensor_msgs::PointCloud2ConstIterator<float>(msg, "x");
  iter_y = sensor_msgs::PointCloud2ConstIterator<float>(msg, "y");
  iter_z = sensor_msgs::PointCloud2ConstIterator<float>(msg, "z");

  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", valid_points);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      fprintf(fp, "%.3f %.3f %.3f\n", *iter_x, *iter_y, *iter_z);
    }
  }

  fflush(fp);
  fclose(fp);
}

void savePointsToPly(std::shared_ptr<ob::Frame> frame, const std::string &fileName) {
  size_t point_size = frame->dataSize() / sizeof(OBPoint);
  FILE *fp = fopen(fileName.c_str(), "wb+");
  CHECK_NOTNULL(fp);
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");

  auto *points = (OBPoint *)frame->data();
  CHECK_NOTNULL(points);
  for (size_t i = 0; i < point_size; i++) {
    fprintf(fp, "%.3f %.3f %.3f\n", points->x, points->y, points->z);
    points++;
  }

  fflush(fp);
  fclose(fp);
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[1], rotation[2], rotation[3], rotation[4], rotation[5], rotation[6],
      rotation[7], rotation[8];
  Eigen::Quaternionf q(m);
  return {q.x(), q.y(), q.z(), q.w()};
}

std::ostream &operator<<(std::ostream &os, const OBCameraParam &rhs) {
  auto depth_intrinsic = rhs.depthIntrinsic;
  auto rgb_intrinsic = rhs.rgbIntrinsic;
  os << "=====depth intrinsic=====\n";
  os << "fx : " << depth_intrinsic.fx << "\n";
  os << "fy : " << depth_intrinsic.fy << "\n";
  os << "cx : " << depth_intrinsic.cx << "\n";
  os << "cy : " << depth_intrinsic.cy << "\n";
  os << "width : " << depth_intrinsic.width << "\n";
  os << "height : " << depth_intrinsic.height << "\n";
  os << "=====rgb intrinsic=====\n";
  os << "fx : " << rgb_intrinsic.fx << "\n";
  os << "fy : " << rgb_intrinsic.fy << "\n";
  os << "cx : " << rgb_intrinsic.cx << "\n";
  os << "cy : " << rgb_intrinsic.cy << "\n";
  os << "width : " << rgb_intrinsic.width << "\n";
  os << "height : " << rgb_intrinsic.height << "\n";
  return os;
}

Extrinsics obExtrinsicsToMsg(const OBD2CTransform &extrinsics, const std::string &frame_id) {
  Extrinsics msg;
  for (int i = 0; i < 9; ++i) {
    msg.rotation[i] = extrinsics.rot[i];
    if (i < 3) {
      msg.translation[i] = extrinsics.trans[i] / 1000.0;
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

ros::Time fromMsToROSTime(uint64_t ms) {
  auto total = static_cast<uint64_t>(ms * 1e6);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  ros::Time stamp(sec, nano_sec);
  return stamp;
}

ros::Time fromUsToROSTime(uint64_t us) {
  auto total = static_cast<uint64_t>(us * 1e3);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  ros::Time stamp(sec, nano_sec);
  return stamp;
}

bool isOpenNIDevice(int pid) {
  static const std::vector<int> OPENNI_DEVICE_PIDS = {
      0x0300, 0x0301, 0x0400, 0x0401, 0x0402, 0x0403, 0x0404, 0x0407, 0x0601, 0x060b,
      0x060e, 0x060f, 0x0610, 0x0613, 0x0614, 0x0616, 0x0617, 0x0618, 0x061b, 0x062b,
      0x062c, 0x062d, 0x0632, 0x0633, 0x0634, 0x0635, 0x0636, 0x0637, 0x0638, 0x0639,
      0x063a, 0x0650, 0x0651, 0x0654, 0x0655, 0x0656, 0x0657, 0x0658, 0x0659, 0x065a,
      0x065b, 0x065c, 0x065d, 0x0698, 0x0699, 0x069a, 0x055c, 0x065e, 0x06a0, 0x69e};

  for (const auto &pid_openni : OPENNI_DEVICE_PIDS) {
    if (pid == pid_openni) {
      return true;
    }
  }
  return false;
}

OBMultiDeviceSyncMode OBSyncModeFromString(const std::string &mode) {
  if (mode == "FREE_RUN") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  } else if (mode == "STANDALONE") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_STANDALONE;
  } else if (mode == "PRIMARY") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_PRIMARY;
  } else if (mode == "SECONDARY") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY;
  } else if (mode == "SECONDARY_SYNCED") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED;
  } else if (mode == "SOFTWARE_TRIGGERING") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING;
  } else if (mode == "HARDWARE_TRIGGERING") {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING;
  } else {
    return OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  }
}

std::string OBSyncModeToString(const OBMultiDeviceSyncMode &mode) {
  switch (mode) {
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN:
      return "FREE_RUN";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_STANDALONE:
      return "STANDALONE";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_PRIMARY:
      return "PRIMARY";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY:
      return "SECONDARY";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SECONDARY_SYNCED:
      return "SECONDARY_SYNCED";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING:
      return "SOFTWARE_TRIGGERING";
    case OBMultiDeviceSyncMode::OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING:
      return "HARDWARE_TRIGGERING";
    default:
      return "FREE_RUN";
  }
}

std::ostream &operator<<(std::ostream &os, const OBMultiDeviceSyncMode &rhs) {
  os << OBSyncModeToString(rhs);
  return os;
}

OB_SAMPLE_RATE sampleRateFromString(std::string &sample_rate) {
  // covert to lower case
  std::transform(sample_rate.begin(), sample_rate.end(), sample_rate.begin(), ::tolower);
  if (sample_rate == "1.5625hz") {
    return OB_SAMPLE_RATE_1_5625_HZ;
  } else if (sample_rate == "3.125hz") {
    return OB_SAMPLE_RATE_3_125_HZ;
  } else if (sample_rate == "6.25hz") {
    return OB_SAMPLE_RATE_6_25_HZ;
  } else if (sample_rate == "12.5hz") {
    return OB_SAMPLE_RATE_12_5_HZ;
  } else if (sample_rate == "25hz") {
    return OB_SAMPLE_RATE_25_HZ;
  } else if (sample_rate == "50hz") {
    return OB_SAMPLE_RATE_50_HZ;
  } else if (sample_rate == "100hz") {
    return OB_SAMPLE_RATE_100_HZ;
  } else if (sample_rate == "200hz") {
    return OB_SAMPLE_RATE_200_HZ;
  } else if (sample_rate == "500hz") {
    return OB_SAMPLE_RATE_500_HZ;
  } else if (sample_rate == "1khz") {
    return OB_SAMPLE_RATE_1_KHZ;
  } else if (sample_rate == "2khz") {
    return OB_SAMPLE_RATE_2_KHZ;
  } else if (sample_rate == "4khz") {
    return OB_SAMPLE_RATE_4_KHZ;
  } else if (sample_rate == "8khz") {
    return OB_SAMPLE_RATE_8_KHZ;
  } else if (sample_rate == "16khz") {
    return OB_SAMPLE_RATE_16_KHZ;
  } else if (sample_rate == "32khz") {
    return OB_SAMPLE_RATE_32_KHZ;
  } else {
    ROS_ERROR_STREAM("Unknown OB_SAMPLE_RATE: " << sample_rate);
    return OB_SAMPLE_RATE_100_HZ;
  }
}

std::string sampleRateToString(const OB_SAMPLE_RATE &sample_rate) {
  switch (sample_rate) {
    case OB_SAMPLE_RATE_1_5625_HZ:
      return "1.5625Hz";
    case OB_SAMPLE_RATE_3_125_HZ:
      return "3.125Hz";
    case OB_SAMPLE_RATE_6_25_HZ:
      return "6.25Hz";
    case OB_SAMPLE_RATE_12_5_HZ:
      return "12.5Hz";
    case OB_SAMPLE_RATE_25_HZ:
      return "25Hz";
    case OB_SAMPLE_RATE_50_HZ:
      return "50Hz";
    case OB_SAMPLE_RATE_100_HZ:
      return "100Hz";
    case OB_SAMPLE_RATE_200_HZ:
      return "200Hz";
    case OB_SAMPLE_RATE_500_HZ:
      return "500Hz";
    case OB_SAMPLE_RATE_1_KHZ:
      return "1kHz";
    case OB_SAMPLE_RATE_2_KHZ:
      return "2kHz";
    case OB_SAMPLE_RATE_4_KHZ:
      return "4kHz";
    case OB_SAMPLE_RATE_8_KHZ:
      return "8kHz";
    case OB_SAMPLE_RATE_16_KHZ:
      return "16kHz";
    case OB_SAMPLE_RATE_32_KHZ:
      return "32kHz";
    default:
      return "100Hz";
  }
}

OB_GYRO_FULL_SCALE_RANGE fullGyroScaleRangeFromString(std::string &full_scale_range) {
  std::transform(full_scale_range.begin(), full_scale_range.end(), full_scale_range.begin(),
                 ::tolower);
  if (full_scale_range == "16dps") {
    return OB_GYRO_FS_16dps;
  } else if (full_scale_range == "31dps") {
    return OB_GYRO_FS_31dps;
  } else if (full_scale_range == "62dps") {
    return OB_GYRO_FS_62dps;
  } else if (full_scale_range == "125dps") {
    return OB_GYRO_FS_125dps;
  } else if (full_scale_range == "250dps") {
    return OB_GYRO_FS_250dps;
  } else if (full_scale_range == "500dps") {
    return OB_GYRO_FS_500dps;
  } else if (full_scale_range == "1000dps") {
    return OB_GYRO_FS_1000dps;
  } else if (full_scale_range == "2000dps") {
    return OB_GYRO_FS_2000dps;
  } else {
    ROS_ERROR_STREAM("Unknown OB_GYRO_FULL_SCALE_RANGE: " << full_scale_range);
    return OB_GYRO_FS_2000dps;
  }
}

std::string fullGyroScaleRangeToString(const OB_GYRO_FULL_SCALE_RANGE &full_scale_range) {
  switch (full_scale_range) {
    case OB_GYRO_FS_16dps:
      return "16dps";
    case OB_GYRO_FS_31dps:
      return "31dps";
    case OB_GYRO_FS_62dps:
      return "62dps";
    case OB_GYRO_FS_125dps:
      return "125dps";
    case OB_GYRO_FS_250dps:
      return "250dps";
    case OB_GYRO_FS_500dps:
      return "500dps";
    case OB_GYRO_FS_1000dps:
      return "1000dps";
    case OB_GYRO_FS_2000dps:
      return "2000dps";
    default:
      return "16dps";
  }
}

OBAccelFullScaleRange fullAccelScaleRangeFromString(std::string &full_scale_range) {
  std::transform(full_scale_range.begin(), full_scale_range.end(), full_scale_range.begin(),
                 ::tolower);
  if (full_scale_range == "2g") {
    return OB_ACCEL_FS_2g;
  } else if (full_scale_range == "4g") {
    return OB_ACCEL_FS_4g;
  } else if (full_scale_range == "8g") {
    return OB_ACCEL_FS_8g;
  } else if (full_scale_range == "16g") {
    return OB_ACCEL_FS_16g;
  } else {
    ROS_ERROR_STREAM("Unknown OB_ACCEL_FULL_SCALE_RANGE: " << full_scale_range);
    return OB_ACCEL_FS_16g;
  }
}

std::string fullAccelScaleRangeToString(const OBAccelFullScaleRange &full_scale_range) {
  switch (full_scale_range) {
    case OB_ACCEL_FS_2g:
      return "2g";
    case OB_ACCEL_FS_4g:
      return "4g";
    case OB_ACCEL_FS_8g:
      return "8g";
    case OB_ACCEL_FS_16g:
      return "16g";
    default:
      return "2g";
  }
}

bool isValidJPEG(const std::shared_ptr<ob::ColorFrame> &frame) {
  if (frame->dataSize() < 2) {  // Checking both start and end markers, so minimal size is 4
    return false;
  }

  const auto *data = static_cast<const uint8_t *>(frame->data());

  // Check for JPEG start marker
  if (data[0] != 0xFF || data[1] != 0xD8) {
    return false;
  }

  return true;
}

std::string fourccToString(const uint32_t fourcc) {
  std::string str;
  str += (fourcc & 0xFF);
  str += ((fourcc >> 8) & 0xFF);
  str += ((fourcc >> 16) & 0xFF);
  str += ((fourcc >> 24) & 0xFF);
  return str;
}

std::string metaDataTypeToString(const OBFrameMetadataType &meta_data_type) {
  switch (meta_data_type) {
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_TIMESTAMP:
      return "timestamp";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP:
      return "sensor_timestamp";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_FRAME_NUMBER:
      return "frame_number";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AUTO_EXPOSURE:
      return "auto_exposure";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EXPOSURE:
      return "exposure";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GAIN:
      return "gain";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AUTO_WHITE_BALANCE:
      return "auto_white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_WHITE_BALANCE:
      return "white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_BRIGHTNESS:
      return "brightness";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_CONTRAST:
      return "contrast";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SATURATION:
      return "saturation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_SHARPNESS:
      return "sharpness";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_BACKLIGHT_COMPENSATION:
      return "backlight_compensation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HUE:
      return "hue";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GAMMA:
      return "gamma";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_POWER_LINE_FREQUENCY:
      return "power_line_frequency";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LOW_LIGHT_COMPENSATION:
      return "low_light_compensation";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_MANUAL_WHITE_BALANCE:
      return "manual_white_balance";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_ACTUAL_FRAME_RATE:
      return "actual_frame_rate";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_FRAME_RATE:
      return "frame_rate";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_LEFT:
      return "ae_roi_left";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_TOP:
      return "ae_roi_top";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_RIGHT:
      return "ae_roi_right";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_AE_ROI_BOTTOM:
      return "ae_roi_bottom";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EXPOSURE_PRIORITY:
      return "exposure_priority";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_NAME:
      return "hdr_sequence_name";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_SIZE:
      return "hdr_sequence_size";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_HDR_SEQUENCE_INDEX:
      return "hdr_sequence_index";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LASER_POWER:
      return "laser_power";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_LASER_POWER_MODE:
      return "laser_power_mode";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_EMITTER_MODE:
      return "emitter_mode";
    case OBFrameMetadataType::OB_FRAME_METADATA_TYPE_GPIO_INPUT_DATA:
      return "gpio_input_data";
    default:
      return "unknown";
  }
}

OBHoleFillingMode holeFillingModeFromString(const std::string &hole_filling_mode) {
  if (hole_filling_mode == "FILL_TOP") {
    return OB_HOLE_FILL_TOP;
  } else if (hole_filling_mode == "FILL_NEAREST") {
    return OB_HOLE_FILL_NEAREST;
  } else if (hole_filling_mode == "FILL_FAREST") {
    return OB_HOLE_FILL_FAREST;
  } else {
    return OB_HOLE_FILL_NEAREST;
  }
}

float depthPrecisionFromString(const std::string &depth_precision_level_str) {
  // covert 0.8mm to 0.8
  if (depth_precision_level_str.size() < 2) {
    return 1.0;
  }
  std::string depth_precision_level_str_num =
      depth_precision_level_str.substr(0, depth_precision_level_str.size() - 2);
  return std::stof(depth_precision_level_str_num);
}

std::ostream &operator<<(std::ostream &os, const OBFormat &rhs) {
  os << OBFormatToString(rhs);
  return os;
}

std::string OBSensorTypeToString(const OBSensorType &type) {
  switch (type) {
    case OB_SENSOR_UNKNOWN:
      return "UNKNOWN";
    case OB_SENSOR_IR:
      return "IR";
    case OB_SENSOR_COLOR:
      return "COLOR";
    case OB_SENSOR_DEPTH:
      return "DEPTH";
    case OB_SENSOR_ACCEL:
      return "ACCEL";
    case OB_SENSOR_GYRO:
      return "GYRO";
    case OB_SENSOR_IR_LEFT:
      return "LEFT_IR";
    case OB_SENSOR_IR_RIGHT:
      return "RIGHT_IR";
    case OB_SENSOR_RAW_PHASE:
      return "RAW_PHASE";
    default:
      return "UNKNOWN";
  }
}
std::ostream &operator<<(std::ostream &os, const OBSensorType &rhs) {
  os << OBSensorTypeToString(rhs);
  return os;
}
}  // namespace orbbec_camera
