#include "orbbec_camera/utils.h"
#include <tf2/LinearMath/Quaternion.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointCloud.h"

#include "sensor_msgs/point_cloud2_iterator.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "ros/ros.h"

namespace orbbec_camera {
OBFormat OBFormatFromString(const std::string& format) {
  std::string fixed_format;
  std::transform(format.begin(), format.end(), std::back_inserter(fixed_format),
                 [](const auto ch) { return std::isalpha(ch) ? toupper(ch) : ch; });
  if (fixed_format == "YUYV") {
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
  } else if (fixed_format == "REL") {
    return OB_FORMAT_RLE;
  } else if (fixed_format == "RGB888") {
    return OB_FORMAT_RGB888;
  } else if (fixed_format == "BGR") {
    return OB_FORMAT_BGR;
  } else if (fixed_format == "Y14") {
    return OB_FORMAT_Y14;
  } else {
    return OB_FORMAT_UNKNOWN;
  }
}

sensor_msgs::CameraInfo convertToCameraInfo(OBCameraIntrinsic intrinsic,
                                            OBCameraDistortion distortion, int width) {
  sensor_msgs::CameraInfo info;
  info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info.width = intrinsic.width;
  info.height = intrinsic.height;
  info.D.resize(5, 0.0);
  info.D[0] = distortion.k1;
  info.D[1] = distortion.k2;
  info.D[2] = distortion.k3;
  info.D[3] = distortion.k4;
  info.D[4] = distortion.k5;

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

  double scaling = static_cast<double>(width) / 640;
  info.K[0] *= scaling;  // fx
  info.K[2] *= scaling;  // cx
  info.K[4] *= scaling;  // fy
  info.K[5] *= scaling;  // cy
  info.P[0] *= scaling;  // fx
  info.P[2] *= scaling;  // cx
  info.P[5] *= scaling;  // fy
  info.P[6] *= scaling;  // cy

  return info;
}
void savePointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string& filename) {
  sensor_msgs::PointCloud out_point_cloud;
  sensor_msgs::convertPointCloud2ToPointCloud(*cloud, out_point_cloud);
  size_t point_size = 0;
  FILE* fp = fopen(filename.c_str(), "wb+");
  for (auto& point : out_point_cloud.points) {
    if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
      point_size++;
    }
  }
  ROS_INFO_STREAM("Point size: " << point_size);
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "end_header\n");
  for (const auto& point : out_point_cloud.points) {
    if (!std::isnan(point.x) && !std::isnan(point.y) && !std::isnan(point.z)) {
      fprintf(fp, "%.3f %.3f %.3f\n", point.x, point.y, point.z);
    }
  }
  fflush(fp);
  fclose(fp);
}

void saveRGBPointToPly(sensor_msgs::PointCloud2::Ptr cloud, const std::string& filename) {
  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*cloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*cloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*cloud, "b");
  size_t point_size = cloud->width * cloud->height;
  std::vector<geometry_msgs::Point32> points;
  std::vector<std::vector<uint8_t>> rgb;
  for (size_t i = 0; i < point_size; i++) {
    if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
      geometry_msgs::Point32 point;
      point.x = *iter_x;
      point.y = *iter_y;
      point.z = *iter_z;
      points.push_back(point);
      std::vector<uint8_t> rgb_point;
      rgb_point.push_back(*iter_r);
      rgb_point.push_back(*iter_g);
      rgb_point.push_back(*iter_b);
      rgb.push_back(rgb_point);
    }
    ++iter_x;
    ++iter_y;
    ++iter_z;
    ++iter_r;
    ++iter_g;
    ++iter_b;
  }
  point_size = points.size();
  FILE* fp = fopen(filename.c_str(), "wb+");

  ROS_INFO_STREAM("Point size: " << point_size);
  fprintf(fp, "ply\n");
  fprintf(fp, "format ascii 1.0\n");
  fprintf(fp, "element vertex %zu\n", point_size);
  fprintf(fp, "property float x\n");
  fprintf(fp, "property float y\n");
  fprintf(fp, "property float z\n");
  fprintf(fp, "property uchar red\n");
  fprintf(fp, "property uchar green\n");
  fprintf(fp, "property uchar blue\n");
  fprintf(fp, "end_header\n");

  for (size_t i = 0; i < point_size; ++i) {
    fprintf(fp, "%.3f %.3f %.3f %d %d %d\n", points[i].x, points[i].y, points[i].z, rgb[i][0],
            rgb[i][1], rgb[i][2]);
  }
  fflush(fp);
  fclose(fp);
}

tf2::Quaternion rotationMatrixToQuaternion(const float rotation[9]) {
  Eigen::Matrix3f m;
  // We need to be careful about the order, as RS2 rotation matrix is
  // column-major, while Eigen::Matrix3f expects row-major.
  m << rotation[0], rotation[3], rotation[6], rotation[1], rotation[4], rotation[7], rotation[2],
      rotation[5], rotation[8];
  Eigen::Quaternionf q(m);
  return {q.x(), q.y(), q.z(), q.w()};
}

std::ostream& operator<<(std::ostream& os, const OBCameraParam& rhs) {
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

Extrinsics obExtrinsicsToMsg(const OBD2CTransform& extrinsics, const std::string& frame_id) {
  Extrinsics msg;
  for (int i = 0; i < 9; ++i) {
    msg.rotation[i] = extrinsics.rot[i];
    if (i < 3) {
      msg.translation[i] = extrinsics.trans[i];
    }
  }

  msg.header.frame_id = frame_id;
  return msg;
}

ros::Time frameTimeStampToROSTime(uint64_t ms) {
  auto total = static_cast<uint64_t>(ms * 1e6);
  uint64_t sec = total / 1000000000;
  uint64_t nano_sec = total % 1000000000;
  ros::Time stamp(sec, nano_sec);
  return stamp;
}

}  // namespace orbbec_camera
