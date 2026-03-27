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

#include "types.h"
#include "utils.h"
#include "ros_sensor.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "orbbec_camera/d2c_viewer.h"
#include "orbbec_camera/GetCameraParams.h"
#include <boost/optional.hpp>
#include <image_transport/image_transport.h>
#include <orbbec_camera/Metadata.h>
#include <orbbec_camera/IMUInfo.h>

#include <diagnostic_updater/diagnostic_updater.h>

namespace orbbec_camera {
namespace orbbec_lidar {
class OBLidarNode {
 public:
  OBLidarNode(ros::NodeHandle &nh, ros::NodeHandle &nh_private, std::shared_ptr<ob::Device> device);

  OBLidarNode(const OBLidarNode &) = delete;

  OBLidarNode &operator=(const OBLidarNode &) = delete;

  OBLidarNode(OBLidarNode &&) = delete;

  OBLidarNode &operator=(OBLidarNode &&) = delete;

  ~OBLidarNode() noexcept;

  bool isInitialized() const;

  void rebootDevice();

  void clean();

  // Safely expose the lock
  template <typename Func>
  auto withDeviceLock(Func &&func) -> decltype(func()) {
    std::lock_guard<std::recursive_mutex> lock(device_lock_);
    return func();
  }

 private:
  void init();

  void setupDevices();

  void selectBaseStream();

  void setupProfiles();

  void getParameters();

  void setupPublishers();

  void startStreams();

  void setupPipelineConfig();

  void stopStreams();

  void printSensorProfiles(const std::shared_ptr<ob::Sensor> &sensor);

  void onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set);

  void onNewIMUFrameCallback(const std::shared_ptr<ob::Frame> &accelframe,
                             const std::shared_ptr<ob::Frame> &gyroframe);

  void publishScan(std::shared_ptr<ob::FrameSet> frame_set);

  void publishScanToPoint(std::shared_ptr<ob::FrameSet> frame_set);

  void publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishSpherePointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishMergedPointCloud();

  void publishMergedSpherePointCloud();

  void publishStaticTransforms();

  void calcAndPublishStaticTransform();

  void publishStaticTF(const ros::Time &t, const tf2::Vector3 &trans, const tf2::Quaternion &q,
                       const std::string &from, const std::string &to);

  void publishDynamicTransforms();

  std::vector<OBLiDARPoint> spherePointToPoint(OBLiDARSpherePoint *sphere_point,
                                               uint32_t point_count);

  uint64_t getFrameTimestampUs(const std::shared_ptr<ob::Frame> &frame);

  void filterScan(sensor_msgs::LaserScan &scan);

  sensor_msgs::PointCloud2 filterPointCloud(sensor_msgs::PointCloud2 &point_cloud) const;

  orbbec_camera::IMUInfo createIMUInfo(const stream_index_pair &stream_index);

  void setDefaultIMUMessage(sensor_msgs::Imu &imu_msg);

  void publishLidarToIMUExtrinsics();

  void startIMU();

  void stopIMU();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_rgb_;
  ros::NodeHandle nh_ir_;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
  std::atomic_bool is_running_{false};
  std::map<stream_index_pair, std::shared_ptr<ob::Sensor>> sensors_;
  std::map<stream_index_pair, ob_format> format_;  // for open stream
  std::map<stream_index_pair, bool> enable_stream_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
  std::map<stream_index_pair, std::vector<std::shared_ptr<ob::LiDARStreamProfile>>>
      supported_profiles_;
  std::map<stream_index_pair, bool> stream_started_;
  std::map<stream_index_pair, std::string> stream_name_;
  OBExtrinsic lidar_to_imu_extrinsic_;

  std::map<stream_index_pair, std::string> format_str_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  stream_index_pair base_stream_ = LIDAR;
  std::string camera_name_ = "camera";

  std::shared_ptr<ros::Publisher> scan_pub_;
  std::shared_ptr<ros::Publisher> point_cloud_pub_;
  std::shared_ptr<ros::Publisher> imu_publisher_;
  std::map<stream_index_pair, ros::Publisher> imu_info_publishers_;
  ros::Publisher lidar_to_imu_extrinsics_publisher_;

  bool publish_tf_ = false;
  bool tf_published_ = false;
  std::condition_variable tf_cv_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::TransformStamped> static_tf_msgs_;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  double tf_publish_rate_ = 10.0;
  std::recursive_mutex device_lock_;
  bool enable_heartbeat_ = false;

  std::shared_ptr<ob::Pipeline> pipeline_ = nullptr;
  std::shared_ptr<ob::Pipeline> imuPipeline_ = nullptr;
  std::shared_ptr<ob::Config> pipeline_config_ = nullptr;
  std::atomic_bool pipeline_started_{false};
  bool is_initialized_ = false;
  std::string time_domain_ = "global";

  // lidar
  std::string lidar_format_ = "ANY";
  int lidar_rate_ = 0;
  std::string echo_mode_ = "";
  std::map<stream_index_pair, int> rate_int_;
  std::map<stream_index_pair, OBLiDARScanRate> rate_;
  std::map<stream_index_pair, std::string> frame_id_;
  float min_angle_ = -135.0;
  float max_angle_ = 135.0;
  float min_range_ = 0.05;
  float max_range_ = 30.0;
  int repetitive_scan_mode_ = -1;
  int filter_level_ = -1;
  float vertical_fov_ = -1;
  bool enable_scan_to_point_ = false;
  double angle_increment_ = 0.0;

  // Multi-frame publishing parameters
  int publish_n_pkts_ = 1;
  std::mutex frame_buffer_mutex_;
  std::vector<std::shared_ptr<ob::FrameSet>> frame_buffer_;

  // IMU
  bool enable_imu_ = false;
  std::string imu_rate_ = "50hz";
  std::string accel_range_ = "2g";
  std::string gyro_range_ = "1000dps";
  bool imu_sync_output_start_ = false;
  std::string accel_gyro_frame_id_ = "camera_imu_frame";
  double liner_accel_cov_ = 0.0001;
  double angular_vel_cov_ = 0.0001;
};
}  // namespace orbbec_lidar
}  // namespace orbbec_camera
