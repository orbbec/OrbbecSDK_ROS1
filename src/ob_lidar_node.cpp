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

#include "orbbec_camera/ob_lidar_node.h"
#include "orbbec_camera/utils.h"
#include "libobsensor/hpp/Utils.hpp"

namespace orbbec_camera {
namespace orbbec_lidar {
OBLidarNode::OBLidarNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                         std::shared_ptr<ob::Device> device)
    : nh_(nh),
      nh_private_(nh_private),
      device_(std::move(device)),
      device_info_(device_->getDeviceInfo()) {
  stream_name_[LIDAR] = "lidar";
  init();
}

void OBLidarNode::init() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  CHECK_NOTNULL(device_.get());
  is_running_ = true;
  getParameters();
  setupDevices();
  selectBaseStream();
  setupProfiles();
  setupPublishers();
  startStreams();
  is_initialized_ = true;
}

bool OBLidarNode::isInitialized() const { return is_initialized_; }

void OBLidarNode::rebootDevice() {
  ROS_INFO("Reboot device");
  if (device_) {
    device_->reboot();
  }
  ROS_INFO("Reboot device DONE");
}

void OBLidarNode::clean() {
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() start");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  is_running_ = false;
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() stop tf thread");
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }
  stopStreams();
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() stop stream");
  //   stopStreams();
  ROS_INFO_STREAM("OBLidarNode::~OBLidarNode() end");
}

void OBLidarNode::stopStreams() {
  if (!pipeline_started_ || !pipeline_) {
    ROS_INFO_STREAM("pipeline not started or not exist, skip stop pipeline");
    return;
  }
  try {
    pipeline_->stop();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to stop pipeline: " << e.getMessage());
  } catch (...) {
    ROS_ERROR_STREAM("Failed to stop pipeline");
  }

  // Stop IMU if enabled
  if (enable_imu_) {
    stopIMU();
  }
}

OBLidarNode::~OBLidarNode() noexcept { clean(); }

void OBLidarNode::getParameters() {
  camera_name_ = nh_private_.param<std::string>("camera_name", "lidar");
  for (const auto& stream_index : LIDAR_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_format";
    format_str_[stream_index] =
        nh_private_.param<std::string>(param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    param_name = stream_name_[stream_index] + "_rate";
    rate_int_[stream_index] = nh_private_.param<int>(param_name, rate_int_[stream_index]);
    rate_[stream_index] = OBScanRateFromInt(rate_int_[stream_index]);
    ROS_INFO_STREAM("Input rate: " << rate_[stream_index]
                                   << "  Input format: " << format_[stream_index]);
    param_name = stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    frame_id_[stream_index] = nh_private_.param<std::string>(param_name, default_frame_id);
    std::string default_optical_frame_id =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    optical_frame_id_[stream_index] =
        nh_private_.param<std::string>(param_name, default_optical_frame_id);
  }
  publish_tf_ = nh_private_.param<bool>("publish_tf", true);
  tf_publish_rate_ = nh_private_.param<double>("tf_publish_rate", 0.0);
  time_domain_ = nh_private_.param<std::string>("time_domain", "global");
  enable_heartbeat_ = nh_private_.param<bool>("enable_heartbeat", false);
  echo_mode_ = nh_private_.param<std::string>("echo_mode", "");
  min_angle_ = nh_private_.param<float>("min_angle", -135.0);
  max_angle_ = nh_private_.param<float>("max_angle", 135.0);
  min_range_ = nh_private_.param<float>("min_range", 0.05);
  max_range_ = nh_private_.param<float>("max_range", 30.0);
  repetitive_scan_mode_ = nh_private_.param<int>("repetitive_scan_mode", -1);
  filter_level_ = nh_private_.param<int>("filter_level", -1);
  vertical_fov_ = nh_private_.param<float>("vertical_fov", -1.0);
  enable_scan_to_point_ = nh_private_.param<bool>("enable_scan_to_point", false);

  // Multi-frame publishing parameter - only for LIDAR_POINT and LIDAR_SPHERE_POINT formats
  publish_n_pkts_ = nh_private_.param<int>("publish_n_pkts", 1);
  if (publish_n_pkts_ < 1 || publish_n_pkts_ > 12000) {
    ROS_WARN_STREAM("publish_n_pkts value " << publish_n_pkts_
                                            << " is out of range [1, 12000], setting to 1");
    publish_n_pkts_ = 1;
  }
  if (publish_n_pkts_ > 1)
    ROS_INFO_STREAM("Multi-frame publishing enabled: " << publish_n_pkts_
                                                       << " frames will be merged");

  // IMU parameters
  enable_imu_ = nh_private_.param<bool>("enable_imu", false);
  imu_rate_ = nh_private_.param<std::string>("imu_rate", "50hz");
  accel_range_ = nh_private_.param<std::string>("accel_range", "2g");
  gyro_range_ = nh_private_.param<std::string>("gyro_range", "1000dps");
  liner_accel_cov_ = nh_private_.param<double>("linear_accel_cov", 0.0001);
  angular_vel_cov_ = nh_private_.param<double>("angular_vel_cov", 0.0001);

  // Setup IMU streams if enabled
  if (enable_imu_) {
    enable_stream_[ACCEL] = true;
    enable_stream_[GYRO] = true;

    accel_gyro_frame_id_ = camera_name_ + "_imu_frame";
  }
}

void OBLidarNode::setupDevices() {
  auto sensor_list = device_->getSensorList();
  for (size_t i = 0; i < sensor_list->getCount(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profiles = sensor->getStreamProfileList();
    for (size_t j = 0; j < profiles->getCount(); j++) {
      auto profile = profiles->getProfile(j);
      stream_index_pair sip{profile->getType(), 0};
      if (sensors_.find(sip) != sensors_.end()) {
        continue;
      }
      sensors_[sip] = sensor;
    }
  }
  if (device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
    ROS_INFO_STREAM("Setting heartbeat to " << (enable_heartbeat_ ? "ON" : "OFF"));
    device_->setIntProperty(OB_PROP_HEARTBEAT_BOOL, enable_heartbeat_);
  }
  if (!echo_mode_.empty() &&
      device_->isPropertySupported(OB_PROP_LIDAR_SPECIFIC_MODE_INT, OB_PERMISSION_READ_WRITE)) {
    if (echo_mode_ == "Last Echo") {
      device_->setIntProperty(OB_PROP_LIDAR_SPECIFIC_MODE_INT, 0);
    } else if (echo_mode_ == "First Echo") {
      device_->setIntProperty(OB_PROP_LIDAR_SPECIFIC_MODE_INT, 1);
    }
    ROS_INFO_STREAM("Setting echo mode to "
                    << (device_->getIntProperty(OB_PROP_LIDAR_SPECIFIC_MODE_INT) ? "First Echo"
                                                                                 : "Last Echo"));
  }
  if (repetitive_scan_mode_ != -1 &&
      device_->isPropertySupported(OB_PROP_LIDAR_REPETITIVE_SCAN_MODE_INT,
                                   OB_PERMISSION_READ_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_LIDAR_REPETITIVE_SCAN_MODE_INT);
    if (repetitive_scan_mode_ < range.min || repetitive_scan_mode_ > range.max) {
      ROS_ERROR("repetitive scan mode value is out of range[%d,%d], please check the value",
                range.min, range.max);
    } else {
      device_->setIntProperty(OB_PROP_LIDAR_REPETITIVE_SCAN_MODE_INT, repetitive_scan_mode_);
      ROS_INFO_STREAM("Setting repetitive scan mode to "
                      << device_->getIntProperty(OB_PROP_LIDAR_REPETITIVE_SCAN_MODE_INT));
    }
  }
  if (filter_level_ != -1 &&
      device_->isPropertySupported(OB_PROP_LIDAR_TAIL_FILTER_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
    auto range = device_->getIntPropertyRange(OB_PROP_LIDAR_TAIL_FILTER_LEVEL_INT);
    if (filter_level_ < range.min || filter_level_ > range.max) {
      ROS_ERROR("filter level value is out of range[%d,%d], please check the value", range.min,
                range.max);
    } else {
      device_->setIntProperty(OB_PROP_LIDAR_TAIL_FILTER_LEVEL_INT, filter_level_);
      device_->setIntProperty(OB_PROP_LIDAR_APPLY_CONFIGS_INT, 1);
      ROS_INFO_STREAM("Setting filter level to "
                      << device_->getIntProperty(OB_PROP_LIDAR_TAIL_FILTER_LEVEL_INT));
    }
  }

  if (vertical_fov_ != -1.0 &&
      device_->isPropertySupported(OB_PROP_LIDAR_MEMS_FOV_SIZE_FLOAT, OB_PERMISSION_READ_WRITE)) {
    auto range = device_->getFloatPropertyRange(OB_PROP_LIDAR_MEMS_FOV_SIZE_FLOAT);
    if (vertical_fov_ < range.min || vertical_fov_ > range.max) {
      ROS_ERROR("vertical fov value is out of range[%f,%f], please check the value", range.min,
                range.max);
    } else {
      device_->setFloatProperty(OB_PROP_LIDAR_MEMS_FOV_SIZE_FLOAT, vertical_fov_);
      ROS_INFO_STREAM("Setting vertical fov to "
                      << device_->getFloatProperty(OB_PROP_LIDAR_MEMS_FOV_SIZE_FLOAT));
    }
  }
}

void OBLidarNode::selectBaseStream() {
  enable_stream_[LIDAR] = true;
  if (enable_stream_[LIDAR]) {
    base_stream_ = LIDAR;
  }
}

void OBLidarNode::setupProfiles() {
  for (const auto& elem : LIDAR_STREAMS) {
    if (enable_stream_[elem]) {
      const auto& sensor = sensors_[elem];
      CHECK_NOTNULL(sensor.get());
      auto profiles = sensor->getStreamProfileList();
      CHECK_NOTNULL(profiles.get());
      CHECK(profiles->getCount() > 0);
      for (size_t i = 0; i < profiles->getCount(); i++) {
        auto base_profile = profiles->getProfile(i)->as<ob::LiDARStreamProfile>();
        if (base_profile == nullptr) {
          throw std::runtime_error("Failed to get profile " + std::to_string(i));
        }
        auto profile = base_profile->as<ob::LiDARStreamProfile>();
        if (profile == nullptr) {
          throw std::runtime_error("Failed cast profile to LiDARStreamProfile");
        }
        ROS_DEBUG_STREAM("Sensor profile: "
                         << "stream_type: " << profile->getType() << "Scan Rate: "
                         << profile->getScanRate() << "Format:" << profile->getFormat());
        supported_profiles_[elem].emplace_back(profile);
      }
      std::shared_ptr<ob::LiDARStreamProfile> selected_profile;
      std::shared_ptr<ob::LiDARStreamProfile> default_profile;
      try {
        if (rate_[elem] == OB_LIDAR_SCAN_UNKNOWN && format_[elem] == OB_FORMAT_UNKNOWN) {
          selected_profile = profiles->getProfile(0)->as<ob::LiDARStreamProfile>();
        } else {
          selected_profile = profiles->getLiDARStreamProfile(rate_[elem], format_[elem]);
        }

      } catch (const ob::Error& ex) {
        ROS_ERROR_STREAM("Failed to get " << stream_name_[elem]
                                          << "  profile: " << ex.getMessage());
        ROS_ERROR_STREAM("Stream: " << elem.first << ", Stream Index: " << elem.second
                                    << ", Scan Rate: " << rate_[elem]
                                    << "Format:" << format_[elem]);
        ROS_INFO_STREAM("Available profiles:");
        printSensorProfiles(sensor);
        ROS_ERROR_STREAM("Because can not set this stream, so exit.");
        exit(-1);
      }
      if (!selected_profile) {
        ROS_WARN_STREAM("Given stream configuration is not supported by the device! "
                        << " Stream: " << elem.first << ", Stream Index: " << elem.second
                        << ", Scan Rate: " << rate_[elem]);
        if (default_profile) {
          ROS_WARN_STREAM("Using default profile instead.");
          ROS_WARN_STREAM("default scan Rate "
                          << default_profile->getScanRate()
                          << "default format:" << default_profile->getFormat());
          selected_profile = default_profile;
        } else {
          ROS_ERROR_STREAM(" NO default_profile found , Stream: " << elem.first
                                                                  << " will be disable");
          enable_stream_[elem] = false;
        }
      }
      CHECK_NOTNULL(selected_profile);
      stream_profile_[elem] = selected_profile;
      rate_[elem] = selected_profile->getScanRate();
      format_[elem] = selected_profile->getFormat();
      ROS_INFO_STREAM(" stream " << stream_name_[elem]
                                 << " is enabled - scan rate: " << selected_profile->getScanRate()
                                 << "  format:" << selected_profile->getFormat());
    }
  }
  // IMU
  for (const auto& stream_index : HID_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    try {
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      if (stream_index == ACCEL) {
        auto full_scale_range = fullAccelScaleRangeFromString(accel_range_);
        auto sample_rate = sampleRateFromString(imu_rate_);
        auto profile = profile_list->getAccelStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      } else if (stream_index == GYRO) {
        auto full_scale_range = fullGyroScaleRangeFromString(gyro_range_);
        auto sample_rate = sampleRateFromString(imu_rate_);
        auto profile = profile_list->getGyroStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      }
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " full scale range "
                                << (stream_index == ACCEL ? accel_range_ : gyro_range_)
                                << " sample rate " << imu_rate_);
    } catch (const ob::Error& e) {
      ROS_INFO_STREAM("Failed to setup " << stream_name_[stream_index]
                                         << " profile: " << e.getMessage());
      enable_stream_[stream_index] = false;
      stream_profile_[stream_index] = nullptr;
    }
  }
}
void OBLidarNode::setupPublishers() {
  if (!enable_scan_to_point_ && format_[LIDAR] == OB_FORMAT_LIDAR_SCAN) {
    scan_pub_ =
        std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::LaserScan>("scan/points", 10));
  } else if (enable_scan_to_point_ || format_[LIDAR] == OB_FORMAT_LIDAR_POINT ||
             format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT) {
    point_cloud_pub_ = std::make_shared<ros::Publisher>(
        nh_.advertise<sensor_msgs::PointCloud2>("cloud/points", 10));
  }

  if (enable_imu_) {
    std::string topic_name = "imu/sample";
    imu_publisher_ =
        std::make_shared<ros::Publisher>(nh_.advertise<sensor_msgs::Imu>(topic_name, 10));

    // for (const auto &stream_index : HID_STREAMS) {
    //   std::string info_topic_name = stream_name_[stream_index] + "/imu_info";
    //   imu_info_publishers_[stream_index] =
    //       nh_.advertise<orbbec_camera::IMUInfo>(info_topic_name, 1, true);
    // }

    // Setup lidar to imu extrinsics publisher
    std::string extrinsics_topic_name = "/" + camera_name_ + "/lidar_to_imu";
    lidar_to_imu_extrinsics_publisher_ =
        nh_.advertise<orbbec_camera::Extrinsics>(extrinsics_topic_name, 1, true);
  }
}

void OBLidarNode::startStreams() {
  if (pipeline_ != nullptr) {
    pipeline_.reset();
  }
  pipeline_ = std::make_shared<ob::Pipeline>(device_);

  try {
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to start pipeline: " << e.getMessage());
    setupPipelineConfig();
    pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
      onNewFrameSetCallback(frame_set);
    });
  } catch (...) {
    ROS_ERROR_STREAM("Failed to start pipeline");
    throw std::runtime_error("Failed to start pipeline");
  }
  pipeline_started_.store(true);

  // Start IMU if enabled
  if (enable_imu_) {
    startIMU();
  }
}

void OBLidarNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  for (const auto& stream_index : LIDAR_STREAMS) {
    if (enable_stream_[stream_index]) {
      ROS_INFO_STREAM("Enable " << stream_name_[stream_index] << " stream");
      auto profile = stream_profile_[stream_index]->as<ob::LiDARStreamProfile>();

      if (enable_stream_[stream_index]) {
        auto video_profile = profile;
        ROS_INFO_STREAM("lidar profile: " << video_profile->getScanRate() << "  "
                                          << video_profile->getFormat());
      }

      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " scan rate: "
                                << profile->getScanRate() << " format: " << profile->getFormat());
      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
  }
}

void OBLidarNode::onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set) {
  if (!is_running_.load()) {
    return;
  }
  if (!is_initialized_) {
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  try {
    ROS_INFO_ONCE("New frame received");
    if (!tf_published_ && !enable_imu_) {
      publishStaticTransforms();
      tf_published_ = true;
    }

    // Handle multi-frame publishing for LIDAR_POINT and LIDAR_SPHERE_POINT formats
    if ((format_[LIDAR] == OB_FORMAT_LIDAR_POINT ||
         format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT)) {
      if (publish_n_pkts_ == 1) {
        if (format_[LIDAR] == OB_FORMAT_LIDAR_POINT) {
          publishPointCloud(frame_set);
          return;
        } else if (format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT) {
          publishSpherePointCloud(frame_set);
          return;
        }
      }
      std::lock_guard<std::mutex> lock(frame_buffer_mutex_);
      frame_buffer_.push_back(frame_set);

      // If we have enough frames, publish merged point cloud
      if (frame_buffer_.size() >= static_cast<size_t>(publish_n_pkts_)) {
        if (format_[LIDAR] == OB_FORMAT_LIDAR_POINT) {
          publishMergedPointCloud();
        } else if (format_[LIDAR] == OB_FORMAT_LIDAR_SPHERE_POINT) {
          publishMergedSpherePointCloud();
        }
        frame_buffer_.clear();
      }
    } else {
      // Original single frame publishing logic
      if (format_[LIDAR] == OB_FORMAT_LIDAR_SCAN && !enable_scan_to_point_) {
        publishScan(frame_set);
      } else if (format_[LIDAR] == OB_FORMAT_LIDAR_SCAN && enable_scan_to_point_) {
        publishScanToPoint(frame_set);
      }
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: " << e.getMessage());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("onNewFrameSetCallback error: unknown error");
  }
}

void OBLidarNode::publishStaticTransforms() {
  if (!publish_tf_) {
    return;
  }
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

void OBLidarNode::publishDynamicTransforms() {
  ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
  std::mutex mu;
  std::unique_lock<std::mutex> lock(mu);
  while (ros::ok() && is_running_) {
    tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                    [this] { return (!(is_running_)); });
    {
      ros::Time t = ros::Time::now();
      for (auto& msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      CHECK_NOTNULL(dynamic_tf_broadcaster_.get());
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
}
void OBLidarNode::publishScan(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  if (frame_set == nullptr) {
    return;
  }
  if (angle_increment_ == 0.0) {
    angle_increment_ = getScanAngleIncrement(rate_[LIDAR]);
  }
  auto lidar_frame = frame_set->getFrame(OB_FRAME_LIDAR_POINTS);
  auto* scans_data = reinterpret_cast<OBLiDARScanPoint*>(lidar_frame->getData());
  auto scan_count = lidar_frame->getDataSize() / sizeof(OBLiDARScanPoint);
  auto frame_timestamp = getFrameTimestampUs(lidar_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto scan_msg = std::make_shared<sensor_msgs::LaserScan>();
  scan_msg->header.stamp = timestamp;
  scan_msg->header.frame_id = frame_id_[LIDAR];
  scan_msg->angle_min = 0.7853981852531433;
  scan_msg->angle_max = 5.495169162750244;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 1.0 / rate_int_[LIDAR] / scan_count;
  scan_msg->scan_time = 1.0 / rate_int_[LIDAR];
  scan_msg->range_min = min_range_;
  scan_msg->range_max = max_range_;
  scan_msg->ranges.resize(scan_count);
  scan_msg->intensities.resize(scan_count);
  for (size_t i = 0; i < scan_count; i++) {
    if (scans_data->distance < min_range_ && scans_data->distance > max_range_) {
      scans_data++;
      continue;
    }
    scan_msg->ranges[i] = scans_data[i].distance / 1000.0;
    scan_msg->intensities[i] = scans_data[i].intensity;
  }
  filterScan(*scan_msg);
  scan_pub_->publish(*scan_msg);
}

void OBLidarNode::publishScanToPoint(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  if (frame_set == nullptr) {
    return;
  }
  if (angle_increment_ == 0.0) {
    angle_increment_ = getScanAngleIncrement(rate_[LIDAR]);
  }
  auto lidar_frame = frame_set->getFrame(OB_FRAME_LIDAR_POINTS);
  auto* scans_data = reinterpret_cast<OBLiDARScanPoint*>(lidar_frame->getData());
  auto scan_count = lidar_frame->getDataSize() / sizeof(OBLiDARScanPoint);
  auto point_cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
  auto frame_timestamp = getFrameTimestampUs(lidar_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::UINT8);
  modifier.resize(scan_count);
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_[LIDAR];
  point_cloud_msg->height = 1;
  point_cloud_msg->width = scan_count;
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  for (size_t i = 0; i < scan_count; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
    double rad = 0.7853981852531433 + angle_increment_ * i;
    *iter_x = static_cast<float>(scans_data[i].distance * cos(rad) / 1000.0);
    *iter_y = static_cast<float>(scans_data[i].distance * sin(rad) / 1000.0);
    *iter_z = static_cast<float>(0.0);
    *iter_intensity = static_cast<uint8_t>(scans_data[i].intensity);
  }
  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(*point_cloud_msg);
}
void OBLidarNode::publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  if (frame_set == nullptr) {
    return;
  }
  auto lidar_frame = frame_set->getFrame(OB_FRAME_LIDAR_POINTS);
  auto* point_data = reinterpret_cast<OBLiDARPoint*>(lidar_frame->getData());
  auto point_count = lidar_frame->getDataSize() / sizeof(OBLiDARPoint);
  auto frame_timestamp = getFrameTimestampUs(lidar_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto point_cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::UINT8, "tag", 1,
      sensor_msgs::PointField::UINT8);
  modifier.resize(point_count);
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_[LIDAR];
  point_cloud_msg->height = 1;
  point_cloud_msg->width = point_count;
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(*point_cloud_msg, "tag");
  for (size_t i = 0; i < point_count;
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag) {
    *iter_x = static_cast<float>(point_data[i].x / 1000.0);
    *iter_y = static_cast<float>(point_data[i].y / 1000.0);
    *iter_z = static_cast<float>(point_data[i].z / 1000.0);
    *iter_intensity = point_data[i].reflectivity;
    *iter_tag = point_data[i].tag;
  }
  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(*point_cloud_msg);
}

void OBLidarNode::publishSpherePointCloud(std::shared_ptr<ob::FrameSet> frame_set) {
  (void)frame_set;
  if (frame_set == nullptr) {
    return;
  }
  auto lidar_frame = frame_set->getFrame(OB_FRAME_LIDAR_POINTS);
  auto* point_data = reinterpret_cast<OBLiDARSpherePoint*>(lidar_frame->getData());
  auto point_count = lidar_frame->getDataSize() / sizeof(OBLiDARSpherePoint);
  auto result_point = spherePointToPoint(point_data, point_count);
  auto frame_timestamp = getFrameTimestampUs(lidar_frame);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  auto point_cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2Fields(
      5, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::UINT8, "tag", 1,
      sensor_msgs::PointField::UINT8);
  modifier.resize(point_count);
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_[LIDAR];
  point_cloud_msg->height = 1;
  point_cloud_msg->width = point_count;
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(*point_cloud_msg, "tag");
  for (size_t i = 0; i < point_count;
       ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag) {
    *iter_x = static_cast<float>(result_point[i].x / 1000.0);
    *iter_y = static_cast<float>(result_point[i].y / 1000.0);
    *iter_z = static_cast<float>(result_point[i].z / 1000.0);
    *iter_intensity = result_point[i].reflectivity;
    *iter_tag = result_point[i].tag;
  }
  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(*point_cloud_msg);
}

std::vector<OBLiDARPoint> OBLidarNode::spherePointToPoint(OBLiDARSpherePoint* sphere_point,
                                                          uint32_t point_count) {
  std::vector<OBLiDARPoint> point_cloud;
  point_cloud.resize(point_count);
  OBLiDARPoint* point_data = point_cloud.data();
  for (uint32_t i = 0; i < point_count; ++i) {
    double theta_rad = sphere_point->theta * M_PI / 180.0f;  // to unit rad
    double phi_rad = sphere_point->phi * M_PI / 180.0f;      // to unit rad
    auto distance = sphere_point->distance;
    auto x = static_cast<float>(distance * cos(theta_rad) * cos(phi_rad));
    auto y = static_cast<float>(distance * sin(theta_rad) * cos(phi_rad));
    auto z = static_cast<float>(distance * sin(phi_rad));
    if (std::isfinite(x) && std::isfinite(y) && std::isfinite(y)) {
      // point: convert to opengl point
      point_data->x = x;
      point_data->y = y;
      point_data->z = z;
      point_data->reflectivity = sphere_point->reflectivity;
      point_data->tag = sphere_point->tag;
      ++point_data;
    }
    ++sphere_point;
  }
  return point_cloud;
}

uint64_t OBLidarNode::getFrameTimestampUs(const std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr) {
    ROS_WARN("getFrameTimestampUs: frame is nullptr, return 0");
    return 0;
  }
  if (time_domain_ == "device") {
    return frame->getTimeStampUs();
  } else if (time_domain_ == "global") {
    return frame->getGlobalTimeStampUs();
  } else {
    return frame->getSystemTimeStampUs();
  }
}

void OBLidarNode::filterScan(sensor_msgs::LaserScan& scan) {
  double current_angle = scan.angle_min;
  double max_angle = deg2rad(max_angle_);
  double min_angle = deg2rad(min_angle_);
  // map to 0 - 2 * M_PI
  max_angle = std::fmod(max_angle + M_PI, 2 * M_PI);
  if (max_angle < 0) {
    max_angle += 2 * M_PI;
  }

  min_angle = std::fmod(min_angle + M_PI, 2 * M_PI);
  if (min_angle < 0) {
    min_angle += 2 * M_PI;
  }
  if (min_angle > max_angle) {
    std::swap(min_angle, max_angle);
  }
  for (size_t i = 0; i < scan.ranges.size(); ++i, current_angle += scan.angle_increment) {
    bool is_angle_in_range = (current_angle >= min_angle && current_angle <= max_angle);

    bool is_range_in_range = (scan.ranges[i] >= min_range_ && scan.ranges[i] <= max_range_);

    if (!(is_angle_in_range && is_range_in_range)) {
      scan.ranges[i] = 0;
      scan.intensities[i] = 0;
    }
  }
}
sensor_msgs::PointCloud2 OBLidarNode::filterPointCloud(
    sensor_msgs::PointCloud2& point_cloud) const {
  // Initialize the filtered point cloud
  sensor_msgs::PointCloud2 filtered_point_cloud;
  filtered_point_cloud.header = point_cloud.header;
  filtered_point_cloud.height = point_cloud.height;
  filtered_point_cloud.width = point_cloud.width;
  filtered_point_cloud.is_dense = point_cloud.is_dense;
  filtered_point_cloud.is_bigendian = point_cloud.is_bigendian;
  filtered_point_cloud.fields = point_cloud.fields;
  filtered_point_cloud.point_step = point_cloud.point_step;

  // Convert filter angles from degrees to radians and normalize to [0, 2π]
  double max_angle = deg2rad(max_angle_);
  double min_angle = deg2rad(min_angle_);
  max_angle = std::fmod(max_angle + M_PI, 2 * M_PI);
  min_angle = std::fmod(min_angle + M_PI, 2 * M_PI);
  if (min_angle < 0) {
    min_angle += 2 * M_PI;
  }
  if (max_angle < 0) {
    max_angle += 2 * M_PI;
  }

  // Swap angles if min is greater than max
  if (min_angle > max_angle) {
    std::swap(min_angle, max_angle);
  }

  // Reserve space for filtered point cloud data
  filtered_point_cloud.data.reserve(point_cloud.data.size());

  // Create iterators for each field
  sensor_msgs::PointCloud2Iterator<float> iter_x(point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(point_cloud, "z");

  // Process each point
  for (size_t i = 0; i < point_cloud.height * point_cloud.width;
       ++i, ++iter_x, ++iter_y, ++iter_z) {
    float x = *iter_x;
    float y = *iter_y;
    float z = *iter_z;

    // Calculate distance from origin
    float distance = std::sqrt(x * x + y * y + z * z);

    // Calculate angle and normalize to [0, 2π]
    float angle = std::atan2(y, x);
    angle = std::fmod(angle + 2 * M_PI, 2 * M_PI);

    // Check if point is within both angle and range limits
    bool is_angle_in_range = (angle >= min_angle && angle <= max_angle);
    bool is_range_in_range = (distance >= min_range_ && distance <= max_range_);

    if (is_angle_in_range && is_range_in_range) {
      // Keep points within the specified range
      filtered_point_cloud.data.insert(filtered_point_cloud.data.end(),
                                       point_cloud.data.begin() + i * point_cloud.point_step,
                                       point_cloud.data.begin() + (i + 1) * point_cloud.point_step);
    } else {
      // Fill zero values for filtered out points
      filtered_point_cloud.data.insert(filtered_point_cloud.data.end(), point_cloud.point_step, 0);
    }
  }

  // Update row step and resize data
  filtered_point_cloud.row_step = filtered_point_cloud.width * filtered_point_cloud.point_step;
  filtered_point_cloud.data.resize(filtered_point_cloud.height * filtered_point_cloud.row_step);

  return filtered_point_cloud;
}
void OBLidarNode::printSensorProfiles(const std::shared_ptr<ob::Sensor>& sensor) {
  auto profiles = sensor->getStreamProfileList();
  for (size_t i = 0; i < profiles->getCount(); i++) {
    auto origin_profile = profiles->getProfile(i);
    if (sensor->getType() == OB_SENSOR_LIDAR) {
      auto profile = origin_profile->as<ob::LiDARStreamProfile>();
      ROS_INFO_STREAM("lidar scan rate: " << profile->getScanRate()
                                          << " format:" << profile->getFormat());
    } else {
      ROS_INFO_STREAM("unknown profile: " << sensor->getType());
    }
  }
}

void OBLidarNode::startIMU() {
  if (!enable_imu_) {
    return;
  }

  if (imuPipeline_ != nullptr) {
    imuPipeline_.reset();
  }

  imuPipeline_ = std::make_shared<ob::Pipeline>(device_);
  if (imu_sync_output_start_) {
    return;
  }

  try {
    // ACCEL
    auto accelProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_ACCEL);
    auto accel_range = fullAccelScaleRangeFromString(accel_range_);
    auto accel_rate = sampleRateFromString(imu_rate_);
    auto accelProfile = accelProfiles->getAccelStreamProfile(accel_range, accel_rate);

    // GYRO
    auto gyroProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_GYRO);
    auto gyro_range = fullGyroScaleRangeFromString(gyro_range_);
    auto gyro_rate = sampleRateFromString(imu_rate_);
    auto gyroProfile = gyroProfiles->getGyroStreamProfile(gyro_range, gyro_rate);

    std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
    imuConfig->enableStream(accelProfile);
    imuConfig->enableStream(gyroProfile);
    imuPipeline_->enableFrameSync();
    imuPipeline_->start(imuConfig, [this](std::shared_ptr<ob::Frame> frame) {
      auto frameSet = frame->as<ob::FrameSet>();
      auto aFrame = frameSet->getFrame(OB_FRAME_ACCEL);
      auto gFrame = frameSet->getFrame(OB_FRAME_GYRO);
      if (aFrame && gFrame) {
        onNewIMUFrameCallback(aFrame, gFrame);
      }
    });

    imu_sync_output_start_ = true;
    ROS_INFO_STREAM("Started IMU stream with accel range: "
                    << fullAccelScaleRangeToString(accel_range)
                    << ", gyro range: " << fullGyroScaleRangeToString(gyro_range)
                    << ", rate: " << sampleRateToString(accel_rate));

    // // Publish LiDAR to IMU extrinsics
    // publishLidarToIMUExtrinsics();

  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to start IMU stream: " << e.getMessage());
    imu_sync_output_start_ = false;
  }
}

void OBLidarNode::stopIMU() {
  if (imuPipeline_ && imu_sync_output_start_) {
    try {
      imuPipeline_->stop();
      imu_sync_output_start_ = false;
      ROS_INFO_STREAM("Stopped IMU stream");
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to stop IMU stream: " << e.getMessage());
    }
  }
}

void OBLidarNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame>& accelframe,
                                        const std::shared_ptr<ob::Frame>& gyroframe) {
  if (!imu_publisher_) {
    ROS_ERROR_STREAM("IMU publisher not initialized");
    return;
  }

  if (!tf_published_) {
    publishStaticTransforms();
    tf_published_ = true;
  }

  bool has_subscriber = imu_publisher_->getNumSubscribers() > 0;
  if (!has_subscriber) {
    return;
  }

  auto imu_msg = sensor_msgs::Imu();
  setDefaultIMUMessage(imu_msg);

  imu_msg.header.frame_id = accel_gyro_frame_id_;
  auto frame_timestamp = getFrameTimestampUs(accelframe);
  auto timestamp = fromUsToROSTime(frame_timestamp);
  imu_msg.header.stamp = timestamp;

  auto gyro_frame = gyroframe->as<ob::GyroFrame>();
  auto gyroData = gyro_frame->getValue();

  imu_msg.angular_velocity.x = -gyroData.x;
  imu_msg.angular_velocity.y = gyroData.y;
  imu_msg.angular_velocity.z = -gyroData.z;

  auto accel_frame = accelframe->as<ob::AccelFrame>();
  auto accelData = accel_frame->getValue();
  imu_msg.linear_acceleration.x = -accelData.x;
  imu_msg.linear_acceleration.y = accelData.y;
  imu_msg.linear_acceleration.z = -accelData.z;

  imu_publisher_->publish(imu_msg);

  // // Publish IMU info for each stream
  // for (const auto &stream_index : HID_STREAMS) {
  //   if (imu_info_publishers_.find(stream_index) != imu_info_publishers_.end()) {
  //     auto imu_info = createIMUInfo(stream_index);
  //     imu_info.header = imu_msg.header;
  //     imu_info_publishers_[stream_index].publish(imu_info);
  //   }
  // }
}

void OBLidarNode::setDefaultIMUMessage(sensor_msgs::Imu& imu_msg) {
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 1.0;

  // Set covariance matrices
  imu_msg.orientation_covariance[0] = -1;  // Not available

  // Linear acceleration covariance
  for (int i = 0; i < 9; ++i) {
    imu_msg.linear_acceleration_covariance[i] = 0.0;
  }
  imu_msg.linear_acceleration_covariance[0] = liner_accel_cov_;
  imu_msg.linear_acceleration_covariance[4] = liner_accel_cov_;
  imu_msg.linear_acceleration_covariance[8] = liner_accel_cov_;

  // Angular velocity covariance
  for (int i = 0; i < 9; ++i) {
    imu_msg.angular_velocity_covariance[i] = 0.0;
  }
  imu_msg.angular_velocity_covariance[0] = angular_vel_cov_;
  imu_msg.angular_velocity_covariance[4] = angular_vel_cov_;
  imu_msg.angular_velocity_covariance[8] = angular_vel_cov_;
}

orbbec_camera::IMUInfo OBLidarNode::createIMUInfo(const stream_index_pair& stream_index) {
  orbbec_camera::IMUInfo imu_info;
  imu_info.header.frame_id = optical_frame_id_[stream_index];

  // Set noise densities and bias
  imu_info.noise_density = 0.001;
  imu_info.random_walk = 0.0001;
  imu_info.bias[0] = 0.0;
  imu_info.bias[1] = 0.0;
  imu_info.bias[2] = 0.0;

  return imu_info;
}

void OBLidarNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);

  auto base_stream_profile = stream_profile_[base_stream_];
  if (!base_stream_profile) {
    ROS_ERROR_STREAM("Failed to get base stream profile");
    return;
  }

  // for (const auto& item : stream_profile_) {
  //   auto stream_index = item.first;
  //   auto stream_profile = item.second;
  //   if (!stream_profile) {
  //     continue;
  //   }

  //   OBExtrinsic ex;
  //   try {
  //     ex = stream_profile->getExtrinsicTo(base_stream_profile);
  //   } catch (const ob::Error& e) {
  //     ROS_ERROR_STREAM("Failed to get " << stream_name_[stream_index] << " extrinsic: " <<
  //     e.getMessage()); ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
  //   }

  //   auto Q = rotationMatrixToQuaternion(ex.rot);
  //   Q = quaternion_optical * Q * quaternion_optical.inverse();
  //   tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);
  //   auto timestamp = ros::Time::now();

  //   if (stream_index.first != base_stream_.first) {
  //     publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
  //   }
  //   publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
  //   optical_frame_id_[stream_index]);

  //   ROS_INFO_STREAM("Publishing static transform from " << stream_name_[stream_index] << " to "
  //   << stream_name_[base_stream_]); ROS_INFO_STREAM("Translation " << trans[0] << ", " <<
  //   trans[1] << ", " << trans[2]); ROS_INFO_STREAM("Rotation " << Q.getX() << ", " << Q.getY() <<
  //   ", " << Q.getZ() << ", " << Q.getW());
  // }

  // Handle IMU extrinsics - same as ROS2 implementation
  if (enable_imu_) {
    publishLidarToIMUExtrinsics();

    // Also publish static TF from lidar to IMU
    OBExtrinsic ex = lidar_to_imu_extrinsic_;
    auto Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);
    auto timestamp = ros::Time::now();
    publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], accel_gyro_frame_id_);

    ROS_INFO_STREAM("Publishing static transform from " << frame_id_[base_stream_] << " to "
                                                        << accel_gyro_frame_id_);
    ROS_INFO_STREAM("Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
    ROS_INFO_STREAM("Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ() << ", "
                                << Q.getW());
  }
}

void OBLidarNode::publishStaticTF(const ros::Time& t, const tf2::Vector3& trans,
                                  const tf2::Quaternion& q, const std::string& from,
                                  const std::string& to) {
  geometry_msgs::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans[2] / 1000.0;
  msg.transform.translation.y = -trans[0] / 1000.0;
  msg.transform.translation.z = -trans[1] / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBLidarNode::publishLidarToIMUExtrinsics() {
  if (!enable_imu_) {
    return;
  }

  ROS_INFO_STREAM("oups1");

  static const char* frame_id = "lidar_to_imu_extrinsics";
  OBExtrinsic ex;

  auto base_stream_profile = stream_profile_[base_stream_];
  if (!base_stream_profile) {
    ROS_ERROR_STREAM("Failed to get base stream profile");
    return;
  }
  try {
    // Try to get extrinsic from ACCEL first
    ex = base_stream_profile->getExtrinsicTo(stream_profile_[ACCEL]);

    // Verify if GYRO has the same extrinsic (they should be identical for the same IMU)
    try {
      auto gyro_ex = base_stream_profile->getExtrinsicTo(stream_profile_[GYRO]);
      // Check if ACCEL and GYRO extrinsics are identical
      bool extrinsics_match = true;

      for (int i = 0; i < 9; i++) {
        if (std::abs(ex.rot[i] - gyro_ex.rot[i]) > 1e-6) {
          extrinsics_match = false;
          break;
        }
      }

      for (int i = 0; i < 3; i++) {
        if (std::abs(ex.trans[i] - gyro_ex.trans[i]) > 1e-6) {
          extrinsics_match = false;
          break;
        }
      }

      if (!extrinsics_match) {
        ROS_WARN_STREAM("ACCEL and GYRO have different extrinsics, using ACCEL");
      } else {
        ROS_DEBUG_STREAM("ACCEL and GYRO extrinsics are identical");
      }
    } catch (const ob::Error& e) {
      ROS_WARN_STREAM("Could not get GYRO extrinsic for verification: " << e.getMessage());
    }

  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Failed to get ACCEL extrinsic, trying GYRO: " << e.getMessage());
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[GYRO]);
      ROS_INFO_STREAM("Using GYRO extrinsic for IMU");
    } catch (const ob::Error& e2) {
      ROS_INFO_STREAM("Failed to get "
                      << frame_id << " extrinsic from both ACCEL and GYRO: " << e2.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
  }

  lidar_to_imu_extrinsic_ = ex;

  // Publish the extrinsics
  orbbec_camera::Extrinsics extrinsics_msg;
  extrinsics_msg.header.frame_id = frame_id;
  extrinsics_msg.header.stamp = ros::Time::now();

  // Copy rotation matrix
  for (int i = 0; i < 9; i++) {
    extrinsics_msg.rotation[i] = lidar_to_imu_extrinsic_.rot[i];
  }

  // Copy translation vector
  for (int i = 0; i < 3; i++) {
    extrinsics_msg.translation[i] = lidar_to_imu_extrinsic_.trans[i];
  }

  lidar_to_imu_extrinsics_publisher_.publish(extrinsics_msg);
  ROS_INFO_STREAM("Published LiDAR to IMU extrinsics with actual device data");
}

void OBLidarNode::publishMergedPointCloud() {
  if (frame_buffer_.empty()) {
    return;
  }

  // Calculate total point count across all frames
  size_t total_point_count = 0;
  std::vector<std::pair<OBLiDARPoint*, size_t>> frame_data;
  std::vector<uint64_t> frame_timestamps;

  for (const auto& fs : frame_buffer_) {
    auto lidar_frame = fs->getFrame(OB_FRAME_LIDAR_POINTS);
    if (lidar_frame) {
      auto* point_data = reinterpret_cast<OBLiDARPoint*>(lidar_frame->getData());
      auto point_count = lidar_frame->getDataSize() / sizeof(OBLiDARPoint);
      frame_data.emplace_back(point_data, point_count);
      frame_timestamps.push_back(getFrameTimestampUs(lidar_frame));
      total_point_count += point_count;
    }
  }

  if (total_point_count == 0) {
    return;
  }

  // Create merged point cloud message with offset_time field
  auto point_cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::UINT8, "tag", 1,
      sensor_msgs::PointField::UINT8, "offset_time", 1, sensor_msgs::PointField::UINT32);
  modifier.resize(total_point_count);

  // Use the timestamp of the latest frame as the header timestamp
  auto timestamp = fromUsToROSTime(frame_timestamps.front());
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_[LIDAR];
  point_cloud_msg->height = 1;
  point_cloud_msg->width = total_point_count;
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);

  // Create iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(*point_cloud_msg, "tag");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(*point_cloud_msg, "offset_time");

  // Calculate frame time interval based on lidar rate
  double frame_interval_us = 1000000.0 / static_cast<double>(rate_int_[LIDAR]);

  // Merge all frames with per-point timestamps
  for (size_t frame_idx = 0; frame_idx < frame_data.size(); ++frame_idx) {
    auto* point_data = frame_data[frame_idx].first;
    auto point_count = frame_data[frame_idx].second;
    uint64_t frame_timestamp_us = frame_timestamps[frame_idx];

    // Calculate time increment per point within this frame (uniform sampling)
    double point_time_increment_us = frame_interval_us / static_cast<double>(point_count);

    for (size_t i = 0; i < point_count;
         ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_intensity, ++iter_tag, ++iter_offset_time) {
      *iter_x = static_cast<float>(point_data[i].x / 1000.0);
      *iter_y = static_cast<float>(point_data[i].y / 1000.0);
      *iter_z = static_cast<float>(point_data[i].z / 1000.0);
      *iter_intensity = point_data[i].reflectivity;
      *iter_tag = point_data[i].tag;

      // Calculate per-point offset time in nanoseconds relative to point cloud header timestamp
      double point_timestamp_us =
          static_cast<double>(frame_timestamp_us) + (i * point_time_increment_us);
      uint64_t header_timestamp_us = frame_timestamps[0];  // First frame timestamp
      *iter_offset_time =
          static_cast<uint32_t>((point_timestamp_us - static_cast<double>(header_timestamp_us)) *
                                1000.0);  // Convert to nanoseconds
    }
  }

  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(*point_cloud_msg);
}

void OBLidarNode::publishMergedSpherePointCloud() {
  if (frame_buffer_.empty()) {
    return;
  }

  // Calculate total point count across all frames
  size_t total_point_count = 0;
  std::vector<std::pair<OBLiDARSpherePoint*, size_t>> frame_data;
  std::vector<uint64_t> frame_timestamps;

  for (const auto& fs : frame_buffer_) {
    auto lidar_frame = fs->getFrame(OB_FRAME_LIDAR_POINTS);
    if (lidar_frame) {
      auto* point_data = reinterpret_cast<OBLiDARSpherePoint*>(lidar_frame->getData());
      auto point_count = lidar_frame->getDataSize() / sizeof(OBLiDARSpherePoint);
      frame_data.emplace_back(point_data, point_count);
      frame_timestamps.push_back(getFrameTimestampUs(lidar_frame));
      total_point_count += point_count;
    }
  }

  if (total_point_count == 0) {
    return;
  }

  // Convert sphere points to cartesian
  std::vector<OBLiDARPoint> all_converted_points;
  all_converted_points.reserve(total_point_count);

  for (const auto& frame_pair : frame_data) {
    auto* sphere_points = frame_pair.first;
    auto count = frame_pair.second;
    auto converted_points = spherePointToPoint(sphere_points, count);
    all_converted_points.insert(all_converted_points.end(), converted_points.begin(),
                                converted_points.end());
  }

  // Create merged point cloud message with offset_time field
  auto point_cloud_msg = std::make_shared<sensor_msgs::PointCloud2>();
  sensor_msgs::PointCloud2Modifier modifier(*point_cloud_msg);
  modifier.setPointCloud2Fields(
      6, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32, "z", 1,
      sensor_msgs::PointField::FLOAT32, "intensity", 1, sensor_msgs::PointField::UINT8, "tag", 1,
      sensor_msgs::PointField::UINT8, "offset_time", 1, sensor_msgs::PointField::UINT32);
  modifier.resize(total_point_count);

  // Use the timestamp of the latest frame as the header timestamp
  auto timestamp = fromUsToROSTime(frame_timestamps.front());
  point_cloud_msg->header.stamp = timestamp;
  point_cloud_msg->header.frame_id = frame_id_[LIDAR];
  point_cloud_msg->height = 1;
  point_cloud_msg->width = total_point_count;
  point_cloud_msg->is_dense = true;
  point_cloud_msg->is_bigendian = false;
  point_cloud_msg->row_step = point_cloud_msg->width * point_cloud_msg->point_step;
  point_cloud_msg->data.resize(point_cloud_msg->height * point_cloud_msg->row_step);

  // Create iterators
  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud_msg, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_intensity(*point_cloud_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_tag(*point_cloud_msg, "tag");
  sensor_msgs::PointCloud2Iterator<uint32_t> iter_offset_time(*point_cloud_msg, "offset_time");

  // Calculate frame time interval based on lidar rate
  double frame_interval_us = 1000000.0 / static_cast<double>(rate_int_[LIDAR]);

  // Merge all frames with per-point timestamps
  size_t point_idx = 0;
  for (size_t frame_idx = 0; frame_idx < frame_data.size(); ++frame_idx) {
    uint64_t frame_timestamp_us = frame_timestamps[frame_idx];
    size_t frame_point_count = frame_data[frame_idx].second;

    // Calculate time increment per point within this frame (uniform sampling)
    double point_time_increment_us = frame_interval_us / static_cast<double>(frame_point_count);

    for (size_t i = 0; i < frame_point_count; ++i, ++point_idx, ++iter_x, ++iter_y, ++iter_z,
                ++iter_intensity, ++iter_tag, ++iter_offset_time) {
      *iter_x = static_cast<float>(all_converted_points[point_idx].x / 1000.0);
      *iter_y = static_cast<float>(all_converted_points[point_idx].y / 1000.0);
      *iter_z = static_cast<float>(all_converted_points[point_idx].z / 1000.0);
      *iter_intensity = all_converted_points[point_idx].reflectivity;
      *iter_tag = all_converted_points[point_idx].tag;

      // Calculate per-point offset time in nanoseconds relative to point cloud header timestamp
      double point_timestamp_us =
          static_cast<double>(frame_timestamp_us) + (i * point_time_increment_us);
      uint64_t header_timestamp_us = frame_timestamps[0];  // First frame timestamp
      *iter_offset_time =
          static_cast<uint32_t>((point_timestamp_us - static_cast<double>(header_timestamp_us)) *
                                1000.0);  // Convert to nanoseconds
    }
  }

  *point_cloud_msg = filterPointCloud(*point_cloud_msg);
  point_cloud_pub_->publish(*point_cloud_msg);
}
}  // namespace orbbec_lidar
}  // namespace orbbec_camera
