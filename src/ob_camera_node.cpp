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

#include "orbbec_camera/ob_camera_node.h"
#if defined(USE_RK_HW_DECODER)
#include "orbbec_camera/rk_mpp_decoder.h"
#elif defined(USE_NV_HW_DECODER)
#include "orbbec_camera/jetson_nv_decoder.h"
#endif

namespace orbbec_camera {
OBCameraNode::OBCameraNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                           std::shared_ptr<ob::Device> device)
    : nh_(nh),
      nh_private_(nh_private),
      device_(std::move(device)),
      device_info_(device_->getDeviceInfo()) {
  stream_name_[COLOR] = "color";
  stream_name_[DEPTH] = "depth";
  stream_name_[INFRA0] = "ir";
  stream_name_[INFRA1] = "ir2";
  stream_name_[ACCEL] = "accel";
  stream_name_[GYRO] = "gyro";
  init();
}

void OBCameraNode::init() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  CHECK_NOTNULL(device_.get());
  is_running_ = true;
  setupConfig();
  getParameters();
  setupDevices();
  setupProfiles();
  setupCameraInfo();
  setupTopics();
  setupCameraCtrlServices();
  setupFrameCallback();
  readDefaultExposure();
  readDefaultGain();
  readDefaultWhiteBalance();
  is_initialized_ = true;
#if defined(USE_RK_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<RKMjpegDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_NV_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 3];
  rgb_is_decoded_ = false;
}

bool OBCameraNode::isInitialized() const { return is_initialized_; }

OBCameraNode::~OBCameraNode() {
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() start");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  is_running_ = false;
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() stop tf thread");
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }

  if (colorFrameThread_ && colorFrameThread_->joinable()) {
    colorFrameCV_.notify_all();
    colorFrameThread_->join();
  }

  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() stop stream");
  stopStreams();
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() delete rgb_buffer");
  delete[] rgb_buffer_;
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() end");
}

void OBCameraNode::getParameters() {
  camera_name_ = nh_private_.param<std::string>("camera_name", "camera");
  camera_link_frame_id_ = camera_name_ + "_link";
  for (const auto& stream_index : IMAGE_STREAMS) {
    frame_id_[stream_index] = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    optical_frame_id_[stream_index] =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    width_[stream_index] = nh_private_.param<int>(param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    height_[stream_index] = nh_private_.param<int>(param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    fps_[stream_index] = nh_private_.param<int>(param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    enable_stream_[stream_index] = nh_private_.param<bool>(param_name, false);
    param_name = "flip_" + stream_name_[stream_index];
    flip_images_[stream_index] = nh_private_.param<bool>(param_name, false);
    param_name = stream_name_[stream_index] + "_format";
    format_str_[stream_index] =
        nh_private_.param<std::string>(param_name, format_str_[stream_index]);
    format_[stream_index] = OBFormatFromString(format_str_[stream_index]);
    if (format_[stream_index] == OB_FORMAT_Y8) {
      CHECK(stream_index.first != OB_STREAM_COLOR);
      image_format_[stream_index] = CV_8UC1;
      encoding_[stream_index] = stream_index.first == OB_STREAM_DEPTH
                                    ? sensor_msgs::image_encodings::TYPE_8UC1
                                    : sensor_msgs::image_encodings::MONO8;
      unit_step_size_[stream_index] = sizeof(uint8_t);
    }
    if (format_[stream_index] == OB_FORMAT_MJPG) {
      if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
          stream_index.first == OB_STREAM_IR_RIGHT) {
        image_format_[stream_index] = CV_8UC1;
        encoding_[stream_index] = sensor_msgs::image_encodings::MONO8;
        unit_step_size_[stream_index] = sizeof(uint8_t);
      }
    }
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    depth_aligned_frame_id_[stream_index] = optical_frame_id_[COLOR];
  }
  publish_tf_ = nh_private_.param<bool>("publish_tf", false);
  depth_registration_ = nh_private_.param<bool>("depth_registration", false);
  enable_frame_sync_ = nh_private_.param<bool>("enable_frame_sync", false);
  ir_info_uri_ = nh_private_.param<std::string>("ir_info_uri", "");
  color_info_uri_ = nh_private_.param<std::string>("color_info_uri", "");
  enable_d2c_viewer_ = nh_private_.param<bool>("enable_d2c_viewer", false);
  enable_pipeline_ = nh_private_.param<bool>("enable_pipeline", false);
  enable_point_cloud_ = nh_private_.param<bool>("enable_point_cloud", true);
  enable_colored_point_cloud_ = nh_private_.param<bool>("enable_colored_point_cloud", false);
  enable_hardware_d2d_ = nh_private_.param<bool>("enable_hardware_d2d", true);
  depth_work_mode_ = nh_private_.param<std::string>("depth_work_mode", "");
  enable_soft_filter_ = nh_private_.param<bool>("enable_soft_filter", true);
  enable_color_auto_exposure_ = nh_private_.param<bool>("enable_color_auto_exposure", true);
  enable_ir_auto_exposure_ = nh_private_.param<bool>("enable_ir_auto_exposure", true);
  sync_mode_str_ = nh_private_.param<std::string>("sync_mode", "free_run");
  std::transform(sync_mode_str_.begin(), sync_mode_str_.end(), sync_mode_str_.begin(), ::toupper);
  sync_mode_ = OBSyncModeFromString(sync_mode_str_);
  depth_delay_us_ = nh_private_.param<int>("depth_delay_us", 0);
  color_delay_us_ = nh_private_.param<int>("color_delay_us", 0);
  trigger2image_delay_us_ = nh_private_.param<int>("trigger2image_delay_us", 0);
  trigger_out_delay_us_ = nh_private_.param<int>("trigger_out_delay_us", 0);
  trigger_out_enabled_ = nh_private_.param<bool>("trigger_out_enabled", false);
  depth_precision_str_ = nh_private_.param<std::string>("depth_precision", "1mm");
  depth_precision_ = DEPTH_PRECISION_STR2ENUM.at(depth_precision_str_);
  if (enable_colored_point_cloud_) {
    depth_registration_ = true;
  }
  enable_ldp_ = nh_private_.param<bool>("enable_ldp", true);
  soft_filter_max_diff_ = nh_private_.param<int>("soft_filter_max_diff", -1);
  soft_filter_speckle_size_ = nh_private_.param<int>("soft_filter_speckle_size", -1);
  for (const auto& stream_index : HID_STREAMS) {
    std::string param_name = "enable_" + stream_name_[stream_index];
    enable_stream_[stream_index] = nh_private_.param<bool>(param_name, false);
    param_name = stream_name_[stream_index] + "_rate";
    imu_rate_[stream_index] = nh_private_.param<std::string>(param_name, "");
    param_name = stream_name_[stream_index] + "_range";
    imu_range_[stream_index] = nh_private_.param<std::string>(param_name, "");
    param_name = camera_name_ + "_" + stream_name_[stream_index] + "_frame_id";
    std::string default_frame_id = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    frame_id_[stream_index] = nh_private_.param<std::string>(param_name, default_frame_id);
    std::string default_optical_frame_id =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    optical_frame_id_[stream_index] =
        nh_private_.param<std::string>(param_name, default_optical_frame_id);
    depth_aligned_frame_id_[stream_index] = stream_name_[COLOR] + "_optical_frame";
  }
}

void OBCameraNode::startStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    CHECK_NOTNULL(pipeline_.get());
    if (enable_frame_sync_) {
      pipeline_->enableFrameSync();
    }
    try {
      setupPipelineConfig();
      pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
        CHECK_NOTNULL(frame_set.get());
        this->onNewFrameSetCallback(frame_set);
      });
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("failed to start pipeline: " << e.getMessage()
                                                    << " try to disable ir stream try again");
      enable_stream_[INFRA0] = false;
      setupPipelineConfig();
      pipeline_->start(pipeline_config_, [this](const std::shared_ptr<ob::FrameSet>& frame_set) {
        CHECK_NOTNULL(frame_set.get());
        this->onNewFrameSetCallback(frame_set);
      });
    } catch (...) {
      ROS_ERROR_STREAM("failed to start pipeline");
      throw;
    }

    if (!colorFrameThread_ && enable_stream_[COLOR]) {
      ROS_INFO_STREAM("Create color frame read thread.");
      colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
    }
    pipeline_started_ = true;
  } else {
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (enable_stream_[stream_index] && !stream_started_[stream_index]) {
        startStream(stream_index);
      }
    }
  }
}

void OBCameraNode::startAccel() {
  const stream_index_pair stream_index = ACCEL;
  if (!enable_stream_[stream_index]) {
    return;
  }
  if (!sensors_[stream_index] || !imu_sensor_[stream_index]) {
    ROS_ERROR_STREAM("accel sensor is null");
    return;
  }
  auto profile_list = sensors_[ACCEL]->getStreamProfileList();
  for (size_t i = 0; i < profile_list->count(); i++) {
    auto item = profile_list->getProfile(i);
    auto profile = item->as<ob::AccelStreamProfile>();
    auto accel_rate = sampleRateFromString(imu_rate_[stream_index]);
    auto accel_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
    if (profile->fullScaleRange() == accel_range && profile->sampleRate() == accel_rate) {
      imu_sensor_[stream_index]->start(
          profile, [this, stream_index](const std::shared_ptr<ob::Frame>& frame) {
            onNewIMUFrameCallback(frame, stream_index);
          });
      imu_started_[stream_index] = true;
      ROS_INFO_STREAM("start accel stream with " << fullAccelScaleRangeToString(accel_range)
                                                 << " range and " << sampleRateToString(accel_rate)
                                                 << " rate");
    }
  }
  if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
    ROS_INFO_STREAM("Failed to start IMU stream: "
                    << stream_name_[stream_index]
                    << ", please check the imu_rate and imu_range parameters");
  }
}

void OBCameraNode::startGyro() {
  const stream_index_pair stream_index = GYRO;
  if (!enable_stream_[stream_index]) {
    return;
  }
  if (!sensors_[stream_index] || !imu_sensor_[stream_index]) {
    ROS_ERROR_STREAM("gyro sensor is null");
  }
  auto profile_list = sensors_[GYRO]->getStreamProfileList();
  for (size_t i = 0; i < profile_list->count(); i++) {
    auto item = profile_list->getProfile(i);
    auto profile = item->as<ob::GyroStreamProfile>();
    auto gyro_rate = sampleRateFromString(imu_rate_[stream_index]);
    auto gyro_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
    if (profile->fullScaleRange() == gyro_range && profile->sampleRate() == gyro_rate) {
      imu_sensor_[stream_index]->start(
          profile, [this, stream_index](const std::shared_ptr<ob::Frame>& frame) {
            onNewIMUFrameCallback(frame, stream_index);
          });
      imu_started_[stream_index] = true;
      ROS_INFO_STREAM("start gyro stream with " << fullGyroScaleRangeToString(gyro_range)
                                                << " range and " << sampleRateToString(gyro_rate)
                                                << " rate");
    }
  }
  if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
    ROS_INFO_STREAM("Failed to start IMU stream: "
                    << stream_name_[stream_index]
                    << ", please check the imu_rate and imu_range parameters");
  }
}

void OBCameraNode::startIMU(const stream_index_pair& stream_index) {
  if (stream_index == ACCEL) {
    startAccel();
  } else if (stream_index == GYRO) {
    startGyro();
  }
}
void OBCameraNode::startIMU() {
  for (const auto& stream_index : HID_STREAMS) {
    if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
      CHECK(sensors_.count(stream_index));
      CHECK(imu_sensor_.count(stream_index));
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      for (size_t i = 0; i < profile_list->count(); i++) {
        auto item = profile_list->getProfile(i);
        if (stream_index == ACCEL) {
          auto profile = item->as<ob::AccelStreamProfile>();
          auto accel_rate = sampleRateFromString(imu_rate_[stream_index]);
          auto accel_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
          if (profile->fullScaleRange() == accel_range && profile->sampleRate() == accel_rate) {
            imu_sensor_[stream_index]->start(
                profile, [this, stream_index](const std::shared_ptr<ob::Frame>& frame) {
                  onNewIMUFrameCallback(frame, stream_index);
                });
            imu_started_[stream_index] = true;
            ROS_INFO_STREAM("start accel stream with "
                            << fullAccelScaleRangeToString(accel_range) << " range and "
                            << sampleRateToString(accel_rate) << " rate");
          }
        } else if (stream_index == GYRO) {
          auto profile = item->as<ob::GyroStreamProfile>();
          auto gyro_rate = sampleRateFromString(imu_rate_[stream_index]);
          auto gyro_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
          if (profile->fullScaleRange() == gyro_range && profile->sampleRate() == gyro_rate) {
            imu_sensor_[stream_index]->start(
                profile, [this, stream_index](const std::shared_ptr<ob::Frame>& frame) {
                  onNewIMUFrameCallback(frame, stream_index);
                });
            ROS_INFO_STREAM("start gyro stream with " << fullGyroScaleRangeToString(gyro_range)
                                                      << " range and "
                                                      << sampleRateToString(gyro_rate) << " rate");
            imu_started_[stream_index] = true;
          }
        }
      }
    }
  }
  for (const auto& stream_index : HID_STREAMS) {
    if (enable_stream_[stream_index] && !imu_started_[stream_index]) {
      ROS_INFO_STREAM("Failed to start IMU stream: "
                      << stream_name_[stream_index]
                      << ", please check the imu_rate and imu_range parameters");
    }
  }
}

void OBCameraNode::stopStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    CHECK_NOTNULL(pipeline_.get());
    pipeline_->stop();
    pipeline_started_ = false;
  } else {
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (stream_started_[stream_index]) {
        stopStream(stream_index);
      }
    }
  }
}

void OBCameraNode::stopIMU(const orbbec_camera::stream_index_pair& stream_index) {
  if (imu_started_[stream_index]) {
    CHECK(sensors_.count(stream_index));
    ROS_INFO_STREAM("stop " << stream_name_[stream_index] << " stream");
    imu_sensor_[stream_index]->stop();
    imu_started_[stream_index] = false;
  }
}

void OBCameraNode::stopIMU() {
  for (const auto& stream_index : HID_STREAMS) {
    if (imu_started_[stream_index]) {
      stopIMU(stream_index);
    }
  }
}

void OBCameraNode::startStream(const stream_index_pair& stream_index) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    ROS_WARN_STREAM("Cannot start stream when pipeline is enabled");
    return;
  }
  if (!enable_stream_[stream_index]) {
    ROS_WARN_STREAM("Stream " << stream_name_[stream_index] << " is not enabled, cannot start it.");
    return;
  }
  if (stream_started_[stream_index]) {
    ROS_WARN_STREAM("Stream " << stream_name_[stream_index] << " is already started.");
    return;
  }
  ROS_INFO_STREAM("Starting stream " << stream_name_[stream_index] << "...");
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (!has_subscriber) {
    ROS_INFO_STREAM("No subscriber for stream " << stream_name_[stream_index] << ", skip it.");
    return;
  }
  CHECK_GE(stream_profile_.count(stream_index), 0u);
  CHECK_GE(frame_callback_.count(stream_index), 0u);
  CHECK_GE(sensors_.count(stream_index), 0u);
  auto callback = frame_callback_[stream_index];
  auto profile = stream_profile_[stream_index];
  try {
    sensors_[stream_index]->startStream(profile, callback);
    stream_started_[stream_index] = true;

    if (!colorFrameThread_ && stream_index == COLOR) {
      ROS_INFO_STREAM("Create color frame read thread.");
      colorFrameThread_ = std::make_shared<std::thread>([this]() { onNewColorFrameCallback(); });
    }
    ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " started.");
  } catch (...) {
    ROS_ERROR_STREAM("Failed to start stream " << stream_name_[stream_index] << ".");
  }
}

void OBCameraNode::stopStream(const stream_index_pair& stream_index) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    ROS_WARN_STREAM("Cannot stop stream when pipeline is enabled");
    return;
  }
  if (!stream_started_[stream_index]) {
    ROS_WARN_STREAM("Stream " << stream_name_[stream_index] << " is not started.");
    return;
  }
  ROS_INFO_STREAM("Stopping stream " << stream_name_[stream_index] << "...");
  sensors_[stream_index]->stopStream();
  stream_started_[stream_index] = false;
  ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " stopped.");
}

void OBCameraNode::publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set, bool isColorPointCloud) {
  try {
    if (isColorPointCloud) {
      if (depth_registration_ || enable_colored_point_cloud_) {
        if (frame_set->depthFrame() != nullptr && frame_set->colorFrame() != nullptr) {
          publishColoredPointCloud(frame_set);
        }
      }
    }

    if (!isColorPointCloud) {
      if (frame_set->depthFrame() != nullptr) {
        publishDepthPointCloud(frame_set);
      }
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM(e.getMessage());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(e.what());
  } catch (...) {
    ROS_ERROR_STREAM("publishPointCloud with unknown error");
  }
}

void OBCameraNode::publishDepthPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (depth_cloud_pub_.getNumSubscribers() == 0 || !enable_point_cloud_) {
    return;
  }

  if(!camera_params_) {
    camera_params_ = pipeline_->getCameraParam();
  }

  if (!camera_params_) {
    ROS_ERROR_STREAM("camera_params_ is null");
    return;
  }
  auto depth_frame = frame_set->depthFrame();
  if (!depth_frame) {
    ROS_ERROR_STREAM("depth frame is null");
    return;
  }
  auto width = depth_frame->width();
  auto height = depth_frame->height();
  float fdx =
      camera_params_->depthIntrinsic.fx * ((float)(width) / camera_params_->depthIntrinsic.width);
  float fdy =
      camera_params_->depthIntrinsic.fy * ((float)(height) / camera_params_->depthIntrinsic.height);
  fdx = 1 / fdx;
  fdy = 1 / fdy;
  float u0 =
      camera_params_->depthIntrinsic.cx * ((float)(width) / camera_params_->depthIntrinsic.width);
  float v0 =
      camera_params_->depthIntrinsic.cy * ((float)(height) / camera_params_->depthIntrinsic.height);

  const auto* depth_data = (uint16_t*)depth_frame->data();
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(width * height);
  cloud_msg_.width = depth_frame->width();
  cloud_msg_.height = depth_frame->height();
  cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
  cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
  size_t valid_count = 0;
  const static float MIN_DISTANCE = 20.0;
  const static float MAX_DISTANCE = 10000.0;
  double depth_scale = depth_frame->getValueScale();
  const static float min_depth = MIN_DISTANCE / depth_scale;
  const static float max_depth = MAX_DISTANCE / depth_scale;
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      if (depth_data[y * width + x] < min_depth || depth_data[y * width + x] > max_depth) {
        continue;
      }
      float xf = (x - u0) * fdx;
      float yf = (y - v0) * fdy;
      float zf = depth_data[y * width + x] * depth_scale;
      *iter_x = zf * xf / 1000.0;
      *iter_y = zf * yf / 1000.0;
      *iter_z = zf / 1000.0;
      ++iter_x, ++iter_y, ++iter_z;
      valid_count++;
    }
  }
  auto timestamp = frameTimeStampToROSTime(depth_frame->systemTimeStamp());
  std::string frame_id =
      depth_registration_ ? depth_aligned_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  cloud_msg_.header.stamp = timestamp;
  cloud_msg_.header.frame_id = frame_id;
  cloud_msg_.is_dense = true;
  cloud_msg_.width = valid_count;
  cloud_msg_.height = 1;
  modifier.resize(valid_count);
  depth_cloud_pub_.publish(cloud_msg_);
  if (save_point_cloud_) {
    save_point_cloud_ = false;
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    std::string filename = current_path + "/point_cloud/points_" + ss.str() + ".ply";
    if (!boost::filesystem::exists(current_path + "/point_cloud")) {
      boost::filesystem::create_directory(current_path + "/point_cloud");
    }
    ROS_INFO_STREAM("Saving point cloud to " << filename);
    saveDepthPointCloudMsgToPly(cloud_msg_, filename);
  }
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (depth_registered_cloud_pub_.getNumSubscribers() == 0 || !enable_colored_point_cloud_) {
    return;
  }
  auto depth_frame = frame_set->depthFrame();
  auto color_frame = frame_set->colorFrame();
  if (!depth_frame || !color_frame) {
    return;
  }
  auto depth_width = depth_frame->width();
  auto depth_height = depth_frame->height();
  auto color_width = color_frame->width();
  auto color_height = color_frame->height();
  if (depth_width != color_width || depth_height != color_height) {
    ROS_ERROR_STREAM("depth frame size is not equal to color frame size");
    return;
  }
  CHECK_NOTNULL(pipeline_.get());
  if (!camera_params_) {
    camera_params_ = pipeline_->getCameraParam();
  }
  CHECK(camera_params_);
  float fdx =
      camera_params_->rgbIntrinsic.fx * ((float)(color_width) / camera_params_->rgbIntrinsic.width);
  float fdy = camera_params_->rgbIntrinsic.fy *
              ((float)(color_height) / camera_params_->rgbIntrinsic.height);
  fdx = 1 / fdx;
  fdy = 1 / fdy;
  float u0 =
      camera_params_->rgbIntrinsic.cx * ((float)(color_width) / camera_params_->rgbIntrinsic.width);
  float v0 = camera_params_->rgbIntrinsic.cy *
             ((float)(color_height) / camera_params_->rgbIntrinsic.height);
  const auto* depth_data = (uint16_t*)depth_frame->data();
  const auto* color_data = (uint8_t*)(rgb_buffer_);
  sensor_msgs::PointCloud2Modifier modifier(cloud_msg_);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  cloud_msg_.width = color_frame->width();
  cloud_msg_.height = color_frame->height();
  std::string format_str = "rgb";
  cloud_msg_.point_step = addPointField(cloud_msg_, format_str, 1, sensor_msgs::PointField::FLOAT32,
                                        static_cast<int>(cloud_msg_.point_step));
  cloud_msg_.row_step = cloud_msg_.width * cloud_msg_.point_step;
  cloud_msg_.data.resize(cloud_msg_.height * cloud_msg_.row_step);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg_, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg_, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg_, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud_msg_, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud_msg_, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud_msg_, "b");
  size_t valid_count = 0;
  static const float MIN_DISTANCE = 20.0;
  static const float MAX_DISTANCE = 10000.0;
  double depth_scale = depth_frame->getValueScale();
  static float min_depth = MIN_DISTANCE / depth_scale;
  static float max_depth = MAX_DISTANCE / depth_scale;
  for (int y = 0; y < color_height; y++) {
    for (int x = 0; x < color_width; x++) {
      float depth = depth_data[y * depth_width + x];
      if (depth < min_depth || depth > max_depth) {
        continue;
      }
      float xf = (x - u0) * fdx;
      float yf = (y - v0) * fdy;
      float zf = depth * depth_scale;
      *iter_x = zf * xf / 1000.0;
      *iter_y = zf * yf / 1000.0;
      *iter_z = zf / 1000.0;
      *iter_r = color_data[(y * color_width + x) * 3];
      *iter_g = color_data[(y * color_width + x) * 3 + 1];
      *iter_b = color_data[(y * color_width + x) * 3 + 2];
      ++iter_x;
      ++iter_y;
      ++iter_z;
      ++iter_r;
      ++iter_g;
      ++iter_b;
      ++valid_count;
    }
  }

  if (valid_count == 0) {
    return;
  }
  auto timestamp = frameTimeStampToROSTime(depth_frame->systemTimeStamp());
  cloud_msg_.header.stamp = timestamp;
  cloud_msg_.header.frame_id = optical_frame_id_[COLOR];
  cloud_msg_.is_dense = true;
  cloud_msg_.width = valid_count;
  cloud_msg_.height = 1;
  modifier.resize(valid_count);
  depth_registered_cloud_pub_.publish(cloud_msg_);
  if (save_colored_point_cloud_) {
    save_colored_point_cloud_ = false;
    auto now = std::time(nullptr);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    std::string filename = current_path + "/point_cloud/colored_points_" + ss.str() + ".ply";
    if (!boost::filesystem::exists(current_path + "/point_cloud")) {
      boost::filesystem::create_directory(current_path + "/point_cloud");
    }
    ROS_INFO_STREAM("Saving point cloud to " << filename);
    saveRGBPointCloudMsgToPly(cloud_msg_, filename);
  }
}

void OBCameraNode::setDefaultIMUMessage(sensor_msgs::Imu& imu_msg) {
  imu_msg.header.frame_id = "imu_link";
  imu_msg.orientation.x = 0.0;
  imu_msg.orientation.y = 0.0;
  imu_msg.orientation.z = 0.0;
  imu_msg.orientation.w = 0.0;

  imu_msg.orientation_covariance = {-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  imu_msg.linear_acceleration_covariance = {
      liner_accel_cov_, 0.0, 0.0, 0.0, liner_accel_cov_, 0.0, 0.0, 0.0, liner_accel_cov_};
  imu_msg.angular_velocity_covariance = {
      angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_, 0.0, 0.0, 0.0, angular_vel_cov_};
}

sensor_msgs::Imu OBCameraNode::createUnitIMUMessage(const IMUData& accel_data,
                                                    const IMUData& gyro_data) {
  sensor_msgs::Imu imu_msg;
  ros::Time timestamp = ros::Time::now();
  imu_msg.header.stamp = timestamp;
  imu_msg.angular_velocity.x = gyro_data.data_.x();
  imu_msg.angular_velocity.y = gyro_data.data_.y();
  imu_msg.angular_velocity.z = gyro_data.data_.z();

  imu_msg.linear_acceleration.x = accel_data.data_.x();
  imu_msg.linear_acceleration.y = accel_data.data_.y();
  imu_msg.linear_acceleration.z = accel_data.data_.z();
  return imu_msg;
}

void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                                         const stream_index_pair& stream_index) {
  if (!imu_publishers_.count(stream_index)) {
    ROS_ERROR_STREAM("stream " << stream_name_[stream_index] << " publisher not initialized");
    return;
  }
  auto subscriber_count = imu_publishers_[stream_index].getNumSubscribers();
  if (subscriber_count == 0) {
    return;
  }
  auto imu_msg = sensor_msgs::Imu();
  setDefaultIMUMessage(imu_msg);
  imu_msg.header.frame_id = optical_frame_id_[stream_index];
  auto timestamp = frameTimeStampToROSTime(frame->systemTimeStamp());
  imu_msg.header.stamp = timestamp;
  if (frame->type() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->value();
    imu_msg.angular_velocity.x = data.x;
    imu_msg.angular_velocity.y = data.y;
    imu_msg.angular_velocity.z = data.z;
  } else if (frame->type() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->value();
    imu_msg.linear_acceleration.x = data.x;
    imu_msg.linear_acceleration.y = data.y;
    imu_msg.linear_acceleration.z = data.z;
  } else {
    ROS_ERROR("Unsupported IMU frame type");
    return;
  }
  imu_publishers_[stream_index].publish(imu_msg);
}

bool OBCameraNode::decodeColorFrameToBuffer(const std::shared_ptr<ob::Frame>& frame,
                                            uint8_t* dest) {
  if (!rgb_buffer_) {
    return false;
  }
  bool has_subscriber = image_publishers_[COLOR].getNumSubscribers() > 0;
  if (enable_colored_point_cloud_ && depth_registered_cloud_pub_.getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return false;
  }
  bool is_decoded = false;
  if (!frame) {
    return false;
  }
#if defined(USE_RK_HW_DECODER) || defined(USE_NV_HW_DECODER)
  if (frame && frame->format() != OB_FORMAT_RGB888) {
    if (frame->format() == OB_FORMAT_MJPG && mjpeg_decoder_) {
      CHECK_NOTNULL(mjpeg_decoder_.get());
      CHECK_NOTNULL(rgb_buffer_);
      auto video_frame = frame->as<ob::ColorFrame>();
      bool ret = mjpeg_decoder_->decode(video_frame, rgb_buffer_);
      if (!ret) {
        ROS_ERROR_STREAM("Decode frame failed");
        is_decoded = false;

      } else {
        is_decoded = true;
      }
    }
  }
#endif
  if (!is_decoded) {
    auto video_frame = softwareDecodeColorFrame(frame);
    if (!video_frame) {
      ROS_ERROR_STREAM("Decode frame failed");
      return false;
    }
    CHECK_NOTNULL(rgb_buffer_);
    memcpy(dest, video_frame->data(), video_frame->dataSize());
    return true;
  }
  return true;
}

std::shared_ptr<ob::Frame> OBCameraNode::decodeIRMJPGFrame(const std::shared_ptr<ob::Frame> &frame) {
  if (frame->format() == OB_FORMAT_MJPEG &&
      (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
       frame->type() == OB_FRAME_IR_RIGHT)) {
    auto video_frame = frame->as<ob::IRFrame>();

    cv::Mat mjpgMat(1, video_frame->dataSize(), CV_8UC1, video_frame->data());
    cv::Mat irRawMat = cv::imdecode(mjpgMat, cv::IMREAD_GRAYSCALE);

    std::shared_ptr<ob::Frame> irFrame = ob::FrameHelper::createFrame(
        video_frame->type(), video_frame->format(), video_frame->width(), video_frame->height(), 0);

    uint32_t buffer_size = irRawMat.rows * irRawMat.cols * irRawMat.channels();

    if(buffer_size > irFrame->dataSize()) {
      ROS_ERROR_STREAM("Insufficient buffer size allocation,failed to decode ir mjpg frame!");
      return nullptr;
    }

    memcpy(irFrame->data(), irRawMat.data, buffer_size);
    ob::FrameHelper::setFrameDeviceTimestamp(irFrame, video_frame->timeStamp());
    ob::FrameHelper::setFrameDeviceTimestampUs(irFrame, video_frame->timeStampUs());
    ob::FrameHelper::setFrameSystemTimestamp(irFrame, video_frame->systemTimeStamp());
    return irFrame;
  }

  return nullptr;
}

void OBCameraNode::onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!is_running_) {
    // is_running_ is false means the node is shutting down
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  try {
    //rgb_is_decoded_ = decodeColorFrameToBuffer(frame_set->colorFrame(), rgb_buffer_);
    std::shared_ptr<ob::ColorFrame> colorFrame = frame_set->colorFrame();
    if (enable_stream_[COLOR] && colorFrame){
      std::lock_guard<std::mutex> colorLock(colorFrameMtx_);
      colorFrameQueue_.push(frame_set);
      colorFrameCV_.notify_all();
    }

    publishPointCloud(frame_set, false);
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (enable_stream_[stream_index]) {
        auto frame_type = STREAM_TYPE_TO_FRAME_TYPE.at(stream_index.first);
        if (frame_type == OB_FRAME_COLOR) {
          continue;
        }

        auto frame = frame_set->getFrame(frame_type);
        if (frame == nullptr) {
          ROS_DEBUG_STREAM("frame type " << frame_type << " is null");
          continue;
        }

        std::shared_ptr<ob::Frame> irFrame = decodeIRMJPGFrame(frame);
        if(irFrame) {
          onNewFrameCallback(irFrame, stream_index);
        } else {
          onNewFrameCallback(frame, stream_index);
        }
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

void OBCameraNode::onNewColorFrameCallback() {
  while (enable_stream_[COLOR] && ros::ok() && is_running_.load()) {
    std::unique_lock<std::mutex> lock(colorFrameMtx_);
    colorFrameCV_.wait(lock,
                       [this]() { return !colorFrameQueue_.empty() || !(is_running_.load()); });

    if (!ros::ok() || !is_running_.load()) {
      break;
    }

    std::shared_ptr<ob::FrameSet> frameSet = colorFrameQueue_.front();
    rgb_is_decoded_ = decodeColorFrameToBuffer(frameSet->colorFrame(), rgb_buffer_);
    publishPointCloud(frameSet, true);
    onNewFrameCallback(frameSet->colorFrame(), IMAGE_STREAMS.at(2));
    colorFrameQueue_.pop();
  }

  ROS_INFO_STREAM("Color frame thread exit!");
}

std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame>& frame) {
  if (frame->format() == OB_FORMAT_RGB || frame->format() == OB_FORMAT_BGR) {
    return frame;
  }
  if (!setupFormatConvertType(frame->format())) {
    ROS_ERROR_STREAM("Unsupported color format: " << frame->format());
    return nullptr;
  }
  auto covert_frame = format_convert_filter_.process(frame);
  if (covert_frame == nullptr) {
    ROS_ERROR_STREAM("Format " << frame->format() << " convert to RGB888 failed");
    return nullptr;
  }
  return covert_frame;
}

void OBCameraNode::onNewFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                                      const stream_index_pair& stream_index) {
  if (frame == nullptr) {
    return;
  }
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (camera_info_publishers_[stream_index].getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (!has_subscriber) {
    return;
  }
  std::shared_ptr<ob::VideoFrame> video_frame;
  if (frame->type() == OB_FRAME_COLOR) {
    video_frame = frame->as<ob::ColorFrame>();
  } else if (frame->type() == OB_FRAME_DEPTH) {
    video_frame = frame->as<ob::DepthFrame>();
  } else if (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
             frame->type() == OB_FRAME_IR_RIGHT) {
    video_frame = frame->as<ob::IRFrame>();
  } else {
    ROS_ERROR_STREAM("Unsupported frame type: " << frame->type());
    return;
  }
  if (!video_frame) {
    ROS_ERROR_STREAM("Failed to convert frame to video frame");
    return;
  }
  int width = static_cast<int>(video_frame->width());
  int height = static_cast<int>(video_frame->height());

  auto timestamp = frameTimeStampToROSTime(video_frame->systemTimeStamp());
  if (!camera_params_) {
    camera_params_ = pipeline_->getCameraParam();
  }

  std::string frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];
  if (camera_params_) {
    auto& intrinsic =
        stream_index == COLOR ? camera_params_->rgbIntrinsic : camera_params_->depthIntrinsic;
    auto& distortion =
        stream_index == COLOR ? camera_params_->rgbDistortion : camera_params_->depthDistortion;

    auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
    CHECK(camera_info_publishers_.count(stream_index) > 0);
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    camera_info.width = width;
    camera_info.height = height;
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher.publish(camera_info);
  }
  CHECK(image_publishers_.count(stream_index));
  if (image_publishers_[stream_index].getNumSubscribers() == 0) {
    return;
  }
  auto& image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (frame->type() == OB_FRAME_COLOR && !rgb_is_decoded_) {
    ROS_ERROR_STREAM("frame is not decoded");
    return;
  }
  if (frame->type() == OB_FRAME_COLOR) {
    memcpy(image.data, rgb_buffer_, width * height * 3);
  } else {
    memcpy(image.data, video_frame->data(), video_frame->dataSize());
  }

  if (stream_index == DEPTH) {
    auto depth_scale = video_frame->as<ob::DepthFrame>()->getValueScale();
    image = image * depth_scale;
  }
  auto image_publisher = image_publishers_[stream_index];
  auto image_msg =
      cv_bridge::CvImage(std_msgs::Header(), encoding_[stream_index], image).toImageMsg();
  CHECK_NOTNULL(image_msg.get());
  image_msg->header.stamp = timestamp;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->header.frame_id = frame_id;

  if (!flip_images_[stream_index]) {
    image_publisher.publish(image_msg);
  } else {
    cv::Mat flipped_image;
    cv::flip(image, flipped_image, 1);
    auto flipped_image_msg =
        cv_bridge::CvImage(std_msgs::Header(), encoding_[stream_index], flipped_image).toImageMsg();
    CHECK_NOTNULL(flipped_image_msg.get());
    flipped_image_msg->header.stamp = timestamp;
    flipped_image_msg->is_bigendian = false;
    flipped_image_msg->step = width * unit_step_size_[stream_index];
    flipped_image_msg->header.frame_id = depth_registration_ ? depth_aligned_frame_id_[stream_index]
                                                             : optical_frame_id_[stream_index];
    image_publisher.publish(flipped_image_msg);
  }
  saveImageToFile(stream_index, image, image_msg);
}

void OBCameraNode::saveImageToFile(const stream_index_pair& stream_index, const cv::Mat& image,
                                   const sensor_msgs::ImagePtr& image_msg) {
  if (save_images_[stream_index]) {
    auto now = time(nullptr);
    std::stringstream ss;
    ss << std::put_time(localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg->width) + "x" +
                           std::to_string(image_msg->height) + "_" + std::to_string(fps) + "hz_" +
                           ss.str() + ".png";
    if (!boost::filesystem::exists(current_path + "/image")) {
      boost::filesystem::create_directory(current_path + "/image");
    }
    ROS_INFO_STREAM("Saving image to " << filename);
    if (stream_index.first == OB_STREAM_DEPTH) {
      auto image_to_save = cv_bridge::toCvCopy(image_msg, encoding_[stream_index])->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_COLOR) {
      auto image_to_save =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
               stream_index.first == OB_STREAM_IR_RIGHT) {
      cv::imwrite(filename, image);
    } else {
      ROS_ERROR_STREAM("Unsupported stream type: " << stream_index.first);
    }
    save_images_[stream_index] = false;
  }
}

void OBCameraNode::imageSubscribedCallback(const stream_index_pair& stream_index) {
  ROS_INFO_STREAM("Image stream " << stream_name_[stream_index] << " subscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    if (pipeline_started_) {
      ROS_INFO_STREAM("pipe line already started");
      return;
    }
    try {
      startStreams();
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to start streams: " << e.getMessage());
      return;
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to start streams: " << e.what());
      return;
    }
  } else {
    if (stream_started_[stream_index]) {
      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " is already started.");
      return;
    }
    startStream(stream_index);
  }
}

void OBCameraNode::imuSubscribedCallback(const orbbec_camera::stream_index_pair& stream_index) {
  ROS_INFO_STREAM("IMU stream " << stream_name_[stream_index] << " subscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    if (imu_started_[stream_index]) {
      ROS_INFO_STREAM("IMU stream " << stream_name_[stream_index] << " is already started.");
      return;
    }
    startIMU(stream_index);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to start streams: " << e.getMessage());
    return;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to start streams: " << e.what());
    return;
  }
}

void OBCameraNode::imageUnsubscribedCallback(const stream_index_pair& stream_index) {
  ROS_INFO_STREAM("Image stream " << stream_name_[stream_index] << " unsubscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    if (!pipeline_started_) {
      ROS_INFO_STREAM("imageUnsubscribedCallback pipe line not start");
      return;
    }
    bool all_stream_no_subscriber = true;
    for (auto& item : image_publishers_) {
      if (item.second.getNumSubscribers() > 0) {
        all_stream_no_subscriber = false;
        break;
      }
    }
    if (enable_point_cloud_) {
      if (depth_cloud_pub_.getNumSubscribers() > 0) {
        all_stream_no_subscriber = false;
      }
    }
    if (enable_colored_point_cloud_) {
      if (depth_registered_cloud_pub_.getNumSubscribers() > 0) {
        all_stream_no_subscriber = false;
      }
    }
    if (all_stream_no_subscriber) {
      stopStreams();
    }
  } else {
    if (!stream_started_[stream_index]) {
      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " is not started.");
      return;
    }
    auto subscriber_count = image_publishers_[stream_index].getNumSubscribers();
    if (subscriber_count == 0) {
      stopStream(stream_index);
    }
  }
}

void OBCameraNode::imuUnsubscribedCallback(const stream_index_pair& stream_index) {
  ROS_INFO_STREAM("IMU stream " << stream_name_[stream_index] << " unsubscribed");
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  stopIMU(stream_index);
}

void OBCameraNode::pointCloudSubscribedCallback() {
  ROS_INFO_STREAM("point cloud subscribed");
  imageSubscribedCallback(DEPTH);
}

void OBCameraNode::pointCloudUnsubscribedCallback() {
  ROS_INFO_STREAM("point cloud unsubscribed");
  if (depth_cloud_pub_.getNumSubscribers() > 0) {
    return;
  }
  imageUnsubscribedCallback(DEPTH);
}

void OBCameraNode::coloredPointCloudSubscribedCallback() {
  ROS_INFO_STREAM("rgb point cloud subscribed");
  imageSubscribedCallback(DEPTH);
  imageSubscribedCallback(COLOR);
}

void OBCameraNode::coloredPointCloudUnsubscribedCallback() {
  ROS_INFO_STREAM("point cloud unsubscribed");
  if (depth_registered_cloud_pub_.getNumSubscribers() > 0) {
    return;
  }
  imageUnsubscribedCallback(DEPTH);
  imageUnsubscribedCallback(COLOR);
}

boost::optional<OBCameraParam> OBCameraNode::getCameraParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if ((depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) &&
        (color_w * height_[COLOR] == color_h * width_[COLOR])) {
      return param;
    }
  }
  return {};
}

boost::optional<OBCameraParam> OBCameraNode::getCameraDepthParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    if (depth_w == width_[DEPTH] && depth_h == height_[DEPTH]) {
      ROS_INFO_STREAM("getCameraDepthParam w=" << depth_w << ", h=" << depth_h);
      return param;
    }
  }

  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    if (depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) {
      ROS_INFO_STREAM("getCameraDepthParam w=" << depth_w << ", h=" << depth_h);
      return param;
    }
  }
  return {};
}

boost::optional<OBCameraParam> OBCameraNode::getCameraColorParam() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if (color_w == width_[COLOR] && color_h == height_[COLOR]) {
      ROS_INFO_STREAM("getCameraColorParam w=" << color_w << ", h=" << color_h);
      return param;
    }
  }

  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if (color_w * height_[COLOR] == color_h * width_[COLOR]) {
      ROS_INFO_STREAM("getCameraColorParam w=" << color_w << ", h=" << color_h);
      return param;
    }
  }
  return {};
}

int OBCameraNode::getCameraParamIndex() {
  auto camera_params = device_->getCalibrationCameraParamList();
  for (size_t i = 0; i < camera_params->count(); i++) {
    auto param = camera_params->getCameraParam(i);
    int depth_w = param.depthIntrinsic.width;
    int depth_h = param.depthIntrinsic.height;
    int color_w = param.rgbIntrinsic.width;
    int color_h = param.rgbIntrinsic.height;
    if ((depth_w * height_[DEPTH] == depth_h * width_[DEPTH]) &&
        (color_w * height_[COLOR] == color_h * width_[COLOR])) {
      return static_cast<int>(i);
    }
  }
  return -1;
}

void OBCameraNode::publishStaticTF(const ros::Time& t, const tf2::Vector3& trans,
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

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot, Q;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);
  tf2::Vector3 trans(0, 0, 0);
  startStreams();
  CHECK_NOTNULL(pipeline_.get());
  auto camera_param = pipeline_->getCameraParam();
  auto ex = camera_param.transform;
  Q = rotationMatrixToQuaternion(ex.rot);
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  for (int i = 0; i < 3; i++) {
    trans[i] = ex.trans[i];
  }
  stopStreams();

  auto tf_timestamp = ros::Time::now();
  tf2::Transform transform(Q, trans);
  transform = transform.inverse();
  Q = transform.getRotation();
  trans = transform.getOrigin();
  if (enable_stream_[COLOR]) {
    publishStaticTF(tf_timestamp, trans, Q, camera_link_frame_id_, frame_id_[COLOR]);
    publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                    optical_frame_id_[COLOR]);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (stream_index == COLOR || !enable_stream_[stream_index]) {
      continue;
    }
    publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_,
                    frame_id_[stream_index]);
    publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
  }
  for (const auto& stream_index : HID_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_,
                    frame_id_[stream_index]);
    publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
  }
}

void OBCameraNode::publishDynamicTransforms() {
  ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
  static std::mutex mu;
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

void OBCameraNode::publishStaticTransforms() {
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>();
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>();
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    CHECK_NOTNULL(static_tf_broadcaster_.get());
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

}  // namespace orbbec_camera
