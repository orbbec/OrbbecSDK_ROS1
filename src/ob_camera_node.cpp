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
#include "libobsensor/hpp/Utils.hpp"
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
  nh_ir_ = ros::NodeHandle(stream_name_[INFRA0]);
  nh_rgb_ = ros::NodeHandle(stream_name_[COLOR]);
  init();
}

void OBCameraNode::init() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  CHECK_NOTNULL(device_.get());
  is_running_ = true;
  setupConfig();
  getParameters();
  setupDevices();
  setupRecommendedPostFilters();
  selectBaseStream();
  setupProfiles();
  setupCameraInfo();
  setupTopics();
  setupCameraCtrlServices();
  setupFrameCallback();
  readDefaultExposure();
  readDefaultGain();
  readDefaultWhiteBalance();
#if defined(USE_RK_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<RKMjpegDecoder>(width_[COLOR], height_[COLOR]);
#elif defined(USE_NV_HW_DECODER)
  mjpeg_decoder_ = std::make_shared<JetsonNvJPEGDecoder>(width_[COLOR], height_[COLOR]);
#endif
  if (enable_stream_[COLOR]) {
    CHECK(width_[COLOR] > 0 && height_[COLOR] > 0);
    rgb_buffer_ = new uint8_t[width_[COLOR] * height_[COLOR] * 3];
  }
  if (enable_colored_point_cloud_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    CHECK(width_[COLOR] > 0 && height_[COLOR] > 0);
    rgb_point_cloud_buffer_size_ = width_[COLOR] * height_[COLOR] * sizeof(OBColorPoint);
    rgb_point_cloud_buffer_ = new uint8_t[rgb_point_cloud_buffer_size_];
    xy_table_data_size_ = width_[COLOR] * height_[COLOR] * 2;
    xy_table_data_ = new float[xy_table_data_size_];
  }
  rgb_is_decoded_ = false;
  if (diagnostics_frequency_ > 0.0) {
    diagnostics_thread_ = std::make_shared<std::thread>([this]() { setupDiagnosticUpdater(); });
  }
  is_initialized_ = true;
}

bool OBCameraNode::isInitialized() const { return is_initialized_; }

void OBCameraNode::rebootDevice() {
  ROS_INFO("Reboot device");
  if (device_) {
    device_->reboot();
  }
  ROS_INFO("Reboot device DONE");
}


void OBCameraNode::clean() {
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
  if (diagnostics_thread_ && diagnostics_thread_->joinable()) {
    diagnostics_thread_->join();
  }

  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() stop stream");
  stopStreams();
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() delete rgb_buffer");
  delete[] rgb_buffer_;
  delete[] rgb_point_cloud_buffer_;
  delete[] xy_table_data_;
  ROS_INFO_STREAM("OBCameraNode::~OBCameraNode() end");
}

OBCameraNode::~OBCameraNode() noexcept {
  clean();
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
  }
  depth_aligned_frame_id_[DEPTH] = optical_frame_id_[COLOR];

  use_hardware_time_ = nh_private_.param<bool>("use_hardware_time", true);
  publish_tf_ = nh_private_.param<bool>("publish_tf", false);
  depth_registration_ = nh_private_.param<bool>("depth_registration", false);
  enable_frame_sync_ = nh_private_.param<bool>("enable_frame_sync", false);
  ir_info_uri_ = nh_private_.param<std::string>("ir_info_uri", "");
  color_info_uri_ = nh_private_.param<std::string>("color_info_uri", "");
  enable_d2c_viewer_ = nh_private_.param<bool>("enable_d2c_viewer", false);
  enable_pipeline_ = nh_private_.param<bool>("enable_pipeline", true);
  enable_point_cloud_ = nh_private_.param<bool>("enable_point_cloud", true);
  enable_colored_point_cloud_ = nh_private_.param<bool>("enable_colored_point_cloud", false);
  enable_hardware_d2d_ = nh_private_.param<bool>("enable_hardware_d2d", true);
  depth_work_mode_ = nh_private_.param<std::string>("depth_work_mode", "");
  enable_soft_filter_ = nh_private_.param<bool>("enable_soft_filter", true);
  enable_color_auto_exposure_ = nh_private_.param<bool>("enable_color_auto_exposure", true);
  color_exposure_ = nh_private_.param<int>("color_exposure_", -1);
  enable_ir_auto_exposure_ = nh_private_.param<bool>("enable_ir_auto_exposure", true);
  ir_exposure_ = nh_private_.param<int>("ir_exposure_", -1);
  enable_ir_long_exposure_ = nh_private_.param<bool>("enable_ir_long_exposure", false);
  sync_mode_str_ = nh_private_.param<std::string>("sync_mode", "standalone");
  depth_delay_us_ = nh_private_.param<int>("depth_delay_us", 0);
  color_delay_us_ = nh_private_.param<int>("color_delay_us", 0);
  trigger2image_delay_us_ = nh_private_.param<int>("trigger2image_delay_us", 0);
  trigger_out_delay_us_ = nh_private_.param<int>("trigger_out_delay_us", 0);
  trigger_out_enabled_ = nh_private_.param<bool>("trigger_out_enabled", false);
  depth_precision_str_ = nh_private_.param<std::string>("depth_precision", "");
  if (!depth_precision_str_.empty()) {
    depth_precision_level_ = DEPTH_PRECISION_STR2ENUM.at(depth_precision_str_);
  }
  if (enable_colored_point_cloud_ || enable_d2c_viewer_) {
    depth_registration_ = true;
  }
  soft_filter_max_diff_ = nh_private_.param<int>("soft_filter_max_diff", -1);
  soft_filter_speckle_size_ = nh_private_.param<int>("soft_filter_speckle_size", -1);
  depth_filter_config_ = nh_private_.param<std::string>("depth_filter_config", "");
  ordered_pc_ = nh_private_.param<bool>("ordered_pc", false);
  max_save_images_count_ = nh_private_.param<int>("max_save_images_count", 10);
  if (!depth_filter_config_.empty()) {
    enable_depth_filter_ = true;
  }

  enable_sync_output_accel_gyro_ = nh_private_.param<bool>("enable_sync_output_accel_gyro", false);
  for (const auto& stream_index : HID_STREAMS) {
    std::string param_name = "enable_" + stream_name_[stream_index];
    enable_stream_[stream_index] = nh_private_.param<bool>(param_name, false);
    if (enable_sync_output_accel_gyro_) {
      enable_stream_[stream_index] = true;
    }
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
  }
  device_preset_ = nh_private_.param<std::string>("device_preset", "");
  // filter switch
  enable_decimation_filter_ = nh_private_.param<bool>("enable_decimation_filter", false);
  enable_hdr_merge_ = nh_private_.param<bool>("enable_hdr_merge", false);
  enable_sequenced_filter_ = nh_private_.param<bool>("enable_sequenced_filter", false);
  enable_threshold_filter_ = nh_private_.param<bool>("enable_threshold_filter", false);
  enable_noise_removal_filter_ = nh_private_.param<bool>("enable_noise_removal_filter", true);
  enable_spatial_filter_ = nh_private_.param<bool>("enable_spatial_filter", false);
  enable_temporal_filter_ = nh_private_.param<bool>("enable_temporal_filter", false);
  enable_hole_filling_filter_ = nh_private_.param<bool>("enable_hole_filling_filter", false);
  decimation_filter_scale_range_ = nh_private_.param<int>("decimation_filter_scale_range", -1);
  sequence_id_filter_id_ = nh_private_.param<int>("sequence_id_filter_id", -1);
  threshold_filter_max_ = nh_private_.param<int>("threshold_filter_max", -1);
  threshold_filter_min_ = nh_private_.param<int>("threshold_filter_min", -1);
  noise_removal_filter_min_diff_ = nh_private_.param<int>("noise_removal_filter_min_diff", 256);
  noise_removal_filter_max_size_ = nh_private_.param<int>("noise_removal_filter_max_size", 80);
  spatial_filter_alpha_ = nh_private_.param<float>("spatial_filter_alpha", -1.0);
  spatial_filter_diff_threshold_ = nh_private_.param<int>("spatial_filter_diff_threshold", -1);
  spatial_filter_magnitude_ = nh_private_.param<int>("spatial_filter_magnitude", -1);
  spatial_filter_radius_ = nh_private_.param<int>("spatial_filter_radius", -1);
  temporal_filter_diff_threshold_ =
      nh_private_.param<float>("temporal_filter_diff_threshold", -1.0);
  temporal_filter_weight_ = nh_private_.param<float>("temporal_filter_weight", -1.0);
  hole_filling_filter_mode_ = nh_private_.param<std::string>("hole_filling_filter_mode", "");
  hdr_merge_exposure_1_ = nh_private_.param<int>("hdr_merge_exposure_1", -1);
  hdr_merge_gain_1_ = nh_private_.param<int>("hdr_merge_gain_1", -1);
  hdr_merge_exposure_2_ = nh_private_.param<int>("hdr_merge_exposure_2", -1);
  hdr_merge_gain_2_ = nh_private_.param<int>("hdr_merge_gain_2", -1);
  diagnostics_frequency_ = nh_private_.param<double>("diagnostics_frequency", 1.0);
  enable_laser_ = nh_private_.param<bool>("enable_laser", true);
  laser_on_off_mode_ = nh_private_.param<int>("laser_on_off_mode", 0);
  align_mode_ = nh_private_.param<std::string>("align_mode", "HW");
  enable_color_hdr_ = nh_private_.param<bool>("enable_color_hdr", false);
  enable_depth_scale_ = nh_private_.param<bool>("enable_depth_scale", true);
  retry_on_usb3_detection_failure_ =
      nh_private_.param<bool>("retry_on_usb3_detection_failure", false);
  laser_energy_level_ = nh_private_.param<int>("laser_energy_level", -1);
  enable_ldp_ = nh_private_.param<bool>("enable_ldp", true);
  tf_publish_rate_ = nh_private_.param<double>("tf_publish_rate", 0.0);
  enable_heartbeat_ = nh_private_.param<bool>("enable_heartbeat", false);
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  if (isOpenNIDevice(device_info->pid())) {
    use_hardware_time_ = false;
  }
}

void OBCameraNode::startStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_) {
    CHECK_NOTNULL(pipeline_.get());
    if (enable_frame_sync_) {
      ROS_INFO_STREAM("====Enable frame sync====");
      pipeline_->enableFrameSync();
    } else {
      pipeline_->disableFrameSync();
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

void OBCameraNode::startIMUSyncStream() {
  if (!imuPipeline_) {
    ROS_INFO_STREAM("start IMU sync stream failed, IMU pileline is not initialized!");
    return;
  }

  if (imu_sync_output_start_) {
    ROS_INFO_STREAM("IMU sync stream already started.");
    return;
  }

  // ACCEL
  auto accelProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_ACCEL);
  auto accel_range = fullAccelScaleRangeFromString(imu_range_[ACCEL]);
  auto accel_rate = sampleRateFromString(imu_rate_[ACCEL]);
  auto accelProfile = accelProfiles->getAccelStreamProfile(accel_range, accel_rate);
  // GYRO
  auto gyroProfiles = imuPipeline_->getStreamProfileList(OB_SENSOR_GYRO);
  auto gyro_range = fullGyroScaleRangeFromString(imu_range_[GYRO]);
  auto gyro_rate = sampleRateFromString(imu_rate_[GYRO]);
  auto gyroProfile = gyroProfiles->getGyroStreamProfile(gyro_range, gyro_rate);
  std::shared_ptr<ob::Config> imuConfig = std::make_shared<ob::Config>();
  imuConfig->enableStream(accelProfile);
  imuConfig->enableStream(gyroProfile);
  imuPipeline_->enableFrameSync();
  imuPipeline_->start(imuConfig, [&](std::shared_ptr<ob::Frame> frame) {
    auto frameSet = frame->as<ob::FrameSet>();
    if (!frameSet) {
      return;
    }
    auto aFrame = frameSet->getFrame(OB_FRAME_ACCEL);
    auto gFrame = frameSet->getFrame(OB_FRAME_GYRO);
    if (aFrame && gFrame) {
      onNewIMUFrameSyncOutputCallback(aFrame, gFrame);
    }
  });

  imu_sync_output_start_ = true;
  if (!imu_sync_output_start_) {
    ROS_ERROR_STREAM(
        "Failed to start IMU stream, please check the imu_rate and imu_range parameters.");
  } else {
    ROS_INFO_STREAM("start accel stream with range: " << fullAccelScaleRangeToString(accel_range)
                                                      << ",rate:" << sampleRateToString(accel_rate)
                                                      << ", and start gyro stream with range:"
                                                      << fullGyroScaleRangeToString(gyro_range)
                                                      << ",rate:" << sampleRateToString(gyro_rate));
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
  if (enable_sync_output_accel_gyro_) {
    startIMUSyncStream();
  } else {
    if (!stream_profile_[stream_index]) {
      ROS_ERROR_STREAM("stream " << stream_name_[stream_index] << " profile is null!");
      return;
    }
    auto profile = stream_profile_[stream_index];
    imu_sensor_[stream_index]->start(profile,
                                     [this, stream_index](const std::shared_ptr<ob::Frame>& frame) {
                                       onNewIMUFrameCallback(frame, stream_index);
                                     });
    imu_started_[stream_index] = true;
    ROS_INFO_STREAM("start IMU stream with " << imu_range_[stream_index] << " range and "
                                             << imu_rate_[stream_index] << " rate");
  }
}

void OBCameraNode::stopStreams() {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (enable_pipeline_ && pipeline_ && pipeline_started_) {
    CHECK_NOTNULL(pipeline_.get());
    try {
      pipeline_->stop();
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to stop pipeline: " << e.getMessage());
    }
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
  if (enable_sync_output_accel_gyro_) {
    if (!imu_sync_output_start_ || !imuPipeline_) {
      return;
    }
    try {
      imuPipeline_->stop();
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to stop imu pipeline: " << e.getMessage());
    }
  } else {
    for (const auto& stream_index : HID_STREAMS) {
      if (imu_started_[stream_index]) {
        stopIMU(stream_index);
      }
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

void OBCameraNode::publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  try {
    if (depth_registration_ || enable_colored_point_cloud_) {
      if (frame_set->depthFrame() != nullptr && frame_set->colorFrame() != nullptr) {
        publishColoredPointCloud(frame_set);
      }
    }

    if (depth_frame_) {
      publishDepthPointCloud(frame_set);
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
  if (!enable_point_cloud_ || depth_cloud_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (!depth_frame_) {
    return;
  }
  std::lock_guard<decltype(cloud_mutex_)> cloud_lock(cloud_mutex_);
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  if (!depth_frame) {
    ROS_ERROR_STREAM("depth frame is null");
    return;
  }
  CHECK_NOTNULL(pipeline_);
  auto camera_params = pipeline_->getCameraParam();
  if (depth_registration_ && isGemini335PID(device_info_->pid())) {
    // if depth registration is enabled and the device is a Gemini 335, use the rgb intrinsic as the
    // depth intrinsic
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  if (device_info_->pid() == DABAI_MAX_PID) {
    // if the device is a DABAI MAX, use the rgb intrinsic as the depth intrinsic
    camera_params.depthIntrinsic = camera_params.rgbIntrinsic;
  }
  depth_point_cloud_filter_.setCameraParam(camera_params);
  float depth_scale = depth_frame->getValueScale();
  depth_point_cloud_filter_.setPositionDataScaled(depth_scale);
  depth_point_cloud_filter_.setCreatePointFormat(OB_FORMAT_POINT);
  auto result_frame = depth_point_cloud_filter_.process(depth_frame);
  if (!result_frame) {
    ROS_DEBUG("Failed to create point cloud");
    return;
  }
  auto point_size = result_frame->dataSize() / sizeof(OBPoint);
  auto* points = static_cast<OBPoint*>(result_frame->data());
  auto width = depth_frame->width();
  auto height = depth_frame->height();
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
  const static double MIN_DISTANCE = 20.0;
  const static double MAX_DISTANCE = 10000.0;
  const static double min_depth = MIN_DISTANCE / depth_scale;
  const static double max_depth = MAX_DISTANCE / depth_scale;
  size_t valid_count = 0;
  for (size_t i = 0; i < point_size; i++) {
    bool valid_point = points[i].z >= min_depth && points[i].z <= max_depth;
    if (valid_point || ordered_pc_) {
      *iter_x = static_cast<float>(points[i].x / 1000.0);
      *iter_y = static_cast<float>(points[i].y / 1000.0);
      *iter_z = static_cast<float>(points[i].z / 1000.0);
      ++iter_x, ++iter_y, ++iter_z;
      valid_count++;
    }
  }
  if (!ordered_pc_) {
    cloud_msg_.is_dense = true;
    cloud_msg_.width = valid_count;
    cloud_msg_.height = 1;
    modifier.resize(valid_count);
  }
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(depth_frame->timeStampUs())
                                      : fromUsToROSTime(depth_frame->systemTimeStampUs());
  std::string frame_id = depth_registration_ ? optical_frame_id_[COLOR] : optical_frame_id_[DEPTH];
  cloud_msg_.header.stamp = timestamp;
  cloud_msg_.header.frame_id = frame_id;
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
    try {
      saveDepthPointCloudMsgToPly(cloud_msg_, filename);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to save point cloud: " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to save point cloud");
    }
  }
}

void OBCameraNode::publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!enable_colored_point_cloud_ || depth_registered_cloud_pub_.getNumSubscribers() == 0) {
    return;
  }
  if (!depth_frame_) {
    return;
  }
  CHECK_NOTNULL(depth_frame_.get());
  std::lock_guard<decltype(cloud_mutex_)> cloud_lock(cloud_mutex_);
  auto depth_frame = depth_frame_->as<ob::DepthFrame>();
  auto color_frame = frame_set->colorFrame();
  if (!depth_frame || !color_frame) {
    return;
  }
  auto depth_width = depth_frame->width();
  auto depth_height = depth_frame->height();
  auto color_width = color_frame->width();
  auto color_height = color_frame->height();
  if (depth_width != color_width || depth_height != color_height) {
    ROS_DEBUG("Depth (%d x %d) and color (%d x %d) frame size mismatch", depth_width, depth_height,
              color_width, color_height);
    return;
  }
  if (!xy_tables_) {
    calibration_param_ = pipeline_->getCalibrationParam(pipeline_config_);

    uint32_t table_size =
        color_width * color_height * 2;  // one for x-coordinate and one for y-coordinate LUT
    if (xy_table_data_size_ != table_size) {
      ROS_INFO_STREAM("Init xy tables with size " << table_size);
      xy_table_data_size_ = table_size;
      delete[] xy_table_data_;
      xy_table_data_ = new float[table_size];
    }

    xy_tables_ = OBXYTables();
    CHECK_NOTNULL(xy_table_data_);
    if (!ob::CoordinateTransformHelper::transformationInitXYTables(
            *calibration_param_, OB_SENSOR_COLOR, xy_table_data_, &table_size, &(*xy_tables_))) {
      ROS_ERROR("Failed to init xy tables");
      return;
    }
  }

  const auto* depth_data = (uint8_t*)depth_frame->data();
  const auto* color_data = (uint8_t*)(rgb_buffer_);
  CHECK_NOTNULL(rgb_point_cloud_buffer_);
  uint32_t point_cloud_buffer_size = color_width * color_height * sizeof(OBColorPoint);
  if (point_cloud_buffer_size > rgb_point_cloud_buffer_size_) {
    delete[] rgb_point_cloud_buffer_;
    rgb_point_cloud_buffer_ = new uint8_t[point_cloud_buffer_size];
    rgb_point_cloud_buffer_size_ = point_cloud_buffer_size;
  }
  memset(rgb_point_cloud_buffer_, 0, rgb_point_cloud_buffer_size_);
  auto* point_cloud = (OBColorPoint*)rgb_point_cloud_buffer_;
  ob::CoordinateTransformHelper::transformationDepthToRGBDPointCloud(&(*xy_tables_), depth_data,
                                                                     color_data, point_cloud);

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
  for (size_t i = 0; i < color_width * color_height; i++) {
    bool valid_point = point_cloud[i].z >= min_depth && point_cloud[i].z <= max_depth;
    if (valid_point || ordered_pc_) {
      *iter_x = static_cast<float>(point_cloud[i].x / 1000.0);
      *iter_y = static_cast<float>(point_cloud[i].y / 1000.0);
      *iter_z = static_cast<float>(point_cloud[i].z / 1000.0);
      *iter_r = static_cast<uint8_t>(point_cloud[i].r);
      *iter_g = static_cast<uint8_t>(point_cloud[i].g);
      *iter_b = static_cast<uint8_t>(point_cloud[i].b);
      ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b;
      ++valid_count;
    }
  }
  if (!ordered_pc_) {
    cloud_msg_.is_dense = true;
    cloud_msg_.width = valid_count;
    cloud_msg_.height = 1;
    modifier.resize(valid_count);
  }
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(depth_frame->timeStampUs())
                                      : fromUsToROSTime(depth_frame->systemTimeStampUs());
  cloud_msg_.header.stamp = timestamp;
  cloud_msg_.header.frame_id = optical_frame_id_[COLOR];
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
    try {
      saveRGBPointCloudMsgToPly(cloud_msg_, filename);
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to save colored point cloud: " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to save colored point cloud with unknown error");
    }
  }
}

IMUInfo OBCameraNode::createIMUInfo(const stream_index_pair& stream_index) {
  IMUInfo imu_info;
  imu_info.header.frame_id = optical_frame_id_[stream_index];
  imu_info.header.stamp = ros::Time::now();
  auto imu_profile = stream_profile_[stream_index];
  if (stream_index == GYRO) {
    auto gyro_profile = stream_profile_[stream_index]->as<ob::GyroStreamProfile>();
    auto gyro_intrinsics = gyro_profile->getIntrinsic();
    imu_info.noise_density = gyro_intrinsics.noiseDensity;
    imu_info.random_walk = gyro_intrinsics.randomWalk;
    imu_info.reference_temperature = gyro_intrinsics.referenceTemp;
    imu_info.bias = {gyro_intrinsics.bias[0], gyro_intrinsics.bias[1], gyro_intrinsics.bias[2]};
    imu_info.scale_misalignment = {
        gyro_intrinsics.scaleMisalignment[0], gyro_intrinsics.scaleMisalignment[1],
        gyro_intrinsics.scaleMisalignment[2], gyro_intrinsics.scaleMisalignment[3],
        gyro_intrinsics.scaleMisalignment[4], gyro_intrinsics.scaleMisalignment[5],
        gyro_intrinsics.scaleMisalignment[6], gyro_intrinsics.scaleMisalignment[7],
        gyro_intrinsics.scaleMisalignment[8]};
    imu_info.temperature_slope = {
        gyro_intrinsics.tempSlope[0], gyro_intrinsics.tempSlope[1], gyro_intrinsics.tempSlope[2],
        gyro_intrinsics.tempSlope[3], gyro_intrinsics.tempSlope[4], gyro_intrinsics.tempSlope[5],
        gyro_intrinsics.tempSlope[6], gyro_intrinsics.tempSlope[7], gyro_intrinsics.tempSlope[8]};
  } else if (stream_index == ACCEL) {
    auto accel_profile = stream_profile_[stream_index]->as<ob::AccelStreamProfile>();
    auto accel_intrinsics = accel_profile->getIntrinsic();
    imu_info.noise_density = accel_intrinsics.noiseDensity;
    imu_info.random_walk = accel_intrinsics.randomWalk;
    imu_info.reference_temperature = accel_intrinsics.referenceTemp;
    imu_info.bias = {accel_intrinsics.bias[0], accel_intrinsics.bias[1], accel_intrinsics.bias[2]};
    imu_info.gravity = {accel_intrinsics.gravity[0], accel_intrinsics.gravity[1],
                        accel_intrinsics.gravity[2]};
    imu_info.scale_misalignment = {
        accel_intrinsics.scaleMisalignment[0], accel_intrinsics.scaleMisalignment[1],
        accel_intrinsics.scaleMisalignment[2], accel_intrinsics.scaleMisalignment[3],
        accel_intrinsics.scaleMisalignment[4], accel_intrinsics.scaleMisalignment[5],
        accel_intrinsics.scaleMisalignment[6], accel_intrinsics.scaleMisalignment[7],
        accel_intrinsics.scaleMisalignment[8]};
    imu_info.temperature_slope = {accel_intrinsics.tempSlope[0], accel_intrinsics.tempSlope[1],
                                  accel_intrinsics.tempSlope[2], accel_intrinsics.tempSlope[3],
                                  accel_intrinsics.tempSlope[4], accel_intrinsics.tempSlope[5],
                                  accel_intrinsics.tempSlope[6], accel_intrinsics.tempSlope[7],
                                  accel_intrinsics.tempSlope[8]};
  }

  return imu_info;
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

void OBCameraNode::onNewIMUFrameSyncOutputCallback(const std::shared_ptr<ob::Frame>& accel_frame,
                                                   const std::shared_ptr<ob::Frame>& gyro_frame) {
  if (!isInitialized()) {
    ROS_WARN_ONCE("IMU sync output callback called before initialization");
    return;
  }
  if (!imu_gyro_accel_publisher_) {
    ROS_ERROR_STREAM("stream Accel Gyro publisher not initialized");
    return;
  }
  ROS_INFO_STREAM_ONCE("IMU sync output callback called");
  auto has_subscriber = imu_gyro_accel_publisher_.getNumSubscribers() > 0;
  has_subscriber = has_subscriber || imu_info_publishers_[ACCEL].getNumSubscribers() > 0;
  has_subscriber = has_subscriber || imu_info_publishers_[GYRO].getNumSubscribers() > 0;
  if (!has_subscriber) {
    return;
  }

  std::this_thread::sleep_for(std::chrono::nanoseconds(1));
  auto imu_msg = sensor_msgs::Imu();
  setDefaultIMUMessage(imu_msg);

  imu_msg.header.frame_id = imu_optical_frame_id_;
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(accel_frame->timeStampUs())
                                      : fromUsToROSTime(accel_frame->systemTimeStampUs());
  imu_msg.header.stamp = timestamp;
  auto gyro_cast_frame = gyro_frame->as<ob::GyroFrame>();
  auto gyro_info = createIMUInfo(GYRO);
  gyro_info.header = imu_msg.header;
  gyro_info.header.frame_id = imu_optical_frame_id_;
  imu_info_publishers_[GYRO].publish(gyro_info);
  auto gyroData = gyro_cast_frame->value();
  imu_msg.angular_velocity.x = gyroData.x - gyro_info.bias[0];
  imu_msg.angular_velocity.y = gyroData.y - gyro_info.bias[1];
  imu_msg.angular_velocity.z = gyroData.z - gyro_info.bias[2];
  auto accel_cast_frame = accel_frame->as<ob::AccelFrame>();
  auto accelData = accel_cast_frame->value();
  auto accel_info = createIMUInfo(ACCEL);
  imu_msg.linear_acceleration.x = accelData.x - accel_info.bias[0];
  imu_msg.linear_acceleration.y = accelData.y - accel_info.bias[1];
  imu_msg.linear_acceleration.z = accelData.z - accel_info.bias[2];
  imu_info_publishers_[ACCEL].publish(accel_info);
  imu_gyro_accel_publisher_.publish(imu_msg);
}

void OBCameraNode::onNewIMUFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                                         const stream_index_pair& stream_index) {
  if (!isInitialized()) {
    ROS_WARN_ONCE("IMU callback called before initialization");
    return;
  }
  if (!imu_publishers_.count(stream_index)) {
    ROS_ERROR_STREAM("stream " << stream_name_[stream_index] << " publisher not initialized");
    return;
  }
  auto has_subscriber = imu_publishers_[stream_index].getNumSubscribers() > 0;
  has_subscriber = has_subscriber || imu_info_publishers_[stream_index].getNumSubscribers() > 0;
  if (!has_subscriber) {
    return;
  }
  std::this_thread::sleep_for(std::chrono::nanoseconds(1));

  auto imu_msg = sensor_msgs::Imu();
  setDefaultIMUMessage(imu_msg);
  imu_msg.header.frame_id = optical_frame_id_[stream_index];
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(frame->timeStampUs())
                                      : fromUsToROSTime(frame->systemTimeStampUs());
  imu_msg.header.stamp = timestamp;
  auto imu_info = createIMUInfo(stream_index);
  imu_info.header = imu_msg.header;
  imu_info.header.frame_id = imu_optical_frame_id_;
  imu_info_publishers_[stream_index].publish(imu_info);
  if (frame->type() == OB_FRAME_GYRO) {
    auto gyro_frame = frame->as<ob::GyroFrame>();
    auto data = gyro_frame->value();
    imu_msg.angular_velocity.x = data.x - imu_info.bias[0];
    imu_msg.angular_velocity.y = data.y - imu_info.bias[1];
    imu_msg.angular_velocity.z = data.z - imu_info.bias[2];
  } else if (frame->type() == OB_FRAME_ACCEL) {
    auto accel_frame = frame->as<ob::AccelFrame>();
    auto data = accel_frame->value();
    imu_msg.linear_acceleration.x = data.x - imu_info.bias[0];
    imu_msg.linear_acceleration.y = data.y - imu_info.bias[1];
    imu_msg.linear_acceleration.z = data.z - imu_info.bias[2];
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
  if (metadata_publishers_.count(COLOR) && metadata_publishers_[COLOR].getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (camera_info_publishers_.count(COLOR) &&
      camera_info_publishers_[COLOR].getNumSubscribers() > 0) {
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
    CHECK_NOTNULL(dest);
    memcpy(dest, video_frame->data(), video_frame->dataSize());
    return true;
  }
  return true;
}

std::shared_ptr<ob::Frame> OBCameraNode::decodeIRMJPGFrame(
    const std::shared_ptr<ob::Frame>& frame) {
  if (frame->format() == OB_FORMAT_MJPEG &&
      (frame->type() == OB_FRAME_IR || frame->type() == OB_FRAME_IR_LEFT ||
       frame->type() == OB_FRAME_IR_RIGHT)) {
    auto video_frame = frame->as<ob::IRFrame>();

    cv::Mat mjpgMat(1, video_frame->dataSize(), CV_8UC1, video_frame->data());
    cv::Mat irRawMat = cv::imdecode(mjpgMat, cv::IMREAD_GRAYSCALE);

    std::shared_ptr<ob::Frame> irFrame = ob::FrameHelper::createFrame(
        video_frame->type(), video_frame->format(), video_frame->width(), video_frame->height(), 0);

    uint32_t buffer_size = irRawMat.rows * irRawMat.cols * irRawMat.channels();

    if (buffer_size > irFrame->dataSize()) {
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

std::shared_ptr<ob::Frame> OBCameraNode::processDepthFrameFilter(
    std::shared_ptr<ob::Frame>& frame) {
  if (frame == nullptr || frame->type() != OB_FRAME_DEPTH) {
    return nullptr;
  }
  auto sensor = device_->getSensor(OB_SENSOR_DEPTH);
  CHECK_NOTNULL(sensor.get());
  auto filter_list = sensor->getRecommendedFilters();
  for (size_t i = 0; i < filter_list->count(); i++) {
    auto filter = filter_list->getFilter(i);
    CHECK_NOTNULL(filter.get());
    if (filter->isEnabled() && frame != nullptr) {
      frame = filter->process(frame);
      if (frame == nullptr) {
        ROS_ERROR_STREAM("Depth filter process failed");
        break;
      }
    }
  }
  return frame;
}

void OBCameraNode::onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!is_running_) {
    ROS_WARN_ONCE("Frame callback called before initialization");
    return;
  }
  if (!isInitialized()) {
    ROS_WARN_ONCE("Frame callback called before initialization");
    return;
  }
  if (frame_set == nullptr) {
    return;
  }
  ROS_INFO_STREAM_ONCE("Received first frame set");
  try {
    std::shared_ptr<ob::ColorFrame> color_frame = frame_set->colorFrame();
    depth_frame_ = frame_set->getFrame(OB_FRAME_DEPTH);
    CHECK_NOTNULL(device_info_);
    if (isGemini335PID(device_info_->pid()) && enable_stream_[DEPTH]) {
      depth_frame_ = processDepthFrameFilter(depth_frame_);
      if (depth_registration_ && align_filter_ && depth_frame_ && color_frame) {
        auto new_frame = align_filter_->process(frame_set);
        if (new_frame) {
          auto new_frame_set = new_frame->as<ob::FrameSet>();
          if (new_frame_set) {
            depth_frame_ = new_frame_set->getFrame(OB_FRAME_DEPTH);
          } else {
            ROS_ERROR_STREAM("cast to FrameSet failed");
            return;
          }
        } else {
          ROS_ERROR_STREAM("Depth frame alignment failed");
          return;
        }
      }
    }
    if (enable_stream_[COLOR] && color_frame) {
      std::unique_lock<std::mutex> colorLock(colorFrameMtx_);
      colorFrameQueue_.push(frame_set);
      colorFrameCV_.notify_all();
    } else {
      publishPointCloud(frame_set);
    }

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
        if (frame_type == OB_FRAME_DEPTH) {
          frame = depth_frame_;
        }

        std::shared_ptr<ob::Frame> irFrame = decodeIRMJPGFrame(frame);
        if (irFrame) {
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
    if (colorFrameQueue_.empty()) {
      continue;
    }
    std::shared_ptr<ob::FrameSet> frameSet = colorFrameQueue_.front();
    colorFrameQueue_.pop();
    rgb_is_decoded_ = decodeColorFrameToBuffer(frameSet->colorFrame(), rgb_buffer_);
    publishPointCloud(frameSet);
    onNewFrameCallback(frameSet->colorFrame(), IMAGE_STREAMS.at(2));
  }

  ROS_INFO_STREAM("Color frame thread exit!");
}

std::shared_ptr<ob::Frame> OBCameraNode::softwareDecodeColorFrame(
    const std::shared_ptr<ob::Frame>& frame) {
  if (frame->format() == OB_FORMAT_RGB || frame->format() == OB_FORMAT_BGR) {
    return frame;
  }
  if (frame->format() == OB_FORMAT_Y16 || frame->format() == OB_FORMAT_Y8) {
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

void OBCameraNode::onNewFrameCallback(std::shared_ptr<ob::Frame> frame,
                                      const stream_index_pair& stream_index) {
  if (frame == nullptr) {
    return;
  }
  bool has_subscriber = image_publishers_[stream_index].getNumSubscribers() > 0;
  if (camera_info_publishers_[stream_index].getNumSubscribers() > 0) {
    has_subscriber = true;
  }
  if (metadata_publishers_.count(stream_index) &&
      metadata_publishers_[stream_index].getNumSubscribers() > 0) {
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
  auto timestamp = use_hardware_time_ ? fromUsToROSTime(video_frame->timeStampUs())
                                      : fromUsToROSTime(video_frame->systemTimeStampUs());
  std::string frame_id = (depth_registration_ && stream_index == DEPTH)
                             ? depth_aligned_frame_id_[stream_index]
                             : optical_frame_id_[stream_index];
  if (color_camera_info_manager_ && color_camera_info_manager_->isCalibrated() &&
      stream_index == COLOR) {
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    auto camera_info = color_camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else if (ir_camera_info_manager_ && ir_camera_info_manager_->isCalibrated() &&
             (stream_index == INFRA0 || stream_index == DEPTH)) {
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    auto camera_info = ir_camera_info_manager_->getCameraInfo();
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  } else {
    OBCameraIntrinsic intrinsic;
    OBCameraDistortion distortion;
    CHECK_NOTNULL(device_info_.get());
    if (isGemini335PID(device_info_->pid())) {
      auto stream_profile = frame->getStreamProfile();
      CHECK_NOTNULL(stream_profile.get());
      auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(video_stream_profile);
      intrinsic = video_stream_profile->getIntrinsic();
      distortion = video_stream_profile->getDistortion();
    } else {
      auto camera_params = pipeline_->getCameraParam();
      intrinsic = stream_index == COLOR ? camera_params.rgbIntrinsic : camera_params.depthIntrinsic;
      distortion =
          stream_index == COLOR ? camera_params.rgbDistortion : camera_params.depthDistortion;
      if(device_info_->pid() == DABAI_MAX_PID){
        // use color extrinsic
        intrinsic = camera_params.rgbIntrinsic;
        distortion = camera_params.rgbDistortion;
      }
    }
    auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
    CHECK(camera_info_publishers_.count(stream_index) > 0);
    auto camera_info_publisher = camera_info_publishers_[stream_index];
    camera_info.width = width;
    camera_info.height = height;
    camera_info.header.stamp = timestamp;
    camera_info.header.frame_id = frame_id;
    if (frame->type() == OB_FRAME_IR_RIGHT && enable_stream_[INFRA1]) {
      auto left_video_profile = stream_profile_[INFRA1]->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(left_video_profile.get());
      auto stream_profile = frame->getStreamProfile();
      CHECK_NOTNULL(stream_profile.get());
      auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
      CHECK_NOTNULL(video_stream_profile.get());
      auto ex = video_stream_profile->getExtrinsicTo(left_video_profile);
      double fx = camera_info.K.at(0);
      double fy = camera_info.K.at(4);
      camera_info.P.at(3) = fx * ex.trans[0] / 1000.0 + 0.0;
      camera_info.P.at(7) = fy * ex.trans[1] / 1000.0 + 0.0;
    }
    camera_info_publisher.publish(camera_info);
    publishMetadata(frame, stream_index, camera_info.header);
  }

  CHECK(image_publishers_.count(stream_index));
  if (!image_publishers_[stream_index].getNumSubscribers()) {
    return;
  }
  auto& image = images_[stream_index];
  if (image.empty() || image.cols != width || image.rows != height) {
    image.create(height, width, image_format_[stream_index]);
  }
  if (frame->type() == OB_FRAME_COLOR && frame->format() != OB_FORMAT_Y8 &&
      frame->format() != OB_FORMAT_Y16 && !rgb_is_decoded_ &&
      image_publishers_[COLOR].getNumSubscribers() > 0) {
    ROS_ERROR_STREAM("frame is not decoded");
    return;
  }
  if (frame->type() == OB_FRAME_COLOR && frame->format() != OB_FORMAT_Y8 &&
      frame->format() != OB_FORMAT_Y16 && image_publishers_[COLOR].getNumSubscribers() > 0) {
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
  auto& seq = image_seq_[stream_index];
  image_msg->header.stamp = timestamp;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size_[stream_index];
  image_msg->header.frame_id = frame_id;
  image_msg->header.seq = seq++;

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
    flipped_image_msg->header.frame_id = frame_id;
    image_publisher.publish(flipped_image_msg);
  }
  saveImageToFile(stream_index, image, image_msg);
}

void OBCameraNode::publishMetadata(const std::shared_ptr<ob::Frame>& frame,
                                   const stream_index_pair& stream_index,
                                   const std_msgs::Header& header) {
  if (metadata_publishers_.count(stream_index) == 0) {
    return;
  }
  auto metadata_publisher = metadata_publishers_[stream_index];
  if (metadata_publisher.getNumSubscribers() == 0) {
    return;
  }
  orbbec_camera::Metadata metadata_msg;
  metadata_msg.header = header;
  nlohmann::json json_data;

  for (int i = 0; i < OB_FRAME_METADATA_TYPE_COUNT; i++) {
    auto meta_data_type = static_cast<OBFrameMetadataType>(i);
    std::string field_name = metaDataTypeToString(meta_data_type);
    if (!frame->hasMetadata(meta_data_type)) {
      continue;
    }
    int64_t value = frame->getMetadataValue(meta_data_type);
    json_data[field_name] = value;
  }
  metadata_msg.json_data = json_data.dump(2);
  metadata_publisher.publish(metadata_msg);
}

void OBCameraNode::saveImageToFile(const stream_index_pair& stream_index, const cv::Mat& image,
                                   const sensor_msgs::ImagePtr& image_msg) {
  if (save_images_[stream_index]) {
    auto now = time(nullptr);
    std::stringstream ss;
    ss << std::put_time(localtime(&now), "%Y%m%d_%H%M%S");
    auto current_path = boost::filesystem::current_path().string();
    auto fps = fps_[stream_index];
    int index = save_images_count_[stream_index];
    std::string file_suffix = stream_index == COLOR ? ".png" : ".raw";
    std::string filename = current_path + "/image/" + stream_name_[stream_index] + "_" +
                           std::to_string(image_msg->width) + "x" +
                           std::to_string(image_msg->height) + "_" + std::to_string(fps) + "hz_" +
                           ss.str() + "_" + std::to_string(index) + file_suffix;
    if (!boost::filesystem::exists(current_path + "/image")) {
      boost::filesystem::create_directory(current_path + "/image");
    }
    ROS_INFO_STREAM("Saving image to " << filename);
    if (stream_index.first == OB_STREAM_COLOR) {
      auto image_to_save =
          cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8)->image;
      cv::imwrite(filename, image_to_save);
    } else if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
               stream_index.first == OB_STREAM_IR_RIGHT || stream_index.first == OB_STREAM_DEPTH) {
      std::ofstream ofs(filename, std::ios::out | std::ios::binary);
      if (!ofs.is_open()) {
        ROS_ERROR_STREAM("Failed to open file: " << filename);
        return;
      }
      if (image.isContinuous()) {
        ofs.write(reinterpret_cast<const char*>(image.data), image.total() * image.elemSize());
      } else {
        int rows = image.rows;
        int cols = image.cols * image.channels();
        for (int r = 0; r < rows; ++r) {
          ofs.write(reinterpret_cast<const char*>(image.ptr<uchar>(r)), cols);
        }
      }
      ofs.close();
    } else {
      ROS_ERROR_STREAM("Unsupported stream type: " << stream_index.first);
    }
    if (++save_images_count_[stream_index] >= max_save_images_count_) {
      save_images_[stream_index] = false;
    }
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
    if (enable_sync_output_accel_gyro_) {
      if (imu_sync_output_start_) {
        ROS_INFO_STREAM("IMU stream accel and gyro are already started.");
        return;
      }
    } else {
      if (imu_started_[stream_index]) {
        ROS_INFO_STREAM("IMU stream " << stream_name_[stream_index] << " is already started.");
        return;
      }
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
    for (auto& item : camera_info_publishers_) {
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
  if (enable_sync_output_accel_gyro_) {
    ROS_INFO_STREAM("IMU stream accel and gyro unsubscribed");
  } else {
    ROS_INFO_STREAM("IMU stream " << stream_name_[stream_index] << " unsubscribed");
  }
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (imu_publishers_.count(stream_index) > 0) {
    auto subscriber_count = imu_publishers_[stream_index].getNumSubscribers();
    if (subscriber_count > 0) {
      return;
    }
  }
  if (imu_gyro_accel_publisher_.getNumSubscribers() > 0) {
    return;
  }
  if (imu_info_publishers_.count(stream_index) > 0) {
    auto subscriber_count = imu_info_publishers_[stream_index].getNumSubscribers();
    if (subscriber_count > 0) {
      return;
    }
  }
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
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  tf2::Vector3 zero_trans(0, 0, 0);
  if (!stream_profile_.count(base_stream_)) {
    ROS_ERROR_STREAM("Base stream is not available");
    return;
  }
  auto base_stream_profile = stream_profile_[base_stream_];
  CHECK_NOTNULL(base_stream_profile.get());
  for (const auto& item : stream_profile_) {
    auto stream_index = item.first;
    auto stream_profile = item.second;
    if (!stream_profile) {
      continue;
    }
    OBExtrinsic ex;
    try {
      ex = stream_profile->getExtrinsicTo(base_stream_profile);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << stream_name_[stream_index]
                                        << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }

    auto Q = rotationMatrixToQuaternion(ex.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    Q = Q.normalize();
    tf2::Vector3 trans(ex.trans[0], ex.trans[1], ex.trans[2]);

    auto timestamp = ros::Time::now();
    if (stream_index.first != base_stream_.first) {
      if (stream_index.first == OB_STREAM_IR_RIGHT && base_stream_.first == OB_STREAM_DEPTH) {
        trans[0] = std::abs(trans[0]);  // because left and right ir calibration is error
      }
      publishStaticTF(timestamp, trans, Q, frame_id_[base_stream_], frame_id_[stream_index]);
    }
    publishStaticTF(timestamp, zero_trans, quaternion_optical, frame_id_[stream_index],
                    optical_frame_id_[stream_index]);
    ROS_INFO_STREAM("Publishing static transform from " << stream_name_[stream_index] << " to "
                                                        << stream_name_[base_stream_]);
    ROS_INFO_STREAM("Translation " << trans[0] << ", " << trans[1] << ", " << trans[2]);
    ROS_INFO_STREAM("Rotation " << Q.getX() << ", " << Q.getY() << ", " << Q.getZ() << ", "
                                << Q.getW());
  }
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->pid();
  if ((pid == FEMTO_BOLT_PID || pid == FEMTO_MEGA_PID) && enable_stream_[DEPTH] &&
      enable_stream_[COLOR]) {
    // calc depth to color
    CHECK_NOTNULL(stream_profile_[COLOR]);
    auto depth_to_color_extrinsics = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    auto Q = rotationMatrixToQuaternion(depth_to_color_extrinsics.rot);
    Q = quaternion_optical * Q * quaternion_optical.inverse();
    Q = Q.normalize();
    publishStaticTF(ros::Time::now(), zero_trans, Q, camera_link_frame_id_,
                    frame_id_[base_stream_]);
  } else {
    publishStaticTF(ros::Time::now(), zero_trans, zero_rot, camera_link_frame_id_,
                    frame_id_[base_stream_]);
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
  auto base_stream_profile = stream_profile_[base_stream_];
  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    static const char* frame_id = "depth_to_color_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[COLOR]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[COLOR] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[COLOR].publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
    static const char* frame_id = "depth_to_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA0]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[INFRA0] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[INFRA0].publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
    static const char* frame_id = "depth_to_left_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA1]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[INFRA1] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[INFRA1].publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
    static const char* frame_id = "depth_to_right_ir_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[INFRA2]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[INFRA2] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[INFRA2].publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[ACCEL]) {
    static const char* frame_id = "depth_to_accel_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[ACCEL]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[ACCEL] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[ACCEL].publish(ex_msg);
  }
  if (enable_stream_[DEPTH] && enable_stream_[GYRO]) {
    static const char* frame_id = "depth_to_gyro_extrinsics";
    OBExtrinsic ex;
    try {
      ex = base_stream_profile->getExtrinsicTo(stream_profile_[GYRO]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get " << frame_id << " extrinsic: " << e.getMessage());
      ex = OBExtrinsic({{1, 0, 0, 0, 1, 0, 0, 0, 1}, {0, 0, 0}});
    }
    depth_to_other_extrinsics_[GYRO] = ex;
    auto ex_msg = obExtrinsicsToMsg(ex, frame_id);
    depth_to_other_extrinsics_publishers_[GYRO].publish(ex_msg);
  }
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    CHECK_NOTNULL(static_tf_broadcaster_.get());
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

bool OBCameraNode::isGemini335PID(uint32_t pid) {
  const uint16_t GEMINI_335_PID = 0x0800;    // Gemini 335 / 335e
  const uint16_t GEMINI_330_PID = 0x0801;    // Gemini 330
  const uint16_t GEMINI_336_PID = 0x0803;    // Gemini 336 / 336e
  const uint16_t GEMINI_335L_PID = 0x0804;   // Gemini 335L
  const uint16_t GEMINI_330L_PID = 0x0805;   // Gemini 336L
  const uint16_t GEMINI_336L_PID = 0x0807;   // Gemini 335Lg
  const uint16_t GEMINI_335LG_PID = 0x080B;  // Gemini 336Lg
  const uint16_t GEMINI_336LG_PID = 0x080D;
  const uint16_t GEMINI_335LE_PID = 0x080E;  // Gemini 335Le
  const uint16_t GEMINI_336LE_PID = 0x0810;  // Gemini 335Le
  return pid == GEMINI_335_PID || pid == GEMINI_330_PID || pid == GEMINI_336_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_330L_PID || pid == GEMINI_336L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_336LG_PID || pid == GEMINI_335LE_PID ||
         pid == GEMINI_336LE_PID;
}

}  // namespace orbbec_camera
