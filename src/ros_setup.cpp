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
#include "orbbec_camera/utils.h"
#include <std_msgs/String.h>

namespace orbbec_camera {

void OBCameraNode::setupConfig() {
  stream_name_[DEPTH] = "depth";
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[DEPTH] = OB_FORMAT_Y16;
  image_format_[DEPTH] = CV_16UC1;
  encoding_[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;
  format_str_[DEPTH] = "Y16";

  stream_name_[COLOR] = "color";
  unit_step_size_[COLOR] = 3;
  format_[COLOR] = OB_FORMAT_RGB888;
  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::RGB8;
  format_str_[COLOR] = "RGB";

  stream_name_[INFRA0] = "ir";
  unit_step_size_[INFRA0] = sizeof(uint16_t);
  format_[INFRA0] = OB_FORMAT_Y16;
  image_format_[INFRA0] = CV_16UC1;
  encoding_[INFRA0] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA0] = "Y16";

  stream_name_[INFRA1] = "left_ir";
  unit_step_size_[INFRA1] = sizeof(uint16_t);
  format_[INFRA1] = OB_FORMAT_Y16;
  image_format_[INFRA1] = CV_16UC1;
  encoding_[INFRA1] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA1] = "Y16";

  stream_name_[INFRA2] = "right_ir";
  unit_step_size_[INFRA2] = sizeof(uint16_t);
  format_[INFRA2] = OB_FORMAT_Y16;
  image_format_[INFRA2] = CV_16UC1;
  encoding_[INFRA2] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA2] = "Y16";
}

void OBCameraNode::selectBaseStream() {
  if (enable_stream_[DEPTH]) {
    base_stream_ = DEPTH;
  } else if (enable_stream_[INFRA0]) {
    base_stream_ = INFRA0;
  } else if (enable_stream_[INFRA1]) {
    base_stream_ = INFRA1;
  } else if (enable_stream_[INFRA2]) {
    base_stream_ = INFRA2;
  } else if (enable_stream_[COLOR]) {
    base_stream_ = COLOR;
  } else {
    ROS_ERROR_STREAM("No base stream is enabled!");
  }
}
void OBCameraNode::setupColorPostProcessFilter() {
  auto color_sensor = device_->getSensor(OB_SENSOR_COLOR);
  color_filter_list_ = color_sensor->createRecommendedFilters();
  if (color_filter_list_.empty()) {
    ROS_ERROR_STREAM("Failed to get color sensor filter list");
    return;
  }
  for (size_t i = 0; i < color_filter_list_.size(); i++) {
    auto filter = color_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_color_decimation_filter_},
    };
    std::string filter_name = filter->type();
    ROS_INFO_STREAM("Setting " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end()) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("set color " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
    }
    if (filter_name == "DecimationFilter" && enable_color_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      auto range = decimation_filter->getScaleRange();
      if (color_decimation_filter_scale_ != -1 && color_decimation_filter_scale_ <= range.max &&
          color_decimation_filter_scale_ >= range.min) {
        ROS_INFO_STREAM("Set color decimation filter scale value to "
                        << color_decimation_filter_scale_);
        decimation_filter->setScaleValue(color_decimation_filter_scale_);
      }
      if (color_decimation_filter_scale_ != -1 && (color_decimation_filter_scale_ < range.min ||
                                                   color_decimation_filter_scale_ > range.max)) {
        ROS_ERROR_STREAM("Color Decimation filter scale value is out of range "
                         << range.min << " - " << range.max);
      }
    }
  }
}
void OBCameraNode::setupLeftIrPostProcessFilter() {
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
  if (isGemini335PID(pid)) {
    auto left_ir_sensor = device_->getSensor(OB_SENSOR_IR_LEFT);
    left_ir_filter_list_ = left_ir_sensor->createRecommendedFilters();
    if (left_ir_filter_list_.empty()) {
      ROS_WARN_STREAM("Failed to get left ir sensor filter list");
      return;
    }
    for (size_t i = 0; i < left_ir_filter_list_.size(); i++) {
      auto filter = left_ir_filter_list_[i];
      std::map<std::string, bool> filter_params = {
          {"SequenceIdFilter", enable_left_ir_sequence_id_filter_},
      };
      std::string filter_name = filter->type();
      ROS_INFO_STREAM("Setting " << filter_name << "......");
      if (filter_params.find(filter_name) != filter_params.end()) {
        std::string value = filter_params[filter_name] ? "true" : "false";
        ROS_INFO_STREAM("set left ir " << filter_name << " to " << value);
        filter->enable(filter_params[filter_name]);
      }
      if (filter_name == "SequenceIdFilter" && enable_left_ir_sequence_id_filter_) {
        auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
        if (left_ir_sequence_id_filter_id_ != -1) {
          sequenced_filter->selectSequenceId(left_ir_sequence_id_filter_id_);
          ROS_INFO_STREAM("Set left ir SequenceIdFilter ID to " << left_ir_sequence_id_filter_id_);
        }
      }
    }
  }
}

void OBCameraNode::setupRightIrPostProcessFilter() {
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
  if (isGemini335PID(pid)) {
    auto right_ir_sensor = device_->getSensor(OB_SENSOR_IR_RIGHT);
    right_ir_filter_list_ = right_ir_sensor->createRecommendedFilters();
    if (right_ir_filter_list_.empty()) {
      ROS_WARN_STREAM("Failed to get right ir sensor filter list");
      return;
    }
    for (size_t i = 0; i < right_ir_filter_list_.size(); i++) {
      auto filter = right_ir_filter_list_[i];
      std::map<std::string, bool> filter_params = {
          {"SequenceIdFilter", enable_right_ir_sequence_id_filter_},
      };
      std::string filter_name = filter->type();
      ROS_INFO_STREAM("Setting " << filter_name << "......");
      if (filter_params.find(filter_name) != filter_params.end()) {
        std::string value = filter_params[filter_name] ? "true" : "false";
        ROS_INFO_STREAM("set right ir " << filter_name << " to " << value);
        filter->enable(filter_params[filter_name]);
      }
      if (filter_name == "SequenceIdFilter" && enable_right_ir_sequence_id_filter_) {
        auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
        if (right_ir_sequence_id_filter_id_ != -1) {
          sequenced_filter->selectSequenceId(right_ir_sequence_id_filter_id_);
          ROS_INFO_STREAM("Set right ir SequenceIdFilter ID to "
                          << right_ir_sequence_id_filter_id_);
        }
      }
    }
  }
}

void OBCameraNode::setupDepthPostProcessFilter() {
  // set depth sensor to filter
  auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
  depth_filter_list_ = depth_sensor->createRecommendedFilters();
  if (depth_filter_list_.empty()) {
    ROS_WARN_STREAM("Failed to get depth sensor filter list");
    return;
  }
  for (size_t i = 0; i < depth_filter_list_.size(); i++) {
    auto filter = depth_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_decimation_filter_},
        {"HDRMerge", enable_hdr_merge_},
        {"SequenceIdFilter", enable_sequenced_filter_},
        {"SpatialAdvancedFilter", enable_spatial_filter_},
        {"TemporalFilter", enable_temporal_filter_},
        {"HoleFillingFilter", enable_hole_filling_filter_},
        {"DisparityTransform", enable_disaparity_to_depth_},
        {"ThresholdFilter", enable_threshold_filter_},
    };
    std::string filter_name = filter->type();
    ROS_INFO_STREAM("Setting " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end()) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("set " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
      filter_status_[filter_name] = filter_params[filter_name];
    }
    if (filter_name == "DecimationFilter" && enable_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      if (decimation_filter_scale_range_ != -1) {
        decimation_filter->setScaleValue(decimation_filter_scale_range_);
      }
    } else if (filter_name == "ThresholdFilter" && enable_threshold_filter_) {
      auto threshold_filter = filter->as<ob::ThresholdFilter>();
      if (threshold_filter_min_ != -1 && threshold_filter_max_ != -1) {
        threshold_filter->setValueRange(threshold_filter_min_, threshold_filter_max_);
      }
    } else if (filter_name == "SpatialAdvancedFilter" && enable_spatial_filter_) {
      auto spatial_filter = filter->as<ob::SpatialAdvancedFilter>();
      OBSpatialAdvancedFilterParams params{};
      if (spatial_filter_alpha_ != -1.0 && spatial_filter_magnitude_ != -1 &&
          spatial_filter_radius_ != -1 && spatial_filter_diff_threshold_ != -1) {
        params.alpha = spatial_filter_alpha_;
        params.magnitude = spatial_filter_magnitude_;
        params.radius = spatial_filter_radius_;
        params.disp_diff = spatial_filter_diff_threshold_;
        spatial_filter->setFilterParams(params);
      }
    } else if (filter_name == "TemporalFilter" && enable_temporal_filter_) {
      auto temporal_filter = filter->as<ob::TemporalFilter>();
      if (temporal_filter_diff_threshold_ != -1 && temporal_filter_weight_ != -1) {
        temporal_filter->setDiffScale(temporal_filter_diff_threshold_);
        temporal_filter->setWeight(temporal_filter_weight_);
      }
    } else if (filter_name == "HoleFillingFilter" && enable_hole_filling_filter_ &&
               !hole_filling_filter_mode_.empty()) {
      auto hole_filling_filter = filter->as<ob::HoleFillingFilter>();
      OBHoleFillingMode hole_filling_mode = holeFillingModeFromString(hole_filling_filter_mode_);
      hole_filling_filter->setFilterMode(hole_filling_mode);
    } else if (filter_name == "SequenceIdFilter" && enable_sequenced_filter_) {
      auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
      if (sequence_id_filter_id_ != -1) {
        sequenced_filter->selectSequenceId(sequence_id_filter_id_);
      }
    } else if (filter_name == "HDRMerge" && enable_hdr_merge_) {
      auto hdr_merge_filter = filter->as<ob::HdrMerge>();
      OBHdrConfig hdr_config{};
      if (hdr_merge_exposure_1_ != -1 && hdr_merge_exposure_2_ != -1 && hdr_merge_gain_1_ != -1 &&
          hdr_merge_gain_2_ != -1) {
        hdr_config.exposure_1 = hdr_merge_exposure_1_;
        hdr_config.exposure_2 = hdr_merge_exposure_2_;
        hdr_config.gain_1 = hdr_merge_gain_1_;
        hdr_config.gain_2 = hdr_merge_gain_2_;
        hdr_config.enable = true;
        ROS_INFO_STREAM("set HDRMerge exposure_1 to " << hdr_merge_exposure_1_);
        ROS_INFO_STREAM("set HDRMerge exposure_2 to " << hdr_merge_exposure_2_);
        ROS_INFO_STREAM("set HDRMerge gain_1 to " << hdr_merge_gain_1_);
        ROS_INFO_STREAM("set HDRMerge gain_2 to " << hdr_merge_gain_2_);
        device_->setStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                   reinterpret_cast<const uint8_t*>(&hdr_config),
                                   sizeof(OBHdrConfig));
      }
      hdr_merge_filter->enable(true);
    } else {
      ROS_INFO_STREAM("Skip setting " << filter_name);
    }
  }
  set_filter_srv_ = nh_.advertiseService<SetFilterRequest, SetFilterResponse>(
      "/" + camera_name_ + "/" + "set_filter",
      [this](SetFilterRequest& request, SetFilterResponse& response) {
        response.success = setFilterCallback(request, response);
        return response.success;
      });
}
void OBCameraNode::setupDevices() {
  auto sensor_list = device_->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profiles = sensor->getStreamProfileList();
    for (size_t j = 0; j < profiles->count(); j++) {
      auto profile = profiles->getProfile(j);
      stream_index_pair sip{profile->type(), 0};
      if (sensors_.find(sip) == sensors_.end()) {
        sensors_[sip] = std::make_shared<ROSOBSensor>(device_, sensor, stream_name_[sip]);
      }
      if (imu_sensor_.find(sip) == imu_sensor_.end()) {
        imu_sensor_[sip] = sensor;
      }
    }
  }
  for (const auto& item : enable_stream_) {
    auto stream_index = item.first;
    auto enable = item.second;
    if (enable && sensors_.find(stream_index) == sensors_.end()) {
      ROS_INFO_STREAM(stream_name_[stream_index]
                      << "sensor isn't supported by current device! -- Skipping...");
      enable_stream_[stream_index] = false;
    }
    if (enable) {
      // set rotation
      OBPropertyID rotationPropertyID = OB_PROP_DEPTH_ROTATE_INT;
      if (stream_index == COLOR) {
        rotationPropertyID = OB_PROP_COLOR_ROTATE_INT;
      } else if (stream_index == DEPTH) {
        rotationPropertyID = OB_PROP_DEPTH_ROTATE_INT;
      } else if (stream_index == INFRA0) {
        rotationPropertyID = OB_PROP_IR_ROTATE_INT;
      } else if (stream_index == INFRA1) {
        rotationPropertyID = OB_PROP_IR_ROTATE_INT;
      } else if (stream_index == INFRA2) {
        rotationPropertyID = OB_PROP_IR_RIGHT_ROTATE_INT;
      }
      if (image_rotation_[stream_index] != -1 &&
          device_->isPropertySupported(rotationPropertyID, OB_PERMISSION_WRITE)) {
        ROS_INFO_STREAM("Setting " << stream_name_[stream_index] << " rotation to "
                                   << image_rotation_[stream_index]);
        device_->setIntProperty(rotationPropertyID, image_rotation_[stream_index]);
      }
      // set flip
      OBPropertyID flipPropertyID = OB_PROP_DEPTH_FLIP_BOOL;
      if (stream_index == COLOR) {
        flipPropertyID = OB_PROP_COLOR_FLIP_BOOL;
      } else if (stream_index == DEPTH) {
        flipPropertyID = OB_PROP_DEPTH_FLIP_BOOL;
      } else if (stream_index == INFRA0) {
        flipPropertyID = OB_PROP_IR_FLIP_BOOL;
      } else if (stream_index == INFRA1) {
        flipPropertyID = OB_PROP_IR_FLIP_BOOL;
      } else if (stream_index == INFRA2) {
        flipPropertyID = OB_PROP_IR_RIGHT_FLIP_BOOL;
      }
      if (device_->isPropertySupported(flipPropertyID, OB_PERMISSION_WRITE)) {
        ROS_INFO_STREAM("Setting " << stream_name_[stream_index] << " flip to "
                                   << (image_flip_[stream_index] ? "ON" : "OFF"));
        device_->setBoolProperty(flipPropertyID, image_flip_[stream_index]);
      }
      // set mirror
      OBPropertyID mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      if (stream_index == COLOR) {
        mirrorPropertyID = OB_PROP_COLOR_MIRROR_BOOL;
      } else if (stream_index == DEPTH) {
        mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      } else if (stream_index == INFRA0) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;
      } else if (stream_index == INFRA1) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;
      } else if (stream_index == INFRA2) {
        mirrorPropertyID = OB_PROP_IR_RIGHT_MIRROR_BOOL;
      }
      if (device_->isPropertySupported(mirrorPropertyID, OB_PERMISSION_WRITE)) {
        ROS_INFO_STREAM("Setting " << stream_name_[stream_index] << " mirror to "
                                   << (image_mirror_[stream_index] ? "ON" : "OFF"));
        device_->setBoolProperty(mirrorPropertyID, image_mirror_[stream_index]);
      }
    }
  }
  if (enable_d2c_viewer_) {
    d2c_viewer_ = std::make_shared<D2CViewer>(nh_, nh_private_);
  }
  CHECK_NOTNULL(device_info_.get());
  if (enable_pipeline_) {
    pipeline_ = std::make_shared<ob::Pipeline>(device_);
  }
  if (enable_sync_output_accel_gyro_) {
    imuPipeline_ = std::make_shared<ob::Pipeline>(device_);
  }

  try {
    if (retry_on_usb3_detection_failure_ &&
        device_->isPropertySupported(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                               retry_on_usb3_detection_failure_);
    }
    if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
      auto default_noise_removal_filter_min_diff =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
      ROS_INFO_STREAM(
          "default_noise_removal_filter_min_diff: " << default_noise_removal_filter_min_diff);
      if (noise_removal_filter_min_diff_ != -1 &&
          default_noise_removal_filter_min_diff != noise_removal_filter_min_diff_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, noise_removal_filter_min_diff_);
        auto new_noise_removal_filter_min_diff =
            device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
        ROS_INFO_STREAM(
            "after set noise_removal_filter_min_diff: " << new_noise_removal_filter_min_diff);
      }
    }
    if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
      auto default_noise_removal_filter_max_size =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
      ROS_INFO_STREAM(
          "default_noise_removal_filter_max_size: " << default_noise_removal_filter_max_size);
      if (noise_removal_filter_max_size_ != -1 &&
          default_noise_removal_filter_max_size != noise_removal_filter_max_size_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, noise_removal_filter_max_size_);
        auto new_noise_removal_filter_max_size =
            device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
        ROS_INFO_STREAM(
            "after set noise_removal_filter_max_size: " << new_noise_removal_filter_max_size);
      }
    }
    if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_noise_removal_filter_);
      ROS_INFO_STREAM("enable_noise_removal_filter:" << enable_noise_removal_filter_);
    }
    if (!depth_work_mode_.empty()) {
      ROS_INFO_STREAM("Set depth work mode: " << depth_work_mode_);
      device_->switchDepthWorkMode(depth_work_mode_.c_str());
    }
    if (laser_energy_level_ != -1 &&
        device_->isPropertySupported(OB_PROP_LASER_ENERGY_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
      ROS_INFO_STREAM("Setting laser energy level to " << laser_energy_level_);
      auto range = device_->getIntPropertyRange(OB_PROP_LASER_ENERGY_LEVEL_INT);
      if (laser_energy_level_ < range.min || laser_energy_level_ > range.max) {
        ROS_INFO_STREAM("Laser energy level is out of range " << range.min << " - " << range.max);
      } else {
        device_->setIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT, laser_energy_level_);
        auto new_laser_energy_level = device_->getIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT);
        ROS_INFO_STREAM("Laser energy level set to " << new_laser_energy_level << " (new value)");
      }
    }
    if (device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_READ_WRITE)) {
      ROS_INFO_STREAM("Setting LDP to " << (enable_ldp_ ? "true" : "false"));
      if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
        auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
        device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
        device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
      } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
        if (!enable_ldp_) {
          auto laser_enable = device_->getIntProperty(OB_PROP_LASER_BOOL);
          device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
          std::this_thread::sleep_for(std::chrono::milliseconds(3));
          device_->setIntProperty(OB_PROP_LASER_BOOL, laser_enable);
        } else {
          device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
        }
      }
    }
    if (device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
      ROS_INFO_STREAM("Setting heartbeat to " << (enable_heartbeat_ ? "true" : "false"));
      device_->setBoolProperty(OB_PROP_HEARTBEAT_BOOL, enable_heartbeat_);
    }

    if (enable_color_hdr_ &&
        device_->isPropertySupported(OB_PROP_COLOR_HDR_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_HDR_BOOL, enable_color_hdr_);
    }
    if (disaparity_to_depth_mode_ == "HW") {
      device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 1);
      device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 0);
      ROS_INFO_STREAM("Depth process is HW");
    } else if (disaparity_to_depth_mode_ == "SW") {
      device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 0);
      device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 1);
      ROS_INFO_STREAM("Depth process is SW");
    } else if (disaparity_to_depth_mode_ == "disable") {
      device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 0);
      device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 0);
      ROS_INFO_STREAM("Depth process is disable");
    } else {
      ROS_ERROR_STREAM("Depth process is keep default");
    }
    if (!device_preset_.empty()) {
      ROS_INFO_STREAM("Loading device preset: " << device_preset_);
      if (isGemini335PID(device_info_->pid())) {
        device_->loadPreset(device_preset_.c_str());
      }
    }
    if (!sync_mode_str_.empty() &&
        device_->isPropertySupported(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      auto sync_config = device_->getMultiDeviceSyncConfig();
      ROS_INFO_STREAM("current sync mode: " << sync_config.syncMode);
      std::transform(sync_mode_str_.begin(), sync_mode_str_.end(), sync_mode_str_.begin(),
                     ::toupper);
      sync_mode_ = OBSyncModeFromString(sync_mode_str_);
      sync_config.syncMode = sync_mode_;
      sync_config.depthDelayUs = depth_delay_us_;
      sync_config.colorDelayUs = color_delay_us_;
      sync_config.trigger2ImageDelayUs = trigger2image_delay_us_;
      sync_config.triggerOutDelayUs = trigger_out_delay_us_;
      sync_config.triggerOutEnable = trigger_out_enabled_;
      sync_config.framesPerTrigger = frames_per_trigger_;
      device_->setMultiDeviceSyncConfig(sync_config);
      sync_config = device_->getMultiDeviceSyncConfig();
      ROS_INFO_STREAM("set sync mode to " << sync_config.syncMode);
      if (sync_mode_ == OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
        ROS_INFO_STREAM("Frames per trigger: " << sync_config.framesPerTrigger);
        sync_host_time_timer_ =
            nh_private_.createTimer(ros::Duration(0, software_trigger_period_ * 1000000),
                                    [this](const ros::TimerEvent&) { device_->triggerCapture(); });
      }
    }

    if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT,
                                     OB_PERMISSION_WRITE)) {
      int set_enable_color_auto_exposure_priority = enable_color_auto_exposure_priority_ ? 1 : 0;
      ROS_INFO_STREAM("Setting color auto exposure priority to "
                      << (set_enable_color_auto_exposure_priority ? "ON" : "OFF"));
      device_->setIntProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT,
                              set_enable_color_auto_exposure_priority);
    }
    if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, OB_PERMISSION_WRITE)) {
      ROS_INFO_STREAM("Setting color auto white balance to "
                      << (enable_color_auto_white_balance_ ? "ON" : "OFF"));
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
                               enable_color_auto_white_balance_);
    }
    if (device_->isPropertySupported(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT,
                                     OB_PERMISSION_WRITE)) {
      int set_enable_color_backlight_compenstation = enable_color_backlight_compenstation_ ? 1 : 0;
      ROS_INFO_STREAM("Setting color backlight compenstation to "
                      << (set_enable_color_backlight_compenstation ? "ON" : "OFF"));
      device_->setIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT,
                              set_enable_color_backlight_compenstation);
    }
    if (!color_powerline_freq_.empty() &&
        device_->isPropertySupported(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, OB_PERMISSION_WRITE)) {
      if (color_powerline_freq_ == "disable") {
        device_->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, 0);
      } else if (color_powerline_freq_ == "50hz") {
        device_->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, 1);
      } else if (color_powerline_freq_ == "60hz") {
        device_->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, 2);
      } else if (color_powerline_freq_ == "auto") {
        device_->setIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT, 3);
      }
      ROS_INFO_STREAM("Setting color powerline freq to " << color_powerline_freq_);
    }
    if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    }
    if (color_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_EXPOSURE_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, color_exposure_);
    }
    if (color_gain_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_GAIN_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_GAIN_INT);
      if (color_gain_ < range.min || color_gain_ > range.max) {
        ROS_ERROR_STREAM("color gain value is out of range[" << range.min << "," << range.max
                                                             << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color gain to " << color_gain_);
        device_->setIntProperty(OB_PROP_COLOR_GAIN_INT, color_gain_);
      }
    }
    if (color_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_BRIGHTNESS_INT);
      if (color_brightness_ < range.min || color_brightness_ > range.max) {
        ROS_ERROR_STREAM("color brightness value is out of range[" << range.min << "," << range.max
                                                                   << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color brightness to " << color_brightness_);
        device_->setIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT, color_brightness_);
      }
    }
    if (color_sharpness_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_SHARPNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_SHARPNESS_INT);
      if (color_sharpness_ < range.min || color_sharpness_ > range.max) {
        ROS_ERROR_STREAM("color sharpness value is out of range[" << range.min << "," << range.max
                                                                  << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color sharpness to " << color_sharpness_);
        device_->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, color_sharpness_);
      }
    }
    if (color_gamma_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_GAMMA_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_GAMMA_INT);
      if (color_gamma_ < range.min || color_gamma_ > range.max) {
        ROS_ERROR_STREAM("color gamm value is out of range[" << range.min << "," << range.max
                                                             << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color gamm to " << color_gamma_);
        device_->setIntProperty(OB_PROP_COLOR_GAMMA_INT, color_gamma_);
      }
    }
    if (color_white_balance_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_WHITE_BALANCE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_WHITE_BALANCE_INT);
      if (color_white_balance_ < range.min || color_white_balance_ > range.max) {
        ROS_ERROR_STREAM("color white balance value is out of range["
                         << range.min << "," << range.max << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color white balance to " << color_white_balance_);
        device_->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, color_white_balance_);
      }
    }
    if (color_saturation_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_SATURATION_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_SATURATION_INT);
      if (color_saturation_ < range.min || color_saturation_ > range.max) {
        ROS_ERROR_STREAM("color saturation value is out of range[" << range.min << "," << range.max
                                                                   << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color saturation to " << color_saturation_);
        device_->setIntProperty(OB_PROP_COLOR_SATURATION_INT, color_saturation_);
      }
    }
    if (color_constrast_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_CONTRAST_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_CONTRAST_INT);
      if (color_constrast_ < range.min || color_constrast_ > range.max) {
        ROS_ERROR_STREAM("color constrast value is out of range[" << range.min << "," << range.max
                                                                  << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color constrast to " << color_constrast_);
        device_->setIntProperty(OB_PROP_COLOR_CONTRAST_INT, color_constrast_);
      }
    }
    if (color_hue_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_HUE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_HUE_INT);
      if (color_hue_ < range.min || color_hue_ > range.max) {
        ROS_ERROR_STREAM("color hue value is out of range[" << range.min << "," << range.max
                                                            << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color hue to " << color_hue_);
        device_->setIntProperty(OB_PROP_COLOR_HUE_INT, color_hue_);
      }
    }
    if (color_ae_max_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT);
      if (color_ae_max_exposure_ < range.min || color_ae_max_exposure_ > range.max) {
        ROS_ERROR_STREAM("color AE max exposure value is out of range["
                         << range.min << "," << range.max << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting color AE max exposure to " << color_ae_max_exposure_);
        device_->setIntProperty(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, color_ae_max_exposure_);
      }
    }
    if (device_->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    }
    if (device_->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT,
                                     OB_PERMISSION_WRITE)) {
      int set_enable_depth_auto_exposure_priority = enable_depth_auto_exposure_priority_ ? 1 : 0;
      ROS_INFO_STREAM("Setting depth auto exposure priority to "
                      << (set_enable_depth_auto_exposure_priority ? "ON" : "OFF"));
      device_->setIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT,
                              set_enable_depth_auto_exposure_priority);
    }
    if (depth_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT);
      if (depth_brightness_ < range.min || depth_brightness_ > range.max) {
        ROS_ERROR_STREAM("depth brightness value is out of range[" << range.min << "," << range.max
                                                                   << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting depth brightness to " << depth_brightness_);
        device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, depth_brightness_);
      }
    }
    if (ir_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_DEPTH_EXPOSURE_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, ir_exposure_);
    }
    if (ir_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT);
      if (ir_brightness_ < range.min || ir_brightness_ > range.max) {
        ROS_ERROR_STREAM("IR brightness value is out of range[" << range.min << "," << range.max
                                                                << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting IR brightness to " << ir_brightness_);
        device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, ir_brightness_);
      }
    }
    if (ir_ae_max_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_AE_MAX_EXPOSURE_INT);
      if (ir_ae_max_exposure_ < range.min || ir_ae_max_exposure_ > range.max) {
        ROS_ERROR_STREAM("IR AE max exposure value is out of range["
                         << range.min << "," << range.max << "]please check the value");
      } else {
        ROS_INFO_STREAM("Setting IR AE max exposure to " << ir_ae_max_exposure_);
        device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, ir_ae_max_exposure_);
      }
    }
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, enable_laser_);
    }
    if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_BOOL, enable_laser_);
    }
    if (device_->isPropertySupported(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      ROS_INFO_STREAM("Set PTP Config: " << (enable_ptp_config_ ? "ON" : "OFF"));
      device_->setBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL, enable_ptp_config_);
    }
    if (!depth_precision_str_.empty() &&
        device_->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
      auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      if (default_precision_level != depth_precision_level_) {
        device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_level_);
        ROS_INFO_STREAM("set depth precision to " << depth_precision_str_);
      }
    } else if (!depth_precision_str_.empty() &&
               device_->isPropertySupported(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                            OB_PERMISSION_READ_WRITE)) {
      auto depth_unit_flexible_adjustment = depthPrecisionFromString(depth_precision_str_);
      auto range = device_->getFloatPropertyRange(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT);
      ROS_INFO_STREAM("Depth unit flexible adjustment range: " << range.min << " - " << range.max);
      if (depth_unit_flexible_adjustment < range.min ||
          depth_unit_flexible_adjustment > range.max) {
        ROS_ERROR_STREAM(
            "depth unit flexible adjustment value is out of range, please check the value");
      } else {
        ROS_INFO_STREAM("set depth unit to " << depth_unit_flexible_adjustment << "mm");
        device_->setFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                  depth_unit_flexible_adjustment);
      }
    }
    if (device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    }

    if (device_->isPropertySupported(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    }

    if (device_->isPropertySupported(OB_PROP_IR_LONG_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_LONG_EXPOSURE_BOOL, enable_ir_long_exposure_);
    }
    if (disparity_range_mode_ != -1 &&
        device_->isPropertySupported(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, OB_PERMISSION_WRITE)) {
      ROS_INFO_STREAM("Setting disparity range mode: " << disparity_range_mode_);
      if (disparity_range_mode_ == 64) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 0);
      } else if (disparity_range_mode_ == 128) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 1);
      } else if (disparity_range_mode_ == 256) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 2);
      } else {
        ROS_ERROR_STREAM("disparity range mode does not support this setting");
      }
    }
    if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                     OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                               enable_hardware_noise_removal_filter_);
      ROS_INFO_STREAM(
          "Setting hardware noise removal filter:" << enable_hardware_noise_removal_filter_);
      if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                       OB_PERMISSION_READ_WRITE)) {
        if (hardware_noise_removal_filter_threshold_ != -1.0 &&
            enable_hardware_noise_removal_filter_) {
          device_->setFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                    hardware_noise_removal_filter_threshold_);
          ROS_INFO_STREAM("Setting hardware noise removal filter threshold :"
                          << hardware_noise_removal_filter_threshold_);
        }
      }
    }
    if (exposure_range_mode_ != "default" &&
        device_->isPropertySupported(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, OB_PERMISSION_WRITE)) {
      ROS_INFO_STREAM("Setting exposure range mode : " << exposure_range_mode_);
      if (exposure_range_mode_ == "ultimate") {
        device_->setIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, 1);
      } else if (exposure_range_mode_ == "regular") {
        device_->setIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, 0);
      } else {
        ROS_ERROR_STREAM("exposure range mode does not support this setting");
      }
    }
    if (!load_config_json_file_path_.empty()) {
      device_->loadPresetFromJsonFile(load_config_json_file_path_.c_str());
      ROS_INFO_STREAM("Loading config json file path : " << load_config_json_file_path_);
    }
    if (!export_config_json_file_path_.empty()) {
      device_->exportSettingsAsPresetJsonFile(export_config_json_file_path_.c_str());
      ROS_INFO_STREAM("Exporting config json file path : " << export_config_json_file_path_);
    }
    if (device_->isPropertySupported(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL,
                                     OB_PERMISSION_WRITE)) {
      ROS_INFO_STREAM("Setting accel data correction to "
                      << (enable_accel_data_correction_ ? "ON" : "OFF"));
      device_->setBoolProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL,
                               enable_accel_data_correction_);
    }
    if (device_->isPropertySupported(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL,
                                     OB_PERMISSION_WRITE)) {
      ROS_INFO_STREAM("Setting gyro data correction to "
                      << (enable_gyro_data_correction_ ? "ON" : "OFF"));
      device_->setBoolProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL,
                               enable_gyro_data_correction_);
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to setup devices: " << e.getMessage());
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to setup devices: " << e.what());
  }
}

void OBCameraNode::setupFrameCallback() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      auto callback = [this, stream_index](std::shared_ptr<ob::Frame> frame) {
        this->onNewFrameCallback(frame, stream_index);
      };
      frame_callback_[stream_index] = callback;
    }
  }
}

bool OBCameraNode::setupFormatConvertType(OBFormat type) {
  switch (type) {
    case OB_FORMAT_I420:
      format_convert_filter_.setFormatConvertType(FORMAT_I420_TO_RGB888);
      break;
    case OB_FORMAT_MJPG:
      format_convert_filter_.setFormatConvertType(FORMAT_MJPEG_TO_RGB888);
      break;
    case OB_FORMAT_YUYV:
      format_convert_filter_.setFormatConvertType(FORMAT_YUYV_TO_RGB888);
      break;
    case OB_FORMAT_NV21:
      format_convert_filter_.setFormatConvertType(FORMAT_NV21_TO_RGB888);
      break;
    case OB_FORMAT_NV12:
      format_convert_filter_.setFormatConvertType(FORMAT_NV12_TO_RGB888);
      break;
    case OB_FORMAT_UYVY:
      format_convert_filter_.setFormatConvertType(FORMAT_UYVY_TO_RGB888);
      break;
    default:
      return false;
  }
  return true;
}

void OBCameraNode::printProfiles(const std::shared_ptr<ob::Sensor>& sensor) {
  auto profiles = sensor->getStreamProfileList();
  for (size_t j = 0; j < profiles->count(); j++) {
    auto origin_profile = profiles->getProfile(j);
    if (sensor->getType() == OB_SENSOR_COLOR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      ROS_INFO_STREAM("available color profile: " << profile->width() << "x" << profile->height()
                                                  << " " << profile->fps() << "fps "
                                                  << profile->format());
    } else if (sensor->getType() == OB_SENSOR_DEPTH) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      ROS_INFO_STREAM("available depth profile: " << profile->width() << "x" << profile->height()
                                                  << " " << profile->fps() << "fps "
                                                  << profile->format());
    } else if (sensor->getType() == OB_SENSOR_IR) {
      auto profile = origin_profile->as<ob::VideoStreamProfile>();
      ROS_INFO_STREAM("available ir profile: " << profile->width() << "x" << profile->height()
                                               << " " << profile->fps() << "fps "
                                               << profile->format());
    } else if (sensor->getType() == OB_SENSOR_ACCEL) {
      auto profile = origin_profile->as<ob::AccelStreamProfile>();
      ROS_INFO_STREAM("available accel profile: sampleRate "
                      << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                      << fullAccelScaleRangeToString(profile->fullScaleRange()));
    } else if (sensor->getType() == OB_SENSOR_GYRO) {
      auto profile = origin_profile->as<ob::GyroStreamProfile>();
      ROS_INFO_STREAM("available gyro profile: sampleRate "
                      << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                      << fullGyroScaleRangeToString(profile->fullScaleRange()));
    } else {
      ROS_INFO_STREAM("unknown profile: " << sensor->getType());
    }
  }
}

void OBCameraNode::setupProfiles() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index] && stream_index != base_stream_) {
      continue;
    }
    try {
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      supported_profiles_[stream_index] = profile_list;
      std::shared_ptr<ob::VideoStreamProfile> selected_profile = nullptr;
      if (width_[stream_index] == 0 && height_[stream_index] == 0 && fps_[stream_index] == 0 &&
          format_[stream_index] == OB_FORMAT_UNKNOWN) {
        selected_profile = profile_list->getProfile(0)->as<ob::VideoStreamProfile>();
      } else {
        selected_profile = profile_list->getVideoStreamProfile(
            width_[stream_index], height_[stream_index], format_[stream_index], fps_[stream_index]);
      }

      auto default_profile = profile_list->getProfile(0)->as<ob::VideoStreamProfile>();
      if (!selected_profile) {
        ROS_WARN_STREAM("Given stream configuration is not supported by the device! "
                        << " Stream: " << stream_name_[stream_index]
                        << ", Width: " << width_[stream_index]
                        << ", Height: " << height_[stream_index] << ", FPS: " << fps_[stream_index]
                        << ", Format: " << format_[stream_index]);
        if (default_profile) {
          ROS_WARN_STREAM("Using default profile instead.");
          ROS_WARN_STREAM("default FPS " << default_profile->fps());
          selected_profile = default_profile;
        } else {
          ROS_WARN_STREAM(" NO default_profile found , Stream: " << stream_index.first
                                                                 << " will be disable");
          enable_stream_[stream_index] = false;
          continue;
        }
      }
      CHECK_NOTNULL(selected_profile.get());
      stream_profile_[stream_index] = selected_profile;
      int width = static_cast<int>(selected_profile->width());
      int height = static_cast<int>(selected_profile->height());
      int fps = static_cast<int>(selected_profile->fps());
      updateImageConfig(stream_index, selected_profile);
      width_[stream_index] = width;
      height_[stream_index] = height;
      fps_[stream_index] = fps;
      if (selected_profile->format() == OB_FORMAT_BGRA) {
        images_[stream_index] = cv::Mat(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        encoding_[COLOR] = sensor_msgs::image_encodings::BGRA8;
        unit_step_size_[stream_index] = 4;
      } else if (selected_profile->format() == OB_FORMAT_RGBA) {
        images_[stream_index] = cv::Mat(height, width, CV_8UC4, cv::Scalar(0, 0, 0, 0));
        encoding_[COLOR] = sensor_msgs::image_encodings::RGBA8;
        unit_step_size_[stream_index] = 4;
      } else {
        images_[stream_index] =
            cv::Mat(height, width, image_format_[stream_index], cv::Scalar(0, 0, 0));
      }
      ROS_INFO_STREAM(" stream " << stream_name_[stream_index] << " is enabled - width: " << width
                                 << ", height: " << height << ", fps: " << fps << ", "
                                 << "Format: " << selected_profile->format());
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to setup  "
                       << stream_name_[stream_index] << " profile: " << width_[stream_index] << "x"
                       << height_[stream_index] << " " << fps_[stream_index] << "fps "
                       << OBFormatToString(format_[stream_index]) << " ERROR:" << e.getMessage());
      printProfiles(sensors_[stream_index]->getSensor());
      ROS_ERROR(
          "Error: The device might be connected via USB 2.0. Please verify your launch file "
          "configuration and "
          "try again. The current process will now exit.");
      exit(1);
    }
  }
  // IMU
  for (const auto& stream_index : HID_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    try {
      auto profile_list = sensors_[stream_index]->getStreamProfileList();
      supported_profiles_[stream_index] = profile_list;
      if (stream_index == ACCEL) {
        auto full_scale_range = fullAccelScaleRangeFromString(imu_range_[stream_index]);
        auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
        auto profile = profile_list->getAccelStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      } else if (stream_index == GYRO) {
        auto full_scale_range = fullGyroScaleRangeFromString(imu_range_[stream_index]);
        auto sample_rate = sampleRateFromString(imu_rate_[stream_index]);
        auto profile = profile_list->getGyroStreamProfile(full_scale_range, sample_rate);
        stream_profile_[stream_index] = profile;
      }
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " full scale range "
                                << imu_range_[stream_index] << " sample rate "
                                << imu_rate_[stream_index]);
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to setup << " << stream_name_[stream_index]
                                             << " profile: " << e.getMessage());
      enable_stream_[stream_index] = false;
      stream_profile_[stream_index] = nullptr;
    }
  }
  if (!enable_pipeline_ && (depth_registration_ || enable_colored_point_cloud_)) {
    int index = getCameraParamIndex();
    try {
      device_->setIntProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_MODE_INT, index);
      device_->setBoolProperty(OB_PROP_DEPTH_ALIGN_HARDWARE_BOOL, depth_registration_);
    } catch (ob::Error& e) {
      ROS_ERROR_STREAM("set d2c error " << e.getMessage());
    }
  }
  if (depth_registration_ || enable_colored_point_cloud_) {
    align_filter_ = std::make_shared<ob::Align>(align_target_stream_);
  }
}
void OBCameraNode::updateImageConfig(
    const stream_index_pair& stream_index,
    const std::shared_ptr<ob::VideoStreamProfile>& selected_profile) {
  if (selected_profile->format() == OB_FORMAT_Y8) {
    image_format_[stream_index] = CV_8UC1;
    encoding_[stream_index] = stream_index.first == OB_STREAM_DEPTH
                                  ? sensor_msgs::image_encodings::TYPE_8UC1
                                  : sensor_msgs::image_encodings::MONO8;
    unit_step_size_[stream_index] = sizeof(uint8_t);
  }
  if (selected_profile->format() == OB_FORMAT_MJPG) {
    if (stream_index.first == OB_STREAM_IR || stream_index.first == OB_STREAM_IR_LEFT ||
        stream_index.first == OB_STREAM_IR_RIGHT) {
      image_format_[stream_index] = CV_8UC1;
      encoding_[stream_index] = sensor_msgs::image_encodings::MONO8;
      unit_step_size_[stream_index] = sizeof(uint8_t);
    }
  }
  if (selected_profile->format() == OB_FORMAT_Y16 && stream_index == COLOR) {
    image_format_[stream_index] = CV_16UC1;
    encoding_[stream_index] = sensor_msgs::image_encodings::MONO16;
    unit_step_size_[stream_index] = sizeof(uint16_t);
  }
}

void OBCameraNode::setupTopics() {
  setupPublishers();
  if (publish_tf_) {
    publishStaticTransforms();
  }
}

void OBCameraNode::setupPublishers() {
  image_transport::ImageTransport image_transport(nh_);
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string name = stream_name_[stream_index];
    std::string topic_name = "/" + camera_name_ + "/" + name + "/image_raw";
    image_transport::SubscriberStatusCallback it_subscribed_cb =
        boost::bind(&OBCameraNode::imageSubscribedCallback, this, stream_index);
    image_transport::SubscriberStatusCallback it_unsubscribed_cb =
        boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, stream_index);
    image_publishers_[stream_index] =
        image_transport.advertise(topic_name, 1, it_subscribed_cb, it_unsubscribed_cb);
    topic_name = "/" + camera_name_ + "/" + name + "/camera_info";
    ros::SubscriberStatusCallback image_subscribed_cb =
        boost::bind(&OBCameraNode::imageSubscribedCallback, this, stream_index);
    ros::SubscriberStatusCallback image_unsubscribed_cb =
        boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, stream_index);
    camera_info_publishers_[stream_index] = nh_.advertise<sensor_msgs::CameraInfo>(
        topic_name, 1, image_subscribed_cb, image_unsubscribed_cb);
    CHECK_NOTNULL(device_info_.get());
    if (isGemini335PID(device_info_->pid())) {
      metadata_publishers_[stream_index] =
          nh_.advertise<orbbec_camera::Metadata>("/" + camera_name_ + "/" + name + "/metadata", 1,
                                                 image_subscribed_cb, image_unsubscribed_cb);
    }
  }
  if (enable_point_cloud_ && enable_stream_[DEPTH]) {
    ros::SubscriberStatusCallback depth_cloud_subscribed_cb =
        boost::bind(&OBCameraNode::pointCloudSubscribedCallback, this);
    ros::SubscriberStatusCallback depth_cloud_unsubscribed_cb =
        boost::bind(&OBCameraNode::pointCloudUnsubscribedCallback, this);
    depth_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "depth/points", 1, depth_cloud_subscribed_cb, depth_cloud_unsubscribed_cb);
  }
  if (enable_colored_point_cloud_ && enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    ros::SubscriberStatusCallback depth_registered_cloud_subscribed_cb =
        boost::bind(&OBCameraNode::coloredPointCloudSubscribedCallback, this);
    ros::SubscriberStatusCallback depth_registered_cloud_unsubscribed_cb =
        boost::bind(&OBCameraNode::coloredPointCloudUnsubscribedCallback, this);
    depth_registered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "depth_registered/points", 1, depth_registered_cloud_subscribed_cb,
        depth_registered_cloud_unsubscribed_cb);
  }

  if (enable_sync_output_accel_gyro_) {
    std::string topic_name = stream_name_[GYRO] + "_" + stream_name_[ACCEL] + "/sample";
    ros::SubscriberStatusCallback imu_subscribed_cb =
        boost::bind(&OBCameraNode::imuSubscribedCallback, this, GYRO);
    ros::SubscriberStatusCallback imu_unsubscribed_cb =
        boost::bind(&OBCameraNode::imuUnsubscribedCallback, this, GYRO);
    imu_gyro_accel_publisher_ =
        nh_.advertise<sensor_msgs::Imu>(topic_name, 1, imu_subscribed_cb, imu_unsubscribed_cb);
    topic_name = stream_name_[GYRO] + "/imu_info";
    imu_info_publishers_[GYRO] = nh_.advertise<orbbec_camera::IMUInfo>(
        topic_name, 1, imu_subscribed_cb, imu_unsubscribed_cb);
    topic_name = stream_name_[ACCEL] + "/imu_info";
    imu_info_publishers_[ACCEL] = nh_.advertise<orbbec_camera::IMUInfo>(
        topic_name, 1, imu_subscribed_cb, imu_unsubscribed_cb);
  } else {
    for (const auto& stream_index : HID_STREAMS) {
      if (!enable_stream_[stream_index]) {
        continue;
      }
      std::string topic_name = stream_name_[stream_index] + "/sample";
      ros::SubscriberStatusCallback imu_subscribed_cb =
          boost::bind(&OBCameraNode::imuSubscribedCallback, this, stream_index);
      ros::SubscriberStatusCallback imu_unsubscribed_cb =
          boost::bind(&OBCameraNode::imuUnsubscribedCallback, this, stream_index);
      imu_publishers_[stream_index] =
          nh_.advertise<sensor_msgs::Imu>(topic_name, 1, imu_subscribed_cb, imu_unsubscribed_cb);
      topic_name = stream_name_[stream_index] + "/imu_info";
      imu_info_publishers_[stream_index] = nh_.advertise<orbbec_camera::IMUInfo>(
          topic_name, 1, imu_subscribed_cb, imu_unsubscribed_cb);
    }
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA0]) {
    depth_to_other_extrinsics_publishers_[INFRA0] =
        nh_.advertise<orbbec_camera::Extrinsics>("/" + camera_name_ + "/depth_to_ir", 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    depth_to_other_extrinsics_publishers_[COLOR] =
        nh_.advertise<orbbec_camera::Extrinsics>("/" + camera_name_ + "/depth_to_color", 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA1]) {
    depth_to_other_extrinsics_publishers_[INFRA1] =
        nh_.advertise<orbbec_camera::Extrinsics>("/" + camera_name_ + "/depth_to_left_ir", 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[INFRA2]) {
    depth_to_other_extrinsics_publishers_[INFRA2] = nh_.advertise<orbbec_camera::Extrinsics>(
        "/" + camera_name_ + "/depth_to_right_ir", 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[ACCEL]) {
    depth_to_other_extrinsics_publishers_[ACCEL] =
        nh_.advertise<orbbec_camera::Extrinsics>("/" + camera_name_ + "/depth_to_accel", 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[GYRO]) {
    depth_to_other_extrinsics_publishers_[GYRO] =
        nh_.advertise<orbbec_camera::Extrinsics>("/" + camera_name_ + "/depth_to_gyro", 1, true);
  }
  filter_status_pub_ =
      nh_.advertise<std_msgs::String>("/" + camera_name_ + "/filter_status", 1, true);
  std_msgs::String msg;
  msg.data = filter_status_.dump(2);
  filter_status_pub_.publish(msg);
  sdk_version_pub_ = nh_.advertise<std_msgs::String>("/" + camera_name_ + "/sdk_version", 1, true);
  auto device_info = device_->getDeviceInfo();
  nlohmann::json data;
  std_msgs::String sdk_msg;
  data["firmware_version"] = device_info->firmwareVersion();
  data["supported_min_sdk_version"] = device_info->supportedMinSdkVersion();
  data["ros_sdk_version"] = OB_ROS_VERSION_STR;
  std::string major = std::to_string(ob::Version::getMajor());
  std::string minor = std::to_string(ob::Version::getMinor());
  std::string patch = std::to_string(ob::Version::getPatch());
  std::string version = major + "." + minor + "." + patch;
  data["ob_sdk_version"] = version;
  sdk_msg.data = data.dump(2);
  sdk_version_pub_.publish(sdk_msg);
}

void OBCameraNode::setupCameraInfo() {
  color_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      nh_rgb_, camera_name_ + "_" + stream_name_[COLOR], color_info_uri_);
  ir_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      nh_ir_, camera_name_ + "_" + stream_name_[INFRA0], ir_info_uri_);
  auto param = getCameraParam();
  if (param) {
    camera_infos_[DEPTH] = convertToCameraInfo(param->depthIntrinsic, param->depthDistortion,
                                               param->depthIntrinsic.width);
    camera_infos_[INFRA0] = convertToCameraInfo(param->depthIntrinsic, param->depthDistortion,
                                                param->depthIntrinsic.width);
    camera_infos_[COLOR] =
        convertToCameraInfo(param->rgbIntrinsic, param->rgbDistortion, param->rgbIntrinsic.width);
  } else {
    // ROS_WARN_STREAM("Failed to get camera parameters");
  }
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  auto pid = device_info_->pid();
  if (!isGemini335PID(pid) && depth_registration_ && enable_stream_[COLOR] &&
      enable_stream_[DEPTH]) {
    OBAlignMode align_mode = align_mode_ == "HW" ? ALIGN_D2C_HW_MODE : ALIGN_D2C_SW_MODE;
    ROS_INFO_STREAM("set align mode to " << align_mode_);
    pipeline_config_->setAlignMode(align_mode);
    pipeline_config_->setDepthScaleRequire(enable_depth_scale_);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      ROS_INFO_STREAM("Enable " << stream_name_[stream_index] << " stream");
      auto profile = stream_profile_[stream_index]->as<ob::VideoStreamProfile>();

      if (stream_index == COLOR && enable_stream_[COLOR] && align_filter_) {
        auto video_profile = profile;
        ROS_INFO_STREAM("color video_profile: "
                        << video_profile->getWidth() << "x" << video_profile->getHeight() << " "
                        << video_profile->getFps() << "fps " << video_profile->getFormat());
        align_filter_->setAlignToStreamProfile(video_profile);
      }

      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " width: " << profile->getWidth()
                                << " height: " << profile->getHeight() << " fps: "
                                << profile->getFps() << " format: " << profile->getFormat());

      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
  }
  if (frame_aggregate_mode_ == "full_frame") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_FULL_FRAME_REQUIRE);
  } else if (frame_aggregate_mode_ == "color_frame") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_COLOR_FRAME_REQUIRE);
  } else if (frame_aggregate_mode_ == "disable") {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_DISABLE);
  } else {
    pipeline_config_->setFrameAggregateOutputMode(OB_FRAME_AGGREGATE_OUTPUT_ANY_SITUATION);
  }
}

void OBCameraNode::diagnosticTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  try {
    OBDeviceTemperature temperature;
    uint32_t data_size = sizeof(OBDeviceTemperature);
    device_->getStructuredData(OB_STRUCT_DEVICE_TEMPERATURE,
                               reinterpret_cast<uint8_t*>(&temperature), &data_size);
    stat.add("CPU Temperature", temperature.cpuTemp);
    stat.add("IR Temperature", temperature.irTemp);
    stat.add("LDM Temperature", temperature.ldmTemp);
    stat.add("MainBoard Temperature", temperature.mainBoardTemp);
    stat.add("TEC Temperature", temperature.tecTemp);
    stat.add("IMU Temperature", temperature.imuTemp);
    stat.add("RGB Temperature", temperature.rgbTemp);
    stat.add("Left IR Temperature", temperature.irLeftTemp);
    stat.add("Right IR Temperature", temperature.irRightTemp);
    stat.add("Chip Top Temperature", temperature.chipTopTemp);
    stat.add("Chip Bottom Temperature", temperature.chipBottomTemp);
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Temperature is normal");
  } catch (const ob::Error& e) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, e.getMessage());
  }
}
void OBCameraNode::setupDiagnosticUpdater() {
  if (!device_->isPropertySupported(OB_STRUCT_DEVICE_TEMPERATURE, OB_PERMISSION_READ)) {
    ROS_WARN_STREAM("Device does not support temperature reading");
    return;
  }
  std::string serial_number = device_info_->serialNumber();
  diagnostic_updater_ =
      std::make_shared<diagnostic_updater::Updater>(nh_, nh_private_, "ob_camera_" + serial_number);
  diagnostic_updater_->setHardwareID(serial_number);
  ros::WallRate rate(diagnostics_frequency_);
  diagnostic_updater_->add("Temperature", this, &OBCameraNode::diagnosticTemperature);
  while (is_running_ && ros::ok()) {
    diagnostic_updater_->force_update();
    rate.sleep();
  }
}

void OBCameraNode::readDefaultGain() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    try {
      auto sensor = sensors_[stream_index];
      CHECK_NOTNULL(sensor.get());
      auto gain = sensor->getGain();
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " gain " << gain);
      default_gain_[stream_index] = gain;
    } catch (ob::Error& e) {
      default_gain_[stream_index] = 0;
      ROS_DEBUG_STREAM("get gain error " << e.getMessage());
    }
  }
}

void OBCameraNode::readDefaultExposure() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    try {
      auto sensor = sensors_[stream_index];
      CHECK_NOTNULL(sensor.get());
      auto exposure = sensor->getExposure();
      bool is_auto_exposure = sensor->getAutoExposure();
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " exposure " << exposure
                                << " auto exposure " << is_auto_exposure);
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " exposure " << exposure);
      default_exposure_[stream_index] = exposure;
    } catch (ob::Error& e) {
      default_exposure_[stream_index] = 0;
      ROS_DEBUG_STREAM("get " << stream_name_[stream_index] << " exposure error "
                              << e.getMessage());
    }
  }
}

void OBCameraNode::readDefaultWhiteBalance() {
  try {
    auto sensor = sensors_[COLOR];
    if (!sensor) {
      ROS_INFO_STREAM("does not have color sensor");
      return;
    }
    CHECK_NOTNULL(sensor.get());
    auto wb = sensor->getWhiteBalance();
    ROS_INFO_STREAM("stream " << stream_name_[COLOR] << " wb " << wb);
    default_white_balance_ = wb;
  } catch (ob::Error& e) {
    default_white_balance_ = 0;
    ROS_DEBUG_STREAM("get white balance error " << e.getMessage());
  }
}

void OBCameraNode::setDisparitySearchOffset() {
  static bool has_run = false;
  if (has_run) {
    return;
  }
  if (device_->isPropertySupported(OB_PROP_DISP_SEARCH_OFFSET_INT, OB_PERMISSION_WRITE)) {
    if (disparity_search_offset_ >= 0 && disparity_search_offset_ <= 127) {
      device_->setIntProperty(OB_PROP_DISP_SEARCH_OFFSET_INT, disparity_search_offset_);
      ROS_INFO_STREAM("disparity_search_offset: " << disparity_search_offset_);
    }
    if (offset_index0_ >= 0 && offset_index0_ <= 127 && offset_index1_ >= 0 &&
        offset_index1_ <= 127) {
      auto config = OBDispOffsetConfig();
      config.enable = disparity_offset_config_;
      config.offset0 = offset_index0_;
      config.offset1 = offset_index1_;
      config.reserved = 0;
      device_->setStructuredData(OB_STRUCT_DISP_OFFSET_CONFIG,
                                 reinterpret_cast<const uint8_t*>(&config), sizeof(config));
      ROS_INFO_STREAM("disparity_offset_config: " << disparity_offset_config_
                                                  << "  offset_index0: " << offset_index0_
                                                  << "  offset_index1: " << offset_index1_);
    }
  }
  has_run = true;
}

void OBCameraNode::setDepthAutoExposureROI() {
  static bool depth_roi_has_run = false;
  if (depth_roi_has_run) {
    return;
  }
  if (device_->isPropertySupported(OB_STRUCT_DEPTH_AE_ROI, OB_PERMISSION_READ_WRITE)) {
    auto config = OBRegionOfInterest();
    uint32_t data_size = sizeof(config);
    device_->getStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                               &data_size);
    if (depth_ae_roi_left_ != -1) {
      config.x0_left = (depth_ae_roi_left_ < 0) ? 0 : depth_ae_roi_left_;
      config.x0_left =
          (depth_ae_roi_left_ > width_[DEPTH] - 1) ? width_[DEPTH] - 1 : config.x0_left;
    }
    if (depth_ae_roi_top_ != -1) {
      config.y0_top = (depth_ae_roi_top_ < 0) ? 0 : depth_ae_roi_top_;
      config.y0_top = (depth_ae_roi_top_ > height_[DEPTH] - 1) ? height_[DEPTH] - 1 : config.y0_top;
    }
    if (depth_ae_roi_right_ != -1) {
      config.x1_right = (depth_ae_roi_right_ < 0) ? 0 : depth_ae_roi_right_;
      config.x1_right =
          (depth_ae_roi_right_ > width_[DEPTH] - 1) ? width_[DEPTH] - 1 : config.x1_right;
    }
    if (depth_ae_roi_bottom_ != -1) {
      config.y1_bottom = (depth_ae_roi_bottom_ < 0) ? 0 : depth_ae_roi_bottom_;
      config.y1_bottom =
          (depth_ae_roi_bottom_ > height_[DEPTH] - 1) ? height_[DEPTH] - 1 : config.y1_bottom;
    }
    device_->setStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<const uint8_t*>(&config),
                               sizeof(config));
    device_->getStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                               &data_size);
    ROS_INFO_STREAM("Setting depth AE ROI to " << config.x0_left << ", " << config.y0_top << ", "
                                               << config.x1_right << ", " << config.y1_bottom);
  }
  depth_roi_has_run = true;
}

void OBCameraNode::setColorAutoExposureROI() {
  static bool color_roi_has_run = false;
  if (color_roi_has_run) {
    return;
  }
  if (device_->isPropertySupported(OB_STRUCT_COLOR_AE_ROI, OB_PERMISSION_READ_WRITE)) {
    auto config = OBRegionOfInterest();
    uint32_t data_size = sizeof(config);
    device_->getStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                               &data_size);
    if (color_ae_roi_left_ != -1) {
      config.x0_left = (color_ae_roi_left_ < 0) ? 0 : color_ae_roi_left_;
      config.x0_left =
          (color_ae_roi_left_ > width_[COLOR] - 1) ? width_[COLOR] - 1 : config.x0_left;
    }
    if (color_ae_roi_top_ != -1) {
      config.y0_top = (color_ae_roi_top_ < 0) ? 0 : color_ae_roi_top_;
      config.y0_top = (color_ae_roi_top_ > height_[COLOR] - 1) ? height_[COLOR] - 1 : config.y0_top;
    }
    if (color_ae_roi_right_ != -1) {
      config.x1_right = (color_ae_roi_right_ < 0) ? 0 : color_ae_roi_right_;
      config.x1_right =
          (color_ae_roi_right_ > width_[COLOR] - 1) ? width_[COLOR] - 1 : config.x1_right;
    }
    if (color_ae_roi_bottom_ != -1) {
      config.y1_bottom = (color_ae_roi_bottom_ < 0) ? 0 : color_ae_roi_bottom_;
      config.y1_bottom =
          (color_ae_roi_bottom_ > height_[COLOR] - 1) ? height_[COLOR] - 1 : config.y1_bottom;
    }
    device_->setStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<const uint8_t*>(&config),
                               sizeof(config));
    device_->getStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                               &data_size);
    ROS_INFO_STREAM("Setting color AE ROI to " << config.x0_left << ", " << config.y0_top << ", "
                                               << config.x1_right << ", " << config.y1_bottom);
  }
  color_roi_has_run = true;
}

bool OBCameraNode::setFilterCallback(SetFilterRequest& request, SetFilterResponse& response) {
  try {
    ROS_INFO_STREAM("filter_name: " << request.filter_name << "  filter_enable: "
                                    << (request.filter_enable ? "true" : "false"));
    auto it = std::remove_if(depth_filter_list_.begin(), depth_filter_list_.end(),
                             [&request](const std::shared_ptr<ob::Filter>& filter) {
                               return filter->getName() == request.filter_name;
                             });
    depth_filter_list_.erase(it, depth_filter_list_.end());
    if (request.filter_name == "DecimationFilter") {
      auto decimation_filter = std::make_shared<ob::DecimationFilter>();
      decimation_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(decimation_filter);
      if (request.filter_param.size() > 0) {
        auto range = decimation_filter->getScaleRange();
        auto decimation_filter_scale = request.filter_param[0];
        if (decimation_filter_scale <= range.max && decimation_filter_scale >= range.min) {
          ROS_INFO_STREAM("Set decimation filter scale value to " << decimation_filter_scale);
          decimation_filter->setScaleValue(decimation_filter_scale);
        }
        if (decimation_filter_scale != -1 &&
            (decimation_filter_scale < range.min || decimation_filter_scale > range.max)) {
          ROS_ERROR_STREAM("Decimation filter scale value is out of range " << range.min << " - "
                                                                            << range.max);
        }
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting fails";
        return true;
      }
    } else if (request.filter_name == "HDRMerge") {
      auto hdr_merge_filter = std::make_shared<ob::HdrMerge>();
      hdr_merge_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(hdr_merge_filter);
      if (request.filter_param.size() > 3) {
        auto config = OBHdrConfig();
        config.enable = true;
        config.exposure_1 = request.filter_param[0];
        config.gain_1 = request.filter_param[1];
        config.exposure_2 = request.filter_param[2];
        config.gain_2 = request.filter_param[3];
        device_->setStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                   reinterpret_cast<const uint8_t*>(&config), sizeof(config));
        ROS_INFO_STREAM("Set HDR merge filter params: "
                        << "\nexposure_1: " << request.filter_param[0] << "\ngain_1: "
                        << request.filter_param[1] << "\nexposure_2: " << request.filter_param[2]
                        << "\ngain_2: " << request.filter_param[3]);
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting fails";
        return true;
      }
    } else if (request.filter_name == "SequenceIdFilter") {
      auto sequenced_filter = std::make_shared<ob::SequenceIdFilter>();
      sequenced_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(sequenced_filter);
      if (request.filter_param.size() > 0) {
        sequenced_filter->selectSequenceId(request.filter_param[0]);
        ROS_INFO_STREAM("Set sequenced filter selectSequenceId value to "
                        << request.filter_param[0]);
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting fails";
        return true;
      }
    } else if (request.filter_name == "ThresholdFilter") {
      auto threshold_filter = std::make_shared<ob::ThresholdFilter>();
      threshold_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(threshold_filter);
      if (request.filter_param.size() > 1) {
        auto threshold_filter_min = request.filter_param[0];
        auto threshold_filter_max = request.filter_param[1];
        threshold_filter->setValueRange(threshold_filter_min, threshold_filter_max);
        ROS_INFO_STREAM("Set threshold filter value range to " << threshold_filter_min << " - "
                                                               << threshold_filter_max);
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting fails";
        return true;
      }
    } else if (request.filter_name == "NoiseRemovalFilter") {
      if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
        device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, request.filter_enable);
        ROS_INFO_STREAM("enable_noise_removal_filter:" << request.filter_enable);
      }
      if (request.filter_param.size() > 1) {
        if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
          auto default_noise_removal_filter_min_diff =
              device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
          ROS_INFO_STREAM(
              "default_noise_removal_filter_min_diff: " << default_noise_removal_filter_min_diff);
          device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, request.filter_param[0]);
          auto new_noise_removal_filter_min_diff =
              device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
          ROS_INFO_STREAM(
              "after set noise_removal_filter_min_diff: " << new_noise_removal_filter_min_diff);
        }
        if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
          auto default_noise_removal_filter_max_size =
              device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
          ROS_INFO_STREAM(
              "default_noise_removal_filter_max_size: " << default_noise_removal_filter_max_size);
          device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, request.filter_param[1]);
          auto new_noise_removal_filter_max_size =
              device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
          ROS_INFO_STREAM(
              "after set noise_removal_filter_max_size: " << new_noise_removal_filter_max_size);
        }
      }
    } else if (request.filter_name == "HardwareNoiseRemoval") {
      if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                       OB_PERMISSION_READ_WRITE)) {
        device_->setBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL, request.filter_enable);
        ROS_INFO_STREAM("Setting hardware_noise_removal_filter:" << request.filter_enable);
        if (request.filter_param.size() > 0 &&
            device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                         OB_PERMISSION_READ_WRITE)) {
          if (request.filter_enable) {
            device_->setFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                      request.filter_param[0]);
            ROS_INFO_STREAM(
                "Setting hardware_noise_removal_filter_threshold :" << request.filter_param[0]);
          }
        } else {
          response.message =
              "The filter switch setting is successful, but the filter parameter setting fails";
          return true;
        }
      }
    } else if (request.filter_name == "SpatialAdvancedFilter") {
      auto spatial_filter = std::make_shared<ob::SpatialAdvancedFilter>();
      spatial_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(spatial_filter);
      if (request.filter_param.size() > 3) {
        OBSpatialAdvancedFilterParams params{};
        params.alpha = request.filter_param[0];
        params.disp_diff = request.filter_param[1];
        params.magnitude = request.filter_param[2];
        params.radius = request.filter_param[3];
        spatial_filter->setFilterParams(params);
        ROS_INFO_STREAM("Set spatial filter params: " << "\nalpha:" << params.alpha
                                                      << "\nradius:" << params.radius
                                                      << "\ndisp_diff:" << params.disp_diff);
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting fails";
        return true;
      }
    } else if (request.filter_name == "TemporalFilter") {
      auto temporal_filter = std::make_shared<ob::TemporalFilter>();
      temporal_filter->enable(request.filter_enable);
      depth_filter_list_.push_back(temporal_filter);
      if (request.filter_param.size() > 1) {
        temporal_filter->setDiffScale(request.filter_param[0]);
        temporal_filter->setWeight(request.filter_param[1]);
        ROS_INFO_STREAM("Set temporal filter value to " << request.filter_param[0] << " - "
                                                        << request.filter_param[1]);
      } else {
        response.message =
            "The filter switch setting is successful, but the filter parameter setting "
            "fails";
        return true;
      }

    } else {
      ROS_INFO_STREAM(request.filter_name
                      << "Cannot be set\n"
                      << "The filter_name value that can be set is "
                         "DecimationFilter, HDRMerge, SequenceIdFilter, ThresholdFilter, Nois"
                         "eRemovalFilter, SpatialAdvancedFilter and TemporalFilter");
    }
    for (auto& filter : depth_filter_list_) {
      std::cout << " - " << filter->getName() << ": "
                << (filter->isEnabled() ? "enabled" : "disabled") << std::endl;
      auto configSchemaVec = filter->getConfigSchemaVec();
      for (auto& configSchema : configSchemaVec) {
        std::cout << "    - {" << configSchema.name << ", " << configSchema.type << ", "
                  << configSchema.min << ", " << configSchema.max << ", " << configSchema.step
                  << ", " << configSchema.def << ", " << configSchema.desc << "}" << std::endl;
      }
    }
    return response.success = true;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set filter: " << e.getMessage());
    return response.success = false;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to set filter: " << e.what());
    return response.success = false;
  } catch (...) {
    ROS_ERROR_STREAM("unknown error");
    return response.success = false;
  }
}
}  // namespace orbbec_camera
