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
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <boost/filesystem.hpp>

namespace orbbec_camera {

namespace {

constexpr char kEnhancedDepthSupportedTargetResolutions[] = "640x480/1280x720/1280x800";
constexpr char kEnhancedDepthSupportedDepthFormats[] = "Y10/Y11/Y12/Y14/Y16/Z16";

std::string getDepthFilterStatusName(const std::string& filter_name) {
  if (filter_name == "SpatialAdvancedFilter") {
    return "SpatialFilter";
  }
  if (filter_name == "DisparityTransform") {
    return "DisparityToDepth";
  }
  return filter_name;
}

const std::unordered_map<OBFormat, OBConvertFormat>& enhancedDepthColorFormatMap() {
  static const std::unordered_map<OBFormat, OBConvertFormat> kFormatMap = {
      {OB_FORMAT_YUYV, FORMAT_YUYV_TO_RGB}, {OB_FORMAT_UYVY, FORMAT_UYVY_TO_RGB},
      {OB_FORMAT_MJPG, FORMAT_MJPG_TO_RGB}, {OB_FORMAT_BGR, FORMAT_BGR_TO_RGB},
      {OB_FORMAT_RGBA, FORMAT_RGBA_TO_RGB}, {OB_FORMAT_Y16, FORMAT_Y16_TO_RGB},
      {OB_FORMAT_Y8, FORMAT_Y8_TO_RGB},
  };
  return kFormatMap;
}

bool isEnhancedDepthColorFormatSupported(OBFormat format) {
  return format == OB_FORMAT_RGB || enhancedDepthColorFormatMap().count(format) > 0;
}

boost::filesystem::path resolveConfigJsonFilePath(const std::string& file_path) {
  boost::filesystem::path path(file_path);
  const char* home_dir = std::getenv("HOME");
  if ((file_path == "~" || file_path.rfind("~/", 0) == 0) && home_dir != nullptr) {
    path = boost::filesystem::path(home_dir);
    if (file_path.size() > 2) {
      path /= file_path.substr(2);
    }
  }
  if (path.is_relative()) {
    path = boost::filesystem::absolute(path);
  }
  return path.lexically_normal();
}

bool configJsonContainsApplicationConfig(const std::string& file_path) {
  if (file_path.empty()) {
    return false;
  }

  std::ifstream config_file(resolveConfigJsonFilePath(file_path).string());
  if (!config_file.good()) {
    return false;
  }

  try {
    nlohmann::json config_json;
    config_file >> config_json;
    return config_json.contains("application_config");
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Config JSON application_config check failed file="
                    << resolveConfigJsonFilePath(file_path).string() << " error=\"" << e.what()
                    << "\"");
  }
  return false;
}

std::string getDepthFilterStatusParamName(const std::string& filter_name,
                                          const std::string& param_name) {
  if (filter_name == "SpatialAdvancedFilter" && param_name == "disp_diff") {
    return "diff_threshold";
  }
  if (filter_name == "SpatialModerateFilter" && param_name == "disp_diff") {
    return "diff_threshold";
  }
  if (filter_name == "TemporalFilter" && param_name == "diff_scale") {
    return "diff_threshold";
  }
  if (filter_name == "DecimationFilter" && param_name == "decimate") {
    return "scale";
  }
  return param_name;
}

std::string getDepthFilterConfigParamName(const std::string& filter_name,
                                          const std::string& param_name) {
  if ((filter_name == "SpatialAdvancedFilter" || filter_name == "SpatialModerateFilter") &&
      param_name == "diff_threshold") {
    return "disp_diff";
  }
  if (filter_name == "TemporalFilter" && param_name == "diff_threshold") {
    return "diff_scale";
  }
  if (filter_name == "DecimationFilter" && param_name == "scale") {
    return "decimate";
  }
  if (filter_name == "SequenceIdFilter" && (param_name == "id" || param_name == "sequence_id" ||
                                            param_name == "sequence_id_filter_id")) {
    return "sequenceid";
  }
  if (filter_name == "HoleFillingFilter" && param_name == "mode") {
    return "hole_filling_mode";
  }
  return param_name;
}

bool shouldExposeDepthFilterParams(const std::string& filter_name) {
  return filter_name != "MgcNoiseRemovalFilter" && filter_name != "LutNoiseRemovalFilter" &&
         filter_name != "DisparityTransform" && filter_name != "EdgeNoiseRemovalFilter";
}

std::string formatFilterConfigValue(const OBFilterConfigSchemaItem& config_schema, double value) {
  switch (config_schema.type) {
    case OB_FILTER_CONFIG_VALUE_TYPE_INT: {
      return std::to_string(static_cast<long long>(value));
    }
    case OB_FILTER_CONFIG_VALUE_TYPE_BOOLEAN:
      return value != 0.0 ? std::string("true") : std::string("false");
    case OB_FILTER_CONFIG_VALUE_TYPE_FLOAT:
    default: {
      std::ostringstream ss;
      ss << value;
      return ss.str();
    }
  }
}

std::string trimFilterConfigValue(const std::string& value) {
  auto begin = value.begin();
  while (begin != value.end() && std::isspace(static_cast<unsigned char>(*begin))) {
    ++begin;
  }
  auto end = value.end();
  while (end != begin && std::isspace(static_cast<unsigned char>(*(end - 1)))) {
    --end;
  }
  return std::string(begin, end);
}

std::string lowerFilterConfigValue(std::string value) {
  std::transform(value.begin(), value.end(), value.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return value;
}

bool parseFilterConfigDouble(const std::string& raw_value, double& parsed_value,
                             std::string& message) {
  const auto value = trimFilterConfigValue(raw_value);
  if (value.empty()) {
    message = "Filter config value is empty";
    return false;
  }

  try {
    size_t parsed_chars = 0;
    parsed_value = std::stod(value, &parsed_chars);
    if (parsed_chars != value.size()) {
      message = "Filter config value '" + raw_value + "' is not a valid number";
      return false;
    }
  } catch (const std::exception&) {
    message = "Filter config value '" + raw_value + "' is not a valid number";
    return false;
  }
  return true;
}

std::string dispOutliersSearchModeToString(int search_mode) {
  switch (search_mode) {
    case 0:
      return "FULL";
    case 1:
      return "OFFSET_80";
    default:
      return "";
  }
}

bool parseDispOutliersSearchMode(const std::string& raw_value, int& search_mode,
                                 std::string& message) {
  const auto value = trimFilterConfigValue(raw_value);
  const auto lower_value = lowerFilterConfigValue(value);
  if (lower_value == "full") {
    search_mode = 0;
    return true;
  }
  if (lower_value == "offset_80") {
    search_mode = 1;
    return true;
  }

  message = "Filter config 'search_mode' expects one of FULL, OFFSET_80";
  return false;
}

bool parseFilterConfigValue(const OBFilterConfigSchemaItem& schema, const std::string& raw_value,
                            double& parsed_value, std::string& message) {
  const auto value = trimFilterConfigValue(raw_value);
  if (schema.type == OB_FILTER_CONFIG_VALUE_TYPE_BOOLEAN) {
    const auto lower_value = lowerFilterConfigValue(value);
    if (lower_value == "true" || lower_value == "1") {
      parsed_value = 1.0;
      return true;
    }
    if (lower_value == "false" || lower_value == "0") {
      parsed_value = 0.0;
      return true;
    }
    message = "Filter config '" + std::string(schema.name) + "' expects a boolean value";
    return false;
  }

  if (!parseFilterConfigDouble(value, parsed_value, message)) {
    return false;
  }

  if (schema.type == OB_FILTER_CONFIG_VALUE_TYPE_INT && std::floor(parsed_value) != parsed_value) {
    message = "Filter config '" + std::string(schema.name) + "' expects an integer value";
    return false;
  }

  if (parsed_value < schema.min || parsed_value > schema.max) {
    std::ostringstream ss;
    ss << "Filter config '" << schema.name << "' value " << parsed_value << " is out of range ["
       << schema.min << ", " << schema.max << "]";
    message = ss.str();
    return false;
  }

  return true;
}

bool equalsIgnoreCase(const std::string& lhs, const char* rhs) {
  if (rhs == nullptr) {
    return false;
  }

  const std::string rhs_value(rhs);
  if (lhs.size() != rhs_value.size()) {
    return false;
  }

  return std::equal(lhs.begin(), lhs.end(), rhs_value.begin(), [](char a, char b) {
    return std::tolower(static_cast<unsigned char>(a)) ==
           std::tolower(static_cast<unsigned char>(b));
  });
}

}  // namespace

std::string OBCameraNode::normalizeDepthFilterName(const std::string& filter_name) {
  if (filter_name == "HardwareNoiseRemoval") {
    return "HardwareNoiseRemovalFilter";
  }
  if (filter_name == "SpatialFilter") {
    return "SpatialAdvancedFilter";
  }
  if (filter_name == "DepthOutliersFilter" || filter_name == "DisparityOutliersFilter") {
    return "DispOutliersFilter";
  }
  if (filter_name == "EnhancedDepth" || filter_name == "EnhancedDepthFilter") {
    return "EnhancedDepthFilter";
  }
  return filter_name;
}

void OBCameraNode::appendDepthFilterParam(orbbec_camera::DepthFilterState& filter_state,
                                          const std::string& name, const std::string& value) {
  orbbec_camera::DepthFilterParam param;
  param.name = name;
  param.value = value;
  filter_state.params.push_back(param);
}

bool OBCameraNode::isConfigJsonLoaded() const { return config_json_loaded_; }

void OBCameraNode::captureInitialRosParameters() {
  std::vector<std::string> param_names;
  nh_private_.getParamNames(param_names);
  initial_ros_params_.clear();
  initial_ros_params_.insert(param_names.begin(), param_names.end());
}

bool OBCameraNode::isLaunchParamProvided(const std::string& param_name) const {
  if (param_name.empty()) {
    return false;
  }
  return initial_ros_params_.find(param_name) != initial_ros_params_.end() ||
         initial_ros_params_.find(nh_private_.resolveName(param_name)) != initial_ros_params_.end();
}

void OBCameraNode::loadConfigJson() {
  if (load_config_json_file_path_.empty()) {
    return;
  }

  const auto resolved_file_path = resolveConfigJsonFilePath(load_config_json_file_path_);
  const auto resolved_file_path_str = resolved_file_path.string();
  std::ifstream load_config_file(resolved_file_path_str);
  if (!load_config_file.good()) {
    ROS_WARN_STREAM("Config JSON load skip file=" << resolved_file_path_str
                                                  << " reason=file_not_found");
    return;
  }

  try {
    device_->loadPresetFromJsonFile(resolved_file_path_str.c_str());
    config_json_loaded_ = true;
    ROS_INFO_STREAM("Config JSON loaded file=" << resolved_file_path_str);
  } catch (const ob::Error& e) {
    config_json_loaded_ = false;
    ROS_ERROR_STREAM("Config JSON load failed file=" << resolved_file_path_str << " error=\""
                                                     << orbbec_camera::formatObErrorWithStatus(e)
                                                     << "\"");
  } catch (const std::exception& e) {
    config_json_loaded_ = false;
    ROS_ERROR_STREAM("Config JSON load failed file=" << resolved_file_path_str << " error=\""
                                                     << e.what() << "\"");
  } catch (...) {
    config_json_loaded_ = false;
    ROS_ERROR_STREAM("Config JSON load failed file=" << resolved_file_path_str);
  }
}

void OBCameraNode::syncConfigJsonApplicationConfig() {
  if (!config_json_loaded_ || !configJsonContainsApplicationConfig(load_config_json_file_path_)) {
    return;
  }

  try {
    if (!ob::ApplicationConfig::isSupported(device_)) {
      ROS_WARN_STREAM(
          "Config JSON application_config is ignored because this device does not "
          "support SDK application config");
      return;
    }

    auto application_config = ob::ApplicationConfig::get(device_);
    CHECK_NOTNULL(application_config.get());

    for (const auto& sensor_config : application_config->sensors()) {
      if (!sensor_config || !sensor_config->streamProfile()) {
        continue;
      }

      auto profile = sensor_config->streamProfile();
      const stream_index_pair stream_index{profile->getType(), 0};
      auto stream_name_it = stream_name_.find(stream_index);
      if (stream_name_it == stream_name_.end()) {
        ROS_DEBUG_STREAM(
            "Config JSON application_config skips unsupported stream type=" << profile->getType());
        continue;
      }

      const auto& stream_name = stream_name_it->second;
      if (!isLaunchParamProvided("enable_" + stream_name)) {
        enable_stream_[stream_index] = sensor_config->isStreamEnabled();
      }

      if (std::find(IMAGE_STREAMS.begin(), IMAGE_STREAMS.end(), stream_index) !=
          IMAGE_STREAMS.end()) {
        auto video_profile = profile->as<ob::VideoStreamProfile>();
        if (!isLaunchParamProvided(stream_name + "_width")) {
          width_[stream_index] = static_cast<int>(video_profile->width());
        }
        if (!isLaunchParamProvided(stream_name + "_height")) {
          height_[stream_index] = static_cast<int>(video_profile->height());
        }
        if (!isLaunchParamProvided(stream_name + "_fps")) {
          fps_[stream_index] = static_cast<int>(video_profile->fps());
        }
        if (!isLaunchParamProvided(stream_name + "_format")) {
          format_[stream_index] = video_profile->format();
          format_str_[stream_index] = OBFormatToString(format_[stream_index]);
        }
        if (!isLaunchParamProvided("enable_" + stream_name + "_undistortion")) {
          enable_undistortion_[stream_index] = sensor_config->isUndistortionEnabled();
        }
      } else if (stream_index == ACCEL) {
        auto accel_profile = profile->as<ob::AccelStreamProfile>();
        if (!isLaunchParamProvided("accel_rate")) {
          imu_rate_[ACCEL] = sampleRateToString(accel_profile->sampleRate());
        }
        if (!isLaunchParamProvided("accel_range")) {
          imu_range_[ACCEL] = fullAccelScaleRangeToString(accel_profile->fullScaleRange());
        }
      } else if (stream_index == GYRO) {
        auto gyro_profile = profile->as<ob::GyroStreamProfile>();
        if (!isLaunchParamProvided("gyro_rate")) {
          imu_rate_[GYRO] = sampleRateToString(gyro_profile->sampleRate());
        }
        if (!isLaunchParamProvided("gyro_range")) {
          imu_range_[GYRO] = fullGyroScaleRangeToString(gyro_profile->fullScaleRange());
        }
      }
    }

    auto point_cloud_config = application_config->pointCloud();
    if (point_cloud_config) {
      const auto point_cloud_enabled = point_cloud_config->isEnabled();
      const auto point_cloud_format = point_cloud_config->format();
      if (!isLaunchParamProvided("enable_point_cloud")) {
        enable_point_cloud_ = point_cloud_enabled && point_cloud_format == OB_FORMAT_POINT;
      }
      if (!isLaunchParamProvided("enable_colored_point_cloud")) {
        enable_colored_point_cloud_ =
            point_cloud_enabled && point_cloud_format == OB_FORMAT_RGB_POINT;
      }
      if (!isLaunchParamProvided("point_cloud_decimation_filter_factor")) {
        point_cloud_decimation_filter_factor_ = point_cloud_config->decimationFactor();
      }
      if (!isLaunchParamProvided("enable_frame_sync")) {
        enable_frame_sync_ = point_cloud_config->isFrameSyncEnabled();
      }
      if (!isLaunchParamProvided("frame_aggregate_mode")) {
        frame_aggregate_mode_ = point_cloud_config->isAllFrameTypeRequired() ? "full_frame" : "ANY";
      }

      const auto align_mode = point_cloud_config->alignMode();
      if (!isLaunchParamProvided("depth_registration")) {
        depth_registration_ = align_mode != ALIGN_DISABLE;
      }
      if (!isLaunchParamProvided("align_mode")) {
        if (align_mode == ALIGN_D2C_HW_MODE) {
          align_mode_ = "HW";
        } else if (align_mode == ALIGN_D2C_SW_MODE || align_mode == ALIGN_C2D_SW_MODE) {
          align_mode_ = "SW";
        }
      }
      if (!isLaunchParamProvided("align_target_stream")) {
        if (align_mode == ALIGN_D2C_HW_MODE || align_mode == ALIGN_D2C_SW_MODE) {
          align_target_stream_ = OB_STREAM_COLOR;
        } else if (align_mode == ALIGN_C2D_SW_MODE) {
          align_target_stream_ = OB_STREAM_DEPTH;
        }
      }
    }

    auto hdr_merge_config = application_config->hdrMerge();
    if (hdr_merge_config && !isLaunchParamProvided("enable_hdr_merge")) {
      enable_hdr_merge_ = hdr_merge_config->isEnabled();
    }

    auto device_decimation_config = application_config->deviceDecimation();
    if (device_decimation_config && device_decimation_config->isEnabled() &&
        !isLaunchParamProvided("preset_resolution_config")) {
      const auto& config = device_decimation_config->presetResolutionConfig();
      std::ostringstream ss;
      ss << config.width << "," << config.height << "," << config.irDecimationFactor << ","
         << config.depthDecimationFactor;
      preset_resolution_config_ = ss.str();
    }

    ROS_INFO_STREAM("Config JSON application_config synced");
  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Config JSON application_config sync failed error=\""
                    << orbbec_camera::formatObErrorWithStatus(e) << "\"");
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Config JSON application_config sync failed error=\"" << e.what() << "\"");
  } catch (...) {
    ROS_WARN_STREAM("Config JSON application_config sync failed");
  }
}

void OBCameraNode::syncApplicationSensorConfigForExport() {
  try {
    if (!ob::ApplicationConfig::isSupported(device_)) {
      return;
    }

    auto application_config = ob::ApplicationConfig::get(device_);
    CHECK_NOTNULL(application_config.get());

    for (const auto& sensor_config : application_config->sensors()) {
      if (!sensor_config || !sensor_config->streamProfile()) {
        continue;
      }

      const auto current_profile = sensor_config->streamProfile();
      const stream_index_pair stream_index{current_profile->getType(), 0};
      const auto stream_name_it = stream_name_.find(stream_index);
      if (stream_name_it == stream_name_.end()) {
        ROS_DEBUG_STREAM("Config JSON sensors export skips unsupported stream type="
                         << current_profile->getType());
        continue;
      }

      auto export_sensor_config =
          std::make_shared<ob::ApplicationSensorConfig>(sensor_config->sensorType());
      const auto enable_stream_it = enable_stream_.find(stream_index);
      export_sensor_config->enableStream(enable_stream_it != enable_stream_.end()
                                             ? enable_stream_it->second
                                             : sensor_config->isStreamEnabled());

      const auto stream_profile_it = stream_profile_.find(stream_index);
      export_sensor_config->setStreamProfile(stream_profile_it != stream_profile_.end() &&
                                                     stream_profile_it->second
                                                 ? stream_profile_it->second
                                                 : current_profile);

      const auto enable_undistortion_it = enable_undistortion_.find(stream_index);
      export_sensor_config->enableUndistortion(enable_undistortion_it != enable_undistortion_.end()
                                                   ? enable_undistortion_it->second
                                                   : sensor_config->isUndistortionEnabled());

      application_config->setSensor(export_sensor_config);
    }
  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Config JSON sensors export sync failed error=\""
                    << orbbec_camera::formatObErrorWithStatus(e) << "\"");
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Config JSON sensors export sync failed error=\"" << e.what() << "\"");
  } catch (...) {
    ROS_WARN_STREAM("Config JSON sensors export sync failed");
  }
}

void OBCameraNode::syncApplicationPointCloudConfigForExport() {
  try {
    if (!ob::ApplicationConfig::isSupported(device_)) {
      return;
    }

    auto application_config = ob::ApplicationConfig::get(device_);
    CHECK_NOTNULL(application_config.get());

    auto point_cloud_config = std::make_shared<ob::ApplicationPointCloudConfig>();
    const auto point_cloud_enabled = enable_point_cloud_ || enable_colored_point_cloud_;
    point_cloud_config->enable(point_cloud_enabled);
    point_cloud_config->setFormat(enable_colored_point_cloud_ ? OB_FORMAT_RGB_POINT
                                                              : OB_FORMAT_POINT);
    point_cloud_config->setDecimationFactor(std::max(1, point_cloud_decimation_filter_factor_));

    auto align_mode = ALIGN_DISABLE;
    if (depth_registration_) {
      if (align_mode_ == "HW") {
        align_mode = ALIGN_D2C_HW_MODE;
      } else if (align_target_stream_ == OB_STREAM_DEPTH) {
        align_mode = ALIGN_C2D_SW_MODE;
      } else {
        align_mode = ALIGN_D2C_SW_MODE;
      }
    }
    point_cloud_config->setAlignMode(align_mode);
    point_cloud_config->enableFrameSync(enable_frame_sync_);
    point_cloud_config->setAllFrameTypeRequired(frame_aggregate_mode_ == "full_frame");
    point_cloud_config->enableMatchTargetResolution(
        align_mode == ALIGN_D2C_HW_MODE ? enable_depth_scale_ : true);
    application_config->setPointCloud(point_cloud_config);
  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Config JSON point_cloud export sync failed error=\""
                    << orbbec_camera::formatObErrorWithStatus(e) << "\"");
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Config JSON point_cloud export sync failed error=\"" << e.what() << "\"");
  } catch (...) {
    ROS_WARN_STREAM("Config JSON point_cloud export sync failed");
  }
}

void OBCameraNode::syncApplicationHdrMergeConfigForExport() {
  try {
    if (!ob::ApplicationConfig::isSupported(device_)) {
      return;
    }

    auto application_config = ob::ApplicationConfig::get(device_);
    CHECK_NOTNULL(application_config.get());

    auto hdr_merge_config = std::make_shared<ob::ApplicationHDRMergeConfig>();
    hdr_merge_config->enable(enable_hdr_merge_);
    hdr_merge_config->enableIR(true);
    application_config->setHDRMerge(hdr_merge_config);
  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Config JSON hdr_merge export sync failed error=\""
                    << orbbec_camera::formatObErrorWithStatus(e) << "\"");
  } catch (const std::exception& e) {
    ROS_WARN_STREAM("Config JSON hdr_merge export sync failed error=\"" << e.what() << "\"");
  } catch (...) {
    ROS_WARN_STREAM("Config JSON hdr_merge export sync failed");
  }
}

bool OBCameraNode::exportConfigJsonToFile(const std::string& file_path, std::string& message) {
  if (file_path.empty()) {
    message = "Config json export file path is empty";
    ROS_ERROR_STREAM(message);
    return false;
  }

  try {
    const auto resolved_file_path = resolveConfigJsonFilePath(file_path);
    const auto parent_path = resolved_file_path.parent_path();
    if (!parent_path.empty()) {
      boost::filesystem::create_directories(parent_path);
    }

    const auto resolved_file_path_str = resolved_file_path.string();
    syncApplicationSensorConfigForExport();
    syncApplicationPointCloudConfigForExport();
    syncApplicationHdrMergeConfigForExport();
    device_->exportSettingsAsPresetJsonFile(resolved_file_path_str.c_str());
    if (!boost::filesystem::exists(resolved_file_path)) {
      message = "Failed to export config json file: file not found after export path=" +
                resolved_file_path_str;
      ROS_ERROR_STREAM(message);
      return false;
    }

    message = "Exported config json file path: " + resolved_file_path_str;
    ROS_INFO_STREAM(message);
    return true;
  } catch (const ob::Error& e) {
    message = "Failed to export config json file: " + orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM(message);
  } catch (const std::exception& e) {
    message = std::string("Failed to export config json file: ") + e.what();
    ROS_ERROR_STREAM(message);
  } catch (...) {
    message = "Failed to export config json file";
    ROS_ERROR_STREAM(message);
  }
  return false;
}

void OBCameraNode::exportConfigJsonIfRequested() {
  if (export_config_json_file_path_.empty()) {
    return;
  }
  std::string message;
  exportConfigJsonToFile(export_config_json_file_path_, message);
}

void OBCameraNode::syncConfigJsonDeviceSettings() {
  if (!config_json_loaded_) {
    return;
  }

  auto can_read = [this](OBPropertyID property_id) {
    return device_->isPropertySupported(property_id, OB_PERMISSION_READ) ||
           device_->isPropertySupported(property_id, OB_PERMISSION_READ_WRITE);
  };
  auto can_write = [this](OBPropertyID property_id) {
    return device_->isPropertySupported(property_id, OB_PERMISSION_WRITE) ||
           device_->isPropertySupported(property_id, OB_PERMISSION_READ_WRITE);
  };
  auto can_read_struct = can_read;
  auto log_readback = [](const std::string& scope, const std::string& name, const auto& value) {
    std::ostringstream ss;
    ss << std::boolalpha << value;
    ROS_INFO_STREAM("Config final readback [" << scope << "] " << name << "=" << ss.str());
  };
  auto log_readback_fields = [](const std::string& scope, const std::string& fields) {
    ROS_INFO_STREAM("Config final readback [" << scope << "] " << fields);
  };
  auto sync_bool = [&](const char* scope, const char* param_name, bool& member,
                       OBPropertyID property_id) {
    if (!isConfigJsonLoaded() || !can_read(property_id)) {
      return;
    }
    try {
      member = device_->getBoolProperty(property_id);
      log_readback(scope, param_name, member);
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("Config final readback failed [" << scope << "] " << param_name
                                                        << " error=\"" << e.what() << "\"");
    }
  };
  auto sync_int = [&](const char* scope, const char* param_name, int& member,
                      OBPropertyID property_id) {
    if (!isConfigJsonLoaded() || !can_read(property_id)) {
      return;
    }
    try {
      member = device_->getIntProperty(property_id);
      log_readback(scope, param_name, member);
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("Config final readback failed [" << scope << "] " << param_name
                                                        << " error=\"" << e.what() << "\"");
    }
  };
  auto sync_float = [&](const char* scope, const char* param_name, float& member,
                        OBPropertyID property_id) {
    if (!isConfigJsonLoaded() || !can_read(property_id)) {
      return;
    }
    try {
      member = device_->getFloatProperty(property_id);
      log_readback(scope, param_name, member);
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("Config final readback failed [" << scope << "] " << param_name
                                                        << " error=\"" << e.what() << "\"");
    }
  };
  auto sync_stream_orientation = [&](const char* param_prefix,
                                     const stream_index_pair& stream_index,
                                     OBPropertyID flip_property_id, OBPropertyID mirror_property_id,
                                     OBPropertyID rotation_property_id) {
    if (!isConfigJsonLoaded()) {
      return;
    }
    if (can_read(flip_property_id)) {
      try {
        image_flip_[stream_index] = device_->getBoolProperty(flip_property_id);
        log_readback(std::string(param_prefix) + ".orientation", "flip", image_flip_[stream_index]);
      } catch (const std::exception&) {
      }
    }
    if (can_read(mirror_property_id)) {
      try {
        image_mirror_[stream_index] = device_->getBoolProperty(mirror_property_id);
        log_readback(std::string(param_prefix) + ".orientation", "mirror",
                     image_mirror_[stream_index]);
      } catch (const std::exception&) {
      }
    }
    if (can_read(rotation_property_id)) {
      try {
        image_rotation_[stream_index] = device_->getIntProperty(rotation_property_id);
        log_readback(std::string(param_prefix) + ".orientation", "rotation",
                     image_rotation_[stream_index]);
      } catch (const std::exception&) {
      }
    }
  };

  sync_bool("device", "enable_heartbeat", enable_heartbeat_, OB_PROP_HEARTBEAT_BOOL);
  sync_bool("device", "retry_on_usb3_detection_failure", retry_on_usb3_detection_failure_,
            OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL);
  sync_bool("device", "enable_ptp_config", enable_ptp_config_,
            OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL);

  if (isConfigJsonLoaded()) {
    try {
      device_preset_ = device_->getCurrentPresetName();
      log_readback("depth", "device_preset", device_preset_);
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("Config final readback failed [depth] device_preset error=\"" << e.what()
                                                                                     << "\"");
    }
  }

  if (isConfigJsonLoaded() && can_read(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT)) {
    try {
      enable_depth_auto_exposure_priority_ =
          device_->getIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) != 0;
      log_readback("depth", "enable_depth_auto_exposure_priority",
                   enable_depth_auto_exposure_priority_);
    } catch (const std::exception&) {
    }
  }
  if (isConfigJsonLoaded() && can_read(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL)) {
    try {
      enable_ir_auto_exposure_ = device_->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
      log_readback("depth", "enable_ir_auto_exposure", enable_ir_auto_exposure_);
    } catch (const std::exception&) {
    }
  }
  sync_int("depth", "ir_ae_max_exposure", ir_ae_max_exposure_, OB_PROP_IR_AE_MAX_EXPOSURE_INT);
  sync_int("depth", "mean_intensity_set_point", mean_intensity_set_point_,
           OB_PROP_IR_BRIGHTNESS_INT);
  sync_int("depth", "depth_exposure", depth_exposure_, OB_PROP_DEPTH_EXPOSURE_INT);
  sync_int("depth", "ir_exposure", ir_exposure_, OB_PROP_IR_EXPOSURE_INT);
  sync_int("depth", "depth_gain", depth_gain_, OB_PROP_DEPTH_GAIN_INT);
  sync_int("depth", "ir_gain", ir_gain_, OB_PROP_IR_GAIN_INT);
  if (isConfigJsonLoaded() && can_read(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT)) {
    try {
      const auto depth_unit =
          device_->getFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT);
      depth_precision_str_ = std::to_string(depth_unit) + "mm";
      log_readback("depth", "depth_unit", depth_unit);
      log_readback("depth", "depth_precision", depth_precision_str_);
    } catch (const std::exception&) {
    }
  }
  if (isConfigJsonLoaded() && can_read(OB_PROP_LASER_CONTROL_INT)) {
    try {
      enable_laser_ = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT) != 0;
      log_readback("depth", "enable_laser", enable_laser_);
    } catch (const std::exception&) {
    }
  } else if (isConfigJsonLoaded() && can_read(OB_PROP_LASER_BOOL)) {
    try {
      enable_laser_ = device_->getBoolProperty(OB_PROP_LASER_BOOL);
      log_readback("depth", "enable_laser", enable_laser_);
    } catch (const std::exception&) {
    }
  }
  sync_int("depth", "laser_energy_level", laser_energy_level_, OB_PROP_LASER_ENERGY_LEVEL_INT);
  sync_bool("depth", "enable_ldp", enable_ldp_, OB_PROP_LDP_BOOL);

  if (isConfigJsonLoaded() && can_read_struct(OB_STRUCT_DEPTH_AE_ROI)) {
    try {
      OBRegionOfInterest config{};
      uint32_t data_size = sizeof(config);
      device_->getStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                 &data_size);
      depth_ae_roi_left_ = config.x0_left;
      depth_ae_roi_top_ = config.y0_top;
      depth_ae_roi_right_ = config.x1_right;
      depth_ae_roi_bottom_ = config.y1_bottom;
      std::ostringstream fields;
      fields << "left=" << depth_ae_roi_left_ << " top=" << depth_ae_roi_top_
             << " right=" << depth_ae_roi_right_ << " bottom=" << depth_ae_roi_bottom_;
      log_readback_fields("depth.ae_roi", fields.str());
    } catch (const std::exception&) {
    }
  }

  if (isConfigJsonLoaded()) {
    sync_bool("depth.interleave", "enable", interleave_frame_enable_,
              OB_PROP_FRAME_INTERLEAVE_ENABLE_BOOL);
    sync_int("depth.interleave", "skip_index", interleave_skip_index_,
             OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT);
    try {
      const auto* frame_interleave_name = device_->getCurrentFrameInterleaveName();
      if (frame_interleave_name != nullptr) {
        std::string mode(frame_interleave_name);
        std::string lower_mode = mode;
        std::transform(lower_mode.begin(), lower_mode.end(), lower_mode.begin(), ::tolower);
        if (lower_mode.find("laser") != std::string::npos) {
          interleave_ae_mode_ = "laser";
        } else if (lower_mode.find("hdr") != std::string::npos) {
          interleave_ae_mode_ = "hdr";
        } else {
          interleave_ae_mode_.clear();
        }
        log_readback("depth.interleave", "ae_mode", interleave_ae_mode_);
      }
    } catch (const std::exception&) {
    }
    if (!interleave_ae_mode_.empty() && can_write(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT)) {
      int original_interleave_index = interleave_skip_index_;
      bool has_original_interleave_index = false;
      if (can_read(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT)) {
        try {
          original_interleave_index =
              device_->getIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT);
          has_original_interleave_index = true;
        } catch (const std::exception&) {
        }
      }

      auto read_int_property = [&](OBPropertyID property_id, int& value) {
        if (!can_read(property_id)) {
          return;
        }
        try {
          value = device_->getIntProperty(property_id);
        } catch (const std::exception&) {
        }
      };
      auto sync_interleave_param = [&](int config_index, int& laser_control, int& depth_exposure,
                                       int& depth_gain, int& ir_brightness,
                                       int& ir_ae_max_exposure) {
        try {
          device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT, config_index);
          read_int_property(OB_PROP_LASER_CONTROL_INT, laser_control);
          read_int_property(OB_PROP_DEPTH_EXPOSURE_INT, depth_exposure);
          read_int_property(OB_PROP_DEPTH_GAIN_INT, depth_gain);
          read_int_property(OB_PROP_IR_BRIGHTNESS_INT, ir_brightness);
          read_int_property(OB_PROP_IR_AE_MAX_EXPOSURE_INT, ir_ae_max_exposure);
          std::ostringstream fields;
          fields << "laser_control=" << laser_control << " depth_exposure=" << depth_exposure
                 << " depth_gain=" << depth_gain << " depth_brightness=" << ir_brightness
                 << " depth_ae_max_exposure=" << ir_ae_max_exposure;
          log_readback_fields("depth.interleave.params." + std::to_string(config_index),
                              fields.str());
        } catch (const std::exception& e) {
          ROS_DEBUG_STREAM("Config final readback failed [depth.interleave.params."
                           << config_index << "] error=\"" << e.what() << "\"");
        }
      };

      if (interleave_ae_mode_ == "hdr") {
        sync_interleave_param(0, hdr_index0_laser_control_, hdr_index0_depth_exposure_,
                              hdr_index0_depth_gain_, hdr_index0_ir_brightness_,
                              hdr_index0_ir_ae_max_exposure_);
        sync_interleave_param(1, hdr_index1_laser_control_, hdr_index1_depth_exposure_,
                              hdr_index1_depth_gain_, hdr_index1_ir_brightness_,
                              hdr_index1_ir_ae_max_exposure_);
      } else if (interleave_ae_mode_ == "laser") {
        sync_interleave_param(0, laser_index0_laser_control_, laser_index0_depth_exposure_,
                              laser_index0_depth_gain_, laser_index0_ir_brightness_,
                              laser_index0_ir_ae_max_exposure_);
        sync_interleave_param(1, laser_index1_laser_control_, laser_index1_depth_exposure_,
                              laser_index1_depth_gain_, laser_index1_ir_brightness_,
                              laser_index1_ir_ae_max_exposure_);
      }

      if (has_original_interleave_index) {
        try {
          device_->setIntProperty(OB_PROP_FRAME_INTERLEAVE_CONFIG_INDEX_INT,
                                  original_interleave_index);
        } catch (const std::exception& e) {
          ROS_DEBUG_STREAM("Failed to restore frame_interleave.config_index: " << e.what());
        }
      }
    }
  }

  if (isConfigJsonLoaded()) {
    try {
      const bool hw = can_read(OB_PROP_DISPARITY_TO_DEPTH_BOOL) &&
                      device_->getBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL);
      const bool sw = can_read(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL) &&
                      device_->getBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL);
      disparity_to_depth_mode_ = hw ? "HW" : (sw ? "SW" : "disable");
      log_readback("depth", "disparity_to_depth_mode", disparity_to_depth_mode_);
    } catch (const std::exception&) {
    }
  }
  if (isConfigJsonLoaded() && can_read(OB_PROP_DISP_SEARCH_RANGE_MODE_INT)) {
    try {
      disparity_range_mode_ = std::stoi(
          disparityRangeModeToString(device_->getIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT)));
      log_readback("depth", "disparity_range_mode", disparity_range_mode_);
    } catch (const std::exception&) {
    }
  }
  sync_int("depth", "disparity_search_offset", disparity_search_offset_,
           OB_PROP_DISP_SEARCH_OFFSET_INT);

  sync_bool("depth", "enable_hardware_noise_removal_filter", enable_hardware_noise_removal_filter_,
            OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL);
  sync_float("depth", "hardware_noise_removal_filter_threshold",
             hardware_noise_removal_filter_threshold_,
             OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT);
  sync_bool("depth", "enable_noise_removal_filter", enable_noise_removal_filter_,
            OB_PROP_DEPTH_SOFT_FILTER_BOOL);
  sync_int("depth", "noise_removal_filter_min_diff", noise_removal_filter_min_diff_,
           OB_PROP_DEPTH_MAX_DIFF_INT);
  sync_int("depth", "noise_removal_filter_max_size", noise_removal_filter_max_size_,
           OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
  sync_bool("depth", "enable_disp_outliers_filter", enable_disp_outliers_filter_,
            OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL);
  sync_int("depth", "disp_outliers_filter_search_mode", disp_outliers_filter_search_mode_,
           OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT);

  sync_stream_orientation("depth", DEPTH, OB_PROP_DEPTH_FLIP_BOOL, OB_PROP_DEPTH_MIRROR_BOOL,
                          OB_PROP_DEPTH_ROTATE_INT);
  sync_stream_orientation("color", COLOR, OB_PROP_COLOR_FLIP_BOOL, OB_PROP_COLOR_MIRROR_BOOL,
                          OB_PROP_COLOR_ROTATE_INT);
  sync_stream_orientation("left_ir", INFRA1, OB_PROP_IR_FLIP_BOOL, OB_PROP_IR_MIRROR_BOOL,
                          OB_PROP_IR_ROTATE_INT);
  sync_stream_orientation("right_ir", INFRA2, OB_PROP_IR_RIGHT_FLIP_BOOL,
                          OB_PROP_IR_RIGHT_MIRROR_BOOL, OB_PROP_IR_RIGHT_ROTATE_INT);

  if (isConfigJsonLoaded() && can_read(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT)) {
    try {
      enable_color_auto_exposure_priority_ =
          device_->getIntProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT) != 0;
      log_readback("color", "enable_color_auto_exposure_priority",
                   enable_color_auto_exposure_priority_);
    } catch (const std::exception&) {
    }
  }
  sync_bool("color", "enable_color_auto_exposure", enable_color_auto_exposure_,
            OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
  sync_int("color", "color_denoising_level", color_denoising_level_,
           OB_PROP_COLOR_DENOISING_LEVEL_INT);
  sync_int("color", "color_ae_max_exposure", color_ae_max_exposure_,
           OB_PROP_COLOR_AE_MAX_EXPOSURE_INT);
  sync_int("color", "color_exposure", color_exposure_, OB_PROP_COLOR_EXPOSURE_INT);
  sync_int("color", "color_gain", color_gain_, OB_PROP_COLOR_GAIN_INT);
  sync_int("color", "color_brightness", color_brightness_, OB_PROP_COLOR_BRIGHTNESS_INT);
  sync_bool("color", "enable_color_auto_white_balance", enable_color_auto_white_balance_,
            OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
  sync_int("color", "color_white_balance", color_white_balance_, OB_PROP_COLOR_WHITE_BALANCE_INT);
  sync_int("color", "color_sharpness", color_sharpness_, OB_PROP_COLOR_SHARPNESS_INT);
  sync_int("color", "color_gamma", color_gamma_, OB_PROP_COLOR_GAMMA_INT);
  sync_int("color", "color_hue", color_hue_, OB_PROP_COLOR_HUE_INT);
  sync_int("color", "color_backlight_compensation", color_backlight_compensation_,
           OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT);
  sync_int("color", "color_contrast", color_contrast_, OB_PROP_COLOR_CONTRAST_INT);
  sync_int("color", "color_saturation", color_saturation_, OB_PROP_COLOR_SATURATION_INT);
  if (isConfigJsonLoaded() && can_read(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT)) {
    try {
      color_powerline_freq_ = colorPowerLineFrequencyToString(
          device_->getIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT));
      log_readback("color", "color_powerline_freq", color_powerline_freq_);
    } catch (const std::exception&) {
    }
  }
  sync_bool("color", "color_anti_flicker", color_anti_flicker_, OB_PROP_COLOR_ANTI_FLICKER_BOOL);
  if (isConfigJsonLoaded()) {
    try {
      if (device_->isColorPresetSupported()) {
        const char* color_preset = device_->getCurrentColorPresetName();
        if (color_preset != nullptr) {
          color_preset_ = color_preset;
          log_readback("color", "color_preset", color_preset_);
        }
      }
    } catch (const std::exception&) {
    }
  }
  if (isConfigJsonLoaded() && can_read_struct(OB_STRUCT_COLOR_AE_ROI)) {
    try {
      OBRegionOfInterest config{};
      uint32_t data_size = sizeof(config);
      device_->getStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                 &data_size);
      color_ae_roi_left_ = config.x0_left;
      color_ae_roi_top_ = config.y0_top;
      color_ae_roi_right_ = config.x1_right;
      color_ae_roi_bottom_ = config.y1_bottom;
      std::ostringstream fields;
      fields << "left=" << color_ae_roi_left_ << " top=" << color_ae_roi_top_
             << " right=" << color_ae_roi_right_ << " bottom=" << color_ae_roi_bottom_;
      log_readback_fields("color.ae_roi", fields.str());
    } catch (const std::exception&) {
    }
  }
}

void OBCameraNode::syncConfigJsonFilterSettings(
    const std::vector<std::shared_ptr<ob::Filter>>& filters, const std::string& sensor_name) {
  if (!config_json_loaded_) {
    return;
  }

  auto filter_scope = [&](const std::string& filter_name) {
    return "filter." + sensor_name + "." + normalizeDepthFilterName(filter_name);
  };
  auto log_readback = [](const std::string& scope, const std::string& name, const auto& value) {
    std::ostringstream ss;
    ss << std::boolalpha << value;
    ROS_INFO_STREAM("Config final readback [" << scope << "] " << name << "=" << ss.str());
  };
  auto log_readback_fields = [](const std::string& scope, const std::string& fields) {
    ROS_INFO_STREAM("Config final readback [" << scope << "] " << fields);
  };
  auto sync_filter_enabled = [&](const std::vector<std::shared_ptr<ob::Filter>>& filters,
                                 const std::string& filter_name, const std::string& param_name,
                                 bool& member) {
    auto it = std::find_if(filters.begin(), filters.end(), [&](const auto& filter) {
      return filter &&
             normalizeDepthFilterName(filter->type()) == normalizeDepthFilterName(filter_name);
    });
    if (it == filters.end()) {
      return std::shared_ptr<ob::Filter>{};
    }
    try {
      member = (*it)->isEnabled();
      log_readback(filter_scope(filter_name), param_name, member);
    } catch (const std::exception&) {
    }
    return *it;
  };
  if (sensor_name == "depth") {
    if (auto filter = sync_filter_enabled(filters, "DecimationFilter", "enable_decimation_filter",
                                          enable_decimation_filter_)) {
      try {
        decimation_filter_scale_range_ =
            static_cast<int>(filter->as<ob::DecimationFilter>()->getScaleValue());
        log_readback(filter_scope("DecimationFilter"), "scale", decimation_filter_scale_range_);
      } catch (const std::exception&) {
      }
    }
    if (auto filter = sync_filter_enabled(filters, "ThresholdFilter", "enable_threshold_filter",
                                          enable_threshold_filter_)) {
      try {
        threshold_filter_min_ = static_cast<int>(filter->getConfigValue("min"));
        threshold_filter_max_ = static_cast<int>(filter->getConfigValue("max"));
        std::ostringstream fields;
        fields << "min=" << threshold_filter_min_ << " max=" << threshold_filter_max_;
        log_readback_fields(filter_scope("ThresholdFilter"), fields.str());
      } catch (const std::exception&) {
      }
    }
    sync_filter_enabled(filters, "HDRMerge", "enable_hdr_merge", enable_hdr_merge_);
    if (auto filter = sync_filter_enabled(filters, "SequenceIdFilter", "enable_sequenced_filter",
                                          enable_sequenced_filter_)) {
      try {
        sequence_id_filter_id_ = filter->as<ob::SequenceIdFilter>()->getSelectSequenceId();
        log_readback(filter_scope("SequenceIdFilter"), "id", sequence_id_filter_id_);
      } catch (const std::exception&) {
      }
    }
    if (auto filter =
            sync_filter_enabled(filters, "SpatialFastFilter", "enable_spatial_fast_filter",
                                enable_spatial_fast_filter_)) {
      try {
        spatial_fast_filter_radius_ = filter->as<ob::SpatialFastFilter>()->getFilterParams().radius;
        log_readback(filter_scope("SpatialFastFilter"), "radius",
                     static_cast<int>(spatial_fast_filter_radius_));
      } catch (const std::exception&) {
      }
    }
    if (auto filter =
            sync_filter_enabled(filters, "SpatialModerateFilter", "enable_spatial_moderate_filter",
                                enable_spatial_moderate_filter_)) {
      try {
        auto params = filter->as<ob::SpatialModerateFilter>()->getFilterParams();
        spatial_moderate_filter_diff_threshold_ = params.disp_diff;
        spatial_moderate_filter_magnitude_ = params.magnitude;
        spatial_moderate_filter_radius_ = params.radius;
        std::ostringstream fields;
        fields << "diff_threshold=" << spatial_moderate_filter_diff_threshold_
               << " magnitude=" << spatial_moderate_filter_magnitude_
               << " radius=" << spatial_moderate_filter_radius_;
        log_readback_fields(filter_scope("SpatialModerateFilter"), fields.str());
      } catch (const std::exception&) {
      }
    }
    if (auto filter = sync_filter_enabled(filters, "SpatialAdvancedFilter", "enable_spatial_filter",
                                          enable_spatial_filter_)) {
      try {
        auto params = filter->as<ob::SpatialAdvancedFilter>()->getFilterParams();
        spatial_filter_alpha_ = params.alpha;
        spatial_filter_diff_threshold_ = params.disp_diff;
        spatial_filter_magnitude_ = params.magnitude;
        spatial_filter_radius_ = params.radius;
        std::ostringstream fields;
        fields << "alpha=" << spatial_filter_alpha_
               << " diff_threshold=" << spatial_filter_diff_threshold_
               << " magnitude=" << spatial_filter_magnitude_
               << " radius=" << spatial_filter_radius_;
        log_readback_fields(filter_scope("SpatialAdvancedFilter"), fields.str());
      } catch (const std::exception&) {
      }
    }
    if (auto filter = sync_filter_enabled(filters, "TemporalFilter", "enable_temporal_filter",
                                          enable_temporal_filter_)) {
      try {
        temporal_filter_diff_threshold_ = static_cast<float>(filter->getConfigValue("diff_scale"));
        temporal_filter_weight_ = static_cast<float>(filter->getConfigValue("weight"));
        std::ostringstream fields;
        fields << "diff_threshold=" << temporal_filter_diff_threshold_
               << " weight=" << temporal_filter_weight_;
        log_readback_fields(filter_scope("TemporalFilter"), fields.str());
      } catch (const std::exception&) {
      }
    }
    if (auto filter =
            sync_filter_enabled(filters, "HoleFillingFilter", "enable_hole_filling_filter",
                                enable_hole_filling_filter_)) {
      try {
        hole_filling_filter_mode_ =
            std::to_string(static_cast<int>(filter->as<ob::HoleFillingFilter>()->getFilterMode()));
        log_readback(filter_scope("HoleFillingFilter"), "mode", hole_filling_filter_mode_);
      } catch (const std::exception&) {
      }
    }
    if (auto filter =
            sync_filter_enabled(filters, "FalsePositiveFilter", "enable_false_positive_filter",
                                enable_false_positive_filter_)) {
      try {
        const auto config_schema_vec = filter->getConfigSchemaVec();
        for (const auto& config_schema : config_schema_vec) {
          if (config_schema.name == nullptr || config_schema.name[0] == '\0') {
            continue;
          }
          try {
            const auto value = filter->getConfigValue(config_schema.name);
            log_readback(filter_scope("FalsePositiveFilter"), config_schema.name,
                         formatFilterConfigValue(config_schema, value));
          } catch (const std::exception&) {
          }
        }
      } catch (const std::exception&) {
      }
    }
    sync_filter_enabled(filters, "DisparityTransform", "enable_disparity_to_depth",
                        enable_disparity_to_depth_);
  } else if (sensor_name == "color") {
    if (auto filter =
            sync_filter_enabled(filters, "DecimationFilter", "enable_color_decimation_filter",
                                enable_color_decimation_filter_)) {
      try {
        color_decimation_filter_scale_ =
            static_cast<int>(filter->as<ob::DecimationFilter>()->getScaleValue());
        log_readback(filter_scope("DecimationFilter"), "scale", color_decimation_filter_scale_);
      } catch (const std::exception&) {
      }
    }
  } else if (sensor_name == "left_ir") {
    if (auto filter =
            sync_filter_enabled(filters, "SequenceIdFilter", "enable_left_ir_sequence_id_filter",
                                enable_left_ir_sequence_id_filter_)) {
      try {
        left_ir_sequence_id_filter_id_ = filter->as<ob::SequenceIdFilter>()->getSelectSequenceId();
        log_readback(filter_scope("SequenceIdFilter"), "id", left_ir_sequence_id_filter_id_);
      } catch (const std::exception&) {
      }
    }
  } else if (sensor_name == "right_ir") {
    if (auto filter =
            sync_filter_enabled(filters, "SequenceIdFilter", "enable_right_ir_sequence_id_filter",
                                enable_right_ir_sequence_id_filter_)) {
      try {
        right_ir_sequence_id_filter_id_ = filter->as<ob::SequenceIdFilter>()->getSelectSequenceId();
        log_readback(filter_scope("SequenceIdFilter"), "id", right_ir_sequence_id_filter_id_);
      } catch (const std::exception&) {
      }
    }
  }
}

orbbec_camera::DepthFilterState OBCameraNode::buildDepthFilterState(
    const std::string& filter_name, bool enabled, const std::shared_ptr<ob::Filter>& filter) const {
  const auto normalized_filter_name = normalizeDepthFilterName(filter_name);
  orbbec_camera::DepthFilterState filter_state;
  filter_state.filter_name = getDepthFilterStatusName(normalized_filter_name);
  filter_state.enabled = enabled;
  auto toParamValue = [](const auto& value) {
    std::ostringstream ss;
    ss << value;
    return ss.str();
  };

  if (normalized_filter_name == "NoiseRemovalFilter") {
    appendDepthFilterParam(filter_state, "min_diff", toParamValue(noise_removal_filter_min_diff_));
    appendDepthFilterParam(filter_state, "max_size", toParamValue(noise_removal_filter_max_size_));
  } else if (normalized_filter_name == "HardwareNoiseRemovalFilter") {
    appendDepthFilterParam(filter_state, "threshold",
                           toParamValue(hardware_noise_removal_filter_threshold_));
  } else if (normalized_filter_name == "DispOutliersFilter") {
    appendDepthFilterParam(filter_state, "search_mode",
                           dispOutliersSearchModeToString(disp_outliers_filter_search_mode_));
  } else if (filter && shouldExposeDepthFilterParams(normalized_filter_name)) {
    try {
      auto config_schema_vec = filter->getConfigSchemaVec();
      for (const auto& config_schema : config_schema_vec) {
        if (config_schema.name == nullptr || config_schema.name[0] == '\0') {
          continue;
        }
        try {
          auto value = filter->getConfigValue(config_schema.name);
          appendDepthFilterParam(
              filter_state,
              getDepthFilterStatusParamName(normalized_filter_name, config_schema.name),
              formatFilterConfigValue(config_schema, value));
        } catch (const std::exception&) {
          // Skip unreadable param and keep exporting others.
        }
      }
    } catch (const std::exception&) {
      // Keep params empty when schema is unavailable.
    }
  }

  return filter_state;
}

orbbec_camera::DepthFilterState OBCameraNode::buildEnhancedDepthFilterState() const {
  orbbec_camera::DepthFilterState filter_state;
  filter_state.filter_name = "EnhancedDepthFilter";
  filter_state.enabled = enable_enhanced_depth_.load();
  appendDepthFilterParam(filter_state, "confidence_threshold",
                         std::to_string(enhanced_depth_confidence_threshold_));
  return filter_state;
}

void OBCameraNode::publishDepthFiltersStatus() {
  if (!depth_filters_status_pub_) {
    return;
  }

  std::vector<std::shared_ptr<ob::Filter>> depth_filters_snapshot;
  {
    std::lock_guard<std::mutex> depth_filter_lock(depth_filter_mutex_);
    depth_filters_snapshot = depth_filter_list_;
  }

  auto find_depth_filter = [&depth_filters_snapshot,
                            this](const std::string& filter_name) -> std::shared_ptr<ob::Filter> {
    const auto normalized_name = normalizeDepthFilterName(filter_name);
    auto it = std::find_if(depth_filters_snapshot.begin(), depth_filters_snapshot.end(),
                           [&normalized_name](const auto& filter) {
                             return normalizeDepthFilterName(filter->type()) == normalized_name ||
                                    normalizeDepthFilterName(filter->getName()) == normalized_name;
                           });
    if (it == depth_filters_snapshot.end()) {
      return nullptr;
    }
    return *it;
  };

  auto sync_filter_enabled = [&find_depth_filter](const std::string& filter_name,
                                                  bool& cached_state) {
    auto filter = find_depth_filter(filter_name);
    if (!filter) {
      return;
    }
    try {
      cached_state = filter->isEnabled();
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  };

  sync_filter_enabled("DecimationFilter", enable_decimation_filter_);
  sync_filter_enabled("HDRMerge", enable_hdr_merge_);
  sync_filter_enabled("SequenceIdFilter", enable_sequenced_filter_);
  sync_filter_enabled("SpatialAdvancedFilter", enable_spatial_filter_);
  sync_filter_enabled("TemporalFilter", enable_temporal_filter_);
  sync_filter_enabled("HoleFillingFilter", enable_hole_filling_filter_);
  sync_filter_enabled("DisparityTransform", enable_disparity_to_depth_);
  sync_filter_enabled("ThresholdFilter", enable_threshold_filter_);
  sync_filter_enabled("SpatialFastFilter", enable_spatial_fast_filter_);
  sync_filter_enabled("SpatialModerateFilter", enable_spatial_moderate_filter_);
  sync_filter_enabled("EdgeNoiseRemovalFilter", enable_edge_noise_removal_filter_);
  sync_filter_enabled("FalsePositiveFilter", enable_false_positive_filter_);
  sync_filter_enabled("MgcNoiseRemovalFilter", enable_mgc_noise_removal_filter_);
  sync_filter_enabled("LutNoiseRemovalFilter", enable_lut_noise_removal_filter_);

  if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
    try {
      enable_noise_removal_filter_ = device_->getBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
    try {
      noise_removal_filter_min_diff_ = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
    try {
      noise_removal_filter_max_size_ = device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                   OB_PERMISSION_READ_WRITE)) {
    try {
      enable_hardware_noise_removal_filter_ =
          device_->getBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                   OB_PERMISSION_READ_WRITE)) {
    try {
      hardware_noise_removal_filter_threshold_ =
          device_->getFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
    try {
      enable_disp_outliers_filter_ = device_->getBoolProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                   OB_PERMISSION_READ_WRITE)) {
    try {
      disp_outliers_filter_search_mode_ =
          device_->getIntProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT);
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }

  if (auto filter = find_depth_filter("DecimationFilter")) {
    try {
      decimation_filter_scale_range_ =
          static_cast<int>(filter->as<ob::DecimationFilter>()->getScaleValue());
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("SequenceIdFilter")) {
    try {
      sequence_id_filter_id_ = filter->as<ob::SequenceIdFilter>()->getSelectSequenceId();
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("ThresholdFilter")) {
    try {
      threshold_filter_min_ = static_cast<int>(filter->getConfigValue("min"));
      threshold_filter_max_ = static_cast<int>(filter->getConfigValue("max"));
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("SpatialAdvancedFilter")) {
    try {
      auto params = filter->as<ob::SpatialAdvancedFilter>()->getFilterParams();
      spatial_filter_alpha_ = params.alpha;
      spatial_filter_diff_threshold_ = params.disp_diff;
      spatial_filter_magnitude_ = params.magnitude;
      spatial_filter_radius_ = params.radius;
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("TemporalFilter")) {
    try {
      temporal_filter_diff_threshold_ = static_cast<float>(filter->getConfigValue("diff_scale"));
      temporal_filter_weight_ = static_cast<float>(filter->getConfigValue("weight"));
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("SpatialFastFilter")) {
    try {
      auto params = filter->as<ob::SpatialFastFilter>()->getFilterParams();
      spatial_fast_filter_radius_ = params.radius;
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }
  if (auto filter = find_depth_filter("SpatialModerateFilter")) {
    try {
      auto params = filter->as<ob::SpatialModerateFilter>()->getFilterParams();
      spatial_moderate_filter_diff_threshold_ = params.disp_diff;
      spatial_moderate_filter_magnitude_ = params.magnitude;
      spatial_moderate_filter_radius_ = params.radius;
    } catch (const std::exception&) {
      // Keep the cached value if runtime querying fails.
    }
  }

  orbbec_camera::DepthFiltersStatus msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = camera_name_;

  const bool noise_removal_filter_supported =
      device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE) ||
      device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE) ||
      device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE);
  const bool hardware_noise_removal_filter_supported =
      device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                   OB_PERMISSION_READ_WRITE) ||
      device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                   OB_PERMISSION_READ_WRITE);
  const bool disp_outliers_filter_supported =
      device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL, OB_PERMISSION_READ_WRITE) ||
      device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                   OB_PERMISSION_READ_WRITE);

  std::vector<std::string> ordered_filter_names;
  ordered_filter_names.reserve(depth_filters_snapshot.size() + 3);
  auto append_unique_filter_name = [&ordered_filter_names](const std::string& filter_name) {
    if (std::find(ordered_filter_names.begin(), ordered_filter_names.end(), filter_name) ==
        ordered_filter_names.end()) {
      ordered_filter_names.push_back(filter_name);
    }
  };
  for (const auto& filter : depth_filters_snapshot) {
    if (!filter) {
      continue;
    }
    append_unique_filter_name(normalizeDepthFilterName(filter->type()));
  }
  if (noise_removal_filter_supported) {
    append_unique_filter_name("NoiseRemovalFilter");
  }
  if (hardware_noise_removal_filter_supported) {
    append_unique_filter_name("HardwareNoiseRemovalFilter");
  }
  if (disp_outliers_filter_supported) {
    append_unique_filter_name("DispOutliersFilter");
  }
  append_unique_filter_name("EnhancedDepthFilter");

  msg.filters.reserve(ordered_filter_names.size());
  for (const auto& filter_name : ordered_filter_names) {
    if (filter_name == "EnhancedDepthFilter") {
      msg.filters.push_back(buildEnhancedDepthFilterState());
      continue;
    }
    bool enabled = false;
    if (filter_name == "NoiseRemovalFilter") {
      enabled = enable_noise_removal_filter_;
    } else if (filter_name == "HardwareNoiseRemovalFilter") {
      enabled = enable_hardware_noise_removal_filter_;
    } else if (filter_name == "DispOutliersFilter") {
      enabled = enable_disp_outliers_filter_;
    }
    auto filter = find_depth_filter(filter_name);
    if (filter && filter_name != "DispOutliersFilter") {
      try {
        enabled = filter->isEnabled();
      } catch (const std::exception&) {
        // Keep default value when runtime querying fails.
      }
    }

    auto filter_state = buildDepthFilterState(filter_name, enabled, filter);

    msg.filters.push_back(filter_state);
  }

  depth_filters_status_pub_.publish(msg);
}

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

  stream_name_[COLOR_LEFT] = "left_color";
  unit_step_size_[COLOR_LEFT] = 3;
  format_[COLOR_LEFT] = OB_FORMAT_RGB888;
  image_format_[COLOR_LEFT] = CV_8UC3;
  encoding_[COLOR_LEFT] = sensor_msgs::image_encodings::RGB8;
  format_str_[COLOR_LEFT] = "RGB";

  stream_name_[COLOR_RIGHT] = "right_color";
  unit_step_size_[COLOR_RIGHT] = 3;
  format_[COLOR_RIGHT] = OB_FORMAT_RGB888;
  image_format_[COLOR_RIGHT] = CV_8UC3;
  encoding_[COLOR_RIGHT] = sensor_msgs::image_encodings::RGB8;
  format_str_[COLOR_RIGHT] = "RGB";

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

  stream_name_[ACCEL] = "accel";  // ← Add this if missing
  stream_name_[GYRO] = "gyro";    // ← Add this if missing

  nh_ir_ = ros::NodeHandle(stream_name_[INFRA0]);
  nh_rgb_ = ros::NodeHandle(stream_name_[COLOR]);
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
  } else if (enable_stream_[COLOR_LEFT]) {
    base_stream_ = COLOR_LEFT;
  } else if (enable_stream_[COLOR_RIGHT]) {
    base_stream_ = COLOR_RIGHT;
  } else {
    ROS_ERROR_STREAM("No base stream is enabled!");
  }
}
void OBCameraNode::setupColorPostProcessFilter() {
  if (!enable_stream_[COLOR] && !enable_stream_[COLOR_LEFT] && !enable_stream_[COLOR_RIGHT]) {
    return;
  }
  try {
    auto color_sensor = device_->getSensor(OB_SENSOR_COLOR);
    if (color_sensor) {
      color_filter_list_ = color_sensor->createRecommendedFilters();
    }
  } catch (const std::exception& e) {
    ROS_DEBUG_STREAM("Main color sensor not found, trying left/right color sensors");
    try {
      auto left_color_sensor = device_->getSensor(OB_SENSOR_COLOR_LEFT);
      if (left_color_sensor) {
        left_color_filter_list_ = left_color_sensor->createRecommendedFilters();
      }
      auto right_color_sensor = device_->getSensor(OB_SENSOR_COLOR_RIGHT);
      if (right_color_sensor) {
        right_color_filter_list_ = right_color_sensor->createRecommendedFilters();
      }
    } catch (const std::exception& e) {
      ROS_DEBUG_STREAM("Left/Right color sensors not found either.");
    }
  }

  if (color_filter_list_.empty() && left_color_filter_list_.empty() &&
      right_color_filter_list_.empty()) {
    ROS_DEBUG("Color sensor filter lists are empty");
    return;
  }
  for (size_t i = 0; i < color_filter_list_.size(); i++) {
    auto filter = color_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_color_decimation_filter_},
    };
    std::string filter_name = filter->type();
    ROS_DEBUG_STREAM("Setting color " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end() &&
        isLaunchParamProvided("enable_color_decimation_filter")) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("set color " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
    }
    if (filter_name == "DecimationFilter" && enable_color_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      auto range = decimation_filter->getScaleRange();
      if (color_decimation_filter_scale_ != -1 && color_decimation_filter_scale_ <= range.max &&
          color_decimation_filter_scale_ >= range.min) {
        decimation_filter->setScaleValue(color_decimation_filter_scale_);
      }
      if (color_decimation_filter_scale_ != -1 && (color_decimation_filter_scale_ < range.min ||
                                                   color_decimation_filter_scale_ > range.max)) {
        ROS_ERROR_STREAM("Color Decimation filter scale value is out of range "
                         << range.min << " - " << range.max);
      }
      ROS_INFO_STREAM("Current color decimation filter scale value: "
                      << static_cast<int>(decimation_filter->getScaleValue()));
    }
  }
  for (size_t i = 0; i < left_color_filter_list_.size(); i++) {
    auto filter = left_color_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_left_color_decimation_filter_},
    };
    std::string filter_name = filter->type();
    ROS_DEBUG_STREAM("Setting left color " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end() &&
        isLaunchParamProvided("enable_left_color_decimation_filter")) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("set left color " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
    }
    if (filter_name == "DecimationFilter" && enable_left_color_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      auto range = decimation_filter->getScaleRange();
      if (left_color_decimation_filter_scale_ != -1 &&
          left_color_decimation_filter_scale_ <= range.max &&
          left_color_decimation_filter_scale_ >= range.min) {
        decimation_filter->setScaleValue(left_color_decimation_filter_scale_);
      }
      if (left_color_decimation_filter_scale_ != -1 &&
          (left_color_decimation_filter_scale_ < range.min ||
           left_color_decimation_filter_scale_ > range.max)) {
        ROS_ERROR_STREAM("Left Color Decimation filter scale value is out of range "
                         << range.min << " - " << range.max);
      }
      ROS_INFO_STREAM("Current left color decimation filter scale value: "
                      << static_cast<int>(decimation_filter->getScaleValue()));
    }
  }

  for (size_t i = 0; i < right_color_filter_list_.size(); i++) {
    auto filter = right_color_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_right_color_decimation_filter_},
    };
    std::string filter_name = filter->type();
    ROS_DEBUG_STREAM("Setting right color " << filter_name << "......");
    if (filter_params.find(filter_name) != filter_params.end() &&
        isLaunchParamProvided("enable_right_color_decimation_filter")) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("set right color " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
    }
    if (filter_name == "DecimationFilter" && enable_right_color_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      auto range = decimation_filter->getScaleRange();
      if (right_color_decimation_filter_scale_ != -1 &&
          right_color_decimation_filter_scale_ <= range.max &&
          right_color_decimation_filter_scale_ >= range.min) {
        decimation_filter->setScaleValue(right_color_decimation_filter_scale_);
      }
      if (right_color_decimation_filter_scale_ != -1 &&
          (right_color_decimation_filter_scale_ < range.min ||
           right_color_decimation_filter_scale_ > range.max)) {
        ROS_ERROR_STREAM("Right Color Decimation filter scale value is out of range "
                         << range.min << " - " << range.max);
      }
      ROS_INFO_STREAM("Current right color decimation filter scale value: "
                      << static_cast<int>(decimation_filter->getScaleValue()));
    }
  }
}

void OBCameraNode::setupIrPostProcessFilter() {
  if (!enable_stream_[INFRA0]) {
    return;
  }
  try {
    auto ir_sensor = device_->getSensor(OB_SENSOR_IR);
    ir_filter_list_ = ir_sensor->createRecommendedFilters();
    if (ir_filter_list_.empty()) {
      ROS_DEBUG_STREAM("IR sensor filter list is empty");
    }
  } catch (const ob::Error& e) {
    ROS_WARN_STREAM("Failed to setup ir filters: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    ROS_DEBUG_STREAM("Failed to setup ir filters: " << e.what());
  }
}

void OBCameraNode::setupUndistortionFilters() {
  hw_d2c_color_undistortion_filter_.reset();
  hw_d2c_color_undistortion_configured_ = false;

  auto remove_undistortion_filter = [](std::vector<std::shared_ptr<ob::Filter>>& filters) {
    filters.erase(std::remove_if(filters.begin(), filters.end(),
                                 [](const std::shared_ptr<ob::Filter>& filter) {
                                   return filter &&
                                          std::string(filter->type()) == "UnDistortionFilter";
                                 }),
                  filters.end());
  };

  auto find_or_create_filter =
      [&](std::vector<std::shared_ptr<ob::Filter>>& filters,
          OBStreamType stream_type) -> std::shared_ptr<ob::UnDistortionFilter> {
    for (auto& filter : filters) {
      if (filter && std::string(filter->type()) == "UnDistortionFilter") {
        auto undistortion_filter = filter->as<ob::UnDistortionFilter>();
        undistortion_filter->setStreamType(stream_type);
        undistortion_filter->enable(true);
        return undistortion_filter;
      }
    }
    auto undistortion_filter = std::make_shared<ob::UnDistortionFilter>(stream_type);
    undistortion_filter->enable(true);
    filters.push_back(undistortion_filter);
    return undistortion_filter;
  };

  auto setup_stream_filter = [&](const stream_index_pair& stream_index,
                                 std::vector<std::shared_ptr<ob::Filter>>& filters) {
    if (!enable_stream_[stream_index] || !enable_undistortion_[stream_index]) {
      return;
    }
    if (stream_index == LIDAR) {
      ROS_WARN_STREAM("Undistortion is not supported for lidar stream");
      return;
    }
    if (stream_index == COLOR && shouldUseHwD2CColorUndistortion()) {
      remove_undistortion_filter(filters);
      hw_d2c_color_undistortion_filter_ = std::make_shared<ob::UnDistortionFilter>(OB_STREAM_COLOR);
      hw_d2c_color_undistortion_filter_->enable(true);
      ROS_INFO_STREAM("Enable color undistortion with HW D2C depth intrinsic projection");
      return;
    }
    auto undistortion_filter = find_or_create_filter(filters, stream_index.first);
    undistortion_filter->clearNewCameraMatrix();
    ROS_INFO_STREAM("Enable " << stream_name_[stream_index] << " undistortion");
  };

  setup_stream_filter(COLOR, color_filter_list_);
  setup_stream_filter(COLOR_LEFT, left_color_filter_list_);
  setup_stream_filter(COLOR_RIGHT, right_color_filter_list_);
  setup_stream_filter(DEPTH, depth_filter_list_);
  setup_stream_filter(INFRA0, ir_filter_list_);
  setup_stream_filter(INFRA1, left_ir_filter_list_);
  setup_stream_filter(INFRA2, right_ir_filter_list_);
}

bool OBCameraNode::shouldUseHwD2CColorUndistortion() const {
  auto color_undistortion = enable_undistortion_.find(COLOR);
  return color_undistortion != enable_undistortion_.end() && color_undistortion->second &&
         isDabaiASeriesForHwD2C(device_info_->pid()) && depth_registration_ &&
         align_mode_ == "HW" && align_target_stream_ == OB_STREAM_COLOR &&
         enable_stream_.at(COLOR) && enable_stream_.at(DEPTH);
}

bool OBCameraNode::isHwD2CProfileSupported() const {
  if (!pipeline_) {
    return false;
  }
  auto color_profile_iter = stream_profile_.find(COLOR);
  auto depth_profile_iter = stream_profile_.find(DEPTH);
  if (color_profile_iter == stream_profile_.end() || depth_profile_iter == stream_profile_.end() ||
      !color_profile_iter->second || !depth_profile_iter->second) {
    return false;
  }

  auto supported_profiles =
      pipeline_->getD2CDepthProfileList(color_profile_iter->second, ALIGN_D2C_HW_MODE);
  if (!supported_profiles || supported_profiles->count() == 0) {
    return false;
  }

  auto selected_depth = depth_profile_iter->second->as<ob::VideoStreamProfile>();
  if (!selected_depth) {
    return false;
  }
  for (uint32_t i = 0; i < supported_profiles->getCount(); ++i) {
    auto supported = supported_profiles->getProfile(i)->as<ob::VideoStreamProfile>();
    if (!supported) {
      continue;
    }
    if (supported->getWidth() == selected_depth->getWidth() &&
        supported->getHeight() == selected_depth->getHeight() &&
        supported->getFormat() == selected_depth->getFormat() &&
        supported->getFps() == selected_depth->getFps()) {
      return true;
    }
  }
  return false;
}

bool OBCameraNode::shouldUseGeneratedCameraInfo(const stream_index_pair& stream_index) const {
  auto undistortion_iter = enable_undistortion_.find(stream_index);
  if (undistortion_iter != enable_undistortion_.end() && undistortion_iter->second) {
    return true;
  }
  if (stream_index == COLOR && hw_d2c_color_undistortion_configured_) {
    return true;
  }
  if (!depth_registration_) {
    return false;
  }
  return (stream_index == DEPTH && align_target_stream_ == OB_STREAM_COLOR) ||
         (stream_index == COLOR && align_target_stream_ == OB_STREAM_DEPTH);
}

std::string OBCameraNode::getEffectiveOpticalFrameId(const stream_index_pair& stream_index) const {
  if (depth_registration_ && stream_index == DEPTH && align_target_stream_ == OB_STREAM_COLOR) {
    return optical_frame_id_.at(COLOR);
  }
  if (depth_registration_ && stream_index == COLOR && align_target_stream_ == OB_STREAM_DEPTH) {
    return optical_frame_id_.at(DEPTH);
  }
  return optical_frame_id_.at(stream_index);
}

void OBCameraNode::configureHwD2CColorUndistortion(const std::shared_ptr<ob::Frame>& depth_frame) {
  if (!hw_d2c_color_undistortion_filter_ || hw_d2c_color_undistortion_configured_) {
    return;
  }
  if (!depth_frame) {
    ROS_WARN_ONCE("Skip HW D2C color undistortion setup because depth frame is not available");
    return;
  }
  auto stream_profile = depth_frame->getStreamProfile();
  if (!stream_profile) {
    ROS_WARN_ONCE("Skip HW D2C color undistortion setup because depth stream profile is null");
    return;
  }
  auto video_profile = stream_profile->as<ob::VideoStreamProfile>();
  if (!video_profile) {
    ROS_WARN_ONCE("Skip HW D2C color undistortion setup because depth profile is not video");
    return;
  }
  hw_d2c_color_undistortion_filter_->setNewCameraMatrix(video_profile->getIntrinsic());
  hw_d2c_color_undistortion_configured_ = true;
  ROS_INFO_STREAM("Configured HW D2C color undistortion with depth camera intrinsic");
}

void OBCameraNode::applyHwD2CColorUndistortion(std::shared_ptr<ob::FrameSet>& frame_set,
                                               const std::shared_ptr<ob::Frame>& depth_frame) {
  if (!frame_set || !hw_d2c_color_undistortion_filter_) {
    return;
  }
  configureHwD2CColorUndistortion(depth_frame);
  if (!hw_d2c_color_undistortion_configured_) {
    return;
  }
  auto undistorted_frame = hw_d2c_color_undistortion_filter_->process(frame_set);
  if (!undistorted_frame) {
    ROS_WARN_STREAM("HW D2C color undistortion filter returned null frame");
    return;
  }
  auto undistorted_frame_set = undistorted_frame->as<ob::FrameSet>();
  if (!undistorted_frame_set) {
    ROS_WARN_STREAM("HW D2C color undistortion filter returned non-frameset output");
    return;
  }
  frame_set = undistorted_frame_set;
}

void OBCameraNode::setupLeftIrPostProcessFilter() {
  if (!enable_stream_[INFRA1]) {
    return;
  }
  if (device_preset_ == "Dual Color Streams") {
    ROS_DEBUG_STREAM("Dual Color Streams preset, skip left ir filter setup");
    return;
  }
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
  if (isGemini335PID(pid)) {
    auto left_ir_sensor = device_->getSensor(OB_SENSOR_IR_LEFT);
    left_ir_filter_list_ = left_ir_sensor->createRecommendedFilters();
    if (left_ir_filter_list_.empty()) {
      ROS_DEBUG_STREAM("Left IR sensor filter list is empty");
      return;
    }
    for (size_t i = 0; i < left_ir_filter_list_.size(); i++) {
      auto filter = left_ir_filter_list_[i];
      std::map<std::string, bool> filter_params = {
          {"SequenceIdFilter", enable_left_ir_sequence_id_filter_},
      };
      std::string filter_name = filter->type();
      ROS_DEBUG_STREAM("Setting " << filter_name << "......");
      if (filter_params.find(filter_name) != filter_params.end() &&
          isLaunchParamProvided("enable_left_ir_sequence_id_filter")) {
        std::string value = filter_params[filter_name] ? "true" : "false";
        ROS_INFO_STREAM("set left ir " << filter_name << " to " << value);
        filter->enable(filter_params[filter_name]);
      }
      if (filter_name == "SequenceIdFilter" && enable_left_ir_sequence_id_filter_) {
        auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
        if (left_ir_sequence_id_filter_id_ != -1) {
          sequenced_filter->selectSequenceId(left_ir_sequence_id_filter_id_);
        }
        ROS_INFO_STREAM(
            "Current left ir SequenceIdFilter ID: " << sequenced_filter->getSelectSequenceId());
      }
    }
  }
}

void OBCameraNode::setupRightIrPostProcessFilter() {
  if (!enable_stream_[INFRA2]) {
    return;
  }
  if (device_preset_ == "Dual Color Streams") {
    ROS_DEBUG_STREAM("Dual Color Streams preset, skip right ir filter setup");
    return;
  }
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
  if (isGemini335PID(pid)) {
    auto right_ir_sensor = device_->getSensor(OB_SENSOR_IR_RIGHT);
    right_ir_filter_list_ = right_ir_sensor->createRecommendedFilters();
    if (right_ir_filter_list_.empty()) {
      ROS_DEBUG_STREAM("Right IR sensor filter list is empty");
      return;
    }
    for (size_t i = 0; i < right_ir_filter_list_.size(); i++) {
      auto filter = right_ir_filter_list_[i];
      std::map<std::string, bool> filter_params = {
          {"SequenceIdFilter", enable_right_ir_sequence_id_filter_},
      };
      std::string filter_name = filter->type();
      ROS_DEBUG_STREAM("Setting " << filter_name << "......");
      if (filter_params.find(filter_name) != filter_params.end() &&
          isLaunchParamProvided("enable_right_ir_sequence_id_filter")) {
        std::string value = filter_params[filter_name] ? "true" : "false";
        ROS_INFO_STREAM("set right ir " << filter_name << " to " << value);
        filter->enable(filter_params[filter_name]);
      }
      if (filter_name == "SequenceIdFilter" && enable_right_ir_sequence_id_filter_) {
        auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
        if (right_ir_sequence_id_filter_id_ != -1) {
          sequenced_filter->selectSequenceId(right_ir_sequence_id_filter_id_);
        }
        ROS_INFO_STREAM(
            "Current right ir SequenceIdFilter ID: " << sequenced_filter->getSelectSequenceId());
      }
    }
  }
}

void OBCameraNode::setupDepthPostProcessFilter() {
  if (device_preset_ == "Dual Color Streams") {
    ROS_DEBUG_STREAM("Dual Color Streams preset, skip depth filter setup");
    return;
  }
  if (!enable_stream_[DEPTH]) {
    return;
  }
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  // set depth sensor to filter
  auto depth_sensor = device_->getSensor(OB_SENSOR_DEPTH);
  depth_filter_list_ = depth_sensor->createRecommendedFilters();
  if (depth_filter_list_.empty()) {
    ROS_DEBUG_STREAM("Depth sensor filter list is empty");
    return;
  }
  auto depth_filter_enable_param = [](const std::string& filter_name) {
    if (filter_name == "DecimationFilter") {
      return std::string("enable_decimation_filter");
    }
    if (filter_name == "HDRMerge") {
      return std::string("enable_hdr_merge");
    }
    if (filter_name == "SequenceIdFilter") {
      return std::string("enable_sequenced_filter");
    }
    if (filter_name == "SpatialAdvancedFilter") {
      return std::string("enable_spatial_filter");
    }
    if (filter_name == "TemporalFilter") {
      return std::string("enable_temporal_filter");
    }
    if (filter_name == "HoleFillingFilter") {
      return std::string("enable_hole_filling_filter");
    }
    if (filter_name == "DisparityTransform") {
      return std::string("enable_disparity_to_depth");
    }
    if (filter_name == "ThresholdFilter") {
      return std::string("enable_threshold_filter");
    }
    if (filter_name == "SpatialFastFilter") {
      return std::string("enable_spatial_fast_filter");
    }
    if (filter_name == "SpatialModerateFilter") {
      return std::string("enable_spatial_moderate_filter");
    }
    if (filter_name == "EdgeNoiseRemovalFilter") {
      return std::string("enable_edge_noise_removal_filter");
    }
    if (filter_name == "FalsePositiveFilter") {
      return std::string("enable_false_positive_filter");
    }
    if (filter_name == "MgcNoiseRemovalFilter") {
      return std::string("enable_mgc_noise_removal_filter");
    }
    if (filter_name == "LutNoiseRemovalFilter") {
      return std::string("enable_lut_noise_removal_filter");
    }
    return std::string();
  };
  for (size_t i = 0; i < depth_filter_list_.size(); i++) {
    auto filter = depth_filter_list_[i];
    std::map<std::string, bool> filter_params = {
        {"DecimationFilter", enable_decimation_filter_},
        {"HDRMerge", enable_hdr_merge_},
        {"SequenceIdFilter", enable_sequenced_filter_},
        {"SpatialAdvancedFilter", enable_spatial_filter_},
        {"TemporalFilter", enable_temporal_filter_},
        {"HoleFillingFilter", enable_hole_filling_filter_},
        {"DisparityTransform", enable_disparity_to_depth_},
        {"ThresholdFilter", enable_threshold_filter_},
        {"SpatialFastFilter", enable_spatial_fast_filter_},
        {"SpatialModerateFilter", enable_spatial_moderate_filter_},
        {"EdgeNoiseRemovalFilter", enable_edge_noise_removal_filter_},
        {"FalsePositiveFilter", enable_false_positive_filter_},
        {"MgcNoiseRemovalFilter", enable_mgc_noise_removal_filter_},
        {"LutNoiseRemovalFilter", enable_lut_noise_removal_filter_},
    };
    std::string filter_name = filter->type();
    ROS_DEBUG_STREAM("Setting " << filter_name << "......");
    const auto enable_param_name = depth_filter_enable_param(filter_name);
    if (filter_params.find(filter_name) != filter_params.end() &&
        isLaunchParamProvided(enable_param_name)) {
      std::string value = filter_params[filter_name] ? "true" : "false";
      ROS_INFO_STREAM("Set depth filter " << filter_name << " to " << value);
      filter->enable(filter_params[filter_name]);
      filter_status_[filter_name] = static_cast<bool>(filter_params[filter_name]);
    }
    if (filter_name == "DecimationFilter" && enable_decimation_filter_) {
      auto decimation_filter = filter->as<ob::DecimationFilter>();
      if (decimation_filter_scale_range_ != -1) {
        decimation_filter->setScaleValue(decimation_filter_scale_range_);
      }
      ROS_INFO_STREAM("Current decimation filter scale value: "
                      << static_cast<int>(decimation_filter->getScaleValue()));
    } else if (filter_name == "ThresholdFilter" && enable_threshold_filter_) {
      auto threshold_filter = filter->as<ob::ThresholdFilter>();
      if (threshold_filter_min_ != -1 && threshold_filter_max_ != -1) {
        threshold_filter->setValueRange(threshold_filter_min_, threshold_filter_max_);
      }
      ROS_INFO_STREAM("Current threshold filter value range: "
                      << static_cast<int>(threshold_filter->getConfigValue("min")) << " - "
                      << static_cast<int>(threshold_filter->getConfigValue("max")));
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
      auto current_params = spatial_filter->getFilterParams();
      ROS_INFO_STREAM("Current SpatialFilter params: "
                      << "alpha=" << current_params.alpha
                      << ", disp_diff=" << current_params.disp_diff
                      << ", magnitude=" << static_cast<int>(current_params.magnitude)
                      << ", radius=" << current_params.radius);
    } else if (filter_name == "TemporalFilter" && enable_temporal_filter_) {
      auto temporal_filter = filter->as<ob::TemporalFilter>();
      if (temporal_filter_diff_threshold_ != -1 && temporal_filter_weight_ != -1) {
        temporal_filter->setDiffScale(temporal_filter_diff_threshold_);
        temporal_filter->setWeight(temporal_filter_weight_);
      }
      ROS_INFO_STREAM(
          "Current TemporalFilter params: "
          << "diff_scale=" << static_cast<float>(temporal_filter->getConfigValue("diff_scale"))
          << ", weight=" << static_cast<float>(temporal_filter->getConfigValue("weight")));
    } else if (filter_name == "HoleFillingFilter" && enable_hole_filling_filter_ &&
               !hole_filling_filter_mode_.empty()) {
      auto hole_filling_filter = filter->as<ob::HoleFillingFilter>();
      OBHoleFillingMode hole_filling_mode = holeFillingModeFromString(hole_filling_filter_mode_);
      hole_filling_filter->setFilterMode(hole_filling_mode);
      ROS_INFO_STREAM("Current HoleFillingFilter mode: "
                      << static_cast<int>(hole_filling_filter->getFilterMode()));
    } else if (filter_name == "SequenceIdFilter" && enable_sequenced_filter_) {
      auto sequenced_filter = filter->as<ob::SequenceIdFilter>();
      if (sequence_id_filter_id_ != -1) {
        sequenced_filter->selectSequenceId(sequence_id_filter_id_);
      }
      ROS_INFO_STREAM("Current SequenceIdFilter ID: " << sequenced_filter->getSelectSequenceId());
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
        device_->setStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                   reinterpret_cast<const uint8_t*>(&hdr_config),
                                   sizeof(OBHdrConfig));
      }
      hdr_merge_filter->enable(true);
      uint32_t hdr_config_size = sizeof(hdr_config);
      device_->getStructuredData(OB_STRUCT_DEPTH_HDR_CONFIG,
                                 reinterpret_cast<uint8_t*>(&hdr_config), &hdr_config_size);
      ROS_INFO_STREAM("Current HDRMerge params: "
                      << "exposure_1=" << hdr_config.exposure_1 << ", gain_1=" << hdr_config.gain_1
                      << ", exposure_2=" << hdr_config.exposure_2
                      << ", gain_2=" << hdr_config.gain_2);
    } else if (filter_name == "SpatialFastFilter" && enable_spatial_fast_filter_) {
      auto spatial_fast_filter = filter->as<ob::SpatialFastFilter>();
      OBSpatialFastFilterParams params{};
      if (spatial_fast_filter_radius_ != -1) {
        params.radius = spatial_fast_filter_radius_;
        spatial_fast_filter->setFilterParams(params);
      }
      auto current_params = spatial_fast_filter->getFilterParams();
      ROS_INFO_STREAM(
          "Current SpatialFastFilter radius: " << static_cast<int>(current_params.radius));
    } else if (filter_name == "SpatialModerateFilter" && enable_spatial_moderate_filter_) {
      auto spatial_moderate_filter = filter->as<ob::SpatialModerateFilter>();
      OBSpatialModerateFilterParams params{};
      if (spatial_moderate_filter_diff_threshold_ != -1 &&
          spatial_moderate_filter_magnitude_ != -1 && spatial_moderate_filter_radius_ != -1) {
        params.magnitude = spatial_moderate_filter_magnitude_;
        params.radius = spatial_moderate_filter_radius_;
        params.disp_diff = spatial_moderate_filter_diff_threshold_;
        spatial_moderate_filter->setFilterParams(params);
      }
      auto current_params = spatial_moderate_filter->getFilterParams();
      ROS_INFO_STREAM("Current SpatialModerateFilter params: "
                      << "disp_diff=" << current_params.disp_diff
                      << ", magnitude=" << static_cast<int>(current_params.magnitude)
                      << ", radius=" << static_cast<int>(current_params.radius));
    } else {
      ROS_DEBUG_STREAM("Skip setting " << filter_name);
    }
  }
  set_filter_srv_ = nh_.advertiseService<SetFilterRequest, SetFilterResponse>(
      "/" + camera_name_ + "/" + "set_filter",
      [this](SetFilterRequest& request, SetFilterResponse& response) {
        return setFilterCallback(request, response);
      });
}
void OBCameraNode::setupDevices() {
  auto should_apply_launch_config = [this](const std::string& param_name) {
    return isLaunchParamProvided(param_name);
  };

  if (!device_preset_.empty()) {
    try {
      ROS_DEBUG_STREAM("Available presets:");
      auto preset_list = device_->getAvailablePresetList();
      for (uint32_t i = 0; i < preset_list->getCount(); i++) {
        ROS_DEBUG_STREAM("Preset " << i << ": " << preset_list->getName(i));
      }
      device_->loadPreset(device_preset_.c_str());
      ROS_INFO_STREAM("Loaded device preset: " << device_->getCurrentPresetName());
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM(
          "Failed to load device preset: " << orbbec_camera::formatObErrorWithStatus(e));
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to load device preset: " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to load device preset");
    }
  }
  if (!color_preset_.empty()) {
    try {
      if (!device_->isColorPresetSupported()) {
        ROS_WARN_STREAM("Color preset is not supported by this device");
      } else {
        auto color_preset_list = device_->getColorPresetList();
        std::string selected_preset;
        std::ostringstream supported_presets;
        bool has_supported_preset = false;
        const uint32_t preset_count = color_preset_list ? color_preset_list->getCount() : 0;
        for (uint32_t i = 0; i < preset_count; ++i) {
          const char* preset_name = color_preset_list->getName(i);
          if (preset_name == nullptr) {
            continue;
          }
          if (has_supported_preset) {
            supported_presets << ", ";
          }
          supported_presets << preset_name;
          has_supported_preset = true;
          if (equalsIgnoreCase(color_preset_, preset_name)) {
            selected_preset = preset_name;
          }
        }

        if (selected_preset.empty()) {
          ROS_WARN_STREAM("Unsupported color_preset: " << color_preset_ << ". Supported values: "
                                                       << supported_presets.str());
        } else {
          device_->switchColorPreset(selected_preset.c_str());
          const char* current_preset = device_->getCurrentColorPresetName();
          color_preset_ = current_preset != nullptr ? current_preset : selected_preset;
          ROS_INFO_STREAM("Current color preset: " << color_preset_);
        }
      }
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM(
          "Failed to switch color preset: " << orbbec_camera::formatObErrorWithStatus(e));
    } catch (const std::exception& e) {
      ROS_ERROR_STREAM("Failed to switch color preset: " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to switch color preset");
    }
  }
  if (!preset_resolution_config_.empty()) {
    OBPresetResolutionConfig presetResolutionConfig;
    std::istringstream iss(preset_resolution_config_);
    std::string token;
    std::vector<int> values;
    values.reserve(4);
    while (std::getline(iss, token, ',')) {
      values.push_back(std::stoi(token));
    }

    if (values.size() >= 4) {
      presetResolutionConfig.width = values[0];
      presetResolutionConfig.height = values[1];
      presetResolutionConfig.irDecimationFactor = values[2];
      presetResolutionConfig.depthDecimationFactor = values[3];
    } else {
      ROS_ERROR_STREAM(
          "Invalid preset_resolution_config parameter. "
          "Expected format: width,height,ir_decimation_factor,depth_decimation_factor");
      return;
    }
    ROS_INFO_STREAM("Set preset resolution config: "
                    << "width=" << presetResolutionConfig.width
                    << ", height=" << presetResolutionConfig.height
                    << ", ir_decimation=" << presetResolutionConfig.irDecimationFactor
                    << ", depth_decimation=" << presetResolutionConfig.depthDecimationFactor);

    device_->setStructuredData(OB_STRUCT_PRESET_RESOLUTION_CONFIG,
                               (uint8_t*)&presetResolutionConfig, sizeof(presetResolutionConfig));
  }
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
      ROS_DEBUG_STREAM(stream_name_[stream_index]
                       << " sensor not supported by current device, skipping");
      enable_stream_[stream_index] = false;
    }
    if (enable) {
      // set rotation
      OBPropertyID rotationPropertyID = OB_PROP_DEPTH_ROTATE_INT;
      if (stream_index == COLOR) {
        rotationPropertyID = OB_PROP_COLOR_ROTATE_INT;
      } else if (stream_index == COLOR_LEFT) {
        rotationPropertyID = OB_PROP_COLOR_LEFT_ROTATE_INT;
      } else if (stream_index == COLOR_RIGHT) {
        rotationPropertyID = OB_PROP_COLOR_RIGHT_ROTATE_INT;
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
        device_->setIntProperty(rotationPropertyID, image_rotation_[stream_index]);
        ROS_INFO_STREAM("Current " << stream_name_[stream_index]
                                   << " rotation: " << device_->getIntProperty(rotationPropertyID));
      }
      // set flip
      OBPropertyID flipPropertyID = OB_PROP_DEPTH_FLIP_BOOL;
      if (stream_index == COLOR) {
        flipPropertyID = OB_PROP_COLOR_FLIP_BOOL;
      } else if (stream_index == COLOR_LEFT) {
        flipPropertyID = OB_PROP_COLOR_LEFT_FLIP_BOOL;
      } else if (stream_index == COLOR_RIGHT) {
        flipPropertyID = OB_PROP_COLOR_RIGHT_FLIP_BOOL;
      } else if (stream_index == DEPTH) {
        flipPropertyID = OB_PROP_DEPTH_FLIP_BOOL;
      } else if (stream_index == INFRA0) {
        flipPropertyID = OB_PROP_IR_FLIP_BOOL;
      } else if (stream_index == INFRA1) {
        flipPropertyID = OB_PROP_IR_FLIP_BOOL;
      } else if (stream_index == INFRA2) {
        flipPropertyID = OB_PROP_IR_RIGHT_FLIP_BOOL;
      }
      if (should_apply_launch_config(stream_name_[stream_index] + "_flip") &&
          device_->isPropertySupported(flipPropertyID, OB_PERMISSION_WRITE)) {
        device_->setBoolProperty(flipPropertyID, image_flip_[stream_index]);
        ROS_INFO_STREAM("Current " << stream_name_[stream_index] << " flip: "
                                   << (device_->getBoolProperty(flipPropertyID) ? "ON" : "OFF"));
      }
      // set mirror
      OBPropertyID mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      if (stream_index == COLOR) {
        mirrorPropertyID = OB_PROP_COLOR_MIRROR_BOOL;
      } else if (stream_index == COLOR_LEFT) {
        mirrorPropertyID = OB_PROP_COLOR_LEFT_MIRROR_BOOL;
      } else if (stream_index == COLOR_RIGHT) {
        mirrorPropertyID = OB_PROP_COLOR_RIGHT_MIRROR_BOOL;
      } else if (stream_index == DEPTH) {
        mirrorPropertyID = OB_PROP_DEPTH_MIRROR_BOOL;
      } else if (stream_index == INFRA0) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;
      } else if (stream_index == INFRA1) {
        mirrorPropertyID = OB_PROP_IR_MIRROR_BOOL;
      } else if (stream_index == INFRA2) {
        mirrorPropertyID = OB_PROP_IR_RIGHT_MIRROR_BOOL;
      }
      if (should_apply_launch_config(stream_name_[stream_index] + "_mirror") &&
          device_->isPropertySupported(mirrorPropertyID, OB_PERMISSION_WRITE)) {
        device_->setBoolProperty(mirrorPropertyID, image_mirror_[stream_index]);
        ROS_INFO_STREAM("Current " << stream_name_[stream_index] << " mirror: "
                                   << (device_->getBoolProperty(mirrorPropertyID) ? "ON" : "OFF"));
      }
    }
  }
  auto device_info = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info);
  auto pid = device_info->getPid();
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

  if (is_playback_device_) {
    return;
  }

  try {
    if (should_apply_launch_config("retry_on_usb3_detection_failure") &&
        retry_on_usb3_detection_failure_ &&
        device_->isPropertySupported(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEVICE_USB3_REPEAT_IDENTIFY_BOOL,
                               retry_on_usb3_detection_failure_);
    }
    if (sync_io_voltage_level_ != -1 &&
        device_->isPropertySupported(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT,
                                     OB_PERMISSION_READ_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT);
      if (sync_io_voltage_level_ < range.min || sync_io_voltage_level_ > range.max) {
        ROS_ERROR_STREAM("sync IO voltage level is out of range " << range.min << " - "
                                                                  << range.max);
      } else {
        device_->setIntProperty(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT, sync_io_voltage_level_);
        ROS_INFO_STREAM("Current sync IO voltage level: "
                        << device_->getIntProperty(OB_PROP_USB_SYNC_VOLTAGE_LEVEL_INT));
      }
    }
    if (noise_removal_filter_min_diff_ != -1 && enable_noise_removal_filter_ &&
        sensors_.find(DEPTH) != sensors_.end() &&
        device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
      auto default_noise_removal_filter_min_diff =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
      if (default_noise_removal_filter_min_diff != noise_removal_filter_min_diff_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, noise_removal_filter_min_diff_);
      }
      ROS_INFO_STREAM("Current noise_removal_filter_min_diff: "
                      << device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT));
    }
    if (noise_removal_filter_max_size_ != -1 && enable_noise_removal_filter_ &&
        sensors_.find(DEPTH) != sensors_.end() &&
        device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE)) {
      auto default_noise_removal_filter_max_size =
          device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
      if (default_noise_removal_filter_max_size != noise_removal_filter_max_size_) {
        device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, noise_removal_filter_max_size_);
      }
      ROS_INFO_STREAM("Current noise_removal_filter_max_size: "
                      << device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT));
    }
    if (should_apply_launch_config("enable_noise_removal_filter") &&
        sensors_.find(DEPTH) != sensors_.end() &&
        device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_noise_removal_filter_);
      ROS_INFO_STREAM("Set noise removal filter to "
                      << (enable_noise_removal_filter_ ? "true" : "false"));
    }
    if (!depth_work_mode_.empty() &&
        device_->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
      auto depthModeList = device_->getDepthWorkModeList();
      for (uint32_t i = 0; i < depthModeList->getCount(); i++) {
        ROS_INFO_STREAM("depthModeList[" << i << "]: " << (*depthModeList)[i].name);
      }
      device_->switchDepthWorkMode(depth_work_mode_.c_str());
      ROS_INFO_STREAM("Set depth work mode: " << depth_work_mode_);
    }
    if (laser_energy_level_ != -1 &&
        device_->isPropertySupported(OB_PROP_LASER_ENERGY_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_LASER_ENERGY_LEVEL_INT);
      if (laser_energy_level_ < range.min || laser_energy_level_ > range.max) {
        ROS_ERROR_STREAM("Laser energy level is out of range " << range.min << " - " << range.max);
      } else {
        device_->setIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT, laser_energy_level_);
        auto new_laser_energy_level = device_->getIntProperty(OB_PROP_LASER_ENERGY_LEVEL_INT);
        ROS_INFO_STREAM("Current laser energy level: " << new_laser_energy_level);
      }
    }
    if (should_apply_launch_config("enable_ldp") &&
        device_->isPropertySupported(OB_PROP_LDP_BOOL, OB_PERMISSION_READ_WRITE)) {
      if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
        auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
        device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
        device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
      } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
        if (!enable_ldp_) {
          auto laser_enable = device_->getBoolProperty(OB_PROP_LASER_BOOL);
          device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
          std::this_thread::sleep_for(std::chrono::milliseconds(3));
          device_->setBoolProperty(OB_PROP_LASER_BOOL, laser_enable);
        } else {
          device_->setBoolProperty(OB_PROP_LDP_BOOL, enable_ldp_);
        }
      }
      ROS_INFO_STREAM(
          "Current LDP: " << (device_->getBoolProperty(OB_PROP_LDP_BOOL) ? "ON" : "OFF"));
    }
    if (should_apply_launch_config("enable_firmware_log")) {
      device_->enableFirmwareLog(enable_firmware_log_);
      ROS_INFO_STREAM("Current firmware log: " << (enable_firmware_log_ ? "ON" : "OFF"));
    }
    if (should_apply_launch_config("enable_heartbeat") &&
        device_->isPropertySupported(OB_PROP_HEARTBEAT_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_HEARTBEAT_BOOL, enable_heartbeat_);
      ROS_INFO_STREAM("Current heartbeat: "
                      << (device_->getBoolProperty(OB_PROP_HEARTBEAT_BOOL) ? "ON" : "OFF"));
    }

    if (should_apply_launch_config("enable_color_hdr") && enable_color_hdr_ &&
        device_->isPropertySupported(OB_PROP_COLOR_HDR_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_HDR_BOOL, enable_color_hdr_);
    }
    if (!disparity_to_depth_mode_.empty() && sensors_.find(DEPTH) != sensors_.end() &&
        device_->isPropertySupported(OB_PROP_DISPARITY_TO_DEPTH_BOOL, OB_PERMISSION_READ_WRITE) &&
        device_->isPropertySupported(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      if (disparity_to_depth_mode_ == "HW") {
        device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 1);
        device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 0);
        ROS_INFO_STREAM("Disparity to depth mode: HW");
      } else if (disparity_to_depth_mode_ == "SW") {
        device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 0);
        device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 1);
        ROS_INFO_STREAM("Disparity to depth mode: SW");
      } else if (disparity_to_depth_mode_ == "disable") {
        device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, 0);
        device_->setBoolProperty(OB_PROP_SDK_DISPARITY_TO_DEPTH_BOOL, 0);
        ROS_INFO_STREAM("Disparity to depth mode: disabled");
      } else {
        ROS_WARN_STREAM("Unknown disparity to depth mode '" << disparity_to_depth_mode_
                                                            << "', keeping default settings");
      }
    }
    if (!sync_mode_str_.empty() &&
        device_->isPropertySupported(OB_PROP_SYNC_SIGNAL_TRIGGER_OUT_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      auto sync_config = device_->getMultiDeviceSyncConfig();
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
      ROS_INFO_STREAM("Current sync mode: " << sync_config.syncMode);
      if (sync_mode_ == OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING) {
        ROS_INFO_STREAM("Frames per trigger: " << sync_config.framesPerTrigger);
        sync_host_time_timer_ =
            nh_private_.createTimer(ros::Duration(0, software_trigger_period_ * 1000000),
                                    [this](const ros::TimerEvent&) { device_->triggerCapture(); });
      }
    }

    if (should_apply_launch_config("enable_color_auto_exposure_priority") &&
        device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT,
                                     OB_PERMISSION_WRITE)) {
      int set_enable_color_auto_exposure_priority = enable_color_auto_exposure_priority_ ? 1 : 0;
      device_->setIntProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT,
                              set_enable_color_auto_exposure_priority);
      ROS_INFO_STREAM(
          "Current color auto exposure priority: "
          << (device_->getIntProperty(OB_PROP_COLOR_AUTO_EXPOSURE_PRIORITY_INT) ? "ON" : "OFF"));
    }
    if (should_apply_launch_config("color_anti_flicker") &&
        device_->isPropertySupported(OB_PROP_COLOR_ANTI_FLICKER_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_ANTI_FLICKER_BOOL, color_anti_flicker_);
      ROS_INFO_STREAM(
          "Current color anti flicker: "
          << (device_->getBoolProperty(OB_PROP_COLOR_ANTI_FLICKER_BOOL) ? "ON" : "OFF"));
    }
    if (should_apply_launch_config("enable_color_auto_white_balance") &&
        device_->isPropertySupported(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL,
                               enable_color_auto_white_balance_);
      ROS_INFO_STREAM(
          "Current color auto white balance: "
          << (device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL) ? "ON" : "OFF"));
    }
    if (color_backlight_compensation_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT,
                                     OB_PERMISSION_WRITE)) {
      device_->setIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT,
                              color_backlight_compensation_);
      ROS_INFO_STREAM("Current color backlight compensation: "
                      << device_->getIntProperty(OB_PROP_COLOR_BACKLIGHT_COMPENSATION_INT));
    }
    if (color_denoising_level_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_DENOISING_LEVEL_INT, OB_PERMISSION_WRITE)) {
      device_->setIntProperty(OB_PROP_COLOR_DENOISING_LEVEL_INT, color_denoising_level_);
      ROS_INFO_STREAM("Current color denoising level: "
                      << device_->getIntProperty(OB_PROP_COLOR_DENOISING_LEVEL_INT));
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
      const auto current_freq = device_->getIntProperty(OB_PROP_COLOR_POWER_LINE_FREQUENCY_INT);
      ROS_INFO_STREAM(
          "Current color powerline freq: " << colorPowerLineFrequencyToString(current_freq));
    }
    if (should_apply_launch_config("enable_color_auto_exposure") &&
        device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ_WRITE)) {
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
        ROS_ERROR_STREAM("color gain value is out of range [" << range.min << "," << range.max
                                                              << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_GAIN_INT, color_gain_);
        ROS_INFO_STREAM("Current color gain: " << device_->getIntProperty(OB_PROP_COLOR_GAIN_INT));
      }
    }
    if (color_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_BRIGHTNESS_INT);
      if (color_brightness_ < range.min || color_brightness_ > range.max) {
        ROS_ERROR_STREAM("color brightness value is out of range [" << range.min << "," << range.max
                                                                    << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT, color_brightness_);
        ROS_INFO_STREAM(
            "Current color brightness: " << device_->getIntProperty(OB_PROP_COLOR_BRIGHTNESS_INT));
      }
    }
    if (color_roi_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_ROI_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_ROI_BRIGHTNESS_INT);
      if (color_roi_brightness_ < range.min || color_roi_brightness_ > range.max) {
        ROS_ERROR_STREAM("color ROI brightness value is out of range ["
                         << range.min << "," << range.max << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_ROI_BRIGHTNESS_INT, color_roi_brightness_);
        ROS_INFO_STREAM("Current color ROI brightness: "
                        << device_->getIntProperty(OB_PROP_COLOR_ROI_BRIGHTNESS_INT));
      }
    }
    if (color_sharpness_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_SHARPNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_SHARPNESS_INT);
      if (color_sharpness_ < range.min || color_sharpness_ > range.max) {
        ROS_ERROR_STREAM("color sharpness value is out of range [" << range.min << "," << range.max
                                                                   << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, color_sharpness_);
        ROS_INFO_STREAM(
            "Current color sharpness: " << device_->getIntProperty(OB_PROP_COLOR_SHARPNESS_INT));
      }
    }
    if (color_gamma_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_GAMMA_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_GAMMA_INT);
      if (color_gamma_ < range.min || color_gamma_ > range.max) {
        ROS_ERROR_STREAM("color gamm value is out of range [" << range.min << "," << range.max
                                                              << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_GAMMA_INT, color_gamma_);
        ROS_INFO_STREAM(
            "Current color gamma: " << device_->getIntProperty(OB_PROP_COLOR_GAMMA_INT));
      }
    }
    if (color_white_balance_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_WHITE_BALANCE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_WHITE_BALANCE_INT);
      if (color_white_balance_ < range.min || color_white_balance_ > range.max) {
        ROS_ERROR_STREAM("color white balance value is out of range ["
                         << range.min << "," << range.max << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, color_white_balance_);
        ROS_INFO_STREAM("Current color white balance: "
                        << device_->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT));
      }
    }
    if (color_saturation_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_SATURATION_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_SATURATION_INT);
      if (color_saturation_ < range.min || color_saturation_ > range.max) {
        ROS_ERROR_STREAM("color saturation value is out of range [" << range.min << "," << range.max
                                                                    << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_SATURATION_INT, color_saturation_);
        ROS_INFO_STREAM(
            "Current color saturation: " << device_->getIntProperty(OB_PROP_COLOR_SATURATION_INT));
      }
    }
    if (color_contrast_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_CONTRAST_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_CONTRAST_INT);
      if (color_contrast_ < range.min || color_contrast_ > range.max) {
        ROS_ERROR_STREAM("color contrast value is out of range [" << range.min << "," << range.max
                                                                  << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_CONTRAST_INT, color_contrast_);
        ROS_INFO_STREAM(
            "Current color contrast: " << device_->getIntProperty(OB_PROP_COLOR_CONTRAST_INT));
      }
    }
    if (color_hue_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_HUE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_HUE_INT);
      if (color_hue_ < range.min || color_hue_ > range.max) {
        ROS_ERROR_STREAM("color hue value is out of range [" << range.min << "," << range.max
                                                             << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_HUE_INT, color_hue_);
        ROS_INFO_STREAM("Current color hue: " << device_->getIntProperty(OB_PROP_COLOR_HUE_INT));
      }
    }
    if (color_ae_max_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT);
      if (color_ae_max_exposure_ < range.min || color_ae_max_exposure_ > range.max) {
        ROS_ERROR_STREAM("color AE max exposure value is out of range ["
                         << range.min << "," << range.max << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT, color_ae_max_exposure_);
        ROS_INFO_STREAM("Current color AE max exposure: "
                        << device_->getIntProperty(OB_PROP_COLOR_AE_MAX_EXPOSURE_INT));
      }
    }
    if (color_ae_max_gain_ != -1 &&
        device_->isPropertySupported(OB_PROP_COLOR_AE_MAX_GAIN_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_COLOR_AE_MAX_GAIN_INT);
      if (color_ae_max_gain_ < range.min || color_ae_max_gain_ > range.max) {
        ROS_ERROR_STREAM("color AE max gain value is out of range ["
                         << range.min << "," << range.max << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_COLOR_AE_MAX_GAIN_INT, color_ae_max_gain_);
        ROS_INFO_STREAM("Current color AE max gain: "
                        << device_->getIntProperty(OB_PROP_COLOR_AE_MAX_GAIN_INT));
      }
    }
    if (should_apply_launch_config("enable_ir_auto_exposure") &&
        device_->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    }
    if (should_apply_launch_config("enable_depth_auto_exposure_priority") &&
        sensors_.find(DEPTH) != sensors_.end() &&
        device_->isPropertySupported(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT,
                                     OB_PERMISSION_WRITE)) {
      int set_enable_depth_auto_exposure_priority = enable_depth_auto_exposure_priority_ ? 1 : 0;
      device_->setIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT,
                              set_enable_depth_auto_exposure_priority);
      ROS_INFO_STREAM(
          "Current depth auto exposure priority: "
          << (device_->getIntProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_PRIORITY_INT) ? "ON" : "OFF"));
    }
    if (mean_intensity_set_point_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT);
      if (mean_intensity_set_point_ < range.min || mean_intensity_set_point_ > range.max) {
        ROS_ERROR_STREAM("depth brightness value is out of range [" << range.min << "," << range.max
                                                                    << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, mean_intensity_set_point_);
        ROS_INFO_STREAM(
            "Current depth brightness: " << device_->getIntProperty(OB_PROP_IR_BRIGHTNESS_INT));
      }
    }
    if (should_apply_launch_config("enable_ir_auto_exposure") &&
        device_->isPropertySupported(OB_PROP_IR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    }
    if (ir_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_EXPOSURE_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_IR_EXPOSURE_INT, ir_exposure_);
    }
    if (ir_gain_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_GAIN_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_IR_GAIN_INT, ir_gain_);
    }
    if (ir_brightness_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_BRIGHTNESS_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_BRIGHTNESS_INT);
      if (ir_brightness_ < range.min || ir_brightness_ > range.max) {
        ROS_ERROR_STREAM("IR brightness value is out of range [" << range.min << "," << range.max
                                                                 << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_IR_BRIGHTNESS_INT, ir_brightness_);
        ROS_INFO_STREAM(
            "Current IR brightness: " << device_->getIntProperty(OB_PROP_IR_BRIGHTNESS_INT));
      }
    }
    if (ir_ae_max_exposure_ != -1 &&
        device_->isPropertySupported(OB_PROP_IR_AE_MAX_EXPOSURE_INT, OB_PERMISSION_WRITE)) {
      auto range = device_->getIntPropertyRange(OB_PROP_IR_AE_MAX_EXPOSURE_INT);
      if (ir_ae_max_exposure_ < range.min || ir_ae_max_exposure_ > range.max) {
        ROS_ERROR_STREAM("IR AE max exposure value is out of range ["
                         << range.min << "," << range.max << "] please check the value");
      } else {
        device_->setIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT, ir_ae_max_exposure_);
        ROS_INFO_STREAM("Current IR AE max exposure: "
                        << device_->getIntProperty(OB_PROP_IR_AE_MAX_EXPOSURE_INT));
      }
    }
    if (should_apply_launch_config("enable_laser") &&
        device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, enable_laser_);
    }
    if (should_apply_launch_config("enable_laser") &&
        device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_LASER_BOOL, enable_laser_);
    }
    if (should_apply_launch_config("enable_ptp_config") &&
        device_->isPropertySupported(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      ROS_INFO_STREAM("Set PTP Config: " << (enable_ptp_config_ ? "ON" : "OFF"));
      device_->setBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL, enable_ptp_config_);
    }
    if (!depth_precision_str_.empty() &&
        device_->isPropertySupported(OB_PROP_DEPTH_PRECISION_LEVEL_INT, OB_PERMISSION_READ_WRITE)) {
      auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      if (default_precision_level != depth_precision_level_) {
        device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_level_);
        const auto current_depth_precision =
            device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
        ROS_INFO_STREAM(
            "Current depth precision: " << depthPrecisionLevelToString(current_depth_precision));
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
        device_->setFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT,
                                  depth_unit_flexible_adjustment);
        ROS_INFO_STREAM("Current depth unit: "
                        << device_->getFloatProperty(OB_PROP_DEPTH_UNIT_FLEXIBLE_ADJUSTMENT_FLOAT)
                        << "mm");
      }
    }
    if (should_apply_launch_config("enable_color_auto_exposure") &&
        device_->isPropertySupported(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    }
    if (should_apply_launch_config("enable_ir_long_exposure") &&
        device_->isPropertySupported(OB_PROP_IR_LONG_EXPOSURE_BOOL, OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_IR_LONG_EXPOSURE_BOOL, enable_ir_long_exposure_);
    }
    if (disparity_range_mode_ != -1 &&
        device_->isPropertySupported(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, OB_PERMISSION_WRITE)) {
      if (disparity_range_mode_ == 64) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 0);
      } else if (disparity_range_mode_ == 128) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 1);
      } else if (disparity_range_mode_ == 256) {
        device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, 2);
      } else {
        ROS_ERROR_STREAM("disparity range mode does not support this setting");
      }
      const auto current_mode = device_->getIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
      ROS_INFO_STREAM("Current disparity range mode: " << disparityRangeModeToString(current_mode));
    }
    if (should_apply_launch_config("enable_hardware_noise_removal_filter") &&
        device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                     OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                               enable_hardware_noise_removal_filter_);
      ROS_INFO_STREAM("Set hardware noise removal filter to "
                      << (device_->getBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL)
                              ? "true"
                              : "false"));
      if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                       OB_PERMISSION_READ_WRITE)) {
        if (hardware_noise_removal_filter_threshold_ != -1.0 &&
            enable_hardware_noise_removal_filter_) {
          device_->setFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                    hardware_noise_removal_filter_threshold_);
          ROS_INFO_STREAM(
              "Current hardware noise removal filter threshold: "
              << device_->getFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT));
        }
      }
    }
    if (should_apply_launch_config("enable_disp_outliers_filter") &&
        device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL, enable_disp_outliers_filter_);
      ROS_INFO_STREAM(
          "Set DispOutliersFilter to "
          << (device_->getBoolProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL) ? "true" : "false"));
    }
    if (disp_outliers_filter_search_mode_ != -1 &&
        device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                              disp_outliers_filter_search_mode_);
      ROS_INFO_STREAM("Current DispOutliersFilter search mode: "
                      << device_->getIntProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT));
    }
    if (!exposure_range_mode_.empty() && exposure_range_mode_ != "default" &&
        device_->isPropertySupported(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, OB_PERMISSION_WRITE)) {
      if (exposure_range_mode_ == "ultimate") {
        device_->setIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, 1);
      } else if (exposure_range_mode_ == "regular") {
        device_->setIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT, 0);
      } else {
        ROS_ERROR_STREAM("exposure range mode does not support this setting");
      }
      const auto current_mode = device_->getIntProperty(OB_PROP_DEVICE_PERFORMANCE_MODE_INT);
      ROS_INFO_STREAM("Current exposure range mode: " << exposureRangeModeToString(current_mode));
    }
    if (should_apply_launch_config("enable_accel_data_correction") &&
        device_->isPropertySupported(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL,
                                     OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL,
                               enable_accel_data_correction_);
      ROS_INFO_STREAM(
          "Current accel data correction: "
          << (device_->getBoolProperty(OB_PROP_SDK_ACCEL_FRAME_TRANSFORMED_BOOL) ? "ON" : "OFF"));
    }
    if (should_apply_launch_config("enable_gyro_data_correction") &&
        device_->isPropertySupported(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL,
                                     OB_PERMISSION_WRITE)) {
      device_->setBoolProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL,
                               enable_gyro_data_correction_);
      ROS_INFO_STREAM(
          "Current gyro data correction: "
          << (device_->getBoolProperty(OB_PROP_SDK_GYRO_FRAME_TRANSFORMED_BOOL) ? "ON" : "OFF"));
    }
    if (isGemini335PID(pid) && !intra_camera_sync_reference_.empty() &&
        (sync_mode_ == OB_MULTI_DEVICE_SYNC_MODE_SOFTWARE_TRIGGERING ||
         sync_mode_ == OB_MULTI_DEVICE_SYNC_MODE_HARDWARE_TRIGGERING) &&
        device_->isPropertySupported(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT,
                                     OB_PERMISSION_WRITE)) {
      if (intra_camera_sync_reference_ == "Start") {
        device_->setIntProperty(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT, 0);
      } else if (intra_camera_sync_reference_ == "Middle") {
        device_->setIntProperty(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT, 1);
      } else if (intra_camera_sync_reference_ == "End") {
        device_->setIntProperty(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT, 2);
      } else {
        ROS_ERROR_STREAM("Intra camera sync reference does not support this setting");
      }
      const auto current_ref = device_->getIntProperty(OB_PROP_INTRA_CAMERA_SYNC_REFERENCE_INT);
      ROS_INFO_STREAM(
          "Current intra camera sync reference: " << intraCameraSyncReferenceToString(current_ref));
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to setup devices: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to setup devices: " << e.what());
  }
  if (!ae_strategy_.empty() &&
      device_->isPropertySupported(OB_PROP_DEVICE_AE_STRATEGY_INT, OB_PERMISSION_WRITE)) {
    device_->setIntProperty(OB_PROP_DEVICE_AE_STRATEGY_INT, ae_strategy_ == "motion" ? 1 : 0);
    ROS_INFO_STREAM("Current AE Strategy: " << ae_strategy_);
  }
  if ((ae_reference_stream_ == "depth" || ae_reference_stream_ == "color") &&
      device_->isPropertySupported(OB_PROP_DEVICE_AE_REFERENCE_INT, OB_PERMISSION_WRITE)) {
    auto ae_reference = ae_reference_stream_ == "depth" ? 0 : 1;
    device_->setIntProperty(OB_PROP_DEVICE_AE_REFERENCE_INT, ae_reference);
    auto current_ae_reference = device_->getIntProperty(OB_PROP_DEVICE_AE_REFERENCE_INT);
    ROS_INFO_STREAM(
        "Current AE Reference Stream: " << (current_ae_reference == 0 ? "depth" : "color"));
  }
}

void OBCameraNode::setupFrameCallback() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      auto callback = [this, stream_index](std::shared_ptr<ob::Frame> frame) {
        this->onNewStandaloneFrameCallback(frame, stream_index);
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
  auto pid = device_->getDeviceInfo()->getPid();
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
        if (isGemini305SeriesPID(pid) && stream_index == DEPTH) {
          OBHardwareDecimationConfig conf;
          conf.originWidth = width_[stream_index];
          conf.originHeight = height_[stream_index];
          conf.factor = depth_decimation_factor_;
          selected_profile =
              profile_list->getVideoStreamProfile(conf, format_[stream_index], fps_[stream_index]);
        } else if (isGemini305SeriesPID(pid) && stream_index == INFRA1) {
          OBHardwareDecimationConfig conf;
          conf.originWidth = width_[stream_index];
          conf.originHeight = height_[stream_index];
          conf.factor = left_ir_decimation_factor_;
          selected_profile =
              profile_list->getVideoStreamProfile(conf, format_[stream_index], fps_[stream_index]);
        } else if (isGemini305SeriesPID(pid) && stream_index == INFRA2) {
          OBHardwareDecimationConfig conf;
          conf.originWidth = width_[stream_index];
          conf.originHeight = height_[stream_index];
          conf.factor = right_ir_decimation_factor_;
          selected_profile =
              profile_list->getVideoStreamProfile(conf, format_[stream_index], fps_[stream_index]);
        } else {
          selected_profile =
              profile_list->getVideoStreamProfile(width_[stream_index], height_[stream_index],
                                                  format_[stream_index], fps_[stream_index]);
        }
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
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to setup  "
                       << stream_name_[stream_index] << " profile: " << width_[stream_index] << "x"
                       << height_[stream_index] << " " << fps_[stream_index] << "fps "
                       << OBFormatToString(format_[stream_index])
                       << " ERROR:" << orbbec_camera::formatObErrorWithStatus(e));
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
      ROS_ERROR_STREAM("Failed to setup << " << stream_name_[stream_index] << " profile: "
                                             << orbbec_camera::formatObErrorWithStatus(e));
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
      ROS_ERROR_STREAM("set d2c error " << orbbec_camera::formatObErrorWithStatus(e));
    }
  }
  if (depth_registration_ && align_mode_ == "SW") {
    align_filter_ = std::make_shared<ob::Align>(align_target_stream_);
    align_filter_->setMatchTargetResolution(true);
    ROS_INFO_STREAM("SW D2C align output resolution will match target stream resolution");
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
  if (selected_profile->format() == OB_FORMAT_Y16 &&
      (stream_index == COLOR || stream_index == COLOR_LEFT || stream_index == COLOR_RIGHT)) {
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
  for (const auto& stream_index : IMAGE_STREAMS) {
    ROS_DEBUG_STREAM("Setting up publisher for stream: " << stream_name_[stream_index]);
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string name = stream_name_[stream_index];
    std::string topic_name = "/" + camera_name_ + "/" + name + "/image_raw";

    // Create subscriber status callbacks for ros::Publisher
    ros::SubscriberStatusCallback image_subscribed_cb =
        boost::bind(&OBCameraNode::imageSubscribedCallback, this, stream_index);
    ros::SubscriberStatusCallback image_unsubscribed_cb =
        boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, stream_index);

    // Create wrapper callbacks for image_transport::Publisher (they have different parameter
    // types)
    image_transport::SubscriberStatusCallback image_transport_subscribed_cb =
        [this, stream_index](const image_transport::SingleSubscriberPublisher&) {
          this->imageSubscribedCallback(stream_index);
        };
    image_transport::SubscriberStatusCallback image_transport_unsubscribed_cb =
        [this, stream_index](const image_transport::SingleSubscriberPublisher&) {
          this->imageUnsubscribedCallback(stream_index);
        };

    if (isMjpgColorStream(stream_index) || !enable_image_transport_plugins_) {
      raw_image_publishers_[stream_index] = nh_.advertise<sensor_msgs::Image>(
          topic_name, 1, image_subscribed_cb, image_unsubscribed_cb);
      if (isMjpgColorStream(stream_index)) {
        compressed_image_publishers_[stream_index] = nh_.advertise<sensor_msgs::CompressedImage>(
            topic_name + "/compressed", 1, image_subscribed_cb, image_unsubscribed_cb);
      }
    } else {
      // Use global publisher cache with callbacks to prevent plugin reloading and enable proper
      // subscriber detection.
      image_publishers_[stream_index] = getGlobalImagePublisher(
          topic_name, image_transport_subscribed_cb, image_transport_unsubscribed_cb);
    }

    topic_name = "/" + camera_name_ + "/" + name + "/camera_info";
    camera_info_publishers_[stream_index] = nh_.advertise<sensor_msgs::CameraInfo>(
        topic_name, 1, image_subscribed_cb, image_unsubscribed_cb);
    CHECK_NOTNULL(device_info_.get());
    auto pid = device_info_->getPid();
    if (isPublishMetaData(pid)) {
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

  if (depth_registration_ && align_mode_ == "SW") {
    ros::SubscriberStatusCallback depth_unaligned_subscribed_cb =
        boost::bind(&OBCameraNode::imageSubscribedCallback, this, DEPTH);
    ros::SubscriberStatusCallback depth_unaligned_unsubscribed_cb =
        boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, DEPTH);
    if (enable_image_transport_plugins_) {
      image_transport::SubscriberStatusCallback image_transport_subscribed_cb =
          boost::bind(&OBCameraNode::imageSubscribedCallback, this, DEPTH);
      image_transport::SubscriberStatusCallback image_transport_unsubscribed_cb =
          boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, DEPTH);
      depth_unaligned_publisher_ = image_transport::ImageTransport(nh_).advertise(
          "depth/image_unaligned", 1, image_transport_subscribed_cb,
          image_transport_unsubscribed_cb);
    } else {
      depth_unaligned_raw_publisher_ = nh_.advertise<sensor_msgs::Image>(
          "depth/image_unaligned", 1, depth_unaligned_subscribed_cb,
          depth_unaligned_unsubscribed_cb);
    }
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
  depth_filters_status_pub_ = nh_.advertise<orbbec_camera::DepthFiltersStatus>(
      "/" + camera_name_ + "/depth_filters/status", 1, true);
  publishDepthFiltersStatus();
  if (enable_enhanced_depth_.load()) {
    setupConfidencePublishers();
  }
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

// Global topic-based publisher cache to prevent plugin reloading
std::map<std::string, image_transport::Publisher> OBCameraNode::global_image_publishers_;
std::shared_ptr<image_transport::ImageTransport> OBCameraNode::global_image_transport_;
std::shared_ptr<ros::NodeHandle> OBCameraNode::global_nh_;
std::mutex OBCameraNode::global_publisher_mutex_;

image_transport::Publisher OBCameraNode::getGlobalImagePublisher(
    const std::string& topic_name, const image_transport::SubscriberStatusCallback& connect_cb,
    const image_transport::SubscriberStatusCallback& disconnect_cb) {
  std::lock_guard<std::mutex> lock(global_publisher_mutex_);

  // Initialize global image transport if needed
  if (!global_image_transport_) {
    if (!global_nh_) {
      global_nh_ = std::make_shared<ros::NodeHandle>();
    }
    global_image_transport_ = std::make_shared<image_transport::ImageTransport>(*global_nh_);
  }

  // Always recreate publisher with callbacks to ensure rostopic hz detection works
  auto it = global_image_publishers_.find(topic_name);
  if (it != global_image_publishers_.end()) {
    ROS_DEBUG_STREAM("Recreating image publisher for topic with callbacks: " << topic_name);
    it->second.shutdown();
    global_image_publishers_.erase(it);
  }

  // Create new publisher with callbacks for this topic and cache it
  image_transport::Publisher pub =
      global_image_transport_->advertise(topic_name, 1, connect_cb, disconnect_cb);
  global_image_publishers_[topic_name] = pub;

  ROS_DEBUG_STREAM("Created new image publisher with callbacks for topic: " << topic_name);
  return pub;
}

void OBCameraNode::releaseGlobalImagePublisher(const std::string& topic_name) {
  std::lock_guard<std::mutex> lock(global_publisher_mutex_);

  auto it = global_image_publishers_.find(topic_name);
  if (it != global_image_publishers_.end()) {
    it->second.shutdown();
    global_image_publishers_.erase(it);
    ROS_DEBUG_STREAM("Released image publisher for topic: " << topic_name);
  }
}

void OBCameraNode::initializeGlobalImageTransport() {
  // Note: This function should be called only when global_publisher_mutex_ is already locked

  if (!global_image_transport_) {
    if (!global_nh_) {
      global_nh_ = std::make_shared<ros::NodeHandle>();
    }
    global_image_transport_ = std::make_shared<image_transport::ImageTransport>(*global_nh_);
    ROS_INFO_STREAM(
        "Created persistent global image_transport instance to prevent plugin reloading");
  }
}

void OBCameraNode::forceCleanupGlobalResources() {
  std::lock_guard<std::mutex> lock(global_publisher_mutex_);

  // Force shutdown all publishers
  for (auto& pair : global_image_publishers_) {
    pair.second.shutdown();
  }
  global_image_publishers_.clear();

  global_image_transport_.reset();
  global_nh_.reset();
  ROS_INFO_STREAM("Force cleanup of global image_transport resources completed");
}

void OBCameraNode::setupCameraInfo() {
  if (!color_info_uri_.empty()) {
    color_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        nh_rgb_, camera_name_ + "_" + stream_name_[COLOR], color_info_uri_);
  }
  if (!ir_info_uri_.empty()) {
    ir_camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
        nh_ir_, camera_name_ + "_" + stream_name_[INFRA0], ir_info_uri_);
  }
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
  if (align_mode_ == "HW" && depth_registration_ && enable_stream_[COLOR] &&
      enable_stream_[DEPTH]) {
    if (!isHwD2CProfileSupported()) {
      std::stringstream ss;
      ss << "Selected profiles do not support HW D2C. color=" << width_[COLOR] << "x"
         << height_[COLOR] << "@" << fps_[COLOR] << ", depth=" << width_[DEPTH] << "x"
         << height_[DEPTH] << "@" << fps_[DEPTH]
         << ". Select a supported profile or use align_mode:=SW.";
      throw std::runtime_error(ss.str());
    }
    OBAlignMode align_mode = ALIGN_D2C_HW_MODE;
    ROS_INFO_STREAM("set align mode to " << align_mode_);
    pipeline_config_->setAlignMode(align_mode);
    pipeline_config_->setDepthScaleRequire(enable_depth_scale_);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      ROS_DEBUG_STREAM("Enable " << stream_name_[stream_index] << " stream");
      auto profile = stream_profile_[stream_index]->as<ob::VideoStreamProfile>();

      if (stream_index.first == align_target_stream_ && align_filter_) {
        auto video_profile = profile;
        align_filter_->setAlignToStreamProfile(video_profile);
      }
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

bool OBCameraNode::validateEnhancedDepthFilterConfig(std::string& message) const {
  if (!enable_stream_.count(COLOR) || !enable_stream_.at(COLOR) || !enable_stream_.count(DEPTH) ||
      !enable_stream_.at(DEPTH)) {
    message = "Enhanced depth filter requires color and depth streams";
    return false;
  }
  if (!depth_registration_) {
    message = "Enhanced depth filter requires D2C/C2D align mode";
    return false;
  }
  if (align_mode_ != "HW" && align_mode_ != "SW") {
    message = "Enhanced depth filter requires D2C/C2D align mode";
    return false;
  }
  if (align_mode_ == "HW" && align_target_stream_ != OB_STREAM_COLOR) {
    message = "Enhanced depth filter requires HW D2C align target COLOR";
    return false;
  }
  if (align_mode_ == "SW" && align_target_stream_ != OB_STREAM_COLOR &&
      align_target_stream_ != OB_STREAM_DEPTH) {
    message = "Enhanced depth filter requires D2C/C2D align mode";
    return false;
  }

  auto color_it = stream_profile_.find(COLOR);
  auto depth_it = stream_profile_.find(DEPTH);
  if (color_it == stream_profile_.end() || !color_it->second || depth_it == stream_profile_.end() ||
      !depth_it->second) {
    message = "Enhanced depth filter requires color and depth stream profiles";
    return false;
  }
  auto color_profile = color_it->second->as<ob::VideoStreamProfile>();
  auto depth_profile = depth_it->second->as<ob::VideoStreamProfile>();
  if (!color_profile || !depth_profile) {
    message = "Enhanced depth filter requires video stream profiles";
    return false;
  }

  const bool d2c = align_mode_ == "HW" || align_target_stream_ == OB_STREAM_COLOR;
  const OBStreamType align_to_stream = d2c ? OB_STREAM_COLOR : OB_STREAM_DEPTH;
  if (!ob::EnhancedDepthFilter::isSupportedResolution(color_profile->getType(), align_to_stream,
                                                      color_profile->getWidth(),
                                                      color_profile->getHeight())) {
    message = std::string("Enhanced depth filter requires supported target resolutions: ") +
              kEnhancedDepthSupportedTargetResolutions;
    return false;
  }
  if (!ob::EnhancedDepthFilter::isSupportedResolution(depth_profile->getType(), align_to_stream,
                                                      depth_profile->getWidth(),
                                                      depth_profile->getHeight()) ||
      !ob::EnhancedDepthFilter::isSupportedFormat(OB_STREAM_DEPTH, depth_profile->getFormat())) {
    message = std::string("Enhanced depth filter requires supported target resolutions: ") +
              kEnhancedDepthSupportedTargetResolutions +
              " and depth formats: " + kEnhancedDepthSupportedDepthFormats;
    return false;
  }
  if (!isEnhancedDepthColorFormatSupported(color_profile->getFormat())) {
    message =
        "Unsupported color stream format for enhanced depth filter. Supported formats are: YUYV "
        "UYVY MJPG BGR RGBA Y16 Y8 RGB";
    return false;
  }
  return true;
}

void OBCameraNode::applyEnhancedDepthConfidenceThreshold() {
  if (!enhanced_depth_filter_ || enhanced_depth_confidence_threshold_ < 0) {
    return;
  }
  auto range = enhanced_depth_filter_->getConfidenceThresholdRange();
  const int confidence_threshold = enhanced_depth_confidence_threshold_;
  if (confidence_threshold < range.min || confidence_threshold > range.max) {
    std::ostringstream ss;
    ss << "Enhanced depth confidence threshold is out of range " << range.min << " - " << range.max;
    throw std::runtime_error(ss.str());
  }
  enhanced_depth_filter_->setConfidenceThreshold(static_cast<uint32_t>(confidence_threshold));
}

bool OBCameraNode::ensureEnhancedDepthFilter(std::string& message) {
  if (!validateEnhancedDepthFilterConfig(message)) {
    return false;
  }

  std::lock_guard<std::mutex> lock(enhanced_depth_filter_mutex_);
  try {
    if (!enhanced_depth_filter_) {
      if (enhanced_depth_model_path_.empty()) {
        message = "Enhanced depth filter requires enhanced_depth_model_path";
        return false;
      }
      std::ifstream model_file(enhanced_depth_model_path_);
      if (!model_file.good()) {
        message = "Enhanced depth model file not found: " + enhanced_depth_model_path_;
        return false;
      }
      if (!device_->isLicenseAuthorizationSupported()) {
        message = "Enhanced depth filter requires device license authorization support";
        return false;
      }
      auto license_info = device_->readLicenseInfo();
      ROS_INFO_STREAM("Enhanced depth license info: " << license_info);
      if (license_info.empty()) {
        message = "Enhanced depth filter requires device license info";
        return false;
      }
      ROS_INFO_STREAM(
          "Creating enhanced depth filter with model path: " << enhanced_depth_model_path_);
      enhanced_depth_filter_ =
          std::make_shared<ob::EnhancedDepthFilter>(device_, enhanced_depth_model_path_);
    }
    const bool d2c = align_mode_ == "HW" || align_target_stream_ == OB_STREAM_COLOR;
    auto target_profile = d2c ? stream_profile_.at(COLOR)->as<ob::VideoStreamProfile>()
                              : stream_profile_.at(DEPTH)->as<ob::VideoStreamProfile>();
    enhanced_depth_filter_->setResolution(target_profile->getWidth(), target_profile->getHeight());
    applyEnhancedDepthConfidenceThreshold();
    setupConfidencePublishers();
  } catch (const ob::Error& e) {
    message =
        "Failed to create enhanced depth filter: " + orbbec_camera::formatObErrorWithStatus(e);
    return false;
  } catch (const std::exception& e) {
    message = "Failed to create enhanced depth filter: " + std::string(e.what());
    return false;
  }
  return true;
}

bool OBCameraNode::convertEnhancedDepthColorFrame(const std::shared_ptr<ob::FrameSet>& frame_set) {
  auto color_frame = frame_set ? frame_set->getFrame(OB_FRAME_COLOR) : nullptr;
  if (!color_frame) {
    return false;
  }
  const auto color_format = color_frame->format();
  if (color_format == OB_FORMAT_RGB) {
    return true;
  }
  const auto& format_map = enhancedDepthColorFormatMap();
  auto it = format_map.find(color_format);
  if (it == format_map.end()) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "Unsupported color stream format for enhanced depth filter: " << color_format);
    return false;
  }

  try {
    enhanced_depth_format_convert_filter_.setFormatConvertType(it->second);
    auto converted = enhanced_depth_format_convert_filter_.process(color_frame);
    if (!converted) {
      ROS_ERROR_THROTTLE(1.0, "Enhanced depth color format conversion failed");
      return false;
    }
    frame_set->pushFrame(converted);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Enhanced depth color format conversion failed: "
                                       << orbbec_camera::formatObErrorWithStatus(e));
    return false;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Enhanced depth color format conversion failed: " << e.what());
    return false;
  }
  return true;
}

std::shared_ptr<ob::FrameSet> OBCameraNode::processEnhancedDepthFilter(
    const std::shared_ptr<ob::FrameSet>& frame_set) {
  if (!frame_set) {
    return frame_set;
  }
  {
    std::lock_guard<std::mutex> lock(enhanced_depth_filter_mutex_);
    if (!enable_enhanced_depth_.load()) {
      return frame_set;
    }
  }

  std::string message;
  if (!ensureEnhancedDepthFilter(message)) {
    ROS_ERROR_STREAM_THROTTLE(1.0, message);
    return frame_set;
  }
  if (!frame_set->getFrame(OB_FRAME_COLOR) || !frame_set->getFrame(OB_FRAME_DEPTH)) {
    ROS_ERROR_THROTTLE(1.0, "Enhanced depth filter requires color and depth frames");
    return frame_set;
  }
  if (!convertEnhancedDepthColorFrame(frame_set)) {
    return frame_set;
  }

  std::shared_ptr<ob::EnhancedDepthFilter> filter;
  {
    std::lock_guard<std::mutex> lock(enhanced_depth_filter_mutex_);
    if (!enable_enhanced_depth_.load()) {
      return frame_set;
    }
    filter = enhanced_depth_filter_;
  }

  try {
    auto processed = filter->process(frame_set);
    if (!processed || !processed->is<ob::FrameSet>()) {
      ROS_ERROR_THROTTLE(1.0, "Enhanced depth filter returned invalid frameset");
      return frame_set;
    }
    auto processed_frame_set = processed->as<ob::FrameSet>();
    publishConfidenceFrame(processed_frame_set->getFrame(OB_FRAME_CONFIDENCE));
    return processed_frame_set;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "Enhanced depth filter failed: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM_THROTTLE(1.0, "Enhanced depth filter failed: " << e.what());
  }
  return frame_set;
}

void OBCameraNode::setupConfidencePublishers() {
  if (confidence_image_publisher_) {
    return;
  }
  confidence_image_publisher_ =
      nh_.advertise<sensor_msgs::Image>("/" + camera_name_ + "/confidence/image_raw", 1);
}

void OBCameraNode::publishConfidenceFrame(const std::shared_ptr<ob::Frame>& confidence_frame) {
  if (!confidence_frame || !confidence_frame->is<ob::VideoFrame>()) {
    return;
  }
  setupConfidencePublishers();
  if (!confidence_image_publisher_ || confidence_image_publisher_.getNumSubscribers() == 0) {
    return;
  }

  auto video_frame = confidence_frame->as<ob::VideoFrame>();
  const int width = static_cast<int>(video_frame->width());
  const int height = static_cast<int>(video_frame->height());
  int image_type = CV_8UC1;
  std::string encoding = sensor_msgs::image_encodings::MONO8;
  int unit_step_size = sizeof(uint8_t);
  if (confidence_frame->format() == OB_FORMAT_Y16) {
    image_type = CV_16UC1;
    encoding = sensor_msgs::image_encodings::MONO16;
    unit_step_size = sizeof(uint16_t);
  } else if (confidence_frame->format() != OB_FORMAT_Y8) {
    ROS_ERROR_STREAM_THROTTLE(
        1.0, "Unsupported confidence frame format: " << confidence_frame->format());
    return;
  }

  if (confidence_image_.empty() || confidence_image_.cols != width ||
      confidence_image_.rows != height || confidence_image_.type() != image_type) {
    confidence_image_.create(height, width, image_type);
  }
  memcpy(confidence_image_.data, video_frame->data(), video_frame->dataSize());

  auto timestamp = fromUsToROSTime(getFrameTimestampUs(confidence_frame));
  std::string frame_id =
      depth_registration_ ? depth_aligned_frame_id_[DEPTH] : optical_frame_id_[DEPTH];

  auto image_msg = cv_bridge::CvImage(std_msgs::Header(), encoding, confidence_image_).toImageMsg();
  image_msg->header.stamp = timestamp;
  image_msg->header.frame_id = frame_id;
  image_msg->is_bigendian = false;
  image_msg->step = width * unit_step_size;
  confidence_image_publisher_.publish(image_msg);
}

void OBCameraNode::diagnosticTemperature(diagnostic_updater::DiagnosticStatusWrapper& stat) {
  try {
    OBDeviceTemperature temperature{};
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
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR,
                 orbbec_camera::formatObErrorWithStatus(e));
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
      ROS_DEBUG_STREAM("stream " << stream_name_[stream_index] << " gain " << gain);
      default_gain_[stream_index] = gain;
    } catch (ob::Error& e) {
      default_gain_[stream_index] = 0;
      ROS_DEBUG_STREAM("get gain error " << orbbec_camera::formatObErrorWithStatus(e));
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
      ROS_DEBUG_STREAM("stream " << stream_name_[stream_index] << " exposure " << exposure
                                 << " auto exposure " << is_auto_exposure);
      ROS_DEBUG_STREAM("stream " << stream_name_[stream_index] << " exposure " << exposure);
      default_exposure_[stream_index] = exposure;
    } catch (ob::Error& e) {
      default_exposure_[stream_index] = 0;
      ROS_DEBUG_STREAM("get " << stream_name_[stream_index] << " exposure error "
                              << orbbec_camera::formatObErrorWithStatus(e));
    }
  }
}

void OBCameraNode::readDefaultWhiteBalance() {
  try {
    auto sensor = sensors_[COLOR];
    if (!sensor) {
      ROS_DEBUG_STREAM("does not have color sensor");
      return;
    }
    CHECK_NOTNULL(sensor.get());
    auto wb = sensor->getWhiteBalance();
    ROS_DEBUG_STREAM("stream " << stream_name_[COLOR] << " wb " << wb);
    default_white_balance_ = wb;
  } catch (ob::Error& e) {
    default_white_balance_ = 0;
    ROS_DEBUG_STREAM("get white balance error " << orbbec_camera::formatObErrorWithStatus(e));
  }
}

void OBCameraNode::setDisparitySearchOffset() {
  static bool has_run = false;
  if (has_run) {
    return;
  }
  if ((disparity_search_offset_ < 0 || disparity_search_offset_ > 127) &&
      (offset_index0_ < 0 || offset_index0_ > 127 || offset_index1_ < 0 || offset_index1_ > 127)) {
    has_run = true;
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
  if (depth_ae_roi_left_ == -1 && depth_ae_roi_top_ == -1 && depth_ae_roi_right_ == -1 &&
      depth_ae_roi_bottom_ == -1) {
    depth_roi_has_run = true;
    return;
  }
  if (isGemini305SeriesPID(device_->getDeviceInfo()->pid()) && ae_reference_stream_ == "color") {
    ROS_WARN_STREAM("Skip setting depth AE ROI because AE Reference Stream is color");
    depth_roi_has_run = true;
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
    ROS_INFO_STREAM("Set depth AE ROI to " << config.x0_left << ", " << config.y0_top << ", "
                                           << config.x1_right << ", " << config.y1_bottom);
  }
  depth_roi_has_run = true;
}

void OBCameraNode::setColorAutoExposureROI() {
  static bool color_roi_has_run = false;
  if (color_roi_has_run) {
    return;
  }
  if (color_ae_roi_left_ == -1 && color_ae_roi_top_ == -1 && color_ae_roi_right_ == -1 &&
      color_ae_roi_bottom_ == -1) {
    color_roi_has_run = true;
    return;
  }
  if (isGemini305SeriesPID(device_->getDeviceInfo()->pid()) && ae_reference_stream_ == "depth") {
    ROS_WARN_STREAM("Skip setting color AE ROI because AE Reference Stream is depth");
    color_roi_has_run = true;
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
    ROS_INFO_STREAM("Set color AE ROI to " << config.x0_left << ", " << config.y0_top << ", "
                                           << config.x1_right << ", " << config.y1_bottom);
  }
  color_roi_has_run = true;
}

void OBCameraNode::updateDepthFilterEnabledCache(const std::string& filter_name, bool enabled) {
  const auto normalized_filter_name = normalizeDepthFilterName(filter_name);
  if (normalized_filter_name == "DecimationFilter") {
    enable_decimation_filter_ = enabled;
  } else if (normalized_filter_name == "HDRMerge") {
    enable_hdr_merge_ = enabled;
  } else if (normalized_filter_name == "SequenceIdFilter") {
    enable_sequenced_filter_ = enabled;
  } else if (normalized_filter_name == "ThresholdFilter") {
    enable_threshold_filter_ = enabled;
  } else if (normalized_filter_name == "SpatialAdvancedFilter") {
    enable_spatial_filter_ = enabled;
  } else if (normalized_filter_name == "TemporalFilter") {
    enable_temporal_filter_ = enabled;
  } else if (normalized_filter_name == "HoleFillingFilter") {
    enable_hole_filling_filter_ = enabled;
  } else if (normalized_filter_name == "SpatialFastFilter") {
    enable_spatial_fast_filter_ = enabled;
  } else if (normalized_filter_name == "SpatialModerateFilter") {
    enable_spatial_moderate_filter_ = enabled;
  } else if (normalized_filter_name == "EdgeNoiseRemovalFilter") {
    enable_edge_noise_removal_filter_ = enabled;
  } else if (normalized_filter_name == "FalsePositiveFilter") {
    enable_false_positive_filter_ = enabled;
  } else if (normalized_filter_name == "MgcNoiseRemovalFilter") {
    enable_mgc_noise_removal_filter_ = enabled;
  } else if (normalized_filter_name == "LutNoiseRemovalFilter") {
    enable_lut_noise_removal_filter_ = enabled;
  } else if (normalized_filter_name == "DispOutliersFilter") {
    enable_disp_outliers_filter_ = enabled;
  } else if (normalized_filter_name == "NoiseRemovalFilter") {
    enable_noise_removal_filter_ = enabled;
  } else if (normalized_filter_name == "HardwareNoiseRemovalFilter") {
    enable_hardware_noise_removal_filter_ = enabled;
  }
}

bool OBCameraNode::applyNamedDepthFilterConfig(
    const std::string& filter_name, bool enabled,
    const std::vector<orbbec_camera::DepthFilterParam>& params, std::string& message) {
  const auto normalized_filter_name = normalizeDepthFilterName(filter_name);
  std::unordered_set<std::string> requested_param_names;
  auto check_duplicate_param = [&requested_param_names, &message](const std::string& param_name) {
    if (param_name.empty()) {
      message = "Filter config param name is empty";
      return false;
    }
    if (!requested_param_names.insert(param_name).second) {
      message = "Duplicate filter config param '" + param_name + "'";
      return false;
    }
    return true;
  };

  if (normalized_filter_name == "NoiseRemovalFilter") {
    const bool supported =
        device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE) ||
        device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE) ||
        device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE);
    if (!supported) {
      message = "Filter '" + normalized_filter_name + "' is not supported by this device";
      return false;
    }

    bool has_min_diff = false;
    bool has_max_size = false;
    int min_diff = 0;
    int max_size = 0;
    for (const auto& param : params) {
      const auto param_name = getDepthFilterConfigParamName(normalized_filter_name, param.name);
      if (!check_duplicate_param(param_name)) {
        return false;
      }

      double parsed_value = 0.0;
      if (!parseFilterConfigDouble(param.value, parsed_value, message)) {
        return false;
      }
      if (std::floor(parsed_value) != parsed_value) {
        message = "Filter config '" + param_name + "' expects an integer value";
        return false;
      }

      if (param_name == "min_diff") {
        if (!device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE)) {
          message = "Filter config 'min_diff' is not supported by this device";
          return false;
        }
        has_min_diff = true;
        min_diff = static_cast<int>(parsed_value);
      } else if (param_name == "max_size") {
        if (!device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT,
                                          OB_PERMISSION_WRITE)) {
          message = "Filter config 'max_size' is not supported by this device";
          return false;
        }
        has_max_size = true;
        max_size = static_cast<int>(parsed_value);
      } else {
        message = "Unknown filter config '" + param.name + "' for " + normalized_filter_name;
        return false;
      }
    }

    if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enabled);
    }
    if (has_min_diff) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, min_diff);
      noise_removal_filter_min_diff_ = min_diff;
    }
    if (has_max_size) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, max_size);
      noise_removal_filter_max_size_ = max_size;
    }
    updateDepthFilterEnabledCache(normalized_filter_name, enabled);
    return true;
  }

  if (normalized_filter_name == "HardwareNoiseRemovalFilter") {
    const bool supported =
        device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                     OB_PERMISSION_READ_WRITE) ||
        device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                     OB_PERMISSION_READ_WRITE);
    if (!supported) {
      message = "Filter '" + normalized_filter_name + "' is not supported by this device";
      return false;
    }

    bool has_threshold = false;
    double threshold = 0.0;
    for (const auto& param : params) {
      const auto param_name = getDepthFilterConfigParamName(normalized_filter_name, param.name);
      if (!check_duplicate_param(param_name)) {
        return false;
      }
      if (param_name != "threshold") {
        message = "Unknown filter config '" + param.name + "' for " + normalized_filter_name;
        return false;
      }
      if (!device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                        OB_PERMISSION_READ_WRITE)) {
        message = "Filter config 'threshold' is not supported by this device";
        return false;
      }
      if (!parseFilterConfigDouble(param.value, threshold, message)) {
        return false;
      }
      has_threshold = true;
    }

    if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL, enabled);
    }
    if (has_threshold) {
      device_->setFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                static_cast<float>(threshold));
      hardware_noise_removal_filter_threshold_ = static_cast<float>(threshold);
    }
    updateDepthFilterEnabledCache(normalized_filter_name, enabled);
    return true;
  }

  if (normalized_filter_name == "DispOutliersFilter") {
    const bool supported =
        device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL,
                                     OB_PERMISSION_READ_WRITE) ||
        device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                     OB_PERMISSION_READ_WRITE);
    if (!supported) {
      message = "Filter '" + normalized_filter_name + "' is not supported by this device";
      return false;
    }

    bool has_search_mode = false;
    int search_mode = 0;
    for (const auto& param : params) {
      const auto param_name = getDepthFilterConfigParamName(normalized_filter_name, param.name);
      if (!check_duplicate_param(param_name)) {
        return false;
      }
      if (param_name != "search_mode") {
        message = "Unknown filter config '" + param.name + "' for " + normalized_filter_name;
        return false;
      }
      if (!device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                        OB_PERMISSION_READ_WRITE)) {
        message = "Filter config 'search_mode' is not supported by this device";
        return false;
      }

      if (!parseDispOutliersSearchMode(param.value, search_mode, message)) {
        return false;
      }
      has_search_mode = true;
    }

    if (device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL,
                                     OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL, enabled);
    }
    if (has_search_mode) {
      device_->setIntProperty(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT, search_mode);
      disp_outliers_filter_search_mode_ = search_mode;
    }
    updateDepthFilterEnabledCache(normalized_filter_name, enabled);
    return true;
  }

  std::unique_lock<std::mutex> depth_filter_lock(depth_filter_mutex_);
  auto is_same_filter = [&normalized_filter_name](const std::shared_ptr<ob::Filter>& filter) {
    if (!filter) {
      return false;
    }
    return normalizeDepthFilterName(filter->getName()) == normalized_filter_name ||
           normalizeDepthFilterName(filter->type()) == normalized_filter_name;
  };

  auto first_match_it =
      std::find_if(depth_filter_list_.begin(), depth_filter_list_.end(),
                   [&is_same_filter](const auto& filter) { return is_same_filter(filter); });
  if (first_match_it == depth_filter_list_.end() || !(*first_match_it)) {
    message = "Filter '" + normalized_filter_name + "' is not supported by this device";
    return false;
  }

  const auto& existing_filter = *first_match_it;
  std::vector<OBFilterConfigSchemaItem> schema_vec;
  std::unordered_map<std::string, OBFilterConfigSchemaItem> schema_by_name;
  if (!params.empty()) {
    schema_vec = existing_filter->getConfigSchemaVec();
    for (const auto& schema : schema_vec) {
      if (schema.name == nullptr || schema.name[0] == '\0') {
        continue;
      }
      schema_by_name.emplace(schema.name, schema);
    }
  }

  std::vector<std::pair<std::string, double>> parsed_params;
  parsed_params.reserve(params.size());
  for (const auto& param : params) {
    const auto param_name = getDepthFilterConfigParamName(normalized_filter_name, param.name);
    if (!check_duplicate_param(param_name)) {
      return false;
    }

    const auto schema_it = schema_by_name.find(param_name);
    if (schema_it == schema_by_name.end()) {
      message = "Unknown filter config '" + param.name + "' for " + normalized_filter_name;
      return false;
    }

    double parsed_value = 0.0;
    if (!parseFilterConfigValue(schema_it->second, param.value, parsed_value, message)) {
      return false;
    }
    parsed_params.emplace_back(param_name, parsed_value);
  }

  existing_filter->enable(enabled);
  for (const auto& parsed_param : parsed_params) {
    existing_filter->setConfigValue(parsed_param.first, parsed_param.second);
    ROS_INFO_STREAM("Set " << normalized_filter_name << " config " << parsed_param.first << " to "
                           << parsed_param.second);
  }
  updateDepthFilterEnabledCache(normalized_filter_name, enabled);
  return true;
}

bool OBCameraNode::applyEnhancedDepthFilterConfig(
    bool enabled, const std::vector<float>& positional_params,
    const std::vector<orbbec_camera::DepthFilterParam>& named_params, std::string& message) {
  if (positional_params.size() > 1) {
    message = "EnhancedDepthFilter only supports one positional parameter";
    return false;
  }
  if (!positional_params.empty() && !named_params.empty()) {
    message = "filter_param and filter_config cannot be used at the same time";
    return false;
  }

  bool has_confidence_threshold = false;
  int confidence_threshold = enhanced_depth_confidence_threshold_;
  auto parse_confidence_threshold = [&message](double value, int& threshold) {
    if (std::floor(value) != value || value < 0.0 || value > 255.0) {
      message =
          "EnhancedDepthFilter confidence_threshold expects an integer value in range 0 - 255";
      return false;
    }
    threshold = static_cast<int>(value);
    return true;
  };
  if (!positional_params.empty()) {
    if (!parse_confidence_threshold(positional_params[0], confidence_threshold)) {
      return false;
    }
    has_confidence_threshold = true;
  }
  for (const auto& param : named_params) {
    if (param.name != "confidence_threshold") {
      message = "Unknown filter config '" + param.name + "' for EnhancedDepthFilter";
      return false;
    }
    double parsed_value = 0.0;
    if (!parseFilterConfigDouble(param.value, parsed_value, message)) {
      return false;
    }
    if (!parse_confidence_threshold(parsed_value, confidence_threshold)) {
      return false;
    }
    has_confidence_threshold = true;
  }

  std::string validate_message;
  if (enabled && !validateEnhancedDepthFilterConfig(validate_message)) {
    message = validate_message;
    return false;
  }

  const int previous_threshold = enhanced_depth_confidence_threshold_;
  if (has_confidence_threshold) {
    enhanced_depth_confidence_threshold_ = confidence_threshold;
  }

  if (enabled) {
    if (!ensureEnhancedDepthFilter(message)) {
      enhanced_depth_confidence_threshold_ = previous_threshold;
      return false;
    }
  } else if (has_confidence_threshold && enhanced_depth_filter_) {
    try {
      applyEnhancedDepthConfidenceThreshold();
    } catch (const std::exception& e) {
      enhanced_depth_confidence_threshold_ = previous_threshold;
      message = e.what();
      return false;
    }
  }

  {
    std::lock_guard<std::mutex> lock(enhanced_depth_filter_mutex_);
    enable_enhanced_depth_.store(enabled);
  }
  filter_status_["EnhancedDepthFilter"] = enabled;
  publishDepthFiltersStatus();
  return true;
}

bool OBCameraNode::setFilterCallback(SetFilterRequest& request, SetFilterResponse& response) {
  try {
    response.success = false;
    response.message.clear();
    auto fail = [&response](const std::string& msg) {
      response.success = false;
      response.message = msg;
      return true;
    };

    const auto normalized_request_filter_name = normalizeDepthFilterName(request.filter_name);
    const bool is_noise_removal_filter = (normalized_request_filter_name == "NoiseRemovalFilter");
    const bool is_hardware_noise_removal_filter =
        (normalized_request_filter_name == "HardwareNoiseRemovalFilter");
    const bool is_disp_outliers_filter = (normalized_request_filter_name == "DispOutliersFilter");
    bool is_supported_by_property = false;
    if (is_noise_removal_filter) {
      is_supported_by_property =
          device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL, OB_PERMISSION_READ_WRITE) ||
          device_->isPropertySupported(OB_PROP_DEPTH_MAX_DIFF_INT, OB_PERMISSION_WRITE) ||
          device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, OB_PERMISSION_WRITE);
    } else if (is_hardware_noise_removal_filter) {
      is_supported_by_property =
          device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                       OB_PERMISSION_READ_WRITE) ||
          device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                       OB_PERMISSION_READ_WRITE);
    } else if (is_disp_outliers_filter) {
      is_supported_by_property =
          device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_BOOL,
                                       OB_PERMISSION_READ_WRITE) ||
          device_->isPropertySupported(OB_PROP_DEPTH_OUTLIERS_FILTER_SEARCH_MODE_INT,
                                       OB_PERMISSION_READ_WRITE);
    }

    ROS_INFO_STREAM("filter_name: " << request.filter_name << "  filter_enable: "
                                    << (request.filter_enable ? "true" : "false"));
    const bool has_positional_params = !request.filter_param.empty();
    const bool has_named_params = !request.filter_config.empty();
    if (has_positional_params && has_named_params) {
      return fail("filter_param and filter_config cannot be used at the same time");
    }
    if (is_disp_outliers_filter && has_positional_params) {
      return fail("DispOutliersFilter search_mode expects filter_config value FULL or OFFSET_80");
    }

    if (normalized_request_filter_name == "EnhancedDepthFilter") {
      std::string message;
      if (!applyEnhancedDepthFilterConfig(request.filter_enable, request.filter_param,
                                          request.filter_config, message)) {
        return fail(message);
      }
      response.success = true;
      return true;
    }

    if (has_named_params || !has_positional_params) {
      std::string message;
      if (!applyNamedDepthFilterConfig(normalized_request_filter_name, request.filter_enable,
                                       request.filter_config, message)) {
        return fail(message);
      }

      filter_status_[normalized_request_filter_name] = static_cast<bool>(request.filter_enable);
      publishDepthFiltersStatus();

      response.success = true;
      return true;
    }

    if (is_noise_removal_filter || is_hardware_noise_removal_filter || is_disp_outliers_filter) {
      if (!is_supported_by_property) {
        return fail("Filter '" + normalized_request_filter_name +
                    "' is not supported by this device");
      }
      if (is_noise_removal_filter) {
        if (device_->isPropertySupported(OB_PROP_DEPTH_SOFT_FILTER_BOOL,
                                         OB_PERMISSION_READ_WRITE)) {
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
            noise_removal_filter_min_diff_ = request.filter_param[0];
          }
          if (device_->isPropertySupported(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT,
                                           OB_PERMISSION_WRITE)) {
            auto default_noise_removal_filter_max_size =
                device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
            ROS_INFO_STREAM(
                "default_noise_removal_filter_max_size: " << default_noise_removal_filter_max_size);
            device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, request.filter_param[1]);
            auto new_noise_removal_filter_max_size =
                device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
            ROS_INFO_STREAM(
                "after set noise_removal_filter_max_size: " << new_noise_removal_filter_max_size);
            noise_removal_filter_max_size_ = request.filter_param[1];
          }
        }
        enable_noise_removal_filter_ = request.filter_enable;
      } else if (is_hardware_noise_removal_filter) {
        if (device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                         OB_PERMISSION_READ_WRITE)) {
          device_->setBoolProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_ENABLE_BOOL,
                                   request.filter_enable);
          ROS_INFO_STREAM("Setting hardware_noise_removal_filter:" << request.filter_enable);
          if (request.filter_param.size() > 0 &&
              device_->isPropertySupported(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                           OB_PERMISSION_READ_WRITE)) {
            if (request.filter_enable) {
              device_->setFloatProperty(OB_PROP_HW_NOISE_REMOVE_FILTER_THRESHOLD_FLOAT,
                                        request.filter_param[0]);
              ROS_INFO_STREAM(
                  "Setting hardware_noise_removal_filter_threshold :" << request.filter_param[0]);
              hardware_noise_removal_filter_threshold_ = request.filter_param[0];
            }
          } else {
            return fail(
                "The filter switch setting is successful, but the filter parameter setting fails");
          }
        }
        enable_hardware_noise_removal_filter_ = request.filter_enable;
      }
    } else {
      std::unique_lock<std::mutex> depth_filter_lock(depth_filter_mutex_);
      auto is_same_filter =
          [&normalized_request_filter_name](const std::shared_ptr<ob::Filter>& filter) {
            if (!filter) {
              return false;
            }
            return normalizeDepthFilterName(filter->getName()) == normalized_request_filter_name ||
                   normalizeDepthFilterName(filter->type()) == normalized_request_filter_name;
          };

      auto first_match_it =
          std::find_if(depth_filter_list_.begin(), depth_filter_list_.end(),
                       [&is_same_filter](const auto& filter) { return is_same_filter(filter); });
      if (first_match_it == depth_filter_list_.end()) {
        return fail("Filter '" + normalized_request_filter_name +
                    "' is not supported by this device");
      }
      const auto& existing_filter = *first_match_it;
      if (!existing_filter) {
        return fail("Filter '" + normalized_request_filter_name +
                    "' is not supported by this device");
      }

      if (normalized_request_filter_name == "DecimationFilter") {
        auto decimation_filter = existing_filter->as<ob::DecimationFilter>();
        decimation_filter->enable(request.filter_enable);
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
            return fail("Decimation filter scale value is out of range");
          }
          if (decimation_filter_scale <= range.max && decimation_filter_scale >= range.min) {
            decimation_filter_scale_range_ = decimation_filter_scale;
          }
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_decimation_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "HDRMerge") {
        auto hdr_merge_filter = existing_filter->as<ob::HdrMerge>();
        hdr_merge_filter->enable(request.filter_enable);
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
          hdr_merge_exposure_1_ = request.filter_param[0];
          hdr_merge_gain_1_ = request.filter_param[1];
          hdr_merge_exposure_2_ = request.filter_param[2];
          hdr_merge_gain_2_ = request.filter_param[3];
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_hdr_merge_ = request.filter_enable;
      } else if (normalized_request_filter_name == "SequenceIdFilter") {
        auto sequenced_filter = existing_filter->as<ob::SequenceIdFilter>();
        sequenced_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 0) {
          sequenced_filter->selectSequenceId(request.filter_param[0]);
          ROS_INFO_STREAM("Set sequenced filter selectSequenceId value to "
                          << request.filter_param[0]);
          sequence_id_filter_id_ = request.filter_param[0];
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_sequenced_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "ThresholdFilter") {
        auto threshold_filter = existing_filter->as<ob::ThresholdFilter>();
        threshold_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 1) {
          auto threshold_filter_min = request.filter_param[0];
          auto threshold_filter_max = request.filter_param[1];
          threshold_filter->setValueRange(threshold_filter_min, threshold_filter_max);
          ROS_INFO_STREAM("Set threshold filter value range to " << threshold_filter_min << " - "
                                                                 << threshold_filter_max);
          threshold_filter_min_ = threshold_filter_min;
          threshold_filter_max_ = threshold_filter_max;
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_threshold_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "SpatialAdvancedFilter") {
        auto spatial_filter = existing_filter->as<ob::SpatialAdvancedFilter>();
        spatial_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 3) {
          OBSpatialAdvancedFilterParams params{};
          params.alpha = request.filter_param[0];
          params.disp_diff = request.filter_param[1];
          params.magnitude = request.filter_param[2];
          params.radius = request.filter_param[3];
          spatial_filter->setFilterParams(params);
          ROS_INFO_STREAM("Set SpatialFilter params: "
                          << "\nalpha:" << params.alpha << "\ndisp_diff:" << params.disp_diff
                          << "\nmagnitude:" << static_cast<int>(params.magnitude)
                          << "\nradius:" << params.radius);
          spatial_filter_alpha_ = params.alpha;
          spatial_filter_diff_threshold_ = params.disp_diff;
          spatial_filter_magnitude_ = params.magnitude;
          spatial_filter_radius_ = params.radius;
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_spatial_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "TemporalFilter") {
        auto temporal_filter = existing_filter->as<ob::TemporalFilter>();
        temporal_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 1) {
          temporal_filter->setDiffScale(request.filter_param[0]);
          temporal_filter->setWeight(request.filter_param[1]);
          ROS_INFO_STREAM("Set temporal filter value to " << request.filter_param[0] << " - "
                                                          << request.filter_param[1]);
          temporal_filter_diff_threshold_ = request.filter_param[0];
          temporal_filter_weight_ = request.filter_param[1];
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_temporal_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "SpatialFastFilter") {
        auto spatial_fast_filter = existing_filter->as<ob::SpatialFastFilter>();
        spatial_fast_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 0) {
          OBSpatialFastFilterParams params{};
          params.radius = request.filter_param[0];
          spatial_fast_filter->setFilterParams(params);
          ROS_INFO_STREAM("Set SpatialFastFilter radius to " << static_cast<int>(params.radius));
          spatial_fast_filter_radius_ = params.radius;
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_spatial_fast_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "SpatialModerateFilter") {
        auto spatial_moderate_filter = existing_filter->as<ob::SpatialModerateFilter>();
        spatial_moderate_filter->enable(request.filter_enable);
        if (request.filter_param.size() > 2) {
          OBSpatialModerateFilterParams params{};
          params.disp_diff = request.filter_param[0];
          params.magnitude = request.filter_param[1];
          params.radius = request.filter_param[2];
          spatial_moderate_filter->setFilterParams(params);
          ROS_INFO_STREAM("Set SpatialModerateFilter params: "
                          << "\ndisp_diff:" << params.disp_diff
                          << "\nmagnitude:" << static_cast<int>(params.magnitude)
                          << "\nradius:" << static_cast<int>(params.radius));
          spatial_moderate_filter_diff_threshold_ = params.disp_diff;
          spatial_moderate_filter_magnitude_ = params.magnitude;
          spatial_moderate_filter_radius_ = params.radius;
        } else {
          return fail(
              "The filter switch setting is successful, but the filter parameter setting fails");
        }
        enable_spatial_moderate_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "FalsePositiveFilter") {
        auto false_positive_filter = existing_filter->as<ob::FalsePositiveFilter>();
        false_positive_filter->enable(request.filter_enable);
        enable_false_positive_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "MgcNoiseRemovalFilter") {
        auto mgc_noise_filter = existing_filter->as<ob::MgcNoiseRemovalFilter>();
        mgc_noise_filter->enable(request.filter_enable);
        enable_mgc_noise_removal_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "LutNoiseRemovalFilter") {
        auto lut_noise_filter = existing_filter->as<ob::LutNoiseRemovalFilter>();
        lut_noise_filter->enable(request.filter_enable);
        enable_lut_noise_removal_filter_ = request.filter_enable;
      } else if (normalized_request_filter_name == "EdgeNoiseRemovalFilter") {
        existing_filter->enable(request.filter_enable);
        enable_edge_noise_removal_filter_ = request.filter_enable;
      } else {
        return fail(normalized_request_filter_name + " cannot be set");
      }
    }

    filter_status_[normalized_request_filter_name] = static_cast<bool>(request.filter_enable);
    publishDepthFiltersStatus();

    response.success = true;
    return true;
  } catch (const ob::Error& e) {
    response.success = false;
    response.message = "Failed to set filter: " + orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM("Failed to set filter: " << orbbec_camera::formatObErrorWithStatus(e));
    return false;
  } catch (const std::exception& e) {
    response.success = false;
    response.message = std::string("Failed to set filter: ") + e.what();
    ROS_ERROR_STREAM("Failed to set filter: " << e.what());
    return false;
  } catch (...) {
    response.success = false;
    response.message = "unknown error";
    ROS_ERROR_STREAM("unknown error");
    return false;
  }
}
}  // namespace orbbec_camera
