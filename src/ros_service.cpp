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

namespace orbbec_camera {

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::SetBool;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " is disabled.");
      continue;
    }
    auto stream_name = stream_name_[stream_index];
    std::string service_name = "/" + camera_name_ + "/" + "get_" + stream_name + "_exposure";
    get_exposure_srv_[stream_index] = nh_.advertiseService<GetInt32Request, GetInt32Response>(
        service_name, [this, stream_index](GetInt32Request& request, GetInt32Response& response) {
          response.success = this->getExposureCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_exposure";
    set_exposure_srv_[stream_index] = nh_.advertiseService<SetInt32Request, SetInt32Response>(
        service_name, [this, stream_index](SetInt32Request& request, SetInt32Response& response) {
          response.success = this->setExposureCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "/" + camera_name_ + "/" + "reset_" + stream_name + "_exposure";
    reset_exposure_srv_[stream_index] =
        nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
            service_name, [this, stream_index](std_srvs::EmptyRequest& request,
                                               std_srvs::EmptyResponse& response) {
              return this->resetCameraExposureCallback(request, response, stream_index);
            });
    service_name = "/" + camera_name_ + "/" + "get_" + stream_name + "_gain";
    get_gain_srv_[stream_index] = nh_.advertiseService<GetInt32Request, GetInt32Response>(
        service_name, [this, stream_index](GetInt32Request& request, GetInt32Response& response) {
          response.success = this->getGainCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_gain";
    set_gain_srv_[stream_index] = nh_.advertiseService<SetInt32Request, SetInt32Response>(
        service_name, [this, stream_index](SetInt32Request& request, SetInt32Response& response) {
          response.success = this->setGainCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "/" + camera_name_ + "/" + "reset_" + stream_name + "_gain";
    reset_gain_srv_[stream_index] =
        nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
            service_name, [this, stream_index](std_srvs::EmptyRequest& request,
                                               std_srvs::EmptyResponse& response) {
              return this->resetCameraGainCallback(request, response, stream_index);
            });
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_mirror";
    set_mirror_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->setMirrorCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_auto_exposure";
    set_auto_exposure_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->setAutoExposureCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "/" + camera_name_ + "/" + "get_" + stream_name + "_auto_exposure";
    get_auto_exposure_srv_[stream_index] = nh_.advertiseService<GetBoolRequest, GetBoolResponse>(
        service_name, [this, stream_index](GetBoolRequest& request, GetBoolResponse& response) {
          response.success = this->getAutoExposureCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "/" + camera_name_ + "/" + "toggle_" + stream_name;
    toggle_sensor_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->toggleSensorCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "/" + camera_name_ + "/" + "get_" + stream_name + "_camera_info";
    get_camera_info_srv_[stream_index] =
        nh_.advertiseService<GetCameraInfoRequest, GetCameraInfoResponse>(
            service_name,
            [this, stream_index](GetCameraInfoRequest& request, GetCameraInfoResponse& response) {
              response.success = this->getCameraInfoCallback(request, response, stream_index);
              return response.success;
            });
  }
  get_auto_white_balance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "/" + camera_name_ + "/" + "get_auto_white_balance",
      [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getAutoWhiteBalanceCallback(request, response);
        return response.success;
      });
  set_auto_white_balance_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "set_auto_white_balance",
      [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->setAutoWhiteBalanceCallback(request, response);
        return response.success;
      });
  get_white_balance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "/" + camera_name_ + "/" + "get_white_balance",
      [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getWhiteBalanceCallback(request, response);
        return response.success;
      });
  set_white_balance_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "set_white_balance",
      [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->setWhiteBalanceCallback(request, response);
        return response.success;
      });
  reset_white_balance_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "/" + camera_name_ + "/" + "reset_white_balance",
      [this](std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
        return this->resetCameraWhiteBalanceCallback(request, response);
      });
  set_fan_work_mode_srv_ =
      nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
          "/" + camera_name_ + "/" + "set_fan_work_mode",
          [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
            response.success = this->setFanWorkModeCallback(request, response);
            return response.success;
          });
  set_flood_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "/" + camera_name_ + "/" + "set_flood",
      [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setFloodCallback(request, response);
        return response.success;
      });
  set_laser_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "/" + camera_name_ + "/" + "set_laser",
      [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setLaserCallback(request, response);
        return response.success;
      });
  set_ldp_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "/" + camera_name_ + "/" + "set_ldp",
      [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setLdpEnableCallback(request, response);
        return response.success;
      });
  get_ldp_status_srv_ = nh_.advertiseService<GetBoolRequest, GetBoolResponse>(
      "/" + camera_name_ + "/" + "get_ldp_status",
      [this](GetBoolRequest& request, GetBoolResponse& response) {
        response.success = this->getLdpStatusCallback(request, response);
        return response.success;
      });
  get_device_info_srv_ = nh_.advertiseService<GetDeviceInfoRequest, GetDeviceInfoResponse>(
      "/" + camera_name_ + "/" + "get_device_info",
      [this](GetDeviceInfoRequest& request, GetDeviceInfoResponse& response) {
        response.success = this->getDeviceInfoCallback(request, response);
        return response.success;
      });
  get_serial_number_srv_ = nh_.advertiseService<GetStringRequest, GetStringResponse>(
      "/" + camera_name_ + "/" + "get_serial",
      [this](GetStringRequest& request, GetStringResponse& response) {
        response.success = this->getSerialNumberCallback(request, response);
        return response.success;
      });
  get_camera_params_srv_ = nh_.advertiseService<GetCameraParamsRequest, GetCameraParamsResponse>(
      "/" + camera_name_ + "/" + "get_camera_params",
      [this](GetCameraParamsRequest& request, GetCameraParamsResponse& response) {
        response.success = this->getCameraParamsCallback(request, response);
        return response.success;
      });

  get_sdk_version_srv_ = nh_.advertiseService<GetStringRequest, GetStringResponse>(
      "/" + camera_name_ + "/" + "get_sdk_version",
      [this](GetStringRequest& request, GetStringResponse& response) {
        response.success = this->getSDKVersionCallback(request, response);
        return response.success;
      });
  get_device_type_srv_ = nh_.advertiseService<GetStringRequest, GetStringResponse>(
      "/" + camera_name_ + "/" + "get_device_type",
      [this](GetStringRequest& request, GetStringResponse& response) {
        response.success = this->getDeviceTypeCallback(request, response);
        return response.success;
      });
  save_point_cloud_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "/" + camera_name_ + "/" + "save_point_cloud",
      [this](std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
        return this->savePointCloudCallback(request, response);
      });
  save_images_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "/" + camera_name_ + "/" + "save_images",
      [this](std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
        return this->saveImagesCallback(request, response);
      });
  switch_ir_mode_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "switch_ir_mode",
      [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->switchIRModeCallback(request, response);
        return response.success;
      });
  switch_ir_data_source_channel_srv_ = nh_.advertiseService<SetStringRequest, SetStringResponse>(
      "/" + camera_name_ + "/" + "switch_ir",
      [this](SetStringRequest& request, SetStringResponse& response) {
        response.success = this->switchIRDataSourceChannelCallback(request, response);
        return response.success;
      });
  get_ldp_measure_distance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "/" + camera_name_ + "/" + "get_ldp_measure_distance",
      [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getLdpMeasureDistanceCallback(request, response);
        return response.success;
      });
}

bool OBCameraNode::setMirrorCallback(std_srvs::SetBoolRequest& request,
                                     std_srvs::SetBoolResponse& response,
                                     const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return response.success;
  }
  auto sensor = sensors_[stream_index];
  try {
    sensor->setMirror(request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set mirror mode: " << e.getMessage());
    response.success = false;
    return response.success;
  }
  return true;
}

bool OBCameraNode::getExposureCallback(GetInt32Request& request, GetInt32Response& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    response.data = sensor->getExposure();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get exposure: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setExposureCallback(SetInt32Request& request, SetInt32Response& response,
                                       const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    auto range = sensor->getExposureRange();
    if (request.data < range.min || request.data > range.max) {
      ROS_ERROR_STREAM("Exposure value " << request.data << " out of range" << range.min << " - "
                                         << range.max);
      response.success = false;
      return false;
    }
    sensor->setExposure(request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set exposure: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::getGainCallback(GetInt32Request& request, GetInt32Response& response,
                                   const stream_index_pair& stream_index) {
  (void)request;
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    response.data = sensor->getGain();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get gain: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setGainCallback(SetInt32Request& request, SetInt32Response& response,
                                   const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    auto range = sensor->getGainRange();
    if (request.data < range.min || request.data > range.max) {
      ROS_ERROR_STREAM("Gain value " << request.data << " out of range" << range.min << " - "
                                     << range.max);
      response.success = false;
      return false;
    }
    sensor->setGain(request.data);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    auto gain = sensor->getGain();
    ROS_INFO_STREAM("After set gain: " << gain);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set gain: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::getAutoWhiteBalanceCallback(GetInt32Request& request,
                                               GetInt32Response& response) {
  (void)request;
  if (!enable_stream_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  try {
    response.data = sensor->getAutoWhiteBalance();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get auto white balance: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setAutoWhiteBalanceCallback(SetInt32Request& request,
                                               SetInt32Response& response) {
  if (!enable_stream_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  try {
    auto result = sensor->getAutoWhiteBalance();
    ROS_INFO_STREAM("Current auto white balance: " << result);
    sensor->setAutoWhiteBalance(request.data);
    ROS_INFO_STREAM("Set auto white balance to: " << request.data);
    result = sensor->getAutoWhiteBalance();
    ROS_INFO_STREAM("After set auto white balance: " << result);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set auto white balance: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::getWhiteBalanceCallback(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  if (!enable_stream_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  try {
    response.data = sensor->getWhiteBalance();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get white balance: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setWhiteBalanceCallback(SetInt32Request& request, SetInt32Response& response) {
  if (!enable_stream_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  try {
    auto range = sensor->getWhiteBalanceRange();
    if (request.data < range.min || request.data > range.max) {
      ROS_ERROR_STREAM("White balance value " << request.data << " out of range" << range.min
                                              << " - " << range.max);
      response.success = false;
      return false;
    }
    bool is_auto_white_balance = sensor->getAutoWhiteBalance();
    if (is_auto_white_balance) {
      ROS_ERROR_STREAM("Auto white balance is enabled, please disable it first.");
      response.success = false;
      return false;
    }
    sensor->setWhiteBalance(request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set white balance: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setAutoExposureCallback(std_srvs::SetBoolRequest& request,
                                           std_srvs::SetBoolResponse& response,
                                           const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    sensor->setAutoExposure(request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set auto exposure: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::getAutoExposureCallback(GetBoolRequest& request, GetBoolResponse& response,
                                           const stream_index_pair& stream_index) {
  (void)request;
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  try {
    response.data = sensor->getAutoExposure();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get auto exposure: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setLaserCallback(std_srvs::SetBoolRequest& request,
                                    std_srvs::SetBoolResponse& response) {
  (void)response;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    int data = request.data ? 1 : 0;
    device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set laser: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::setLdpEnableCallback(std_srvs::SetBoolRequest& request,
                                        std_srvs::SetBoolResponse& response) {
  (void)response;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    device_->setBoolProperty(OB_PROP_LDP_BOOL, request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set LDP: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::getLdpStatusCallback(GetBoolRequest& request, GetBoolResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    response.data = device_->getBoolProperty(OB_PROP_LDP_BOOL);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get LDP status: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setFanWorkModeCallback(std_srvs::SetBoolRequest& request,
                                          std_srvs::SetBoolResponse& response) {
  (void)response;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    device_->setBoolProperty(OB_PROP_FAN_WORK_MODE_INT, request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("set fan failed: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::setFloodCallback(std_srvs::SetBoolRequest& request,
                                    std_srvs::SetBoolResponse& response) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    device_->setBoolProperty(OB_PROP_FLOOD_BOOL, request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("set flood failed: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::getDeviceInfoCallback(GetDeviceInfoRequest& request,
                                         GetDeviceInfoResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  auto device_info = device_->getDeviceInfo();
  response.info.name = device_info->name();
  response.info.serial_number = device_info->serialNumber();
  response.info.firmware_version = device_info->firmwareVersion();
  response.info.supported_min_sdk_version = device_info->supportedMinSdkVersion();
  response.success = true;
  return true;
}

bool OBCameraNode::getSDKVersionCallback(GetStringRequest& request, GetStringResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  auto device_info = device_->getDeviceInfo();
  nlohmann::json data;
  data["firmware_version"] = device_info->firmwareVersion();
  data["supported_min_sdk_version"] = device_info->supportedMinSdkVersion();
  data["ros_sdk_version"] = OB_ROS_VERSION_STR;
  std::string major = std::to_string(ob::Version::getMajor());
  std::string minor = std::to_string(ob::Version::getMinor());
  std::string patch = std::to_string(ob::Version::getPatch());
  std::string version = major + "." + minor + "." + patch;
  data["ob_sdk_version"] = version;
  response.data = data.dump(2);
  response.success = true;
  return true;
}

bool OBCameraNode::toggleSensorCallback(std_srvs::SetBoolRequest& request,
                                        std_srvs::SetBoolResponse& response,
                                        const stream_index_pair& stream_index) {
  std::string msg;
  if (request.data) {
    if (enable_stream_[stream_index]) {
      msg = stream_name_[stream_index] + " Already ON";
    }
    ROS_INFO_STREAM("toggling sensor " << stream_name_[stream_index] << " ON");
  } else {
    if (!enable_stream_[stream_index]) {
      msg = stream_name_[stream_index] + " Already OFF";
    }
    ROS_INFO_STREAM("toggling sensor " << stream_name_[stream_index] << " OFF");
  }
  if (!msg.empty()) {
    ROS_INFO_STREAM(msg);
    response.success = false;
    response.message = msg;
    return false;
  }
  response.success = toggleSensor(stream_index, request.data, response.message);
  return response.success;
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  (void)msg;
  stopStreams();
  enable_stream_[stream_index] = enabled;
  startStreams();
  return true;
}

bool OBCameraNode::saveImagesCallback(std_srvs::EmptyRequest& request,
                                      std_srvs::EmptyResponse& response) {
  (void)request;
  (void)response;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      save_images_[stream_index] = true;
      save_images_count_[stream_index] = 0;
    } else {
      ROS_WARN_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    }
  }
  return true;
}

bool OBCameraNode::savePointCloudCallback(std_srvs::EmptyRequest& request,
                                          std_srvs::EmptyResponse& response) {
  (void)request;
  (void)response;
  save_point_cloud_ = true;
  save_colored_point_cloud_ = true;
  return true;
}

bool OBCameraNode::getCameraParamsCallback(orbbec_camera::GetCameraParamsRequest& request,
                                           orbbec_camera::GetCameraParamsResponse& response) {
  (void)request;
  try {
    OBCameraParam camera_param{};
    auto default_param = getCameraParam();
    if (depth_registration_ && pipeline_started_ && pipeline_ != nullptr) {
      camera_param = pipeline_->getCameraParam();
    } else if (default_param) {
      camera_param = *default_param;
    } else {
      ROS_ERROR_STREAM("get camera param failed");
      response.message = "get camera param failed";
      return false;
    }
    response.l_intr_p[0] = camera_param.depthIntrinsic.fx;
    response.l_intr_p[1] = camera_param.depthIntrinsic.fy;
    response.l_intr_p[2] = camera_param.depthIntrinsic.cx;
    response.l_intr_p[3] = camera_param.depthIntrinsic.cy;
    response.r_intr_p[0] = camera_param.rgbIntrinsic.fx;
    response.r_intr_p[1] = camera_param.rgbIntrinsic.fy;
    response.r_intr_p[2] = camera_param.rgbIntrinsic.cx;
    response.r_intr_p[3] = camera_param.rgbIntrinsic.cy;
    for (int i = 0; i < 9; i++) {
      if (i < 3) {
        response.r2l_t[i] = camera_param.transform.trans[i];
      }
      response.r2l_r[i] = camera_param.transform.rot[i];
    }

  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get camera params: " << e.getMessage());
    return false;
  }
  return true;
}

bool OBCameraNode::getSerialNumberCallback(GetStringRequest& request, GetStringResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    auto device_info = device_->getDeviceInfo();
    response.data = device_info->serialNumber();
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get serial number: " << e.getMessage());
    return false;
  }
  response.success = true;
  return true;
}

bool OBCameraNode::getDeviceTypeCallback(GetStringRequest& request, GetStringResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  auto device_info = device_->getDeviceInfo();
  response.data = ObDeviceTypeToString(device_info->deviceType());
  response.success = true;
  return true;
}

bool OBCameraNode::getLdpMeasureDistanceCallback(GetInt32Request& request,
                                                 GetInt32Response& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    response.data = device_->getIntProperty(OB_PROP_LDP_MEASURE_DISTANCE_INT);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get ldp measure distance: " << e.getMessage());
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::getCameraInfoCallback(GetCameraInfoRequest& request,
                                         GetCameraInfoResponse& response,
                                         const stream_index_pair& stream_index) {
  (void)request;
  try {
    auto camera_param = pipeline_->getCameraParam();
    auto& intrinsic =
        stream_index == COLOR ? camera_param.rgbIntrinsic : camera_param.depthIntrinsic;
    auto& distortion =
        stream_index == COLOR ? camera_param.rgbDistortion : camera_param.depthDistortion;
    auto width = width_[stream_index];
    auto camera_info = convertToCameraInfo(intrinsic, distortion, width);
    response.info = camera_info;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get camera info: " << e.getMessage());
    return false;
  }
  return true;
}

bool OBCameraNode::resetCameraGainCallback(std_srvs::EmptyRequest& request,
                                           std_srvs::EmptyResponse& response,
                                           const stream_index_pair& stream_index) {
  (void)request;
  (void)response;
  auto data = default_gain_[stream_index];
  auto sensor = sensors_[stream_index];
  if (sensor) {
    try {
      auto range = sensor->getGainRange();
      if (data < range.min || data > range.max) {
        ROS_ERROR_STREAM("Failed to set gain: invalid value");
        return false;
      }
      sensor->setGain(data);
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to set gain: " << e.getMessage());
      return false;
    }
  } else {
    ROS_ERROR_STREAM("Failed to set gain: sensor is not initialized");
    return false;
  }
}

bool OBCameraNode::resetCameraExposureCallback(std_srvs::EmptyRequest& request,
                                               std_srvs::EmptyResponse& response,
                                               const stream_index_pair& stream_index) {
  (void)request;
  (void)response;
  auto data = default_exposure_[stream_index];
  auto sensor = sensors_[stream_index];
  if (sensor) {
    try {
      sensor->setExposure(data);
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to set exposure: " << e.getMessage());
      return false;
    }
  } else {
    ROS_ERROR_STREAM("Failed to set exposure: sensor is not initialized");
    return false;
  }
}

bool OBCameraNode::resetCameraWhiteBalanceCallback(std_srvs::EmptyRequest& request,
                                                   std_srvs::EmptyResponse& response) {
  (void)request;
  (void)response;
  auto data = default_white_balance_;
  auto sensor = sensors_[COLOR];
  if (sensor) {
    try {
      auto range = sensor->getWhiteBalanceRange();
      if (data < range.min || data > range.max) {
        ROS_ERROR_STREAM("Failed to set white balance: invalid value");
        return false;
      }
      sensor->setWhiteBalance(data);
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to set white balance: " << e.getMessage());
      return false;
    }
  } else {
    ROS_ERROR_STREAM("Failed to set white balance: sensor is not initialized");
    return false;
  }
}

bool OBCameraNode::switchIRModeCallback(SetInt32Request& request, SetInt32Response& response) {
  try {
    device_->setIntProperty(OB_PROP_SWITCH_IR_MODE_INT, request.data);
    return true;
  } catch (const ob::Error& e) {
    std::stringstream ss;
    ss << "Failed to switch IR mode: " << e.getMessage();
    ROS_ERROR_STREAM(ss.str());
    response.message = ss.str();
    return false;
  }
}

bool OBCameraNode::switchIRDataSourceChannelCallback(SetStringRequest& request,
                                                     SetStringResponse& response) {
  if (request.data != "left" && request.data != "right") {
    ROS_ERROR_STREAM("Failed to switch IR data source channel: invalid channel name(left/right)");
    return false;
  }
  try {
    int data = request.data == "left" ? 0 : 1;
    device_->setIntProperty(OB_PROP_IR_CHANNEL_DATA_SOURCE_INT, data);
    return true;
  } catch (const ob::Error& e) {
    std::stringstream ss;
    ss << "Failed to switch IR data source channel: " << e.getMessage();
    ROS_ERROR_STREAM(ss.str());
    response.message = ss.str();
    return false;
  }
  return false;
}
}  // namespace orbbec_camera
