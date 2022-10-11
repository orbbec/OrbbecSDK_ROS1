#include "orbbec_camera/ob_camera_node.h"

namespace orbbec_camera {

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::SetBool;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_[stream_index]) {
      ROS_INFO_STREAM("Stream " << stream_name_[stream_index] << " is disabled.");
      continue;
    }
    auto stream_name = stream_name_[stream_index];
    std::string service_name = "get_" + stream_name + "_exposure";
    get_exposure_srv_[stream_index] = nh_.advertiseService<GetInt32Request, GetInt32Response>(
        service_name, [this, stream_index](GetInt32Request& request, GetInt32Response& response) {
          response.success = this->getExposureCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "set_" + stream_name + "_exposure";
    set_exposure_srv_[stream_index] = nh_.advertiseService<SetInt32Request, SetInt32Response>(
        service_name, [this, stream_index](SetInt32Request& request, SetInt32Response& response) {
          response.success = this->setExposureCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "get_" + stream_name + "_gain";
    get_gain_srv_[stream_index] = nh_.advertiseService<GetInt32Request, GetInt32Response>(
        service_name, [this, stream_index](GetInt32Request& request, GetInt32Response& response) {
          response.success = this->getGainCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "set_" + stream_name + "_gain";
    set_gain_srv_[stream_index] = nh_.advertiseService<SetInt32Request, SetInt32Response>(
        service_name, [this, stream_index](SetInt32Request& request, SetInt32Response& response) {
          response.success = this->setGainCallback(request, response, stream_index);
          return response.success;
        });
    service_name = "set_" + stream_name + "_mirror";
    set_mirror_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->setMirrorCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "set_" + stream_name + "_auto_exposure";
    set_auto_exposure_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->setAutoExposureCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "toggle_" + stream_name;
    toggle_sensor_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index = stream_index](auto&& request, auto&& response) {
              response.success = this->toggleSensorCallback(request, response, stream_index);
              return response.success;
            });
  }
  get_auto_white_balance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_auto_white_balance", [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getAutoWhiteBalanceCallback(request, response);
        return response.success;
      });
  set_auto_white_balance_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "set_auto_white_balance", [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->setAutoWhiteBalanceCallback(request, response);
        return response.success;
      });
  get_white_balance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "get_white_balance", [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getWhiteBalanceCallback(request, response);
        return response.success;
      });
  set_white_balance_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "set_white_balance", [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->setWhiteBalanceCallback(request, response);
        return response.success;
      });
  set_fan_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "set_fan", [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setFanCallback(request, response);
        return response.success;
      });
  set_floor_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "set_floor", [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setFloorCallback(request, response);
        return response.success;
      });
  set_laser_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "set_laser", [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setLaserCallback(request, response);
        return response.success;
      });
  set_ldp_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "set_ldp", [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setLdpEnableCallback(request, response);
        return response.success;
      });
  get_device_info_srv_ = nh_.advertiseService<GetDeviceInfoRequest, GetDeviceInfoResponse>(
      "get_device_info", [this](GetDeviceInfoRequest& request, GetDeviceInfoResponse& response) {
        response.success = this->getDeviceInfoCallback(request, response);
        return response.success;
      });

  get_sdk_version_srv_ = nh_.advertiseService<GetStringRequest, GetStringResponse>(
      "get_sdk_version", [this](GetStringRequest& request, GetStringResponse& response) {
        response.success = this->getSDKVersionCallback(request, response);
        return response.success;
      });
  save_images_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "save_images", [this](std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response) {
        return this->saveImagesCallback(request, response);
      });
}

bool OBCameraNode::setMirrorCallback(std_srvs::SetBoolRequest& request,
                                     std_srvs::SetBoolResponse& response,
                                     const stream_index_pair& stream_index) {
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return response.success;
  }
  auto sensor = sensors_[stream_index];
  sensor->setMirror(request.data);
  return true;
}

bool OBCameraNode::getExposureCallback(GetInt32Request& request, GetInt32Response& response,
                                       const stream_index_pair& stream_index) {
  (void)request;
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  response.data = sensor->getExposure();
  return true;
}

bool OBCameraNode::setExposureCallback(SetInt32Request& request, SetInt32Response& response,
                                       const stream_index_pair& stream_index) {
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  sensor->setExposure(request.data);
  return true;
}

bool OBCameraNode::getGainCallback(GetInt32Request& request, GetInt32Response& response,
                                   const stream_index_pair& stream_index) {
  (void)request;
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  response.data = sensor->getGain();
  return true;
}

bool OBCameraNode::setGainCallback(SetInt32Request& request, SetInt32Response& response,
                                   const stream_index_pair& stream_index) {
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  sensor->setGain(request.data);
  return true;
}

bool OBCameraNode::getAutoWhiteBalanceCallback(GetInt32Request& request,
                                               GetInt32Response& response) {
  (void)request;
  if (!enable_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  response.data = sensor->getAutoWhiteBalance();
  return true;
}

bool OBCameraNode::setAutoWhiteBalanceCallback(SetInt32Request& request,
                                               SetInt32Response& response) {
  if (!enable_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  sensor->setAutoWhiteBalance(request.data);
  return true;
}

bool OBCameraNode::getWhiteBalanceCallback(GetInt32Request& request, GetInt32Response& response) {
  (void)request;
  if (!enable_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  response.data = sensor->getWhiteBalance();
  return true;
}

bool OBCameraNode::setWhiteBalanceCallback(SetInt32Request& request, SetInt32Response& response) {
  if (!enable_[COLOR]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[COLOR] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[COLOR];
  sensor->setWhiteBalance(request.data);
  return true;
}

bool OBCameraNode::setAutoExposureCallback(std_srvs::SetBoolRequest& request,
                                           std_srvs::SetBoolResponse& response,
                                           const stream_index_pair& stream_index) {
  if (!enable_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto sensor = sensors_[stream_index];
  sensor->setAutoExposure(request.data);
  return true;
}

bool OBCameraNode::setLaserCallback(std_srvs::SetBoolRequest& request,
                                    std_srvs::SetBoolResponse& response) {
  (void)response;
  try {
    device_->setBoolProperty(OB_PROP_LASER_BOOL, request.data);
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
  try {
    device_->setBoolProperty(OB_PROP_LDP_BOOL, request.data);
  }catch (const ob::Error &e){
    ROS_ERROR_STREAM("Failed to set LDP: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::setFanCallback(std_srvs::SetBoolRequest& request,
                                  std_srvs::SetBoolResponse& response) {
  (void)response;
  try {
    device_->setBoolProperty(OB_PROP_FAN_WORK_MODE_INT, request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("set fan failed: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::setFloorCallback(std_srvs::SetBoolRequest& request,
                                    std_srvs::SetBoolResponse& response) {
  try {
    device_->setBoolProperty(OB_PROP_FLOOD_BOOL, request.data);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("set floor failed: " << e.getMessage());
    response.message = e.getMessage();
    return false;
  }
  return true;
}

bool OBCameraNode::getDeviceInfoCallback(GetDeviceInfoRequest& request,
                                         GetDeviceInfoResponse& response) {
  (void)request;
  auto device_info = device_->getDeviceInfo();
  response.info.name = device_info->name();
  response.info.pid = device_info->pid();
  response.info.vid = device_info->vid();
  response.info.serial_number = device_info->serialNumber();
  response.info.firmware_version = device_info->firmwareVersion();
  response.info.supported_min_sdk_version = device_info->supportedMinSdkVersion();
  response.success = true;
  return true;
}

bool OBCameraNode::getSDKVersionCallback(GetStringRequest& request, GetStringResponse& response) {
  (void)request;
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
    if (enable_[stream_index]) {
      msg = stream_name_[stream_index] + " Already ON";
    }
    ROS_INFO_STREAM("toggling sensor " << stream_name_[stream_index] << " ON");
  } else {
    if (!enable_[stream_index]) {
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
  enable_[stream_index] = enabled;
  startStreams();
  return true;
}

bool OBCameraNode::saveImagesCallback(std_srvs::EmptyRequest& request,
                                      std_srvs::EmptyResponse& response) {
  (void)request;
  (void)response;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_[stream_index]) {
      save_images_[stream_index] = true;
    } else {
      ROS_WARN_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    }
  }
  return true;
}

}  // namespace orbbec_camera
