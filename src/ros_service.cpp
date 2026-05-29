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

namespace {

bool isGemini330SeriesForDisparity(uint32_t pid) {
  return pid == GEMINI_335_PID || pid == GEMINI_336_PID || pid == GEMINI_330_PID ||
         pid == GEMINI_335L_PID || pid == GEMINI_336L_PID || pid == GEMINI_330L_PID ||
         pid == GEMINI_335LG_PID || pid == GEMINI_335LE_PID || pid == GEMINI_338_PID ||
         pid == GEMINI_338L_PID || pid == GEMINI_338LE_PID || pid == GEMINI_338LG_PID ||
         pid == GEMINI_331L_PID;
}

bool isSupportedDisparityResolutionForPid(uint32_t pid, int width, int height) {
  if (pid == GEMINI_335LE_PID || pid == GEMINI_338LE_PID) {
    return (width == 1280 && height == 800) || (width == 640 && height == 400) ||
           (width == 424 && height == 266) || (width == 320 && height == 200);
  }

  return (width == 1280 && height == 800) || (width == 1280 && height == 720) ||
         (width == 640 && height == 400) || (width == 424 && height == 266);
}

std::string getDisparityResolutionHintByPid(uint32_t pid) {
  if (pid == GEMINI_335LE_PID || pid == GEMINI_338LE_PID) {
    return "Supported resolutions for the current device: 1280x800/640x400/424x266/320x200";
  }

  return "Supported resolutions for the current device: "
         "1280x800/1280x720/640x400/424x266";
}

bool setDisparityServiceFailure(SetInt32Response& response, const std::string& message) {
  response.success = false;
  response.message = message;
  ROS_INFO_STREAM(message);
  return true;
}

}  // namespace

void OBCameraNode::setupCameraCtrlServices() {
  using std_srvs::SetBool;
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      ROS_DEBUG_STREAM("Stream " << stream_name_[stream_index] << " is disabled.");
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
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_ae_roi";
    set_ae_roi_srv_[stream_index] = nh_.advertiseService<SetArraysRequest, SetArraysResponse>(
        service_name, [this, stream_index](SetArraysRequest& request, SetArraysResponse& response) {
          this->setAeRoiCallback(request, response, stream_index);
          return true;
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
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_flip";
    set_flip_srv_[stream_index] =
        nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
            service_name, [this, stream_index](std_srvs::SetBoolRequest& request,
                                               std_srvs::SetBoolResponse& response) {
              response.success = this->setFlipCallback(request, response, stream_index);
              return response.success;
            });
    service_name = "/" + camera_name_ + "/" + "set_" + stream_name + "_rotation";
    set_rotation_srv_[stream_index] = nh_.advertiseService<SetInt32Request, SetInt32Response>(
        service_name, [this, stream_index](SetInt32Request& request, SetInt32Response& response) {
          response.success = this->setRotationCallback(request, response, stream_index);
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
  set_ptp_config_srv_ = nh_.advertiseService<std_srvs::SetBoolRequest, std_srvs::SetBoolResponse>(
      "/" + camera_name_ + "/" + "set_ptp_config",
      [this](std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response) {
        response.success = this->setPtpConfigCallback(request, response);
        return response.success;
      });
  get_ptp_config_srv_ = nh_.advertiseService<GetBoolRequest, GetBoolResponse>(
      "/" + camera_name_ + "/" + "get_ptp_config",
      [this](GetBoolRequest& request, GetBoolResponse& response) {
        response.success = this->getPtpConfigCallback(request, response);
        return response.success;
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
  export_config_json_srv_ = nh_.advertiseService<SetStringRequest, SetStringResponse>(
      "/" + camera_name_ + "/" + "export_config_json",
      [this](SetStringRequest& request, SetStringResponse& response) {
        return this->exportConfigJsonCallback(request, response);
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
  get_lrm_measure_distance_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "/" + camera_name_ + "/" + "get_lrm_measure_distance",
      [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getLrmMeasureDistanceCallback(request, response);
        return response.success;
      });

  set_write_customerdata_srv_ = nh_.advertiseService<SetStringRequest, SetStringResponse>(
      "/" + camera_name_ + "/" + "set_write_customer_data",
      [this](SetStringRequest& request, SetStringResponse& response) {
        response.success = this->setWriteCustomerData(request, response);
        return response.success;
      });
  set_read_customerdata_srv_ = nh_.advertiseService<GetStringRequest, GetStringResponse>(
      "/" + camera_name_ + "/" + "set_read_customer_data",
      [this](GetStringRequest& request, GetStringResponse& response) {
        response.success = this->setReadCustomerData(request, response);
        return response.success;
      });
  get_laser_status_srv_ = nh_.advertiseService<GetBoolRequest, GetBoolResponse>(
      "/" + camera_name_ + "/" + "get_laser_status",
      [this](GetBoolRequest& request, GetBoolResponse& response) {
        response.success = this->getLaserStatusCallback(request, response);
        return response.success;
      });
  set_point_cloud_decimation_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "set_point_cloud_decimation",
      [this](SetInt32Request& request, SetInt32Response& response) {
        response.success = this->setPointCloudDecimationCallback(request, response);
        return response.success;
      });
  get_point_cloud_decimation_srv_ = nh_.advertiseService<GetInt32Request, GetInt32Response>(
      "/" + camera_name_ + "/" + "get_point_cloud_decimation",
      [this](GetInt32Request& request, GetInt32Response& response) {
        response.success = this->getPointCloudDecimationCallback(request, response);
        return response.success;
      });
  set_disparity_range_mode_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "set_disparity_range_mode",
      [this](SetInt32Request& request, SetInt32Response& response) {
        return this->setDisparityRangeModeCallback(request, response);
      });
  set_disparity_search_offset_srv_ = nh_.advertiseService<SetInt32Request, SetInt32Response>(
      "/" + camera_name_ + "/" + "set_disparity_search_offset",
      [this](SetInt32Request& request, SetInt32Response& response) {
        return this->setDisparitySearchOffsetCallback(request, response);
      });
  set_ae_reference_stream_srv_ = nh_.advertiseService<SetString::Request, SetString::Response>(
      "/" + camera_name_ + "/" + "set_ae_reference_stream",
      [this](const SetStringRequest& request, SetStringResponse& response) {
        this->setAEReferenceStreamCallback(request, response);
        return true;
      });

  set_ae_strategy_srv_ = nh_.advertiseService<SetString::Request, SetString::Response>(
      "/" + camera_name_ + "/" + "set_ae_strategy",
      [this](const SetStringRequest& request, SetStringResponse& response) {
        this->setAEStrategyCallback(request, response);
        return true;
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
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setBoolProperty(OB_PROP_IR_RIGHT_MIRROR_BOOL, request.data);
        break;
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_MIRROR_BOOL, request.data);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, request.data);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, request.data);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setBoolProperty(OB_PROP_COLOR_LEFT_MIRROR_BOOL, request.data);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setBoolProperty(OB_PROP_COLOR_RIGHT_MIRROR_BOOL, request.data);
        break;
      default:
        ROS_ERROR_STREAM(" NOT a video stream" << __FUNCTION__);
        return false;
        break;
    }
    image_mirror_[stream_index] = request.data;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set mirror mode: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return response.success;
  }
  return true;
}

bool OBCameraNode::setFlipCallback(std_srvs::SetBoolRequest& request,
                                   std_srvs::SetBoolResponse& response,
                                   const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return response.success;
  }
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setBoolProperty(OB_PROP_IR_RIGHT_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_IR:
        device_->setBoolProperty(OB_PROP_IR_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_DEPTH:
        device_->setBoolProperty(OB_PROP_DEPTH_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_COLOR:
        device_->setBoolProperty(OB_PROP_COLOR_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setBoolProperty(OB_PROP_COLOR_LEFT_FLIP_BOOL, request.data);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setBoolProperty(OB_PROP_COLOR_RIGHT_FLIP_BOOL, request.data);
        break;
      default:
        ROS_ERROR_STREAM(" NOT a video stream" << __FUNCTION__);
        return false;
        break;
    }
    image_flip_[stream_index] = request.data;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set flip mode: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return response.success;
  }
  return true;
}

bool OBCameraNode::setRotationCallback(SetInt32Request& request, SetInt32Response& response,
                                       const stream_index_pair& stream_index) {
  if (!enable_stream_[stream_index]) {
    ROS_ERROR_STREAM("Camera " << stream_name_[stream_index] << " is not enabled.");
    response.success = false;
    return false;
  }
  auto stream = stream_index.first;
  try {
    switch (stream) {
      case OB_STREAM_IR_RIGHT:
        device_->setIntProperty(OB_PROP_IR_RIGHT_ROTATE_INT, request.data);
        break;
      case OB_STREAM_IR_LEFT:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request.data);
        break;
      case OB_STREAM_IR:
        device_->setIntProperty(OB_PROP_IR_ROTATE_INT, request.data);
        break;
      case OB_STREAM_DEPTH:
        device_->setIntProperty(OB_PROP_DEPTH_ROTATE_INT, request.data);
        break;
      case OB_STREAM_COLOR:
        device_->setIntProperty(OB_PROP_COLOR_ROTATE_INT, request.data);
        break;
      case OB_STREAM_COLOR_LEFT:
        device_->setIntProperty(OB_PROP_COLOR_LEFT_ROTATE_INT, request.data);
        break;
      case OB_STREAM_COLOR_RIGHT:
        device_->setIntProperty(OB_PROP_COLOR_RIGHT_ROTATE_INT, request.data);
        break;
      default:
        ROS_ERROR_STREAM(" NOT a video stream" << __FUNCTION__);
        return false;
        break;
    }
    image_rotation_[stream_index] = request.data;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set rotation mode: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return false;
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
    ROS_ERROR_STREAM("Failed to get exposure: " << orbbec_camera::formatObErrorWithStatus(e));
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
    if (stream_index == COLOR) {
      color_exposure_ = request.data;
    } else if (stream_index == DEPTH) {
      depth_exposure_ = request.data;
    } else if (stream_index == INFRA0 || stream_index == INFRA1 || stream_index == INFRA2) {
      ir_exposure_ = request.data;
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set exposure: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setAeRoiCallback(SetArraysRequest& request, SetArraysResponse& response,
                                    const stream_index_pair& stream_index) {
  auto stream = stream_index.first;
  if (isGemini305SeriesPID(device_->getDeviceInfo()->pid()) &&
      (stream != OB_STREAM_COLOR && ae_reference_stream_ == "color")) {
    response.success = false;
    response.message = "AE Reference Stream is color, other sensors setting is not supported";
    ROS_ERROR_STREAM(response.message);
    return true;
  }
  if (isGemini305SeriesPID(device_->getDeviceInfo()->pid()) &&
      (stream != OB_STREAM_DEPTH && ae_reference_stream_ == "depth")) {
    response.success = false;
    response.message = "AE Reference Stream is depth, other sensors setting is not supported";
    ROS_ERROR_STREAM(response.message);
    return true;
  }
  auto config = OBRegionOfInterest();
  uint32_t data_size = sizeof(config);
  try {
    switch (stream) {
      case OB_STREAM_IR_LEFT:
      case OB_STREAM_IR_RIGHT:
      case OB_STREAM_IR:
      case OB_STREAM_DEPTH:
        config.x0_left = (static_cast<short int>(request.data_param[0]) < 0)
                             ? 0
                             : static_cast<short int>(request.data_param[0]);
        config.x0_left = (static_cast<short int>(request.data_param[0]) > width_[DEPTH] - 1)
                             ? width_[DEPTH] - 1
                             : config.x0_left;
        config.y0_top = (static_cast<short int>(request.data_param[2]) < 0)
                            ? 0
                            : static_cast<short int>(request.data_param[2]);
        config.y0_top = (static_cast<short int>(request.data_param[2]) > height_[DEPTH] - 1)
                            ? height_[DEPTH] - 1
                            : config.y0_top;
        config.x1_right = (static_cast<short int>(request.data_param[1]) < 0)
                              ? 0
                              : static_cast<short int>(request.data_param[1]);
        config.x1_right = (static_cast<short int>(request.data_param[1]) > width_[DEPTH] - 1)
                              ? width_[DEPTH] - 1
                              : config.x1_right;
        config.y1_bottom = (static_cast<short int>(request.data_param[3]) < 0)
                               ? 0
                               : static_cast<short int>(request.data_param[3]);
        config.y1_bottom = (static_cast<short int>(request.data_param[3]) > height_[DEPTH] - 1)
                               ? height_[DEPTH] - 1
                               : config.y1_bottom;
        device_->setStructuredData(OB_STRUCT_DEPTH_AE_ROI,
                                   reinterpret_cast<const uint8_t*>(&config), sizeof(config));
        device_->getStructuredData(OB_STRUCT_DEPTH_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                   &data_size);
        ROS_INFO_STREAM("set depth AE ROI : "
                        << "[Left: " << config.x0_left << ", Right: " << config.x1_right
                        << ", Top: " << config.y0_top << ", Bottom: " << config.y1_bottom << " ]");
        depth_ae_roi_left_ = config.x0_left;
        depth_ae_roi_right_ = config.x1_right;
        depth_ae_roi_top_ = config.y0_top;
        depth_ae_roi_bottom_ = config.y1_bottom;
        break;
      case OB_STREAM_COLOR:
      case OB_STREAM_COLOR_LEFT:
      case OB_STREAM_COLOR_RIGHT:
        config.x0_left = (static_cast<short int>(request.data_param[0]) < 0)
                             ? 0
                             : static_cast<short int>(request.data_param[0]);
        config.x0_left = (static_cast<short int>(request.data_param[0]) > width_[COLOR] - 1)
                             ? width_[COLOR] - 1
                             : config.x0_left;
        config.y0_top = (static_cast<short int>(request.data_param[2]) < 0)
                            ? 0
                            : static_cast<short int>(request.data_param[2]);
        config.y0_top = (static_cast<short int>(request.data_param[2]) > height_[COLOR] - 1)
                            ? height_[COLOR] - 1
                            : config.y0_top;
        config.x1_right = (static_cast<short int>(request.data_param[1]) < 0)
                              ? 0
                              : static_cast<short int>(request.data_param[1]);
        config.x1_right = (static_cast<short int>(request.data_param[1]) > width_[COLOR] - 1)
                              ? width_[COLOR] - 1
                              : config.x1_right;
        config.y1_bottom = (static_cast<short int>(request.data_param[3]) < 0)
                               ? 0
                               : static_cast<short int>(request.data_param[3]);
        config.y1_bottom = (static_cast<short int>(request.data_param[3]) > height_[COLOR] - 1)
                               ? height_[COLOR] - 1
                               : config.y1_bottom;
        device_->setStructuredData(OB_STRUCT_COLOR_AE_ROI,
                                   reinterpret_cast<const uint8_t*>(&config), sizeof(config));
        device_->getStructuredData(OB_STRUCT_COLOR_AE_ROI, reinterpret_cast<uint8_t*>(&config),
                                   &data_size);
        ROS_INFO_STREAM("set color AE ROI : "
                        << "[Left: " << config.x0_left << ", Right: " << config.x1_right
                        << ", Top: " << config.y0_top << ", Bottom: " << config.y1_bottom << " ]");
        color_ae_roi_left_ = config.x0_left;
        color_ae_roi_right_ = config.x1_right;
        color_ae_roi_top_ = config.y0_top;
        color_ae_roi_bottom_ = config.y1_bottom;
        break;
      default:
        ROS_ERROR_STREAM(" NOT a video stream" << __FUNCTION__);
        response.success = false;
        response.message = "NOT a video stream";
        return true;
    }
    response.success = true;
    response.message = "set AE ROI success";
    return true;
  } catch (const ob::Error& e) {
    response.message = orbbec_camera::formatObErrorWithStatus(e);
    response.success = false;
    ROS_ERROR_STREAM("Failed to set AE ROI: " << orbbec_camera::formatObErrorWithStatus(e));
    return true;
  } catch (const std::exception& e) {
    response.message = e.what();
    response.success = false;
    ROS_ERROR_STREAM("Failed to set AE ROI: " << response.message);
    return true;
  } catch (...) {
    response.message = "unknown error";
    response.success = false;
    ROS_ERROR_STREAM("Failed to set AE ROI: " << response.message);
    return true;
  }
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
    ROS_ERROR_STREAM("Failed to get gain: " << orbbec_camera::formatObErrorWithStatus(e));
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
    if (stream_index == COLOR) {
      color_gain_ = gain;
    } else if (stream_index == DEPTH) {
      depth_gain_ = gain;
    } else if (stream_index == INFRA0 || stream_index == INFRA1 || stream_index == INFRA2) {
      ir_gain_ = gain;
    }
    ROS_INFO_STREAM("After set gain: " << gain);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set gain: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM(
        "Failed to get auto white balance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    enable_color_auto_white_balance_ = result;
    ROS_INFO_STREAM("After set auto white balance: " << result);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM(
        "Failed to set auto white balance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM("Failed to get white balance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    color_white_balance_ = request.data;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set white balance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    if (stream_index == COLOR) {
      enable_color_auto_exposure_ = request.data;
    } else if (stream_index == DEPTH || stream_index == INFRA0 || stream_index == INFRA1 ||
               stream_index == INFRA2) {
      enable_ir_auto_exposure_ = request.data;
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set auto exposure: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM("Failed to get auto exposure: " << orbbec_camera::formatObErrorWithStatus(e));
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
    bool cache_updated = false;
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, data);
      cache_updated = true;
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      device_->setBoolProperty(OB_PROP_LASER_BOOL, data);
      cache_updated = true;
    }
    if (cache_updated) {
      enable_laser_ = request.data;
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set laser: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = orbbec_camera::formatObErrorWithStatus(e);
    return false;
  }
  return true;
}

bool OBCameraNode::setPtpConfigCallback(std_srvs::SetBoolRequest& request,
                                        std_srvs::SetBoolResponse& response) {
  (void)response;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    device_->setBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL, request.data);
    enable_ptp_config_ = request.data;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("set ptp config failed: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = orbbec_camera::formatObErrorWithStatus(e);
    return false;
  }
  return true;
}

bool OBCameraNode::getPtpConfigCallback(GetBoolRequest& request, GetBoolResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    response.data = device_->getBoolProperty(OB_DEVICE_PTP_CLOCK_SYNC_ENABLE_BOOL);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM(
        "Failed to get config sync status: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setLdpEnableCallback(std_srvs::SetBoolRequest& request,
                                        std_srvs::SetBoolResponse& response) {
  (void)response;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  bool ldp_enable = request.data;
  try {
    bool cache_updated = false;
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      auto laser_enable = device_->getIntProperty(OB_PROP_LASER_CONTROL_INT);
      device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      device_->setIntProperty(OB_PROP_LASER_CONTROL_INT, laser_enable);
      cache_updated = true;
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      if (!ldp_enable) {
        auto laser_enable = device_->getBoolProperty(OB_PROP_LASER_BOOL);
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        device_->setBoolProperty(OB_PROP_LASER_BOOL, laser_enable);
      } else {
        device_->setBoolProperty(OB_PROP_LDP_BOOL, ldp_enable);
      }
      cache_updated = true;
    }
    if (cache_updated) {
      enable_ldp_ = ldp_enable;
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to set LDP: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = orbbec_camera::formatObErrorWithStatus(e);
    return false;
  }
  return true;
}

bool OBCameraNode::getLdpStatusCallback(GetBoolRequest& request, GetBoolResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    response.data = device_->getBoolProperty(OB_PROP_LDP_STATUS_BOOL);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get LDP status: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM("set fan failed: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = orbbec_camera::formatObErrorWithStatus(e);
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
    ROS_ERROR_STREAM("set flood failed: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = orbbec_camera::formatObErrorWithStatus(e);
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
  bool current_state = enable_stream_[stream_index];
  bool target_state = request.data;

  if (target_state == current_state) {
    msg = stream_name_[stream_index] + (target_state ? " Already ON" : " Already OFF");
    ROS_INFO_STREAM(msg);
    response.success = true;
    response.message = msg;
    return true;
  }
  ROS_INFO_STREAM("Toggling sensor " << stream_name_[stream_index]
                                     << (target_state ? " ON" : " OFF"));

  response.success = toggleSensor(stream_index, target_state, response.message);
  return true;
}

bool OBCameraNode::toggleSensor(const stream_index_pair& stream_index, bool enabled,
                                std::string& msg) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    stopStreams();
    enable_stream_[stream_index] = enabled;
    startStreams();

    msg = "Toggling sensor " + stream_name_[stream_index] + (enabled ? " on" : " off");
    return true;
  } catch (const std::exception& e) {
    msg = "Failed to toggle " + stream_name_[stream_index] + ": " + e.what();
    ROS_ERROR_STREAM(msg);
    return false;
  }
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

bool OBCameraNode::exportConfigJsonCallback(SetStringRequest& request,
                                            SetStringResponse& response) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  response.success = exportConfigJsonToFile(request.data, response.message);
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
    ROS_ERROR_STREAM("Failed to get camera params: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM("Failed to get serial number: " << orbbec_camera::formatObErrorWithStatus(e));
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

bool OBCameraNode::getLrmMeasureDistanceCallback(GetInt32Request& request,
                                                 GetInt32Response& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    response.data = device_->getIntProperty(OB_PROP_LDP_MEASURE_DISTANCE_INT);
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM(
        "Failed to get ldp measure distance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ROS_ERROR_STREAM("Failed to get camera info: " << orbbec_camera::formatObErrorWithStatus(e));
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
        ROS_ERROR_STREAM("Failed to set gain: invalid value "
                         << data << ", valid range: " << range.min << " - " << range.max);
        return false;
      }
      sensor->setGain(data);
      const auto gain = sensor->getGain();
      if (stream_index == COLOR) {
        color_gain_ = gain;
      } else if (stream_index == DEPTH) {
        depth_gain_ = gain;
      } else if (stream_index == INFRA0 || stream_index == INFRA1 || stream_index == INFRA2) {
        ir_gain_ = gain;
      }
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to set gain: " << orbbec_camera::formatObErrorWithStatus(e));
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
      if (stream_index == COLOR) {
        color_exposure_ = data;
      } else if (stream_index == DEPTH) {
        depth_exposure_ = data;
      } else if (stream_index == INFRA0 || stream_index == INFRA1 || stream_index == INFRA2) {
        ir_exposure_ = data;
      }
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM("Failed to set exposure: " << orbbec_camera::formatObErrorWithStatus(e));
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
      color_white_balance_ = data;
      return true;
    } catch (const ob::Error& e) {
      ROS_ERROR_STREAM(
          "Failed to set white balance: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ss << "Failed to switch IR mode: " << orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM("Failed to switch IR mode: " << orbbec_camera::formatObErrorWithStatus(e));
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
    ss << "Failed to switch IR data source channel: "
       << orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM(
        "Failed to switch IR data source channel: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = ss.str();
    return false;
  }
  return false;
}

bool OBCameraNode::setWriteCustomerData(SetStringRequest& request, SetStringResponse& response) {
  if (request.data.empty()) {
    response.success = false;
    response.message = "set write customer data is empty";
    return false;
  }
  try {
    device_->writeCustomerData(request.data.c_str(), request.data.size());
    response.message = "set write customer data is " + request.data;
    response.success = true;
    return true;
  } catch (const ob::Error& e) {
    std::stringstream ss;
    ss << "Failed to set write customer data: " << orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM(
        "Failed to set write customer data: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = ss.str();
    return false;
  }
  return false;
}

bool OBCameraNode::setReadCustomerData(GetStringRequest& request, GetStringResponse& response) {
  (void)request;
  try {
    std::vector<uint8_t> customer_date;
    customer_date.resize(40960);
    uint32_t customer_date_len = 0;
    device_->readCustomerData(customer_date.data(), &customer_date_len);
    std::string customer_date_str(customer_date.begin(), customer_date.end());
    response.message = "read customer data is " + customer_date_str;
    response.success = true;
    return true;
  } catch (const ob::Error& e) {
    std::stringstream ss;
    ss << "Failed to read customer data: " << orbbec_camera::formatObErrorWithStatus(e);
    ROS_ERROR_STREAM("Failed to read customer data: " << orbbec_camera::formatObErrorWithStatus(e));
    response.message = ss.str();
    return false;
  }
  return false;
}
bool OBCameraNode::getLaserStatusCallback(GetBoolRequest& request, GetBoolResponse& response) {
  (void)request;
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  try {
    if (device_->isPropertySupported(OB_PROP_LASER_CONTROL_INT, OB_PERMISSION_READ_WRITE)) {
      response.data = device_->getBoolProperty(OB_PROP_LASER_CONTROL_INT) ? true : false;
    } else if (device_->isPropertySupported(OB_PROP_LASER_BOOL, OB_PERMISSION_READ_WRITE)) {
      response.data = device_->getBoolProperty(OB_PROP_LASER_BOOL) ? true : false;
    }
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get laser status: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    return false;
  }
  return true;
}

bool OBCameraNode::setPointCloudDecimationCallback(SetInt32Request& request,
                                                   SetInt32Response& response) {
  try {
    if (request.data < 1 || request.data > 8) {
      ROS_ERROR_STREAM("Point cloud decimation factor " << request.data << " out of range (1-8)");
      response.success = false;
      return false;
    }

    point_cloud_decimation_filter_factor_ = request.data;

    ROS_INFO_STREAM("Set point cloud decimation factor to: " << request.data);
    response.success = true;
    return true;
  } catch (const ob::Error& e) {
    ROS_ERROR_STREAM(
        "Failed to set point cloud decimation: " << orbbec_camera::formatObErrorWithStatus(e));
    response.success = false;
    response.message = orbbec_camera::formatObErrorWithStatus(e);
    return false;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to set point cloud decimation: " << e.what());
    response.success = false;
    response.message = e.what();
    return false;
  }
}

bool OBCameraNode::getPointCloudDecimationCallback(GetInt32Request& request,
                                                   GetInt32Response& response) {
  (void)request;
  try {
    response.data = point_cloud_decimation_filter_factor_;
    response.success = true;
    response.message = "Successfully retrieved point cloud decimation factor";
    return true;
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("Failed to get point cloud decimation: " << e.what());
    response.success = false;
    response.message = e.what();
    return false;
  }
}

bool OBCameraNode::setDisparityRangeModeCallback(SetInt32Request& request,
                                                 SetInt32Response& response) {
  try {
    if (!device_->isPropertySupported(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, OB_PERMISSION_WRITE)) {
      return setDisparityServiceFailure(response,
                                        "Current device does not support disparity range mode");
    }

    const bool allow_set = isGemini435LePID(device_info_->pid()) ||
                           (enable_stream_.count(DEPTH) > 0 && enable_stream_.at(DEPTH));
    if (!allow_set) {
      return setDisparityServiceFailure(
          response, "Disparity range mode can only be set when depth stream is enabled");
    }

    const int depth_width = width_.count(DEPTH) > 0 ? width_.at(DEPTH) : 0;
    const int depth_height = height_.count(DEPTH) > 0 ? height_.at(DEPTH) : 0;
    if (isGemini330SeriesForDisparity(device_info_->pid()) &&
        !isSupportedDisparityResolutionForPid(device_info_->pid(), depth_width, depth_height)) {
      return setDisparityServiceFailure(
          response, "Current depth resolution " + std::to_string(depth_width) + "x" +
                        std::to_string(depth_height) + " is not supported. " +
                        getDisparityResolutionHintByPid(device_info_->pid()));
    }

    auto range = device_->getIntPropertyRange(OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
    const int requested_mode_value = request.data;
    int hw_mode_index = -1;
    if (requested_mode_value == 64) {
      hw_mode_index = 0;
    } else if (requested_mode_value == 128) {
      hw_mode_index = 1;
    } else if (requested_mode_value == 256) {
      hw_mode_index = 2;
    }

    if (hw_mode_index < range.min || hw_mode_index > range.max) {
      std::string supported_mode;
      for (int i = range.min; i <= range.max; ++i) {
        supported_mode += (i == 0)   ? "64"
                          : (i == 1) ? "/128"
                          : (i == 2) ? "/256"
                                     : "/" + std::to_string(i);
      }
      return setDisparityServiceFailure(
          response, "Invalid disparity range mode. Allowed values:" + supported_mode);
    }

    device_->setIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT, hw_mode_index);
    auto current_mode_index = device_->getIntProperty(OB_PROP_DISP_SEARCH_RANGE_MODE_INT);
    auto current_mode_value = (current_mode_index == 0)   ? 64
                              : (current_mode_index == 1) ? 128
                              : (current_mode_index == 2) ? 256
                                                          : current_mode_index;
    disparity_range_mode_ = current_mode_value;
    response.success = true;
    response.message = "disparity_range_mode updated to " + std::to_string(current_mode_value);
    ROS_INFO_STREAM(response.message);
    return true;
  } catch (const ob::Error& e) {
    return setDisparityServiceFailure(response, orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    return setDisparityServiceFailure(response, e.what());
  } catch (...) {
    return setDisparityServiceFailure(response, "unknown error");
  }
}

bool OBCameraNode::setDisparitySearchOffsetCallback(SetInt32Request& request,
                                                    SetInt32Response& response) {
  try {
    if (!device_->isPropertySupported(OB_PROP_DISP_SEARCH_OFFSET_INT, OB_PERMISSION_WRITE)) {
      return setDisparityServiceFailure(response,
                                        "Current device does not support disparity search offset");
    }

    const bool allow_set = isGemini435LePID(device_info_->pid()) ||
                           (enable_stream_.count(DEPTH) > 0 && enable_stream_.at(DEPTH));
    if (!allow_set) {
      return setDisparityServiceFailure(
          response, "Disparity search offset can only be set when depth stream is enabled");
    }

    const int depth_width = width_.count(DEPTH) > 0 ? width_.at(DEPTH) : 0;
    const int depth_height = height_.count(DEPTH) > 0 ? height_.at(DEPTH) : 0;
    if (isGemini330SeriesForDisparity(device_info_->pid()) &&
        !isSupportedDisparityResolutionForPid(device_info_->pid(), depth_width, depth_height)) {
      return setDisparityServiceFailure(
          response, "Current depth resolution " + std::to_string(depth_width) + "x" +
                        std::to_string(depth_height) + " is not supported. " +
                        getDisparityResolutionHintByPid(device_info_->pid()));
    }

    auto range = device_->getIntPropertyRange(OB_PROP_DISP_SEARCH_OFFSET_INT);
    if (request.data < range.min || request.data > range.max) {
      return setDisparityServiceFailure(
          response, "Invalid disparity search offset. Allowed values:" + std::to_string(range.min) +
                        " to " + std::to_string(range.max));
    }

    device_->setIntProperty(OB_PROP_DISP_SEARCH_OFFSET_INT, request.data);
    auto current_offset = device_->getIntProperty(OB_PROP_DISP_SEARCH_OFFSET_INT);
    disparity_search_offset_ = current_offset;
    ROS_INFO_STREAM("Set disparity_search_offset to " << current_offset);
    response.success = true;
    response.message = "disparity_search_offset updated to " + std::to_string(current_offset);
    return true;
  } catch (const ob::Error& e) {
    return setDisparityServiceFailure(response, orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    return setDisparityServiceFailure(response, e.what());
  } catch (...) {
    return setDisparityServiceFailure(response, "unknown error");
  }
}

void OBCameraNode::setAEReferenceStreamCallback(const SetStringRequest& request,
                                                SetStringResponse& response) {
  try {
    if (device_->isPropertySupported(OB_PROP_DEVICE_AE_REFERENCE_INT, OB_PERMISSION_WRITE) &&
        (request.data == "depth" || request.data == "color")) {
      device_->setIntProperty(OB_PROP_DEVICE_AE_REFERENCE_INT, request.data == "depth" ? 0 : 1);
      ae_reference_stream_ = request.data;
      response.success = true;
      response.message = "set AE reference stream success";
    } else {
      response.success = false;
      response.message = "set AE reference stream failed";
    }
  } catch (...) {
    response.success = false;
    response.message = "exception occurred";
  }
}

void OBCameraNode::setAEStrategyCallback(const SetStringRequest& request,
                                         SetStringResponse& response) {
  try {
    if (device_->isPropertySupported(OB_PROP_DEVICE_AE_STRATEGY_INT, OB_PERMISSION_WRITE) &&
        (request.data == "default" || request.data == "motion")) {
      device_->setIntProperty(OB_PROP_DEVICE_AE_STRATEGY_INT, request.data == "motion" ? 1 : 0);
      ae_strategy_ = request.data;
      response.success = true;
      response.message = "set AE strategy success";
    } else {
      response.success = false;
      response.message = "set AE strategy failed";
    }
  } catch (...) {
    response.success = false;
    response.message = "exception occurred";
  }
}
}  // namespace orbbec_camera
