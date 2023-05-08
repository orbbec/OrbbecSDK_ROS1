#include "orbbec_camera/ob_camera_node.h"
#include "orbbec_camera/utils.h"

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
  unit_step_size_[INFRA0] = sizeof(uint8_t);
  format_[INFRA0] = OB_FORMAT_Y16;
  image_format_[INFRA0] = CV_16UC1;
  encoding_[INFRA0] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA0] = "Y16";

  stream_name_[INFRA1] = "ir1";
  unit_step_size_[INFRA1] = sizeof(uint8_t);
  format_[INFRA1] = OB_FORMAT_Y16;
  image_format_[INFRA1] = CV_16UC1;
  encoding_[INFRA1] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA1] = "Y16";

  stream_name_[INFRA2] = "ir2";
  unit_step_size_[INFRA2] = sizeof(uint8_t);
  format_[INFRA2] = OB_FORMAT_Y16;
  image_format_[INFRA2] = CV_16UC1;
  encoding_[INFRA2] = sensor_msgs::image_encodings::MONO16;
  format_str_[INFRA2] = "Y16";
}

void OBCameraNode::setupDevices() {
  auto sensor_list = device_->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profiles = sensor->getStreamProfileList();
    for (size_t j = 0; j < profiles->count(); j++) {
      auto profile = profiles->getProfile(j);
      stream_index_pair sip{profile->type(), 0};
      if (sensors_.find(sip) != sensors_.end()) {
        continue;
      }
      sensors_[sip] = std::make_shared<ROSOBSensor>(device_, sensor, stream_name_[sip]);
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
  }
  if (enable_d2c_viewer_) {
    d2c_viewer_ = std::make_shared<D2CViewer>(nh_, nh_private_);
  }
  CHECK_NOTNULL(device_info_.get());
  if (enable_pipeline_) {
    pipeline_ = std::make_shared<ob::Pipeline>(device_);
  }
  try {
    if (enable_hardware_d2d_ && device_info_->pid() == GEMINI2_PID) {
      device_->setBoolProperty(OB_PROP_DISPARITY_TO_DEPTH_BOOL, true);
    }
    if (!depth_work_mode_.empty()) {
      device_->switchDepthWorkMode(depth_work_mode_.c_str());
    }
    if (sync_mode_ != OB_SYNC_MODE_CLOSE) {
      OBDeviceSyncConfig sync_config;
      sync_config.syncMode = sync_mode_;
      sync_config.irTriggerSignalInDelay = ir_trigger_signal_in_delay_;
      sync_config.rgbTriggerSignalInDelay = rgb_trigger_signal_in_delay_;
      sync_config.deviceTriggerSignalOutDelay = device_trigger_signal_out_delay_;
      device_->setSyncConfig(sync_config);
      // TODO: set up trigger_signal_out
    }
    if (device_info_->pid() == GEMINI2_PID) {
      auto default_precision_level = device_->getIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT);
      if (default_precision_level != depth_precision_) {
        device_->setIntProperty(OB_PROP_DEPTH_PRECISION_LEVEL_INT, depth_precision_);
      }
    }

    device_->setBoolProperty(OB_PROP_DEPTH_SOFT_FILTER_BOOL, enable_soft_filter_);
    device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, enable_color_auto_exposure_);
    device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, enable_ir_auto_exposure_);
    auto default_soft_filter_max_diff = device_->getIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT);
    if (soft_filter_max_diff_ != -1 && default_soft_filter_max_diff != soft_filter_max_diff_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_DIFF_INT, soft_filter_max_diff_);
    }
    auto default_soft_filter_speckle_size =
        device_->getIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT);
    if (soft_filter_speckle_size_ != -1 &&
        default_soft_filter_speckle_size != soft_filter_speckle_size_) {
      device_->setIntProperty(OB_PROP_DEPTH_MAX_SPECKLE_SIZE_INT, soft_filter_speckle_size_);
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
    default:
      return false;
  }
  return true;
}

void OBCameraNode::setupProfiles() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    auto profile_list = sensors_[stream_index]->getStreamProfileList();
    supported_profiles_[stream_index] = profile_list;
    auto selected_profile = profile_list->getVideoStreamProfile(
        width_[stream_index], height_[stream_index], format_[stream_index], fps_[stream_index]);
    auto default_profile = profile_list->getVideoStreamProfile(
        width_[stream_index], height_[stream_index], format_[stream_index]);
    if (!selected_profile) {
      ROS_WARN_STREAM("Given stream configuration is not supported by the device! "
                      << " Stream: " << stream_index.first << ", Stream Index: "
                      << stream_index.second << ", Width: " << width_[stream_index]
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
    images_[stream_index] = cv::Mat(height_[stream_index], width_[stream_index],
                                    image_format_[stream_index], cv::Scalar(0, 0, 0));
    ROS_INFO_STREAM(" stream " << stream_name_[stream_index] << " is enabled - width: "
                               << width_[stream_index] << ", height: " << height_[stream_index]
                               << ", fps: " << fps_[stream_index] << ", "
                               << "Format: " << selected_profile->format());
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
}

void OBCameraNode::setupTopics() {
  setupPublishers();
  if (publish_tf_) {
    publishStaticTransforms();
  }
}

void OBCameraNode::setupPublishers() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (!enable_stream_[stream_index]) {
      continue;
    }
    std::string name = stream_name_[stream_index];
    std::string topic_name = "/" + camera_name_ + "/" + name + "/image_raw";
    ros::SubscriberStatusCallback image_subscribed_cb =
        boost::bind(&OBCameraNode::imageSubscribedCallback, this, stream_index);
    ros::SubscriberStatusCallback image_unsubscribed_cb =
        boost::bind(&OBCameraNode::imageUnsubscribedCallback, this, stream_index);
    image_publishers_[stream_index] = nh_.advertise<sensor_msgs::Image>(
        topic_name, 1, image_subscribed_cb, image_unsubscribed_cb);
    topic_name = "/" + camera_name_ + "/" + name + "/camera_info";
    camera_info_publishers_[stream_index] =
        nh_.advertise<sensor_msgs::CameraInfo>(topic_name, 1, true);
  }
  if (enable_stream_[DEPTH] && enable_stream_[COLOR]) {
    // extrinsics_publisher_ = nh_.advertise<Extrinsics>("extrinsic/depth_to_color", 1, true);
  }
  if (enable_point_cloud_) {
    ros::SubscriberStatusCallback depth_cloud_subscribed_cb =
        boost::bind(&OBCameraNode::pointCloudSubscribedCallback, this);
    ros::SubscriberStatusCallback depth_cloud_unsubscribed_cb =
        boost::bind(&OBCameraNode::pointCloudUnsubscribedCallback, this);
    depth_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "depth/points", 1, depth_cloud_subscribed_cb, depth_cloud_unsubscribed_cb);
  }
  if (enable_colored_point_cloud_) {
    ros::SubscriberStatusCallback depth_registered_cloud_subscribed_cb =
        boost::bind(&OBCameraNode::coloredPointCloudSubscribedCallback, this);
    ros::SubscriberStatusCallback depth_registered_cloud_unsubscribed_cb =
        boost::bind(&OBCameraNode::coloredPointCloudUnsubscribedCallback, this);
    depth_registered_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(
        "depth_registered/points", 1, depth_registered_cloud_subscribed_cb,
        depth_registered_cloud_unsubscribed_cb);
  }
}

void OBCameraNode::setupCameraInfo() {
  auto param = getCameraParam();
  if (param) {
    int base_depth_width = param->depthIntrinsic.width == 0 ? 640 : param->depthIntrinsic.width;
    int base_rgb_width = param->rgbIntrinsic.width == 0 ? 640 : param->rgbIntrinsic.width;
    camera_infos_[DEPTH] =
        convertToCameraInfo(param->depthIntrinsic, param->depthDistortion, base_depth_width);
    camera_infos_[INFRA0] =
        convertToCameraInfo(param->depthIntrinsic, param->depthDistortion, base_depth_width);
    camera_infos_[COLOR] =
        convertToCameraInfo(param->rgbIntrinsic, param->rgbDistortion, base_rgb_width);
  } else {
    ROS_WARN_STREAM("Failed to get camera parameters");
  }
}

void OBCameraNode::setupPipelineConfig() {
  if (pipeline_config_) {
    pipeline_config_.reset();
  }
  pipeline_config_ = std::make_shared<ob::Config>();
  if (depth_registration_ && enable_stream_[COLOR] && enable_stream_[DEPTH]) {
    pipeline_config_->setAlignMode(ALIGN_D2C_HW_MODE);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      pipeline_config_->enableStream(stream_profile_[stream_index]);
    }
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
      ROS_ERROR_STREAM("get gain error " << e.getMessage());
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
      ROS_INFO_STREAM("stream " << stream_name_[stream_index] << " exposure " << exposure);
      default_exposure_[stream_index] = exposure;
    } catch (ob::Error& e) {
      default_exposure_[stream_index] = 0;
      ROS_ERROR_STREAM("get exposure error " << e.getMessage());
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
    ROS_WARN_STREAM("get white balance error " << e.getMessage());
  }
}
}  // namespace orbbec_camera
