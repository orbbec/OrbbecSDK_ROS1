#include "orbbec_camera/ros_sensor.h"
#include "ros/ros.h"

namespace orbbec_camera {

ROSOBSensor::ROSOBSensor(std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor,
                         std::string name)
    : device_(std::move(device)), sensor_(std::move(sensor)), name_(std::move(name)) {
  CHECK_NOTNULL(sensor_.get());
}

ROSOBSensor::~ROSOBSensor() { stopStream(); }

void ROSOBSensor::startStream(std::shared_ptr<ob::StreamProfile> profile,
                              ob::FrameCallback callback) {
  if (is_started_) {
    return;
  }
  CHECK_NOTNULL(profile.get());
  sensor_->start(profile, std::move(callback));
  profile_ = profile;
  is_started_ = true;
}

void ROSOBSensor::stopStream() {
  if (!is_started_) {
    return;
  }
  sensor_->stop();
  is_started_ = false;
}

int ROSOBSensor::getExposure() {
  int data = 0;
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      data = device_->getIntProperty(OB_PROP_DEPTH_EXPOSURE_INT);
      break;
    case OB_SENSOR_COLOR:
      data = device_->getIntProperty(OB_PROP_COLOR_EXPOSURE_INT);
      break;
    case OB_SENSOR_IR:
      data = device_->getIntProperty(OB_PROP_IR_EXPOSURE_INT);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support get exposure");
      break;
  }
  return data;
}

void ROSOBSensor::setExposure(int data) {
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      device_->setIntProperty(OB_PROP_DEPTH_EXPOSURE_INT, data);
      break;
    case OB_SENSOR_COLOR:
      device_->setIntProperty(OB_PROP_COLOR_EXPOSURE_INT, data);
      break;
    case OB_SENSOR_IR:
      device_->setIntProperty(OB_PROP_IR_EXPOSURE_INT, data);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support set exposure");
      break;
  }
}

int ROSOBSensor::getGain() {
  int data = 0;
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      data = device_->getIntProperty(OB_PROP_DEPTH_GAIN_INT);
      break;
    case OB_SENSOR_IR:
      data = device_->getIntProperty(OB_PROP_IR_GAIN_INT);
      break;
    case OB_SENSOR_COLOR:
      data = device_->getIntProperty(OB_PROP_COLOR_GAIN_INT);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support get gain");
      break;
  }
  return data;
}

void ROSOBSensor::setGain(int data) {
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      device_->setIntProperty(OB_PROP_DEPTH_GAIN_INT, data);
      break;
    case OB_SENSOR_IR:
      device_->setIntProperty(OB_PROP_IR_GAIN_INT, data);
      break;
    case OB_SENSOR_COLOR:
      device_->setIntProperty(OB_PROP_COLOR_GAIN_INT, data);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support set gain");
      break;
  }
}

int ROSOBSensor::getWhiteBalance() {
  int data = 0;
  if (sensor_->type() == OB_SENSOR_COLOR) {
    data = device_->getIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT);
  } else {
    ROS_ERROR_STREAM(name_ << " does not support get white balance");
  }
  return data;
}

void ROSOBSensor::setWhiteBalance(int data) {
  if (sensor_->type() == OB_SENSOR_COLOR) {
    device_->setIntProperty(OB_PROP_COLOR_WHITE_BALANCE_INT, data);
  } else {
    ROS_ERROR_STREAM(name_ << " does not support set white balance");
  }
}

bool ROSOBSensor::getAutoWhiteBalance() {
  bool data = false;
  if (sensor_->type() == OB_SENSOR_COLOR) {
    data = device_->getBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL);
  } else {
    ROS_ERROR_STREAM(name_ << " does not support get auto white balance");
  }
  return data;
}

void ROSOBSensor::setAutoWhiteBalance(bool data) {
  if (sensor_->type() == OB_SENSOR_COLOR) {
    device_->setBoolProperty(OB_PROP_COLOR_AUTO_WHITE_BALANCE_BOOL, data);
  } else {
    ROS_ERROR_STREAM(name_ << " does not support set auto white balance");
  }
}

void ROSOBSensor::setAutoExposure(bool data) {
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      device_->setBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL, data);
      break;
    case OB_SENSOR_IR:
      device_->setBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL, data);
      break;
    case OB_SENSOR_COLOR:
      device_->setBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, data);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support set auto exposure");
      break;
  }
}

bool ROSOBSensor::getAutoExposure() {
  bool data = false;
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      data = device_->getBoolProperty(OB_PROP_DEPTH_AUTO_EXPOSURE_BOOL);
      break;
    case OB_SENSOR_IR:
      data = device_->getBoolProperty(OB_PROP_IR_AUTO_EXPOSURE_BOOL);
      break;
    case OB_SENSOR_COLOR:
      data = device_->getBoolProperty(OB_PROP_COLOR_AUTO_EXPOSURE_BOOL);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support set auto exposure");
      break;
  }
  return data;
}

OBSensorType ROSOBSensor::getSensorType() { return sensor_->type(); }

void ROSOBSensor::setMirror(bool data) {
  is_mirrored_ = data;
  switch (sensor_->type()) {
    case OB_SENSOR_DEPTH:
      device_->setBoolProperty(OB_PROP_DEPTH_MIRROR_BOOL, data);
      break;
    case OB_SENSOR_IR:
      device_->setBoolProperty(OB_PROP_IR_MIRROR_BOOL, data);
      break;
    case OB_SENSOR_COLOR:
      device_->setBoolProperty(OB_PROP_COLOR_MIRROR_BOOL, data);
      break;
    default:
      ROS_ERROR_STREAM(name_ << " does not support set mirror");
      break;
  }
}

bool ROSOBSensor::isMirrored() const { return is_mirrored_; }

std::shared_ptr<ob::StreamProfileList> ROSOBSensor::getStreamProfileList() const {
  return sensor_->getStreamProfileList();
}

}  // namespace orbbec_camera
