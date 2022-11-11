#include "orbbec_camera/ob_camera_node_factory.h"
namespace orbbec_camera {
OBCameraNodeFactory::OBCameraNodeFactory(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), ctx_(std::make_unique<ob::Context>()) {
  init();
}

OBCameraNodeFactory::~OBCameraNodeFactory() {
  is_alive_ = false;
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
}

void OBCameraNodeFactory::init() {
  is_alive_ = true;
  auto log_level = nh_private_.param<std::string>("log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerSeverity(ob_log_level);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 1.0);
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                        std::shared_ptr<ob::DeviceList> added_list) {
    deviceDisconnectCallback(removed_list);
    deviceConnectCallback(added_list);
  });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
}

void OBCameraNodeFactory::startDevice(const std::shared_ptr<ob::DeviceList>& list) {
  if (device_) {
    return;
  }
  if (list->deviceCount() == 0) {
    ROS_WARN("No device found");
    return;
  }
  std::this_thread::sleep_for(std::chrono::seconds(connection_delay_));
  if (serial_number_.empty()) {
    device_ = list->getDevice(0);
    device_info_ = device_->getDeviceInfo();
  } else {
    device_ = list->getDeviceBySN(serial_number_.c_str());
    if (!device_) {
      std::string lower_sn;
      std::transform(serial_number_.begin(), serial_number_.end(), std::back_inserter(lower_sn),
                     [](auto ch) { return isalpha(ch) ? tolower(ch) : static_cast<int>(ch); });
      device_ = list->getDeviceBySN(lower_sn.c_str());
    }
    if (device_ == nullptr) {
      ROS_WARN("Device with serial number %s not found", serial_number_.c_str());
      return;
    }
  }
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_);
  device_connected_ = true;
  ROS_INFO_STREAM("Device " << device_info_->name() << " connected");
  ROS_INFO_STREAM("Serial number: " << device_info_->serialNumber());
  ROS_INFO_STREAM("Firmware version: " << device_info_->firmwareVersion());
  ROS_INFO_STREAM("Hardware version: " << device_info_->hardwareVersion());
  ROS_INFO_STREAM("device type: " << ObDeviceTypeToString(device_info_->deviceType()));
}

void OBCameraNodeFactory::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_INFO_STREAM("wait for device " << serial_number_ << " to be connected");
  }
}

void OBCameraNodeFactory::deviceConnectCallback(
    const std::shared_ptr<ob::DeviceList>& device_list) {
  if (device_list->deviceCount() == 0) {
    ROS_WARN_STREAM("device list is empty");
    return;
  }
  ROS_INFO_STREAM("device connected");
  CHECK_NOTNULL(device_list.get());
  if (!device_) {
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    startDevice(device_list);
  }
}

void OBCameraNodeFactory::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList>& device_list) {
  if (device_list->deviceCount() == 0) {
    ROS_WARN_STREAM("device list is empty");
    return;
  }
  ROS_INFO("Device disconnected");
  CHECK_NOTNULL(device_list.get());
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    auto serial = device_list->serialNumber(i);
    if (serial == device_info_->serialNumber()) {
      ob_camera_node_.reset();
      device_.reset();
    }
  }
}

OBLogSeverity OBCameraNodeFactory::obLogSeverityFromString(const std::string& log_level) {
  if (log_level == "debug") {
    return OBLogSeverity::OB_LOG_SEVERITY_DEBUG;
  } else if (log_level == "warn") {
    return OBLogSeverity::OB_LOG_SEVERITY_WARN;
  } else if (log_level == "error") {
    return OBLogSeverity::OB_LOG_SEVERITY_ERROR;
  } else if (log_level == "fatal") {
    return OBLogSeverity::OB_LOG_SEVERITY_FATAL;
  } else if (log_level == "info") {
    return OBLogSeverity::OB_LOG_SEVERITY_INFO;
  } else {
    return OBLogSeverity::OB_LOG_SEVERITY_NONE;
  }
}

void OBCameraNodeFactory::queryDevice() {
  while (is_alive_ && !device_) {
    auto list = ctx_->queryDeviceList();
    if (list->deviceCount() > 0) {
      std::lock_guard<decltype(device_lock_)> lock(device_lock_);
      startDevice(list);
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace orbbec_camera
