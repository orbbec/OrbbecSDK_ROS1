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

#include "orbbec_camera/ob_camera_node_driver.h"
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <ros/package.h>
#include <regex>
#include <sys/mman.h>

namespace orbbec_camera {
OBCameraNodeDriver::OBCameraNodeDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      config_path_(ros::package::getPath("orbbec_camera") + "/config/OrbbecSDKConfig_v1.0.xml"),
      ctx_(std::make_shared<ob::Context>(config_path_.c_str())) {
  init();
}

OBCameraNodeDriver::~OBCameraNodeDriver() {
  is_alive_ = false;
  if (reset_device_thread_ && reset_device_thread_->joinable()) {
    reset_device_cv_.notify_all();
    reset_device_thread_->join();
  }
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
}

void OBCameraNodeDriver::init() {
  is_alive_ = true;
  auto log_level = nh_private_.param<std::string>("log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerSeverity(ob_log_level);
  orb_device_lock_shm_fd_ = shm_open(ORB_DEFAULT_LOCK_NAME.c_str(), O_CREAT | O_RDWR, 0666);
  if (orb_device_lock_shm_fd_ < 0) {
    ROS_ERROR_STREAM("Failed to open shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  int ret = ftruncate(orb_device_lock_shm_fd_, sizeof(pthread_mutex_t));
  if (ret < 0) {
    ROS_ERROR_STREAM("Failed to truncate shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  orb_device_lock_shm_addr_ =
      static_cast<uint8_t*>(mmap(NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED,
                                 orb_device_lock_shm_fd_, 0));
  if (orb_device_lock_shm_addr_ == MAP_FAILED) {
    ROS_ERROR_STREAM("Failed to map shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  pthread_mutexattr_init(&orb_device_lock_attr_);
  pthread_mutexattr_setpshared(&orb_device_lock_attr_, PTHREAD_PROCESS_SHARED);
  orb_device_lock_ = (pthread_mutex_t*)orb_device_lock_shm_addr_;
  pthread_mutex_init(orb_device_lock_, &orb_device_lock_attr_);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  usb_port_ = nh_private_.param<std::string>("usb_port", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 100);
  device_num_ = static_cast<int>(nh_private_.param<int>("device_num", 1));
  auto enumerate_net_device_ =
      static_cast<int>(nh_private_.param<bool>("enumerate_net_device", false));
  ctx_->enableNetDeviceEnumeration(enumerate_net_device_);
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  ctx_->setDeviceChangedCallback([this](const std::shared_ptr<ob::DeviceList>& removed_list,
                                        const std::shared_ptr<ob::DeviceList>& added_list) {
    deviceConnectCallback(added_list);
    deviceDisconnectCallback(removed_list);
  });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDeviceThread(); });
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList>& list) {
  if (device_num_ == 1) {
    ROS_INFO_STREAM("Connecting to the default device");
    return list->getDevice(0);
  }

  std::shared_ptr<ob::Device> device = nullptr;
  if (!serial_number_.empty()) {
    ROS_INFO_STREAM("Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    ROS_INFO_STREAM("Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  }
  if (device == nullptr) {
    ROS_WARN_THROTTLE(1.0, "Device with serial number %s not found", serial_number_.c_str());
    device_connected_ = false;
    return nullptr;
  }

  return device;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceBySerialNumber(
    const std::shared_ptr<ob::DeviceList>& list, const std::string& serial_number) {
  for (size_t i = 0; i < list->deviceCount(); i++) {
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    try {
      auto pid = list->pid(i);
      if (isOpenNIDevice(pid)) {
        // openNI device
        auto dev = list->getDevice(i);
        auto device_info = dev->getDeviceInfo();
        if (device_info->serialNumber() == serial_number) {
          ROS_INFO_STREAM("Device serial number " << device_info->serialNumber() << " matched");
          return dev;
        }
      } else {
        std::string sn = list->serialNumber(i);
        ROS_INFO_STREAM("Device serial number: " << sn);
        if (sn == serial_number) {
          ROS_INFO_STREAM("Device serial number <<" << sn << " matched");
          return list->getDevice(i);
        }
      }
    } catch (ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.getMessage());
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to get device info");
    }
  }
  return nullptr;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByUSBPort(
    const std::shared_ptr<ob::DeviceList>& list, const std::string& usb_port) {
  try {
    ROS_INFO_STREAM("selectDeviceByUSBPort : Before device lock lock");
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    ROS_INFO_STREAM("selectDeviceByUSBPort : After device lock lock");
    auto device = list->getDeviceByUid(usb_port.c_str());
    ROS_INFO_STREAM("selectDeviceByUSBPort : After getDeviceByUid");
    return device;
  } catch (ob::Error& e) {
    ROS_ERROR_STREAM("Failed to get device info " << e.getMessage());
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Failed to get device info " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("Failed to get device info");
  }

  return nullptr;
}

void OBCameraNodeDriver::initializeDevice(const std::shared_ptr<ob::Device>& device) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (device_) {
    ROS_WARN("device_ is not null, reset device_");
    device_.reset();
  }
  device_ = device;
  CHECK_NOTNULL(device_.get());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  ob_camera_node_ = std::make_shared<OBCameraNode>(nh_, nh_private_, device_);
  if (ob_camera_node_ && ob_camera_node_->isInitialized()) {
    device_connected_ = true;
  } else {
    device_connected_ = false;
    ob_camera_node_.reset();
    return;
  }
  device_info_ = device_->getDeviceInfo();
  device_uid_ = device_info_->uid();
  if (!isOpenNIDevice(device_info_->pid())) {
    ctx_->enableDeviceClockSync(5000);
  }
  CHECK_NOTNULL(device_info_.get());
  ROS_INFO_STREAM("Device " << device_info_->name() << " connected");
  ROS_INFO_STREAM("Serial number: " << device_info_->serialNumber());
  ROS_INFO_STREAM("Firmware version: " << device_info_->firmwareVersion());
  ROS_INFO_STREAM("Hardware version: " << device_info_->hardwareVersion());
  ROS_INFO_STREAM("device uid: " << device_info_->uid());
}

void OBCameraNodeDriver::deviceConnectCallback(const std::shared_ptr<ob::DeviceList>& list) {
  ROS_INFO_STREAM("deviceConnectCallback : deviceConnectCallback start");
  CHECK_NOTNULL(list.get());
  if (device_connected_) {
    return;
  }
  if (list->deviceCount() == 0) {
    ROS_WARN("No device found");
    return;
  }
  bool start_device_failed = false;
  try {
    std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
    ROS_INFO_STREAM("deviceConnectCallback : Before process lock lock");
    pthread_mutex_lock(orb_device_lock_);
    ROS_INFO_STREAM("deviceConnectCallback : After process lock lock");
    std::shared_ptr<int> lock_guard(nullptr,
                                    [this](int*) { pthread_mutex_unlock(orb_device_lock_); });
    ROS_INFO_STREAM("deviceConnectCallback : selectDevice start");
    auto device = selectDevice(list);
    ROS_INFO_STREAM("deviceConnectCallback : selectDevice end");
    if (device == nullptr) {
      if (!serial_number_.empty()) {
        ROS_WARN_THROTTLE(1.0, "Device with serial number %s not found", serial_number_.c_str());
      } else if (!usb_port_.empty()) {
        ROS_WARN_THROTTLE(1.0, "Device with usb port %s not found", usb_port_.c_str());
      }
      device_connected_ = false;
      ROS_WARN_STREAM("deviceConnectCallback : start device failed, return");
      return;
    }
    initializeDevice(device);
  } catch (ob::Error& e) {
    start_device_failed = true;
    ROS_ERROR_STREAM("Failed to initialize device " << e.getMessage());
  } catch (std::exception& e) {
    start_device_failed = true;
    ROS_ERROR_STREAM("Failed to initialize device " << e.what());
  } catch (...) {
    start_device_failed = true;
    ROS_ERROR_STREAM("Failed to initialize device");
  }
  if (start_device_failed) {
    std::unique_lock<decltype(reset_device_lock_)> reset_lock(reset_device_lock_);
    reset_device_ = true;
    reset_device_cv_.notify_all();
  }
  ROS_INFO_STREAM("deviceConnectCallback : deviceConnectCallback end");
}

void OBCameraNodeDriver::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_DEBUG_STREAM("wait for device " << serial_number_ << " to be connected");
  } else if (!ob_camera_node_) {
    device_connected_ = false;
  }
}

void OBCameraNodeDriver::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList>& device_list) {
  CHECK_NOTNULL(device_list.get());
  if (device_list->deviceCount() == 0) {
    ROS_WARN_STREAM("device list is empty");
    return;
  }
  ROS_INFO("Device disconnected");
  if (device_info_ != nullptr) {
    ROS_INFO_STREAM("current node serial " << device_info_->serialNumber());
  }
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string device_uid = device_list->uid(i);
    ROS_INFO_STREAM("Device with uid " << device_uid << " disconnected");
    if (device_uid == device_uid_) {
      ROS_INFO_STREAM("deviceDisconnectCallback : Before reset device, wait for device lock");
      std::unique_lock<decltype(reset_device_lock_)> reset_lock(reset_device_lock_);
      reset_device_ = true;
      reset_device_cv_.notify_all();
      ROS_INFO_STREAM(device_uid << " reset device " << device_uid << " notification sent");
      break;
    }
  }
  ROS_INFO_STREAM("deviceDisconnectCallback : deviceDisconnectCallback end");
}

OBLogSeverity OBCameraNodeDriver::obLogSeverityFromString(const std::string& log_level) {
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

void OBCameraNodeDriver::queryDevice() {
  if (!device_connected_) {
    ROS_INFO_STREAM("queryDevice: first query device");
    auto list = ctx_->queryDeviceList();
    CHECK_NOTNULL(list.get());
    if (list->deviceCount() == 0) {
      ROS_WARN_STREAM("No device found");
      return;
    }
    deviceConnectCallback(list);
  }
}

void OBCameraNodeDriver::resetDeviceThread() {
  while (is_alive_ && ros::ok()) {
    std::unique_lock<decltype(reset_device_lock_)> lock(reset_device_lock_);
    reset_device_cv_.wait(lock, [this]() { return !is_alive_ || reset_device_; });
    if (!is_alive_) {
      break;
    }
    ROS_INFO_STREAM("resetDeviceThread: device is disconnected, reset device start");
    {
      std::lock_guard<decltype(device_lock_)> device_lock(device_lock_);
      ob_camera_node_.reset();
      ROS_INFO_STREAM("resetDeviceThread: device is disconnected, reset device");
      device_.reset();
      device_info_.reset();
      device_connected_ = false;
      device_uid_.clear();
      reset_device_ = false;
    }
    ROS_INFO_STREAM("resetDeviceThread: device is disconnected, reset device end");
  }
}
std::string OBCameraNodeDriver::parseUsbPort(const std::string& line) {
  std::string port_id;
  std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*",
                        std::regex_constants::ECMAScript);
  std::smatch base_match;
  bool found = std::regex_match(line, base_match, self_regex);
  if (found) {
    port_id = base_match[1].str();
    if (base_match[2].str().empty())  // This is libuvc string. Remove counter is exists.
    {
      std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
      bool found_end = std::regex_match(port_id, base_match, end_regex);
      if (found_end) {
        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
      }
    }
  }
  return port_id;
}
}  // namespace orbbec_camera
