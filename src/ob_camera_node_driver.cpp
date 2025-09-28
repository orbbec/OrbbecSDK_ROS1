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
#include <iomanip>  // For std::put_time

#include <boost/filesystem.hpp>

#include <stdio.h>
#include <sys/ioctl.h>
#include <linux/usbdevice_fs.h>
#include <iostream>
#include <sstream>
#include <string>

namespace orbbec_camera {
backward::SignalHandling sh;

std::string g_camera_name = "camera";
static std::string g_current_device_uid = "";
const std::string GMSL_TYPE = "gmsl";
const std::string MIPI_TYPE = "mipi";

bool startsWith(const std::string &str, const std::string &prefix) { return str.find(prefix) == 0; }

std::pair<std::string, std::string> getFirstAndLastField(const std::string &str, char delimiter) {
  std::string firstField, lastField;
  size_t start = 0, end = str.find(delimiter);
  // Find the first field
  if (end != std::string::npos) {
    firstField = str.substr(start, end - start);
    start = end + 1;
    // Find the last field starting from after the first field
    end = str.find_last_of(delimiter, str.size() - 1);
    if (end != std::string::npos) {
      lastField = str.substr(end + 1);
    } else {
      // If no more delimiters are found, the rest of the string is the last field
      lastField = str.substr(start);
    }
  } else {
    // If no delimiter is found, the whole string is both the first and last field
    firstField = "";
    lastField = str;
  }
  return {firstField, lastField};
}

std::string get_device_bus_path() {
  std::cout << "Original UID: " << g_current_device_uid << std::endl;
  if (g_current_device_uid.empty()) {
    std::cerr << "Error: current_device_uid is empty" << std::endl;
    return "";
  }
  if (startsWith(g_current_device_uid, GMSL_TYPE) || startsWith(g_current_device_uid, MIPI_TYPE)) {
    std::cout << "The string startswith 'gmsl/mipi'. GMSL/MIPI device skip usbdevfs_reset"
              << std::endl;
    return "";
  }
  std::pair<std::string, std::string> result = getFirstAndLastField(g_current_device_uid, '-');
  std::string first_number = result.first;
  std::string last_number = result.second;
  if (first_number.empty() || last_number.empty()) {
    std::cerr << "Error: Failed to extract numbers from device UID" << std::endl;
    return "";
  }
  // Convert the number to a string and format it as a three-digit representation.
  auto format_number = [](const std::string &number) {
    std::ostringstream oss;
    oss << std::setw(3) << std::setfill('0') << number;
    return oss.str();
  };
  std::string first_number_str = format_number(first_number);
  std::string last_number_str = format_number(last_number);
  std::string device_bus_path = "/dev/bus/usb/" + first_number_str + "/" + last_number_str;
  std::cout << "First Number: " << first_number_str << " Last Number: " << last_number_str
            << std::endl;
  std::cout << "device_bus_path: " << device_bus_path << std::endl;
  return device_bus_path;
}

int usbdevfs_reset() {
  // std::string device_bus_path = "/dev/bus/usb/002/007";
  std::string device_bus_path = get_device_bus_path();
  if (device_bus_path.empty()) {
    std::cerr << "Error: device_bus_path is empty" << std::endl;
    return -1;
  }
  const char *filename = device_bus_path.c_str();
  std::cout << "usbdevfs_reset device bus: " << device_bus_path << std::endl;
  int fd = open(filename, O_WRONLY);
  if (fd < 0) {
    std::cerr << "Error opening device " << filename << ": " << strerror(errno) << std::endl;
    return -1;
  }
  std::cout << "Usbdevfs Resetting USB device " << filename << std::endl;
  if (ioctl(fd, USBDEVFS_RESET, 0) < 0) {
    std::cerr << "Error in ioctl: " << strerror(errno) << std::endl;
    close(fd);
    return -1;
  }
  std::cout << "Usbdevfs Reset successful" << std::endl;
  close(fd);
  return 0;
}

void signalHandler(int signum) {
  std::cout << "Received signal: " << signum << std::endl;
  if (signum == SIGINT || signum == SIGTERM) {
    ros::shutdown();
  } else {
    std::string log_dir = "Log/";

    // Get current time and format it.format as "2024_05_20_12_34_56"
    std::time_t now = std::time(nullptr);
    std::tm *local_time = std::localtime(&now);
    std::ostringstream time_stream;
    time_stream << std::put_time(local_time, "%Y_%m_%d_%H_%M_%S");
    std::string log_file_name = g_camera_name + "_crash_stack_trace_" + time_stream.str() + ".log";
    std::string log_file_path = log_dir + log_file_name;

    // Ensure log directory exists
    if (!boost::filesystem::exists(log_dir)) {
      if (!boost::filesystem::create_directories(log_dir)) {
        std::cerr << "Failed to create log directory: " << log_dir << std::endl;
        exit(signum);
      }
    }
    auto abs_path = boost::filesystem::absolute(log_dir);
    std::cout << "Log crash stack trace to " << abs_path.string() << "/" << log_file_name
              << std::endl;

    // Write stack trace to log file.
    {
      std::ofstream log_file(log_file_path, std::ios::app);
      if (log_file.is_open()) {
        log_file << "Received signal: " << signum << std::endl;
        backward::StackTrace st;
        st.load_here(32);  // Capture stack
        backward::Printer p;
        p.print(st, log_file);  // Print stack to log file
      } else {
        std::cerr << "Failed to open log file: " << log_file_path << std::endl;
      }
      log_file.close();
    }

    // usbdevfs reset before exit
    std::cout << "start usbdevfs reset " << std::endl;
    usbdevfs_reset();
    std::cout << "start usbdevfs reset finish" << std::endl;

    std::cout << "save crash stack trace log to file finish and exit program." << std::endl;
    exit(signum);  // Exit program
  }
}

OBCameraNodeDriver::OBCameraNodeDriver(ros::NodeHandle &nh, ros::NodeHandle &nh_private)
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
  struct sigaction sa;
  sa.sa_handler = signalHandler;
  sigemptyset(&sa.sa_mask);
  sa.sa_flags = SA_RESTART;
  sigaction(SIGSEGV, &sa, nullptr);
  sigaction(SIGABRT, &sa, nullptr);
  sigaction(SIGFPE, &sa, nullptr);
  sigaction(SIGILL, &sa, nullptr);
  sigaction(SIGINT, &sa, nullptr);
  sigaction(SIGTERM, &sa, nullptr);

  auto log_level = nh_private_.param<std::string>("log_level", "info");
  g_camera_name = nh_private_.param<std::string>("camera_name", "camera");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerToConsole(ob_log_level);

  orb_device_lock_shm_fd_ = shm_open(ORB_DEFAULT_LOCK_NAME.c_str(), O_CREAT | O_RDWR, 0666);
  if (orb_device_lock_shm_fd_ < 0) {
    ROS_ERROR_STREAM("Failed to open shared memory " << ORB_DEFAULT_LOCK_NAME);
    return;
  }
  int ret = ftruncate(orb_device_lock_shm_fd_, sizeof(pthread_mutex_t));
  if (ret < 0) {
    ROS_ERROR_STREAM("Failed to truncate shared memory " << ORB_DEFAULT_LOCK_NAME);
    close(orb_device_lock_shm_fd_);
    return;
  }
  orb_device_lock_shm_addr_ =
      static_cast<uint8_t *>(mmap(NULL, sizeof(pthread_mutex_t), PROT_READ | PROT_WRITE, MAP_SHARED,
                                  orb_device_lock_shm_fd_, 0));
  if (orb_device_lock_shm_addr_ == MAP_FAILED) {
    ROS_ERROR_STREAM("Failed to map shared memory " << ORB_DEFAULT_LOCK_NAME);
    close(orb_device_lock_shm_fd_);
    return;
  }
  pthread_mutexattr_init(&orb_device_lock_attr_);
  pthread_mutexattr_setpshared(&orb_device_lock_attr_, PTHREAD_PROCESS_SHARED);
  orb_device_lock_ = (pthread_mutex_t *)orb_device_lock_shm_addr_;
  if (pthread_mutex_init(orb_device_lock_, &orb_device_lock_attr_) != 0) {
    ROS_ERROR_STREAM("Failed to initialize shared mutex " << ORB_DEFAULT_LOCK_NAME);
    munmap(orb_device_lock_shm_addr_, sizeof(pthread_mutex_t));
    close(orb_device_lock_shm_fd_);
    return;
  }

  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  usb_port_ = nh_private_.param<std::string>("usb_port", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 100);
  device_num_ = static_cast<int>(nh_private_.param<int>("device_num", 1));
  enumerate_net_device_ = nh_private_.param<bool>("enumerate_net_device", false);
  ip_address_ = nh_private_.param<std::string>("ip_address", "");
  port_ = nh_private_.param<int>("port", 0);
  enable_hardware_reset_ = nh_private_.param<bool>("enable_hardware_reset", false);

  reboot_service_srv_ = nh_.advertiseService<std_srvs::EmptyRequest, std_srvs::EmptyResponse>(
      "reboot_device", [this](std_srvs::EmptyRequest &request, std_srvs::EmptyResponse &response) {
        return rebootDeviceServiceCallback(request, response);
      });
  ctx_->enableNetDeviceEnumeration(enumerate_net_device_);

  check_connection_timer_ =
      nh_.createWallTimer(ros::WallDuration(1.0),
                          [this](const ros::WallTimerEvent &) { this->checkConnectionTimer(); });
  ctx_->setDeviceChangedCallback([this](const std::shared_ptr<ob::DeviceList> &removed_list,
                                        const std::shared_ptr<ob::DeviceList> &added_list) {
    deviceConnectCallback(added_list);
    deviceDisconnectCallback(removed_list);
  });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  reset_device_thread_ = std::make_shared<std::thread>([this]() { resetDeviceThread(); });
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList> &list) {
  if (!list) {
    ROS_ERROR("Invalid device list provided");
    return nullptr;
  }
  if (list->deviceCount() == 0) {
    ROS_WARN("No device found");
    return nullptr;
  }

  std::shared_ptr<ob::Device> device = nullptr;
  if (!serial_number_.empty()) {
    ROS_INFO_STREAM("Connecting to device with serial number: " << serial_number_);
    device = selectDeviceBySerialNumber(list, serial_number_);
  } else if (!usb_port_.empty()) {
    ROS_INFO_STREAM("Connecting to device with usb port: " << usb_port_);
    device = selectDeviceByUSBPort(list, usb_port_);
  } else if (device_num_ == 1) {
    ROS_INFO_STREAM("Connecting to the default device");
    return list->getDevice(0);
  }
  if (device == nullptr) {
    ROS_WARN_THROTTLE(1.0, "Device with serial number %s not found", serial_number_.c_str());
    device_connected_ = false;
    return nullptr;
  }

  return device;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceBySerialNumber(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &serial_number) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  for (size_t i = 0; i < list->deviceCount(); i++) {
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
    } catch (ob::Error &e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.getMessage());
    } catch (std::exception &e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to get device info,Unknown exception");
    }
  }
  return nullptr;
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDeviceByUSBPort(
    const std::shared_ptr<ob::DeviceList> &list, const std::string &usb_port) {
  try {
    ROS_INFO_STREAM("selectDeviceByUSBPort : Before device lock lock");
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    ROS_INFO_STREAM("selectDeviceByUSBPort : After device lock lock");
    auto device = list->getDeviceByUid(usb_port.c_str());
    ROS_INFO_STREAM("selectDeviceByUSBPort : After getDeviceByUid");
    return device;
  } catch (ob::Error &e) {
    ROS_ERROR_STREAM("Failed to get device info " << e.getMessage());
  } catch (std::exception &e) {
    ROS_ERROR_STREAM("Failed to get device info " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("Failed to get device info");
  }

  return nullptr;
}

void OBCameraNodeDriver::initializeDevice(const std::shared_ptr<ob::Device> &device) {
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);

  // check device connected flag again after get lock
  if (device_connected_) {
    return;
  }

  if (device_) {
    ROS_WARN("device_ is not null, reset device_");
    device_.reset();
  }
  if (enable_hardware_reset_ && !hardware_reset_done_) {
    ROS_INFO("Reboot device");
    device->reboot();
    ROS_INFO("Reboot device done");
    device_connected_ = false;
    hardware_reset_done_ = true;
    return;
  }
  device_ = device;
  device_info_ = device_->getDeviceInfo();
  device_uid_ = device_info_->uid();
  ROS_INFO_STREAM("device uid: " << device_uid_);
  g_current_device_uid = device_info_->uid();
  ROS_INFO_STREAM("g_current_device_uid: " << g_current_device_uid);

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
  // if (!isOpenNIDevice(device_info_->pid())) {
  //   ctx_->enableDeviceClockSync(1800000);
  // }
  CHECK_NOTNULL(device_info_.get());
  std::string connection_type = device_info_->connectionType();
  ROS_INFO_STREAM("Device " << device_info_->name() << " connected");
  ROS_INFO_STREAM("Serial number: " << device_info_->serialNumber());
  ROS_INFO_STREAM("Firmware version: " << device_info_->firmwareVersion());
  ROS_INFO_STREAM("Hardware version: " << device_info_->hardwareVersion());
  ROS_INFO_STREAM("device uid: " << device_info_->uid());
  ROS_INFO_STREAM("device connection Type: " << connection_type);
  ROS_INFO_STREAM("Current node pid: " << getpid());
  if (device_info_->pid() == FEMTO_BOLT_PID) {
    if (connection_type != "USB3.0" && connection_type != "USB3.1" && connection_type != "USB3.2") {
      ROS_ERROR("Femto Bolt only supports USB3.0/3.1/3.2 connection, please reconnect the device");
      std::terminate();
    }
  }
}

void OBCameraNodeDriver::deviceConnectCallback(const std::shared_ptr<ob::DeviceList> &list) {
  ROS_INFO_STREAM("deviceConnectCallback : deviceConnectCallback start");
  CHECK_NOTNULL(list.get());
  if (device_connected_) {
    return;
  }
  if (list->deviceCount() == 0) {
    ROS_WARN("No device found");
    return;
  }
  ROS_INFO_STREAM("deviceCount: list->deviceCount() : " << list->deviceCount());

  bool start_device_failed = false;
  try {
    std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
    ROS_INFO_STREAM("deviceConnectCallback : connection_delay_ : " << connection_delay_);
    // use try lock to avoid deadlock
    int max_try_lock_count = 10;
    int try_lock_count = 0;
    bool is_orb_device_lock_locked = false;
    bool has_orb_openni_device = false;

    // check if has openni device
    for (size_t i = 0; i < list->deviceCount(); i++) {
      auto pid = list->pid(i);
      ROS_INFO_STREAM("deviceConnectCallback : device pid: " << pid);
      if (isOpenNIDevice(pid)) {
        ROS_INFO_STREAM("deviceConnectCallback : has_orb_openni_device pid: " << pid);
        has_orb_openni_device = true;
      }
    }
    if (has_orb_openni_device) {
      ROS_INFO_STREAM("deviceConnectCallback : Before process lock lock");
      for (; try_lock_count < max_try_lock_count; try_lock_count++) {
        if (pthread_mutex_trylock(orb_device_lock_) == 0) {
          ROS_INFO_STREAM("deviceConnectCallback : acquire orb_device_lock_ lock");
          is_orb_device_lock_locked = true;
          break;
        }
        ROS_INFO_STREAM("deviceConnectCallback : Try get lock orb_device_lock_, wait for 100ms");
        if (try_lock_count < 5) {
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
        } else {
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
      }
      if (try_lock_count == max_try_lock_count) {
        ROS_ERROR_STREAM(
            "deviceConnectCallback : has try maxcount, Failed get lock orb_device_lock_, return");
        return;
      }
      ROS_INFO_STREAM("deviceConnectCallback : After process lock lock");
    }

    std::shared_ptr<int> lock_guard(nullptr, [&](int *) {
      if (is_orb_device_lock_locked) {
        pthread_mutex_unlock(orb_device_lock_);
        ROS_INFO_STREAM("deviceConnectCallback : release orb_device_lock_ unlock");
      }
    });

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

    auto pid = device->getDeviceInfo()->pid();
    ROS_INFO_STREAM("deviceConnectCallback : current device pid: " << pid);
    if ((!isOpenNIDevice(pid)) && (is_orb_device_lock_locked)) {
      pthread_mutex_unlock(orb_device_lock_);
      is_orb_device_lock_locked = false;
      ROS_INFO_STREAM(
          "deviceConnectCallback : not openni device, direct release orb_device_lock_ unlock");
    }

    initializeDevice(device);
    ROS_INFO_STREAM("deviceConnectCallback : initializeDevice end");
  } catch (ob::Error &e) {
    start_device_failed = true;
    ROS_ERROR_STREAM("Failed to initialize device " << e.getMessage());
  } catch (std::exception &e) {
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

void OBCameraNodeDriver::connectNetDevice(const std::string &ip_address, int port) {
  if (ip_address.empty() || port == 0) {
    ROS_ERROR_STREAM("Invalid ip address or port");
    return;
  }
  ROS_INFO_STREAM("Connecting to net device " << ip_address << ":" << port);
  auto device = ctx_->createNetDevice(ip_address.c_str(), port);
  if (device == nullptr) {
    ROS_ERROR_STREAM("Failed to create net device");
    return;
  }
  initializeDevice(device);
}

void OBCameraNodeDriver::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_DEBUG_STREAM("wait for device " << serial_number_ << " to be connected");
  } else if (!ob_camera_node_) {
    device_connected_ = false;
  }
}

void OBCameraNodeDriver::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList> &device_list) {
  CHECK_NOTNULL(device_list.get());
  if (device_list->deviceCount() == 0) {
    ROS_DEBUG_STREAM("device list is empty");
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

OBLogSeverity OBCameraNodeDriver::obLogSeverityFromString(const std::string &log_level) {
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
  while (is_alive_ && ros::ok() && !device_connected_) {
    ROS_INFO_STREAM("queryDevice: first query device");
    if (enumerate_net_device_ && !ip_address_.empty() && port_ != 0) {
      ROS_INFO_STREAM("queryDevice: connect to net device " << ip_address_ << ":" << port_);
      connectNetDevice(ip_address_, port_);
    } else {
      auto list = ctx_->queryDeviceList();
      CHECK_NOTNULL(list.get());
      if (list->deviceCount() == 0) {
        ROS_WARN_STREAM("No device found, using callback to wait for device");
        return;
      }
      deviceConnectCallback(list);
      if (hardware_reset_done_) {
        break;
      }
    }
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

std::string OBCameraNodeDriver::parseUsbPort(const std::string &line) {
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

bool OBCameraNodeDriver::rebootDeviceServiceCallback(std_srvs::EmptyRequest &req,
                                                     std_srvs::EmptyResponse &res) {
  (void)req;
  (void)res;
  if (!device_connected_) {
    ROS_WARN("Device not connected");
    return false;
  }
  ROS_INFO("Reboot device");
  try {
    ob_camera_node_->rebootDevice();
    device_connected_ = false;
    device_ = nullptr;
    return true;
  } catch (const ob::Error &e) {
    ROS_WARN("Failed to reboot device (expected in some cases): %s", e.getMessage());
  } catch (const std::exception &e) {
    ROS_ERROR("Exception during reboot: %s", e.what());
  } catch (...) {
    ROS_ERROR("Unknown error occurred during reboot");
  }
  return false;
}
}  // namespace orbbec_camera
