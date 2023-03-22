#include "orbbec_camera/ob_camera_node_driver.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/shm.h>
#include <ros/package.h>

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
  if (device_count_update_thread_ && device_count_update_thread_->joinable()) {
    device_count_update_thread_->join();
  }
  if (query_thread_ && query_thread_->joinable()) {
    query_thread_->join();
  }
  if (device_num_ > 1) {
    sem_unlink(DEFAULT_SEM_NAME.c_str());
    int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
    if (shm_id != -1) {
      shmctl(shm_id, IPC_RMID, nullptr);
    }
  }
}

void OBCameraNodeDriver::releaseDeviceSemaphore(sem_t* device_sem, int& num_devices_connected) {
  ROS_INFO_STREAM_THROTTLE(1.0, "Release device semaphore");
  sem_post(device_sem);
  int sem_value = 0;
  sem_getvalue(device_sem, &sem_value);
  ROS_INFO_STREAM_THROTTLE(1.0, "semaphore value: " << sem_value);
  ROS_INFO_STREAM_THROTTLE(1.0, "Release device semaphore done");
  if (num_devices_connected >= device_num_) {
    sem_destroy(device_sem);
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeDriver::updateConnectedDeviceCount(int& num_devices_connected,
                                                    DeviceConnectionEvent connection_event) {
  int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
  if (shm_id == -1) {
    ROS_ERROR_STREAM("Failed to create shared memory " << strerror(errno));
    return;
  }
  auto shm_ptr = (int*)shmat(shm_id, nullptr, 0);
  if (shm_ptr == (void*)-1) {
    ROS_ERROR_STREAM("Failed to attach shared memory " << strerror(errno));
    return;
  }
  if (connection_event == DeviceConnectionEvent::kDeviceConnected) {
    num_devices_connected = *shm_ptr + 1;
  } else if (connection_event == DeviceConnectionEvent::kDeviceDisconnected && *shm_ptr > 0) {
    num_devices_connected = *shm_ptr - 1;
  } else {
    num_devices_connected = *shm_ptr;
  }
  ROS_DEBUG_STREAM_THROTTLE(1.0, "Current connected device " << num_devices_connected);
  *shm_ptr = static_cast<int>(num_devices_connected);
  shmdt(shm_ptr);
  if (connection_event == DeviceConnectionEvent::kDeviceDisconnected &&
      num_devices_connected == 0) {
    shmctl(shm_id, IPC_RMID, nullptr);
    sem_unlink(DEFAULT_SEM_NAME.c_str());
  }
}

void OBCameraNodeDriver::init() {
  is_alive_ = true;
  auto log_level = nh_private_.param<std::string>("log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerSeverity(ob_log_level);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 100);
  device_num_ = static_cast<int>(nh_private_.param<int>("device_num", 1));
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                        std::shared_ptr<ob::DeviceList> added_list) {
    (void)added_list;
    deviceDisconnectCallback(removed_list);
  });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
  device_count_update_thread_ = std::make_shared<std::thread>([this]() { deviceCountUpdate(); });
}

std::shared_ptr<ob::Device> OBCameraNodeDriver::selectDevice(
    const std::shared_ptr<ob::DeviceList>& list) {
  if (device_num_ == 1) {
    ROS_INFO_STREAM("Connecting to the default device");
    return list->getDevice(0);
  }

  sem_t* device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
  if (device_sem == SEM_FAILED) {
    ROS_ERROR_STREAM("Failed to open semaphore");
    return nullptr;
  }
  ROS_INFO_STREAM_THROTTLE(1.0, "Connecting to device with serial number: " << serial_number_);

  int sem_value = 0;
  sem_getvalue(device_sem, reinterpret_cast<int*>(&sem_value));
  ROS_INFO_STREAM_THROTTLE(1.0, "semaphore value: " << sem_value);

  int ret = sem_wait(device_sem);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to wait semaphore " << strerror(errno));
    releaseDeviceSemaphore(device_sem, num_devices_connected_);
    return nullptr;
  }

  std::shared_ptr<ob::Device> device = selectDeviceBySerialNumber(list, serial_number_);
  std::shared_ptr<int> sem_guard(nullptr, [&, device](int*) {
    auto connect_event = device != nullptr ? DeviceConnectionEvent::kDeviceConnected
                                           : DeviceConnectionEvent::kOtherDeviceConnected;
    updateConnectedDeviceCount(num_devices_connected_, connect_event);

    releaseDeviceSemaphore(device_sem, num_devices_connected_);
  });
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

void OBCameraNodeDriver::initializeDevice(const std::shared_ptr<ob::Device>& device) {
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
  CHECK_NOTNULL(device_info_.get());
  ROS_INFO_STREAM("Device " << device_info_->name() << " connected");
  ROS_INFO_STREAM("Serial number: " << device_info_->serialNumber());
  ROS_INFO_STREAM("Firmware version: " << device_info_->firmwareVersion());
  ROS_INFO_STREAM("Hardware version: " << device_info_->hardwareVersion());
  ROS_INFO_STREAM("device type: " << ObDeviceTypeToString(device_info_->deviceType()));
  ROS_INFO_STREAM("device uid: " << device_info_->uid());
}

void OBCameraNodeDriver::startDevice(const std::shared_ptr<ob::DeviceList>& list) {
  CHECK_NOTNULL(list.get());
  std::lock_guard<decltype(device_lock_)> lock(device_lock_);
  if (device_connected_) {
    return;
  }
  if (list->deviceCount() == 0) {
    ROS_WARN("No device found");
    return;
  }
  if (device_) {
    device_.reset();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(connection_delay_));
  auto device = selectDevice(list);
  if (device == nullptr) {
    ROS_WARN_THROTTLE(1.0, "Device with serial number %s not found", serial_number_.c_str());
    device_connected_ = false;
    return;
  }
  initializeDevice(device);
}

void OBCameraNodeDriver::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_INFO_STREAM("wait for device " << serial_number_ << " to be connected");
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
  bool current_device_disconnected = false;
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    std::string device_uid = device_list->uid(i);
    ROS_INFO_STREAM("Device with uid " << device_uid << " disconnected");
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    if (device_uid == device_uid_) {
      ob_camera_node_.reset();
      device_.reset();
      device_info_.reset();
      device_connected_ = false;
      current_device_disconnected = true;
      device_uid_.clear();
      break;
    }
  }
  auto connect_event = current_device_disconnected
                           ? DeviceConnectionEvent::kDeviceDisconnected
                           : DeviceConnectionEvent::kOtherDeviceDisconnected;
  updateConnectedDeviceCount(num_devices_connected_, connect_event);
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
  while (is_alive_ && ros::ok()) {
    if (!device_connected_) {
      ROS_INFO_STREAM_THROTTLE(1, "query device");
      auto list = ctx_->queryDeviceList();
      CHECK_NOTNULL(list.get());
      if (list->deviceCount() == 0) {
        ROS_WARN_STREAM_THROTTLE(1, "No device found");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
      try {
        startDevice(list);
      } catch (const ob::Error& e) {
        ROS_WARN_STREAM("Failed to start device: " << e.getMessage());
      } catch (const std::exception& e) {
        ROS_WARN_STREAM("Failed to start device: " << e.what());
      } catch (...) {
        ROS_WARN_STREAM("Failed to start device");
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

void OBCameraNodeDriver::deviceCountUpdate() {
  while (is_alive_ && ros::ok()) {
    updateConnectedDeviceCount(num_devices_connected_, DeviceConnectionEvent::kDeviceCountUpdate);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
}
}  // namespace orbbec_camera
