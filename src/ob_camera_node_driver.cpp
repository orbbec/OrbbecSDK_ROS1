#include "orbbec_camera/ob_camera_node_driver.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/shm.h>

namespace orbbec_camera {
OBCameraNodeDriver::OBCameraNodeDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), ctx_(std::make_unique<ob::Context>()) {
  init();
}

OBCameraNodeDriver::~OBCameraNodeDriver() {
  is_alive_ = false;
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

void OBCameraNodeDriver::init() {
  is_alive_ = true;
  auto log_level = nh_private_.param<std::string>("log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerSeverity(ob_log_level);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 100);
  device_num_ = static_cast<size_t>(nh_private_.param<int>("device_num", 1));
  check_connection_timer_ = nh_.createWallTimer(
      ros::WallDuration(1.0), [this](const ros::WallTimerEvent&) { this->checkConnectionTimer(); });
  ctx_->setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removed_list,
                                        std::shared_ptr<ob::DeviceList> added_list) {
    (void)added_list;
    deviceDisconnectCallback(removed_list);
  });
  query_thread_ = std::make_shared<std::thread>([this]() { queryDevice(); });
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

  std::shared_ptr<int> sem_guard(nullptr, [&](int*) {
    if (device_num_ > 1 && device_sem) {
      ROS_INFO_STREAM("Release device semaphore");
      sem_post(device_sem);
      int sem_value = 0;
      sem_getvalue(device_sem, reinterpret_cast<int*>(&sem_value));
      ROS_INFO_STREAM("semaphore value: " << sem_value);
      ROS_INFO_STREAM("Release device semaphore done");
    }
  });

  ROS_INFO_STREAM("Connecting to device with serial number: " << serial_number_);

  int sem_value = 0;
  sem_getvalue(device_sem, reinterpret_cast<int*>(&sem_value));
  ROS_INFO_STREAM("semaphore value: " << sem_value);

  int ret = sem_wait(device_sem);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to wait semaphore " << strerror(errno));
    return nullptr;
  }

  std::shared_ptr<ob::Device> device = selectDeviceBySerialNumber(list, serial_number_);
  if (device == nullptr) {
    ROS_WARN("Device with serial number %s not found", serial_number_.c_str());
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
      if ((pid >= OPENNI_START_PID && pid <= OPENNI_END_PID) || pid == ASTRA_MINI_PID ||
          pid == ASTRA_MINI_S_PID) {
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
  ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_);
  device_connected_ = true;
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
  CHECK_NOTNULL(list);
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
    ROS_WARN("Device with serial number %s not found", serial_number_.c_str());
    device_connected_ = false;
    return;
  }
  initializeDevice(device);
}

void OBCameraNodeDriver::checkConnectionTimer() {
  if (!device_connected_) {
    ROS_INFO_STREAM("wait for device " << serial_number_ << " to be connected");
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
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    if (device_uid == device_uid_) {
      ob_camera_node_.reset();
      device_.reset();
      device_info_.reset();
      device_connected_ = false;
      device_uid_.clear();
      break;
    }
  }
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
      if (list->deviceCount() > 0) {
        try {
          startDevice(list);
        } catch (const ob::Error& e) {
          ROS_WARN_STREAM("Failed to start device: " << e.getMessage());
        } catch (const std::exception& e) {
          ROS_WARN_STREAM("Failed to start device: " << e.what());
        } catch (...) {
          ROS_WARN_STREAM("Failed to start device");
        }
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
  }
}

}  // namespace orbbec_camera
