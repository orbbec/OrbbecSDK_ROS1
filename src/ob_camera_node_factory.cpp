#include "orbbec_camera/ob_camera_node_factory.h"
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <semaphore.h>
#include <sys/shm.h>

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
  if (device_num_ > 1) {
    sem_unlink(DEFAULT_SEM_NAME.c_str());
    int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
    if (shm_id != -1) {
      shmctl(shm_id, IPC_RMID, nullptr);
    }
  }
}

void OBCameraNodeFactory::init() {
  is_alive_ = true;
  auto log_level = nh_private_.param<std::string>("log_level", "info");
  auto ob_log_level = obLogSeverityFromString(log_level);
  ctx_->setLoggerSeverity(ob_log_level);
  serial_number_ = nh_private_.param<std::string>("serial_number", "");
  connection_delay_ = nh_private_.param<int>("connection_delay", 1.0);
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

void OBCameraNodeFactory::startDevice(const std::shared_ptr<ob::DeviceList>& list) {
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
  std::this_thread::sleep_for(std::chrono::milliseconds (connection_delay_));
  size_t connected_device_num = 0;
  sem_t* device_sem = nullptr;
  std::shared_ptr<int> sem_guard(nullptr, [&](int*) {
    if (device_num_ > 1 && device_sem) {
      ROS_INFO_STREAM("Release device semaphore");
      sem_post(device_sem);
      int sem_value = 0;
      sem_getvalue(device_sem, reinterpret_cast<int*>(&sem_value));
      ROS_INFO_STREAM("semaphore value: " << sem_value);
      ROS_INFO_STREAM("Release device semaphore done");
      if (connected_device_num >= device_num_) {
        ROS_INFO_STREAM("All devices connected,  sem_unlink");
        sem_destroy(device_sem);
        sem_unlink(DEFAULT_SEM_NAME.c_str());
        ROS_INFO_STREAM("All devices connected,  sem_unlink done..");
      }
    }
  });
  if (device_num_ == 1) {
    ROS_INFO_STREAM("Connecting to the default device");
    device_ = list->getDevice(0);
  } else {
    std::string lower_sn;
    std::transform(serial_number_.begin(), serial_number_.end(), std::back_inserter(lower_sn),
                   [](auto ch) { return isalpha(ch) ? tolower(ch) : static_cast<int>(ch); });
    device_sem = sem_open(DEFAULT_SEM_NAME.c_str(), O_CREAT, 0644, 1);
    if (device_sem == SEM_FAILED) {
      ROS_ERROR_STREAM("Failed to open semaphore");
      return;
    }
    ROS_INFO_STREAM("Connecting to device with serial number: " << serial_number_);
    int sem_value = 0;
    sem_getvalue(device_sem, reinterpret_cast<int*>(&sem_value));
    ROS_INFO_STREAM("semaphore value: " << sem_value);
    int ret = sem_wait(device_sem);
    if (ret != 0) {
      ROS_ERROR_STREAM("Failed to wait semaphore " << strerror(errno));
      return;
    }
    try {
      auto device = list->getDeviceBySN(serial_number_.c_str());
      device_ = device;
    } catch (ob::Error& e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.getMessage());
    } catch (std::exception& e) {
      ROS_ERROR_STREAM("Failed to get device info " << e.what());
    } catch (...) {
      ROS_ERROR_STREAM("Failed to get device info");
    }

    if (device_ == nullptr) {
      ROS_WARN("Device with serial number %s not found", serial_number_.c_str());
      device_connected_ = false;
      return;
    } else {
      // write connected device info to file
      int shm_id = shmget(DEFAULT_SEM_KEY, 1, 0666 | IPC_CREAT);
      if (shm_id == -1) {
        ROS_ERROR_STREAM("Failed to create shared memory " << strerror(errno));
      } else {
        ROS_INFO_STREAM("Created shared memory");
        auto shm_ptr = (int*)shmat(shm_id, nullptr, 0);
        if (shm_ptr == (void*)-1) {
          ROS_ERROR_STREAM("Failed to attach shared memory " << strerror(errno));
        } else {
          ROS_INFO_STREAM("Attached shared memory");
          connected_device_num = *shm_ptr + 1;
          ROS_INFO_STREAM("Current connected device " << connected_device_num);
          *shm_ptr = static_cast<int>(connected_device_num);
          ROS_INFO_STREAM("Wrote to shared memory");
          shmdt(shm_ptr);
          if (connected_device_num >= device_num_) {
            ROS_INFO_STREAM("All devices connected, removing shared memory");
            shmctl(shm_id, IPC_RMID, nullptr);
          }
        }
      }
    }
  }
  CHECK_NOTNULL(device_.get());
  if (ob_camera_node_) {
    ob_camera_node_.reset();
  }
  ob_camera_node_ = std::make_unique<OBCameraNode>(nh_, nh_private_, device_);
  device_connected_ = true;
  device_info_ = device_->getDeviceInfo();
  CHECK_NOTNULL(device_info_.get());
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

void OBCameraNodeFactory::deviceDisconnectCallback(
    const std::shared_ptr<ob::DeviceList>& device_list) {
  if (device_list->deviceCount() == 0) {
    ROS_WARN_STREAM("device list is empty");
    return;
  }
  ROS_INFO("Device disconnected");
  if (device_info_ != nullptr) {
    ROS_INFO_STREAM("current node serial " << device_info_->serialNumber());
  }
  CHECK_NOTNULL(device_list.get());
  for (size_t i = 0; i < device_list->deviceCount(); i++) {
    auto serial = device_list->serialNumber(i);
    ROS_INFO_STREAM("Device " << serial << " disconnected");
    std::lock_guard<decltype(device_lock_)> lock(device_lock_);
    if (device_info_ != nullptr && serial == device_info_->serialNumber()) {
      ob_camera_node_.reset();
      device_.reset();
      device_info_.reset();
      device_connected_ = false;
      break;
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
