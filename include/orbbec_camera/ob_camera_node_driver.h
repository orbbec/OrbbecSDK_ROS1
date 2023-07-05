#pragma once
#include "ob_camera_node.h"
#include <thread>
#include <mutex>
#include <semaphore.h>

namespace orbbec_camera {
enum DeviceConnectionEvent {
  kDeviceConnected = 0,
  kDeviceDisconnected,
  kOtherDeviceConnected,
  kOtherDeviceDisconnected,
  kDeviceCountUpdate,
};

class OBCameraNodeDriver {
 public:
  explicit OBCameraNodeDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  ~OBCameraNodeDriver();

 private:
  void init();

  void releaseDeviceSemaphore(sem_t* device_sem, int& num_devices_connected) const;

  static void updateConnectedDeviceCount(int& num_devices_connected,
                                         DeviceConnectionEvent connection_event);

  std::shared_ptr<ob::Device> selectDevice(const std::shared_ptr<ob::DeviceList>& list);

  static std::shared_ptr<ob::Device> selectDeviceBySerialNumber(
      const std::shared_ptr<ob::DeviceList>& list, const std::string& serial_number);
  static std::shared_ptr<ob::Device> selectDeviceByUSBPort(
      const std::shared_ptr<ob::DeviceList>& list, const std::string& usb_port);

  void initializeDevice(const std::shared_ptr<ob::Device>& device);

  void startDevice(const std::shared_ptr<ob::DeviceList>& list);

  void checkConnectionTimer();

  void deviceDisconnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string& log_level);

  void queryDevice();

  void deviceCountUpdate();

  void syncTimeThread();

  static std::string parseUsbPort(const std::string& line);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::string config_path_;
  std::shared_ptr<ob::Context> ctx_ = nullptr;
  std::shared_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
  ros::WallTimer check_connection_timer_;
  std::atomic_bool device_connected_{false};
  std::atomic_bool is_alive_{false};
  std::string serial_number_;
  std::string device_uid_;
  std::string log_level_;
  std::string usb_port_;
  int connection_delay_ = 1.0;
  std::shared_ptr<std::thread> query_thread_ = nullptr;
  std::shared_ptr<std::thread> device_count_update_thread_ = nullptr;
  std::recursive_mutex device_lock_;
  int device_num_ = 1;
  int num_devices_connected_ = 0;
  std::shared_ptr<std::thread> sync_time_thread_ = nullptr;
};
}  // namespace orbbec_camera