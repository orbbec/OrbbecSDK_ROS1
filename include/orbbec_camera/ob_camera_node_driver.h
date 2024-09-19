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

#pragma once
#include "ob_camera_node.h"
#include <thread>
#include <mutex>
#include <semaphore.h>
#include <pthread.h>
#define BACKWARD_HAS_DW 1
#include <backward-cpp/backward.hpp>

namespace orbbec_camera {

class OBCameraNodeDriver {
 public:
  explicit OBCameraNodeDriver(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  ~OBCameraNodeDriver();

 private:
  void init();

  std::shared_ptr<ob::Device> selectDevice(const std::shared_ptr<ob::DeviceList>& list);

  bool rebootDeviceServiceCallback(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res);

  std::shared_ptr<ob::Device> selectDeviceBySerialNumber(
      const std::shared_ptr<ob::DeviceList>& list, const std::string& serial_number);
  std::shared_ptr<ob::Device> selectDeviceByUSBPort(const std::shared_ptr<ob::DeviceList>& list,
                                                    const std::string& usb_port);

  void initializeDevice(const std::shared_ptr<ob::Device>& device);

  void deviceConnectCallback(const std::shared_ptr<ob::DeviceList>& list);

  void connectNetDevice(const std::string& ip_address, int port);

  void checkConnectionTimer();

  void deviceDisconnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string& log_level);

  void queryDevice();

  void resetDeviceThread();

  static std::string parseUsbPort(const std::string& line);

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
  std::recursive_mutex device_lock_;
  int device_num_ = 1;
  bool enumerate_net_device_ = false;
  std::shared_ptr<std::thread> reset_device_thread_ = nullptr;
  std::condition_variable reset_device_cv_;
  std::atomic_bool reset_device_{false};
  std::mutex reset_device_lock_;
  pthread_mutex_t* orb_device_lock_ = nullptr;
  pthread_mutexattr_t orb_device_lock_attr_;
  uint8_t* orb_device_lock_shm_addr_ = nullptr;
  int orb_device_lock_shm_fd_ = -1;
  // net work config
  std::string ip_address_;
  int port_ = 0;
  ros::ServiceServer reboot_service_srv_;
  static backward::SignalHandling sh;
  bool enable_hardware_reset_ = false;
  bool hardware_reset_done_ = false;
};
}  // namespace orbbec_camera
