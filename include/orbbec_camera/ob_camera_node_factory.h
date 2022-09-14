#pragma once
#include "ob_camera_node.h"
#include "point_cloud_proc/point_cloud_proc.h"

namespace orbbec_camera {
class OBCameraNodeFactory {
 public:
  explicit OBCameraNodeFactory(ros::NodeHandle& nh, ros::NodeHandle& nh_private);

  ~OBCameraNodeFactory();

 private:
  void init();

  void startDevice(const std::shared_ptr<ob::DeviceList>& list);

  void checkConnectionTimer();

  void deviceConnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  void deviceDisconnectCallback(const std::shared_ptr<ob::DeviceList>& device_list);

  static OBLogSeverity obLogSeverityFromString(const std::string& log_level);

  void queryDevice();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::unique_ptr<ob::Context> ctx_ = nullptr;
  std::unique_ptr<OBCameraNode> ob_camera_node_ = nullptr;
  std::unique_ptr<PointCloudXyzNode> point_cloud_xyz_node_ = nullptr;
  std::unique_ptr<PointCloudXyzrgbNode> point_cloud_xyzrgb_node_ = nullptr;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
  ros::WallTimer check_connection_timer_;
  std::atomic_bool device_connected_{false};
  std::atomic_bool is_alive_{false};
  std::string serial_number_;
  std::string log_level_;
  bool enable_point_cloud_ = true;
  bool enable_point_cloud_xyzrgb_ = true;
  int connection_delay_ = 1.0;
  std::shared_ptr<std::thread> query_thread_ = nullptr;
};
}  // namespace orbbec_camera
