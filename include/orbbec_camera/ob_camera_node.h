#pragma once
#include "types.h"
#include "utils.h"
#include "ros_sensor.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <thread>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "orbbec_camera/d2c_viewer.h"
#include "orbbec_camera/GetCameraParams.h"

namespace orbbec_camera {
class OBCameraNode {
 public:
  OBCameraNode(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
               std::shared_ptr<ob::Device> device);
  OBCameraNode(const OBCameraNode&) = delete;
  OBCameraNode& operator=(const OBCameraNode&) = delete;
  OBCameraNode(OBCameraNode&&) = delete;
  OBCameraNode& operator=(OBCameraNode&&) = delete;
  ~OBCameraNode();

 private:
  void init();

  void setupCameraCtrlServices();

  void setupConfig();

  void getParameters();

  void setupDevices();

  void setupFrameCallback();

  void onNewFrameCallback(std::shared_ptr<ob::Frame> frame, const stream_index_pair& stream_index);

  void onNewFrameSetCallback(std::shared_ptr<ob::FrameSet> frame_set);

  void publishPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishDepthPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  void publishColorPointCloud(std::shared_ptr<ob::FrameSet> frame_set);

  bool setupFormatConvertType(OBFormat type);

  void setupProfiles();

  void setupTopics();

  void setupPipelineConfig();

  void setupPublishers();

  void publishStaticTF(const ros::Time& t, const std::vector<float>& trans,
                       const tf2::Quaternion& q, const std::string& from, const std::string& to);

  void startStreams();

  void stopStreams();

  void startStream(const stream_index_pair& stream_index);

  void stopStream(const stream_index_pair& stream_index);

  void imageSubscribedCallback(const stream_index_pair& stream_index);

  void imageUnsubscribedCallback(const stream_index_pair& stream_index);

  void pointCloudXYZSubscribedCallback();

  void pointCloudXYZUnsubscribedCallback();

  void pointCloudXYZRGBSubscribedCallback();

  void pointCloudXYZRGBUnsubscribedCallback();

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  boost::optional<OBCameraParam> getCameraParam();

  int getCameraParamIndex();

  void setupCameraInfo();

  // camera control services
  bool setMirrorCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response,
                         const stream_index_pair& stream_index);

  bool getExposureCallback(GetInt32Request& request, GetInt32Response& response,
                           const stream_index_pair& stream_index);

  bool setExposureCallback(SetInt32Request& request, SetInt32Response& response,
                           const stream_index_pair& stream_index);

  bool getGainCallback(GetInt32Request& request, GetInt32Response& response,
                       const stream_index_pair& stream_index);

  bool setGainCallback(SetInt32Request& request, SetInt32Response& response,
                       const stream_index_pair& stream_index);

  bool getAutoWhiteBalanceCallback(GetInt32Request& request, GetInt32Response& response);

  bool setAutoWhiteBalanceCallback(SetInt32Request& request, SetInt32Response& response);

  bool getWhiteBalanceCallback(GetInt32Request& request, GetInt32Response& response);

  bool setWhiteBalanceCallback(SetInt32Request& request, SetInt32Response& response);

  bool setAutoExposureCallback(std_srvs::SetBoolRequest& request,
                               std_srvs::SetBoolResponse& response,
                               const stream_index_pair& stream_index);

  bool getAutoExposureCallback(GetBoolRequest& request, GetBoolResponse& response,
                               const stream_index_pair& stream_index);

  bool setLaserCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool setLdpEnableCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool setFanCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool setFloorCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool getDeviceInfoCallback(GetDeviceInfoRequest& request, GetDeviceInfoResponse& response);

  bool getSDKVersionCallback(GetStringRequest& request, GetStringResponse& response);

  bool toggleSensorCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response,
                            const stream_index_pair& stream_index);
  bool saveImagesCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  bool savePointCloudCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  bool toggleSensor(const stream_index_pair& stream_index, bool enabled, std::string& msg);

  bool getCameraParamsCallback(orbbec_camera::GetCameraParamsRequest& request,
                               orbbec_camera::GetCameraParamsResponse& response);

  bool getSerialNumberCallback(GetStringRequest& request, GetStringResponse& response);

  bool getDeviceTypeCallback(GetStringRequest& request, GetStringResponse& response);

  bool getCameraInfoCallback(GetCameraInfoRequest& request, GetCameraInfoResponse& response,
                             const stream_index_pair& stream_index);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::atomic_bool is_running_{false};
  std::map<stream_index_pair, std::shared_ptr<ROSOBSensor>> sensors_;
  std::map<stream_index_pair, int> width_;
  std::map<stream_index_pair, int> height_;
  std::map<stream_index_pair, int> fps_;
  std::map<stream_index_pair, ob_format> format_;  // for open stream
  std::map<stream_index_pair, std::string> encoding_;
  std::map<stream_index_pair, cv::Mat> images_;
  std::map<stream_index_pair, int> image_format_;  // for cv_bridge
  std::map<stream_index_pair, int> unit_step_size_;
  std::map<stream_index_pair, bool> enable_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfileList>> supported_profiles_;
  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, std::atomic_bool> save_images_;
  std::map<stream_index_pair, ros::Publisher> image_publishers_;
  std::map<stream_index_pair, ros::Publisher> camera_info_publishers_;
  std::map<stream_index_pair, ob::FrameCallback> frame_callback_;
  std::map<stream_index_pair, sensor_msgs::CameraInfo> camera_infos_;
  std::map<stream_index_pair, bool> stream_started_;
  std::vector<int> compression_params_;
  ob::FormatConvertFilter format_convert_filter_;

  std::map<stream_index_pair, std::string> format_str_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::string camera_link_frame_id_ = "camera_link";
  std::string camera_name_ = "camera";
  std::string base_frame_id_ = "camera_link";
  std::map<stream_index_pair, ros::ServiceServer> get_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> get_gain_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_gain_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_mirror_srv_;
  std::map<stream_index_pair, ros::ServiceServer> toggle_sensor_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_auto_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> get_auto_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> get_camera_info_srv_;
  ros::ServiceServer get_sdk_version_srv_;
  ros::ServiceServer get_device_info_srv_;
  ros::ServiceServer set_laser_srv_;
  ros::ServiceServer set_floor_srv_;
  ros::ServiceServer set_ldp_srv_;
  ros::ServiceServer set_fan_srv_;
  ros::ServiceServer get_auto_white_balance_srv_;
  ros::ServiceServer set_auto_white_balance_srv_;
  ros::ServiceServer get_white_balance_srv_;
  ros::ServiceServer set_white_balance_srv_;
  ros::ServiceServer get_serial_number_srv_;
  ros::ServiceServer reset_ir_exposure_srv_;
  ros::ServiceServer get_camera_params_srv_;
  ros::ServiceServer get_device_type_srv_;
  ros::ServiceServer save_point_cloud_srv_;
  ros::ServiceServer save_images_srv_;

  bool publish_tf_ = true;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::TransformStamped> static_tf_msgs_;
  ros::Publisher extrinsics_publisher_;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
  bool depth_align_ = false;
  std::recursive_mutex device_lock_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> color_camera_info_ = nullptr;
  std::unique_ptr<camera_info_manager::CameraInfoManager> ir_camera_info_ = nullptr;
  std::string ir_info_uri_;
  std::string color_info_uri_;
  bool enable_d2c_viewer_ = false;
  std::unique_ptr<D2CViewer> d2c_viewer_ = nullptr;
  bool enable_pipeline_ = false;
  std::unique_ptr<ob::Pipeline> pipeline_ = nullptr;
  std::shared_ptr<ob::Config> pipeline_config_ = nullptr;
  ros::Publisher depth_cloud_pub_;
  ros::Publisher depth_registered_cloud_pub_;
  sensor_msgs::PointCloud2 cloud_msg_;
  ob::PointCloudFilter cloud_filter_;
  std::atomic_bool pipeline_started_{false};
  bool enable_point_cloud_ = true;
  bool enable_point_cloud_xyzrgb_ = true;
  std::atomic_bool save_point_cloud_xyz_{false};
  std::atomic_bool save_point_cloud_xyzrgb_{false};
};

}  // namespace orbbec_camera
