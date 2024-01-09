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
#include "types.h"
#include "utils.h"
#include "ros_sensor.h"
#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <condition_variable>
#include <thread>
#include <camera_info_manager/camera_info_manager.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include "orbbec_camera/d2c_viewer.h"
#include "orbbec_camera/GetCameraParams.h"
#include <boost/optional.hpp>

#include "jpeg_decoder.h"

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
  bool isInitialized() const;

 private:
  struct IMUData {
    IMUData() = default;
    IMUData(stream_index_pair stream, Eigen::Vector3d data, double timestamp)
        : stream_(std::move(stream)), data_(std::move(data)), timestamp_(timestamp) {}
    bool isSet() const { return timestamp_ >= 0; }
    stream_index_pair stream_{};
    Eigen::Vector3d data_{};
    double timestamp_ = -1;  // in nanoseconds
  };

  void init();

  void setupCameraCtrlServices();

  void setupConfig();

  void getParameters();

  void setupDevices();

  void setupFrameCallback();

  void readDefaultGain();

  void readDefaultExposure();

  void readDefaultWhiteBalance();

  std::shared_ptr<ob::Frame> softwareDecodeColorFrame(const std::shared_ptr<ob::Frame>& frame);

  void onNewFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                          const stream_index_pair& stream_index);

  void onNewIMUFrameSyncOutputCallback(const std::shared_ptr<ob::Frame>& aframe,
                             const std::shared_ptr<ob::Frame>& gframe);

  void onNewIMUFrameCallback(const std::shared_ptr<ob::Frame>& frame,
                             const stream_index_pair& stream_index);

  bool decodeColorFrameToBuffer(const std::shared_ptr<ob::Frame>& frame, uint8_t* dest);

  std::shared_ptr<ob::Frame> decodeIRMJPGFrame(const std::shared_ptr<ob::Frame> &frame);

  void onNewFrameSetCallback(const std::shared_ptr<ob::FrameSet>& frame_set);

  void onNewColorFrameCallback();

  void publishPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  void publishDepthPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  void publishColoredPointCloud(const std::shared_ptr<ob::FrameSet>& frame_set);

  bool setupFormatConvertType(OBFormat type);

  void setupProfiles();

  void setupTopics();

  void setupPipelineConfig();

  void setupPublishers();

  void publishStaticTF(const ros::Time& t, const tf2::Vector3& trans, const tf2::Quaternion& q,
                       const std::string& from, const std::string& to);

  void startStreams();

  void startIMUSyncStream();

  void startAccel();

  void startGyro();

  void startIMU(const stream_index_pair& stream_index);

  void startIMU();

  void stopStreams();

  void stopIMU(const stream_index_pair& stream_index);

  void stopIMU();

  void setDefaultIMUMessage(sensor_msgs::Imu& imu_msg);

  sensor_msgs::Imu createUnitIMUMessage(const IMUData& accel_data, const IMUData& gyro_data);

  void startStream(const stream_index_pair& stream_index);

  void stopStream(const stream_index_pair& stream_index);

  void imageSubscribedCallback(const stream_index_pair& stream_index);

  void imuSubscribedCallback(const stream_index_pair& stream_index);

  void imageUnsubscribedCallback(const stream_index_pair& stream_index);

  void imuUnsubscribedCallback(const stream_index_pair& stream_index);

  void pointCloudSubscribedCallback();

  void pointCloudUnsubscribedCallback();

  void coloredPointCloudSubscribedCallback();

  void coloredPointCloudUnsubscribedCallback();

  void calcAndPublishStaticTransform();

  void publishDynamicTransforms();

  void publishStaticTransforms();

  boost::optional<OBCameraParam> getCameraParam();

  boost::optional<OBCameraParam> getCameraDepthParam();

  boost::optional<OBCameraParam> getCameraColorParam();

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

  bool getLdpStatusCallback(GetBoolRequest& request, GetBoolResponse& response);

  bool setFanWorkModeCallback(std_srvs::SetBoolRequest& request,
                              std_srvs::SetBoolResponse& response);

  bool setFloorCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response);

  bool getDeviceInfoCallback(GetDeviceInfoRequest& request, GetDeviceInfoResponse& response);

  bool getSDKVersionCallback(GetStringRequest& request, GetStringResponse& response);

  bool toggleSensorCallback(std_srvs::SetBoolRequest& request, std_srvs::SetBoolResponse& response,
                            const stream_index_pair& stream_index);
  bool saveImagesCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  void saveImageToFile(const stream_index_pair& stream_index, const cv::Mat& image,
                       const sensor_msgs::ImagePtr& image_msg);

  bool savePointCloudCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response);

  bool toggleSensor(const stream_index_pair& stream_index, bool enabled, std::string& msg);

  bool getCameraParamsCallback(orbbec_camera::GetCameraParamsRequest& request,
                               orbbec_camera::GetCameraParamsResponse& response);

  bool getSerialNumberCallback(GetStringRequest& request, GetStringResponse& response);

  bool getDeviceTypeCallback(GetStringRequest& request, GetStringResponse& response);

  bool getCameraInfoCallback(GetCameraInfoRequest& request, GetCameraInfoResponse& response,
                             const stream_index_pair& stream_index);

  bool resetCameraGainCallback(std_srvs::EmptyRequest& request, std_srvs::EmptyResponse& response,
                               const stream_index_pair& stream_index);

  bool resetCameraExposureCallback(std_srvs::EmptyRequest& request,
                                   std_srvs::EmptyResponse& response,
                                   const stream_index_pair& stream_index);

  bool resetCameraWhiteBalanceCallback(std_srvs::EmptyRequest& request,
                                       std_srvs::EmptyResponse& response);

  bool switchIRModeCallback(SetInt32Request& request, SetInt32Response& response);

  bool switchIRDataSourceChannelCallback(SetStringRequest& request, SetStringResponse& response);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_rgb_;
  ros::NodeHandle nh_ir_;
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::DeviceInfo> device_info_ = nullptr;
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
  std::map<stream_index_pair, bool> enable_stream_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfile>> stream_profile_;
  std::map<stream_index_pair, std::shared_ptr<ob::StreamProfileList>> supported_profiles_;
  std::map<stream_index_pair, std::string> stream_name_;
  std::map<stream_index_pair, std::atomic_bool> save_images_;
  std::map<stream_index_pair, ros::Publisher> image_publishers_;
  std::map<stream_index_pair, ros::Publisher> camera_info_publishers_;
  std::map<stream_index_pair, ob::FrameCallback> frame_callback_;
  std::map<stream_index_pair, sensor_msgs::CameraInfo> camera_infos_;
  std::map<stream_index_pair, bool> flip_images_;
  std::map<stream_index_pair, bool> stream_started_;
  std::vector<int> compression_params_;
  ob::FormatConvertFilter format_convert_filter_;

  std::map<stream_index_pair, std::string> format_str_;
  std::map<stream_index_pair, std::string> frame_id_;
  std::map<stream_index_pair, std::string> optical_frame_id_;
  std::map<stream_index_pair, std::string> depth_aligned_frame_id_;
  std::map<stream_index_pair, int> default_gain_;
  std::map<stream_index_pair, int> default_exposure_;
  int default_white_balance_ = 0;
  std::string camera_link_frame_id_ = "camera_link";
  std::string camera_name_ = "camera";
  const std::string imu_optical_frame_id_ = "camera_gyro_accel_optical_frame";
  const std::string imu_frame_id_ = "camera_gyro_accel_frame";
  std::map<stream_index_pair, ros::ServiceServer> get_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> reset_exposure_srv_;
  std::map<stream_index_pair, ros::ServiceServer> get_gain_srv_;
  std::map<stream_index_pair, ros::ServiceServer> set_gain_srv_;
  std::map<stream_index_pair, ros::ServiceServer> reset_gain_srv_;
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
  ros::ServiceServer get_ldp_status_srv_;
  ros::ServiceServer set_fan_work_mode_srv_;
  ros::ServiceServer get_auto_white_balance_srv_;
  ros::ServiceServer set_auto_white_balance_srv_;
  ros::ServiceServer get_white_balance_srv_;
  ros::ServiceServer set_white_balance_srv_;
  ros::ServiceServer reset_white_balance_srv_;
  ros::ServiceServer get_serial_number_srv_;
  ros::ServiceServer get_camera_params_srv_;
  ros::ServiceServer get_device_type_srv_;
  ros::ServiceServer save_point_cloud_srv_;
  ros::ServiceServer save_images_srv_;
  ros::ServiceServer switch_ir_mode_srv_;
  ros::ServiceServer switch_ir_data_source_channel_srv_;

  bool publish_tf_ = true;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_ = nullptr;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_ = nullptr;
  std::vector<geometry_msgs::TransformStamped> static_tf_msgs_;
  std::shared_ptr<std::thread> tf_thread_ = nullptr;
  std::condition_variable tf_cv_;
  double tf_publish_rate_ = 10.0;
  bool depth_registration_ = false;
  bool enable_frame_sync_ = false;
  std::recursive_mutex device_lock_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> color_camera_info_manager_ = nullptr;
  std::shared_ptr<camera_info_manager::CameraInfoManager> ir_camera_info_manager_ = nullptr;
  std::string ir_info_uri_;
  std::string color_info_uri_;
  bool enable_d2c_viewer_ = false;
  std::shared_ptr<D2CViewer> d2c_viewer_ = nullptr;
  bool enable_pipeline_ = false;
  std::shared_ptr<ob::Pipeline> pipeline_ = nullptr;
  std::shared_ptr<ob::Config> pipeline_config_ = nullptr;
  ros::Publisher depth_cloud_pub_;
  ros::Publisher depth_registered_cloud_pub_;
  sensor_msgs::PointCloud2 cloud_msg_;
  std::atomic_bool pipeline_started_{false};
  bool enable_point_cloud_ = false;
  bool enable_colored_point_cloud_ = false;
  std::atomic_bool save_point_cloud_{false};
  std::atomic_bool save_colored_point_cloud_{false};
  boost::optional<OBCameraParam> camera_params_;
  bool is_initialized_ = false;
  bool enable_soft_filter_ = true;
  bool enable_color_auto_exposure_ = true;
  bool enable_ir_auto_exposure_ = true;
  bool enable_ldp_ = true;
  int soft_filter_max_diff_ = -1;
  int soft_filter_speckle_size_ = -1;
  std::string depth_filter_config_;
  bool enable_depth_filter_ = false;

  // Only for Gemini2 device
  bool enable_hardware_d2d_ = true;
  std::string depth_work_mode_;
  OBMultiDeviceSyncMode sync_mode_ = OB_MULTI_DEVICE_SYNC_MODE_FREE_RUN;
  std::string sync_mode_str_;
  int depth_delay_us_ = 0;
  int color_delay_us_ = 0;
  int trigger2image_delay_us_ = 0;
  int trigger_out_delay_us_ = 0;
  bool trigger_out_enabled_ = false;
  std::string depth_precision_str_;
  OB_DEPTH_PRECISION_LEVEL depth_precision_ = OB_PRECISION_1MM;
  // IMU
  std::map<stream_index_pair, ros::Publisher> imu_publishers_;
  std::map<stream_index_pair, std::string> imu_rate_;
  std::map<stream_index_pair, std::string> imu_range_;
  std::map<stream_index_pair, std::string> imu_qos_;
  std::map<stream_index_pair, bool> imu_started_;
  std::map<stream_index_pair, std::shared_ptr<ob::Sensor>> imu_sensor_;
  double liner_accel_cov_ = 0.0001;
  double angular_vel_cov_ = 0.0001;
  std::deque<IMUData> imu_history_;
  IMUData accel_data_{ACCEL, {0, 0, 0}, -1.0};

  bool enable_sync_output_accel_gyro_ = false;
  std::shared_ptr<ob::Pipeline> imuPipeline_ = nullptr;
  ros::Publisher imu_gyro_accel_publisher_;
  bool imu_sync_output_start_ = false;

  // mjpeg decoder
  std::shared_ptr<JPEGDecoder> mjpeg_decoder_ = nullptr;
  uint8_t* rgb_buffer_ = nullptr;
  bool rgb_is_decoded_ = false;

  // double infrared
  bool enable_left_ir_ = false;
  bool enable_right_ir_ = false;

  //For color
  std::queue<std::shared_ptr<ob::FrameSet>> colorFrameQueue_;
  std::shared_ptr<std::thread> colorFrameThread_ = nullptr;
  std::mutex colorFrameMtx_;
  std::condition_variable colorFrameCV_;
};

}  // namespace orbbec_camera
