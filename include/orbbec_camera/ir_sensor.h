#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"
#include "orbbec_camera/GetCameraInfo.h"
#include "orbbec_camera/GetExposure.h"
#include "orbbec_camera/SetExposure.h"
#include "orbbec_camera/GetGain.h"
#include "orbbec_camera/SetGain.h"
#include "orbbec_camera/GetWhiteBalance.h"
#include "orbbec_camera/SetWhiteBalance.h"
#include "orbbec_camera/SetAutoExposure.h"
#include "orbbec_camera/SetAutoWhiteBalance.h"
#include "orbbec_camera/EnableStream.h"

class IrSensor
{
  private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;
    image_transport::Publisher mIrPub;
    ros::ServiceServer mGetCameraInfoService;
    ros::ServiceServer mGetExposureService;
    ros::ServiceServer mSetExposureService;
    ros::ServiceServer mGetGainService;
    ros::ServiceServer mSetGainService;
    ros::ServiceServer mGetWhiteBalanceService;
    ros::ServiceServer mSetWhiteBalanceService;
    ros::ServiceServer mSetAutoExposureService;
    ros::ServiceServer mSetAutoWhiteBalanceService;
    ros::ServiceServer mEnableStreamService;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mIrSensor;
    std::shared_ptr<ob::StreamProfile> mIrProfile;

    bool mIsStreaming;

    std::shared_ptr<ob::StreamProfile> findProfile(int width = 0, int height = 0, int fps = 0);

    bool getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res);
    bool getExposureCallback(orbbec_camera::GetExposureRequest& req, orbbec_camera::GetExposureResponse& res);
    bool setExposureCallback(orbbec_camera::SetExposureRequest& req, orbbec_camera::SetExposureResponse& res);
    bool getGainCallback(orbbec_camera::GetGainRequest& req, orbbec_camera::GetGainResponse& res);
    bool setGainCallback(orbbec_camera::SetGainRequest& req, orbbec_camera::SetGainResponse& res);
    bool getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req,
                                 orbbec_camera::GetWhiteBalanceResponse& res);
    bool setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req,
                                 orbbec_camera::SetWhiteBalanceResponse& res);
    bool setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req,
                                 orbbec_camera::SetAutoExposureResponse& res);
    bool setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req,
                                     orbbec_camera::SetAutoWhiteBalanceResponse& res);
    bool enableStreamCallback(orbbec_camera::EnableStreamRequest& req, orbbec_camera::EnableStreamResponse& res);

  public:
    IrSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> device,
             std::shared_ptr<ob::Sensor> sensor);
    ~IrSensor();

    void startIrStream();
    void stopIrStream();
    void reconfigIrStream(int width, int height, int fps);
};
