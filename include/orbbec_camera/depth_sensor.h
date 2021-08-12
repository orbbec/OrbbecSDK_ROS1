#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"
#include "orbbec_camera/GetCameraInfo.h"

class DepthSensor
{
private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;
    image_transport::Publisher mDepthPub;
    ros::ServiceServer mDepthInfoService;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mDepthSensor;
    std::shared_ptr<ob::StreamProfile> mDepthProfile;

    bool getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res);

public:
    DepthSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor);
    ~DepthSensor();

    void startDepthStream();
    void stopDepthStream();
    void reconfigDepthStream(int width, int height, int fps);
};
