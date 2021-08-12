#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"
#include "orbbec_camera/GetCameraInfo.h"

class ColorSensor
{
private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;
    image_transport::Publisher mColorPub;
    ros::ServiceServer mColorInfoService;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mColorSensor;
    std::shared_ptr<ob::StreamProfile> mColorProfile;

    size_t mArgbBufferSize;
    void* mArgbBuffer;
    size_t mRgbBufferSize;
    void* mRgbBuffer;

    void* getArgbBuffer(size_t bufferSize);
    void* getRgbBuffer(size_t bufferSize);

    bool getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res);

public:
    ColorSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor);
    ~ColorSensor();

    void startColorStream();
    void stopColorStream();
    void reconfigColorStream(int width, int height, int fps);
};
