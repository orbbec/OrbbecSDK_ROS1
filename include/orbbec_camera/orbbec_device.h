#pragma once

#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <libobsensor/hpp/Sensor.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>

class OrbbecDevice
{
private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mColorSensor;
    std::shared_ptr<ob::Sensor> mDepthSensor;
    std::shared_ptr<ob::Sensor> mIrSensor;

    // image_transport::ImageTransport mColorIt;
    // image_transport::ImageTransport mDepthIt;
    // image_transport::ImageTransport mIrIt;

    // image_transport::CameraPublisher mColorPub;
    // image_transport::CameraPublisher mDepthPub;
    // image_transport::CameraPublisher mIrPub;

    image_transport::Publisher mColorPub;
    image_transport::Publisher mDepthPub;
    image_transport::Publisher mIrPub;

    sensor_msgs::CameraInfo mInfo;

public:
    OrbbecDevice(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> dev);
    ~OrbbecDevice();

    void startColorStream();
    void startDepthStream();
    void startIRStream();
};
