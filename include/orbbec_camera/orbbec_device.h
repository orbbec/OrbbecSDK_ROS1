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
    ros::NodeHandle& nodeHandle;
    ros::NodeHandle& privateNodeHandle;

    std::shared_ptr<ob::Device> device;
    std::shared_ptr<ob::Sensor> colorSensor;
    std::shared_ptr<ob::Sensor> depthSensor;
    std::shared_ptr<ob::Sensor> irSensor;

    // image_transport::ImageTransport colorIt;
    // image_transport::ImageTransport depthIt;
    // image_transport::ImageTransport irIt;

    // image_transport::CameraPublisher colorPub;
    // image_transport::CameraPublisher depthPub;
    // image_transport::CameraPublisher irPub;

    image_transport::Publisher colorPub;
    image_transport::Publisher depthPub;
    image_transport::Publisher irPub;

    sensor_msgs::CameraInfo info;

public:
    OrbbecDevice(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> dev);
    ~OrbbecDevice();

    void startColorStream();
    void startDepthStream();
    void startIRStream();
};
