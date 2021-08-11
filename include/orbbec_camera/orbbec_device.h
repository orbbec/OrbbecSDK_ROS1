#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/hpp/Sensor.hpp"
#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "image_transport/camera_publisher.h"
#include "color_sensor.h"
#include "depth_sensor.h"
#include "ir_sensor.h"

class OrbbecDevice
{
private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;

    std::shared_ptr<ob::Device> mDevice;
    std::shared_ptr<ob::Sensor> mColorSensor;
    std::shared_ptr<ob::Sensor> mDepthSensor;
    std::shared_ptr<ob::Sensor> mIrSensor;

    std::shared_ptr<ColorSensor> mColorSensorNode;
    std::shared_ptr<DepthSensor> mDepthSensorNode;
    std::shared_ptr<IrSensor> mIrSensorNode;

public:
    OrbbecDevice(ros::NodeHandle& nh, ros::NodeHandle& pnh, std::shared_ptr<ob::Device> dev);
    ~OrbbecDevice();
};
