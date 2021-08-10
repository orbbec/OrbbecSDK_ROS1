
#pragma once

#include <libobsensor/hpp/Context.hpp>
#include <libobsensor/hpp/Device.hpp>
#include <ros/ros.h>
#include "orbbec_camera/GetDeviceList.h"
#include "orbbec_camera/DeviceInfo.h"
#include "orbbec_device.h"
#include <string>
#include <vector>

class OrbbecDeviceManager
{
private:
    ros::NodeHandle& nodeHandle;
    ros::NodeHandle& privateNodeHandle;
    ros::ServiceServer deviceListService;

    ob::Context ctx;

    std::shared_ptr<ob::DeviceList> deviceList;

    std::vector<std::shared_ptr<ob::Device>> devices;

    std::shared_ptr<ob::Device> device;

    std::shared_ptr<OrbbecDevice> orbbecDevice;

    std::string deviceName;
    std::string serialNumber;
    int pid;
    int vid;

    void findDevice();
    void openDevice();
    
    void DeviceConnectCallback(std::shared_ptr< ob::DeviceList > connectList);
    void DeviceDisconnectCallback(std::shared_ptr< ob::DeviceList > disconnectList);

    bool getDeviceList(orbbec_camera::GetDeviceList::Request& request, orbbec_camera::GetDeviceList::Response& response);

public:
    OrbbecDeviceManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~OrbbecDeviceManager();
};
