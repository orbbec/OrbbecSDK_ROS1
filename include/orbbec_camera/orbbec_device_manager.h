
#pragma once

#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "ros/ros.h"
#include "orbbec_camera/GetDeviceList.h"
#include "orbbec_camera/DeviceInfo.h"
#include "orbbec_camera/GetVersion.h"
#include "orbbec_device.h"
#include <string>
#include <vector>

class OrbbecDeviceManager
{
private:
    ros::NodeHandle& mNodeHandle;
    ros::NodeHandle& mPrivateNodeHandle;
    ros::ServiceServer mDeviceListService;
    ros::ServiceServer mGetVersionService;

    ob::Context mCtx;

    std::shared_ptr<ob::DeviceList> mDeviceList;
    // std::vector<std::shared_ptr<ob::Device>> mDevices;
    std::shared_ptr<ob::Device> mDevice;

    std::shared_ptr<OrbbecDevice> mDeviceNode;

    std::vector<orbbec_camera::DeviceInfo> mDevInfos;

    std::string version;
    std::string coreVersion;

    std::string mDeviceName;
    std::string mSerialNumber;
    int mPid;
    int mVid;

    void findDevice();
    void openDevice();
    
    void DeviceConnectCallback(std::shared_ptr< ob::DeviceList > connectList);
    void DeviceDisconnectCallback(std::shared_ptr< ob::DeviceList > disconnectList);

    bool getVersionCallback(orbbec_camera::GetVersion::Request& request, orbbec_camera::GetVersion::Response& response);
    bool getDeviceListCallback(orbbec_camera::GetDeviceList::Request& request, orbbec_camera::GetDeviceList::Response& response);

public:
    OrbbecDeviceManager(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~OrbbecDeviceManager();
};
