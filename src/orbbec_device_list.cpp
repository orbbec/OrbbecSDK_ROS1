#include "libobsensor/hpp/Context.hpp"
#include "libobsensor/hpp/Device.hpp"
#include "libobsensor/h/Version.h"
#include "libobsensor/ObSensor.h"
#include "libobsensor/ObSensor.hpp"
#include "ros/ros.h"
#include "version.h"
#include "orbbec_camera/GetVersion.h"
#include "orbbec_camera/GetDeviceList.h"
#include <vector>

std::vector<orbbec_camera::DeviceInfo> devInfos;

void deviceConnectCallback(std::shared_ptr<ob::DeviceList> connectList)
{
    ROS_INFO("Device connect: %d", connectList->deviceCount());
}

void deviceDisconnectCallback(std::shared_ptr<ob::DeviceList> disconnectList)
{
    ROS_INFO("Device disconnect: %d", disconnectList->deviceCount());
}

bool getVersionCallback(orbbec_camera::GetVersion::Request& request,
                                             orbbec_camera::GetVersion::Response& response)
{
    response.version = OB_ROS_VERSION_STR;
    std::stringstream ss; 
    ss << ob_get_major_version() << "." << ob_get_minor_version() << "." << ob_get_patch_version(); 
    response.core_version = ss.str();
    return true;
}

bool getDeviceListCallback(orbbec_camera::GetDeviceList::Request& request,
                                                orbbec_camera::GetDeviceList::Response& response)
{
    response.dev_infos = devInfos;
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ob_device_list");

    ros::NodeHandle nh;

    ob::Context ctx;

    ctx.setDeviceChangedCallback(
        [](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
            deviceConnectCallback(addedList);
            deviceDisconnectCallback(removedList);
        });

    auto deviceList = ctx.queryDeviceList();
    for (int i = 0; i < deviceList->deviceCount(); i++)
    {
        auto dev = deviceList->getDevice(i);
        auto devInfo = dev->getDeviceInfo();
        orbbec_camera::DeviceInfo info;
        info.name = devInfo->name();
        info.vid = devInfo->vid();
        info.pid = devInfo->pid();
        info.sn = devInfo->serialNumber();
        devInfos.push_back(info);
        ROS_INFO("Device found: %s %x:%x %s", info.name.c_str(), info.vid, info.pid, info.sn.c_str());
    }

    ros::ServiceServer getVersionService = nh.advertiseService("get_version", getVersionCallback);
    ros::ServiceServer deviceListService = nh.advertiseService("get_device_list", getDeviceListCallback);

    ros::spin();

    return 0;
}
