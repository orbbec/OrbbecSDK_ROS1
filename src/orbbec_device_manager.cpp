#include "orbbec_device_manager.h"

OrbbecDeviceManager::OrbbecDeviceManager(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nodeHandle(nh), privateNodeHandle(pnh), deviceName(""), serialNumber(""), pid(0), vid(0)
{
    ctx.setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList)
                                 {
                                     DeviceConnectCallback(addedList);
                                     DeviceDisconnectCallback(removedList);
                                 });

    deviceList = ctx.queryDeviceList();
    for (int i = 0; i < deviceList->deviceCount(); i++)
    {
        ROS_INFO("Device found: %s, %x:%x", deviceList->name(i), deviceList->vid(i), deviceList->pid(i));
        auto dev = deviceList->createDevice(i);
        devices.push_back(dev);
    }

    deviceListService = nodeHandle.advertiseService("get_device_list", &OrbbecDeviceManager::getDeviceList, this);

    findDevice();

    if(device)
    {
        openDevice();
    }
}

OrbbecDeviceManager::~OrbbecDeviceManager()
{
}

void OrbbecDeviceManager::findDevice()
{
    if (devices.size() == 0)
    {
        ROS_WARN("No device found");
        return;
    }
    if (serialNumber == "" && deviceName == "" && pid == 0 && vid == 0)
    {
        device = devices[0];
    }
    else
    {
        for (int i = 0; i < devices.size(); i++)
        {
            auto devInfo = devices[i]->getDeviceInfo();
            if (serialNumber != "" && devInfo->serialNumber() == serialNumber)
            {
                device = devices[i];
            }
            else if (pid != 0 && vid != 0 && devInfo->pid() == pid && devInfo->vid() == vid)
            {
                device = devices[i];
            }
            if (deviceName != "" && devInfo->name() == deviceName)
            {
                device = devices[i];
            }
        }
    }
    if(!device)
    {
        ROS_WARN("No device found");
        return;
    }
}

void OrbbecDeviceManager::openDevice()
{
    orbbecDevice = std::make_shared<OrbbecDevice>(nodeHandle, privateNodeHandle, device);
}

void OrbbecDeviceManager::DeviceConnectCallback(std::shared_ptr<ob::DeviceList> connectList)
{
    ROS_INFO("Device connect: %d", connectList->deviceCount());
}

void OrbbecDeviceManager::DeviceDisconnectCallback(std::shared_ptr<ob::DeviceList> disconnectList)
{
    ROS_INFO("Device disconnect: %d", disconnectList->deviceCount());
}

bool OrbbecDeviceManager::getDeviceList(orbbec_camera::GetDeviceList::Request &request, orbbec_camera::GetDeviceList::Response &response)
{
    std::vector<orbbec_camera::DeviceInfo> devInfos;
    for (int i = 0; i < devices.size(); i++)
    {
        auto devInfo = devices[i]->getDeviceInfo();
        orbbec_camera::DeviceInfo info;
        info.name = devInfo->name();
        info.vid = devInfo->vid();
        info.pid = devInfo->pid();
        info.sn = devInfo->serialNumber();
        devInfos.push_back(info);
    }
    response.dev_infos = devInfos;
    return true;
}
