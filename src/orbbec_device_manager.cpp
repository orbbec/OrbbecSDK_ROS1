#include <orbbec_device_manager.h>

void OrbbecDeviceManager::DeviceConnectCallback(std::shared_ptr< ob::DeviceList > connectList)
{
    ROS_INFO("Device connect: %d", connectList->deviceCount());
}

void OrbbecDeviceManager::DeviceDisconnectCallback(std::shared_ptr< ob::DeviceList > disconnectList)
{
    ROS_INFO("Device disconnect: %d", disconnectList->deviceCount());
}

bool OrbbecDeviceManager::getDeviceList(orbbec_camera::GetDeviceList::Request& request, orbbec_camera::GetDeviceList::Response& response)
{
    std::vector<orbbec_camera::DeviceInfo> devInfos;
    for(int i = 0; i < devices.size(); i++)
    {
        auto devInfo = devices[i]->getDeviceInfo();
        orbbec_camera::DeviceInfo info;
        info.name = devInfo->name();
        info.vid = devInfo->vid();
        info.pid = devInfo->pid();
        devInfos.push_back(info);
    }
    response.dev_infos = devInfos;
    return true;
}

OrbbecDeviceManager::OrbbecDeviceManager(ros::NodeHandle nh) : nodeHandle(nh)
{
    ob::Context ctx;

    ctx.setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
        DeviceConnectCallback(addedList);
        DeviceDisconnectCallback(removedList);
    });

    deviceList = ctx.queryDeviceList();
    for(int i = 0; i < deviceList->deviceCount(); i++)
    {
        ROS_INFO("Device found: %s, %x:%x", deviceList->name(i), deviceList->vid(i), deviceList->pid(i));
        auto dev = deviceList->createDevice(i);
        devices.push_back(dev);
    }

    nodeHandle.advertiseService("get_device_list", &OrbbecDeviceManager::getDeviceList, this);
}

OrbbecDeviceManager::~OrbbecDeviceManager()
{
}
