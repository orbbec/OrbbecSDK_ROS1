#include "orbbec_device_manager.h"

OrbbecDeviceManager::OrbbecDeviceManager(ros::NodeHandle& nh, ros::NodeHandle& pnh) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDeviceName(""), mSerialNumber(""), mPid(0), mVid(0)
{
    mCtx.setDeviceChangedCallback([this](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList)
                                 {
                                     DeviceConnectCallback(addedList);
                                     DeviceDisconnectCallback(removedList);
                                 });

    mDeviceList = mCtx.queryDeviceList();
    for (int i = 0; i < mDeviceList->deviceCount(); i++)
    {
        ROS_INFO("Device found: %s, %x:%x", mDeviceList->name(i), mDeviceList->vid(i), mDeviceList->pid(i));
        auto dev = mDeviceList->createDevice(i);
        mDevices.push_back(dev);
    }

    mDeviceListService = mNodeHandle.advertiseService("get_device_list", &OrbbecDeviceManager::getDeviceList, this);

    findDevice();

    if(mDevice)
    {
        openDevice();
    }
}

OrbbecDeviceManager::~OrbbecDeviceManager()
{
}

void OrbbecDeviceManager::findDevice()
{
    if (mDevices.size() == 0)
    {
        ROS_WARN("No device found");
        return;
    }
    if (mSerialNumber == "" && mDeviceName == "" && mPid == 0 && mVid == 0)
    {
        mDevice = mDevices[0];
    }
    else
    {
        for (int i = 0; i < mDevices.size(); i++)
        {
            auto devInfo = mDevices[i]->getDeviceInfo();
            if (mSerialNumber != "" && devInfo->serialNumber() == mSerialNumber)
            {
                mDevice = mDevices[i];
            }
            else if (mPid != 0 && mVid != 0 && devInfo->pid() == mPid && devInfo->vid() == mVid)
            {
                mDevice = mDevices[i];
            }
            if (mDeviceName != "" && devInfo->name() == mDeviceName)
            {
                mDevice = mDevices[i];
            }
        }
    }
    if(!mDevice)
    {
        ROS_WARN("No device found");
        return;
    }
}

void OrbbecDeviceManager::openDevice()
{
    mOrbbecDevice = std::make_shared<OrbbecDevice>(mNodeHandle, mPrivateNodeHandle, mDevice);
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
    for (int i = 0; i < mDevices.size(); i++)
    {
        auto devInfo = mDevices[i]->getDeviceInfo();
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
