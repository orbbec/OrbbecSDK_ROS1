#include "orbbec_device_manager.h"
#include "version.h"
#include "libobsensor/ObSensor.h"
#include <chrono>
#include <thread>

OrbbecDeviceManager::OrbbecDeviceManager(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDeviceName(""), mSerialNumber(""), mPid(0), mVid(0)
{
    mPrivateNodeHandle.getParam("device_name", mDeviceName);
    mPrivateNodeHandle.getParam("sn", mSerialNumber);
    mPrivateNodeHandle.getParam("vid", mVid);
    mPrivateNodeHandle.getParam("pid", mPid);

    std::this_thread::sleep_for(std::chrono::milliseconds(3000));

    // std::string ns = ros::this_node::getNamespace();
    // ros::ServiceClient client = nh.serviceClient<orbbec_camera::GetDeviceList>(ns + "get_device_list");
    ros::ServiceClient client = nh.serviceClient<orbbec_camera::GetDeviceList>("/get_device_list");
    orbbec_camera::GetDeviceList srv;
    if(client.call(srv))
    {
        ROS_INFO("Get device list");
        mDevInfos = srv.response.dev_infos;
    }
    else
    {
        ROS_INFO("Get device list failed");
    }
    // mCtx.setDeviceChangedCallback(
    //     [this](std::shared_ptr<ob::DeviceList> removedList, std::shared_ptr<ob::DeviceList> addedList) {
    //         DeviceConnectCallback(addedList);
    //         DeviceDisconnectCallback(removedList);
    //     });

    // mDeviceList = mCtx.queryDeviceList();
    // ROS_INFO("dev count: %d", mDeviceList->deviceCount());
    // for (int i = 0; i < mDeviceList->deviceCount(); i++)
    // {
    //     auto dev = mDeviceList->createDevice(i);
    //     auto devInfo = dev->getDeviceInfo();
    //     orbbec_camera::DeviceInfo info;
    //     info.name = devInfo->name();
    //     info.vid = devInfo->vid();
    //     info.pid = devInfo->pid();
    //     info.sn = devInfo->serialNumber();
    //     mDevInfos.push_back(info);
    //     ROS_INFO("Device found: %s %x:%x %s", info.name.c_str(), info.vid, info.pid, info.sn.c_str());
    // }

    // mGetVersionService = mNodeHandle.advertiseService("get_version", &OrbbecDeviceManager::getVersionCallback, this);
    // mDeviceListService =
    //     mNodeHandle.advertiseService("get_device_list", &OrbbecDeviceManager::getDeviceListCallback, this);

    findDevice();

    if (mDevice)
    {
        openDevice();
    }
}

OrbbecDeviceManager::~OrbbecDeviceManager()
{
}

void OrbbecDeviceManager::findDevice()
{
    mDeviceList = mCtx.queryDeviceList();

    if (mDevInfos.size() == 0)
    {
        ROS_WARN("No device connect");
        return;
    }
    if (mSerialNumber == "" && mDeviceName == "" && mPid == 0 && mVid == 0)
    {
        mDevice = mDeviceList->createDevice(0);
    }
    else
    {
        for (int i = 0; i < mDevInfos.size(); i++)
        {
            auto devInfo = mDevInfos[i];
            if (mSerialNumber != "" && devInfo.sn == mSerialNumber)
            {
                mDevice = mDeviceList->createDevice(i);
                ROS_INFO("Find device with sn: %s", mSerialNumber.c_str());
                break;
            }
            else if (mPid != 0 && mVid != 0 && devInfo.pid == mPid && devInfo.vid == mVid)
            {
                mDevice = mDeviceList->createDevice(i);
                ROS_INFO("Find device with vid pid: %d, %d", mVid, mPid);
                break;
            }
            if (mDeviceName != "" && devInfo.name == mDeviceName)
            {
                mDevice = mDeviceList->createDevice(i);
                ROS_INFO("Find device with name: %s", mDeviceName.c_str());
                break;
            }
        }
    }
    if (!mDevice)
    {
        ROS_WARN("No device found");
        return;
    }
}

void OrbbecDeviceManager::openDevice()
{
    mDeviceNode = std::make_shared<OrbbecDevice>(mNodeHandle, mPrivateNodeHandle, mDevice);
}

void OrbbecDeviceManager::DeviceConnectCallback(std::shared_ptr<ob::DeviceList> connectList)
{
    ROS_INFO("Device connect: %d", connectList->deviceCount());
}

void OrbbecDeviceManager::DeviceDisconnectCallback(std::shared_ptr<ob::DeviceList> disconnectList)
{
    ROS_INFO("Device disconnect: %d", disconnectList->deviceCount());
}

bool OrbbecDeviceManager::getVersionCallback(orbbec_camera::GetVersion::Request& request,
                                             orbbec_camera::GetVersion::Response& response)
{
    response.version = OB_ROS_VERSION_STR;
    response.core_version = OB_API_VERSION_STR;
    return true;
}

bool OrbbecDeviceManager::getDeviceListCallback(orbbec_camera::GetDeviceList::Request& request,
                                                orbbec_camera::GetDeviceList::Response& response)
{
    response.dev_infos = mDevInfos;
    return true;
}
