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
    ros::ServiceClient client = nh.serviceClient<orbbec_camera::GetDeviceList>("/get_device_list");
    orbbec_camera::GetDeviceList srv;
    if (client.call(srv))
    {
        ROS_INFO("Get device list");
        mDevInfos = srv.response.dev_infos;
    }
    else
    {
        ROS_INFO("Get device list failed");
    }
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
        mDevice = mDeviceList->getDevice(0);
    }
    else
    {
        for (int i = 0; i < mDevInfos.size(); i++)
        {
            auto devInfo = mDevInfos[i];
            if (mSerialNumber != "" && devInfo.sn == mSerialNumber)
            {
                mDevice = mDeviceList->getDevice(i);
                ROS_INFO("Find device with sn: %s", mSerialNumber.c_str());
                break;
            }
            else if (mPid != 0 && mVid != 0 && devInfo.pid == mPid && devInfo.vid == mVid)
            {
                mDevice = mDeviceList->getDevice(i);
                ROS_INFO("Find device with vid pid: %d, %d", mVid, mPid);
                break;
            }
            if (mDeviceName != "" && devInfo.name == mDeviceName)
            {
                mDevice = mDeviceList->getDevice(i);
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
