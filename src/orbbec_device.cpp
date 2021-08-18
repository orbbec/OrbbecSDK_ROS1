#include "orbbec_device.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"

OrbbecDevice::OrbbecDevice(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> dev) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mReconfigServer(pnh), mDevice(dev)
{
    mSetLaserEnableService = mNodeHandle.advertiseService("device/set_laser_enable", &OrbbecDevice::setLaserEnableCallback, this);
    mSetLdpEnableService = mNodeHandle.advertiseService("device/set_ldp_enable", &OrbbecDevice::setLdpEnableCallback, this);
    mSetFloorEnableService = mNodeHandle.advertiseService("device/set_floor_enable", &OrbbecDevice::setFloorEnableCallback, this);
    mSetFanModeService = mNodeHandle.advertiseService("device/set_fan_mode", &OrbbecDevice::setFanModeCallback, this);

    mColorSensor = mDevice->getSensor(OB_SENSOR_COLOR);
    mDepthSensor = mDevice->getSensor(OB_SENSOR_DEPTH);
    mIrSensor = mDevice->getSensor(OB_SENSOR_IR);

    mColorSensorNode = std::make_shared<ColorSensor>(mNodeHandle, mPrivateNodeHandle, mDevice, mColorSensor);
    mDepthSensorNode = std::make_shared<DepthSensor>(mNodeHandle, mPrivateNodeHandle, mDevice, mDepthSensor);
    mIrSensorNode = std::make_shared<IrSensor>(mNodeHandle, mPrivateNodeHandle, mDevice, mIrSensor);
    
    mReconfigServer.setCallback(boost::bind(&OrbbecDevice::reconfigCallback, this, _1, _2));
}

OrbbecDevice::~OrbbecDevice()
{
}

bool OrbbecDevice::setLaserEnableCallback(orbbec_camera::SetLaserEnableRequest& req, orbbec_camera::SetLaserEnableResponse& res)
{
    bool laserEnable = req.enable;
    mDevice->setBoolProperty(OB_DEVICE_PROPERTY_EMITTER_BOOL, laserEnable);
    return true;
}

bool OrbbecDevice::setFloorEnableCallback(orbbec_camera::SetFloorEnableRequest& req, orbbec_camera::SetFloorEnableResponse& res)
{
    bool floorEnable = req.enable;
    mDevice->setBoolProperty(OB_DEVICE_PROPERTY_FLOOD_BOOL, floorEnable);
    return true;
}

bool OrbbecDevice::setLdpEnableCallback(orbbec_camera::SetLdpEnableRequest& req, orbbec_camera::SetLdpEnableResponse& res)
{
    bool ldpEnable = req.enable;
    mDevice->setBoolProperty(OB_DEVICE_PROPERTY_LDP_BOOL, ldpEnable);
    return true;
}

bool OrbbecDevice::setFanModeCallback(orbbec_camera::SetFanModeRequest& req, orbbec_camera::SetFanModeResponse& res)
{
    int32_t fanMode = req.mode;
    mDevice->setBoolProperty(OB_DEVICE_PROPERTY_FAN_WORK_MODE_INT, fanMode);
    return true;
}

void OrbbecDevice::reconfigCallback(orbbec_camera::OrbbecConfig& config, uint32_t level)
{
    int colorWidth = 0;
    int colorHeight = 0;
    int colorFps = 0;
    switch (config.color_mode)
    {
    case 1:
        colorWidth = 640;
        colorHeight = 480;
        colorFps = 30;
        break;
    case 2:
        colorWidth = 800;
        colorHeight = 600;
        colorFps = 30;
        break;
    case 3:
        colorWidth = 928;
        colorHeight = 696;
        colorFps = 30;
        break;
    case 4:
        colorWidth = 1920;
        colorHeight = 1080;
        colorFps = 30;
        break;
    case 5:
        colorWidth = 1024;
        colorHeight = 768;
        colorFps = 30;
        break;
    case 6:
        colorWidth = 640;
        colorHeight = 360;
        colorFps = 30;
        break;
    case 7:
        colorWidth = 1280;
        colorHeight = 720;
        colorFps = 30;
        break;
    case 8:
        colorWidth = 2048;
        colorHeight = 1536;
        colorFps = 15;
        break;
    default:
        break;
    }
    mColorSensorNode->reconfigColorStream(colorWidth, colorHeight, colorFps);

    int depthWidth = 0;
    int depthHeight = 0;
    int depthFps = 0;
    switch (config.depth_mode)
    {
    case 1:
        depthWidth = 160;
        depthHeight = 120;
        depthFps = 30;
        break;
    case 2:
        depthWidth = 160;
        depthHeight = 120;
        depthFps = 15;
        break;
    case 3:
        depthWidth = 160;
        depthHeight = 120;
        depthFps = 10;
        break;
    case 4:
        depthWidth = 160;
        depthHeight = 120;
        depthFps = 7;
        break;
    case 5:
        depthWidth = 320;
        depthHeight = 240;
        depthFps = 30;
        break;
    case 6:
        depthWidth = 320;
        depthHeight = 240;
        depthFps = 15;
        break;
    case 7:
        depthWidth = 320;
        depthHeight = 240;
        depthFps = 10;
        break;
    case 8:
        depthWidth = 320;
        depthHeight = 240;
        depthFps = 7;
        break;
    case 9:
        depthWidth = 640;
        depthHeight = 480;
        depthFps = 30;
        break;
    case 10:
        depthWidth = 640;
        depthHeight = 480;
        depthFps = 15;
        break;
    case 11:
        depthWidth = 640;
        depthHeight = 480;
        depthFps = 10;
        break;
    case 12:
        depthWidth = 640;
        depthHeight = 480;
        depthFps = 7;
        break;
    case 13:
        depthWidth = 1280;
        depthHeight = 1024;
        depthFps = 30;
        break;
    case 14:
        depthWidth = 1280;
        depthHeight = 1024;
        depthFps = 15;
        break;
    case 15:
        depthWidth = 1280;
        depthHeight = 1024;
        depthFps = 10;
        break;
    case 16:
        depthWidth = 1280;
        depthHeight = 1024;
        depthFps = 7;
        break;
    default:
        break;
    }
    mDepthSensorNode->reconfigDepthStream(depthWidth, depthHeight, depthFps);

    int irWidth = 0;
    int irHeight = 0;
    int irFps = 0;
    switch (config.ir_mode)
    {
    case 1:
        irWidth = 160;
        irHeight = 120;
        irFps = 30;
        break;
    case 2:
        irWidth = 160;
        irHeight = 120;
        irFps = 15;
        break;
    case 3:
        irWidth = 160;
        irHeight = 120;
        irFps = 10;
        break;
    case 4:
        irWidth = 160;
        irHeight = 120;
        irFps = 7;
        break;
    case 5:
        irWidth = 320;
        irHeight = 240;
        irFps = 30;
        break;
    case 6:
        irWidth = 320;
        irHeight = 240;
        irFps = 15;
        break;
    case 7:
        irWidth = 320;
        irHeight = 240;
        irFps = 10;
        break;
    case 8:
        irWidth = 320;
        irHeight = 240;
        irFps = 7;
        break;
    case 9:
        irWidth = 640;
        irHeight = 480;
        irFps = 30;
        break;
    case 10:
        irWidth = 640;
        irHeight = 480;
        irFps = 15;
        break;
    case 11:
        irWidth = 640;
        irHeight = 480;
        irFps = 10;
        break;
    case 12:
        irWidth = 640;
        irHeight = 480;
        irFps = 7;
        break;
    case 13:
        irWidth = 1280;
        irHeight = 1024;
        irFps = 30;
        break;
    case 14:
        irWidth = 1280;
        irHeight = 1024;
        irFps = 15;
        break;
    case 15:
        irWidth = 1280;
        irHeight = 1024;
        irFps = 10;
        break;
    case 16:
        irWidth = 1280;
        irHeight = 1024;
        irFps = 7;
        break;
    default:
        break;
    }
    mIrSensorNode->reconfigIrStream(irWidth, irHeight, irFps);
}