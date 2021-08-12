#include "orbbec_device.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"

OrbbecDevice::OrbbecDevice(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> dev) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(dev)
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
