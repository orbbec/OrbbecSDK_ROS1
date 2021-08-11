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
