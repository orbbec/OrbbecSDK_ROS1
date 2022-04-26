#include "ir_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

IrSensor::IrSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<ob::Device> device,
                   const std::shared_ptr<ob::Sensor> sensor)
    : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mIrSensor(sensor), mIsStreaming(false)
{
    mGetCameraInfoService = mNodeHandle.advertiseService("ir/get_camera_info", &IrSensor::getCameraInfoCallback, this);
    mGetExposureService = mNodeHandle.advertiseService("ir/get_exposure", &IrSensor::getExposureCallback, this);
    mSetExposureService = mNodeHandle.advertiseService("ir/set_exposure", &IrSensor::setExposureCallback, this);
    mGetGainService = mNodeHandle.advertiseService("ir/get_gain", &IrSensor::getGainCallback, this);
    mSetGainService = mNodeHandle.advertiseService("ir/set_gain", &IrSensor::setGainCallback, this);
    mEnableStreamService = mNodeHandle.advertiseService("ir/enable_stream", &IrSensor::enableStreamCallback, this);

    image_transport::ImageTransport it(nh);
    mIrPub = it.advertise("ir/image", 1);

    startIrStream();
}

IrSensor::~IrSensor() = default;

bool IrSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req,
                                     orbbec_camera::GetCameraInfoResponse& res)
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_IR);
    OBCameraDistortion distortion = mDevice->getCameraDistortion(OB_SENSOR_IR);
    sensor_msgs::CameraInfo info = Utils::convertToCameraInfo(intrinsic, distortion);
    res.info = info;
    return true;
}

bool IrSensor::getExposureCallback(orbbec_camera::GetExposureRequest& req, orbbec_camera::GetExposureResponse& res)
{
    int32_t exposure = mIrSensor->getIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT);
    res.value = exposure;
    return true;
}

bool IrSensor::setExposureCallback(orbbec_camera::SetExposureRequest& req, orbbec_camera::SetExposureResponse& res)
{
    int32_t exposure = req.value;
    mIrSensor->setIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT, exposure);
    return true;
}

bool IrSensor::getGainCallback(orbbec_camera::GetGainRequest& req, orbbec_camera::GetGainResponse& res)
{
    int32_t gain = mIrSensor->getIntProperty(OB_SENSOR_PROPERTY_GAIN_INT);
    res.value = gain;
    return true;
}

bool IrSensor::setGainCallback(orbbec_camera::SetGainRequest& req, orbbec_camera::SetGainResponse& res)
{
    int32_t gain = req.value;
    mIrSensor->setIntProperty(OB_SENSOR_PROPERTY_GAIN_INT, gain);
    return true;
}

bool IrSensor::getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req,
                                       orbbec_camera::GetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = mIrSensor->getIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT);
    res.value = whiteBalance;
    return true;
}

bool IrSensor::setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req,
                                       orbbec_camera::SetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = req.value;
    mIrSensor->setIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT, whiteBalance);
    return true;
}

bool IrSensor::setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req,
                                       orbbec_camera::SetAutoExposureResponse& res)
{
    bool autoExposure = req.enable;
    mIrSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL, autoExposure);
    return true;
}

bool IrSensor::setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req,
                                           orbbec_camera::SetAutoWhiteBalanceResponse& res)
{
    bool autoWhiteBalance = req.enable;
    mIrSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL, autoWhiteBalance);
    return true;
}

bool IrSensor::enableStreamCallback(orbbec_camera::EnableStreamRequest& req, orbbec_camera::EnableStreamResponse& res)
{
    bool enable = req.enable;
    if (enable)
    {
        startIrStream();
    }
    else
    {
        stopIrStream();
    }
    return true;
}

void IrSensor::startIrStream()
{
    if (mIsStreaming)
        return;

    if (mIrProfile == nullptr)
    {
        mIrProfile = findProfile();
    }
    if (mIrProfile != nullptr)
    {
        mIrSensor->start(mIrProfile, [&](std::shared_ptr<ob::Frame> frame) {
            auto irFrame = frame->as<ob::VideoFrame>();

            ros::Time ros_now = ros::Time::now();

            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->header.stamp = ros_now;
            image->width = irFrame->width();
            image->height = irFrame->height();
            image->step = irFrame->width() * 2;
            image->encoding = sensor_msgs::image_encodings::MONO16;
            image->data.resize(irFrame->dataSize());
            memcpy(&image->data[0], irFrame->data(), irFrame->dataSize());
            mIrPub.publish(image);
        });
        mIsStreaming = true;
        ROS_INFO("Start ir stream: %dx%d(%d)", mIrProfile->width(), mIrProfile->height(), mIrProfile->fps());
    }
    else
    {
        ROS_WARN("Start ir stream failed: no profile found");
    }
}

void IrSensor::stopIrStream()
{
    if (!mIsStreaming)
        return;

    mIrSensor->stop();
    mIsStreaming = false;
    ROS_INFO("Stop ir stream");
}

void IrSensor::reconfigIrStream(int width, int height, int fps)
{
    if (mIrProfile != nullptr && mIrProfile->width() == width && mIrProfile->height() == height &&
        mIrProfile->fps() == fps)
    {
        return;
    }
    else
    {
        auto profile = findProfile(width, height, fps);
        if (profile != nullptr)
        {
            mIrProfile = profile;
            if (mIsStreaming)
            {
                stopIrStream();
                startIrStream();
                // mIrSensor->switchProfile(mIrProfile);
            }
            ROS_INFO("Reconfig ir stream: %dx%d(%d)", mIrProfile->width(), mIrProfile->height(), mIrProfile->fps());
        }
        else
        {
            ROS_WARN("Reconfig ir stream failed: no profile found");
        }
    }
}

std::shared_ptr<ob::StreamProfile> IrSensor::findProfile(int width, int height, int fps)
{
    auto profiles = mIrSensor->getStreamProfileList();
    for (int i = 0; i < profiles->count(); i++)
    {
        auto profile = profiles->getProfile(i);
        if (profile->format() == OB_FORMAT_Y16)
        {
            if (width == 0 && height == 0 && fps == 0)
            {
                return profile;
            }
            if (profile->width() == width && profile->height() == height && profile->fps() == fps)
            {
                return profile;
            }
        }
    }
    return nullptr;
}