#include "color_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

ColorSensor::ColorSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<ob::Device> device,
                         const std::shared_ptr<ob::Sensor> sensor)
    : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mColorSensor(sensor), mIsStreaming(false),
      mArgbBufferSize(0), mArgbBuffer(NULL), mRgbBufferSize(0), mRgbBuffer(NULL)
{
    mGetCameraInfoService =
        mNodeHandle.advertiseService("color/get_camera_info", &ColorSensor::getCameraInfoCallback, this);
    mGetExposureService = mNodeHandle.advertiseService("color/get_exposure", &ColorSensor::getExposureCallback, this);
    mSetExposureService = mNodeHandle.advertiseService("color/set_exposure", &ColorSensor::setExposureCallback, this);
    mGetGainService = mNodeHandle.advertiseService("color/get_gain", &ColorSensor::getGainCallback, this);
    mSetGainService = mNodeHandle.advertiseService("color/set_gain", &ColorSensor::setGainCallback, this);
    mGetWhiteBalanceService =
        mNodeHandle.advertiseService("color/get_white_balance", &ColorSensor::getWhiteBalanceCallback, this);
    mSetWhiteBalanceService =
        mNodeHandle.advertiseService("color/set_white_balance", &ColorSensor::setWhiteBalanceCallback, this);
    mSetAutoExposureService =
        mNodeHandle.advertiseService("color/set_auto_exposure", &ColorSensor::setAutoExposureCallback, this);
    mSetAutoWhiteBalanceService =
        mNodeHandle.advertiseService("color/set_auto_white_balance", &ColorSensor::setAutoWhiteBalanceCallback, this);
    mEnableStreamService =
        mNodeHandle.advertiseService("color/enable_stream", &ColorSensor::enableStreamCallback, this);

    image_transport::ImageTransport it(nh);
    mColorPub = it.advertise("color/image_raw", 1);
    mCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("color/camera_info", 1);

    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_COLOR);
    mInfo = Utils::convertToCameraInfo(intrinsic);

    startColorStream();
}

ColorSensor::~ColorSensor()
{
    // if (mArgbBuffer)
    // {
    //     free(mArgbBuffer);
    //     mArgbBuffer = NULL;
    // }
    // if (mRgbBuffer)
    // {
    //     free(mRgbBuffer);
    //     mRgbBuffer = NULL;
    // }
}

void* ColorSensor::getArgbBuffer(size_t bufferSize)
{
    if (bufferSize != mArgbBufferSize)
    {
        if (mArgbBuffer)
        {
            free(mArgbBuffer);
            mArgbBuffer = NULL;
        }
        mArgbBuffer = malloc(bufferSize);
        mArgbBufferSize = bufferSize;
    }
    return mArgbBuffer;
}

void* ColorSensor::getRgbBuffer(size_t bufferSize)
{
    if (bufferSize != mRgbBufferSize)
    {
        if (mRgbBuffer)
        {
            free(mRgbBuffer);
            mRgbBuffer = NULL;
        }
        mRgbBuffer = malloc(bufferSize);
        mRgbBufferSize = bufferSize;
    }
    return mRgbBuffer;
}

bool ColorSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req,
                                        orbbec_camera::GetCameraInfoResponse& res)
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_COLOR);
    sensor_msgs::CameraInfo info = Utils::convertToCameraInfo(intrinsic);
    res.info = info;
    return true;
}

bool ColorSensor::getExposureCallback(orbbec_camera::GetExposureRequest& req, orbbec_camera::GetExposureResponse& res)
{
    int32_t exposure = mColorSensor->getIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT);
    res.value = exposure;
    return true;
}

bool ColorSensor::setExposureCallback(orbbec_camera::SetExposureRequest& req, orbbec_camera::SetExposureResponse& res)
{
    int32_t exposure = req.value;
    mColorSensor->setIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT, exposure);
    return true;
}

bool ColorSensor::getGainCallback(orbbec_camera::GetGainRequest& req, orbbec_camera::GetGainResponse& res)
{
    int32_t gain = mColorSensor->getIntProperty(OB_SENSOR_PROPERTY_GAIN_INT);
    res.value = gain;
    return true;
}

bool ColorSensor::setGainCallback(orbbec_camera::SetGainRequest& req, orbbec_camera::SetGainResponse& res)
{
    int32_t gain = req.value;
    mColorSensor->setIntProperty(OB_SENSOR_PROPERTY_GAIN_INT, gain);
    return true;
}

bool ColorSensor::getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req,
                                          orbbec_camera::GetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = mColorSensor->getIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT);
    res.value = whiteBalance;
    return true;
}

bool ColorSensor::setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req,
                                          orbbec_camera::SetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = req.value;
    mColorSensor->setIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT, whiteBalance);
    return true;
}

bool ColorSensor::setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req,
                                          orbbec_camera::SetAutoExposureResponse& res)
{
    bool autoExposure = req.enable;
    mColorSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL, autoExposure);
    return true;
}

bool ColorSensor::setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req,
                                              orbbec_camera::SetAutoWhiteBalanceResponse& res)
{
    bool autoWhiteBalance = req.enable;
    mColorSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL, autoWhiteBalance);
    return true;
}

bool ColorSensor::enableStreamCallback(orbbec_camera::EnableStreamRequest& req,
                                       orbbec_camera::EnableStreamResponse& res)
{
    bool enable = req.enable;
    if (enable)
    {
        startColorStream();
    }
    else
    {
        stopColorStream();
    }
    return true;
}

void ColorSensor::startColorStream()
{
    if (mIsStreaming)
        return;

    if (mColorProfile == nullptr)
    {
        mColorProfile = findProfile();
    }
    if (mColorProfile != nullptr)
    {
        mColorSensor->start(mColorProfile, [&](std::shared_ptr<ob::Frame> frame) {
            int width = frame->width();
            int height = frame->height();

            void* argbBuffer = getArgbBuffer(width * height * 4);
            libyuv::MJPGToARGB((uint8_t*)frame->data(), frame->dataSize(), (uint8_t*)argbBuffer, width * 4, width,
                               height, width, height);

            void* rgbBuffer = getRgbBuffer(width * height * 3);
            libyuv::ARGBToRGB24((uint8_t*)argbBuffer, width * 4, (uint8_t*)rgbBuffer, width * 3, width, height);

            ros::Time ros_now = ros::Time::now();

            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->header.stamp = ros_now;
            image->header.frame_id = "orbbec_color_frame";
            image->width = width;
            image->height = height;
            image->step = width * 3;
            image->encoding = sensor_msgs::image_encodings::BGR8;
            image->data.resize(mRgbBufferSize);
            memcpy(&image->data[0], mRgbBuffer, mRgbBufferSize);

            sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
            cinfo->header.stamp = ros_now;
            cinfo->header.frame_id = "orbbec_color_frame";
            cinfo->width = frame->width();
            cinfo->height = frame->height();
            cinfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            //    mColorPub.publish(image, cinfo);

            mColorPub.publish(image);
            mCameraInfoPub.publish(cinfo);
        });
        mIsStreaming = true;
        ROS_INFO("Start color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(),
                 mColorProfile->fps());
    }
    else
    {
        ROS_WARN("Start color stream failed: no profile found");
    }
}

void ColorSensor::stopColorStream()
{
    if (!mIsStreaming)
        return;

    mColorSensor->stop();
    mIsStreaming = false;
    ROS_INFO("Stop color stream");
}

void ColorSensor::reconfigColorStream(int width, int height, int fps)
{
    if (mColorProfile != nullptr && mColorProfile->width() == width && mColorProfile->height() == height &&
        mColorProfile->fps() == fps)
    {
        return;
    }
    else
    {
        auto profile = findProfile(width, height, fps);
        if (profile != nullptr)
        {
            mColorProfile = profile;
            if (mIsStreaming)
            {
                mColorSensor->switchProfile(mColorProfile);
            }
            ROS_INFO("Reconfig color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(),
                     mColorProfile->fps());
        }
        else
        {
            ROS_WARN("Reconfig color stream failed: no profile found");
        }
    }
}

std::shared_ptr<ob::StreamProfile> ColorSensor::findProfile(int width, int height, int fps)
{
    auto profiles = mColorSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_MJPG)
        {
            if (width == 0 && height == 0 && fps == 0)
            {
                return profile;
            }
            if (profile->width() == width && profile->height() == height && profile->fps() == fps)
            {
                return profile;
            }
            return profile;
        }
    }
    return nullptr;
}
