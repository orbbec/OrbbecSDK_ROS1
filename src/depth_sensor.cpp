#include "depth_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

DepthSensor::DepthSensor(ros::NodeHandle& nh, ros::NodeHandle& pnh, const std::shared_ptr<ob::Device> device,
                         const std::shared_ptr<ob::Sensor> sensor)
    : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mDepthSensor(sensor), mFrameId(""), mIsStreaming(false)
{
    mCameraInfoService =
        mNodeHandle.advertiseService("depth/get_camera_info", &DepthSensor::getCameraInfoCallback, this);
    // mGetExposureService = mNodeHandle.advertiseService("depth/get_exposure", &DepthSensor::getExposureCallback,
    // this); mSetExposureService = mNodeHandle.advertiseService("depth/set_exposure",
    // &DepthSensor::setExposureCallback, this); mGetGainService = mNodeHandle.advertiseService("depth/get_gain",
    // &DepthSensor::getGainCallback, this); mSetGainService = mNodeHandle.advertiseService("depth/set_gain",
    // &DepthSensor::setGainCallback, this); mGetWhiteBalanceService =
    //     mNodeHandle.advertiseService("depth/get_white_balance", &DepthSensor::getWhiteBalanceCallback, this);
    // mSetWhiteBalanceService =
    //     mNodeHandle.advertiseService("depth/set_white_balance", &DepthSensor::setWhiteBalanceCallback, this);
    // mSetAutoExposureService =
    //     mNodeHandle.advertiseService("depth/set_auto_exposure", &DepthSensor::setAutoExposureCallback, this);
    // mSetAutoWhiteBalanceService =
    //     mNodeHandle.advertiseService("depth/set_auto_white_balance", &DepthSensor::setAutoWhiteBalanceCallback,
    //     this);
    mEnableStreamService =
        mNodeHandle.advertiseService("depth/enable_stream", &DepthSensor::enableStreamCallback, this);

    image_transport::ImageTransport it(nh);
    mDepthPub = it.advertise("depth/image_raw", 1);
    // mDepthPub = it.advertiseCamera("depth/image_raw", 1);
    mCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);

    startDepthStream();
}

DepthSensor::~DepthSensor()
{
}

bool DepthSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req,
                                        orbbec_camera::GetCameraInfoResponse& res)
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_DEPTH);
    OBCameraDistortion distortion = mDevice->getCameraDistortion(OB_SENSOR_DEPTH);
    sensor_msgs::CameraInfo info = Utils::convertToCameraInfo(intrinsic, distortion);
    res.info = info;
    return true;
}

bool DepthSensor::getExposureCallback(orbbec_camera::GetExposureRequest& req, orbbec_camera::GetExposureResponse& res)
{
    int32_t exposure = mDepthSensor->getIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT);
    res.value = exposure;
    return true;
}

bool DepthSensor::setExposureCallback(orbbec_camera::SetExposureRequest& req, orbbec_camera::SetExposureResponse& res)
{
    int32_t exposure = req.value;
    mDepthSensor->setIntProperty(OB_SENSOR_PROPERTY_EXPOSURE_INT, exposure);
    return true;
}

bool DepthSensor::getGainCallback(orbbec_camera::GetGainRequest& req, orbbec_camera::GetGainResponse& res)
{
    int32_t gain = mDepthSensor->getIntProperty(OB_SENSOR_PROPERTY_GAIN_INT);
    res.value = gain;
    return true;
}

bool DepthSensor::setGainCallback(orbbec_camera::SetGainRequest& req, orbbec_camera::SetGainResponse& res)
{
    int32_t gain = req.value;
    mDepthSensor->setIntProperty(OB_SENSOR_PROPERTY_GAIN_INT, gain);
    return true;
}

bool DepthSensor::getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req,
                                          orbbec_camera::GetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = mDepthSensor->getIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT);
    res.value = whiteBalance;
    return true;
}

bool DepthSensor::setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req,
                                          orbbec_camera::SetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = req.value;
    mDepthSensor->setIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT, whiteBalance);
    return true;
}

bool DepthSensor::setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req,
                                          orbbec_camera::SetAutoExposureResponse& res)
{
    bool autoExposure = req.enable;
    mDepthSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL, autoExposure);
    return true;
}

bool DepthSensor::setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req,
                                              orbbec_camera::SetAutoWhiteBalanceResponse& res)
{
    bool autoWhiteBalance = req.enable;
    mDepthSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL, autoWhiteBalance);
    return true;
}

bool DepthSensor::enableStreamCallback(orbbec_camera::EnableStreamRequest& req,
                                       orbbec_camera::EnableStreamResponse& res)
{
    bool enable = req.enable;
    if (enable)
    {
        startDepthStream();
    }
    else
    {
        stopDepthStream();
    }
    return true;
}

void DepthSensor::startDepthStream()
{
    if (mIsStreaming)
        return;

    if (mDepthProfile == nullptr)
    {
        mDepthProfile = findProfile();
    }
    if (mDepthProfile != nullptr)
    {
        mDepthSensor->start(mDepthProfile, [&](std::shared_ptr<ob::Frame> frame) {
            auto depthFrame = frame->as<ob::VideoFrame>();
            int width = depthFrame->width();
            int height = depthFrame->height();

            if (mInfo.width != width || mInfo.height != height)
            {
                OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_DEPTH);
                OBCameraDistortion distortion = mDevice->getCameraDistortion(OB_SENSOR_DEPTH);
                mInfo = Utils::convertToCameraInfo(intrinsic, distortion);
                mInfo.width = width;
                mInfo.height = height;
            }
            ros::Time ros_now = ros::Time::now();

            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->header.stamp = ros_now;
            image->header.frame_id = "orbbec_depth_frame";
            image->width = width;
            image->height = height;
            image->step = width * 2;
            image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
            image->data.resize(depthFrame->dataSize());
            memcpy(&image->data[0], depthFrame->data(), depthFrame->dataSize());

            sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
            cinfo->header.stamp = ros_now;
            cinfo->header.frame_id = "orbbec_depth_frame";
            cinfo->width = width;
            cinfo->height = height;
            cinfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

            mDepthPub.publish(image);
            // mDepthPub.publish(image, cinfo);
            mCameraInfoPub.publish(cinfo);
        });
        mIsStreaming = true;
        ROS_INFO("Start depth stream: %dx%d(%d)", mDepthProfile->width(), mDepthProfile->height(),
                 mDepthProfile->fps());
    }
    else
    {
        ROS_WARN("Start depth stream failed: no profile found");
    }
}

void DepthSensor::stopDepthStream()
{
    if (!mIsStreaming)
        return;

    mDepthSensor->stop();
    mIsStreaming = false;
    ROS_INFO("Stop depth stream");
}

void DepthSensor::reconfigDepthStream(int width, int height, int fps)
{
    if (mDepthProfile != nullptr && mDepthProfile->width() == width && mDepthProfile->height() == height &&
        mDepthProfile->fps() == fps)
    {
        return;
    }
    else
    {
        auto profile = findProfile(width, height, fps);
        if (profile != nullptr)
        {
            mDepthProfile = profile;
            if (mIsStreaming)
            {
                stopDepthStream();
                startDepthStream();
                // mDepthSensor->switchProfile(mDepthProfile);
            }
            ROS_INFO("Reconfig depth stream: %dx%d(%d)", mDepthProfile->width(), mDepthProfile->height(),
                     mDepthProfile->fps());
        }
        else
        {
            ROS_WARN("Reconfig depth stream failed: no profile found");
        }
    }
}

std::shared_ptr<ob::StreamProfile> DepthSensor::findProfile(int width, int height, int fps)
{
    auto profiles = mDepthSensor->getStreamProfileList();
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
