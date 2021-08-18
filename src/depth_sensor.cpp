#include "depth_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

DepthSensor::DepthSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mDepthSensor(sensor), mFrameId("")
{
    mCameraInfoService = mNodeHandle.advertiseService("depth/get_camera_info", &DepthSensor::getCameraInfoCallback, this);
    mGetExposureService = mNodeHandle.advertiseService("depth/get_exposure", &DepthSensor::getExposureCallback, this);
    mSetExposureService = mNodeHandle.advertiseService("depth/set_exposure", &DepthSensor::setExposureCallback, this);
    mGetGainService = mNodeHandle.advertiseService("depth/get_gain", &DepthSensor::getGainCallback, this);
    mSetGainService = mNodeHandle.advertiseService("depth/set_gain", &DepthSensor::setGainCallback, this);
    mGetWhiteBalanceService = mNodeHandle.advertiseService("depth/get_white_balance", &DepthSensor::getWhiteBalanceCallback, this);
    mSetWhiteBalanceService = mNodeHandle.advertiseService("depth/set_white_balance", &DepthSensor::setWhiteBalanceCallback, this);
    mSetAutoExposureService = mNodeHandle.advertiseService("depth/set_auto_exposure", &DepthSensor::setAutoExposureCallback, this);
    mSetAutoWhiteBalanceService = mNodeHandle.advertiseService("depth/set_auto_white_balance", &DepthSensor::setAutoWhiteBalanceCallback, this);

    image_transport::ImageTransport it(nh);
    // mDepthPub = it.advertise("camera/depth", 1);
    mDepthPub = it.advertiseCamera("camera/depth/image_raw", 1);
    mCameraInfoPub = nh.advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info", 1);

    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_DEPTH);
    mInfo = Utils::convertToCameraInfo(intrinsic);

    startDepthStream();
}

DepthSensor::~DepthSensor()
{
}

bool DepthSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res)
{
    res.info = mInfo;
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

bool DepthSensor::getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req, orbbec_camera::GetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = mDepthSensor->getIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT);
    res.value = whiteBalance;
    return true;
}

bool DepthSensor::setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req, orbbec_camera::SetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = req.value;
    mDepthSensor->setIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT, whiteBalance);
    return true; 
}

bool DepthSensor::setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req, orbbec_camera::SetAutoExposureResponse& res)
{
    bool autoExposure = req.enable;
    mDepthSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL, autoExposure);
    return true;
}

bool DepthSensor::setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req, orbbec_camera::SetAutoWhiteBalanceResponse& res)
{
    bool autoWhiteBalance = req.enable;
    mDepthSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL, autoWhiteBalance);
    return true;
}

void DepthSensor::startDepthStream()
{
    bool found = false;
    auto profiles = mDepthSensor->getStreamProfiles();
    // for (int i = 0; i < profiles.size(); i++)
    // {
    //     auto profile = profiles[i];
    //     if (profile->format() == OB_FORMAT_Y16)
    //     {
    //         ROS_INFO("Depth profile: %d x %d (%d)", profile->width(), profile->height(), profile->fps());
    //     }
    // }
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16)
        {
            mDepthProfile = profile;
            found = true;
            break;
        }
    }

    if (found)
    {
        mDepthSensor->start(mDepthProfile, [&](std::shared_ptr<ob::Frame> frame)
                            {
                                sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
                                image->width = frame->width();
                                image->height = frame->height();
                                image->step = frame->width() * 2;
                                image->encoding = sensor_msgs::image_encodings::MONO16;
                                image->data.resize(frame->dataSize());
                                memcpy(&image->data[0], frame->data(), frame->dataSize());

                                sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                                cinfo->width = frame->width();
                                cinfo->height = frame->height();
                                cinfo->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
                                cinfo->header.frame_id = mFrameId;
                                cinfo->header.stamp = ros::Time(frame->timeStamp());

                                // mDepthPub.publish(image);
                                mDepthPub.publish(image, cinfo);
                                mCameraInfoPub.publish(cinfo);
                            });
        ROS_INFO("Start depth stream: %dx%d(%d)", mDepthProfile->width(), mDepthProfile->height(), mDepthProfile->fps());
    }
    else
    {
        ROS_WARN("Start depth stream failed: no profile found");
    }
}

void DepthSensor::stopDepthStream()
{
    mDepthSensor->stop();
    ROS_INFO("Stop depth stream");
}

void DepthSensor::reconfigDepthStream(int width, int height, int fps)
{
    if(mDepthProfile == nullptr)
    {
        return;
    }
    if(width == mDepthProfile->width() && height == mDepthProfile->height() && fps == mDepthProfile->fps())
    {
        return;
    }
    bool found = false;
    auto profiles = mDepthSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16 && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mDepthProfile = profile;
            found = true;
            break;
        }
    }
    if (found)
    {
        mDepthSensor->switchProfile(mDepthProfile);
        ROS_INFO("Reconfig depth stream: %dx%d(%d)", mDepthProfile->width(), mDepthProfile->height(), mDepthProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig depth stream failed: no profile found");
    }
}
