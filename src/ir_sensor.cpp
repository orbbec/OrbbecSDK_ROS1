#include "ir_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

IrSensor::IrSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mIrSensor(sensor)
{
    mGetCameraInfoService = mNodeHandle.advertiseService("ir/get_camera_info", &IrSensor::getCameraInfoCallback, this);
    mGetExposureService = mNodeHandle.advertiseService("ir/get_exposure", &IrSensor::getExposureCallback, this);
    mSetExposureService = mNodeHandle.advertiseService("ir/set_exposure", &IrSensor::setExposureCallback, this);
    mGetGainService = mNodeHandle.advertiseService("ir/get_gain", &IrSensor::getGainCallback, this);
    mSetGainService = mNodeHandle.advertiseService("ir/set_gain", &IrSensor::setGainCallback, this);
    mGetWhiteBalanceService = mNodeHandle.advertiseService("ir/get_white_balance", &IrSensor::getWhiteBalanceCallback, this);
    mSetWhiteBalanceService = mNodeHandle.advertiseService("ir/set_white_balance", &IrSensor::setWhiteBalanceCallback, this);
    mSetAutoExposureService = mNodeHandle.advertiseService("ir/set_auto_exposure", &IrSensor::setAutoExposureCallback, this);
    mSetAutoWhiteBalanceService = mNodeHandle.advertiseService("ir/set_auto_white_balance", &IrSensor::setAutoWhiteBalanceCallback, this);

    image_transport::ImageTransport it(nh);
    mIrPub = it.advertise("camera/ir", 1);

    startIrStream();
}

IrSensor::~IrSensor()
{
}

bool IrSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res)
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_COLOR);
    sensor_msgs::CameraInfo info = Utils::convertToCameraInfo(intrinsic);
    res.info = info;
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

bool IrSensor::getWhiteBalanceCallback(orbbec_camera::GetWhiteBalanceRequest& req, orbbec_camera::GetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = mIrSensor->getIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT);
    res.value = whiteBalance;
    return true;
}

bool IrSensor::setWhiteBalanceCallback(orbbec_camera::SetWhiteBalanceRequest& req, orbbec_camera::SetWhiteBalanceResponse& res)
{
    int32_t whiteBalance = req.value;
    mIrSensor->setIntProperty(OB_SENSOR_PROPERTY_WHITE_BALANCE_INT, whiteBalance);
    return true; 
}

bool IrSensor::setAutoExposureCallback(orbbec_camera::SetAutoExposureRequest& req, orbbec_camera::SetAutoExposureResponse& res)
{
    bool autoExposure = req.enable;
    mIrSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_EXPOSURE_BOOL, autoExposure);
    return true;
}

bool IrSensor::setAutoWhiteBalanceCallback(orbbec_camera::SetAutoWhiteBalanceRequest& req, orbbec_camera::SetAutoWhiteBalanceResponse& res)
{
    bool autoWhiteBalance = req.enable;
    mIrSensor->setBoolProperty(OB_SENSOR_PROPERTY_ENABLE_AUTO_WHITE_BALANCE_BOOL, autoWhiteBalance);
    return true;
}

void IrSensor::startIrStream()
{
    bool found = false;
    auto profiles = mIrSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16)
        {
            mIrProfile = profile;
            found = true;
            break;
        }
    }

    if (found)
    {
        mIrSensor->start(mIrProfile, [&](std::shared_ptr<ob::Frame> frame)
                         {
                             sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
                             image->width = frame->width();
                             image->height = frame->height();
                             image->step = frame->width() * 2;
                             image->encoding = sensor_msgs::image_encodings::MONO16;
                             image->data.resize(frame->dataSize());
                             memcpy(&image->data[0], frame->data(), frame->dataSize());

                            //  sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                            //  cinfo->width = frame->width();
                            //  cinfo->height = frame->height();
                            //  cinfo->header.frame_id = frame->index();

                             // mIrPub.publish(image, cinfo);

                             mIrPub.publish(image);
                         });
        ROS_INFO("Start ir stream: %dx%d(%d)", mIrProfile->width(), mIrProfile->height(), mIrProfile->fps());
    }
    else
    {
        ROS_WARN("Start ir stream failed: no profile found");
    }
}

void IrSensor::stopIrStream()
{
    mIrSensor->stop();
    ROS_INFO("Stop ir stream");
}

void IrSensor::reconfigIrStream(int width, int height, int fps)
{
    bool found = false;
    auto profiles = mIrSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16 && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mIrProfile = profile;
            found = true;
            break;
        }
    }
    if (found)
    {
        mIrSensor->switchProfile(mIrProfile);
        ROS_INFO("Reconfig ir stream: %dx%d(%d)", mIrProfile->width(), mIrProfile->height(), mIrProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig ir stream failed: no profile found");
    }
}
