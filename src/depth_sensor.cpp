#include "depth_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"
#include "utils.h"

DepthSensor::DepthSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mDepthSensor(sensor)
{
    mDepthInfoService = mNodeHandle.advertiseService("depth/get_camera_info", &DepthSensor::getCameraInfoCallback, this);

    image_transport::ImageTransport it(nh);
    mDepthPub = it.advertise("camera/depth", 1);

    startDepthStream();
}

DepthSensor::~DepthSensor()
{
}

bool DepthSensor::getCameraInfoCallback(orbbec_camera::GetCameraInfoRequest& req, orbbec_camera::GetCameraInfoResponse& res)
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_COLOR);
    sensor_msgs::CameraInfo info = Utils::convertToCameraInfo(intrinsic);
    res.info = info;
}

void DepthSensor::startDepthStream()
{
    bool found = false;
    auto profiles = mDepthSensor->getStreamProfiles();
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

                                // sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                                // cinfo->width = frame->width();
                                // cinfo->height = frame->height();
                                // cinfo->header.frame_id = frame->index();

                                //    mDepthPub.publish(image, cinfo);

                                mDepthPub.publish(image);
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
