#include "ir_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"

IrSensor::IrSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mIrSensor(sensor)
{
    getIrCameraInfo();

    image_transport::ImageTransport it(nh);
    mIrPub = it.advertise("camera/ir", 1);

    startIrStream();
}

IrSensor::~IrSensor()
{
}

sensor_msgs::CameraInfo IrSensor::convertToCameraInfo(OBCameraIntrinsic obParam)
{
    sensor_msgs::CameraInfo info;
    info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info.width = obParam.width;
    info.height = obParam.height;
    info.D.resize(5, 0.0);

    info.K.assign(0.0);
    info.K[0] = obParam.fx;
    info.K[2] = obParam.cx;
    info.K[4] = obParam.fy;
    info.K[5] = obParam.cy;
    info.K[8] = 1.0;

    info.R.assign(0.0);
    info.R[0] = 1;
    info.R[4] = 1;
    info.R[8] = 1;

    info.P.assign(0.0);
    info.P[0] = info.K[0];
    info.P[2] = info.K[2];
    info.P[5] = info.K[4];
    info.P[6] = info.K[5];
    info.P[10] = 1.0;

    // int width = mIrProfile->width();
    // double scaling = (double)width / 640;
    // info.K[0] *= scaling; // fx
    // info.K[2] *= scaling; // cx
    // info.K[4] *= scaling; // fy
    // info.K[5] *= scaling; // cy
    // info.P[0] *= scaling; // fx
    // info.P[2] *= scaling; // cx
    // info.P[5] *= scaling; // fy
    // info.P[6] *= scaling; // cy

    return info;
}

void IrSensor::getIrCameraInfo()
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_IR);
    sensor_msgs::CameraInfo info = convertToCameraInfo(intrinsic);
    sensor_msgs::CameraInfo::Ptr infoPtr(new sensor_msgs::CameraInfo(info));
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
