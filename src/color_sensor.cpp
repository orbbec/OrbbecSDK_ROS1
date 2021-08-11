#include "color_sensor.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "sensor_msgs/distortion_models.h"
#include "libyuv.h"

ColorSensor::ColorSensor(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor) : mNodeHandle(nh), mPrivateNodeHandle(pnh), mDevice(device), mColorSensor(sensor), mArgbBufferSize(0), mArgbBuffer(NULL), mRgbBufferSize(0), mRgbBuffer(NULL)
{
    getColorCameraInfo();

    image_transport::ImageTransport it(nh);
    mColorPub = it.advertise("camera/color", 1);

    startColorStream();
}

ColorSensor::~ColorSensor()
{
    if (mArgbBuffer)
    {
        free(mArgbBuffer);
        mArgbBuffer = NULL;
    }
    if (mRgbBuffer)
    {
        free(mRgbBuffer);
        mRgbBuffer = NULL;
    }
}

void *ColorSensor::getArgbBuffer(size_t bufferSize)
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

void *ColorSensor::getRgbBuffer(size_t bufferSize)
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

sensor_msgs::CameraInfo ColorSensor::convertToCameraInfo(OBCameraIntrinsic obParam)
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

    // int width = mColorProfile->width();
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

void ColorSensor::getColorCameraInfo()
{
    OBCameraIntrinsic intrinsic = mDevice->getCameraIntrinsic(OB_SENSOR_COLOR);
    sensor_msgs::CameraInfo info = convertToCameraInfo(intrinsic);
    sensor_msgs::CameraInfo::Ptr infoPtr(new sensor_msgs::CameraInfo(info));
}

void ColorSensor::startColorStream()
{
    bool found = false;
    auto profiles = mColorSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_MJPG)
        {
            mColorProfile = profile;
            found = true;
            break;
        }
    }

    if (found)
    {
        mColorSensor->start(mColorProfile, [&](std::shared_ptr<ob::Frame> frame)
                            {
                                int width = frame->width();
                                int height = frame->height();

                                void *argbBuffer = getArgbBuffer(width * height * 4);
                                libyuv::MJPGToARGB((uint8_t *)frame->data(), frame->dataSize(), (uint8_t *)argbBuffer, width * 4, width, height, width, height);

                                void *rgbBuffer = getRgbBuffer(width * height * 3);
                                libyuv::ARGBToRGB24((uint8_t *)argbBuffer, width * 4, (uint8_t *)rgbBuffer, width * 3, width, height);

                                sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
                                image->width = width;
                                image->height = height;
                                image->step = width * 3;
                                image->encoding = sensor_msgs::image_encodings::BGR8;
                                image->data.resize(mRgbBufferSize);
                                memcpy(&image->data[0], mRgbBuffer, mRgbBufferSize);

                                // sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                                // cinfo->width = frame->width();
                                // cinfo->height = frame->height();
                                // cinfo->header.frame_id = frame->index();

                                //    mColorPub.publish(image, cinfo);

                                mColorPub.publish(image);
                            });
        ROS_INFO("Start color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(), mColorProfile->fps());
    }
    else
    {
        ROS_WARN("Start color stream failed: no profile found");
    }
}

void ColorSensor::stopColorStream()
{
    mColorSensor->stop();
    ROS_INFO("Stop color stream");
}

void ColorSensor::reconfigColorStream(int width, int height, int fps)
{
    bool found = false;
    auto profiles = mColorSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_MJPG && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mColorProfile = profile;
            found = true;
            break;
        }
    }
    if (found)
    {
        mColorSensor->switchProfile(mColorProfile);
        ROS_INFO("Reconfig color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(), mColorProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig color stream failed: no profile found");
    }
}
