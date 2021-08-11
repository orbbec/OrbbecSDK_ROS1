#include "orbbec_device.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "libobsensor/hpp/StreamProfile.hpp"
#include "libobsensor/hpp/Frame.hpp"
#include "std_msgs/String.h"
#include "libyuv.h"

OrbbecDevice::OrbbecDevice(ros::NodeHandle &nh, ros::NodeHandle &pnh, std::shared_ptr<ob::Device> dev) : mNodeHandle(nh), mPrivateNodeHandle(pnh) /*, colorIt(nh), depthIt(nh), irIt(nh)*/, mDevice(dev), mArgbBufferSize(0), mArgbBuffer(NULL), mRgbBufferSize(0), mRgbBuffer(NULL)
{
    // mColorPub = colorIt.advertiseCamera("image/color", 1);
    // mDepthPub = depthIt.advertiseCamera("image/depth", 1);
    // mIrPub = irIt.advertiseCamera("image/ir", 1);

    image_transport::ImageTransport it(nh);
    mColorPub = it.advertise("camera/color", 1);
    mDepthPub = it.advertise("camera/depth", 1);
    mIrPub = it.advertise("camera/ir", 1);

    mColorSensor = dev->getSensor(OB_SENSOR_COLOR);
    mDepthSensor = dev->getSensor(OB_SENSOR_DEPTH);
    mIrSensor = dev->getSensor(OB_SENSOR_IR);

    startColorStream();
    startDepthStream();
    startIrStream();
}

OrbbecDevice::~OrbbecDevice()
{
}

void* OrbbecDevice::getArgbBuffer(size_t bufferSize)
{
    if(bufferSize != mArgbBufferSize)
    {
        if(mArgbBuffer)
        {
            free(mArgbBuffer);
        }
        mArgbBuffer = malloc(bufferSize);
        mArgbBufferSize = bufferSize;
    }
    return mArgbBuffer;
}

void* OrbbecDevice::getRgbBuffer(size_t bufferSize)
{
    if(bufferSize != mRgbBufferSize)
    {
        if(mRgbBuffer)
        {
            free(mRgbBuffer);
        }
        mRgbBuffer = malloc(bufferSize);
        mRgbBufferSize = bufferSize;
    }
    return mRgbBuffer;
}

void OrbbecDevice::startColorStream()
{
    auto profiles = mColorSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_MJPG)
        {
            mColorProfile = profile;
            break;
        }
    }

    if (mColorProfile != NULL)
    {
        mColorSensor->start(mColorProfile, [&](std::shared_ptr<ob::Frame> frame)
                            {
                                int width = frame->width();
                                int height = frame->height();

                                void* argbBuffer = getArgbBuffer(width * height * 4);
                                libyuv::MJPGToARGB((uint8_t*)frame->data(), frame->dataSize(), (uint8_t*)argbBuffer, width * 4, width, height, width, height);

                                void* rgbBuffer = getRgbBuffer(width * height * 3);
                                libyuv::ARGBToRGB24((uint8_t*)argbBuffer, width * 4, (uint8_t*)rgbBuffer, width * 3, width, height);

                                sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
                                image->width = width;
                                image->height = height;
                                image->step = width * 3;
                                image->encoding = sensor_msgs::image_encodings::BGR8;
                                image->data.resize(mRgbBufferSize);
                                memcpy(&image->data[0], mRgbBuffer, mRgbBufferSize);

                                sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                                cinfo->width = frame->width();
                                cinfo->height = frame->height();
                                cinfo->header.frame_id = frame->index();

                                //    mColorPub.publish(image, cinfo);

                                mColorPub.publish(image);
                            });
    }
}

void OrbbecDevice::stopColorStream()
{
    mColorSensor->stop();
}

void OrbbecDevice::reconfigColorStream(int width, int height, int fps)
{
    auto profiles = mColorSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_MJPG && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mColorProfile = profile;
            mColorSensor->switchProfile(mColorProfile);
            ROS_INFO("Reconfig color stream: %dx%d(%d)", width, height, fps);
            break;
        }
    }
}

void OrbbecDevice::startDepthStream()
{
    auto profiles = mDepthSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16)
        {
            mDepthProfile = profile;
            break;
        }
    }

    if (mDepthProfile != NULL)
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
                                cinfo->header.frame_id = frame->index();

                                //    mDepthPub.publish(image, cinfo);

                                mDepthPub.publish(image);
                            });
    }
}

void OrbbecDevice::stopDepthStream()
{
    mDepthSensor->stop();
}

void OrbbecDevice::reconfigDepthStream(int width, int height, int fps)
{
    auto profiles = mDepthSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16 && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mDepthProfile = profile;
            mDepthSensor->switchProfile(mDepthProfile);
            ROS_INFO("Reconfig depth stream: %dx%d(%d)", width, height, fps);
            break;
        }
    }
}

void OrbbecDevice::startIrStream()
{
    auto profiles = mIrSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16)
        {
            mIrProfile = profile;
            break;
        }
    }

    if (mIrProfile != NULL)
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

                             sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                             cinfo->width = frame->width();
                             cinfo->height = frame->height();
                             cinfo->header.frame_id = frame->index();

                             // mIrPub.publish(image, cinfo);

                             mIrPub.publish(image);
                         });
    }
}

void OrbbecDevice::stopIrStream()
{
    mIrSensor->stop();
}

void OrbbecDevice::reconfigIrStream(int width, int height, int fps)
{
    auto profiles = mIrSensor->getStreamProfiles();
    for (int i = 0; i < profiles.size(); i++)
    {
        auto profile = profiles[i];
        if (profile->format() == OB_FORMAT_Y16 && profile->width() == width && profile->height() == height && profile->fps() == fps)
        {
            mIrProfile = profile;
            mIrSensor->switchProfile(mIrProfile);
            ROS_INFO("Reconfig ir stream: %dx%d(%d)", width, height, fps);
            break;
        }
    }
}
