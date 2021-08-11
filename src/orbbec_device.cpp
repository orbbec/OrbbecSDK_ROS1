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
        ROS_INFO("Start color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(), mColorProfile->fps());
    }
    else
    {
        ROS_WARN("Start color stream failed: no profile found");
    }
}

void OrbbecDevice::stopColorStream()
{
    mColorSensor->stop();
    ROS_INFO("Stop color stream");
}

void OrbbecDevice::reconfigColorStream(int width, int height, int fps)
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
    if(found)
    {
        mColorSensor->switchProfile(mColorProfile);
        ROS_INFO("Reconfig color stream: %dx%d(%d)", mColorProfile->width(), mColorProfile->height(), mColorProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig color stream failed: no profile found");
    }
}

void OrbbecDevice::startDepthStream()
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

                                sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                                cinfo->width = frame->width();
                                cinfo->height = frame->height();
                                cinfo->header.frame_id = frame->index();

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

void OrbbecDevice::stopDepthStream()
{
    mDepthSensor->stop();
    ROS_INFO("Stop depth stream");
}

void OrbbecDevice::reconfigDepthStream(int width, int height, int fps)
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
    if(found)
    {
        mDepthSensor->switchProfile(mDepthProfile);
        ROS_INFO("Reconfig depth stream: %dx%d(%d)", mDepthProfile->width(), mDepthProfile->height(), mDepthProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig depth stream failed: no profile found");
    }
}

void OrbbecDevice::startIrStream()
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

                             sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(mInfo));
                             cinfo->width = frame->width();
                             cinfo->height = frame->height();
                             cinfo->header.frame_id = frame->index();

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

void OrbbecDevice::stopIrStream()
{
    mIrSensor->stop();
    ROS_INFO("Stop ir stream");
}

void OrbbecDevice::reconfigIrStream(int width, int height, int fps)
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
    if(found)
    {
        mIrSensor->switchProfile(mIrProfile);
        ROS_INFO("Reconfig ir stream: %dx%d(%d)", mIrProfile->width(), mIrProfile->height(), mIrProfile->fps());
    }
    else
    {
        ROS_WARN("Reconfig ir stream failed: no profile found");
    }
}
