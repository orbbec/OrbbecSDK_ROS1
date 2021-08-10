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
    startIRStream();
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
    std::shared_ptr<ob::StreamProfile> profile;
    for (int i = 0; i < profiles.size(); i++)
    {
        if (profiles[i]->format() == OB_FORMAT_MJPG)
        {
            profile = profiles[i];
            break;
        }
    }

    if (profile != NULL)
    {
        mColorSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame)
                            {
                                void* argbBuffer = getArgbBuffer(frame->width() * frame->height() * 4);
                                libyuv::MJPGToARGB((uint8_t*)frame->data(), frame->dataSize(), (uint8_t*)argbBuffer, frame->width() * 4, frame->width(), frame->height(), frame->width(), frame->height());

                                void* rgbBuffer = getRgbBuffer(frame->width() * frame->height() * 3);
                                libyuv::ARGBToRGB24((uint8_t*)argbBuffer, frame->width() * 4, (uint8_t*)rgbBuffer, frame->width() * 3, frame->width(), frame->height());

                                sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
                                image->width = frame->width();
                                image->height = frame->height();
                                image->step = frame->width() * 3;
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

void OrbbecDevice::startDepthStream()
{
    auto profiles = mDepthSensor->getStreamProfiles();
    std::shared_ptr<ob::StreamProfile> profile;
    for (int i = 0; i < profiles.size(); i++)
    {
        if (profiles[i]->format() == OB_FORMAT_Y16)
        {
            profile = profiles[i];
            break;
        }
    }

    if (profile != NULL)
    {
        mDepthSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame)
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

void OrbbecDevice::startIRStream()
{
    auto profiles = mIrSensor->getStreamProfiles();
    std::shared_ptr<ob::StreamProfile> profile;
    for (int i = 0; i < profiles.size(); i++)
    {
        if (profiles[i]->format() == OB_FORMAT_Y16)
        {
            profile = profiles[i];
            break;
        }
    }

    if (profile != NULL)
    {
        mIrSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame)
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