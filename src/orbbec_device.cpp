#include <orbbec_device.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <libobsensor/hpp/StreamProfile.hpp>
#include <libobsensor/hpp/Frame.hpp>

OrbbecDevice::OrbbecDevice(ros::NodeHandle nh, ros::NodeHandle pnh, std::shared_ptr<ob::Device> dev) : 
                nodeHandle(nh), privateNodeHandle(pnh), colorIt(nh), depthIt(nh), irIt(nh), device(dev)
{
    colorPub = colorIt.advertiseCamera("image/color", 1);
    depthPub = depthIt.advertiseCamera("image/depth", 1);
    irPub = irIt.advertiseCamera("image/ir", 1);

    colorSensor = dev->getSensor(OB_SENSOR_COLOR);
    depthSensor = dev->getSensor(OB_SENSOR_DEPTH);
    irSensor = dev->getSensor(OB_SENSOR_IR);

    startColorStream();
    startDepthStream();
    startIRStream();    
}

OrbbecDevice::~OrbbecDevice()
{
}

void OrbbecDevice::startColorStream()
{
    auto profiles = colorSensor->getStreamProfiles();
    std::shared_ptr<ob::StreamProfile> profile;
    for(int i = 0; i < profiles.size(); i++)
    {
        if(profiles[i]->format() == OB_FORMAT_YUY2)
        {
            profile = profiles[i];
        }
    }

    if(profile != NULL)
    {
        colorSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame){
            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->width = frame->width();
            image->height = frame->height();
            image->step = frame->width() * 4;
            image->encoding = sensor_msgs::image_encodings::YUV422;
            image->data.resize(frame->dataSize());
            memcpy(&image->data[0], frame->data(), frame->dataSize());

            sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(info));
            cinfo->width = frame->width();
            cinfo->height = frame->height();
            cinfo->header.frame_id = frame->index();

            colorPub.publish(image, cinfo);
        });
    }    
}

void OrbbecDevice::startDepthStream()
{
    auto profiles = depthSensor->getStreamProfiles();
    std::shared_ptr<ob::StreamProfile> profile;
    for(int i = 0; i < profiles.size(); i++)
    {
        if(profiles[i]->format() == OB_FORMAT_Y16)
        {
            profile = profiles[i];
        }
    }

    if(profile != NULL)
    {
        depthSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame){
            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->width = frame->width();
            image->height = frame->height();
            image->step = frame->width() * 2;
            image->encoding = sensor_msgs::image_encodings::MONO16;
            image->data.resize(frame->dataSize());
            memcpy(&image->data[0], frame->data(), frame->dataSize());

            sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(info));
            cinfo->width = frame->width();
            cinfo->height = frame->height();
            cinfo->header.frame_id = frame->index();

            depthPub.publish(image, cinfo);
        });
    }
}

void OrbbecDevice::startIRStream()
{
    auto profiles = irSensor->getStreamProfiles();
    std::shared_ptr<ob::StreamProfile> profile;
    for(int i = 0; i < profiles.size(); i++)
    {
        if(profiles[i]->format() == OB_FORMAT_Y16)
        {
            profile = profiles[i];
        }
    }

    if(profile != NULL)
    {
        irSensor->start(profile, [&](std::shared_ptr<ob::Frame> frame){
            sensor_msgs::Image::Ptr image(new sensor_msgs::Image());
            image->width = frame->width();
            image->height = frame->height();
            image->step = frame->width() * 2;
            image->encoding = sensor_msgs::image_encodings::MONO16;
            image->data.resize(frame->dataSize());
            memcpy(&image->data[0], frame->data(), frame->dataSize());

            sensor_msgs::CameraInfo::Ptr cinfo(new sensor_msgs::CameraInfo(info));
            cinfo->width = frame->width();
            cinfo->height = frame->height();
            cinfo->header.frame_id = frame->index();

            irPub.publish(image, cinfo);
        });
    }
}