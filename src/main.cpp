#include "orbbec_device_manager.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ob_camera");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    OrbbecDeviceManager manager(nh, pnh);

    ros::spin();

    return 0;
}
