#include "libobsensor/ObSensor.hpp"
#include "orbbec_device_manager.h"
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
    // ROS节点初始化
    ros::init(argc, argv, "ob_ros");
    // 创建节点句柄
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    OrbbecDeviceManager manager(nh, pnh);

    ros::spin();

    return 0;
}