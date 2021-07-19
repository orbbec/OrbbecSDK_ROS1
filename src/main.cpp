#include <ros/ros.h>
#include <libobsensor/ObSensor.hpp>

int main(int argc, char** argv) {
    ros::init(argc, argv, "hello_ros");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("Hello, ROS");
}