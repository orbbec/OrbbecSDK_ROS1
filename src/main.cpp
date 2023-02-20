#include "ros/ros.h"
#include "orbbec_camera/ob_camera_node_driver.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "orbbec_camera");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  orbbec_camera::OBCameraNodeDriver ob_camera_node_factory(nh, nh_private);
  ros::spin();
  ros::shutdown();
  return 0;
}
