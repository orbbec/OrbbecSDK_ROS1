#pragma once
#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>

#include "types.h"
#include "utils.h"

namespace orbbec_camera {
class D2CViewer {
 public:
  D2CViewer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~D2CViewer();
  void messageCallback(const sensor_msgs::ImageConstPtr& rgb_msg,
                       const sensor_msgs::ImageConstPtr& depth_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  using MySyncPolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image>;
  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;
  ros::Publisher d2c_viewer_pub_;
};
}  // namespace orbbec_camera
