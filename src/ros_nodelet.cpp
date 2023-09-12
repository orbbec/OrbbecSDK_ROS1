/*******************************************************************************
 * Copyright (c) 2023 Orbbec 3D Technology, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *******************************************************************************/
#include "ros/ros.h"
#include "orbbec_camera/ob_camera_node_driver.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace orbbec_camera {
class OBCameraNodelet : public nodelet::Nodelet {
 public:
  OBCameraNodelet() {}

  ~OBCameraNodelet() {}

 private:
  void onInit() override {
    ros::NodeHandle nh = getNodeHandle();
    ros::NodeHandle nh_private = getPrivateNodeHandle();
    ob_camera_node_driver_.reset(new OBCameraNodeDriver(nh, nh_private));
  }

  boost::shared_ptr<OBCameraNodeDriver> ob_camera_node_driver_;
};
}  // namespace orbbec_camera

PLUGINLIB_EXPORT_CLASS(orbbec_camera::OBCameraNodelet, nodelet::Nodelet)