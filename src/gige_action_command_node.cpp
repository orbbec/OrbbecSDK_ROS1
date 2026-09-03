/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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

#include <exception>
#include <memory>
#include <string>

#include <ros/ros.h>

#include "libobsensor/hpp/Context.hpp"
#include "orbbec_camera/SendActionCommand.h"
#include "orbbec_camera/utils.h"

namespace orbbec_camera {

class GigEActionCommandNode {
 public:
  explicit GigEActionCommandNode(ros::NodeHandle& nh_private)
      : context_(std::make_unique<ob::Context>()) {
    context_->enableNetDeviceEnumeration(true);
    send_action_command_service_ = nh_private.advertiseService(
        "send_action_command", &GigEActionCommandNode::sendActionCommandCallback, this);
    ROS_INFO_STREAM("GigE Action Command service is ready");
  }

 private:
  bool sendActionCommandCallback(SendActionCommandRequest& request,
                                 SendActionCommandResponse& response) {
    const std::string destination_ip =
        request.destination_ip.empty() ? "255.255.255.255" : request.destination_ip;
    try {
      response.success =
          context_->sendActionCommand(request.device_key, request.group_key, request.group_mask,
                                      destination_ip.c_str(), request.scheduled_time);
      response.message =
          response.success ? "Action Command dispatched" : "SDK failed to send Action Command";
    } catch (const ob::Error& error) {
      response.success = false;
      response.message = orbbec_camera::formatObErrorWithStatus(error);
    } catch (const std::exception& error) {
      response.success = false;
      response.message = error.what();
    } catch (...) {
      response.success = false;
      response.message = "Unknown error";
    }
    return true;
  }

  std::unique_ptr<ob::Context> context_;
  ros::ServiceServer send_action_command_service_;
};

}  // namespace orbbec_camera

int main(int argc, char** argv) {
  ros::init(argc, argv, "gige_action_command_node");
  ros::NodeHandle nh_private("~");
  orbbec_camera::GigEActionCommandNode node(nh_private);
  ros::spin();
  return 0;
}
