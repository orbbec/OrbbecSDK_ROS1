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
#include <ros/ros.h>
#include <orbbec_camera/types.h>
#include <orbbec_camera/utils.h>
#include <string>
#include <regex>
#include <thread>
std::string parseUsbPort(const std::string &line) {
  std::string port_id;
  std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*",
                        std::regex_constants::ECMAScript);
  std::smatch base_match;
  bool found = std::regex_match(line, base_match, self_regex);
  if (found) {
    port_id = base_match[1].str();
    if (base_match[2].str().empty())  // This is libuvc string. Remove counter is exists.
    {
      std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
      bool found_end = std::regex_match(port_id, base_match, end_regex);
      if (found_end) {
        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
      }
    }
  }
  return port_id;
}
int main() {
  try {
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<ob::Context>();
    auto list = context->queryDeviceList();
    for (size_t i = 0; i < list->deviceCount(); i++) {
      if (std::string(list->getConnectionType(i)) != "Ethernet") {
        std::string serial = list->serialNumber(i);
        std::string uid = list->uid(i);
        auto usb_port = parseUsbPort(uid);
        auto connection_type = list->getConnectionType(i);
        std::stringstream pid_hex;
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        ROS_INFO_STREAM("- Name: " << list->getName(i) << ", PID: 0x" << pid_hex.str()
                                   << ", SN/ID: " << serial << ", Connection: " << connection_type);
        ROS_INFO_STREAM("serial: " << serial);
        ROS_INFO_STREAM("port id : " << usb_port);
        ROS_INFO_STREAM("usb connect type: " << connection_type);
        std::cout << std::endl;
      } else {
        std::string serial = list->serialNumber(i);
        auto connection_type = list->getConnectionType(i);
        auto ip_address = list->getIpAddress(i);
        std::stringstream pid_hex;
        pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
        ROS_INFO_STREAM("- Name: " << list->getName(i) << ", PID: 0x" << pid_hex.str()
                                   << ", SN/ID: " << serial << ", Connection: " << connection_type);
        ROS_INFO_STREAM("serial: " << serial);
        ROS_INFO_STREAM("ip address: " << ip_address);
        ROS_INFO_STREAM("usb connect type: " << connection_type);
        ROS_INFO_STREAM("mac : " << list->getUid(i));
        ROS_INFO_STREAM("subnet mask : " << list->getSubnetMask(i));
        ROS_INFO_STREAM("gateway : " << list->getGateway(i));
        std::cout << std::endl;
      }
    }
  } catch (ob::Error &e) {
    ROS_ERROR_STREAM("list_device_node: " << e.getMessage());
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("list_device_node: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("list_device_node: "
                     << "unknown error");
  }
  return 0;
}
