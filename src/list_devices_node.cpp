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
#include <chrono>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <regex>
#include <thread>

namespace {

constexpr int kFirmwareLogDrainDelaySec = 3;

struct CliArgs {
  bool help = false;
  std::string sdk_log_level = "off";
};

void printUsage() {
  std::cout << "Usage:\n"
            << "  rosrun orbbec_camera list_devices_node [options]\n\n"
            << "Options:\n"
            << "  --sdk_log_level LEVEL  SDK file log level: debug/info/warn/error/fatal/off "
               "(default: off).\n\n"
            << "Examples:\n"
            << "  rosrun orbbec_camera list_devices_node --sdk_log_level debug\n";
}

bool parseArgs(int argc, char **argv, CliArgs &args, std::string &error) {
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }
    if (current.find(":=") != std::string::npos || current.rfind("__", 0) == 0 ||
        current.rfind("_", 0) == 0) {
      continue;
    }
    if (current.rfind("--sdk_log_level=", 0) == 0) {
      args.sdk_log_level = current.substr(std::strlen("--sdk_log_level="));
      continue;
    }
    if (current == "--sdk_log_level") {
      if (++i >= argc) {
        error = "--sdk_log_level requires a value";
        return false;
      }
      args.sdk_log_level = argv[i];
      continue;
    }
    error = "Unknown argument: " + current;
    return false;
  }

  const auto log_severity = orbbec_camera::obLogSeverityFromString(args.sdk_log_level);
  if (log_severity == OBLogSeverity::OB_LOG_SEVERITY_OFF && args.sdk_log_level != "off" &&
      args.sdk_log_level != "none") {
    error = "--sdk_log_level expects one of: debug, info, warn, error, fatal, off";
    return false;
  }

  return true;
}
}  // namespace

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

std::string ipSourceTypeToString(int type) {
  switch (type) {
    case OB_IP_SOURCE_NONE:
      return "NONE";
    case OB_IP_SOURCE_LLA:
      return "LLA";
    case OB_IP_SOURCE_DHCP:
      return "DHCP";
    case OB_IP_SOURCE_PERSISTENT:
      return "PERSISTENT";
    default:
      return "UNKNOWN(" + std::to_string(type) + ")";
  }
}

void printPresetInfo(const std::shared_ptr<ob::Device> &device) {
  try {
    auto preset_list = device->getAvailablePresetList();
    ROS_INFO_STREAM("Preset count: " << preset_list->getCount());
    for (uint32_t i = 0; i < preset_list->getCount(); ++i) {
      ROS_INFO_STREAM("  - " << preset_list->getName(i));
    }

    std::string key = "PresetVer";
    if (device->isExtensionInfoExist(key)) {
      std::string value = device->getExtensionInfo(key);
      ROS_INFO_STREAM("Preset version: " << value);
    } else {
      ROS_INFO_STREAM("Preset version: not available");
    }
  } catch (ob::Error &e) {
    ROS_WARN_STREAM("Failed to get preset info: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception &e) {
    ROS_WARN_STREAM("Failed to get preset info: " << e.what());
  } catch (...) {
    ROS_WARN_STREAM("Failed to get preset info");
  }
}

void waitForFirmwareLogDrain() {
  ROS_INFO("Waiting %d seconds to keep firmware log alive...", kFirmwareLogDrainDelaySec);
  std::this_thread::sleep_for(std::chrono::seconds(kFirmwareLogDrainDelaySec));
}

bool enableFirmwareLog(const std::shared_ptr<ob::Device> &device) {
  try {
    device->enableFirmwareLog(true);
    ROS_INFO("Firmware log enabled.");
    return true;
  } catch (const ob::Error &e) {
    ROS_WARN("Failed to enable firmware log: %s",
             orbbec_camera::formatObErrorWithStatus(e).c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Failed to enable firmware log: %s", e.what());
  }
  return false;
}

bool isSdkLogEnabled(const std::string &log_level) {
  return orbbec_camera::obLogSeverityFromString(log_level) != OBLogSeverity::OB_LOG_SEVERITY_OFF;
}

int main(int argc, char **argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << "Argument error: " << parse_error << std::endl;
    printUsage();
    return 1;
  }
  if (args.help) {
    printUsage();
    return 0;
  }

  try {
    const auto sdk_log_path =
        orbbec_camera::configureObSdkLoggerForTool("list_devices_node", args.sdk_log_level);
    if (!sdk_log_path.empty()) {
      ROS_INFO("SDK file log enabled: %s", sdk_log_path.c_str());
    }
    auto context = std::make_shared<ob::Context>();
    auto list = context->queryDeviceList();
    bool firmware_log_enabled = false;
    for (size_t i = 0; i < list->deviceCount(); i++) {
      try {
        auto device_ = list->getDevice(i);
        if (isSdkLogEnabled(args.sdk_log_level)) {
          firmware_log_enabled = enableFirmwareLog(device_) || firmware_log_enabled;
        }
        auto device_info_ = device_->getDeviceInfo();
        if (std::string(list->getConnectionType(i)) != "Ethernet") {
          std::string serial = list->serialNumber(i);
          std::string uid = list->uid(i);
          auto usb_port = parseUsbPort(uid);
          auto connection_type = list->getConnectionType(i);
          auto firmware_version = device_info_->getFirmwareVersion();
          std::stringstream pid_hex;
          pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
          ROS_INFO_STREAM("name: " << list->getName(i));
          ROS_INFO_STREAM("pid: 0x" << pid_hex.str());
          ROS_INFO_STREAM("serial: " << serial);
          ROS_INFO_STREAM("connection: " << connection_type);
          ROS_INFO_STREAM("firmware version: " << firmware_version);
          ROS_INFO_STREAM("usb port: " << usb_port);
          printPresetInfo(device_);
          std::cout << std::endl;
        } else {
          std::string serial = list->serialNumber(i);
          auto connection_type = list->getConnectionType(i);
          auto ip_address = list->getIpAddress(i);
          std::stringstream pid_hex;
          auto firmware_version = device_info_->getFirmwareVersion();
          pid_hex << std::hex << std::setw(4) << std::setfill('0') << list->getPid(i);
          ROS_INFO_STREAM("name: " << list->getName(i));
          ROS_INFO_STREAM("pid: 0x" << pid_hex.str());
          ROS_INFO_STREAM("serial: " << serial);
          ROS_INFO_STREAM("connection: " << connection_type);
          ROS_INFO_STREAM("firmware version: " << firmware_version);
          ROS_INFO_STREAM("ip address: " << ip_address);
          ROS_INFO_STREAM("MAC address: " << list->getUid(i));
          ROS_INFO_STREAM("subnet mask: " << list->getSubnetMask(i));
          ROS_INFO_STREAM("gateway: " << list->getGateway(i));
          ROS_INFO_STREAM(
              "local net interface: " << list->getLocalNetInterfaceName(static_cast<uint32_t>(i)));
          ROS_INFO_STREAM("ip source type: " << ipSourceTypeToString(
                              static_cast<int>(list->getIpSourceType(static_cast<uint32_t>(i)))));
          printPresetInfo(device_);
          std::cout << std::endl;
        }
      } catch (ob::Error &e) {
        ROS_ERROR_STREAM("Failed to list device at index "
                         << i << ": " << orbbec_camera::formatObErrorWithStatus(e));
      } catch (const std::exception &e) {
        ROS_ERROR_STREAM("Failed to list device at index " << i << ": " << e.what());
      } catch (...) {
        ROS_ERROR_STREAM("Failed to list device at index " << i << ": unknown error");
      }
    }
    if (firmware_log_enabled) {
      waitForFirmwareLogDrain();
    }
  } catch (ob::Error &e) {
    ROS_ERROR_STREAM("list_device_node: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("list_device_node: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("list_device_node: "
                     << "unknown error");
  }
  return 0;
}
