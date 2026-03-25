#include <ros/ros.h>

#include <chrono>
#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "libobsensor/ObSensor.hpp"

using namespace ob;

namespace {

bool parseIpString(const std::string &ipStr, uint8_t ip[4]) {
  std::stringstream ss(ipStr);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, '.')) {
    if (i >= 4) {
      return false;
    }
    try {
      const int num = std::stoi(item);
      if (num < 0 || num > 255) {
        return false;
      }
      ip[i++] = static_cast<uint8_t>(num);
    } catch (...) {
      return false;
    }
  }
  return i == 4;
}

bool isParamProvided(int argc, char **argv, const std::string &key) {
  const std::string pattern = key + ":=";
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg.find(pattern) != std::string::npos) {
      return true;
    }
  }
  return false;
}

void printHelp() {
  std::cout
      << "Usage:\n"
      << "  rosrun orbbec_camera ip_config_tool [params]\n"
      << "  (legacy alias: set_device_ip)\n\n"
      << "Parameters:\n"
      << "  _old_ip:=<ip>            Current device IP (default: 192.168.1.10)\n"
      << "  _port:=<port>            Device port (default: 8090)\n"
      << "  _enable_lla:=<bool>      Set LLA switch directly (true: enable, false: disable, "
         "default: false)\n"
      << "                            Note: LLA is applied only when this parameter is explicitly "
         "provided.\n"
      << "  _enable_set_ip:=<bool>   Enable set-ip operation (default: false)\n"
      << "  _dhcp:=<bool>            DHCP flag for set-ip/force-ip config (default: false)\n"
      << "  _new_ip:=<ip>            Static IP for set-ip/force-ip (default: 192.168.1.200)\n"
      << "  _mask:=<ip>              Subnet mask for set-ip/force-ip (default: 255.255.255.0)\n"
      << "  _gateway:=<ip>           Gateway for set-ip/force-ip (default: 192.168.1.1)\n"
      << "  _enable_force_ip:=<bool> Enable force-ip operation (default: false)\n"
      << "  _force_ip_mac:=<mac>     Target MAC for force-ip (required, e.g. "
         "54:14:FD:06:07:DA)\n\n"
      << "Examples:\n"
      << "\n"
      << "  [LLA]\n"
      << "    enable:  rosrun orbbec_camera ip_config_tool _old_ip:=192.168.1.10 "
         "_enable_lla:=true\n"
      << "    disable: rosrun orbbec_camera ip_config_tool _old_ip:=192.168.1.10 "
         "_enable_lla:=false\n"
      << "\n"
      << "  [Set IP]\n"
      << "    DHCP:    rosrun orbbec_camera ip_config_tool \\\n"
      << "             _old_ip:=192.168.1.10 _enable_set_ip:=true _dhcp:=true\n"
      << "    Static:  rosrun orbbec_camera ip_config_tool \\\n"
      << "             _old_ip:=192.168.1.10 _enable_set_ip:=true _dhcp:=false \\\n"
      << "             _new_ip:=192.168.1.200 _mask:=255.255.255.0 _gateway:=192.168.1.1\n"
      << "\n"
      << "  [Force IP]\n"
      << "    by MAC:  rosrun orbbec_camera ip_config_tool \\\n"
      << "             _enable_force_ip:=true \\\n"
      << "             _force_ip_mac:=54:14:FD:06:07:DA _dhcp:=false \\\n"
      << "             _new_ip:=192.168.1.200 _mask:=255.255.255.0 _gateway:=192.168.1.1\n";
}

}  // namespace

int main(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      printHelp();
      return 0;
    }
  }

  ros::init(argc, argv, "ip_config_tool");
  ros::NodeHandle nh("~");

  std::string deviceIpStr;
  int port = 8090;
  bool enableLla = false;
  bool enableSetIp = false;
  bool dhcp = false;
  std::string newIpStr;
  std::string maskStr;
  std::string gatewayStr;
  bool enableForceIp = false;
  std::string forceIpMac;

  nh.param<std::string>("old_ip", deviceIpStr, "192.168.1.10");
  nh.param<int>("port", port, 8090);

  bool doLla = false;
  if (nh.getParam("enable_lla", enableLla)) {
    doLla = true;
  } else if (nh.getParam("lla", enableLla)) {
    doLla = true;
    ROS_WARN("Parameter '~lla' is deprecated. Please use '~enable_lla'.");
  } else if (isParamProvided(argc, argv, "enable_lla")) {
    doLla = true;
  }

  nh.param<bool>("enable_set_ip", enableSetIp, false);
  nh.param<bool>("dhcp", dhcp, false);
  nh.param<std::string>("new_ip", newIpStr, "192.168.1.200");
  nh.param<std::string>("mask", maskStr, "255.255.255.0");
  nh.param<std::string>("gateway", gatewayStr, "192.168.1.1");
  nh.param<bool>("enable_force_ip", enableForceIp, false);
  nh.param<std::string>("force_ip_mac", forceIpMac, std::string(""));

  if (!doLla && !enableSetIp && !enableForceIp) {
    ROS_ERROR("No operation enabled. Please enable at least one of: enable_lla, enable_set_ip, "
              "enable_force_ip.");
    return 1;
  }

  OBNetIpConfig ipConfig{};
  ipConfig.dhcp = dhcp ? 1 : 0;

  if ((enableSetIp || enableForceIp) && !dhcp) {
    if (!parseIpString(newIpStr, ipConfig.address)) {
      ROS_ERROR("Invalid new_ip format: %s", newIpStr.c_str());
      return 1;
    }
    if (!parseIpString(maskStr, ipConfig.mask)) {
      ROS_ERROR("Invalid mask format: %s", maskStr.c_str());
      return 1;
    }
    if (!parseIpString(gatewayStr, ipConfig.gateway)) {
      ROS_ERROR("Invalid gateway format: %s", gatewayStr.c_str());
      return 1;
    }
  }

  try {
    Context::setLoggerSeverity(OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<Context>();

    if (doLla || enableSetIp) {
      if (port <= 0 || port > 65535) {
        ROS_ERROR("Invalid port: %d (valid range: 1-65535)", port);
        return 1;
      }

      ROS_INFO("Connecting to device %s:%d ...", deviceIpStr.c_str(), port);
      auto device = context->createNetDevice(deviceIpStr.c_str(), static_cast<uint16_t>(port));

      if (doLla) {
        if (device->isPropertySupported(OB_PROP_DEVICE_NETWORK_LLA_BOOL, OB_PERMISSION_READ_WRITE)) {
          device->setBoolProperty(OB_PROP_DEVICE_NETWORK_LLA_BOOL, enableLla);
          ROS_INFO("LLA set successfully. target=%s", enableLla ? "enabled" : "disabled");
        } else {
          ROS_WARN("LLA property is not supported on this device.");
        }
      } else {
        ROS_INFO("LLA operation skipped (enable_lla not explicitly provided).");
      }

      if (enableSetIp) {
        ROS_INFO("Applying set-ip configuration...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<const uint8_t *>(&ipConfig), sizeof(ipConfig));

        ROS_INFO("Set-ip configuration applied successfully.");
        if (dhcp) {
          ROS_INFO("Set-ip target mode: DHCP.");
        } else {
          ROS_INFO("Set-ip target static IP: %d.%d.%d.%d", ipConfig.address[0], ipConfig.address[1],
                   ipConfig.address[2], ipConfig.address[3]);
          ROS_INFO("Set-ip target mask: %d.%d.%d.%d", ipConfig.mask[0], ipConfig.mask[1],
                   ipConfig.mask[2], ipConfig.mask[3]);
          ROS_INFO("Set-ip target gateway: %d.%d.%d.%d", ipConfig.gateway[0], ipConfig.gateway[1],
                   ipConfig.gateway[2], ipConfig.gateway[3]);
        }
      } else {
        ROS_INFO("Set-ip operation skipped (enable_set_ip=false).");
      }
    }

    if (enableForceIp) {
      if (forceIpMac.empty()) {
        ROS_ERROR("force_ip_mac is required when enable_force_ip=true.");
        return 1;
      }

      ROS_INFO("Applying force-ip to MAC %s ...", forceIpMac.c_str());
      if (context->forceIp(forceIpMac.c_str(), ipConfig)) {
        ROS_INFO("Force-ip operation applied successfully.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (dhcp) {
          ROS_INFO("Force-ip target mode: DHCP.");
        } else {
          ROS_INFO("Force-ip target static IP: %s", newIpStr.c_str());
          ROS_INFO("Force-ip target mask: %s", maskStr.c_str());
          ROS_INFO("Force-ip target gateway: %s", gatewayStr.c_str());
        }
      } else {
        ROS_ERROR("Force-ip failed (SDK returned false).");
        return 1;
      }
    }
  } catch (const ob::Error &e) {
    ROS_ERROR_STREAM("ip_config_tool: " << e.getMessage());
    return 1;
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("ip_config_tool: " << e.what());
    return 1;
  } catch (...) {
    ROS_ERROR_STREAM("ip_config_tool: unknown error");
    return 1;
  }

  return 0;
}
