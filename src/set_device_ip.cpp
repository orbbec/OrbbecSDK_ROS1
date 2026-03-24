#include <ros/ros.h>
#include <orbbec_camera/types.h>
#include <iostream>
#include <sstream>
#include <string>

using namespace ob;

void printUsage(const char *program) {
  std::cout << "Usage:\n"
            << "  " << program << " [OPTIONS] [_old_ip:=IP] [_port:=PORT] [_dhcp:=BOOL] "
            << "[_new_ip:=IP] [_mask:=IP] [_gateway:=IP] [_lla:=BOOL]\n\n"
            << "Parameters:\n"
            << "  -h, --help                Show this help message and exit\n"
            << "  _old_ip (string)          Device current IP (default: 192.168.1.10)\n"
            << "  _port (int)               Device port (default: 8090)\n"
            << "  _dhcp (bool)              Enable DHCP (default: false)\n"
            << "  _new_ip (string)          New static IP (default: 192.168.1.200)\n"
            << "  _mask (string)            Subnet mask (default: 255.255.255.0)\n"
            << "  _gateway (string)         Gateway (default: 192.168.1.1)\n"
            << "  _lla (bool)               Enable/disable LLA (optional)\n\n"
            << "Examples:\n"
            << "  rosrun orbbec_camera set_device_ip _old_ip:=192.168.1.10 _new_ip:=192.168.1.200\n"
            << "  rosrun orbbec_camera set_device_ip _old_ip:=192.168.1.10 _dhcp:=true\n"
            << "  rosrun orbbec_camera set_device_ip _old_ip:=192.168.1.10 _lla:=true\n";
}

bool parseIpString(const std::string &ip_str, uint8_t ip[4]) {
  std::stringstream ss(ip_str);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, '.')) {
    if (i >= 4) return false;
    try {
      int num = std::stoi(item);
      if (num < 0 || num > 255) return false;
      ip[i++] = static_cast<uint8_t>(num);
    } catch (...) {
      return false;
    }
  }
  return i == 4;
}

int main(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "-h" || arg == "--help") {
      printUsage(argv[0]);
      return 0;
    }
  }

  ros::init(argc, argv, "set_device_ip");
  ros::NodeHandle nh("~");

  std::string device_ip_str, new_ip_str, mask_str, gateway_str;
  int port;
  bool dhcp;
  bool lla = false;
  bool lla_set = false;

  nh.param<std::string>("old_ip", device_ip_str, "192.168.1.10");
  nh.param<int>("port", port, 8090);
  nh.param<bool>("dhcp", dhcp, false);
  nh.param<std::string>("new_ip", new_ip_str, "192.168.1.200");
  nh.param<std::string>("mask", mask_str, "255.255.255.0");
  nh.param<std::string>("gateway", gateway_str, "192.168.1.1");
  if (nh.getParam("lla", lla)) {
    lla_set = true;
  }

  ob_net_ip_config ip_config{};
  ip_config.dhcp = dhcp ? 1 : 0;

  if (!parseIpString(new_ip_str, ip_config.address)) {
    ROS_ERROR("Invalid new_ip format: %s", new_ip_str.c_str());
    return 1;
  }
  if (!parseIpString(mask_str, ip_config.mask)) {
    ROS_ERROR("Invalid mask format: %s", mask_str.c_str());
    return 1;
  }
  if (!parseIpString(gateway_str, ip_config.gateway)) {
    ROS_ERROR("Invalid gateway format: %s", gateway_str.c_str());
    return 1;
  }

  try {
    ROS_INFO("Connecting to device %s:%d ...", device_ip_str.c_str(), port);
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<ob::Context>();
    auto device = context->createNetDevice(device_ip_str.c_str(), port);

    ROS_INFO("Setting new IP configuration...");
    device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                              reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));
    if (lla_set) {
      ROS_INFO("Setting LLA to %s...", lla ? "true" : "false");
      device->setBoolProperty(OB_PROP_DEVICE_NETWORK_LLA_BOOL, lla);
      ROS_INFO("LLA set to %s.", lla ? "true" : "false");
    } else {
      ROS_INFO("LLA not specified, keeping current device LLA setting.");
    }

    ROS_INFO("IP configuration applied successfully.");
    if (dhcp) {
      ROS_INFO("DHCP mode enabled.");
    } else {
      ROS_INFO("Static IP set to %d.%d.%d.%d", ip_config.address[0], ip_config.address[1],
               ip_config.address[2], ip_config.address[3]);
      ROS_INFO("Mask: %d.%d.%d.%d", ip_config.mask[0], ip_config.mask[1], ip_config.mask[2],
               ip_config.mask[3]);
      ROS_INFO("Gateway: %d.%d.%d.%d", ip_config.gateway[0], ip_config.gateway[1],
               ip_config.gateway[2], ip_config.gateway[3]);
    }

  } catch (ob::Error &e) {
    ROS_ERROR_STREAM("set_device_ip: " << e.getMessage());
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("set_device_ip: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("set_device_ip: unknown error");
  }

  return 0;
}
