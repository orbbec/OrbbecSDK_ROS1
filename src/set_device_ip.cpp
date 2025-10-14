#include <ros/ros.h>
#include <orbbec_camera/types.h>
#include <sstream>
#include <string>

using namespace ob;

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
  ros::init(argc, argv, "set_device_ip");
  ros::NodeHandle nh("~");

  std::string device_ip_str, new_ip_str, mask_str, gateway_str;
  int port;
  bool dhcp;

  nh.param<std::string>("old_ip", device_ip_str, "192.168.1.10");
  nh.param<int>("port", port, 8090);
  nh.param<bool>("dhcp", dhcp, false);
  nh.param<std::string>("new_ip", new_ip_str, "192.168.1.200");
  nh.param<std::string>("mask", mask_str, "255.255.255.0");
  nh.param<std::string>("gateway", gateway_str, "192.168.1.1");

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
