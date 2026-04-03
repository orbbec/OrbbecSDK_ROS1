#include <ros/ros.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "libobsensor/ObSensor.hpp"

using namespace ob;

namespace {

struct CliArgs {
  enum class Operation {
    NONE,
    SET_IP,
    FORCE_IP,
  };

  bool help = false;
  Operation operation = Operation::NONE;
  bool dhcp = false;
  std::string force_ip_mac;
  std::string old_ip = "192.168.1.10";
  int port = 8090;
  std::string new_ip = "192.168.1.200";
  std::string mask = "255.255.255.0";
  std::string gateway = "192.168.1.1";
};

bool parseIpString(const std::string &ip_str, uint8_t ip[4]) {
  std::stringstream ss(ip_str);
  std::string item;
  int i = 0;
  while (std::getline(ss, item, '.')) {
    if (i >= 4) {
      return false;
    }
    try {
      int num = std::stoi(item);
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

bool parseBool(const std::string &value, bool &out) {
  if (value == "true" || value == "1") {
    out = true;
    return true;
  }
  if (value == "false" || value == "0") {
    out = false;
    return true;
  }
  return false;
}

bool parseInt(const std::string &value, int &out) {
  try {
    size_t consumed = 0;
    const int parsed = std::stoi(value, &consumed);
    if (consumed != value.size()) {
      return false;
    }
    out = parsed;
    return true;
  } catch (...) {
    return false;
  }
}

bool isRosRemapArg(const std::string &arg) {
  return arg.find(":=") != std::string::npos;
}

void printHelp() {
  std::cout
      << "Usage:\n"
      << "  rosrun orbbec_camera ip_config_tool <set_ip|force_ip> [options]\n"
      << "  rosrun orbbec_camera set_device_ip <set_ip|force_ip> [options]\n"
      << "  ros2 run orbbec_camera ip_config_tool -- <set_ip|force_ip> [options]\n"
      << "  (legacy executable alias: set_device_ip)\n\n"
      << "Subcommands:\n"
      << "  set_ip                     Configure IP on device by current device address.\n"
      << "  force_ip                   Force IP by MAC address.\n\n"
      << "Parameters:\n"
      << "  --enable_dhcp <bool>       DHCP flag for set-ip/force-ip (default: false).\n"
      << "  --old_ip <ip>              Current device IP for set-ip (default: 192.168.1.10).\n"
      << "  --port <port>              Device port for set-ip (default: 8090).\n"
      << "  --new_ip <ip>              Static IP for set-ip/force-ip (default: 192.168.1.200).\n"
      << "  --mask <ip>                Subnet mask for set-ip/force-ip (default: 255.255.255.0).\n"
      << "  --gateway <ip>             Gateway for set-ip/force-ip (default: 192.168.1.1).\n"
      << "  --force_ip_mac <mac>       Target MAC for force-ip (required, e.g. "
         "54:14:FD:06:07:DA).\n\n"
      << "Examples:\n"
      << "\n"
      << "  [Set IP]\n"
      << "    DHCP:    rosrun orbbec_camera ip_config_tool set_ip \\\n"
      << "             --old_ip 192.168.1.10 --enable_dhcp true\n"
      << "    Static:  rosrun orbbec_camera ip_config_tool set_ip \\\n"
      << "             --old_ip 192.168.1.10 --enable_dhcp false \\\n"
      << "             --new_ip 192.168.1.200 --mask 255.255.255.0 --gateway 192.168.1.1\n"
      << "\n"
      << "  [Force IP]\n"
      << "    by MAC:  rosrun orbbec_camera ip_config_tool force_ip \\\n"
      << "             --force_ip_mac 54:14:FD:06:07:DA --enable_dhcp false \\\n"
      << "             --new_ip 192.168.1.200 --mask 255.255.255.0 --gateway 192.168.1.1\n";
}

bool parseArgs(int argc, char **argv, CliArgs &args, std::string &error) {
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];

    if (current == "--") {
      continue;
    }

    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }

    if (isRosRemapArg(current)) {
      continue;
    }

    if (current == "set_ip" || current == "set-ip" || current == "set_device_ip") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed: set_ip or force_ip";
        return false;
      }
      args.operation = CliArgs::Operation::SET_IP;
      continue;
    }

    if (current == "force_ip" || current == "force-ip") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed: set_ip or force_ip";
        return false;
      }
      args.operation = CliArgs::Operation::FORCE_IP;
      continue;
    }

    if (current.rfind("--enable_dhcp=", 0) == 0) {
      if (!parseBool(current.substr(std::strlen("--enable_dhcp=")), args.dhcp)) {
        error = "--enable_dhcp expects true/false";
        return false;
      }
      continue;
    }
    if (current == "--enable_dhcp") {
      if (++i >= argc || !parseBool(argv[i], args.dhcp)) {
        error = "--enable_dhcp expects true/false";
        return false;
      }
      continue;
    }

    if (current.rfind("--old_ip=", 0) == 0) {
      args.old_ip = current.substr(std::strlen("--old_ip="));
      continue;
    }
    if (current == "--old_ip") {
      if (++i >= argc) {
        error = "--old_ip requires a value";
        return false;
      }
      args.old_ip = argv[i];
      continue;
    }

    if (current.rfind("--port=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--port=")), args.port)) {
        error = "--port expects an integer";
        return false;
      }
      continue;
    }
    if (current == "--port") {
      if (++i >= argc || !parseInt(argv[i], args.port)) {
        error = "--port expects an integer";
        return false;
      }
      continue;
    }

    if (current.rfind("--new_ip=", 0) == 0) {
      args.new_ip = current.substr(std::strlen("--new_ip="));
      continue;
    }
    if (current == "--new_ip") {
      if (++i >= argc) {
        error = "--new_ip requires a value";
        return false;
      }
      args.new_ip = argv[i];
      continue;
    }

    if (current.rfind("--mask=", 0) == 0) {
      args.mask = current.substr(std::strlen("--mask="));
      continue;
    }
    if (current == "--mask") {
      if (++i >= argc) {
        error = "--mask requires a value";
        return false;
      }
      args.mask = argv[i];
      continue;
    }

    if (current.rfind("--gateway=", 0) == 0) {
      args.gateway = current.substr(std::strlen("--gateway="));
      continue;
    }
    if (current == "--gateway") {
      if (++i >= argc) {
        error = "--gateway requires a value";
        return false;
      }
      args.gateway = argv[i];
      continue;
    }

    if (current.rfind("--force_ip_mac=", 0) == 0) {
      args.force_ip_mac = current.substr(std::strlen("--force_ip_mac="));
      continue;
    }
    if (current == "--force_ip_mac") {
      if (++i >= argc) {
        error = "--force_ip_mac requires a value";
        return false;
      }
      args.force_ip_mac = argv[i];
      continue;
    }

    error = "Unknown argument: " + current;
    return false;
  }

  if (args.help) {
    return true;
  }

  if (args.operation == CliArgs::Operation::NONE) {
    error = "Missing subcommand. Use one of: set_ip, force_ip";
    return false;
  }

  if (args.port <= 0 || args.port > 65535) {
    error = "--port must be in range 1-65535";
    return false;
  }

  if (args.operation == CliArgs::Operation::FORCE_IP && args.force_ip_mac.empty()) {
    error = "force_ip requires --force_ip_mac <mac>";
    return false;
  }

  return true;
}

}  // namespace

int main(int argc, char **argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << "Argument error: " << parse_error << std::endl;
    printHelp();
    return 1;
  }

  if (args.help) {
    printHelp();
    return 0;
  }

  ros::init(argc, argv, "ip_config_tool");

  try {
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto context = std::make_shared<ob::Context>();

    if (args.operation == CliArgs::Operation::SET_IP) {
      ROS_INFO("Connecting to device %s:%d ...", args.old_ip.c_str(), args.port);
      auto device = context->createNetDevice(args.old_ip.c_str(), static_cast<uint16_t>(args.port));

      bool v2_supported = false;
#if defined(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2) && defined(OB_NET_IP_FLAG_PERSISTENT) && \
    defined(OB_NET_IP_FLAG_DHCP)
      v2_supported =
          device->isPropertySupported(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2, OB_PERMISSION_READ_WRITE);
#endif

      if (v2_supported) {
#if defined(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2) && defined(OB_NET_IP_FLAG_PERSISTENT) && \
    defined(OB_NET_IP_FLAG_DHCP)
        uint8_t address[4] = {0};
        uint8_t mask[4] = {0};
        uint8_t gateway[4] = {0};
        if (!parseIpString(args.new_ip, address)) {
          ROS_ERROR("Invalid new_ip format: %s", args.new_ip.c_str());
          return 1;
        }
        if (!parseIpString(args.mask, mask)) {
          ROS_ERROR("Invalid mask format: %s", args.mask.c_str());
          return 1;
        }
        if (!parseIpString(args.gateway, gateway)) {
          ROS_ERROR("Invalid gateway format: %s", args.gateway.c_str());
          return 1;
        }

        OBNetIpConfigV2 ip_config_v2{};
        ip_config_v2.flags = OB_NET_IP_FLAG_PERSISTENT;
        if (args.dhcp) {
          ip_config_v2.flags = static_cast<uint16_t>(ip_config_v2.flags | OB_NET_IP_FLAG_DHCP);
        }

        std::memcpy(ip_config_v2.address, address, sizeof(address));
        std::memcpy(ip_config_v2.mask, mask, sizeof(mask));
        std::memcpy(ip_config_v2.gateway, gateway, sizeof(gateway));

        ROS_INFO("Applying set-ip configuration with V2 property (1088)...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                                  reinterpret_cast<const uint8_t *>(&ip_config_v2),
                                  sizeof(ip_config_v2));

        ROS_INFO("Set-ip configuration applied successfully (V2).");
        ROS_INFO("Set-ip target DHCP: %s", args.dhcp ? "enabled" : "disabled");
        ROS_INFO("Set-ip target persistent(static): enabled");
        ROS_INFO("Set-ip target static IP: %d.%d.%d.%d", ip_config_v2.address[0],
                 ip_config_v2.address[1], ip_config_v2.address[2], ip_config_v2.address[3]);
        ROS_INFO("Set-ip target mask: %d.%d.%d.%d", ip_config_v2.mask[0], ip_config_v2.mask[1],
                 ip_config_v2.mask[2], ip_config_v2.mask[3]);
        ROS_INFO("Set-ip target gateway: %d.%d.%d.%d", ip_config_v2.gateway[0],
                 ip_config_v2.gateway[1], ip_config_v2.gateway[2], ip_config_v2.gateway[3]);
#else
        ROS_WARN("IP config V2 symbols are unavailable in current SDK; fallback to legacy property.");
#endif
      } else {
        OBNetIpConfig ip_config{};
        ip_config.dhcp = args.dhcp ? 1 : 0;

        if (!args.dhcp) {
          uint8_t address[4] = {0};
          uint8_t mask[4] = {0};
          uint8_t gateway[4] = {0};
          if (!parseIpString(args.new_ip, address)) {
            ROS_ERROR("Invalid new_ip format: %s", args.new_ip.c_str());
            return 1;
          }
          if (!parseIpString(args.mask, mask)) {
            ROS_ERROR("Invalid mask format: %s", args.mask.c_str());
            return 1;
          }
          if (!parseIpString(args.gateway, gateway)) {
            ROS_ERROR("Invalid gateway format: %s", args.gateway.c_str());
            return 1;
          }

          std::memcpy(ip_config.address, address, sizeof(address));
          std::memcpy(ip_config.mask, mask, sizeof(mask));
          std::memcpy(ip_config.gateway, gateway, sizeof(gateway));
        }

        ROS_WARN("Device does not support IP config V2 (1088), fallback to legacy property (1041).");
        if (args.dhcp) {
          ROS_WARN(
              "Legacy IP config does not support DHCP and static IP simultaneously; static fields will "
              "be ignored.");
        }

        ROS_INFO("Applying set-ip configuration...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));

        ROS_INFO("Set-ip configuration applied successfully.");
        if (args.dhcp) {
          ROS_INFO("Set-ip target mode: DHCP.");
        } else {
          ROS_INFO("Set-ip target static IP: %d.%d.%d.%d", ip_config.address[0], ip_config.address[1],
                   ip_config.address[2], ip_config.address[3]);
          ROS_INFO("Set-ip target mask: %d.%d.%d.%d", ip_config.mask[0], ip_config.mask[1],
                   ip_config.mask[2], ip_config.mask[3]);
          ROS_INFO("Set-ip target gateway: %d.%d.%d.%d", ip_config.gateway[0], ip_config.gateway[1],
                   ip_config.gateway[2], ip_config.gateway[3]);
        }
      }
    }

    if (args.operation == CliArgs::Operation::FORCE_IP) {
      OBNetIpConfig ip_config{};
      ip_config.dhcp = args.dhcp ? 1 : 0;

      if (!args.dhcp) {
        if (!parseIpString(args.new_ip, ip_config.address)) {
          ROS_ERROR("Invalid new_ip format: %s", args.new_ip.c_str());
          return 1;
        }
        if (!parseIpString(args.mask, ip_config.mask)) {
          ROS_ERROR("Invalid mask format: %s", args.mask.c_str());
          return 1;
        }
        if (!parseIpString(args.gateway, ip_config.gateway)) {
          ROS_ERROR("Invalid gateway format: %s", args.gateway.c_str());
          return 1;
        }
      }

      ROS_INFO("Applying force-ip to MAC %s ...", args.force_ip_mac.c_str());
      if (context->forceIp(args.force_ip_mac.c_str(), ip_config)) {
        ROS_INFO("Force-ip operation applied successfully.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        if (args.dhcp) {
          ROS_INFO("Force-ip target mode: DHCP.");
        } else {
          ROS_INFO("Force-ip target static IP: %s", args.new_ip.c_str());
          ROS_INFO("Force-ip target mask: %s", args.mask.c_str());
          ROS_INFO("Force-ip target gateway: %s", args.gateway.c_str());
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
