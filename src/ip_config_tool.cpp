#include <ros/ros.h>

#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "libobsensor/ObSensor.hpp"
#include "orbbec_camera/utils.h"

using namespace ob;

namespace {

struct CliArgs {
  enum class Operation {
    NONE,
    DHCP,
    SET_IP,
    FORCE_IP,
    SET_DHCP_TIMEOUT,
  };

  bool help = false;
  Operation operation = Operation::NONE;
  bool dhcp = false;
  bool dhcp_option_set = false;
  int dhcp_assign_ip_timeout = -1;
  bool dhcp_assign_ip_timeout_set = false;
  std::string force_ip_mac;
  std::string current_ip = "192.168.1.10";
  int port = 8090;
  std::string new_ip = "192.168.1.200";
  std::string mask = "255.255.255.0";
  std::string gateway = "192.168.1.1";
  std::string sdk_log_level = "off";
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

bool isRosRemapArg(const std::string &arg) { return arg.find(":=") != std::string::npos; }

void printHelp() {
  std::cout
      << "Usage:\n"
      << "  rosrun orbbec_camera ip_config_tool <dhcp|set_ip|force_ip|set_dhcp_timeout> "
         "[options]\n"
      << "      [--sdk_log_level debug]\n"
      << "Subcommands:\n"
      << "  dhcp                       Configure DHCP on device by current device address.\n"
      << "  set_ip                     Configure static IP on device by current device address.\n"
      << "  force_ip                   Force IP by MAC address.\n"
      << "  set_dhcp_timeout           Configure DHCP address assignment timeout.\n\n"
      << "Parameters:\n"
      << "  --enable_dhcp <bool>       DHCP flag for dhcp/force-ip (default: false).\n"
      << "  --current_ip <ip>          Current device IP for dhcp/set-ip/set_dhcp_timeout "
         "(default: "
         "192.168.1.10).\n"
      << "  --port <port>              Device port for dhcp/set-ip/set_dhcp_timeout "
         "(default: 8090).\n"
      << "  --new_ip <ip>              Static IP for set-ip/force-ip (default: 192.168.1.200).\n"
      << "  --mask <ip>                Subnet mask for set-ip/force-ip (default: 255.255.255.0).\n"
      << "  --gateway <ip>             Gateway for set-ip/force-ip (default: 192.168.1.1).\n"
      << "  --force_ip_mac <mac>       Target MAC for force-ip (required, e.g. "
         "54:14:FD:06:07:DA).\n"
      << "  --timeout <sec>            DHCP timeout in seconds for set_dhcp_timeout.\n"
      << "  --dhcp_assign_ip_timeout <sec>\n"
      << "                             Alias of --timeout.\n\n"
      << "  --sdk_log_level LEVEL      SDK file log level: debug/info/warn/error/fatal/off "
         "(default: off).\n\n"
      << "Examples:\n"
      << "\n"
      << "  [DHCP]\n"
      << "    Enable: rosrun orbbec_camera ip_config_tool dhcp \\\n"
      << "             --current_ip 192.168.1.10 --enable_dhcp true\n"
      << "  [Set IP]\n"
      << "    Static:  rosrun orbbec_camera ip_config_tool set_ip \\\n"
      << "             --current_ip 192.168.1.10 \\\n"
      << "             --new_ip 192.168.1.200 --mask 255.255.255.0 --gateway 192.168.1.1\n"
      << "\n"
      << "  [Force IP]\n"
      << "    by MAC:  rosrun orbbec_camera ip_config_tool force_ip \\\n"
      << "             --force_ip_mac 54:14:FD:06:07:DA \\\n"
      << "             --new_ip 192.168.1.200 --mask 255.255.255.0 --gateway 192.168.1.1\n";
  std::cout << "\n"
            << "  [Set DHCP Timeout]\n"
            << "    Timeout: rosrun orbbec_camera ip_config_tool set_dhcp_timeout \\\n"
            << "             --current_ip 192.168.1.10 --timeout 10\n"
            << "  [SDK Log]\n"
            << "    Debug:   rosrun orbbec_camera ip_config_tool dhcp \\\n"
            << "             --current_ip 192.168.1.10 --enable_dhcp true \\\n"
            << "             --sdk_log_level debug\n";
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

    if (current == "dhcp") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed";
        return false;
      }
      args.operation = CliArgs::Operation::DHCP;
      continue;
    }

    if (current == "set_ip" || current == "set-ip" || current == "set_device_ip") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed";
        return false;
      }
      args.operation = CliArgs::Operation::SET_IP;
      continue;
    }

    if (current == "force_ip" || current == "force-ip") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed";
        return false;
      }
      args.operation = CliArgs::Operation::FORCE_IP;
      continue;
    }

    if (current == "set_dhcp_timeout" || current == "set-dhcp-timeout") {
      if (args.operation != CliArgs::Operation::NONE) {
        error = "Only one subcommand is allowed";
        return false;
      }
      args.operation = CliArgs::Operation::SET_DHCP_TIMEOUT;
      continue;
    }

    if (current.rfind("--enable_dhcp=", 0) == 0) {
      if (!parseBool(current.substr(std::strlen("--enable_dhcp=")), args.dhcp)) {
        error = "--enable_dhcp expects true/false";
        return false;
      }
      args.dhcp_option_set = true;
      continue;
    }
    if (current == "--enable_dhcp") {
      if (++i >= argc || !parseBool(argv[i], args.dhcp)) {
        error = "--enable_dhcp expects true/false";
        return false;
      }
      args.dhcp_option_set = true;
      continue;
    }

    if (current.rfind("--current_ip=", 0) == 0) {
      args.current_ip = current.substr(std::strlen("--current_ip="));
      continue;
    }
    if (current == "--current_ip") {
      if (++i >= argc) {
        error = "--current_ip requires a value";
        return false;
      }
      args.current_ip = argv[i];
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

    if (current.rfind("--timeout=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--timeout=")), args.dhcp_assign_ip_timeout)) {
        error = "--timeout expects an integer";
        return false;
      }
      args.dhcp_assign_ip_timeout_set = true;
      continue;
    }
    if (current == "--timeout") {
      if (++i >= argc || !parseInt(argv[i], args.dhcp_assign_ip_timeout)) {
        error = "--timeout expects an integer";
        return false;
      }
      args.dhcp_assign_ip_timeout_set = true;
      continue;
    }

    if (current.rfind("--dhcp_assign_ip_timeout=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--dhcp_assign_ip_timeout=")),
                    args.dhcp_assign_ip_timeout)) {
        error = "--dhcp_assign_ip_timeout expects an integer";
        return false;
      }
      args.dhcp_assign_ip_timeout_set = true;
      continue;
    }
    if (current == "--dhcp_assign_ip_timeout") {
      if (++i >= argc || !parseInt(argv[i], args.dhcp_assign_ip_timeout)) {
        error = "--dhcp_assign_ip_timeout expects an integer";
        return false;
      }
      args.dhcp_assign_ip_timeout_set = true;
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

  if (args.help) {
    return true;
  }

  if (args.operation == CliArgs::Operation::NONE) {
    error = "Missing subcommand. Use one of: dhcp, set_ip, force_ip, set_dhcp_timeout";
    return false;
  }

  if (args.operation == CliArgs::Operation::DHCP && !args.dhcp_option_set) {
    error = "dhcp requires --enable_dhcp <true|false>";
    return false;
  }

  if (args.operation == CliArgs::Operation::SET_IP && args.dhcp_option_set) {
    error = "set_ip only supports static IP now. Use subcommand 'dhcp' for --enable_dhcp";
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

  if (args.operation == CliArgs::Operation::SET_DHCP_TIMEOUT && !args.dhcp_assign_ip_timeout_set) {
    error = "set_dhcp_timeout requires --timeout <seconds>";
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

void enableFirmwareLog(const std::shared_ptr<ob::Device> &device) {
  try {
    device->enableFirmwareLog(true);
  } catch (const ob::Error &e) {
    ROS_WARN("Failed to enable firmware log: %s",
             orbbec_camera::formatObErrorWithStatus(e).c_str());
  } catch (const std::exception &e) {
    ROS_WARN("Failed to enable firmware log: %s", e.what());
  }
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
    const auto sdk_log_path =
        orbbec_camera::configureObSdkLoggerForTool("ip_config_tool", args.sdk_log_level);
    if (!sdk_log_path.empty()) {
      ROS_INFO("SDK file log enabled: %s", sdk_log_path.c_str());
    }
    auto context = std::make_shared<ob::Context>();

    if (args.operation == CliArgs::Operation::DHCP) {
      ROS_INFO("Connecting to device %s:%d ...", args.current_ip.c_str(), args.port);
      auto device =
          context->createNetDevice(args.current_ip.c_str(), static_cast<uint16_t>(args.port));
      enableFirmwareLog(device);

      bool v2_supported =
          device->isPropertySupported(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2, OB_PERMISSION_READ_WRITE);

      if (v2_supported) {
        // Read current IP configuration from device to preserve existing settings
        OBNetIpConfigV2 ip_config_v2{};
        uint32_t data_size = sizeof(ip_config_v2);
        device->getStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                                  reinterpret_cast<uint8_t *>(&ip_config_v2), &data_size);

        if (args.dhcp) {
          // Enable DHCP, keep existing IP settings
          ip_config_v2.flags = static_cast<uint16_t>(
              (ip_config_v2.flags & ~OB_NET_IP_FLAG_PERSISTENT) | OB_NET_IP_FLAG_DHCP);
        } else {
          // Disable DHCP - only set PERSISTENT flag, keep existing static IP settings
          ip_config_v2.flags = static_cast<uint16_t>((ip_config_v2.flags & ~OB_NET_IP_FLAG_DHCP) |
                                                     OB_NET_IP_FLAG_PERSISTENT);
        }

        ROS_INFO("Applying dhcp configuration with V2 property (1088)...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                                  reinterpret_cast<const uint8_t *>(&ip_config_v2),
                                  sizeof(ip_config_v2));
        ROS_INFO("DHCP configuration applied successfully (V2).");
        ROS_INFO("DHCP target state: %s", args.dhcp ? "enabled" : "disabled");
      } else {
        uint8_t current_ip_bytes[4] = {0};
        if (args.dhcp && !parseIpString(args.current_ip, current_ip_bytes)) {
          ROS_ERROR("Invalid current_ip format: %s", args.current_ip.c_str());
          return 1;
        }

        OBNetIpConfig ip_config{};
        ip_config.dhcp = args.dhcp ? 1 : 0;
        if (args.dhcp) {
          std::memcpy(ip_config.address, current_ip_bytes, sizeof(current_ip_bytes));
        }

        ROS_WARN(
            "Device does not support IP config V2 (1088), fallback to legacy property (1041).");
        ROS_INFO("Applying dhcp configuration...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));
        ROS_INFO("DHCP configuration applied successfully.");
        ROS_INFO("DHCP target state: %s", args.dhcp ? "enabled" : "disabled");
      }
    }

    if (args.operation == CliArgs::Operation::SET_IP) {
      ROS_INFO("Connecting to device %s:%d ...", args.current_ip.c_str(), args.port);
      auto device =
          context->createNetDevice(args.current_ip.c_str(), static_cast<uint16_t>(args.port));
      enableFirmwareLog(device);

      bool v2_supported =
          device->isPropertySupported(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2, OB_PERMISSION_READ_WRITE);

      if (v2_supported) {
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
        // Preserve current addressing mode flags (DHCP/PERSISTENT/LLA), only update static IP
        // values.
        uint32_t data_size = sizeof(ip_config_v2);
        device->getStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                                  reinterpret_cast<uint8_t *>(&ip_config_v2), &data_size);
        // Ensure persistent profile is enabled so updated static fields can be used,
        // while keeping DHCP bit unchanged.
        ip_config_v2.flags = static_cast<uint16_t>(ip_config_v2.flags | OB_NET_IP_FLAG_PERSISTENT);
        std::memcpy(ip_config_v2.address, address, sizeof(address));
        std::memcpy(ip_config_v2.mask, mask, sizeof(mask));
        std::memcpy(ip_config_v2.gateway, gateway, sizeof(gateway));

        ROS_INFO("Applying set-ip configuration with V2 property (1088)...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG_V2,
                                  reinterpret_cast<const uint8_t *>(&ip_config_v2),
                                  sizeof(ip_config_v2));

        ROS_INFO("Set-ip configuration applied successfully (V2).");
        ROS_INFO("Set-ip target mode: unchanged (flags=%u).", ip_config_v2.flags);
        ROS_INFO("Set-ip target static IP: %d.%d.%d.%d", ip_config_v2.address[0],
                 ip_config_v2.address[1], ip_config_v2.address[2], ip_config_v2.address[3]);
        ROS_INFO("Set-ip target mask: %d.%d.%d.%d", ip_config_v2.mask[0], ip_config_v2.mask[1],
                 ip_config_v2.mask[2], ip_config_v2.mask[3]);
        ROS_INFO("Set-ip target gateway: %d.%d.%d.%d", ip_config_v2.gateway[0],
                 ip_config_v2.gateway[1], ip_config_v2.gateway[2], ip_config_v2.gateway[3]);
      } else {
        OBNetIpConfig ip_config{};
        // Preserve current DHCP mode on legacy property as well.
        uint32_t data_size = sizeof(ip_config);
        device->getStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<uint8_t *>(&ip_config), &data_size);
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

        ROS_WARN(
            "Device does not support IP config V2 (1088), fallback to legacy property (1041).");

        ROS_INFO("Applying set-ip configuration...");
        device->setStructuredData(OB_STRUCT_DEVICE_IP_ADDR_CONFIG,
                                  reinterpret_cast<const uint8_t *>(&ip_config), sizeof(ip_config));

        ROS_INFO("Set-ip configuration applied successfully.");
        ROS_INFO("Set-ip target mode: unchanged (dhcp=%u).", ip_config.dhcp);
        ROS_INFO("Set-ip target static IP: %d.%d.%d.%d", ip_config.address[0], ip_config.address[1],
                 ip_config.address[2], ip_config.address[3]);
        ROS_INFO("Set-ip target mask: %d.%d.%d.%d", ip_config.mask[0], ip_config.mask[1],
                 ip_config.mask[2], ip_config.mask[3]);
        ROS_INFO("Set-ip target gateway: %d.%d.%d.%d", ip_config.gateway[0], ip_config.gateway[1],
                 ip_config.gateway[2], ip_config.gateway[3]);
      }
    }

    if (args.operation == CliArgs::Operation::FORCE_IP) {
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

      OBNetIpConfig ip_config{};
      ip_config.dhcp = 0;
      std::memcpy(ip_config.address, address, sizeof(address));
      std::memcpy(ip_config.mask, mask, sizeof(mask));
      std::memcpy(ip_config.gateway, gateway, sizeof(gateway));

      ROS_INFO("Applying force-ip to MAC %s ...", args.force_ip_mac.c_str());
      if (context->forceIp(args.force_ip_mac.c_str(), ip_config)) {
        ROS_INFO("Force-ip operation applied successfully.");
        std::this_thread::sleep_for(std::chrono::seconds(5));
        ROS_INFO("Force-ip target static IP: %s", args.new_ip.c_str());
        ROS_INFO("Force-ip target mask: %s", args.mask.c_str());
        ROS_INFO("Force-ip target gateway: %s", args.gateway.c_str());
      } else {
        ROS_ERROR("Force-ip failed (SDK returned false).");
        return 1;
      }
    }

    if (args.operation == CliArgs::Operation::SET_DHCP_TIMEOUT) {
      ROS_INFO("Connecting to device %s:%d ...", args.current_ip.c_str(), args.port);
      auto device =
          context->createNetDevice(args.current_ip.c_str(), static_cast<uint16_t>(args.port));
      enableFirmwareLog(device);

      if (!device->isPropertySupported(OB_PROP_DHCP_ASSIGN_IP_TIMEOUT_INT, OB_PERMISSION_WRITE)) {
        ROS_ERROR("Current device or firmware does not support DHCP assign IP timeout");
        return 1;
      }

      auto range = device->getIntPropertyRange(OB_PROP_DHCP_ASSIGN_IP_TIMEOUT_INT);
      if (args.dhcp_assign_ip_timeout < range.min || args.dhcp_assign_ip_timeout > range.max) {
        ROS_ERROR("Timeout %d is out of range [%d, %d]", args.dhcp_assign_ip_timeout, range.min,
                  range.max);
        return 1;
      }

      device->setIntProperty(OB_PROP_DHCP_ASSIGN_IP_TIMEOUT_INT, args.dhcp_assign_ip_timeout);
      const int current_timeout = device->getIntProperty(OB_PROP_DHCP_ASSIGN_IP_TIMEOUT_INT);
      ROS_INFO("DHCP assign IP timeout applied successfully: %d second(s)", current_timeout);
    }

  } catch (const ob::Error &e) {
    ROS_ERROR_STREAM("ip_config_tool: " << orbbec_camera::formatObErrorWithStatus(e));
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
