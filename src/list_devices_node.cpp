#include <ros/ros.h>
#include <orbbec_camera/types.h>
#include <string>
#include <regex>
std::string parseUsbPort(const std::string& line) {
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
  auto context = std::make_shared<ob::Context>();
  context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
  auto list = context->queryDeviceList();
  for (size_t i = 0; i < list->deviceCount(); i++) {
    auto device = list->getDevice(i);
    auto device_info = device->getDeviceInfo();
    std::string serial = device_info->serialNumber();
    std::string uid = device_info->uid();
    auto port_id = parseUsbPort(uid);
    ROS_INFO_STREAM("serial: " << serial);
    ROS_INFO_STREAM("port id : " << port_id);
  }
  return 0;
}
