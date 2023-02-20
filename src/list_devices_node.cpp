#include <ros/ros.h>
#include <orbbec_camera/types.h>

int main() {
  auto context = std::make_shared<ob::Context>();
  context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
  auto list = context->queryDeviceList();
  for (size_t i = 0; i < list->deviceCount(); i++) {
    auto serial = list->getDevice(i)->getDeviceInfo()->serialNumber();
    ROS_INFO_STREAM("serial: " << serial);
  }
  return 0;
}
