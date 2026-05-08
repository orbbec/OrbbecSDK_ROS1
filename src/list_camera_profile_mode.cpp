#include <orbbec_camera/ob_camera_node.h>
#include <iostream>
#include <memory>
#include <string>

using namespace orbbec_camera;

namespace {

struct CommandLineOptions {
  bool show_help = false;
  std::string serial_number;
  std::string sdk_log_level = "off";
};

void printUsage(const char* program_name) {
  (void)program_name;
  std::cout
      << "Usage:\n"
      << "rosrun orbbec_camera list_camera_profile_mode_node\\\n"
      << "      [--serial_number SN]\n\n"
      << "Parameters:\n"
      << "  --serial_number SN  Select a specific camera by serial number.\n"
      << "  --enable_sdk_log    Enable SDK file log at debug level under ~/.ros/Log.\n"
      << "  --sdk_log_level LEVEL\n"
      << "                      SDK file log level: debug/info/warn/error/fatal/off "
         "(default: off).\n"
      << "  -h, --help          Show this help message.\n"
      << "Examples:\n"
      << "  rosrun orbbec_camera list_camera_profile_mode_node --enable_sdk_log "
         "--sdk_log_level debug\n";
}

CommandLineOptions parseCommandLine(int argc, char** argv) {
  CommandLineOptions options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "-h" || arg == "--help") {
      options.show_help = true;
      continue;
    }

    if (arg == "--serial_number") {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " requires a value");
      }
      options.serial_number = argv[++i];
      continue;
    }

    const std::string prefix = "--serial_number=";
    if (arg.rfind(prefix, 0) == 0) {
      options.serial_number = arg.substr(prefix.size());
      if (options.serial_number.empty()) {
        throw std::runtime_error("--serial_number requires a value");
      }
      continue;
    }

    if (arg == "--enable_sdk_log") {
      options.sdk_log_level = "debug";
      continue;
    }

    const std::string sdk_log_prefix = "--sdk_log_level=";
    if (arg.rfind(sdk_log_prefix, 0) == 0) {
      options.sdk_log_level = arg.substr(sdk_log_prefix.size());
      continue;
    }
    if (arg == "--sdk_log_level") {
      if (i + 1 >= argc) {
        throw std::runtime_error(arg + " requires a value");
      }
      options.sdk_log_level = argv[++i];
      continue;
    }

    // Ignore ROS remapping arguments and other unknown flags to preserve compatibility.
    if (arg.find(":=") != std::string::npos || arg.rfind("__", 0) == 0 || arg.rfind("_", 0) == 0) {
      continue;
    }

    throw std::runtime_error("Unknown argument: " + arg);
  }

  const auto log_severity = obLogSeverityFromString(options.sdk_log_level);
  if (log_severity == OBLogSeverity::OB_LOG_SEVERITY_OFF && options.sdk_log_level != "off" &&
      options.sdk_log_level != "none") {
    throw std::runtime_error("--sdk_log_level expects one of: debug, info, warn, error, fatal, off");
  }

  return options;
}

std::shared_ptr<ob::Device> initializeDevice(const std::string& serial_number) {
  auto context = std::make_shared<ob::Context>();
  auto device_list = context->queryDeviceList();
  if (!device_list || device_list->deviceCount() == 0) {
    std::cout << "No device found" << std::endl;
    return nullptr;
  }

  if (!serial_number.empty()) {
    return device_list->getDeviceBySN(serial_number.c_str(), OB_DEVICE_DEFAULT_ACCESS);
  }

  return device_list->getDevice(0, OB_DEVICE_DEFAULT_ACCESS);
}

}  // namespace

void listSensorProfiles(const std::shared_ptr<ob::Device>& device) {
  auto sensor_list = device->getSensorList();
  auto pid = device->getDeviceInfo()->getPid();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->count(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if ((sensor->getType() == OB_SENSOR_DEPTH || sensor->getType() == OB_SENSOR_IR_LEFT ||
           sensor->getType() == OB_SENSOR_IR_RIGHT) &&
          isGemini305SeriesPID(pid)) {
        // Gemini 305 series
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << sensor->type() << " profile: " << profile->getWidth() << "x"
                  << profile->getHeight() << " " << profile->getFps() << "fps " << sensor->type()
                  << " | width: " << profile->getDecimationConfig().originWidth
                  << " height: " << profile->getDecimationConfig().originHeight
                  << " downscale:" << profile->getDecimationConfig().factor << std::endl;
      } else if (sensor->type() == OB_SENSOR_COLOR || sensor->type() == OB_SENSOR_DEPTH ||
                 sensor->type() == OB_SENSOR_IR || sensor->type() == OB_SENSOR_IR_LEFT ||
                 sensor->type() == OB_SENSOR_IR_RIGHT) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        std::cout << sensor->type() << " profile: " << profile->width() << "x" << profile->height()
                  << " " << profile->fps() << "fps " << profile->format() << std::endl;
      } else if (sensor->type() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        std::cout << sensor->type() << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << std::endl;
      } else if (sensor->type() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        std::cout << sensor->type() << " profile: " << profile->sampleRate()
                  << "  full scale_range " << profile->fullScaleRange() << std::endl;
      } else if (sensor->type() == OB_SENSOR_LIDAR) {
        auto profile = origin_profile->as<ob::LiDARStreamProfile>();
        std::cout << sensor->type() << " scan rate: " << profile->getScanRate()
                  << "  format:" << profile->getFormat() << std::endl;
      } else {
        std::cout << "Unknown profile: " << sensor->type() << std::endl;
      }
    }
  }
}

void printDeviceProperties(const std::shared_ptr<ob::Device>& device) {
  if (!device->isPropertySupported(OB_STRUCT_CURRENT_DEPTH_ALG_MODE, OB_PERMISSION_READ_WRITE)) {
    std::cout << "Current device not support depth work mode!" << std::endl;
    return;
  }
  auto current_depth_mode = device->getCurrentDepthWorkMode();
  std::cout << "Current depth mode: " << current_depth_mode.name << std::endl;
  auto depth_mode_list = device->getDepthWorkModeList();
  std::cout << "Depth mode list: " << std::endl;
  for (uint32_t i = 0; i < depth_mode_list->count(); i++) {
    std::cout << "Depth_mode_list[" << i << "]: " << (*depth_mode_list)[i].name << std::endl;
  }
}

void printPreset(const std::shared_ptr<ob::Device>& device) {
  auto preset_list = device->getAvailablePresetList();
  if (!preset_list || preset_list->count() == 0) {
    return;
  }
  std::cout << "Preset list:" << std::endl;
  for (uint32_t i = 0; i < preset_list->count(); i++) {
    auto name = preset_list->getName(i);
    std::cout << "Preset list[" << i << "]: " << name << std::endl;
  }
}

int main(int argc, char** argv) {
  try {
    const auto options = parseCommandLine(argc, argv);
    if (options.show_help) {
      printUsage(argv[0]);
      return 0;
    }

    const auto sdk_log_path =
        configureObSdkLoggerForTool("list_camera_profile_mode_node", options.sdk_log_level);
    if (!sdk_log_path.empty()) {
      std::cout << "SDK file log enabled: " << sdk_log_path << std::endl;
    }
    auto device = initializeDevice(options.serial_number);
    if (!device) {
      return -1;  // Device initialization failed
    }
    listSensorProfiles(device);
    printDeviceProperties(device);
    printPreset(device);
  } catch (ob::Error& e) {
    ROS_ERROR_STREAM("list_camera_profile_mode: " << orbbec_camera::formatObErrorWithStatus(e));
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM("list_camera_profile_mode: " << e.what());
  } catch (...) {
    ROS_ERROR_STREAM("list_camera_profile_mode: "
                     << "unknown error");
  }
  return 0;
}
