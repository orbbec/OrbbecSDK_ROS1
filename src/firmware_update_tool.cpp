/*******************************************************************************
 * Copyright (c) 2026 Orbbec 3D Technology, Inc
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

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "libobsensor/ObSensor.hpp"

namespace {

struct CliArgs {
  std::string serial_number;
  std::string usb_port;
  std::string device_ip;
  int device_port = 8090;
  std::string firmware_path;
  std::string preset_path;
  int reconnect_timeout_sec = 120;
  int reconnect_poll_ms = 1000;
  bool continue_on_error = false;
  bool used_deprecated_upgrade_firmware = false;
  bool used_deprecated_preset_firmware_path = false;
  bool help = false;
};

struct FirmwareUpdateResult {
  bool success = false;
  bool need_reupdate = false;
  bool retryable = true;
  OBFwUpdateState final_state = STAT_START;
  std::string error_message;
};

std::string trim(std::string value) {
  const auto not_space = [](unsigned char ch) { return !std::isspace(ch); };
  value.erase(value.begin(), std::find_if(value.begin(), value.end(), not_space));
  value.erase(std::find_if(value.rbegin(), value.rend(), not_space).base(), value.end());
  return value;
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

std::vector<std::string> splitCsv(const std::string &value) {
  std::vector<std::string> result;
  std::stringstream ss(value);
  std::string segment;
  while (std::getline(ss, segment, ',')) {
    segment = trim(segment);
    if (!segment.empty()) {
      result.push_back(segment);
    }
  }
  return result;
}

void printUsage(const char *program) {
  std::cout
      << "Usage:\n"
      << "rosrun orbbec_camera firmware_update_tool\\\n"
      << "      [--serial_number SN[,SN2...]]\\\n"
      << "      [--firmware_path /path/to/firmware.bin]\\\n"
      << "      [--preset_path /path/a.bin,/path/b.bin]\\\n"
      << "      [--continue_on_error]\n\n"
      << "Notes:\n"
      << "  1) At least one of --firmware_path / --preset_path must be provided.\n"
      << "  2) If multiple devices are connected, specify target by serial/usb/ip to avoid wrong "
         "updates.\n"
      << "  3) Repeating --serial_number or passing comma-separated values enables sequential "
         "batch update.\n";
}

bool parseArgs(int argc, char **argv, CliArgs &args, std::string &error) {
  auto appendSerialNumbers = [&](const std::string &value) {
    auto list = splitCsv(value);
    if (list.empty()) {
      return;
    }

    std::stringstream ss;
    if (!args.serial_number.empty()) {
      ss << args.serial_number << ",";
    }
    for (size_t i = 0; i < list.size(); ++i) {
      if (i > 0) {
        ss << ",";
      }
      ss << list[i];
    }
    args.serial_number = ss.str();
  };

  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    if (current == "-h" || current == "--help") {
      args.help = true;
      return true;
    }

    if (current.rfind("--serial_number=", 0) == 0) {
      appendSerialNumbers(current.substr(std::strlen("--serial_number=")));
      continue;
    }
    if (current == "--serial_number") {
      if (++i >= argc) {
        error = "--serial_number requires a value";
        return false;
      }
      appendSerialNumbers(argv[i]);
      continue;
    }

    if (current.rfind("--usb_port=", 0) == 0) {
      args.usb_port = current.substr(std::strlen("--usb_port="));
      continue;
    }
    if (current == "--usb_port") {
      if (++i >= argc) {
        error = "--usb_port requires a value";
        return false;
      }
      args.usb_port = argv[i];
      continue;
    }

    if (current.rfind("--device_ip=", 0) == 0) {
      args.device_ip = current.substr(std::strlen("--device_ip="));
      continue;
    }
    if (current == "--device_ip") {
      if (++i >= argc) {
        error = "--device_ip requires a value";
        return false;
      }
      args.device_ip = argv[i];
      continue;
    }

    if (current.rfind("--device_port=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--device_port=")), args.device_port)) {
        error = "--device_port expects an integer";
        return false;
      }
      continue;
    }
    if (current == "--device_port") {
      if (++i >= argc || !parseInt(argv[i], args.device_port)) {
        error = "--device_port expects an integer";
        return false;
      }
      continue;
    }

    if (current.rfind("--firmware_path=", 0) == 0) {
      args.firmware_path = current.substr(std::strlen("--firmware_path="));
      continue;
    }
    if (current == "--firmware_path") {
      if (++i >= argc) {
        error = "--firmware_path requires a value";
        return false;
      }
      args.firmware_path = argv[i];
      continue;
    }

    // Backward compatibility: prefer --firmware_path
    if (current.rfind("--upgrade_firmware=", 0) == 0) {
      args.firmware_path = current.substr(std::strlen("--upgrade_firmware="));
      args.used_deprecated_upgrade_firmware = true;
      continue;
    }
    if (current == "--upgrade_firmware") {
      if (++i >= argc) {
        error = "--upgrade_firmware requires a value";
        return false;
      }
      args.firmware_path = argv[i];
      args.used_deprecated_upgrade_firmware = true;
      continue;
    }

    if (current.rfind("--preset_path=", 0) == 0) {
      args.preset_path = current.substr(std::strlen("--preset_path="));
      continue;
    }
    if (current == "--preset_path") {
      if (++i >= argc) {
        error = "--preset_path requires a value";
        return false;
      }
      args.preset_path = argv[i];
      continue;
    }

    // Backward compatibility: prefer --preset_path
    if (current.rfind("--preset_firmware_path=", 0) == 0) {
      args.preset_path = current.substr(std::strlen("--preset_firmware_path="));
      args.used_deprecated_preset_firmware_path = true;
      continue;
    }
    if (current == "--preset_firmware_path") {
      if (++i >= argc) {
        error = "--preset_firmware_path requires a value";
        return false;
      }
      args.preset_path = argv[i];
      args.used_deprecated_preset_firmware_path = true;
      continue;
    }

    if (current.rfind("--reconnect_timeout_sec=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--reconnect_timeout_sec=")),
                    args.reconnect_timeout_sec)) {
        error = "--reconnect_timeout_sec expects an integer";
        return false;
      }
      continue;
    }
    if (current == "--reconnect_timeout_sec") {
      if (++i >= argc || !parseInt(argv[i], args.reconnect_timeout_sec)) {
        error = "--reconnect_timeout_sec expects an integer";
        return false;
      }
      continue;
    }

    if (current.rfind("--reconnect_poll_ms=", 0) == 0) {
      if (!parseInt(current.substr(std::strlen("--reconnect_poll_ms=")), args.reconnect_poll_ms)) {
        error = "--reconnect_poll_ms expects an integer";
        return false;
      }
      continue;
    }
    if (current == "--reconnect_poll_ms") {
      if (++i >= argc || !parseInt(argv[i], args.reconnect_poll_ms)) {
        error = "--reconnect_poll_ms expects an integer";
        return false;
      }
      continue;
    }

    if (current == "--continue_on_error") {
      args.continue_on_error = true;
      continue;
    }

    error = "Unknown argument: " + current;
    return false;
  }

  const int selected_by = (!args.serial_number.empty() ? 1 : 0) + (!args.usb_port.empty() ? 1 : 0) +
                          (!args.device_ip.empty() ? 1 : 0);
  if (selected_by > 1) {
    error = "Only one selector can be used at a time: serial_number / usb_port / device_ip";
    return false;
  }

  if (args.device_port <= 0 || args.device_port > 65535) {
    error = "--device_port must be in range 1-65535";
    return false;
  }

  if (args.reconnect_timeout_sec <= 0) {
    error = "--reconnect_timeout_sec must be > 0";
    return false;
  }

  if (args.reconnect_poll_ms <= 0) {
    error = "--reconnect_poll_ms must be > 0";
    return false;
  }

  if (args.firmware_path.empty() && args.preset_path.empty()) {
    error = "At least one action is required: --firmware_path or --preset_path";
    return false;
  }

  return true;
}

std::vector<std::string> splitPresetPaths(const std::string &path_arg) {
  return splitCsv(path_arg);
}

bool isRegularFilePath(const std::string &path) {
  struct stat st = {};
  return !path.empty() && ::stat(path.c_str(), &st) == 0 && S_ISREG(st.st_mode);
}

const char *stateToString(OBFwUpdateState state) {
  switch (state) {
    case STAT_VERIFY_SUCCESS:
      return "Image file verification success";
    case STAT_FILE_TRANSFER:
      return "File transfer in progress";
    case STAT_DONE:
      return "Update completed";
    case STAT_DONE_REBOOT_AND_REUPDATE:
      return "Update completed, requires reboot and reupdate";
    case STAT_DONE_WITH_DUPLICATES:
      return "Update completed, duplicated presets were ignored";
    case STAT_IN_PROGRESS:
      return "Update in progress";
    case STAT_START:
      return "Starting the update";
    case STAT_VERIFY_IMAGE:
      return "Verifying image file";
    default:
      return "Unknown status or error";
  }
}

bool isBootDeviceName(const std::string &name) {
  std::string lower = name;
  std::transform(lower.begin(), lower.end(), lower.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return lower == "boot" || lower.find("boot") != std::string::npos;
}

void printUpdateProgress(const std::string &task, OBFwUpdateState state, const char *message,
                         uint8_t percent) {
  std::cout << "[" << task << "] " << static_cast<uint32_t>(percent) << "% | "
            << stateToString(state) << " | " << (message != nullptr ? message : "") << std::endl;
}

void logCurrentPresetList(const std::shared_ptr<ob::Device> &device, const char *stage) {
  try {
    auto preset_list = device->getAvailablePresetList();
    if (!preset_list) {
      ROS_INFO("[%s] Current preset list is empty or unavailable", stage);
      return;
    }

    const uint32_t count = preset_list->getCount();
    ROS_INFO("[%s] Current preset count: %u", stage, count);
    for (uint32_t i = 0; i < count; ++i) {
      ROS_INFO("[%s] Preset[%u]: %s", stage, i, preset_list->getName(i));
    }
  } catch (const ob::Error &e) {
    ROS_WARN("[%s] Failed to query preset list: %s", stage, e.getMessage());
  } catch (const std::exception &e) {
    ROS_WARN("[%s] Failed to query preset list: %s", stage, e.what());
  }
}

std::shared_ptr<ob::Device> selectDeviceFromList(const std::shared_ptr<ob::DeviceList> &list,
                                                 const CliArgs &args) {
  const size_t count = list->getCount();
  if (count == 0) {
    throw std::runtime_error("No device found");
  }

  if (!args.serial_number.empty()) {
    for (size_t i = 0; i < count; ++i) {
      try {
        if (list->getSerialNumber(i) == args.serial_number) {
          return list->getDevice(i, OB_DEVICE_DEFAULT_ACCESS);
        }
      } catch (const ob::Error &) {
        continue;
      }
    }
    throw std::runtime_error("Device not found by serial_number: " + args.serial_number);
  }

  if (!args.usb_port.empty()) {
    auto device = list->getDeviceByUid(args.usb_port.c_str(), OB_DEVICE_DEFAULT_ACCESS);
    if (!device) {
      throw std::runtime_error("Device not found by usb_port(uid): " + args.usb_port);
    }
    return device;
  }

  if (count == 1) {
    return list->getDevice(0, OB_DEVICE_DEFAULT_ACCESS);
  }

  ROS_ERROR(
      "Multiple devices detected (%zu). Please provide --serial_number, --usb_port, or "
      "--device_ip.",
      count);
  throw std::runtime_error("Multiple devices detected without explicit selector");
}

std::shared_ptr<ob::Device> connectDevice(const std::shared_ptr<ob::Context> &ctx,
                                          const CliArgs &args) {
  if (!args.device_ip.empty()) {
    ROS_INFO("Connecting network device %s:%d", args.device_ip.c_str(), args.device_port);
    return ctx->createNetDevice(args.device_ip.c_str(), static_cast<uint16_t>(args.device_port),
                                OB_DEVICE_DEFAULT_ACCESS);
  }

  auto list = ctx->queryDeviceList();
  return selectDeviceFromList(list, args);
}

std::shared_ptr<ob::Device> waitForReconnect(const std::shared_ptr<ob::Context> &ctx,
                                             const CliArgs &args, bool require_non_boot = false) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(args.reconnect_timeout_sec);
  while (std::chrono::steady_clock::now() < deadline) {
    try {
      auto device = connectDevice(ctx, args);
      if (device) {
        auto device_info = device->getDeviceInfo();
        const std::string name = device_info->getName();
        const std::string serial = device_info->getSerialNumber();
        const bool is_boot = isBootDeviceName(name);
        ROS_INFO("Device reconnected: %s (%s)", name.c_str(), serial.c_str());
        if (require_non_boot && is_boot) {
          ROS_INFO("Device is still in boot stage, waiting for normal mode...");
          std::this_thread::sleep_for(std::chrono::milliseconds(args.reconnect_poll_ms));
          continue;
        }
        return device;
      }
    } catch (const ob::Error &e) {
      ROS_WARN_THROTTLE(5.0, "Reconnect attempt failed (SDK): %s", e.getMessage());
    } catch (const std::exception &e) {
      ROS_WARN_THROTTLE(5.0, "Reconnect attempt failed: %s", e.what());
    } catch (...) {
      ROS_WARN_THROTTLE(5.0, "Reconnect attempt failed: unknown error");
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(args.reconnect_poll_ms));
  }

  throw std::runtime_error("Timeout waiting for device reconnection");
}

std::shared_ptr<ob::Device> waitForReconnectUntil(
    const std::shared_ptr<ob::Context> &ctx, const CliArgs &args, bool require_non_boot,
    const std::chrono::steady_clock::time_point &deadline) {
  const auto now = std::chrono::steady_clock::now();
  if (now >= deadline) {
    throw std::runtime_error("Timeout waiting for device reconnection");
  }

  const auto remaining_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count();
  CliArgs bounded_args = args;
  bounded_args.reconnect_timeout_sec = std::max(1, static_cast<int>((remaining_ms + 999) / 1000));
  bounded_args.reconnect_poll_ms =
      std::max(100, std::min(bounded_args.reconnect_poll_ms, static_cast<int>(remaining_ms)));
  return waitForReconnect(ctx, bounded_args, require_non_boot);
}

bool updatePresetFirmware(const std::shared_ptr<ob::Device> &device, const std::string &path_arg,
                          std::string *error_message = nullptr) {
  auto set_error = [&](const std::string &message) {
    if (error_message != nullptr) {
      *error_message = message;
    }
  };

  if (path_arg.empty()) {
    return true;
  }

  logCurrentPresetList(device, "before preset update");

  auto paths = splitPresetPaths(path_arg);
  if (paths.empty()) {
    set_error("preset_path is empty after parsing");
    return false;
  }
  if (paths.size() > 255) {
    set_error("Too many preset files: " + std::to_string(paths.size()) + ", maximum is 255");
    return false;
  }

  const uint8_t count = static_cast<uint8_t>(paths.size());
  std::unique_ptr<char[]> raw_file_paths(new char[static_cast<size_t>(count) * OB_PATH_MAX]{});
  auto *file_paths = reinterpret_cast<char(*)[OB_PATH_MAX]>(raw_file_paths.get());

  for (uint8_t i = 0; i < count; ++i) {
    if (!isRegularFilePath(paths[i])) {
      set_error("Preset file not found: " + paths[i]);
      return false;
    }
    if (paths[i].size() >= OB_PATH_MAX) {
      set_error("Preset path too long: " + paths[i]);
      return false;
    }
    std::strncpy(file_paths[i], paths[i].c_str(), OB_PATH_MAX - 1);
    file_paths[i][OB_PATH_MAX - 1] = '\0';
    ROS_INFO("Preset file[%d]: %s", static_cast<int>(i), file_paths[i]);
  }

  OBFwUpdateState final_state = STAT_START;
  ROS_INFO("Start updating optional depth presets...");

  device->updateOptionalDepthPresets(
      file_paths, count,
      [&final_state](OBFwUpdateState state, const char *message, uint8_t percent) {
        final_state = state;
        printUpdateProgress("preset", state, message, percent);
      });

  if (final_state == STAT_DONE || final_state == STAT_DONE_WITH_DUPLICATES) {
    logCurrentPresetList(device, "after preset update");
    ROS_INFO("Preset update succeeded.");
    return true;
  }

  set_error("Preset update ended with state: " + std::to_string(static_cast<int>(final_state)));
  return false;
}

FirmwareUpdateResult updateFirmware(const std::shared_ptr<ob::Device> &device,
                                    const std::string &firmware_path) {
  FirmwareUpdateResult result;
  if (firmware_path.empty()) {
    result.success = true;
    return result;
  }

  if (!isRegularFilePath(firmware_path)) {
    result.retryable = false;
    result.error_message = "Firmware file not found: " + firmware_path;
    return result;
  }

  ROS_INFO("Start firmware update from: %s", firmware_path.c_str());
  OBFwUpdateState last_logged_state = STAT_START;
  int last_logged_bucket = -1;
  device->updateFirmware(
      firmware_path.c_str(),
      [&result, &last_logged_state, &last_logged_bucket](OBFwUpdateState state, const char *message,
                                                         uint8_t percent) {
        result.final_state = state;

        const int bucket = static_cast<int>(percent) / 10;
        const bool first_log = (last_logged_bucket < 0);
        const bool state_changed = (!first_log && state != last_logged_state);
        const bool bucket_changed = (!first_log && bucket != last_logged_bucket);
        const bool is_boundary = (percent == 0 || percent == 100);

        if (first_log || state_changed || bucket_changed || is_boundary) {
          printUpdateProgress("firmware", state, message, percent);
          last_logged_state = state;
          last_logged_bucket = bucket;
        }
      },
      false);

  if (result.final_state == STAT_DONE || result.final_state == STAT_DONE_REBOOT_AND_REUPDATE) {
    result.success = true;
    result.need_reupdate = (result.final_state == STAT_DONE_REBOOT_AND_REUPDATE);
  }

  if (!result.success) {
    result.error_message = "Firmware update failed with state: " +
                           std::to_string(static_cast<int>(result.final_state));
    return result;
  }

  ROS_INFO("Rebooting device after firmware update...");
  device->reboot();
  ROS_INFO("Device reboot command sent.");
  return result;
}

}  // namespace

int main(int argc, char **argv) {
  CliArgs args;
  std::string parse_error;
  if (!parseArgs(argc, argv, args, parse_error)) {
    std::cerr << "Argument error: " << parse_error << std::endl;
    printUsage(argv[0]);
    return 1;
  }
  if (args.help) {
    printUsage(argv[0]);
    return 0;
  }

  ros::init(argc, argv, "firmware_update_tool");
  ros::NodeHandle nh("~");
  (void)nh;

  try {
    ob::Context::setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_OFF);
    auto ctx = std::make_shared<ob::Context>();
    std::vector<std::string> batch_targets = splitCsv(args.serial_number);
    if (batch_targets.empty()) {
      batch_targets.emplace_back("");
    }

    if (args.used_deprecated_upgrade_firmware) {
      ROS_WARN("Argument --upgrade_firmware is deprecated, please use --firmware_path instead.");
    }
    if (args.used_deprecated_preset_firmware_path) {
      ROS_WARN("Argument --preset_firmware_path is deprecated, please use --preset_path instead.");
    }

    size_t success_count = 0;
    std::vector<std::string> failed_targets;
    for (size_t i = 0; i < batch_targets.size(); ++i) {
      CliArgs run_args = args;
      if (!batch_targets[i].empty()) {
        run_args.serial_number = batch_targets[i];
        run_args.usb_port.clear();
        run_args.device_ip.clear();
      }
      if (batch_targets.size() > 1) {
        ROS_INFO("========== Batch %zu/%zu, target SN: %s ==========", i + 1, batch_targets.size(),
                 run_args.serial_number.c_str());
      }

      try {
        auto device = connectDevice(ctx, run_args);
        auto device_info = device->getDeviceInfo();
        ROS_INFO("Selected device: %s, SN: %s, UID: %s", device_info->getName(),
                 device_info->getSerialNumber(), device_info->getUid());

        if (!run_args.preset_path.empty()) {
          std::string preset_error;
          const bool preset_ok = updatePresetFirmware(device, run_args.preset_path, &preset_error);
          if (!preset_ok) {
            throw std::runtime_error(preset_error.empty() ? "Preset firmware update failed"
                                                          : preset_error);
          }
        }

        if (!run_args.firmware_path.empty()) {
          auto first_update = updateFirmware(device, run_args.firmware_path);
          if (!first_update.success) {
            throw std::runtime_error(first_update.error_message.empty()
                                         ? "First firmware update failed"
                                         : first_update.error_message);
          }

          if (first_update.need_reupdate) {
            ROS_INFO("Firmware requires reboot and second update. Waiting for device reconnect...");
            const auto second_deadline = std::chrono::steady_clock::now() +
                                         std::chrono::seconds(run_args.reconnect_timeout_sec);
            device = waitForReconnectUntil(ctx, run_args, true, second_deadline);
            bool second_ok = false;
            while (std::chrono::steady_clock::now() < second_deadline) {
              try {
                auto second_update = updateFirmware(device, run_args.firmware_path);
                if (!second_update.success) {
                  if (!second_update.retryable) {
                    throw std::runtime_error(second_update.error_message.empty()
                                                 ? "Second firmware update failed"
                                                 : second_update.error_message);
                  }
                  if (second_update.error_message.empty()) {
                    ROS_WARN("Second firmware update attempt failed, retrying...");
                  } else {
                    ROS_WARN("Second firmware update attempt failed: %s, retrying...",
                             second_update.error_message.c_str());
                  }
                  device = waitForReconnectUntil(ctx, run_args, true, second_deadline);
                  continue;
                }
                if (second_update.need_reupdate) {
                  ROS_WARN("Second attempt still requires reupdate, waiting and retrying...");
                  device = waitForReconnectUntil(ctx, run_args, true, second_deadline);
                  continue;
                }
                second_ok = true;
                break;
              } catch (const ob::Error &e) {
                ROS_WARN("Second update transient error: %s, retrying...", e.getMessage());
                device = waitForReconnectUntil(ctx, run_args, true, second_deadline);
              }
            }
            if (!second_ok) {
              throw std::runtime_error("Second firmware update failed after retries");
            }
          }
        }

        success_count++;
      } catch (const ob::Error &e) {
        const std::string target =
            run_args.serial_number.empty() ? "<default>" : run_args.serial_number;
        ROS_ERROR("Target %s failed: %s", target.c_str(), e.getMessage());
        failed_targets.push_back(target);
        if (!args.continue_on_error) {
          throw;
        }
      } catch (const std::exception &e) {
        const std::string target =
            run_args.serial_number.empty() ? "<default>" : run_args.serial_number;
        ROS_ERROR("Target %s failed: %s", target.c_str(), e.what());
        failed_targets.push_back(target);
        if (!args.continue_on_error) {
          throw;
        }
      }
    }

    if (!failed_targets.empty()) {
      std::stringstream ss;
      for (size_t i = 0; i < failed_targets.size(); ++i) {
        if (i > 0) ss << ", ";
        ss << failed_targets[i];
      }
      ROS_ERROR("Batch completed with failures. success=%zu, failed=%zu, targets=[%s]",
                success_count, failed_targets.size(), ss.str().c_str());
      throw std::runtime_error("Batch firmware update finished with failures");
    }

    ROS_INFO("Firmware tool completed successfully. Updated %zu/%zu target device(s).",
             success_count, batch_targets.size());
    ros::shutdown();
    return 0;
  } catch (const ob::Error &e) {
    ROS_ERROR("ob::Error: %s", e.getMessage());
  } catch (const std::exception &e) {
    ROS_ERROR("Exception: %s", e.what());
  } catch (...) {
    ROS_ERROR("Unknown error");
  }

  ros::shutdown();
  return 1;
}
