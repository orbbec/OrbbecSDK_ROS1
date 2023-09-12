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
#include <memory>
#include <orbbec_camera/utils.h>
int main() {
  using namespace orbbec_camera;
  auto context = std::make_shared<ob::Context>();
  context->setLoggerSeverity(OBLogSeverity::OB_LOG_SEVERITY_NONE);
  auto device_list = context->queryDeviceList();
  auto device = device_list->getDevice(0);
  auto sensor_list = device->getSensorList();
  for (size_t i = 0; i < sensor_list->count(); i++) {
    auto sensor = sensor_list->getSensor(i);
    auto profile_list = sensor->getStreamProfileList();
    for (size_t j = 0; j < profile_list->count(); j++) {
      auto origin_profile = profile_list->getProfile(j);
      if (sensor->type() == OB_SENSOR_COLOR) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        ROS_INFO_STREAM("color profile: " << profile->width() << "x" << profile->height() << " "
                                          << profile->fps() << "fps " << profile->format());
      } else if (sensor->type() == OB_SENSOR_DEPTH) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        ROS_INFO_STREAM("depth profile: " << profile->width() << "x" << profile->height() << " "
                                          << profile->fps() << "fps " << profile->format());
      } else if (sensor->type() == OB_SENSOR_IR) {
        auto profile = origin_profile->as<ob::VideoStreamProfile>();
        ROS_INFO_STREAM("ir profile: " << profile->width() << "x" << profile->height() << " "
                                       << profile->fps() << "fps " << profile->format());
      } else if (sensor->type() == OB_SENSOR_ACCEL) {
        auto profile = origin_profile->as<ob::AccelStreamProfile>();
        ROS_INFO_STREAM("accel profile: sampleRate "
                        << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                        << fullAccelScaleRangeToString(profile->fullScaleRange()));
      } else if (sensor->type() == OB_SENSOR_GYRO) {
        auto profile = origin_profile->as<ob::GyroStreamProfile>();
        ROS_INFO_STREAM("gyro profile: sampleRate "
                        << sampleRateToString(profile->sampleRate()) << "  full scale_range "
                        << fullGyroScaleRangeToString(profile->fullScaleRange()));
      } else {
        ROS_INFO_STREAM("unknown profile: " << sensor->type());
      }
    }
  }
  return 0;
}
