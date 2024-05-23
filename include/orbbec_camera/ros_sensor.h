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

#pragma once
#include "constants.h"
#include "utils.h"
#include "types.h"

namespace orbbec_camera {
class ROSOBSensor {
 public:
  explicit ROSOBSensor(std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor,
                       std::string name);

  ~ROSOBSensor();

  void startStream(std::shared_ptr<ob::StreamProfile> profile, ob::FrameCallback callback);

  void stopStream();

  int getExposure();

  OBIntPropertyRange getExposureRange();

  void setExposure(int data);

  int getGain();

  OBIntPropertyRange getGainRange();

  void setGain(int data);

  int getWhiteBalance();

  OBIntPropertyRange getWhiteBalanceRange();

  void setWhiteBalance(int data);

  bool getAutoWhiteBalance();

  void setAutoWhiteBalance(bool data);

  void setAutoExposure(bool data);

  bool getAutoExposure();

  OBSensorType getSensorType();

  void setMirror(bool data);

  bool isMirrored() const;

  std::shared_ptr<ob::StreamProfileList> getStreamProfileList() const;

  std::shared_ptr<ob::Sensor> getSensor() const;

 private:
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::Sensor> sensor_ = nullptr;
  std::shared_ptr<ob::StreamProfile> profile_ = nullptr;
  std::string name_;
  bool is_started_ = false;
  bool is_mirrored_ = false;
};
}  // namespace orbbec_camera
