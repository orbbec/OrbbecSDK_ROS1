#pragma once
#include "constants.h"
#include "utils.h"
#include "types.h"
#include <glog/logging.h>

namespace orbbec_camera {
class ROSOBSensor {
 public:
  explicit ROSOBSensor(std::shared_ptr<ob::Device> device, std::shared_ptr<ob::Sensor> sensor,
                           std::string name);

  ~ROSOBSensor();

  void startStream(std::shared_ptr<ob::StreamProfile> profile, ob::FrameCallback callback);

  void stopStream();

  int getExposure();

  void setExposure(int data);

  int getGain();

  void setGain(int data);

  int getWhiteBalance();

  void setWhiteBalance(int data);

  bool getAutoWhiteBalance();

  void setAutoWhiteBalance(bool data);

  void setAutoExposure(bool data);

  bool getAutoExposure();

  OBSensorType getSensorType();

  void setMirror(bool data);

  bool isMirrored() const;

  std::shared_ptr<ob::StreamProfileList> getStreamProfileList() const;

 private:
  std::shared_ptr<ob::Device> device_ = nullptr;
  std::shared_ptr<ob::Sensor> sensor_ = nullptr;
  std::shared_ptr<ob::StreamProfile> profile_ = nullptr;
  std::string name_;
  bool is_started_ = false;
  bool is_mirrored_ = false;
};
}  // namespace orbbec_camera
