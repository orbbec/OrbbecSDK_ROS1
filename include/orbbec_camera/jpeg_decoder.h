#pragma once

#include <string>
#include <vector>
#include "libobsensor/ObSensor.hpp"

namespace orbbec_camera {
class JPEGDecoder {
 public:
  JPEGDecoder(int width, int height);

  virtual ~JPEGDecoder();

  virtual bool decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) { return false; }

 protected:
  int width_ = 0;
  int height_ = 0;
};
}  // namespace orbbec_camera
