#pragma once
#include "utils.h"
#include "jpeg_decoder.h"
#include <NvJpegDecoder.h>
#include <NvUtils.h>
#include <NvV4l2Element.h>
#include <NvJpegDecoder.h>
#include <nvbuf_utils.h>
#include <NvV4l2Element.h>
#include <NvVideoConverter.h>

namespace orbbec_camera {
class JetsonNvJPEGDecoder : public JPEGDecoder {
 public:
  JetsonNvJPEGDecoder(int width, int height);
  ~JetsonNvJPEGDecoder() override;

  bool decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) override;

 private:
  int width_;
  int height_;
  NvJPEGDecoder* decoder_;
  NvBuffer* converted_buffer_ = nullptr;
  NvBuffer* output_buffer_ = nullptr;
  NvVideoConverter* converter_ = nullptr;
  int dmabuff_fd_ = -1;
};
}  // namespace orbbec_camera