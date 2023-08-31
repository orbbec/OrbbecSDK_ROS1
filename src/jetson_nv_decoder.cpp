#include "orbbec_camera/jetson_nv_decoder.h"
#include <ros/ros.h>
#include <NvJpegDecoder.h>
#include <nvbuf_utils.h>
#include <NvV4l2Element.h>
#include <NvVideoConverter.h>
#include <algorithm>
#include "orbbec_camera/yuv2rgb.h"

namespace orbbec_camera {

JetsonNvJPEGDecoder::JetsonNvJPEGDecoder(int width, int height) : JPEGDecoder(width, height) {
  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
  if (!decoder_) {
    ROS_ERROR_STREAM("Failed to create JPEG decoder");
    throw std::runtime_error("Failed to create JPEG decoder");
  }
  yuv422InitMemory(width_, height_);
}

JetsonNvJPEGDecoder::~JetsonNvJPEGDecoder() {
  delete decoder_;
  yuv422FreeMemory();
}

bool JetsonNvJPEGDecoder::decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) {
  if (!isValidJPEG(frame)) {
    ROS_ERROR_STREAM("Invalid JPEG");
    return false;
  }
  uint32_t pixfmt = 0;
  auto *data = static_cast<uint8_t *>(frame->data());
  uint32_t width = width_;
  uint32_t height = height_;
  NvBuffer *buffer = nullptr;
  int ret = decoder_->decodeToBuffer(&buffer, data, frame->dataSize(), &pixfmt, &width, &height);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to decode JPEG");
    return false;
  }
  ROS_INFO_STREAM("pixfmt: " << fourccToString(pixfmt) << " width: " << width
                             << " height: " << height);
  uint8_t *y_plane = buffer->planes[0].data;
  uint8_t *u_plane = buffer->planes[1].data;
  uint8_t *v_plane = buffer->planes[2].data;

  yuv422ToRgb(y_plane, u_plane, v_plane, rgb_buffer_, width_, height_);
  return true;
}
}  // namespace orbbec_camera