#include "orbbec_camera/jetson_nv_decoder.h"
#include <ros/ros.h>
#include <NvJpegDecoder.h>
#include <NvV4l2Element.h>
#include <algorithm>
#include "orbbec_camera/yuv2rgb.h"
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <NvBufSurface.h>
#include <fstream>
namespace orbbec_camera {

JetsonNvJPEGDecoder::JetsonNvJPEGDecoder(int width, int height) : JPEGDecoder(width, height) {
  yuv422InitMemory(width, height);
}

JetsonNvJPEGDecoder::~JetsonNvJPEGDecoder() {
  yuv422FreeMemory();
}

bool JetsonNvJPEGDecoder::decode(const std::shared_ptr<ob::ColorFrame> &frame, uint8_t *dest) {
  if (!isValidJPEG(frame)) {
    ROS_ERROR_STREAM("Invalid JPEG");
    return false;
  }
  uint32_t pixfmt = 0;
  auto *data = static_cast<uint8_t *>(frame->data());
  uint32_t width = 0;
  uint32_t height = 0;
  auto data_size = frame->dataSize();
  while (data[data_size - 1] == 0) {
    data_size--;
  }

  NvBuffer *buffer = nullptr;
  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
  if (!decoder_) {
    ROS_ERROR_STREAM("Failed to create JPEG decoder");
  }
  int ret = decoder_->decodeToBuffer(&buffer, data, data_size, &pixfmt, &width, &height);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to decode JPEG");
    return false;
  }
  if (pixfmt != V4L2_PIX_FMT_YUV422M) {
    ROS_ERROR_STREAM("Unexpected pixfmt: " << pixfmt);
    return false;
  }
  if (width != width_ || height != height_) {
    ROS_ERROR_STREAM("Unexpected width/height: " << width << "x" << height);
    return false;
  }
  uint8_t *y_plane = buffer->planes[0].data;
  uint8_t *v_plane = buffer->planes[1].data;
  uint8_t *u_plane = buffer->planes[2].data;
  yuv422ToRgb(y_plane, u_plane, v_plane, dest, width, height);
  delete buffer;
  delete decoder_;

  return true;
}
}  // namespace orbbec_camera