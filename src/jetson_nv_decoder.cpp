#include "orbbec_camera/jetson_nv_decoder.h"
#include <ros/ros.h>
#include <NvJpegDecoder.h>
#include <nvbuf_utils.h>
#include <NvV4l2Element.h>
#include <NvVideoConverter.h>

namespace orbbec_camera {



JetsonNvJPEGDecoder::JetsonNvJPEGDecoder(int width, int height) : JPEGDecoder(width, height) {
  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
  converter_ = NvVideoConverter::createVideoConverter("conv");
  if (!decoder_) {
    ROS_ERROR_STREAM("Failed to create JPEG decoder");
    throw std::runtime_error("Failed to create JPEG decoder");
  }
  if (!converter_) {
    ROS_ERROR_STREAM("Failed to create video converter");
    throw std::runtime_error("Failed to create video converter");
  }
  converter_->setOutputPlaneFormat(V4L2_PIX_FMT_RGB24, width, height, V4L2_NV_BUFFER_LAYOUT_PITCH);
  // Additional setup might be needed depending on your requirements
}

JetsonNvJPEGDecoder::~JetsonNvJPEGDecoder() {
  if (output_buffer_) {
    NvBufferDestroy(output_buffer_->planes[0].fd);
  }
  if (converted_buffer_) {
    NvBufferDestroy(converted_buffer_->planes[0].fd);
  }
  delete converter_;
  delete decoder_;
}

bool JetsonNvJPEGDecoder::decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) {
  if (!isValidJPEG(frame)) {
    ROS_ERROR_STREAM("Invalid JPEG");
    return false;
  }
  uint32_t pixfmt = 0;
  auto* data = static_cast<uint8_t*>(frame->data());
  uint32_t width = width_;
  uint32_t height = height_;
  int ret =
      decoder_->decodeToBuffer(&output_buffer_, data, frame->dataSize(), &pixfmt, &width, &height);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to decode JPEG");
    return false;
  }
  ROS_INFO_STREAM("Decoded JPEG to " << width << "x" << height << " " << pixfmt);

  // Enqueue the decoded buffer to the converter's output plane
  struct v4l2_buffer v4l2_buf {};
  struct v4l2_plane planes[MAX_PLANES];
  memset(&v4l2_buf, 0, sizeof(v4l2_buf));
  memset(planes, 0, sizeof(planes));
  v4l2_buf.index = 0;
  v4l2_buf.m.planes = planes;
  v4l2_buf.m.planes[0].m.fd = output_buffer_->planes[0].fd;
  ret = converter_->output_plane.qBuffer(v4l2_buf, NULL);
  if (ret < 0) {
    ROS_ERROR_STREAM("Failed to enqueue buffer to converter output plane");
    return false;
  }

  // Dequeue from the converter's capture plane to get the converted data
  ret = converter_->capture_plane.dqBuffer(v4l2_buf, &converted_buffer_, NULL, 10);
  if (ret < 0) {
    ROS_ERROR_STREAM("Failed to dequeue buffer from converter capture plane");
    return false;
  }

  // Copy the converted data from converted_buffer_ to dest
  // Note: You'll need to manage the memory and ensure it's synchronized correctly.
  memcpy(dest, converted_buffer_->planes[0].data, width * height * 3);  // Assuming RGB24 format

  return true;
}
}  // namespace orbbec_camera