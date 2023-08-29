#pragma once
#include "jpeg_decoder.h"
#include <gst/gst.h>
namespace orbbec_camera {

class GstreamerJPEGDecoder : public JPEGDecoder {
 public:
  GstreamerJPEGDecoder(int width, int height, std::string jpeg_decoder, std::string video_convert,
                        std::string jpeg_parse);
  ~GstreamerJPEGDecoder() override;

  bool decode(const std::shared_ptr<ob::ColorFrame>& frame, uint8_t* dest) override;

 private:
  std::string jpeg_decoder_;
  std::string video_convert_;
  std::string jpeg_parse_;
  guint buffer_size_ = 0;
  GstBufferPool* buffer_pool_ = nullptr;
  GstElement* pipeline_ = nullptr;
  GstElement* appsrc_ = nullptr;
  GstElement* jpegparse_ = nullptr;
  GstElement* jpegdec_ = nullptr;
  GstElement* videoconvert_ = nullptr;
  GstElement* appsink_ = nullptr;
};

}  // namespace orbbec_camera