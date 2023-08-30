
#include <orbbec_camera/jpeg_decoder.h>

namespace orbbec_camera {
JPEGDecoder::JPEGDecoder(int width, int height) : width_(width), height_(height) {
  rgb_buffer_ = new uint8_t[width_ * height_ * 3];
}
JPEGDecoder::~JPEGDecoder() {
  if (rgb_buffer_) {
    delete[] rgb_buffer_;
    rgb_buffer_ = nullptr;
  }
}

}  // namespace orbbec_camera