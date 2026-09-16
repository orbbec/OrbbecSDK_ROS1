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

#include "orbbec_camera/jetson_nv_decoder.h"

#include <linux/videodev2.h>
#include <setjmp.h>
#include <unistd.h>

#include <NvBufSurface.h>
#include <cstring>
#include <jpeglib.h>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <libyuv.h>
#include <ros/ros.h>

#include "jpegint.h"
#include "orbbec_camera/utils.h"

namespace orbbec_camera {
namespace {

struct JpegErrorManager {
  jpeg_error_mgr base;
  jmp_buf jump_buffer;
  char message[JMSG_LENGTH_MAX];
};

void jpegErrorExit(j_common_ptr cinfo) {
  auto *error = reinterpret_cast<JpegErrorManager *>(cinfo->err);
  (*cinfo->err->format_message)(cinfo, error->message);
  longjmp(error->jump_buffer, 1);
}

}  // namespace

class JetsonNvJPEGDecoder::Impl {
 public:
  Impl() {
    std::memset(&cinfo_, 0, sizeof(cinfo_));
    std::memset(&error_, 0, sizeof(error_));
    cinfo_.err = jpeg_std_error(&error_.base);
    error_.base.error_exit = jpegErrorExit;

    if (setjmp(error_.jump_buffer) != 0) {
      return;
    }

    jpeg_create_decompress(&cinfo_);
    initialized_ = true;
    cinfo_.mjpeg_decode = TRUE;
  }

  ~Impl() {
    if (initialized_) {
      jpeg_destroy_decompress(&cinfo_);
    }
  }

  Impl(const Impl &) = delete;
  Impl &operator=(const Impl &) = delete;

  bool isInitialized() const { return initialized_; }

  const char *lastError() const { return error_.message; }

  int decodeToFd(int &fd, unsigned char *input, unsigned long input_size, uint32_t &pixfmt,
                 uint32_t &width, uint32_t &height) {
    if (!initialized_ || input == nullptr || input_size == 0) {
      return -1;
    }

    error_.message[0] = '\0';
    if (setjmp(error_.jump_buffer) != 0) {
      return -1;
    }

    NvBufSurface surface;
    cinfo_.out_color_space = JCS_YCbCr;
    jpeg_mem_src(&cinfo_, input, input_size);

    (void)jpeg_read_header(&cinfo_, TRUE);

    cinfo_.out_color_space = JCS_YCbCr;
    cinfo_.IsVendorbuf = TRUE;
    cinfo_.pVendor_buf = reinterpret_cast<unsigned char *>(&surface);

    uint32_t pixel_format = 0;
    if (cinfo_.comp_info[0].h_samp_factor == 2) {
      pixel_format =
          cinfo_.comp_info[0].v_samp_factor == 2 ? V4L2_PIX_FMT_YUV420M : V4L2_PIX_FMT_YUV422M;
    } else {
      pixel_format =
          cinfo_.comp_info[0].v_samp_factor == 1 ? V4L2_PIX_FMT_YUV444M : V4L2_PIX_FMT_YUV422RM;
    }

    jpeg_start_decompress(&cinfo_);
    if (cinfo_.global_state != DSTATE_READY) {
      return -1;
    }

    jpeg_read_raw_data(&cinfo_, nullptr, cinfo_.comp_info[0].v_samp_factor * DCTSIZE);
    jpeg_finish_decompress(&cinfo_);

    width = cinfo_.image_width % 2 == 1 ? cinfo_.image_width + 1 : cinfo_.image_width;
    height = cinfo_.image_height % 2 == 1 ? cinfo_.image_height + 1 : cinfo_.image_height;
    pixfmt = pixel_format;
    fd = cinfo_.fd;
    return 0;
  }

 private:
  jpeg_decompress_struct cinfo_{};
  JpegErrorManager error_{};
  bool initialized_ = false;
};

JetsonNvJPEGDecoder::JetsonNvJPEGDecoder(int width, int height)
    : JPEGDecoder(width, height), decoder_(std::make_unique<Impl>()) {}

JetsonNvJPEGDecoder::~JetsonNvJPEGDecoder() = default;

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
  while (data_size > 4 && data[data_size - 1] == 0) {
    data_size--;
  }

  if (!decoder_ || !decoder_->isInitialized()) {
    decoder_ = std::make_unique<Impl>();
  }
  if (!decoder_->isInitialized()) {
    ROS_ERROR_STREAM("Failed to initialize NVIDIA JPEG decoder");
    decoder_.reset();
    return false;
  }

  int fd = -1;
  if (decoder_->decodeToFd(fd, data, data_size, pixfmt, width, height) != 0) {
    ROS_ERROR_STREAM("Failed to decode JPEG frame");
    decoder_.reset();
    return false;
  }

  if (pixfmt != V4L2_PIX_FMT_YUV422M) {
    ROS_ERROR_STREAM("Unexpected pixfmt: " << pixfmt);
    if (fd != -1) {
      close(fd);
    }
    return false;
  }
  if (width != width_ || height != height_) {
    ROS_ERROR_STREAM("Unexpected width/height: " << width << "x" << height);
    return false;
  }
  NvBufSurf::NvCommonAllocateParams nvbufParams = {0};

  nvbufParams.memType = NVBUF_MEM_SURFACE_ARRAY;
  nvbufParams.width = width;
  nvbufParams.height = height;
  nvbufParams.layout = NVBUF_LAYOUT_PITCH;
  nvbufParams.colorFormat = NVBUF_COLOR_FORMAT_RGBA;
  int rgba_fd = -1;
  int ret = NvBufSurf::NvAllocate(&nvbufParams, 1, &rgba_fd);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to allocate buffer");
    return false;
  }
  NvBufSurf::NvCommonTransformParams transform_params;
  transform_params.src_top = 0;
  transform_params.src_left = 0;
  transform_params.src_width = width;
  transform_params.src_height = height;
  transform_params.dst_top = 0;
  transform_params.dst_left = 0;
  transform_params.dst_width = width;
  transform_params.dst_height = height;
  transform_params.flag = NVBUFSURF_TRANSFORM_FILTER;
  transform_params.flip = NvBufSurfTransform_None;
  transform_params.filter = NvBufSurfTransformInter_Nearest;
  ret = NvBufSurf::NvTransform(&transform_params, fd, rgba_fd);
  if (ret != 0) {
    ROS_ERROR_STREAM("Failed to transform buffer");
    if (rgba_fd != -1) {
      NvBufSurf::NvDestroy(rgba_fd);
    }
    return false;
  }
  NvBufSurface *nvbuf_surf = 0;
  NvBufSurfaceFromFd(rgba_fd, (void **)&nvbuf_surf);
  ret = NvBufSurfaceMap(nvbuf_surf, 0, 0, NVBUF_MAP_READ_WRITE);
  if (ret < 0) {
    printf("NvBufSurfaceMap failed\n");
    return false;
  }
  NvBufSurfaceSyncForCpu(nvbuf_surf, 0, 0);
  uint8_t *rgba = (uint8_t *)nvbuf_surf->surfaceList[0].mappedAddr.addr[0];
  int src_stride_argb = width * 4;
  int dst_stride_rgb24 = width * 3;

  libyuv::ARGBToRGB24(rgba, src_stride_argb, dest, dst_stride_rgb24, width, height);
  NvBufSurfaceUnMap(nvbuf_surf, 0, 0);
  if (rgba_fd != -1) {
    NvBufSurf::NvDestroy(rgba_fd);
  }
  return true;
}
}  // namespace orbbec_camera
