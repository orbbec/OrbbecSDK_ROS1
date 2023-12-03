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
#include <ros/ros.h>
#include <NvJpegDecoder.h>
#include <NvV4l2Element.h>
#include <algorithm>
#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <NvBufSurface.h>
#include <fstream>
#include <libyuv.h>
namespace orbbec_camera {

JetsonNvJPEGDecoder::JetsonNvJPEGDecoder(int width, int height) : JPEGDecoder(width, height) {}

JetsonNvJPEGDecoder::~JetsonNvJPEGDecoder() { delete decoder_; }

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
  int fd = -1;
  decoder_ = NvJPEGDecoder::createJPEGDecoder("jpegdec");
  std::shared_ptr<int> decoder_deleter(nullptr, [&](int *) { delete decoder_; });
  decoder_->decodeToFd(fd, data, data_size, pixfmt, width, height);
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
