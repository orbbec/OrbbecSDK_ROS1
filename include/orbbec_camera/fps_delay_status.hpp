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

#pragma once

#include "ros/ros.h"
#include <orbbec_camera/DeviceStatus.h>
#include <chrono>
#include <functional>
#include <limits>
#include <mutex>
namespace orbbec_camera {
class FpsDelayStatus {
 public:
  FpsDelayStatus() = default;

  void tick(u_int64_t stream_timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);

    double dt = (stream_timestamp - last_stream_timestamp_) / 1000000.0;
    double fps = (dt > 0) ? (1.0 / dt) : 0.0;

    // Convert now to milliseconds since steady_clock epoch
    auto now2 = std::chrono::system_clock::now();
    uint64_t ms_since_epoch =
        std::chrono::duration_cast<std::chrono::milliseconds>(now2.time_since_epoch()).count();
    double delay_ms =
        static_cast<double>(ms_since_epoch) - static_cast<double>(stream_timestamp / 1000.0);

    frame_count_++;
    fps_sum_ += fps;
    delay_sum_ += delay_ms;

    last_fps_ = fps;
    last_delay_ms_ = delay_ms;
    last_stream_timestamp_ = stream_timestamp;

    if (fps_max_ <= 0) fps_max_ = fps;
    if (fps_min_ <= 0) fps_min_ = fps;
    fps_max_ = std::max(fps_max_, fps);
    fps_min_ = std::min(fps_min_, fps);
    if (delay_max_ <= 0) delay_max_ = delay_ms;
    if (delay_min_ <= 0) delay_min_ = delay_ms;
    delay_max_ = std::max(delay_max_, delay_ms);
    delay_min_ = std::min(delay_min_, delay_ms);
  }

  void fillColorStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.color_frame_rate_cur, msg.color_frame_rate_avg, msg.color_frame_rate_min,
               msg.color_frame_rate_max, msg.color_delay_ms_cur, msg.color_delay_ms_avg,
               msg.color_delay_ms_min, msg.color_delay_ms_max);
  }

  void fillLeftColorStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.left_color_frame_rate_cur, msg.left_color_frame_rate_avg,
               msg.left_color_frame_rate_min, msg.left_color_frame_rate_max,
               msg.left_color_delay_ms_cur, msg.left_color_delay_ms_avg,
               msg.left_color_delay_ms_min, msg.left_color_delay_ms_max);
  }

  void fillRightColorStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.right_color_frame_rate_cur, msg.right_color_frame_rate_avg,
               msg.right_color_frame_rate_min, msg.right_color_frame_rate_max,
               msg.right_color_delay_ms_cur, msg.right_color_delay_ms_avg,
               msg.right_color_delay_ms_min, msg.right_color_delay_ms_max);
  }

  void fillDepthStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.depth_frame_rate_cur, msg.depth_frame_rate_avg, msg.depth_frame_rate_min,
               msg.depth_frame_rate_max, msg.depth_delay_ms_cur, msg.depth_delay_ms_avg,
               msg.depth_delay_ms_min, msg.depth_delay_ms_max);
  }

  void fillLeftIrStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.left_ir_frame_rate_cur, msg.left_ir_frame_rate_avg, msg.left_ir_frame_rate_min,
               msg.left_ir_frame_rate_max, msg.left_ir_delay_ms_cur, msg.left_ir_delay_ms_avg,
               msg.left_ir_delay_ms_min, msg.left_ir_delay_ms_max);
  }

  void fillRightIrStatus(orbbec_camera::DeviceStatus &msg) {
    fillStatus(msg.right_ir_frame_rate_cur, msg.right_ir_frame_rate_avg,
               msg.right_ir_frame_rate_min, msg.right_ir_frame_rate_max, msg.right_ir_delay_ms_cur,
               msg.right_ir_delay_ms_avg, msg.right_ir_delay_ms_min, msg.right_ir_delay_ms_max);
  }

 private:
  void fillStatus(double &frame_rate_cur, double &frame_rate_avg, double &frame_rate_min,
                  double &frame_rate_max, double &delay_ms_cur, double &delay_ms_avg,
                  double &delay_ms_min, double &delay_ms_max) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame_rate_cur = last_fps_;
    frame_rate_avg = frame_count_ > 0 ? fps_sum_ / frame_count_ : 0;
    frame_rate_min = frame_count_ > 0 ? fps_min_ : 0;
    frame_rate_max = frame_count_ > 0 ? fps_max_ : 0;

    delay_ms_cur = last_delay_ms_;
    delay_ms_avg = frame_count_ > 0 ? delay_sum_ / frame_count_ : 0;
    delay_ms_min = frame_count_ > 0 ? delay_min_ : 0;
    delay_ms_max = frame_count_ > 0 ? delay_max_ : 0;

    last_delay_ms_ = 0.0;
    last_fps_ = 0.0;
    frame_count_ = 0;
    fps_sum_ = delay_sum_ = 0.0;
    fps_max_ = delay_max_ = 0.0;
    fps_min_ = delay_min_ = 0.0;
  }

  mutable std::mutex mutex_;
  u_int64_t last_stream_timestamp_{0};
  double last_delay_ms_{0.0};
  double last_fps_{0.0};

  int frame_count_{0};
  double fps_sum_{0.0};
  double delay_sum_{0.0};
  double fps_max_{std::numeric_limits<double>::lowest()};
  double fps_min_{std::numeric_limits<double>::max()};
  double delay_max_{std::numeric_limits<double>::lowest()};
  double delay_min_{std::numeric_limits<double>::max()};
};

}  // namespace orbbec_camera
