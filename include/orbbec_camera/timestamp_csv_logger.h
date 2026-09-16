#pragma once

#include "types.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

namespace orbbec_camera {

class FrameTimestampCsvLogger;
class ImuTimestampCsvLogger;

class TimestampCsvLogger {
 public:
  struct Config {
    bool frame_drop_log_enabled = false;
    std::string csv_file_path;
    bool frame_sync_enabled = false;
    bool color_enabled = false;
    bool left_color_enabled = false;
    bool right_color_enabled = false;
    bool depth_enabled = false;
    bool left_ir_enabled = false;
    bool right_ir_enabled = false;
    bool imu_sync_enabled = false;
    bool accel_enabled = false;
    bool gyro_enabled = false;
  };

  explicit TimestampCsvLogger(Config config);
  ~TimestampCsvLogger() noexcept;

  TimestampCsvLogger(const TimestampCsvLogger &) = delete;
  TimestampCsvLogger &operator=(const TimestampCsvLogger &) = delete;

  bool enabled() const;
  bool imageEnabled() const;
  bool imageStreamEnabled(const stream_index_pair &stream_index) const;
  bool syncedImuEnabled() const;
  bool standaloneImuEnabled(const stream_index_pair &stream_index) const;

  void recordImageFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                           const std::shared_ptr<ob::Frame> &depth_frame, int64_t arrival_system_us,
                           int64_t arrival_steady_us, bool track_color, bool track_depth,
                           bool color_image_publish_expected, bool depth_image_publish_expected);
  void recordStandaloneImageArrival(const stream_index_pair &stream_index,
                                    const std::shared_ptr<ob::Frame> &frame,
                                    int64_t arrival_system_us, int64_t arrival_steady_us,
                                    bool image_publish_expected);
  void recordImageFrameArrival(const stream_index_pair &stream_index,
                               const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
                               int64_t arrival_steady_us, bool image_publish_expected);
  void recordImagePrePublish(const stream_index_pair &stream_index,
                             const std::shared_ptr<ob::Frame> &frame, int64_t publish_system_us,
                             int64_t publish_steady_us);
  void recordImagePublishSkipped(const stream_index_pair &stream_index,
                                 const std::shared_ptr<ob::Frame> &frame);

  void recordSyncedImu(const std::shared_ptr<ob::Frame> &accel_frame,
                       const std::shared_ptr<ob::Frame> &gyro_frame, int64_t arrival_system_us,
                       std::optional<int64_t> publish_system_us);
  void recordStandaloneImu(const stream_index_pair &stream_index,
                           const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
                           std::optional<int64_t> publish_system_us);

  void shutdown() noexcept;

 private:
  FrameTimestampCsvLogger *imageLoggerForStream(const stream_index_pair &stream_index) const;
  ImuTimestampCsvLogger *standaloneImuLoggerForStream(const stream_index_pair &stream_index) const;

  std::atomic_bool shutdown_requested_{false};
  std::unique_ptr<FrameTimestampCsvLogger> synced_image_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> color_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> left_color_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> right_color_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> depth_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> left_ir_logger_;
  std::unique_ptr<FrameTimestampCsvLogger> right_ir_logger_;
  std::unique_ptr<ImuTimestampCsvLogger> synced_imu_logger_;
  std::unique_ptr<ImuTimestampCsvLogger> accel_logger_;
  std::unique_ptr<ImuTimestampCsvLogger> gyro_logger_;
};

}  // namespace orbbec_camera
