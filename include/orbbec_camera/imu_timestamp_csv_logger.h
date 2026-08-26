#pragma once

#include "types.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace orbbec_camera {

class ImuTimestampCsvLogger {
 public:
  enum class OutputMode { SYNCED, ACCEL, GYRO };

  ImuTimestampCsvLogger(const std::string &frame_csv_file_path, OutputMode output_mode);

  ~ImuTimestampCsvLogger() noexcept;

  ImuTimestampCsvLogger(const ImuTimestampCsvLogger &) = delete;
  ImuTimestampCsvLogger &operator=(const ImuTimestampCsvLogger &) = delete;

  void recordFrameSet(const std::shared_ptr<ob::Frame> &accel_frame,
                      const std::shared_ptr<ob::Frame> &gyro_frame, int64_t arrival_system_us,
                      std::optional<int64_t> publish_system_us);

  void recordStandaloneFrame(const stream_index_pair &stream_index,
                             const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
                             std::optional<int64_t> publish_system_us);

  void shutdown();

  bool enabled() const { return enabled_; }

 private:
  struct StreamState {
    bool has_frame = false;
    int64_t device_ts_us = 0;
    int64_t global_ts_us = 0;
    int64_t sdk_system_ts_us = 0;
    int64_t arrival_system_us = 0;
    std::optional<int64_t> publish_system_us;
  };

  struct PendingRow {
    uint64_t row_id = 0;
    StreamState accel;
    StreamState gyro;
  };

  void recordFrames(const std::shared_ptr<ob::Frame> &accel_frame,
                    const std::shared_ptr<ob::Frame> &gyro_frame, int64_t arrival_system_us,
                    std::optional<int64_t> publish_system_us);
  static void populateStreamState(StreamState &state, const std::shared_ptr<ob::Frame> &frame,
                                  int64_t arrival_system_us,
                                  std::optional<int64_t> publish_system_us);

  void enqueueCompletedRow(const PendingRow &row);
  std::string serializeRow(const PendingRow &row) const;
  static std::string serializeStreamColumns(const StreamState &state);
  static std::string formatSecondsColumn(int64_t time_us);
  static std::string formatOptionalSecondsColumn(const std::optional<int64_t> &value);
  std::string csvHeader() const;

  void writerThreadMain();
  std::string csvFilePathForIndex(uint64_t file_index) const;
  bool openCsvFile(uint64_t file_index);
  bool rotateCsvFile();

  bool enabled_ = false;
  bool csv_enabled_ = false;
  std::atomic_bool shutdown_requested_{false};
  std::atomic_bool csv_writer_failed_{false};
  bool queue_warning_active_ = false;
  std::string frame_csv_file_path_;
  OutputMode output_mode_;
  std::ofstream csv_stream_;
  std::thread writer_thread_;
  uint64_t csv_file_index_ = 0;
  uint64_t csv_rows_written_ = 0;

  uint64_t next_row_id_ = 1;
  std::deque<PendingRow> completed_rows_;

  std::mutex state_mutex_;
  std::mutex completed_rows_mutex_;
  std::condition_variable completed_rows_cv_;
};

}  // namespace orbbec_camera
