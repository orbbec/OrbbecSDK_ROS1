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
#include <thread>
#include <unordered_map>
#include <vector>

namespace orbbec_camera {

class FrameTimestampCsvLogger {
 public:
  FrameTimestampCsvLogger(bool drop_log_enabled, const std::string &csv_file_path);

  ~FrameTimestampCsvLogger() noexcept;

  FrameTimestampCsvLogger(const FrameTimestampCsvLogger &) = delete;
  FrameTimestampCsvLogger &operator=(const FrameTimestampCsvLogger &) = delete;

  void recordFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                      const std::shared_ptr<ob::Frame> &depth_frame, int64_t arrival_system_us,
                      int64_t arrival_steady_us, bool track_color, bool track_depth,
                      bool color_image_publish_expected, bool depth_image_publish_expected);

  void recordStandaloneFrameArrival(const stream_index_pair &stream_index,
                                    const std::shared_ptr<ob::Frame> &frame,
                                    int64_t arrival_system_us, int64_t arrival_steady_us,
                                    bool image_publish_expected);

  void recordPreImagePublish(const stream_index_pair &stream_index,
                             const std::shared_ptr<ob::Frame> &frame, int64_t publish_system_us,
                             int64_t publish_steady_us);

  void shutdown();

  bool enabled() const { return enabled_; }

 private:
  enum class TrackedStream { COLOR, DEPTH };

  struct StreamState {
    bool has_frame = false;
    bool publish_expected = false;
    bool final = false;
    uint64_t frame_index = 0;

    std::optional<int64_t> metadata_frame_number;
    std::optional<int64_t> sensor_ts_us;
    int64_t device_ts_us = 0;
    int64_t global_ts_us = 0;
    int64_t sdk_system_ts_us = 0;
    int64_t arrival_system_us = 0;
    int64_t arrival_steady_us = 0;
    int64_t expected_interval_us = 0;
    std::optional<int64_t> publish_system_us;
    std::optional<int64_t> publish_steady_us;

    std::optional<int64_t> device_ts_delta_us;
    std::optional<int64_t> sensor_ts_delta_us;
    std::optional<int64_t> global_ts_delta_us;
    std::optional<int64_t> sdk_system_ts_delta_us;
    std::optional<int64_t> arrival_system_delta_us;
    std::optional<int64_t> arrival_steady_delta_us;
    std::optional<int64_t> publish_system_delta_us;
    std::optional<int64_t> publish_steady_delta_us;
    std::optional<int64_t> arrival_to_publish_system_us;
    std::optional<int64_t> arrival_to_publish_steady_us;
    std::optional<int64_t> sdk_delay_from_global_us;
    std::optional<int64_t> sdk_delay_from_system_us;
  };

  struct PendingRow {
    uint64_t row_id = 0;
    bool pipeline_row = false;
    StreamState color;
    StreamState depth;
  };

  struct PreviousStreamTimestamps {
    std::optional<int64_t> device_ts_us;
    std::optional<int64_t> publish_device_ts_us;
    int64_t expected_interval_us = 0;
    int64_t dropped_frames = 0;
    int64_t publish_dropped_frames = 0;
    std::optional<int64_t> sensor_ts_us;
    std::optional<int64_t> global_ts_us;
    std::optional<int64_t> sdk_system_ts_us;
    std::optional<int64_t> arrival_system_us;
    std::optional<int64_t> arrival_steady_us;
    std::optional<int64_t> publish_system_us;
    std::optional<int64_t> publish_steady_us;
  };

  TrackedStream toTrackedStream(const stream_index_pair &stream_index) const;
  bool isTrackedStream(const stream_index_pair &stream_index) const;

  void recordFrameSetInternal(const std::shared_ptr<ob::Frame> &color_frame,
                              const std::shared_ptr<ob::Frame> &depth_frame,
                              int64_t arrival_system_us, int64_t arrival_steady_us,
                              bool track_color, bool track_depth, bool color_image_publish_expected,
                              bool depth_image_publish_expected);
  void recordStandaloneFrameArrivalInternal(const stream_index_pair &stream_index,
                                            const std::shared_ptr<ob::Frame> &frame,
                                            int64_t arrival_system_us, int64_t arrival_steady_us,
                                            bool image_publish_expected);
  void recordPreImagePublishInternal(const stream_index_pair &stream_index,
                                     const std::shared_ptr<ob::Frame> &frame,
                                     int64_t publish_system_us, int64_t publish_steady_us);

  void populateArrivalData(StreamState &state, TrackedStream stream,
                           const std::shared_ptr<ob::Frame> &frame, int64_t arrival_system_us,
                           int64_t arrival_steady_us, bool publish_expected);
  void populatePublishData(StreamState &state, TrackedStream stream, int64_t publish_system_us,
                           int64_t publish_steady_us);
  std::optional<int64_t> updateDelta(std::optional<int64_t> &previous, int64_t current);

  void finalizeStreamWithoutPublish(StreamState &state);
  bool isRowReady(const PendingRow &row) const;
  void enqueueCompletedRow(const PendingRow &row);
  void flushPendingRowsLocked(std::vector<PendingRow> &rows);
  void eraseFrameIndexMappingLocked(const PendingRow &row);

  std::string serializeRow(const PendingRow &row) const;
  std::string serializeStreamColumns(const StreamState &state) const;
  static std::string formatSecondsColumn(int64_t time_us);
  static std::string formatOptionalIntColumn(const std::optional<int64_t> &value);
  static std::string csvHeader();

  void writerThreadMain();
  void openCsvIfNeeded();

  bool enabled_ = false;
  bool csv_enabled_ = false;
  bool drop_log_enabled_ = false;
  std::atomic_bool shutdown_requested_{false};
  bool csv_writer_failed_ = false;
  bool queue_warning_active_ = false;
  std::string csv_file_path_;
  std::ofstream csv_stream_;
  std::thread writer_thread_;

  uint64_t next_row_id_ = 1;
  std::unordered_map<uint64_t, PendingRow> pending_rows_;
  std::unordered_map<uint64_t, uint64_t> color_frame_index_to_row_id_;
  std::unordered_map<uint64_t, uint64_t> depth_frame_index_to_row_id_;
  PreviousStreamTimestamps color_previous_;
  PreviousStreamTimestamps depth_previous_;

  std::deque<PendingRow> completed_rows_;

  mutable std::mutex state_mutex_;
  std::mutex completed_rows_mutex_;
  std::condition_variable completed_rows_cv_;
};

}  // namespace orbbec_camera
