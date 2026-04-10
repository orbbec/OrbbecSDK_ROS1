#include "orbbec_camera/frame_timestamp_csv_logger.h"
#include "ros/ros.h"
#include <boost/filesystem.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

namespace orbbec_camera {
namespace {

constexpr size_t kCompletedQueueSoftLimit = 1000;
constexpr size_t kFlushBatchSize = 100;
constexpr auto kFlushInterval = std::chrono::seconds(1);

}  // namespace

FrameTimestampCsvLogger::FrameTimestampCsvLogger(bool enabled, const std::string &csv_file_path)
    : enabled_(enabled), csv_file_path_(csv_file_path) {
  if (!enabled_) {
    return;
  }

  try {
    auto path = boost::filesystem::path(csv_file_path_);
    if (path.has_parent_path() && !boost::filesystem::exists(path.parent_path())) {
      boost::filesystem::create_directories(path.parent_path());
    }
  } catch (const std::exception &e) {
    ROS_ERROR_STREAM("Failed to prepare frame timestamp CSV path " << csv_file_path_ << ": "
                                                                   << e.what());
    enabled_ = false;
    writer_failed_ = true;
    return;
  }

  openCsvIfNeeded();
  if (!enabled_) {
    return;
  }

  writer_thread_ = std::thread([this]() { writerThreadMain(); });
}

FrameTimestampCsvLogger::~FrameTimestampCsvLogger() noexcept { shutdown(); }

void FrameTimestampCsvLogger::recordFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                                             const std::shared_ptr<ob::Frame> &depth_frame,
                                             int64_t arrival_system_us,
                                             int64_t arrival_steady_us, bool track_color,
                                             bool track_depth,
                                             bool color_image_publish_expected,
                                             bool depth_image_publish_expected) {
  if (!enabled_ || writer_failed_) {
    return;
  }
  recordFrameSetInternal(color_frame, depth_frame, arrival_system_us, arrival_steady_us, track_color,
                         track_depth, color_image_publish_expected, depth_image_publish_expected);
}

void FrameTimestampCsvLogger::recordStandaloneFrameArrival(const stream_index_pair &stream_index,
                                                           const std::shared_ptr<ob::Frame> &frame,
                                                           int64_t arrival_system_us,
                                                           int64_t arrival_steady_us,
                                                           bool image_publish_expected) {
  if (!enabled_ || writer_failed_ || !frame || !isTrackedStream(stream_index)) {
    return;
  }
  recordStandaloneFrameArrivalInternal(stream_index, frame, arrival_system_us, arrival_steady_us,
                                       image_publish_expected);
}

void FrameTimestampCsvLogger::recordPreImagePublish(const stream_index_pair &stream_index,
                                                    const std::shared_ptr<ob::Frame> &frame,
                                                    int64_t publish_system_us,
                                                    int64_t publish_steady_us) {
  if (!enabled_ || writer_failed_ || !frame || !isTrackedStream(stream_index)) {
    return;
  }
  recordPreImagePublishInternal(stream_index, frame, publish_system_us, publish_steady_us);
}

void FrameTimestampCsvLogger::shutdown() {
  if (!enabled_) {
    return;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }
    shutdown_requested_ = true;

    std::vector<PendingRow> rows_to_flush;
    flushPendingRowsLocked(rows_to_flush);
    for (const auto &row : rows_to_flush) {
      enqueueCompletedRow(row);
    }
  }

  completed_rows_cv_.notify_all();
  if (writer_thread_.joinable()) {
    writer_thread_.join();
  }

  if (csv_stream_.is_open()) {
    csv_stream_.flush();
    csv_stream_.close();
  }
}

FrameTimestampCsvLogger::TrackedStream FrameTimestampCsvLogger::toTrackedStream(
    const stream_index_pair &stream_index) const {
  if (stream_index == COLOR) {
    return TrackedStream::COLOR;
  }
  return TrackedStream::DEPTH;
}

bool FrameTimestampCsvLogger::isTrackedStream(const stream_index_pair &stream_index) const {
  return stream_index == COLOR || stream_index == DEPTH;
}

void FrameTimestampCsvLogger::recordFrameSetInternal(const std::shared_ptr<ob::Frame> &color_frame,
                                                     const std::shared_ptr<ob::Frame> &depth_frame,
                                                     int64_t arrival_system_us,
                                                     int64_t arrival_steady_us, bool track_color,
                                                     bool track_depth,
                                                     bool color_image_publish_expected,
                                                     bool depth_image_publish_expected) {
  if (!track_color && !track_depth) {
    return;
  }

  std::vector<PendingRow> ready_rows;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    PendingRow row;
    row.row_id = next_row_id_++;
    row.pipeline_row = true;

    if (track_color && color_frame) {
      populateArrivalData(row.color, TrackedStream::COLOR, color_frame, arrival_system_us,
                          arrival_steady_us, color_image_publish_expected);
      color_frame_index_to_row_id_[row.color.frame_index] = row.row_id;
      if (!color_image_publish_expected) {
        finalizeStreamWithoutPublish(row.color);
      }
    } else {
      row.color.final = true;
    }

    if (track_depth && depth_frame) {
      populateArrivalData(row.depth, TrackedStream::DEPTH, depth_frame, arrival_system_us,
                          arrival_steady_us, depth_image_publish_expected);
      depth_frame_index_to_row_id_[row.depth.frame_index] = row.row_id;
      if (!depth_image_publish_expected) {
        finalizeStreamWithoutPublish(row.depth);
      }
    } else {
      row.depth.final = true;
    }

    pending_rows_.emplace(row.row_id, row);
    if (isRowReady(row)) {
      auto it = pending_rows_.find(row.row_id);
      if (it != pending_rows_.end()) {
        ready_rows.push_back(it->second);
        eraseFrameIndexMappingLocked(it->second);
        pending_rows_.erase(it);
      }
    }
  }

  for (const auto &ready_row : ready_rows) {
    enqueueCompletedRow(ready_row);
  }
}

void FrameTimestampCsvLogger::recordStandaloneFrameArrivalInternal(
    const stream_index_pair &stream_index, const std::shared_ptr<ob::Frame> &frame,
    int64_t arrival_system_us, int64_t arrival_steady_us, bool image_publish_expected) {
  std::optional<PendingRow> ready_row;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    PendingRow row;
    row.row_id = next_row_id_++;
    row.pipeline_row = false;

    const auto tracked_stream = toTrackedStream(stream_index);
    auto &state = tracked_stream == TrackedStream::COLOR ? row.color : row.depth;
    auto &other_state = tracked_stream == TrackedStream::COLOR ? row.depth : row.color;

    populateArrivalData(state, tracked_stream, frame, arrival_system_us, arrival_steady_us,
                        image_publish_expected);
    other_state.final = true;

    if (tracked_stream == TrackedStream::COLOR) {
      color_frame_index_to_row_id_[state.frame_index] = row.row_id;
    } else {
      depth_frame_index_to_row_id_[state.frame_index] = row.row_id;
    }

    if (!image_publish_expected) {
      finalizeStreamWithoutPublish(state);
    }

    pending_rows_.emplace(row.row_id, row);
    if (isRowReady(row)) {
      auto it = pending_rows_.find(row.row_id);
      if (it != pending_rows_.end()) {
        ready_row = it->second;
        eraseFrameIndexMappingLocked(*ready_row);
        pending_rows_.erase(it);
      }
    }
  }

  if (ready_row.has_value()) {
    enqueueCompletedRow(*ready_row);
  }
}

void FrameTimestampCsvLogger::recordPreImagePublishInternal(const stream_index_pair &stream_index,
                                                            const std::shared_ptr<ob::Frame> &frame,
                                                            int64_t publish_system_us,
                                                            int64_t publish_steady_us) {
  std::optional<PendingRow> ready_row;
  const auto frame_index = frame->index();

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }

    const auto tracked_stream = toTrackedStream(stream_index);
    auto &row_map = tracked_stream == TrackedStream::COLOR ? color_frame_index_to_row_id_
                                                           : depth_frame_index_to_row_id_;
    auto row_id_it = row_map.find(frame_index);
    if (row_id_it == row_map.end()) {
      ROS_WARN_STREAM_THROTTLE(5.0, "Frame timestamp CSV logger missed row mapping for stream "
                                        << (tracked_stream == TrackedStream::COLOR ? "color"
                                                                                  : "depth")
                                        << " frame index " << frame_index);
      return;
    }
    const auto row_id = row_id_it->second;

    auto pending_it = pending_rows_.find(row_id);
    if (pending_it == pending_rows_.end()) {
      return;
    }

    auto &state = tracked_stream == TrackedStream::COLOR ? pending_it->second.color
                                                         : pending_it->second.depth;
    populatePublishData(state, tracked_stream, publish_system_us, publish_steady_us);
    state.final = true;

    if (isRowReady(pending_it->second)) {
      ready_row = pending_it->second;
      eraseFrameIndexMappingLocked(*ready_row);
      pending_rows_.erase(pending_it);
    }
  }

  if (ready_row.has_value()) {
    enqueueCompletedRow(*ready_row);
  }
}

void FrameTimestampCsvLogger::populateArrivalData(StreamState &state, TrackedStream stream,
                                                  const std::shared_ptr<ob::Frame> &frame,
                                                  int64_t arrival_system_us,
                                                  int64_t arrival_steady_us,
                                                  bool publish_expected) {
  auto &previous = stream == TrackedStream::COLOR ? color_previous_ : depth_previous_;

  state.has_frame = true;
  state.publish_expected = publish_expected;
  state.frame_index = frame->index();
  state.device_ts_us = static_cast<int64_t>(frame->timeStampUs());
  state.global_ts_us = static_cast<int64_t>(frame->globalTimeStampUs());
  state.sdk_system_ts_us = static_cast<int64_t>(frame->systemTimeStampUs());
  state.arrival_system_us = arrival_system_us;
  state.arrival_steady_us = arrival_steady_us;
  state.device_ts_delta_us = updateDelta(previous.device_ts_us, state.device_ts_us);
  state.global_ts_delta_us = updateDelta(previous.global_ts_us, state.global_ts_us);
  state.sdk_system_ts_delta_us = updateDelta(previous.sdk_system_ts_us, state.sdk_system_ts_us);
  state.arrival_system_delta_us =
      updateDelta(previous.arrival_system_us, state.arrival_system_us);
  state.arrival_steady_delta_us =
      updateDelta(previous.arrival_steady_us, state.arrival_steady_us);
  state.sdk_delay_from_global_us = state.arrival_system_us - state.global_ts_us;
  state.sdk_delay_from_system_us = state.arrival_system_us - state.sdk_system_ts_us;
}

void FrameTimestampCsvLogger::populatePublishData(StreamState &state, TrackedStream stream,
                                                  int64_t publish_system_us,
                                                  int64_t publish_steady_us) {
  auto &previous = stream == TrackedStream::COLOR ? color_previous_ : depth_previous_;

  state.publish_system_us = publish_system_us;
  state.publish_steady_us = publish_steady_us;
  state.publish_system_delta_us =
      updateDelta(previous.publish_system_us, state.publish_system_us.value());
  state.publish_steady_delta_us =
      updateDelta(previous.publish_steady_us, state.publish_steady_us.value());
  state.arrival_to_publish_system_us = state.publish_system_us.value() - state.arrival_system_us;
  state.arrival_to_publish_steady_us = state.publish_steady_us.value() - state.arrival_steady_us;
}

std::optional<int64_t> FrameTimestampCsvLogger::updateDelta(std::optional<int64_t> &previous,
                                                            int64_t current) {
  std::optional<int64_t> delta;
  if (previous.has_value()) {
    delta = current - previous.value();
  }
  previous = current;
  return delta;
}

void FrameTimestampCsvLogger::finalizeStreamWithoutPublish(StreamState &state) { state.final = true; }

bool FrameTimestampCsvLogger::isRowReady(const PendingRow &row) const {
  return row.color.final && row.depth.final;
}

void FrameTimestampCsvLogger::enqueueCompletedRow(const PendingRow &row) {
  std::lock_guard<std::mutex> queue_lock(completed_rows_mutex_);
  completed_rows_.push_back(row);
  if (completed_rows_.size() > kCompletedQueueSoftLimit) {
    if (!queue_warning_active_) {
      ROS_WARN_STREAM("Frame timestamp CSV queue size exceeded " << kCompletedQueueSoftLimit
                                                                 << " rows");
      queue_warning_active_ = true;
    }
  } else {
    queue_warning_active_ = false;
  }
  completed_rows_cv_.notify_one();
}

void FrameTimestampCsvLogger::flushPendingRowsLocked(std::vector<PendingRow> &rows) {
  rows.reserve(rows.size() + pending_rows_.size());
  for (auto &item : pending_rows_) {
    auto row = item.second;
    row.color.final = true;
    row.depth.final = true;
    rows.push_back(std::move(row));
  }
  pending_rows_.clear();
  color_frame_index_to_row_id_.clear();
  depth_frame_index_to_row_id_.clear();
}

void FrameTimestampCsvLogger::eraseFrameIndexMappingLocked(const PendingRow &row) {
  if (row.color.has_frame) {
    color_frame_index_to_row_id_.erase(row.color.frame_index);
  }
  if (row.depth.has_frame) {
    depth_frame_index_to_row_id_.erase(row.depth.frame_index);
  }
}

std::string FrameTimestampCsvLogger::serializeRow(const PendingRow &row) const {
  std::ostringstream ss;
  ss << serializeStreamColumns(row.color) << "," << serializeStreamColumns(row.depth);
  return ss.str();
}

std::string FrameTimestampCsvLogger::serializeStreamColumns(const StreamState &state) const {
  std::vector<std::string> fields(19, "");
  if (state.has_frame) {
    fields[0] = std::to_string(state.frame_index);
    fields[1] = formatSecondsColumn(state.device_ts_us);
    fields[2] = formatOptionalIntColumn(state.device_ts_delta_us);
    fields[3] = formatSecondsColumn(state.global_ts_us);
    fields[4] = formatOptionalIntColumn(state.global_ts_delta_us);
    fields[5] = formatSecondsColumn(state.sdk_system_ts_us);
    fields[6] = formatOptionalIntColumn(state.sdk_system_ts_delta_us);
    fields[7] = formatSecondsColumn(state.arrival_system_us);
    fields[8] = formatOptionalIntColumn(state.arrival_system_delta_us);
    fields[9] = formatSecondsColumn(state.arrival_steady_us);
    fields[10] = formatOptionalIntColumn(state.arrival_steady_delta_us);
    if (state.publish_system_us.has_value()) {
      fields[11] = formatSecondsColumn(state.publish_system_us.value());
    }
    fields[12] = formatOptionalIntColumn(state.publish_system_delta_us);
    if (state.publish_steady_us.has_value()) {
      fields[13] = formatSecondsColumn(state.publish_steady_us.value());
    }
    fields[14] = formatOptionalIntColumn(state.publish_steady_delta_us);
    fields[15] = formatOptionalIntColumn(state.arrival_to_publish_system_us);
    fields[16] = formatOptionalIntColumn(state.arrival_to_publish_steady_us);
    fields[17] = formatOptionalIntColumn(state.sdk_delay_from_global_us);
    fields[18] = formatOptionalIntColumn(state.sdk_delay_from_system_us);
  }

  std::ostringstream ss;
  for (size_t i = 0; i < fields.size(); ++i) {
    if (i != 0) {
      ss << ",";
    }
    ss << fields[i];
  }
  return ss.str();
}

std::string FrameTimestampCsvLogger::formatSecondsColumn(int64_t time_us) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6)
     << (static_cast<long double>(time_us) / 1000000.0L);
  return ss.str();
}

std::string FrameTimestampCsvLogger::formatOptionalIntColumn(const std::optional<int64_t> &value) {
  if (!value.has_value()) {
    return "";
  }
  return std::to_string(*value);
}

std::string FrameTimestampCsvLogger::csvHeader() {
  std::ostringstream ss;
  for (const auto *prefix : {"color", "depth"}) {
    ss << prefix << "_frame_index,";
    ss << prefix << "_device_ts_sec,";
    ss << prefix << "_device_ts_delta_us,";
    ss << prefix << "_global_ts_sec,";
    ss << prefix << "_global_ts_delta_us,";
    ss << prefix << "_system_ts_sec,";
    ss << prefix << "_system_ts_delta_us,";
    ss << prefix << "_arrival_system_sec,";
    ss << prefix << "_arrival_system_delta_us,";
    ss << prefix << "_arrival_steady_sec,";
    ss << prefix << "_arrival_steady_delta_us,";
    ss << prefix << "_publish_system_sec,";
    ss << prefix << "_publish_system_delta_us,";
    ss << prefix << "_publish_steady_sec,";
    ss << prefix << "_publish_steady_delta_us,";
    ss << prefix << "_arrival_to_publish_system_us,";
    ss << prefix << "_arrival_to_publish_steady_us,";
    ss << prefix << "_sdk_delay_from_global_us,";
    ss << prefix << "_sdk_delay_from_system_us";
    if (std::string(prefix) == "color") {
      ss << ",";
    }
  }
  return ss.str();
}

void FrameTimestampCsvLogger::writerThreadMain() {
  if (!enabled_ || writer_failed_) {
    return;
  }

  size_t rows_since_flush = 0;
  auto last_flush = std::chrono::steady_clock::now();

  while (true) {
    std::deque<PendingRow> rows_to_write;
    {
      std::unique_lock<std::mutex> lock(completed_rows_mutex_);
      completed_rows_cv_.wait_for(lock, kFlushInterval, [this]() {
        return shutdown_requested_ || !completed_rows_.empty();
      });
      rows_to_write.swap(completed_rows_);
    }

    for (const auto &row : rows_to_write) {
      if (csv_stream_.is_open()) {
        csv_stream_ << serializeRow(row) << "\n";
        rows_since_flush++;
      }
    }

    const auto now = std::chrono::steady_clock::now();
    if (csv_stream_.is_open() &&
        (rows_since_flush >= kFlushBatchSize || now - last_flush >= kFlushInterval ||
         shutdown_requested_)) {
      csv_stream_.flush();
      rows_since_flush = 0;
      last_flush = now;
    }

    std::lock_guard<std::mutex> lock(completed_rows_mutex_);
    if (shutdown_requested_ && completed_rows_.empty()) {
      break;
    }
  }
}

void FrameTimestampCsvLogger::openCsvIfNeeded() {
  csv_stream_.open(csv_file_path_, std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    ROS_ERROR_STREAM("Failed to open frame timestamp CSV file: " << csv_file_path_);
    enabled_ = false;
    writer_failed_ = true;
    return;
  }
  csv_stream_ << csvHeader() << "\n";
  csv_stream_.flush();
  ROS_INFO_STREAM("Frame timestamp CSV logger enabled: " << csv_file_path_);
}

}  // namespace orbbec_camera
