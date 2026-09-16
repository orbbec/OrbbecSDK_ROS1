#include "orbbec_camera/frame_timestamp_csv_logger.h"
#include "ros/ros.h"
#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <utility>

namespace orbbec_camera {
namespace {

constexpr size_t kCompletedQueueSoftLimit = 1000;
constexpr size_t kFlushBatchSize = 100;
// The header occupies the first row, leaving 1,024,575 rows for frame data.
constexpr uint64_t kMaxCsvRowsPerFileIncludingHeader = 1'024'576;
constexpr auto kFlushInterval = std::chrono::seconds(1);

int64_t getExpectedIntervalUs(const std::shared_ptr<ob::Frame> &frame) {
  if (!frame || !frame->is<ob::VideoFrame>()) {
    return 0;
  }
  auto stream_profile = frame->getStreamProfile();
  if (!stream_profile || !stream_profile->is<ob::VideoStreamProfile>()) {
    return 0;
  }
  auto video_stream_profile = stream_profile->as<ob::VideoStreamProfile>();
  if (!video_stream_profile) {
    return 0;
  }
  const auto fps = video_stream_profile->getFps();
  if (fps == 0) {
    return 0;
  }
  return static_cast<int64_t>(1000000.0 / static_cast<double>(fps));
}

std::optional<FrameTimestampCsvLogger::OutputMode> outputModeForStream(
    const stream_index_pair &stream_index) {
  using OutputMode = FrameTimestampCsvLogger::OutputMode;
  if (stream_index == COLOR) {
    return OutputMode::COLOR;
  }
  if (stream_index == COLOR_LEFT) {
    return OutputMode::LEFT_COLOR;
  }
  if (stream_index == COLOR_RIGHT) {
    return OutputMode::RIGHT_COLOR;
  }
  if (stream_index == DEPTH) {
    return OutputMode::DEPTH;
  }
  if (stream_index == INFRA1) {
    return OutputMode::LEFT_IR;
  }
  if (stream_index == INFRA2) {
    return OutputMode::RIGHT_IR;
  }
  return std::nullopt;
}

const char *outputModeName(FrameTimestampCsvLogger::OutputMode output_mode) {
  using OutputMode = FrameTimestampCsvLogger::OutputMode;
  switch (output_mode) {
    case OutputMode::COLOR:
      return "color";
    case OutputMode::LEFT_COLOR:
      return "left_color";
    case OutputMode::RIGHT_COLOR:
      return "right_color";
    case OutputMode::DEPTH:
      return "depth";
    case OutputMode::LEFT_IR:
      return "left_ir";
    case OutputMode::RIGHT_IR:
      return "right_ir";
    case OutputMode::SYNCED:
      return "synced";
  }
  return "unknown";
}

}  // namespace

FrameTimestampCsvLogger::FrameTimestampCsvLogger(bool drop_log_enabled,
                                                 const std::string &csv_file_path)
    : FrameTimestampCsvLogger(drop_log_enabled, csv_file_path, OutputMode::SYNCED) {}

FrameTimestampCsvLogger::FrameTimestampCsvLogger(bool drop_log_enabled,
                                                 const std::string &csv_file_path,
                                                 OutputMode output_mode)
    : enabled_(drop_log_enabled || !csv_file_path.empty()),
      csv_enabled_(!csv_file_path.empty()),
      drop_log_enabled_(drop_log_enabled),
      csv_file_path_(csv_file_path),
      output_mode_(output_mode) {
  if (!enabled_) {
    return;
  }

  if (csv_enabled_) {
    try {
      auto path = boost::filesystem::path(csvFilePathForIndex(0));
      if (path.has_parent_path() && !boost::filesystem::exists(path.parent_path())) {
        boost::filesystem::create_directories(path.parent_path());
      }
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Failed to prepare frame timestamp CSV path " << csvFilePathForIndex(0)
                                                                     << ": " << e.what());
      csv_enabled_ = false;
      csv_writer_failed_ = true;
    }
  }

  if (csv_enabled_) {
    if (!openCsvFile(0)) {
      csv_enabled_ = false;
      csv_writer_failed_ = true;
    }
  }

  enabled_ = csv_enabled_ || drop_log_enabled_;
  if (csv_enabled_) {
    writer_thread_ = std::thread([this]() { writerThreadMain(); });
  }

  if (enabled_) {
    ROS_INFO_STREAM("Frame timestamp logger enabled: csv_file="
                    << (csv_enabled_ ? csvFilePathForIndex(0) : "disabled")
                    << " frame_drop_log=" << (drop_log_enabled_ ? "enabled" : "disabled"));
  }
}

FrameTimestampCsvLogger::~FrameTimestampCsvLogger() noexcept { shutdown(); }

void FrameTimestampCsvLogger::recordFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                                             const std::shared_ptr<ob::Frame> &depth_frame,
                                             int64_t arrival_system_us, int64_t arrival_steady_us,
                                             bool track_color, bool track_depth,
                                             bool color_image_publish_expected,
                                             bool depth_image_publish_expected) {
  if (!enabled_) {
    return;
  }
  if (output_mode_ != OutputMode::SYNCED) {
    return;
  }
  recordFrameSetInternal(color_frame, depth_frame, arrival_system_us, arrival_steady_us,
                         track_color, track_depth, color_image_publish_expected,
                         depth_image_publish_expected);
}

void FrameTimestampCsvLogger::recordStandaloneFrameArrival(const stream_index_pair &stream_index,
                                                           const std::shared_ptr<ob::Frame> &frame,
                                                           int64_t arrival_system_us,
                                                           int64_t arrival_steady_us,
                                                           bool image_publish_expected) {
  const auto expected_output_mode = outputModeForStream(stream_index);
  if (!enabled_ || !frame || !expected_output_mode || output_mode_ != *expected_output_mode) {
    return;
  }
  recordStandaloneFrameArrivalInternal(stream_index, frame, arrival_system_us, arrival_steady_us,
                                       image_publish_expected);
}

void FrameTimestampCsvLogger::recordPreImagePublish(const stream_index_pair &stream_index,
                                                    const std::shared_ptr<ob::Frame> &frame,
                                                    int64_t publish_system_us,
                                                    int64_t publish_steady_us) {
  if (!enabled_ || !frame || !isTrackedStream(stream_index)) {
    return;
  }
  completeImagePublishInternal(stream_index, frame, publish_system_us, publish_steady_us);
}

void FrameTimestampCsvLogger::recordImagePublishSkipped(const stream_index_pair &stream_index,
                                                        const std::shared_ptr<ob::Frame> &frame) {
  if (!enabled_ || !frame || !isTrackedStream(stream_index)) {
    return;
  }
  completeImagePublishInternal(stream_index, frame, std::nullopt, std::nullopt);
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
  return stream_index == DEPTH ? TrackedStream::DEPTH : TrackedStream::COLOR;
}

bool FrameTimestampCsvLogger::isTrackedStream(const stream_index_pair &stream_index) const {
  return outputModeForStream(stream_index).has_value();
}

void FrameTimestampCsvLogger::recordFrameSetInternal(
    const std::shared_ptr<ob::Frame> &color_frame, const std::shared_ptr<ob::Frame> &depth_frame,
    int64_t arrival_system_us, int64_t arrival_steady_us, bool track_color, bool track_depth,
    bool color_image_publish_expected, bool depth_image_publish_expected) {
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

void FrameTimestampCsvLogger::completeImagePublishInternal(
    const stream_index_pair &stream_index, const std::shared_ptr<ob::Frame> &frame,
    std::optional<int64_t> publish_system_us, std::optional<int64_t> publish_steady_us) {
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
      if (publish_system_us.has_value()) {
        ROS_WARN_STREAM_THROTTLE(
            5.0, "Frame timestamp CSV logger missed row mapping for stream "
                     << (output_mode_ == OutputMode::SYNCED
                             ? (tracked_stream == TrackedStream::COLOR ? "color" : "depth")
                             : outputModeName(output_mode_))
                     << " frame index " << frame_index);
      }
      return;
    }
    const auto row_id = row_id_it->second;

    auto pending_it = pending_rows_.find(row_id);
    if (pending_it == pending_rows_.end()) {
      return;
    }

    auto &state = tracked_stream == TrackedStream::COLOR ? pending_it->second.color
                                                         : pending_it->second.depth;
    if (state.final) {
      return;
    }
    if (publish_system_us.has_value() && publish_steady_us.has_value()) {
      populatePublishData(state, tracked_stream, publish_system_us.value(),
                          publish_steady_us.value());
      state.final = true;
    } else {
      finalizeStreamWithoutPublish(state);
    }

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
  if (previous.expected_interval_us <= 0) {
    previous.expected_interval_us = getExpectedIntervalUs(frame);
  }
  state.expected_interval_us = previous.expected_interval_us;
  if (previous.device_ts_us.has_value() && state.expected_interval_us > 0) {
    const auto device_ts_delta_us = state.device_ts_us - previous.device_ts_us.value();
    if (device_ts_delta_us > state.expected_interval_us * 3 / 2) {
      const auto lost_frames =
          std::max<int64_t>(1, device_ts_delta_us / state.expected_interval_us - 1);
      if (drop_log_enabled_) {
        previous.dropped_frames += lost_frames;
        ROS_WARN_STREAM("Frame drop detected: stage=SDK_RECEIVE"
                        << " stream="
                        << (output_mode_ == OutputMode::SYNCED
                                ? (stream == TrackedStream::COLOR ? "color" : "depth")
                                : outputModeName(output_mode_))
                        << " frame_index=" << state.frame_index
                        << " dropped=" << previous.dropped_frames);
      }
    }
  }
  if (frame->hasMetadata(OB_FRAME_METADATA_TYPE_FRAME_NUMBER)) {
    state.metadata_frame_number =
        static_cast<int64_t>(frame->getMetadataValue(OB_FRAME_METADATA_TYPE_FRAME_NUMBER));
  } else {
    state.metadata_frame_number.reset();
  }
  if (frame->hasMetadata(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP)) {
    state.sensor_ts_us =
        static_cast<int64_t>(frame->getMetadataValue(OB_FRAME_METADATA_TYPE_SENSOR_TIMESTAMP));
  } else {
    state.sensor_ts_us.reset();
  }
  state.global_ts_us = static_cast<int64_t>(frame->globalTimeStampUs());
  state.sdk_system_ts_us = static_cast<int64_t>(frame->systemTimeStampUs());
  state.arrival_system_us = arrival_system_us;
  state.arrival_steady_us = arrival_steady_us;
  state.device_ts_delta_us = updateDelta(previous.device_ts_us, state.device_ts_us);
  if (state.sensor_ts_us.has_value()) {
    state.sensor_ts_delta_us = updateDelta(previous.sensor_ts_us, state.sensor_ts_us.value());
  } else {
    state.sensor_ts_delta_us.reset();
    previous.sensor_ts_us.reset();
  }
  state.global_ts_delta_us = updateDelta(previous.global_ts_us, state.global_ts_us);
  state.sdk_system_ts_delta_us = updateDelta(previous.sdk_system_ts_us, state.sdk_system_ts_us);
  previous.arrival_system_us = state.arrival_system_us;
  state.arrival_steady_delta_us = updateDelta(previous.arrival_steady_us, state.arrival_steady_us);
  state.sdk_delay_from_global_us = state.arrival_system_us - state.global_ts_us;
  state.sdk_delay_from_system_us = state.arrival_system_us - state.sdk_system_ts_us;
}

void FrameTimestampCsvLogger::populatePublishData(StreamState &state, TrackedStream stream,
                                                  int64_t publish_system_us,
                                                  int64_t publish_steady_us) {
  auto &previous = stream == TrackedStream::COLOR ? color_previous_ : depth_previous_;

  if (previous.publish_device_ts_us.has_value()) {
    const auto device_ts_delta_us = state.device_ts_us - previous.publish_device_ts_us.value();
    if (state.expected_interval_us > 0 && device_ts_delta_us > state.expected_interval_us * 3 / 2) {
      const auto lost_frames =
          std::max<int64_t>(1, device_ts_delta_us / state.expected_interval_us - 1);
      if (drop_log_enabled_) {
        previous.publish_dropped_frames += lost_frames;
        ROS_WARN_STREAM("Frame drop detected: stage=ROS_PUBLISH"
                        << " stream="
                        << (output_mode_ == OutputMode::SYNCED
                                ? (stream == TrackedStream::COLOR ? "color" : "depth")
                                : outputModeName(output_mode_))
                        << " frame_index=" << state.frame_index
                        << " dropped=" << previous.publish_dropped_frames);
      }
    }
  }
  previous.publish_device_ts_us = state.device_ts_us;

  state.publish_system_us = publish_system_us;
  state.publish_steady_us = publish_steady_us;
  previous.publish_system_us = state.publish_system_us.value();
  state.publish_steady_delta_us =
      updateDelta(previous.publish_steady_us, state.publish_steady_us.value());
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

void FrameTimestampCsvLogger::finalizeStreamWithoutPublish(StreamState &state) {
  state.final = true;
}

bool FrameTimestampCsvLogger::isRowReady(const PendingRow &row) const {
  return row.color.final && row.depth.final;
}

void FrameTimestampCsvLogger::enqueueCompletedRow(const PendingRow &row) {
  if (!csv_enabled_ || csv_writer_failed_) {
    return;
  }
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
  std::stable_sort(rows.begin(), rows.end(),
                   [](const auto &lhs, const auto &rhs) { return lhs.row_id < rhs.row_id; });
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
  if (output_mode_ == OutputMode::DEPTH) {
    return serializeStreamColumns(row.depth);
  }
  if (output_mode_ != OutputMode::SYNCED) {
    return serializeStreamColumns(row.color);
  }
  std::ostringstream ss;
  ss << serializeStreamColumns(row.color) << "," << serializeStreamColumns(row.depth);
  return ss.str();
}

std::string FrameTimestampCsvLogger::serializeStreamColumns(const StreamState &state) const {
  std::vector<std::string> fields(15, "");
  if (state.has_frame) {
    fields[0] = std::to_string(state.frame_index);
    fields[1] = formatOptionalIntColumn(state.metadata_frame_number);
    if (state.sensor_ts_us.has_value()) {
      fields[2] = formatSecondsColumn(state.sensor_ts_us.value());
    }
    fields[3] = formatOptionalIntColumn(state.sensor_ts_delta_us);
    fields[4] = formatSecondsColumn(state.device_ts_us);
    fields[5] = formatOptionalIntColumn(state.device_ts_delta_us);
    fields[6] = formatSecondsColumn(state.global_ts_us);
    fields[7] = formatOptionalIntColumn(state.global_ts_delta_us);
    fields[8] = formatSecondsColumn(state.sdk_system_ts_us);
    fields[9] = formatOptionalIntColumn(state.sdk_system_ts_delta_us);
    fields[10] = formatOptionalIntColumn(state.arrival_steady_delta_us);
    fields[11] = formatOptionalIntColumn(state.publish_steady_delta_us);
    fields[12] = formatOptionalIntColumn(state.arrival_to_publish_steady_us);
    fields[13] = formatOptionalIntColumn(state.sdk_delay_from_global_us);
    fields[14] = formatOptionalIntColumn(state.sdk_delay_from_system_us);
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
  ss << std::fixed << std::setprecision(6) << (static_cast<long double>(time_us) / 1000000.0L);
  return ss.str();
}

std::string FrameTimestampCsvLogger::formatOptionalIntColumn(const std::optional<int64_t> &value) {
  if (!value.has_value()) {
    return "";
  }
  return std::to_string(*value);
}

std::string FrameTimestampCsvLogger::csvHeader() const {
  std::ostringstream ss;
  const auto append_stream_header = [&ss](const char *prefix) {
    ss << prefix << "_sdk_frame_index,";
    ss << prefix << "_hardware_frame_number,";
    ss << prefix << "_sensor_ts_sec,";
    ss << prefix << "_sensor_ts_delta_us,";
    ss << prefix << "_device_ts_sec,";
    ss << prefix << "_device_ts_delta_us,";
    ss << prefix << "_global_ts_sec,";
    ss << prefix << "_global_ts_delta_us,";
    ss << prefix << "_system_ts_sec,";
    ss << prefix << "_system_ts_delta_us,";
    ss << prefix << "_arrival_steady_delta_us,";
    ss << prefix << "_publish_steady_delta_us,";
    ss << prefix << "_arrival_to_publish_steady_us,";
    ss << prefix << "_sdk_delay_from_global_us,";
    ss << prefix << "_sdk_delay_from_system_us";
  };
  if (output_mode_ == OutputMode::SYNCED) {
    append_stream_header("color");
    ss << ",";
    append_stream_header("depth");
  } else {
    append_stream_header(outputModeName(output_mode_));
  }
  return ss.str();
}

void FrameTimestampCsvLogger::writerThreadMain() {
  if (!csv_enabled_ || csv_writer_failed_) {
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

    std::stable_sort(rows_to_write.begin(), rows_to_write.end(),
                     [](const auto &lhs, const auto &rhs) { return lhs.row_id < rhs.row_id; });

    for (const auto &row : rows_to_write) {
      if (csv_rows_written_ >= kMaxCsvRowsPerFileIncludingHeader) {
        if (!rotateCsvFile()) {
          csv_writer_failed_ = true;
          break;
        }
        rows_since_flush = 0;
        last_flush = std::chrono::steady_clock::now();
      }

      if (!csv_stream_.is_open()) {
        csv_writer_failed_ = true;
        break;
      }

      csv_stream_ << serializeRow(row) << "\n";
      if (!csv_stream_) {
        ROS_ERROR_STREAM(
            "Failed to write frame timestamp CSV file: " << csvFilePathForIndex(csv_file_index_));
        csv_writer_failed_ = true;
        break;
      }
      ++csv_rows_written_;
      ++rows_since_flush;
    }

    if (csv_writer_failed_) {
      break;
    }

    const auto now = std::chrono::steady_clock::now();
    if (csv_stream_.is_open() && (rows_since_flush >= kFlushBatchSize ||
                                  now - last_flush >= kFlushInterval || shutdown_requested_)) {
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

std::string FrameTimestampCsvLogger::csvFilePathForIndex(uint64_t file_index) const {
  const boost::filesystem::path original_path(csv_file_path_);
  std::string suffix;
  if (output_mode_ != OutputMode::SYNCED) {
    suffix = "_" + std::string(outputModeName(output_mode_));
  }

  auto indexed_filename = original_path.stem().string() + suffix;
  if (file_index != 0) {
    indexed_filename += "_" + std::to_string(file_index);
  }
  indexed_filename += original_path.extension().string();
  return (original_path.parent_path() / indexed_filename).string();
}

bool FrameTimestampCsvLogger::openCsvFile(uint64_t file_index) {
  const auto file_path = csvFilePathForIndex(file_index);
  csv_stream_.clear();
  csv_stream_.open(file_path, std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    ROS_ERROR_STREAM("Failed to open frame timestamp CSV file: " << file_path);
    return false;
  }

  csv_stream_ << csvHeader() << "\n";
  csv_stream_.flush();
  if (!csv_stream_) {
    ROS_ERROR_STREAM("Failed to write frame timestamp CSV header: " << file_path);
    csv_stream_.close();
    return false;
  }

  csv_file_index_ = file_index;
  csv_rows_written_ = 1;
  return true;
}

bool FrameTimestampCsvLogger::rotateCsvFile() {
  if (csv_stream_.is_open()) {
    csv_stream_.flush();
    csv_stream_.close();
  }

  const auto next_file_index = csv_file_index_ + 1;
  if (!openCsvFile(next_file_index)) {
    return false;
  }

  ROS_INFO_STREAM("Frame timestamp CSV reached " << kMaxCsvRowsPerFileIncludingHeader
                                                 << " rows; continuing in "
                                                 << csvFilePathForIndex(csv_file_index_));
  return true;
}

}  // namespace orbbec_camera
