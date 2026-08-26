#include "orbbec_camera/imu_timestamp_csv_logger.h"

#include "ros/ros.h"

#include <algorithm>
#include <boost/filesystem.hpp>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <vector>

namespace orbbec_camera {
namespace {

constexpr size_t kCompletedQueueSoftLimit = 1000;
constexpr size_t kFlushBatchSize = 100;
// The header occupies the first row, leaving 1,024,575 rows for IMU data.
constexpr uint64_t kMaxCsvRowsPerFileIncludingHeader = 1'024'576;
constexpr auto kFlushInterval = std::chrono::seconds(1);

}  // namespace

ImuTimestampCsvLogger::ImuTimestampCsvLogger(const std::string &frame_csv_file_path,
                                             OutputMode output_mode)
    : enabled_(!frame_csv_file_path.empty()),
      csv_enabled_(!frame_csv_file_path.empty()),
      frame_csv_file_path_(frame_csv_file_path),
      output_mode_(output_mode) {
  if (!enabled_) {
    return;
  }

  if (csv_enabled_) {
    try {
      const auto path = boost::filesystem::path(csvFilePathForIndex(0));
      if (path.has_parent_path() && !boost::filesystem::exists(path.parent_path())) {
        boost::filesystem::create_directories(path.parent_path());
      }
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Failed to prepare IMU timestamp CSV path " << csvFilePathForIndex(0) << ": "
                                                                   << e.what());
      csv_enabled_ = false;
      csv_writer_failed_ = true;
    }
  }

  if (csv_enabled_ && !openCsvFile(0)) {
    csv_enabled_ = false;
    csv_writer_failed_ = true;
  }

  enabled_ = csv_enabled_;
  if (csv_enabled_) {
    writer_thread_ = std::thread([this]() { writerThreadMain(); });
  }

  if (enabled_) {
    ROS_INFO_STREAM("IMU timestamp logger enabled: csv_file=" << csvFilePathForIndex(0));
  }
}

ImuTimestampCsvLogger::~ImuTimestampCsvLogger() noexcept { shutdown(); }

void ImuTimestampCsvLogger::recordFrameSet(const std::shared_ptr<ob::Frame> &accel_frame,
                                           const std::shared_ptr<ob::Frame> &gyro_frame,
                                           int64_t arrival_system_us,
                                           std::optional<int64_t> publish_system_us) {
  if (!enabled_ || output_mode_ != OutputMode::SYNCED || (!accel_frame && !gyro_frame)) {
    return;
  }
  recordFrames(accel_frame, gyro_frame, arrival_system_us, publish_system_us);
}

void ImuTimestampCsvLogger::recordStandaloneFrame(const stream_index_pair &stream_index,
                                                  const std::shared_ptr<ob::Frame> &frame,
                                                  int64_t arrival_system_us,
                                                  std::optional<int64_t> publish_system_us) {
  if (!enabled_ || !frame) {
    return;
  }
  if (stream_index == ACCEL && output_mode_ == OutputMode::ACCEL) {
    recordFrames(frame, nullptr, arrival_system_us, publish_system_us);
  } else if (stream_index == GYRO && output_mode_ == OutputMode::GYRO) {
    recordFrames(nullptr, frame, arrival_system_us, publish_system_us);
  }
}

void ImuTimestampCsvLogger::shutdown() {
  if (!enabled_) {
    return;
  }

  {
    std::lock_guard<std::mutex> state_lock(state_mutex_);
    if (shutdown_requested_) {
      return;
    }
    shutdown_requested_ = true;
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

void ImuTimestampCsvLogger::recordFrames(const std::shared_ptr<ob::Frame> &accel_frame,
                                         const std::shared_ptr<ob::Frame> &gyro_frame,
                                         int64_t arrival_system_us,
                                         std::optional<int64_t> publish_system_us) {
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (shutdown_requested_) {
    return;
  }

  PendingRow row;
  row.row_id = next_row_id_++;
  if (accel_frame) {
    populateStreamState(row.accel, accel_frame, arrival_system_us, publish_system_us);
  }
  if (gyro_frame) {
    populateStreamState(row.gyro, gyro_frame, arrival_system_us, publish_system_us);
  }
  enqueueCompletedRow(row);
}

void ImuTimestampCsvLogger::populateStreamState(StreamState &state,
                                                const std::shared_ptr<ob::Frame> &frame,
                                                int64_t arrival_system_us,
                                                std::optional<int64_t> publish_system_us) {
  state.has_frame = true;
  state.device_ts_us = static_cast<int64_t>(frame->timeStampUs());
  state.global_ts_us = static_cast<int64_t>(frame->globalTimeStampUs());
  state.sdk_system_ts_us = static_cast<int64_t>(frame->systemTimeStampUs());
  state.arrival_system_us = arrival_system_us;
  state.publish_system_us = publish_system_us;
}

void ImuTimestampCsvLogger::enqueueCompletedRow(const PendingRow &row) {
  if (!csv_enabled_ || csv_writer_failed_) {
    return;
  }

  std::lock_guard<std::mutex> queue_lock(completed_rows_mutex_);
  completed_rows_.push_back(row);
  if (completed_rows_.size() > kCompletedQueueSoftLimit) {
    if (!queue_warning_active_) {
      ROS_WARN_STREAM("IMU timestamp CSV queue size exceeded " << kCompletedQueueSoftLimit
                                                               << " rows");
      queue_warning_active_ = true;
    }
  } else {
    queue_warning_active_ = false;
  }
  completed_rows_cv_.notify_one();
}

std::string ImuTimestampCsvLogger::serializeRow(const PendingRow &row) const {
  if (output_mode_ == OutputMode::ACCEL) {
    return serializeStreamColumns(row.accel);
  }
  if (output_mode_ == OutputMode::GYRO) {
    return serializeStreamColumns(row.gyro);
  }
  std::ostringstream ss;
  ss << serializeStreamColumns(row.accel) << "," << serializeStreamColumns(row.gyro);
  return ss.str();
}

std::string ImuTimestampCsvLogger::serializeStreamColumns(const StreamState &state) {
  std::vector<std::string> fields(5, "");
  if (state.has_frame) {
    fields[0] = formatSecondsColumn(state.device_ts_us);
    fields[1] = formatSecondsColumn(state.global_ts_us);
    fields[2] = formatSecondsColumn(state.sdk_system_ts_us);
    fields[3] = formatSecondsColumn(state.arrival_system_us);
    fields[4] = formatOptionalSecondsColumn(state.publish_system_us);
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

std::string ImuTimestampCsvLogger::formatSecondsColumn(int64_t time_us) {
  std::ostringstream ss;
  ss << std::fixed << std::setprecision(6) << (static_cast<long double>(time_us) / 1000000.0L);
  return ss.str();
}

std::string ImuTimestampCsvLogger::formatOptionalSecondsColumn(
    const std::optional<int64_t> &value) {
  if (!value.has_value()) {
    return "";
  }
  return formatSecondsColumn(*value);
}

std::string ImuTimestampCsvLogger::csvHeader() const {
  std::ostringstream ss;
  const auto append_stream_header = [&ss](const char *prefix) {
    ss << prefix << "_device_ts_sec,";
    ss << prefix << "_global_ts_sec,";
    ss << prefix << "_system_ts_sec,";
    ss << prefix << "_arrival_system_ts_sec,";
    ss << prefix << "_publish_system_ts_sec";
  };
  if (output_mode_ == OutputMode::SYNCED || output_mode_ == OutputMode::ACCEL) {
    append_stream_header("accel");
  }
  if (output_mode_ == OutputMode::SYNCED) {
    ss << ",";
  }
  if (output_mode_ == OutputMode::SYNCED || output_mode_ == OutputMode::GYRO) {
    append_stream_header("gyro");
  }
  return ss.str();
}

void ImuTimestampCsvLogger::writerThreadMain() {
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
            "Failed to write IMU timestamp CSV file: " << csvFilePathForIndex(csv_file_index_));
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

std::string ImuTimestampCsvLogger::csvFilePathForIndex(uint64_t file_index) const {
  const boost::filesystem::path original_path(frame_csv_file_path_);
  std::string suffix;
  if (output_mode_ == OutputMode::SYNCED) {
    suffix = "_imu";
  } else if (output_mode_ == OutputMode::ACCEL) {
    suffix = "_accel";
  } else {
    suffix = "_gyro";
  }
  auto indexed_filename = original_path.stem().string() + suffix;
  if (file_index != 0) {
    indexed_filename += "_" + std::to_string(file_index);
  }
  indexed_filename += original_path.extension().string();
  return (original_path.parent_path() / indexed_filename).string();
}

bool ImuTimestampCsvLogger::openCsvFile(uint64_t file_index) {
  const auto file_path = csvFilePathForIndex(file_index);
  csv_stream_.clear();
  csv_stream_.open(file_path, std::ios::out | std::ios::trunc);
  if (!csv_stream_.is_open()) {
    ROS_ERROR_STREAM("Failed to open IMU timestamp CSV file: " << file_path);
    return false;
  }

  csv_stream_ << csvHeader() << "\n";
  csv_stream_.flush();
  if (!csv_stream_) {
    ROS_ERROR_STREAM("Failed to write IMU timestamp CSV header: " << file_path);
    csv_stream_.close();
    return false;
  }

  csv_file_index_ = file_index;
  csv_rows_written_ = 1;
  return true;
}

bool ImuTimestampCsvLogger::rotateCsvFile() {
  if (csv_stream_.is_open()) {
    csv_stream_.flush();
    csv_stream_.close();
  }

  const auto next_file_index = csv_file_index_ + 1;
  if (!openCsvFile(next_file_index)) {
    return false;
  }

  ROS_INFO_STREAM("IMU timestamp CSV reached " << kMaxCsvRowsPerFileIncludingHeader
                                               << " rows; continuing in "
                                               << csvFilePathForIndex(csv_file_index_));
  return true;
}

}  // namespace orbbec_camera
