#include "orbbec_camera/timestamp_csv_logger.h"

#include "orbbec_camera/frame_timestamp_csv_logger.h"
#include "orbbec_camera/imu_timestamp_csv_logger.h"
#include "ros/ros.h"

#include <exception>

namespace orbbec_camera {

TimestampCsvLogger::TimestampCsvLogger(Config config) {
  const auto create_image_logger = [&config](FrameTimestampCsvLogger::OutputMode mode) {
    auto timestamp_logger = std::make_unique<FrameTimestampCsvLogger>(config.frame_drop_log_enabled,
                                                                      config.csv_file_path, mode);
    if (!timestamp_logger->enabled()) {
      timestamp_logger.reset();
    }
    return timestamp_logger;
  };

  if (config.frame_sync_enabled && (config.color_enabled || config.depth_enabled)) {
    synced_image_logger_ = create_image_logger(FrameTimestampCsvLogger::OutputMode::SYNCED);
  } else {
    if (config.color_enabled) {
      color_logger_ = create_image_logger(FrameTimestampCsvLogger::OutputMode::COLOR);
    }
    if (config.depth_enabled) {
      depth_logger_ = create_image_logger(FrameTimestampCsvLogger::OutputMode::DEPTH);
    }
  }

  if (config.csv_file_path.empty()) {
    return;
  }

  const auto create_imu_logger = [&config](ImuTimestampCsvLogger::OutputMode mode) {
    auto timestamp_logger = std::make_unique<ImuTimestampCsvLogger>(config.csv_file_path, mode);
    if (!timestamp_logger->enabled()) {
      timestamp_logger.reset();
    }
    return timestamp_logger;
  };

  if (config.imu_sync_enabled) {
    synced_imu_logger_ = create_imu_logger(ImuTimestampCsvLogger::OutputMode::SYNCED);
  } else {
    if (config.accel_enabled) {
      accel_logger_ = create_imu_logger(ImuTimestampCsvLogger::OutputMode::ACCEL);
    }
    if (config.gyro_enabled) {
      gyro_logger_ = create_imu_logger(ImuTimestampCsvLogger::OutputMode::GYRO);
    }
  }
}

TimestampCsvLogger::~TimestampCsvLogger() noexcept { shutdown(); }

bool TimestampCsvLogger::enabled() const {
  return imageEnabled() || syncedImuEnabled() || standaloneImuEnabled(ACCEL) ||
         standaloneImuEnabled(GYRO);
}

bool TimestampCsvLogger::imageEnabled() const {
  return (synced_image_logger_ && synced_image_logger_->enabled()) ||
         (color_logger_ && color_logger_->enabled()) || (depth_logger_ && depth_logger_->enabled());
}

bool TimestampCsvLogger::imageStreamEnabled(const stream_index_pair &stream_index) const {
  const auto *timestamp_logger = imageLoggerForStream(stream_index);
  return timestamp_logger && timestamp_logger->enabled();
}

bool TimestampCsvLogger::syncedImuEnabled() const {
  return synced_imu_logger_ && synced_imu_logger_->enabled();
}

bool TimestampCsvLogger::standaloneImuEnabled(const stream_index_pair &stream_index) const {
  const auto *timestamp_logger = standaloneImuLoggerForStream(stream_index);
  return timestamp_logger && timestamp_logger->enabled();
}

void TimestampCsvLogger::recordImageFrameSet(const std::shared_ptr<ob::Frame> &color_frame,
                                             const std::shared_ptr<ob::Frame> &depth_frame,
                                             int64_t arrival_system_us, int64_t arrival_steady_us,
                                             bool track_color, bool track_depth,
                                             bool color_image_publish_expected,
                                             bool depth_image_publish_expected) {
  if (synced_image_logger_) {
    synced_image_logger_->recordFrameSet(
        color_frame, depth_frame, arrival_system_us, arrival_steady_us, track_color, track_depth,
        color_image_publish_expected, depth_image_publish_expected);
    return;
  }

  if (track_color && color_logger_) {
    color_logger_->recordStandaloneFrameArrival(COLOR, color_frame, arrival_system_us,
                                                arrival_steady_us, color_image_publish_expected);
  }
  if (track_depth && depth_logger_) {
    depth_logger_->recordStandaloneFrameArrival(DEPTH, depth_frame, arrival_system_us,
                                                arrival_steady_us, depth_image_publish_expected);
  }
}

void TimestampCsvLogger::recordStandaloneImageArrival(const stream_index_pair &stream_index,
                                                      const std::shared_ptr<ob::Frame> &frame,
                                                      int64_t arrival_system_us,
                                                      int64_t arrival_steady_us,
                                                      bool image_publish_expected) {
  auto *timestamp_logger = imageLoggerForStream(stream_index);
  if (timestamp_logger) {
    timestamp_logger->recordStandaloneFrameArrival(stream_index, frame, arrival_system_us,
                                                   arrival_steady_us, image_publish_expected);
  }
}

void TimestampCsvLogger::recordImagePrePublish(const stream_index_pair &stream_index,
                                               const std::shared_ptr<ob::Frame> &frame,
                                               int64_t publish_system_us,
                                               int64_t publish_steady_us) {
  auto *timestamp_logger = imageLoggerForStream(stream_index);
  if (timestamp_logger) {
    timestamp_logger->recordPreImagePublish(stream_index, frame, publish_system_us,
                                            publish_steady_us);
  }
}

void TimestampCsvLogger::recordImagePublishSkipped(const stream_index_pair &stream_index,
                                                   const std::shared_ptr<ob::Frame> &frame) {
  auto *timestamp_logger = imageLoggerForStream(stream_index);
  if (timestamp_logger) {
    timestamp_logger->recordImagePublishSkipped(stream_index, frame);
  }
}

void TimestampCsvLogger::recordSyncedImu(const std::shared_ptr<ob::Frame> &accel_frame,
                                         const std::shared_ptr<ob::Frame> &gyro_frame,
                                         int64_t arrival_system_us,
                                         std::optional<int64_t> publish_system_us) {
  if (synced_imu_logger_) {
    synced_imu_logger_->recordFrameSet(accel_frame, gyro_frame, arrival_system_us,
                                       publish_system_us);
  }
}

void TimestampCsvLogger::recordStandaloneImu(const stream_index_pair &stream_index,
                                             const std::shared_ptr<ob::Frame> &frame,
                                             int64_t arrival_system_us,
                                             std::optional<int64_t> publish_system_us) {
  auto *timestamp_logger = standaloneImuLoggerForStream(stream_index);
  if (timestamp_logger) {
    timestamp_logger->recordStandaloneFrame(stream_index, frame, arrival_system_us,
                                            publish_system_us);
  }
}

void TimestampCsvLogger::shutdown() noexcept {
  if (shutdown_requested_.exchange(true)) {
    return;
  }

  const auto shutdown_logger = [](auto &timestamp_logger, const char *name) {
    if (!timestamp_logger) {
      return;
    }
    try {
      timestamp_logger->shutdown();
    } catch (const std::exception &e) {
      ROS_WARN_STREAM("Exception while shutting down " << name << ": " << e.what());
    } catch (...) {
      ROS_WARN_STREAM("Unknown exception while shutting down " << name);
    }
  };

  shutdown_logger(synced_image_logger_, "synced image timestamp CSV logger");
  shutdown_logger(color_logger_, "color timestamp CSV logger");
  shutdown_logger(depth_logger_, "depth timestamp CSV logger");
  shutdown_logger(synced_imu_logger_, "synced IMU timestamp CSV logger");
  shutdown_logger(accel_logger_, "accel timestamp CSV logger");
  shutdown_logger(gyro_logger_, "gyro timestamp CSV logger");
}

FrameTimestampCsvLogger *TimestampCsvLogger::imageLoggerForStream(
    const stream_index_pair &stream_index) const {
  if (stream_index == COLOR) {
    return synced_image_logger_ ? synced_image_logger_.get() : color_logger_.get();
  }
  if (stream_index == DEPTH) {
    return synced_image_logger_ ? synced_image_logger_.get() : depth_logger_.get();
  }
  return nullptr;
}

ImuTimestampCsvLogger *TimestampCsvLogger::standaloneImuLoggerForStream(
    const stream_index_pair &stream_index) const {
  if (stream_index == ACCEL) {
    return accel_logger_.get();
  }
  if (stream_index == GYRO) {
    return gyro_logger_.get();
  }
  return nullptr;
}

}  // namespace orbbec_camera
