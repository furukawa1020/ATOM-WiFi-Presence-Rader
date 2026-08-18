#include "CalibrationManager.hpp"

#include <math.h>
#include <string.h>

CalibrationManager::CalibrationManager() {
  configure(CalibrationConfig{});
  reset();
}

CalibrationManager::CalibrationManager(const CalibrationConfig& config) {
  configure(config);
  reset();
}

void CalibrationManager::configure(const CalibrationConfig& config) {
  config_ = config;
  if (config_.max_empty_frames == 0 ||
      config_.max_empty_frames > kMaxEmptyFrames) {
    config_.max_empty_frames = kMaxEmptyFrames;
  }
  if (config_.min_empty_frames == 0) {
    config_.min_empty_frames = 1;
  }
  if (config_.min_empty_frames > config_.max_empty_frames) {
    config_.min_empty_frames = config_.max_empty_frames;
  }
  if (config_.min_usable_subcarriers == 0) {
    config_.min_usable_subcarriers = 1;
  }
  if (config_.min_usable_subcarriers > kMaxSubcarriers) {
    config_.min_usable_subcarriers = kMaxSubcarriers;
  }
  if (config_.min_valid_ratio < 0.0F) {
    config_.min_valid_ratio = 0.0F;
  } else if (config_.min_valid_ratio > 1.0F) {
    config_.min_valid_ratio = 1.0F;
  }
  if (config_.min_packet_rate_hz < 0.0F) {
    config_.min_packet_rate_hz = 0.0F;
  }
  if (config_.max_empty_change <= 0.0F) {
    config_.max_empty_change = 0.35F;
  }
  if (config_.mad_floor < 0.0F) {
    config_.mad_floor = 0.0F;
  }
  if (config_.empty_sample_interval_us < 0) {
    config_.empty_sample_interval_us = 0;
  }
}

void CalibrationManager::reset() {
  memset(&progress_, 0, sizeof(progress_));
  progress_.stage = CalibrationStage::Idle;
  progress_.last_reject_reason = CalibrationRejectReason::None;
  memset(sessions_, 0, sizeof(sessions_));
  memset(&baseline_, 0, sizeof(baseline_));
  active_session_index_ = -1;
  session_count_ = 0;
  resetEmptyCollection();
}

CalibrationStatus CalibrationManager::beginCalibration(
    uint32_t calibration_id) {
  if (calibration_id == 0) {
    return CalibrationStatus::InvalidArgument;
  }

  reset();
  progress_.calibration_id = calibration_id;
  progress_.stage = CalibrationStage::RadioSurvey;
  return CalibrationStatus::Ok;
}

CalibrationStatus CalibrationManager::transitionTo(
    CalibrationStage next_stage) {
  if (progress_.active_session) {
    return CalibrationStatus::SessionAlreadyActive;
  }
  if (next_stage == progress_.stage) {
    return CalibrationStatus::Ok;
  }
  if (!isAllowedTransition(progress_.stage, next_stage)) {
    return CalibrationStatus::InvalidTransition;
  }

  progress_.stage = next_stage;
  return CalibrationStatus::Ok;
}

CalibrationStatus CalibrationManager::startSession(
    const CalibrationSessionDescriptor& descriptor,
    int64_t started_at_us) {
  if (descriptor.session_id == 0 || started_at_us <= 0) {
    return CalibrationStatus::InvalidArgument;
  }
  if (progress_.active_session) {
    return CalibrationStatus::SessionAlreadyActive;
  }
  if (hasSessionId(descriptor.session_id)) {
    return CalibrationStatus::DuplicateSessionId;
  }
  if (session_count_ >= kMaxSessions) {
    return CalibrationStatus::SessionCapacityReached;
  }
  if (stageForSession(descriptor) != progress_.stage) {
    return CalibrationStatus::WrongSessionType;
  }

  CalibrationSessionSummary& session = sessions_[session_count_];
  session = CalibrationSessionSummary{};
  session.descriptor = descriptor;
  session.state = CalibrationSessionState::Active;
  session.started_at_us = started_at_us;
  active_session_index_ = static_cast<int8_t>(session_count_);
  ++session_count_;

  progress_.active_session = true;
  progress_.active_session_id = descriptor.session_id;
  progress_.recorded_sessions = session_count_;
  progress_.accepted_frames = 0;
  progress_.rejected_frames = 0;
  progress_.rejected_link_quality = 0;
  progress_.rejected_queue_drop = 0;
  progress_.rejected_device_movement = 0;
  progress_.rejected_sample_interval = 0;
  progress_.rejected_frame_shape = 0;
  progress_.rejected_scene_change = 0;
  progress_.rejected_capacity = 0;
  progress_.last_empty_change = 0.0F;
  progress_.last_reject_reason = CalibrationRejectReason::None;

  if (descriptor.dataset == CalibrationDataset::Training &&
      descriptor.label == CalibrationLabel::Empty) {
    resetEmptyCollection();
  }

  return CalibrationStatus::Ok;
}

CalibrationStatus CalibrationManager::ingestEmptyFrame(
    const PreprocessedCsiFrame& frame,
    const CalibrationQualitySnapshot& quality) {
  if (!progress_.active_session || active_session_index_ < 0) {
    return CalibrationStatus::NoActiveSession;
  }

  const CalibrationSessionDescriptor& descriptor =
      sessions_[active_session_index_].descriptor;
  if (progress_.stage != CalibrationStage::EmptyTraining ||
      descriptor.dataset != CalibrationDataset::Training ||
      descriptor.label != CalibrationLabel::Empty) {
    return CalibrationStatus::WrongSessionType;
  }

  if (!queue_drop_count_ready_) {
    last_queue_drop_count_ = quality.queue_drop_count;
    queue_drop_count_ready_ = true;
  } else if (quality.queue_drop_count != last_queue_drop_count_) {
    last_queue_drop_count_ = quality.queue_drop_count;
    return reject(
        CalibrationRejectReason::QueueDrop,
        CalibrationStatus::QueueDropRejected);
  }
  if (!quality.all_required_links_healthy ||
      !(quality.packet_rate_hz >= config_.min_packet_rate_hz)) {
    return reject(
        CalibrationRejectReason::LinkQuality,
        CalibrationStatus::LinkQualityRejected);
  }
  if (!quality.device_fixed) {
    return reject(
        CalibrationRejectReason::DeviceMovement,
        CalibrationStatus::DeviceMovementRejected);
  }
  if (empty_frame_count_ >= config_.max_empty_frames) {
    return reject(
        CalibrationRejectReason::Capacity,
        CalibrationStatus::CollectionFull);
  }
  if (frame.received_at_us <= 0 || frame.sample_count == 0 ||
      frame.sample_count > kMaxSubcarriers) {
    return reject(
        CalibrationRejectReason::FrameShape,
        CalibrationStatus::FrameShapeRejected);
  }
  if (last_accepted_at_us_ > 0 &&
      frame.received_at_us - last_accepted_at_us_ <
          config_.empty_sample_interval_us) {
    return reject(
        CalibrationRejectReason::SampleInterval,
        CalibrationStatus::SampleIntervalRejected);
  }

  uint16_t valid_count = 0;
  for (uint16_t i = 0; i < frame.sample_count; ++i) {
    if (frame.samples[i].valid &&
        isfinite(frame.samples[i].centered_amplitude)) {
      ++valid_count;
    }
  }
  if (valid_count < config_.min_usable_subcarriers) {
    return reject(
        CalibrationRejectReason::InsufficientValidSubcarriers,
        CalibrationStatus::InsufficientValidSubcarriers);
  }

  if (subcarrier_count_ == 0) {
    subcarrier_count_ = frame.sample_count;
    for (uint16_t i = 0; i < subcarrier_count_; ++i) {
      subcarrier_order_[i] = frame.samples[i].subcarrier;
    }
  } else {
    if (frame.sample_count != subcarrier_count_) {
      return reject(
          CalibrationRejectReason::FrameShape,
          CalibrationStatus::FrameShapeRejected);
    }
    for (uint16_t i = 0; i < subcarrier_count_; ++i) {
      if (frame.samples[i].subcarrier != subcarrier_order_[i]) {
        return reject(
            CalibrationRejectReason::FrameShape,
            CalibrationStatus::FrameShapeRejected);
      }
    }
  }

  if (previous_frame_ready_) {
    uint16_t delta_count = 0;
    for (uint16_t i = 0; i < subcarrier_count_; ++i) {
      if (previous_valid_[i] && frame.samples[i].valid &&
          isfinite(frame.samples[i].centered_amplitude)) {
        scratch_[delta_count++] = fabsf(
            frame.samples[i].centered_amplitude - previous_empty_frame_[i]);
      }
    }
    if (delta_count < config_.min_usable_subcarriers) {
      return reject(
          CalibrationRejectReason::InsufficientValidSubcarriers,
          CalibrationStatus::InsufficientValidSubcarriers);
    }

    progress_.last_empty_change = median(scratch_, delta_count);
    if (progress_.last_empty_change > config_.max_empty_change) {
      return reject(
          CalibrationRejectReason::EmptySceneChange,
          CalibrationStatus::EmptySceneChangeRejected);
    }
  }

  for (uint16_t i = 0; i < subcarrier_count_; ++i) {
    const bool valid = frame.samples[i].valid &&
        isfinite(frame.samples[i].centered_amplitude);
    empty_samples_[empty_frame_count_][i] =
        valid ? frame.samples[i].centered_amplitude : NAN;
    previous_empty_frame_[i] = frame.samples[i].centered_amplitude;
    previous_valid_[i] = valid;
  }

  previous_frame_ready_ = true;
  last_accepted_at_us_ = frame.received_at_us;
  ++empty_frame_count_;
  ++progress_.accepted_frames;
  progress_.last_reject_reason = CalibrationRejectReason::None;
  return CalibrationStatus::Ok;
}

CalibrationStatus CalibrationManager::finishSession(
    int64_t finished_at_us) {
  if (!progress_.active_session || active_session_index_ < 0) {
    return CalibrationStatus::NoActiveSession;
  }
  if (finished_at_us <= sessions_[active_session_index_].started_at_us) {
    return CalibrationStatus::InvalidArgument;
  }

  CalibrationStatus status = CalibrationStatus::Ok;
  const CalibrationSessionDescriptor& descriptor =
      sessions_[active_session_index_].descriptor;
  if (descriptor.dataset == CalibrationDataset::Training &&
      descriptor.label == CalibrationLabel::Empty) {
    status = buildBaseline();
  }

  closeActiveSession(
      status == CalibrationStatus::Ok
          ? CalibrationSessionState::Completed
          : CalibrationSessionState::Failed,
      finished_at_us);
  return status;
}

CalibrationStatus CalibrationManager::cancelSession(
    int64_t finished_at_us) {
  if (!progress_.active_session || active_session_index_ < 0) {
    return CalibrationStatus::NoActiveSession;
  }
  if (finished_at_us <= sessions_[active_session_index_].started_at_us) {
    return CalibrationStatus::InvalidArgument;
  }

  closeActiveSession(CalibrationSessionState::Cancelled, finished_at_us);
  return CalibrationStatus::Ok;
}

const CalibrationProgress& CalibrationManager::progress() const {
  return progress_;
}

const RobustBaseline* CalibrationManager::baseline() const {
  return baseline_.ready ? &baseline_ : nullptr;
}

uint8_t CalibrationManager::sessionCount() const {
  return session_count_;
}

bool CalibrationManager::sessionSummary(
    uint8_t index,
    CalibrationSessionSummary& output) const {
  if (index >= session_count_) {
    return false;
  }
  output = sessions_[index];
  return true;
}

bool CalibrationManager::isAllowedTransition(
    CalibrationStage current,
    CalibrationStage next) {
  switch (current) {
    case CalibrationStage::RadioSurvey:
      return next == CalibrationStage::EmptyTraining;
    case CalibrationStage::EmptyTraining:
      return next == CalibrationStage::StillTraining;
    case CalibrationStage::StillTraining:
      return next == CalibrationStage::MotionTraining;
    case CalibrationStage::MotionTraining:
      return next == CalibrationStage::NuisanceTraining ||
          next == CalibrationStage::Validation;
    case CalibrationStage::NuisanceTraining:
      return next == CalibrationStage::Validation;
    case CalibrationStage::Validation:
      return next == CalibrationStage::ModelCommit;
    case CalibrationStage::ModelCommit:
      return next == CalibrationStage::Complete;
    case CalibrationStage::Idle:
    case CalibrationStage::Complete:
    default:
      return false;
  }
}

CalibrationStage CalibrationManager::stageForSession(
    const CalibrationSessionDescriptor& descriptor) {
  if (descriptor.dataset == CalibrationDataset::Validation) {
    return CalibrationStage::Validation;
  }

  switch (descriptor.label) {
    case CalibrationLabel::Empty:
      return CalibrationStage::EmptyTraining;
    case CalibrationLabel::Still:
      return CalibrationStage::StillTraining;
    case CalibrationLabel::Motion:
      return CalibrationStage::MotionTraining;
    case CalibrationLabel::Nuisance:
      return CalibrationStage::NuisanceTraining;
    default:
      return CalibrationStage::Idle;
  }
}

void CalibrationManager::sortAscending(float* values, uint16_t count) {
  for (uint16_t i = 1; i < count; ++i) {
    const float value = values[i];
    uint16_t position = i;
    while (position > 0 && values[position - 1] > value) {
      values[position] = values[position - 1];
      --position;
    }
    values[position] = value;
  }
}

float CalibrationManager::median(float* values, uint16_t count) {
  if (count == 0) {
    return 0.0F;
  }
  sortAscending(values, count);
  const uint16_t middle = count / 2;
  if ((count & 1U) != 0U) {
    return values[middle];
  }
  return 0.5F * (values[middle - 1] + values[middle]);
}

CalibrationStatus CalibrationManager::reject(
    CalibrationRejectReason reason,
    CalibrationStatus status) {
  ++progress_.rejected_frames;
  progress_.last_reject_reason = reason;
  switch (reason) {
    case CalibrationRejectReason::LinkQuality:
      ++progress_.rejected_link_quality;
      break;
    case CalibrationRejectReason::QueueDrop:
      ++progress_.rejected_queue_drop;
      break;
    case CalibrationRejectReason::DeviceMovement:
      ++progress_.rejected_device_movement;
      break;
    case CalibrationRejectReason::SampleInterval:
      ++progress_.rejected_sample_interval;
      break;
    case CalibrationRejectReason::FrameShape:
    case CalibrationRejectReason::InsufficientValidSubcarriers:
      ++progress_.rejected_frame_shape;
      break;
    case CalibrationRejectReason::EmptySceneChange:
      ++progress_.rejected_scene_change;
      break;
    case CalibrationRejectReason::Capacity:
      ++progress_.rejected_capacity;
      break;
    case CalibrationRejectReason::None:
    default:
      break;
  }
  return status;
}

CalibrationStatus CalibrationManager::buildBaseline() {
  if (empty_frame_count_ < config_.min_empty_frames ||
      subcarrier_count_ == 0) {
    return CalibrationStatus::InsufficientSamples;
  }

  RobustBaseline candidate{};
  candidate.sample_count = subcarrier_count_;
  const uint16_t required_valid = static_cast<uint16_t>(ceilf(
      static_cast<float>(empty_frame_count_) * config_.min_valid_ratio));
  uint16_t usable_count = 0;

  for (uint16_t carrier = 0; carrier < subcarrier_count_; ++carrier) {
    candidate.subcarriers[carrier] = subcarrier_order_[carrier];
    uint16_t value_count = 0;
    for (uint16_t frame = 0; frame < empty_frame_count_; ++frame) {
      const float value = empty_samples_[frame][carrier];
      if (isfinite(value)) {
        scratch_[value_count++] = value;
      }
    }
    if (value_count < required_valid) {
      candidate.usable[carrier] = false;
      continue;
    }

    const float carrier_median = median(scratch_, value_count);
    uint16_t deviation_count = 0;
    for (uint16_t frame = 0; frame < empty_frame_count_; ++frame) {
      const float value = empty_samples_[frame][carrier];
      if (isfinite(value)) {
        scratch_[deviation_count++] = fabsf(value - carrier_median);
      }
    }
    float carrier_mad = median(scratch_, deviation_count);
    if (carrier_mad < config_.mad_floor) {
      carrier_mad = config_.mad_floor;
    }

    candidate.median[carrier] = carrier_median;
    candidate.mad[carrier] = carrier_mad;
    candidate.usable[carrier] = true;
    ++usable_count;
  }

  progress_.usable_subcarriers = usable_count;
  if (usable_count < config_.min_usable_subcarriers) {
    return CalibrationStatus::BaselineUnavailable;
  }

  candidate.ready = true;
  baseline_ = candidate;
  progress_.baseline_ready = true;
  return CalibrationStatus::Ok;
}

void CalibrationManager::resetEmptyCollection() {
  empty_frame_count_ = 0;
  subcarrier_count_ = 0;
  previous_frame_ready_ = false;
  last_accepted_at_us_ = 0;
  queue_drop_count_ready_ = false;
  last_queue_drop_count_ = 0;
  memset(subcarrier_order_, 0, sizeof(subcarrier_order_));
  memset(previous_empty_frame_, 0, sizeof(previous_empty_frame_));
  memset(previous_valid_, 0, sizeof(previous_valid_));
}

void CalibrationManager::closeActiveSession(
    CalibrationSessionState state,
    int64_t finished_at_us) {
  CalibrationSessionSummary& session = sessions_[active_session_index_];
  session.state = state;
  session.accepted_frames = progress_.accepted_frames;
  session.rejected_frames = progress_.rejected_frames;
  session.finished_at_us = finished_at_us;
  progress_.active_session = false;
  progress_.active_session_id = 0;
  progress_.baseline_ready = baseline_.ready;
  active_session_index_ = -1;
}

bool CalibrationManager::hasSessionId(uint32_t session_id) const {
  for (uint8_t i = 0; i < session_count_; ++i) {
    if (sessions_[i].descriptor.session_id == session_id) {
      return true;
    }
  }
  return false;
}
