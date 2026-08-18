#pragma once

#include <stdint.h>

enum class CalibrationStage : uint8_t {
  Idle = 0,
  RadioSurvey,
  EmptyTraining,
  StillTraining,
  MotionTraining,
  NuisanceTraining,
  Validation,
  ModelCommit,
  Complete,
};

enum class CalibrationDataset : uint8_t {
  Training = 0,
  Validation,
};

enum class CalibrationLabel : uint8_t {
  Empty = 0,
  Still,
  Motion,
  Nuisance,
};

enum class CalibrationSessionState : uint8_t {
  Active = 0,
  Completed,
  Failed,
  Cancelled,
};

enum class CalibrationRejectReason : uint8_t {
  None = 0,
  LinkQuality,
  QueueDrop,
  DeviceMovement,
  SampleInterval,
  FrameShape,
  InsufficientValidSubcarriers,
  EmptySceneChange,
  Capacity,
};

enum class CalibrationStatus : uint8_t {
  Ok = 0,
  InvalidArgument,
  InvalidTransition,
  SessionAlreadyActive,
  DuplicateSessionId,
  SessionCapacityReached,
  NoActiveSession,
  WrongSessionType,
  LinkQualityRejected,
  QueueDropRejected,
  DeviceMovementRejected,
  SampleIntervalRejected,
  FrameShapeRejected,
  InsufficientValidSubcarriers,
  EmptySceneChangeRejected,
  CollectionFull,
  InsufficientSamples,
  BaselineUnavailable,
};

struct CalibrationSessionDescriptor {
  uint32_t session_id = 0;
  CalibrationDataset dataset = CalibrationDataset::Training;
  CalibrationLabel label = CalibrationLabel::Empty;
  uint16_t repetition = 0;
};

struct CalibrationQualitySnapshot {
  bool all_required_links_healthy = false;
  bool device_fixed = false;
  float packet_rate_hz = 0.0F;
  uint32_t queue_drop_count = 0;
};

struct CalibrationSessionSummary {
  CalibrationSessionDescriptor descriptor{};
  CalibrationSessionState state = CalibrationSessionState::Cancelled;
  uint16_t accepted_frames = 0;
  uint16_t rejected_frames = 0;
  int64_t started_at_us = 0;
  int64_t finished_at_us = 0;
};

struct CalibrationProgress {
  uint32_t calibration_id = 0;
  CalibrationStage stage = CalibrationStage::Idle;
  bool active_session = false;
  uint32_t active_session_id = 0;
  uint8_t recorded_sessions = 0;
  uint16_t accepted_frames = 0;
  uint16_t rejected_frames = 0;
  uint16_t rejected_link_quality = 0;
  uint16_t rejected_queue_drop = 0;
  uint16_t rejected_device_movement = 0;
  uint16_t rejected_sample_interval = 0;
  uint16_t rejected_frame_shape = 0;
  uint16_t rejected_scene_change = 0;
  uint16_t rejected_capacity = 0;
  uint16_t usable_subcarriers = 0;
  float last_empty_change = 0.0F;
  bool baseline_ready = false;
  CalibrationRejectReason last_reject_reason = CalibrationRejectReason::None;
};
