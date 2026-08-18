#pragma once

#include <stdint.h>

#include "calibration_model_types.hpp"

constexpr uint8_t kTemporalEncoderMaxStates = 8;

enum class TemporalEncoderStatus : uint8_t {
  Ok = 0,
  Warmup,
  NoActiveModel,
  InvalidModel,
  InvalidInput,
  SchemaMismatch,
  FeatureCountMismatch,
  QualityRejected,
  StateReset,
  NumericalFailure,
};

enum class TemporalEncoderResetReason : uint8_t {
  None = 0,
  Manual,
  ModelActivated,
  FrameGap,
  TimeRollback,
  LinkLoss,
  QueueDrop,
  DeviceMovement,
  EnvironmentDrift,
  NumericalFailure,
};

struct AdaptiveTemporalEncoderModel {
  bool ready = false;
  uint32_t model_id = 0;
  uint16_t model_version = 0;
  CalibrationFeatureSchema input_schema{};
  CalibrationFeatureSchema output_schema{};
  uint8_t state_count = 0;
  uint8_t output_count = 0;
  uint16_t warmup_steps = 0;
  int64_t nominal_interval_us = 0;
  int64_t max_gap_us = 0;
  float input_clip = 0.0F;
  float drift_reset_threshold = 0.0F;
  float input_mean[kCalibrationMaxFeatures]{};
  float input_scale[kCalibrationMaxFeatures]{};
  float input_projection[
      kTemporalEncoderMaxStates][kCalibrationMaxFeatures]{};
  float input_bias[kTemporalEncoderMaxStates]{};
  float update_gate_gain[kTemporalEncoderMaxStates]{};
  float update_gate_bias[kTemporalEncoderMaxStates]{};
  float decay_rate[kTemporalEncoderMaxStates]{};
  float output_projection[
      kCalibrationMaxFeatures][kTemporalEncoderMaxStates]{};
  float skip_scale[kCalibrationMaxFeatures]{};
  float output_bias[kCalibrationMaxFeatures]{};
};

struct TemporalEncoderStepContext {
  int64_t timestamp_us = 0;
  bool link_healthy = false;
  bool queue_drop_detected = false;
  bool device_moved = false;
  float environment_drift_score = 0.0F;
};

struct TemporalEncoderDiagnostics {
  uint32_t processed_steps = 0;
  uint32_t emitted_steps = 0;
  uint32_t rejected_steps = 0;
  uint32_t reset_count = 0;
  uint32_t frame_gap_resets = 0;
  uint32_t time_rollback_resets = 0;
  uint32_t link_loss_resets = 0;
  uint32_t queue_drop_resets = 0;
  uint32_t device_movement_resets = 0;
  uint32_t environment_drift_resets = 0;
  uint32_t numerical_resets = 0;
  uint16_t warmup_remaining = 0;
  float last_mean_gate = 0.0F;
  float last_state_norm = 0.0F;
  TemporalEncoderResetReason last_reset_reason =
      TemporalEncoderResetReason::None;
};
