#pragma once

#include <atomic>
#include <stdint.h>

#include "CalibratedModelRuntime.hpp"
#include "adaptive_temporal_encoder_types.hpp"

class AdaptiveTemporalEncoder {
 public:
  AdaptiveTemporalEncoder();

  void reset();
  void resetState(
      TemporalEncoderResetReason reason = TemporalEncoderResetReason::Manual);
  TemporalEncoderStatus activate(const AdaptiveTemporalEncoderModel& candidate);
  TemporalEncoderStatus step(
      const CalibrationFeatureVector& input,
      const TemporalEncoderStepContext& context,
      CalibrationFeatureVector& output);

  bool hasActiveModel() const;
  const TemporalEncoderDiagnostics& diagnostics() const;

  static bool validateModel(const AdaptiveTemporalEncoderModel& model);
  static bool initializeDefaultModel(
      uint32_t model_id,
      const CalibrationFeatureSchema& input_schema,
      const CalibrationFeatureSchema& output_schema,
      AdaptiveTemporalEncoderModel& output);

 private:
  static bool validSchema(const CalibrationFeatureSchema& schema);
  static float softplus(float value);
  static float clamp(float value, float minimum, float maximum);
  void resetStateInternal(TemporalEncoderResetReason reason);

  AdaptiveTemporalEncoderModel models_[2]{};
  std::atomic<uint8_t> active_index_{0};
  std::atomic<bool> active_ready_{false};
  float state_[kTemporalEncoderMaxStates]{};
  int64_t last_timestamp_us_ = 0;
  uint16_t warmup_count_ = 0;
  TemporalEncoderDiagnostics diagnostics_{};
};
