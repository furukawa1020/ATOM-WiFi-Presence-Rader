#pragma once

#include <cstdint>

#include <AdaptiveTemporalEncoder.hpp>
#include <CalibratedModelRuntime.hpp>
#include <CalibrationStorage.hpp>
#include <calibration_runtime_types.hpp>

enum class CalibrationBootstrapStatus : uint8_t {
  Ok = 0,
  InvalidRuntimeFingerprint,
  StorageLoadFailed,
  CompatibilityFailed,
  TemporalModelInvalid,
  DetectionModelInvalid,
  TemporalActivationFailed,
  DetectionActivationFailed,
};

struct CalibrationBootstrapReport {
  CalibrationBootstrapStatus status =
      CalibrationBootstrapStatus::InvalidRuntimeFingerprint;
  CalibrationCompatibilityStatus compatibility_status =
      CalibrationCompatibilityStatus::InvalidRuntimeFingerprint;
  CalibrationStorageInfo storage_info{};
  uint8_t storage_status_code = 0xff;
  uint8_t temporal_status_code = 0xff;
  uint8_t detection_status_code = 0xff;
  uint32_t candidate_calibration_id = 0;
  uint32_t active_calibration_id = 0;
  uint32_t active_generation = 0;
  bool active = false;
};

class CalibrationBootstrap {
 public:
  CalibrationBootstrapStatus loadAndActivate(
      const CalibrationRuntimeFingerprint& runtime,
      CalibrationStorage& storage,
      AdaptiveTemporalEncoder& temporal_encoder,
      CalibratedModelRuntime& detection_runtime,
      CalibrationBootstrapReport& report);

  static bool validateRuntimeFingerprint(
      const CalibrationRuntimeFingerprint& runtime);
  static CalibrationCompatibilityStatus checkCompatibility(
      const CalibrationRuntimeFingerprint& runtime,
      const CalibrationArtifact& artifact);

  bool ready() const;
  uint32_t activeCalibrationId() const;
  uint32_t activeGeneration() const;
  const CalibrationArtifact* artifact() const;
  const RobustBaseline* baseline() const;
  uint8_t selectedSubcarrierCount() const;
  const int16_t* selectedSubcarriers() const;
  uint16_t covarianceValueCount() const;
  const float* covarianceLowerTriangle() const;

 private:
  void publishActiveState(CalibrationBootstrapReport& report) const;

  CalibrationArtifact candidate_{};
  CalibrationArtifact active_artifact_{};
  uint32_t active_generation_ = 0;
  bool active_ = false;
};
