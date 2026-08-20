#pragma once

#include <cstdint>

#include "bathroom_integrated_safety_types.hpp"

namespace atom::radar {

enum class BathroomNullContext : uint8_t {
  EmptyRoom = 0,
  OccupiedQuiet,
  Shower,
  VentilationFan,
  WaterDynamics,
  DoorTransition,
  Unknown,
};

struct BathroomNullCalibratedFeature {
  float raw_evidence = 0.0F;
  float baseline_mean = 0.0F;
  float baseline_scale = 0.0F;
  float standardized_excess = 0.0F;
  float calibrated_evidence = 0.0F;
  float maturity = 0.0F;
  uint32_t samples = 0;
};

struct BathroomContextNullCalibrationEvidence {
  uint32_t probe_sequence = 0;
  uint64_t observed_at_us = 0;
  BathroomNullContext context = BathroomNullContext::Unknown;
  BathroomIntegratedSafetyLevel raw_level =
      BathroomIntegratedSafetyLevel::Normal;
  BathroomIntegratedSafetyLevel recommended_level =
      BathroomIntegratedSafetyLevel::Normal;

  BathroomNullCalibratedFeature floor_fall{};
  BathroomNullCalibratedFeature bathtub_immobility{};
  BathroomNullCalibratedFeature dangerous_posture{};
  BathroomNullCalibratedFeature respiratory_motion_loss{};
  BathroomNullCalibratedFeature occupied_unknown{};

  float prolonged_bathing_evidence = 0.0F;
  float raw_overall_risk = 0.0F;
  float calibrated_overall_risk = 0.0F;
  float explained_context_risk = 0.0F;
  float context_confidence = 0.0F;
  float profile_maturity = 0.0F;
  float update_quality = 0.0F;
  float profile_drift_score = 0.0F;
  float shadow_profile_maturity = 0.0F;
  uint32_t drift_consecutive_samples = 0U;
  uint32_t shadow_profile_samples = 0U;
  uint32_t shadow_stable_samples = 0U;
  uint32_t profile_generation = 0U;
  uint32_t updates_since_checkpoint = 0U;
  uint32_t checkpoint_age_ms = 0U;

  bool null_update_allowed = false;
  bool null_profile_updated = false;
  bool calibration_applied = false;
  bool danger_lock = false;
  bool physically_observable = false;
  bool drift_monitor_allowed = false;
  bool drift_warning = false;
  bool context_quarantined = false;
  bool shadow_update_allowed = false;
  bool rebaseline_accepted = false;
  bool storage_available = false;
  bool persisted_profile_loaded = false;
  bool recovered_from_single_slot = false;
  bool profile_dirty = false;
  bool checkpoint_ok = false;
  bool evidence_ready = false;
};

}  // namespace atom::radar
