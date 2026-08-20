#pragma once

#include <cstdint>

namespace atom::radar {

enum class BathroomIntegratedSafetyLevel : uint8_t {
  Normal = 0,
  Attention,
  CheckRequired,
  Urgent,
  EmergencyNotificationRecommended,
  MeasurementUnavailable,
};

struct BathroomIntegratedSafetyEvidence {
  uint32_t probe_sequence = 0;
  uint64_t observed_at_us = 0;
  BathroomIntegratedSafetyLevel safety_level =
      BathroomIntegratedSafetyLevel::Normal;

  float floor_fall_evidence = 0.0F;
  float bathtub_immobility_evidence = 0.0F;
  float dangerous_posture_evidence = 0.0F;
  float respiratory_motion_loss_evidence = 0.0F;
  float prolonged_bathing_evidence = 0.0F;
  float occupied_unknown_evidence = 0.0F;
  float nuisance_alternative = 0.0F;
  float measurement_failure_evidence = 0.0F;

  float fast_risk = 0.0F;
  float medium_risk = 0.0F;
  float slow_risk = 0.0F;
  float overall_risk = 0.0F;
  float stale_danger_memory = 0.0F;
  float bathing_time_thermal_context = 0.0F;
  float analysis_quality = 0.0F;

  uint32_t level_duration_ms = 0;
  uint32_t consecutive_risk_duration_ms = 0;
  uint32_t measurement_unavailable_duration_ms = 0;

  bool room_temperature_available = false;
  bool water_temperature_available = false;
  bool heat_shock_assessment_available = false;
  bool thermal_measurement_unavailable = true;
  bool physically_observable = false;
  bool evidence_ready = false;
};

}  // namespace atom::radar
