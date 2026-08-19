#pragma once

#include <cstdint>

#include "bathroom_bathing_session_types.hpp"
#include "bathroom_csi_evidence_types.hpp"
#include "bathroom_integrated_safety_types.hpp"
#include "bathroom_respiration_evidence_types.hpp"
#include "bathroom_room_calibration_types.hpp"
#include "bathroom_safety_evidence_types.hpp"
#include "bathroom_spatial_evidence_types.hpp"
#include "m5atom_csi_precision_types.hpp"

namespace atom::radar {

struct BathroomIntegratedSafetyConfig {
  float attention_threshold = 0.26F;
  float check_required_threshold = 0.43F;
  float urgent_threshold = 0.62F;
  float emergency_threshold = 0.82F;
  float direct_emergency_threshold = 0.86F;
  float retained_danger_threshold = 0.55F;
  float physical_measurement_failure_threshold = 0.90F;
  uint32_t attention_confirmation_ms = 8000U;
  uint32_t check_required_confirmation_ms = 5000U;
  uint32_t urgent_confirmation_ms = 2000U;
  uint32_t downgrade_confirmation_ms = 20000U;
  uint32_t emergency_downgrade_confirmation_ms = 30000U;
  uint32_t measurement_unavailable_confirmation_ms = 5000U;
  uint32_t bathtub_time_context_start_ms = 10U * 60U * 1000U;
  uint32_t bathtub_time_context_full_ms = 20U * 60U * 1000U;
  uint32_t emit_interval_ms = 1000U;
};

enum class BathroomIntegratedSafetyUpdateStatus : uint8_t {
  Accepted = 0,
  NotReady,
  StaleObservation,
};

class BathroomIntegratedSafetyAnalyzer {
 public:
  explicit BathroomIntegratedSafetyAnalyzer(
      const BathroomIntegratedSafetyConfig& config = {});

  BathroomIntegratedSafetyUpdateStatus update(
      const FusedCsiObservation& fused,
      const BathroomCsiEvidence& csi,
      const BathroomSafetyEvidence& safety,
      const BathroomSpatialEvidence& spatial,
      const BathroomRespirationEvidence& respiration,
      const BathroomRoomCalibrationEvidence& calibration,
      const BathroomBathingSessionEvidence& session,
      BathroomIntegratedSafetyEvidence& output);

  void reset();

 private:
  struct TemporalState {
    float fast_risk = 0.0F;
    float medium_risk = 0.0F;
    float slow_risk = 0.0F;
    float stale_danger_memory = 0.0F;
    BathroomIntegratedSafetyLevel level =
        BathroomIntegratedSafetyLevel::Normal;
    BathroomIntegratedSafetyLevel candidate_level =
        BathroomIntegratedSafetyLevel::Normal;
    uint64_t observed_at_us = 0;
    uint64_t level_since_us = 0;
    uint64_t candidate_since_us = 0;
    uint64_t risk_since_us = 0;
    uint64_t unavailable_since_us = 0;
  };

  static float clamp01(float value);
  static float maximum(float a, float b);
  static uint8_t dangerRank(BathroomIntegratedSafetyLevel level);
  static const char* levelName(BathroomIntegratedSafetyLevel level);
  float bathtubTimeContext(const BathroomBathingSessionEvidence& session) const;
  BathroomIntegratedSafetyLevel desiredLevel(
      float overall_risk,
      float floor_fall,
      float bathtub_immobility) const;
  uint32_t transitionConfirmationMs(
      BathroomIntegratedSafetyLevel current,
      BathroomIntegratedSafetyLevel desired) const;
  void emitIfDue(const BathroomIntegratedSafetyEvidence& evidence);

  BathroomIntegratedSafetyConfig config_;
  TemporalState state_{};
  TemporalState before_current_probe_{};
  uint32_t current_probe_sequence_ = 0;
  uint64_t last_emit_at_us_ = 0;
  bool has_current_probe_ = false;
};

}  // namespace atom::radar
