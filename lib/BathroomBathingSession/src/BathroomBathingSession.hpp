#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_bathing_session_types.hpp>
#include <bathroom_csi_evidence_types.hpp>
#include <bathroom_respiration_evidence_types.hpp>
#include <bathroom_room_calibration_types.hpp>
#include <bathroom_safety_evidence_types.hpp>
#include <bathroom_spatial_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomBathingSessionTracker final {
 public:
  explicit BathroomBathingSessionTracker(
      BathroomBathingSessionConfig config = {});

  BathroomBathingSessionUpdateStatus update(
      const FusedCsiObservation &observation, int64_t observed_at_us,
      const BathroomCsiEvidence &bathroom_evidence,
      const BathroomSafetyEvidence &safety_evidence,
      const BathroomSpatialEvidence &spatial_evidence,
      const BathroomRespirationEvidence &respiration_evidence,
      const BathroomRoomCalibrationEvidence &calibrated_evidence,
      BathroomBathingSessionEvidence &session_evidence);
  void reset();

 private:
  static constexpr std::size_t kStateCount = 7;

  void updateStateProbabilities(
      const FusedCsiObservation &observation,
      const BathroomCsiEvidence &bathroom_evidence,
      const BathroomSafetyEvidence &safety_evidence,
      const BathroomSpatialEvidence &spatial_evidence,
      const BathroomRespirationEvidence &respiration_evidence,
      const BathroomRoomCalibrationEvidence &calibrated_evidence,
      float measurement_loss);
  void updateDurations(int64_t observed_at_us, float measurement_loss);
  void updateSessionLifecycle(
      int64_t observed_at_us, float measurement_loss, float danger_context);
  void buildEvidence(
      uint32_t probe_sequence, int64_t observed_at_us,
      const BathroomSafetyEvidence &safety_evidence,
      const BathroomRespirationEvidence &respiration_evidence,
      const BathroomRoomCalibrationEvidence &calibrated_evidence,
      float measurement_loss,
      BathroomBathingSessionEvidence &session_evidence) const;
  void emitJsonIfDue(const BathroomBathingSessionEvidence &evidence);
  uint64_t currentBathtubDurationUs(int64_t observed_at_us) const;
  static uint32_t milliseconds(uint64_t microseconds);
  static float durationRamp(uint32_t value, uint32_t start, uint32_t full);
  static float clamp01(float value);
  static float maximum(float first, float second);

  BathroomBathingSessionConfig config_;
  float state_probability_[kStateCount]{};
  float state_before_probe_[kStateCount]{};
  bool has_probe_{false};
  uint32_t last_probe_sequence_{0};
  int64_t last_observed_at_us_{-1};
  uint32_t observations_{0};

  bool session_active_{false};
  bool bathtub_interval_active_{false};
  uint32_t session_id_{0};
  int64_t session_started_at_us_{-1};
  int64_t bathtub_interval_started_at_us_{-1};
  int64_t entry_candidate_at_us_{-1};
  int64_t exit_candidate_at_us_{-1};
  int64_t bathtub_entry_candidate_at_us_{-1};
  int64_t bathtub_exit_candidate_at_us_{-1};
  uint64_t accumulated_bathtub_us_{0};
  uint64_t wash_floor_duration_us_{0};
  uint64_t boundary_duration_us_{0};
  uint64_t unobserved_duration_us_{0};
  uint64_t last_completed_session_us_{0};
  uint64_t last_completed_bathtub_us_{0};
  uint16_t bathtub_intervals_{0};
  uint16_t bathtub_reentries_{0};
  float latest_entry_evidence_{0.0F};
  float latest_exit_evidence_{0.0F};
  float latest_state_confidence_{0.0F};
  float latest_transition_confidence_{0.0F};
  int64_t last_emit_at_us_{-1};
};

}  // namespace atom::radar
