#pragma once

#include <cstdint>

namespace atom::radar {

enum class BathroomSessionState : uint8_t {
  Vacant = 0,
  Entering,
  WashFloor,
  Boundary,
  Bathtub,
  Exiting,
  OccupiedUnknown,
};

const char *bathroomSessionStateName(BathroomSessionState state);

struct BathroomBathingSessionConfig {
  int64_t entry_confirmation_us{2000000};
  int64_t exit_confirmation_us{5000000};
  int64_t bathtub_confirmation_us{1500000};
  int64_t maximum_observed_gap_us{2000000};
  uint32_t prolonged_bathtub_start_ms{15U * 60U * 1000U};
  uint32_t prolonged_bathtub_full_ms{30U * 60U * 1000U};
  uint32_t prolonged_session_start_ms{30U * 60U * 1000U};
  uint32_t prolonged_session_full_ms{60U * 60U * 1000U};
};

enum class BathroomBathingSessionUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomBathingSessionEvidence {
  uint32_t probe_sequence;
  int64_t observed_at_us;
  uint32_t session_id;
  uint32_t observations;
  BathroomSessionState state;
  float vacant_probability;
  float entering_probability;
  float wash_floor_probability;
  float boundary_probability;
  float bathtub_probability;
  float exiting_probability;
  float occupied_unknown_probability;
  float occupancy_evidence;
  float entry_evidence;
  float exit_evidence;
  float state_confidence;
  float transition_confidence;
  float measurement_loss_alternative;
  float prolonged_bathtub_evidence;
  float prolonged_session_evidence;
  float bathtub_immobility_context;
  float analysis_quality;
  uint32_t session_duration_ms;
  uint32_t bathtub_duration_ms;
  uint32_t wash_floor_duration_ms;
  uint32_t boundary_duration_ms;
  uint32_t unobserved_duration_ms;
  uint32_t last_completed_session_duration_ms;
  uint32_t last_completed_bathtub_duration_ms;
  uint16_t bathtub_intervals;
  uint16_t bathtub_reentries;
  bool session_active;
  bool bathtub_interval_active;
  bool entry_pending;
  bool exit_pending;
  bool evidence_ready;
};

}  // namespace atom::radar
