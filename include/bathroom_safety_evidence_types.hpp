#pragma once

#include <cstddef>
#include <cstdint>

namespace atom::radar {

struct BathroomSafetyEvidenceConfig {
  uint16_t sample_rate_hz{10};
  uint16_t history_points{300};
  uint16_t pre_impact_points{30};
  uint16_t impact_settling_points{5};
  uint16_t immobility_maturity_points{20};
  uint16_t recovery_window_points{50};
};

enum class BathroomSafetyEvidenceUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomSafetyEvidence {
  uint32_t probe_sequence;
  int64_t evaluated_at_us;
  uint32_t impact_probe_sequence;
  uint32_t milliseconds_since_impact;
  uint16_t history_points;
  float impact_evidence;
  float pre_impact_human_motion;
  float relocation_evidence;
  float post_impact_immobility;
  float recovery_motion;
  float respiration_after_impact;
  float door_or_object_alternative;
  float persistent_nuisance_alternative;
  float fall_sequence_evidence;
  float dangerous_immobility_evidence;
  float analysis_quality;
  bool short_history_ready;
  bool medium_history_ready;
  bool long_history_ready;
  bool temporal_sequence_ready;
  bool evidence_ready;
};

}  // namespace atom::radar
