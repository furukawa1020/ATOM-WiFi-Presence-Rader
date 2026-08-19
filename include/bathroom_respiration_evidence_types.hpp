#pragma once

#include <cstdint>

namespace atom::radar {

struct BathroomRespirationEvidenceConfig {
  uint16_t history_points{300};
  uint16_t short_window_points{50};
  uint16_t loss_maturity_points{20};
  uint8_t maximum_sequence_skew{2};
  int64_t link_freshness_us{500000};
  float minimum_link_confidence{0.08F};
  float minimum_presence_confidence{0.18F};
};

enum class BathroomRespirationPacketStatus : uint8_t {
  Accepted = 0,
  ReplacedSameProbe,
  NotCsiObservation,
  WrongSystem,
  NonMonotonicProbe,
  LinkCapacityExceeded,
};

enum class BathroomRespirationEvidenceUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomRespirationEvidence {
  uint32_t probe_sequence;
  int64_t observed_at_us;
  uint16_t history_points;
  uint8_t active_links;
  float tracked_rate_normalized;
  float rate_stability;
  float respiration_micro_motion_evidence;
  float stable_respiration_evidence;
  float weak_respiration_evidence;
  float respiration_loss_evidence;
  float measurement_loss_alternative;
  float periodic_nuisance_alternative;
  float short_continuity;
  float long_continuity;
  float multi_link_support;
  float position_robustness;
  float analysis_quality;
  bool multi_link_ready;
  bool short_history_ready;
  bool long_history_ready;
  bool evidence_ready;
};

}  // namespace atom::radar
