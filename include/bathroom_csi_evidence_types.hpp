#pragma once

#include <cstddef>
#include <cstdint>

namespace atom::radar {

struct BathroomCsiEvidenceConfig {
  uint16_t sample_rate_hz{10};
  uint16_t short_window_points{10};
  uint16_t medium_window_points{50};
  uint16_t long_window_points{300};
  uint16_t minimum_ready_points{20};
  float periodic_motion_threshold{0.08F};
  float broadband_threshold{0.15F};
  float baseline_shift_threshold{0.08F};
};

enum class BathroomCsiEvidenceUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomCsiEvidence {
  uint32_t probe_sequence;
  int64_t observed_at_us;
  uint16_t history_points;
  float fan_like_evidence;
  float shower_like_evidence;
  float water_drift_like_evidence;
  float door_transient_like_evidence;
  float human_motion_evidence;
  float nuisance_reduced_human_motion;
  float unexplained_innovation;
  float nuisance_confidence;
  float analysis_quality;
  bool short_window_ready;
  bool medium_window_ready;
  bool long_window_ready;
  bool evidence_ready;
};

}  // namespace atom::radar
