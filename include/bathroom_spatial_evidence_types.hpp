#pragma once

#include <cstdint>

namespace atom::radar {

enum class BathroomSpatialZone : uint8_t {
  Unknown = 0,
  Bathtub,
  Boundary,
  WashFloor,
};

const char *bathroomSpatialZoneName(BathroomSpatialZone zone);

struct BathroomSpatialEvidenceConfig {
  uint8_t bathtub_receiver_id{1};
  uint8_t wash_floor_receiver_id{2};
  uint8_t boundary_receiver_id{3};
  uint8_t maximum_sequence_skew{2};
  uint8_t minimum_spatial_links{2};
  int64_t link_freshness_us{500000};
  float baseline_adaptation_alpha{0.001F};
  uint16_t baseline_maturity_samples{300};
};

enum class BathroomSpatialPacketStatus : uint8_t {
  Accepted = 0,
  ReplacedSameProbe,
  NotCsiObservation,
  WrongSystem,
  NonMonotonicProbe,
  LinkCapacityExceeded,
};

enum class BathroomSpatialEvidenceUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomSpatialEvidence {
  uint32_t probe_sequence;
  int64_t observed_at_us;
  uint8_t active_links;
  BathroomSpatialZone dominant_zone;
  float bathtub_position_evidence;
  float boundary_position_evidence;
  float wash_floor_position_evidence;
  float position_uncertain_evidence;
  float standing_posture_evidence;
  float seated_posture_evidence;
  float low_posture_evidence;
  float lying_posture_evidence;
  float dangerous_posture_evidence;
  float link_consistency;
  float spatial_separation;
  float role_coverage;
  float baseline_maturity;
  float analysis_quality;
  bool spatial_links_ready;
  bool baseline_ready;
  bool evidence_ready;
};

}  // namespace atom::radar
