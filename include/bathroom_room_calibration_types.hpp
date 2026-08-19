#pragma once

#include <cstdint>

namespace atom::radar {

enum class BathroomCalibrationCategory : uint8_t {
  None = 0,
  EmptyRoom,
  Bathtub,
  Boundary,
  WashFloor,
  Standing,
  Seated,
  StableRespiration,
  PeriodicNuisance,
};

const char *bathroomCalibrationCategoryName(
    BathroomCalibrationCategory category);

struct BathroomRoomCalibrationConfig {
  uint16_t minimum_mature_samples{30};
  uint16_t checkpoint_interval_updates{6000};
  float minimum_update_quality{0.55F};
  float residual_clip_sigma{3.0F};
  float variance_floor{0.0025F};
  float mature_adaptation_alpha{0.0025F};
};

enum class BathroomRoomCalibrationUpdateStatus : uint8_t {
  Updated = 0,
  ReplacedSameProbe,
  NonMonotonicProbe,
  InvalidObservation,
};

struct BathroomRoomCalibrationEvidence {
  uint32_t probe_sequence;
  int64_t observed_at_us;
  uint32_t profile_generation;
  uint8_t mature_categories;
  BathroomCalibrationCategory last_updated_category;
  float calibrated_empty_room_evidence;
  float calibrated_bathtub_evidence;
  float calibrated_boundary_evidence;
  float calibrated_wash_floor_evidence;
  float calibrated_standing_evidence;
  float calibrated_seated_evidence;
  float calibrated_low_posture_evidence;
  float calibrated_lying_evidence;
  float calibrated_dangerous_posture_evidence;
  float calibrated_respiration_evidence;
  float calibrated_periodic_nuisance_evidence;
  float profile_maturity;
  float profile_drift;
  float update_quality;
  float analysis_quality;
  bool normal_update_frozen;
  bool persisted_profile_loaded;
  bool storage_available;
  bool checkpoint_ok;
  bool profile_dirty;
  bool evidence_ready;
};

}  // namespace atom::radar
