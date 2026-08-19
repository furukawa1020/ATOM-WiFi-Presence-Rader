#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_csi_evidence_types.hpp>
#include <bathroom_respiration_evidence_types.hpp>
#include <bathroom_room_calibration_types.hpp>
#include <bathroom_safety_evidence_types.hpp>
#include <bathroom_spatial_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomRoomCalibration final {
 public:
  explicit BathroomRoomCalibration(BathroomRoomCalibrationConfig config = {});

  BathroomRoomCalibrationUpdateStatus update(
      const FusedCsiObservation &observation, int64_t observed_at_us,
      const BathroomCsiEvidence &bathroom_evidence,
      const BathroomSafetyEvidence &safety_evidence,
      const BathroomSpatialEvidence &spatial_evidence,
      const BathroomRespirationEvidence &respiration_evidence,
      BathroomRoomCalibrationEvidence &calibrated_evidence);
  bool checkpoint();
  void resetRuntime();

 private:
  static constexpr uint32_t kProfileMagic = 0x42435250U;
  static constexpr uint16_t kProfileVersion = 1;
  static constexpr std::size_t kFeatureCount = 24;
  static constexpr std::size_t kCategoryCount = 8;

  struct Prototype {
    uint32_t samples;
    float mean[kFeatureCount];
    float variance[kFeatureCount];
    float drift;
    float update_quality;
  };

  struct PersistedProfile {
    uint32_t magic;
    uint16_t version;
    uint16_t feature_count;
    uint16_t category_count;
    uint16_t reserved;
    uint32_t generation;
    Prototype prototypes[kCategoryCount];
    uint32_t crc32;
  };

  void ensureLoaded();
  void initializeProfile();
  bool loadProfile();
  bool validateProfile(const PersistedProfile &profile) const;
  bool saveProfile();
  void buildFeatures(const FusedCsiObservation &observation,
                     const BathroomCsiEvidence &bathroom_evidence,
                     const BathroomSpatialEvidence &spatial_evidence,
                     const BathroomRespirationEvidence &respiration_evidence,
                     float features[kFeatureCount]) const;
  bool updatePrototype(std::size_t category,
                       const float features[kFeatureCount], float quality);
  float similarity(std::size_t category,
                   const float features[kFeatureCount]) const;
  float maturity(std::size_t category) const;
  void buildEvidence(
      uint32_t probe_sequence, int64_t observed_at_us,
      const float features[kFeatureCount],
      const BathroomSpatialEvidence &spatial_evidence,
      const BathroomRespirationEvidence &respiration_evidence,
      bool normal_update_frozen,
      BathroomRoomCalibrationEvidence &calibrated_evidence) const;
  void emitJsonIfDue(const BathroomRoomCalibrationEvidence &evidence);
  static uint32_t crc32(const uint8_t *data, std::size_t length);
  static float clamp01(float value);
  static float maximum(float first, float second);
  static float absolute(float value);

  BathroomRoomCalibrationConfig config_;
  PersistedProfile profile_{};
  bool load_attempted_{false};
  bool persisted_profile_loaded_{false};
  bool storage_available_{false};
  bool last_checkpoint_ok_{true};
  bool dirty_{false};
  bool has_probe_{false};
  uint32_t last_probe_sequence_{0};
  uint32_t updates_since_checkpoint_{0};
  BathroomCalibrationCategory last_updated_category_{
      BathroomCalibrationCategory::None};
  int64_t last_emit_at_us_{-1};
};

}  // namespace atom::radar
