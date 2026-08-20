#pragma once

#include <cstddef>
#include <cstdint>

#include "bathroom_bathing_session_types.hpp"
#include "bathroom_context_null_calibration_types.hpp"
#include "bathroom_csi_evidence_types.hpp"
#include "bathroom_integrated_safety_types.hpp"
#include "bathroom_respiration_evidence_types.hpp"
#include "bathroom_safety_evidence_types.hpp"

class Preferences;

namespace atom::radar {

struct BathroomContextNullCalibrationConfig {
  float minimum_update_quality = 0.68F;
  float minimum_context_confidence = 0.55F;
  float maximum_safe_floor_fall = 0.18F;
  float maximum_safe_bathtub_immobility = 0.22F;
  float maximum_safe_dangerous_posture = 0.22F;
  float maximum_safe_respiration_loss = 0.18F;
  float maximum_safe_prolonged_bathing = 0.10F;
  float maximum_safe_stale_danger = 0.24F;
  float scale_floor = 0.035F;
  float residual_clip_sigma = 3.0F;
  float tail_start_sigma = 2.5F;
  float maximum_evidence_reduction = 0.30F;
  float danger_lock_memory = 0.55F;
  uint32_t mature_sample_count = 48U;
  uint32_t maximum_sample_count = 65535U;
  uint32_t minimum_checkpoint_updates = 32U;
  uint32_t checkpoint_interval_ms = 5U * 60U * 1000U;
  uint32_t emit_interval_ms = 1000U;
};

enum class BathroomContextNullCalibrationUpdateStatus : uint8_t {
  Accepted = 0,
  NotReady,
  StaleObservation,
};

class BathroomContextNullCalibration {
 public:
  explicit BathroomContextNullCalibration(
      const BathroomContextNullCalibrationConfig& config = {});

  BathroomContextNullCalibrationUpdateStatus update(
      const BathroomCsiEvidence& csi,
      const BathroomSafetyEvidence& safety,
      const BathroomRespirationEvidence& respiration,
      const BathroomBathingSessionEvidence& session,
      const BathroomIntegratedSafetyEvidence& integrated,
      BathroomContextNullCalibrationEvidence& output);

  void reset();

 private:
  static constexpr size_t kContextCount = 7U;
  static constexpr size_t kFeatureCount = 5U;

  enum class FeatureIndex : uint8_t {
    FloorFall = 0,
    BathtubImmobility,
    DangerousPosture,
    RespiratoryMotionLoss,
    OccupiedUnknown,
  };

  struct NullStatistic {
    float mean = 0.0F;
    float variance = 0.0F;
    uint32_t samples = 0U;
  };

  struct __attribute__((packed)) PersistedStatistic {
    float mean;
    float variance;
    uint32_t samples;
  };

  struct __attribute__((packed)) PersistedProfile {
    uint32_t magic;
    uint16_t format_version;
    uint16_t structure_size;
    uint32_t generation;
    PersistedStatistic statistics[kContextCount][kFeatureCount];
    uint32_t crc32;
  };

  static float clamp01(float value);
  static float minimum(float a, float b);
  static float maximum(float a, float b);
  static uint8_t dangerRank(BathroomIntegratedSafetyLevel level);
  static BathroomIntegratedSafetyLevel levelForRank(uint8_t rank);
  static const char* contextName(BathroomNullContext context);
  static const char* levelName(BathroomIntegratedSafetyLevel level);
  BathroomNullContext selectContext(
      const BathroomCsiEvidence& csi,
      const BathroomRespirationEvidence& respiration,
      const BathroomBathingSessionEvidence& session,
      float& confidence) const;
  bool safeUpdateGate(
      BathroomNullContext context,
      float context_confidence,
      const BathroomCsiEvidence& csi,
      const BathroomSafetyEvidence& safety,
      const BathroomRespirationEvidence& respiration,
      const BathroomBathingSessionEvidence& session,
      const BathroomIntegratedSafetyEvidence& integrated,
      float update_quality) const;
  void updateStatistic(NullStatistic& statistic, float value);
  BathroomNullCalibratedFeature calibrateFeature(
      float raw,
      const NullStatistic& statistic,
      bool danger_lock) const;
  BathroomIntegratedSafetyLevel recommendLevel(
      BathroomIntegratedSafetyLevel raw_level,
      float calibrated_risk,
      bool calibration_applied,
      bool danger_lock) const;
  void copyProfiles(
      NullStatistic destination[kContextCount][kFeatureCount],
      const NullStatistic source[kContextCount][kFeatureCount]) const;
  static uint32_t calculateCrc32(const uint8_t* data, size_t length);
  static bool generationIsNewer(uint32_t candidate, uint32_t reference);
  bool validatePersistedProfile(const PersistedProfile& profile) const;
  bool readSlot(
      Preferences& preferences,
      const char* key,
      PersistedProfile& profile) const;
  void applyPersistedProfile(const PersistedProfile& profile);
  PersistedProfile makePersistedProfile(uint32_t generation) const;
  void ensureStorageLoaded();
  bool checkpointIfDue(uint64_t observed_at_us);
  void emitIfDue(const BathroomContextNullCalibrationEvidence& evidence);

  BathroomContextNullCalibrationConfig config_;
  NullStatistic profiles_[kContextCount][kFeatureCount]{};
  NullStatistic before_current_probe_[kContextCount][kFeatureCount]{};
  uint32_t before_current_probe_updates_ = 0U;
  bool before_current_probe_dirty_ = false;
  uint32_t current_probe_sequence_ = 0U;
  uint64_t last_observed_at_us_ = 0U;
  uint64_t last_emit_at_us_ = 0U;
  uint64_t last_checkpoint_at_us_ = 0U;
  uint64_t last_checkpoint_attempt_at_us_ = 0U;
  uint32_t profile_generation_ = 0U;
  uint32_t updates_since_checkpoint_ = 0U;
  uint8_t active_slot_ = 0U;
  bool has_current_probe_ = false;
  bool storage_initialized_ = false;
  bool storage_available_ = false;
  bool persisted_profile_loaded_ = false;
  bool recovered_from_single_slot_ = false;
  bool profile_dirty_ = false;
  bool checkpoint_ok_ = false;
};

}  // namespace atom::radar
