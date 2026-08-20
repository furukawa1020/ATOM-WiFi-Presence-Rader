#include "BathroomContextNullCalibration.hpp"

#include <algorithm>
#include <cstddef>
#include <cmath>

#include <Arduino.h>
#include <Preferences.h>

namespace atom::radar {
namespace {

constexpr uint64_t kUsPerMs = 1000ULL;
constexpr uint32_t kProfileMagic = 0x4E43414CUL;
constexpr uint16_t kProfileFormatVersion = 1U;
constexpr char kStorageNamespace[] = "ctxnull";
constexpr char kSlotAKey[] = "profile_a";
constexpr char kSlotBKey[] = "profile_b";

size_t contextIndex(BathroomNullContext context) {
  return static_cast<size_t>(context);
}

uint32_t elapsedMilliseconds(uint64_t now_us, uint64_t since_us) {
  if (since_us == 0U || now_us <= since_us) {
    return 0U;
  }
  const uint64_t elapsed_ms = (now_us - since_us) / kUsPerMs;
  return elapsed_ms > UINT32_MAX ? UINT32_MAX
                                 : static_cast<uint32_t>(elapsed_ms);
}

}  // namespace

BathroomContextNullCalibration::BathroomContextNullCalibration(
    const BathroomContextNullCalibrationConfig& config)
    : config_(config) {}

float BathroomContextNullCalibration::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomContextNullCalibration::minimum(float a, float b) {
  return a < b ? a : b;
}

float BathroomContextNullCalibration::maximum(float a, float b) {
  return a > b ? a : b;
}

uint8_t BathroomContextNullCalibration::dangerRank(
    BathroomIntegratedSafetyLevel level) {
  switch (level) {
    case BathroomIntegratedSafetyLevel::Attention:
      return 1U;
    case BathroomIntegratedSafetyLevel::CheckRequired:
      return 2U;
    case BathroomIntegratedSafetyLevel::Urgent:
      return 3U;
    case BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended:
      return 4U;
    case BathroomIntegratedSafetyLevel::Normal:
    case BathroomIntegratedSafetyLevel::MeasurementUnavailable:
    default:
      return 0U;
  }
}

BathroomIntegratedSafetyLevel BathroomContextNullCalibration::levelForRank(
    uint8_t rank) {
  switch (rank) {
    case 1U:
      return BathroomIntegratedSafetyLevel::Attention;
    case 2U:
      return BathroomIntegratedSafetyLevel::CheckRequired;
    case 3U:
      return BathroomIntegratedSafetyLevel::Urgent;
    case 4U:
      return BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended;
    case 0U:
    default:
      return BathroomIntegratedSafetyLevel::Normal;
  }
}

const char* BathroomContextNullCalibration::contextName(
    BathroomNullContext context) {
  switch (context) {
    case BathroomNullContext::EmptyRoom:
      return "empty_room";
    case BathroomNullContext::OccupiedQuiet:
      return "occupied_quiet";
    case BathroomNullContext::Shower:
      return "shower";
    case BathroomNullContext::VentilationFan:
      return "ventilation_fan";
    case BathroomNullContext::WaterDynamics:
      return "water_dynamics";
    case BathroomNullContext::DoorTransition:
      return "door_transition";
    case BathroomNullContext::Unknown:
    default:
      return "unknown";
  }
}

const char* BathroomContextNullCalibration::levelName(
    BathroomIntegratedSafetyLevel level) {
  switch (level) {
    case BathroomIntegratedSafetyLevel::Attention:
      return "attention";
    case BathroomIntegratedSafetyLevel::CheckRequired:
      return "check_required";
    case BathroomIntegratedSafetyLevel::Urgent:
      return "urgent";
    case BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended:
      return "emergency_notification_recommended";
    case BathroomIntegratedSafetyLevel::MeasurementUnavailable:
      return "measurement_unavailable";
    case BathroomIntegratedSafetyLevel::Normal:
    default:
      return "normal";
  }
}

BathroomNullContext BathroomContextNullCalibration::selectContext(
    const BathroomCsiEvidence& csi,
    const BathroomRespirationEvidence& respiration,
    const BathroomBathingSessionEvidence& session,
    float& confidence) const {
  BathroomNullContext selected = BathroomNullContext::Unknown;
  confidence = 0.0F;

  const struct {
    BathroomNullContext context;
    float evidence;
  } nuisance_candidates[] = {
      {BathroomNullContext::DoorTransition,
       csi.door_transient_like_evidence},
      {BathroomNullContext::Shower, csi.shower_like_evidence},
      {BathroomNullContext::VentilationFan, csi.fan_like_evidence},
      {BathroomNullContext::WaterDynamics, csi.water_drift_like_evidence},
  };
  for (const auto& candidate : nuisance_candidates) {
    if (candidate.evidence > confidence) {
      selected = candidate.context;
      confidence = candidate.evidence;
    }
  }
  if (confidence >= config_.minimum_context_confidence) {
    return selected;
  }

  const float occupied_quiet =
      minimum(session.occupancy_evidence,
              maximum(respiration.stable_respiration_evidence,
                      respiration.respiration_micro_motion_evidence));
  if (session.vacant_probability >= occupied_quiet &&
      session.vacant_probability >= config_.minimum_context_confidence) {
    confidence = session.vacant_probability;
    return BathroomNullContext::EmptyRoom;
  }
  if (occupied_quiet >= config_.minimum_context_confidence) {
    confidence = occupied_quiet;
    return BathroomNullContext::OccupiedQuiet;
  }
  confidence = maximum(confidence,
                       maximum(session.vacant_probability, occupied_quiet));
  return BathroomNullContext::Unknown;
}

bool BathroomContextNullCalibration::safeUpdateGate(
    BathroomNullContext context,
    float context_confidence,
    const BathroomCsiEvidence& csi,
    const BathroomSafetyEvidence& safety,
    const BathroomRespirationEvidence& respiration,
    const BathroomBathingSessionEvidence& session,
    const BathroomIntegratedSafetyEvidence& integrated,
    float update_quality) const {
  if (context == BathroomNullContext::Unknown ||
      context_confidence < config_.minimum_context_confidence ||
      update_quality < config_.minimum_update_quality ||
      !integrated.physically_observable ||
      integrated.safety_level != BathroomIntegratedSafetyLevel::Normal ||
      integrated.floor_fall_evidence > config_.maximum_safe_floor_fall ||
      integrated.bathtub_immobility_evidence >
          config_.maximum_safe_bathtub_immobility ||
      integrated.dangerous_posture_evidence >
          config_.maximum_safe_dangerous_posture ||
      integrated.respiratory_motion_loss_evidence >
          config_.maximum_safe_respiration_loss ||
      integrated.prolonged_bathing_evidence >
          config_.maximum_safe_prolonged_bathing ||
      integrated.stale_danger_memory > config_.maximum_safe_stale_danger ||
      safety.impact_evidence > config_.maximum_safe_floor_fall ||
      safety.dangerous_immobility_evidence >
          config_.maximum_safe_bathtub_immobility) {
    return false;
  }

  if (context == BathroomNullContext::EmptyRoom) {
    return session.vacant_probability >= 0.70F &&
           csi.human_motion_evidence <= 0.20F;
  }
  if (context == BathroomNullContext::OccupiedQuiet) {
    return session.occupancy_evidence >= 0.55F &&
           respiration.stable_respiration_evidence >= 0.45F &&
           csi.human_motion_evidence <= 0.38F;
  }
  return session.vacant_probability >= 0.60F ||
         respiration.stable_respiration_evidence >= 0.45F;
}

bool BathroomContextNullCalibration::driftObservationGate(
    BathroomNullContext context,
    float context_confidence,
    const BathroomCsiEvidence& csi,
    const BathroomSafetyEvidence& safety,
    const BathroomRespirationEvidence& respiration,
    const BathroomBathingSessionEvidence& session,
    const BathroomIntegratedSafetyEvidence& integrated,
    float update_quality) const {
  if (context == BathroomNullContext::Unknown ||
      context_confidence < config_.minimum_context_confidence ||
      update_quality < config_.minimum_update_quality ||
      !integrated.physically_observable ||
      dangerRank(integrated.safety_level) >
          dangerRank(BathroomIntegratedSafetyLevel::CheckRequired) ||
      integrated.stale_danger_memory >= 0.45F ||
      integrated.prolonged_bathing_evidence >= 0.15F ||
      integrated.measurement_failure_evidence >= 0.35F ||
      safety.impact_evidence >= 0.18F ||
      safety.fall_sequence_evidence >= 0.22F ||
      safety.dangerous_immobility_evidence >= 0.25F ||
      respiration.measurement_loss_alternative >= 0.30F) {
    return false;
  }
  if (context == BathroomNullContext::EmptyRoom) {
    return session.vacant_probability >= 0.70F &&
           csi.human_motion_evidence <= 0.25F;
  }
  if (context == BathroomNullContext::OccupiedQuiet) {
    return session.occupancy_evidence >= 0.55F &&
           respiration.stable_respiration_evidence >= 0.45F &&
           csi.human_motion_evidence <= 0.42F;
  }
  return session.vacant_probability >= 0.60F ||
         respiration.stable_respiration_evidence >= 0.45F;
}

bool BathroomContextNullCalibration::shadowUpdateGate(
    BathroomNullContext context,
    const BathroomCsiEvidence& csi,
    const BathroomSafetyEvidence& safety,
    const BathroomRespirationEvidence& respiration,
    const BathroomBathingSessionEvidence& session,
    const BathroomIntegratedSafetyEvidence& integrated) const {
  if (dangerRank(integrated.safety_level) >
          dangerRank(BathroomIntegratedSafetyLevel::Attention) ||
      integrated.floor_fall_evidence >= 0.26F ||
      integrated.bathtub_immobility_evidence >= 0.30F ||
      integrated.dangerous_posture_evidence >= 0.30F ||
      integrated.respiratory_motion_loss_evidence >= 0.25F ||
      integrated.stale_danger_memory >= 0.35F ||
      safety.impact_evidence >= 0.15F ||
      safety.dangerous_immobility_evidence >= 0.22F) {
    return false;
  }
  if (context == BathroomNullContext::EmptyRoom) {
    return session.vacant_probability >= 0.75F &&
           csi.human_motion_evidence <= 0.20F;
  }
  if (context == BathroomNullContext::OccupiedQuiet) {
    return session.occupancy_evidence >= 0.60F &&
           respiration.stable_respiration_evidence >= 0.55F &&
           csi.human_motion_evidence <= 0.35F;
  }
  return session.vacant_probability >= 0.65F ||
         respiration.stable_respiration_evidence >= 0.55F;
}

bool BathroomContextNullCalibration::profileMature(
    const NullStatistic profile[kFeatureCount],
    uint32_t required_samples) const {
  for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
    if (profile[feature].samples < required_samples) {
      return false;
    }
  }
  return true;
}

float BathroomContextNullCalibration::profileDistance(
    const float values[kFeatureCount],
    const NullStatistic profile[kFeatureCount]) const {
  float normalized_residuals[kFeatureCount]{};
  for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
    const float scale = std::sqrt(maximum(
        profile[feature].variance,
        config_.scale_floor * config_.scale_floor));
    normalized_residuals[feature] =
        std::clamp(std::fabs(values[feature] - profile[feature].mean) /
                       scale,
                   0.0F, 12.0F);
  }
  std::sort(normalized_residuals,
            normalized_residuals + kFeatureCount);
  return normalized_residuals[kFeatureCount / 2U];
}

uint32_t BathroomContextNullCalibration::profileMinimumSamples(
    const NullStatistic profile[kFeatureCount]) const {
  uint32_t samples = UINT32_MAX;
  for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
    samples = std::min(samples, profile[feature].samples);
  }
  return samples == UINT32_MAX ? 0U : samples;
}

void BathroomContextNullCalibration::clearShadowContext(size_t context) {
  for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
    shadow_profiles_[context][feature] = {};
  }
  drift_states_[context].shadow_stable_samples = 0U;
}

void BathroomContextNullCalibration::updateStatistic(
    NullStatistic& statistic,
    float value) {
  value = clamp01(value);
  if (statistic.samples == 0U) {
    statistic.mean = value;
    statistic.variance = config_.scale_floor * config_.scale_floor;
    statistic.samples = 1U;
    return;
  }

  const float scale =
      std::sqrt(maximum(statistic.variance,
                        config_.scale_floor * config_.scale_floor));
  const float residual = value - statistic.mean;
  const float clipped_residual =
      std::clamp(residual, -config_.residual_clip_sigma * scale,
                 config_.residual_clip_sigma * scale);
  const float reciprocal_count =
      1.0F / static_cast<float>(statistic.samples + 1U);
  const float alpha = maximum(0.005F, reciprocal_count);
  statistic.mean = clamp01(statistic.mean + alpha * clipped_residual);
  statistic.variance = maximum(
      config_.scale_floor * config_.scale_floor,
      (1.0F - alpha) * statistic.variance +
          alpha * clipped_residual * clipped_residual);
  if (statistic.samples < config_.maximum_sample_count) {
    ++statistic.samples;
  }
}

BathroomNullCalibratedFeature
BathroomContextNullCalibration::calibrateFeature(
    float raw,
    const NullStatistic& statistic,
    bool danger_lock) const {
  BathroomNullCalibratedFeature result{};
  result.raw_evidence = clamp01(raw);
  result.baseline_mean = statistic.mean;
  result.baseline_scale =
      std::sqrt(maximum(statistic.variance,
                        config_.scale_floor * config_.scale_floor));
  result.samples = statistic.samples;
  result.maturity =
      config_.mature_sample_count == 0U
          ? 1.0F
          : clamp01(static_cast<float>(statistic.samples) /
                    static_cast<float>(config_.mature_sample_count));

  const float z_score =
      (result.raw_evidence - result.baseline_mean) / result.baseline_scale;
  result.standardized_excess =
      std::clamp(z_score - config_.tail_start_sigma, 0.0F, 12.0F);
  if (result.maturity < 1.0F || danger_lock) {
    result.calibrated_evidence = result.raw_evidence;
    return result;
  }

  const float tail_evidence =
      clamp01(result.standardized_excess / 4.0F);
  const float reduction_floor =
      result.raw_evidence *
      (1.0F - config_.maximum_evidence_reduction * result.maturity);
  result.calibrated_evidence = maximum(reduction_floor, tail_evidence);
  return result;
}

BathroomIntegratedSafetyLevel BathroomContextNullCalibration::recommendLevel(
    BathroomIntegratedSafetyLevel raw_level,
    float calibrated_risk,
    bool calibration_applied,
    bool danger_lock) const {
  if (!calibration_applied || danger_lock ||
      raw_level == BathroomIntegratedSafetyLevel::MeasurementUnavailable) {
    return raw_level;
  }

  uint8_t calibrated_rank = 0U;
  if (calibrated_risk >= 0.82F) {
    calibrated_rank = 4U;
  } else if (calibrated_risk >= 0.62F) {
    calibrated_rank = 3U;
  } else if (calibrated_risk >= 0.43F) {
    calibrated_rank = 2U;
  } else if (calibrated_risk >= 0.26F) {
    calibrated_rank = 1U;
  }

  const uint8_t raw_rank = dangerRank(raw_level);
  if (calibrated_rank >= raw_rank || raw_rank == 0U) {
    return raw_level;
  }
  return levelForRank(maximum(static_cast<float>(calibrated_rank),
                              static_cast<float>(raw_rank - 1U)));
}

void BathroomContextNullCalibration::copyProfiles(
    NullStatistic destination[kContextCount][kFeatureCount],
    const NullStatistic source[kContextCount][kFeatureCount]) const {
  for (size_t context = 0U; context < kContextCount; ++context) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      destination[context][feature] = source[context][feature];
    }
  }
}

uint32_t BathroomContextNullCalibration::calculateCrc32(
    const uint8_t* data,
    size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0U; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0U; bit < 8U; ++bit) {
      const uint32_t mask =
          static_cast<uint32_t>(-static_cast<int32_t>(crc & 1U));
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

bool BathroomContextNullCalibration::generationIsNewer(
    uint32_t candidate,
    uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

bool BathroomContextNullCalibration::validatePersistedProfile(
    const PersistedProfile& profile) const {
  if (profile.magic != kProfileMagic ||
      profile.format_version != kProfileFormatVersion ||
      profile.structure_size != sizeof(PersistedProfile)) {
    return false;
  }
  const uint32_t expected_crc = calculateCrc32(
      reinterpret_cast<const uint8_t*>(&profile),
      offsetof(PersistedProfile, crc32));
  if (profile.crc32 != expected_crc) {
    return false;
  }

  for (size_t context = 0U; context < kContextCount; ++context) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      const PersistedStatistic& statistic =
          profile.statistics[context][feature];
      if (!std::isfinite(statistic.mean) ||
          !std::isfinite(statistic.variance) ||
          statistic.mean < 0.0F || statistic.mean > 1.0F ||
          statistic.variance < 0.0F || statistic.variance > 1.0F ||
          statistic.samples > config_.maximum_sample_count) {
        return false;
      }
      if (statistic.samples > 0U &&
          statistic.variance <
              config_.scale_floor * config_.scale_floor) {
        return false;
      }
    }
  }
  return true;
}

bool BathroomContextNullCalibration::readSlot(
    Preferences& preferences,
    const char* key,
    PersistedProfile& profile) const {
  if (preferences.getBytesLength(key) != sizeof(PersistedProfile)) {
    return false;
  }
  PersistedProfile candidate{};
  if (preferences.getBytes(key, &candidate, sizeof(candidate)) !=
      sizeof(candidate)) {
    return false;
  }
  if (!validatePersistedProfile(candidate)) {
    return false;
  }
  profile = candidate;
  return true;
}

void BathroomContextNullCalibration::applyPersistedProfile(
    const PersistedProfile& profile) {
  for (size_t context = 0U; context < kContextCount; ++context) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      profiles_[context][feature].mean =
          profile.statistics[context][feature].mean;
      profiles_[context][feature].variance =
          profile.statistics[context][feature].variance;
      profiles_[context][feature].samples =
          profile.statistics[context][feature].samples;
    }
  }
}

BathroomContextNullCalibration::PersistedProfile
BathroomContextNullCalibration::makePersistedProfile(
    uint32_t generation) const {
  PersistedProfile profile{};
  profile.magic = kProfileMagic;
  profile.format_version = kProfileFormatVersion;
  profile.structure_size = sizeof(PersistedProfile);
  profile.generation = generation;
  for (size_t context = 0U; context < kContextCount; ++context) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      profile.statistics[context][feature].mean =
          profiles_[context][feature].mean;
      profile.statistics[context][feature].variance =
          profiles_[context][feature].variance;
      profile.statistics[context][feature].samples =
          profiles_[context][feature].samples;
    }
  }
  profile.crc32 = calculateCrc32(
      reinterpret_cast<const uint8_t*>(&profile),
      offsetof(PersistedProfile, crc32));
  return profile;
}

void BathroomContextNullCalibration::ensureStorageLoaded() {
  if (storage_initialized_) {
    return;
  }
  storage_initialized_ = true;
  Preferences preferences;
  if (!preferences.begin(kStorageNamespace, true)) {
    storage_available_ = false;
    checkpoint_ok_ = false;
    return;
  }
  storage_available_ = true;

  PersistedProfile slot_a{};
  PersistedProfile slot_b{};
  const bool slot_a_valid = readSlot(preferences, kSlotAKey, slot_a);
  const bool slot_b_valid = readSlot(preferences, kSlotBKey, slot_b);
  preferences.end();

  if (!slot_a_valid && !slot_b_valid) {
    checkpoint_ok_ = true;
    return;
  }

  const bool select_b =
      slot_b_valid &&
      (!slot_a_valid ||
       generationIsNewer(slot_b.generation, slot_a.generation));
  const PersistedProfile& selected = select_b ? slot_b : slot_a;
  applyPersistedProfile(selected);
  profile_generation_ = selected.generation;
  active_slot_ = select_b ? 1U : 0U;
  persisted_profile_loaded_ = true;
  recovered_from_single_slot_ = slot_a_valid != slot_b_valid;
  checkpoint_ok_ = true;
}

bool BathroomContextNullCalibration::checkpointIfDue(
    uint64_t observed_at_us) {
  if (last_checkpoint_attempt_at_us_ == 0U) {
    last_checkpoint_attempt_at_us_ = observed_at_us;
    if (persisted_profile_loaded_) {
      last_checkpoint_at_us_ = observed_at_us;
    }
    return checkpoint_ok_;
  }
  if (!profile_dirty_ ||
      updates_since_checkpoint_ < config_.minimum_checkpoint_updates ||
      elapsedMilliseconds(observed_at_us,
                          last_checkpoint_attempt_at_us_) <
          config_.checkpoint_interval_ms) {
    return checkpoint_ok_;
  }
  last_checkpoint_attempt_at_us_ = observed_at_us;

  const uint32_t next_generation = profile_generation_ + 1U;
  const PersistedProfile candidate =
      makePersistedProfile(next_generation);
  const uint8_t target_slot = active_slot_ == 0U ? 1U : 0U;
  const char* target_key =
      target_slot == 0U ? kSlotAKey : kSlotBKey;

  Preferences preferences;
  if (!preferences.begin(kStorageNamespace, false)) {
    storage_available_ = false;
    checkpoint_ok_ = false;
    return false;
  }
  storage_available_ = true;
  const bool write_ok =
      preferences.putBytes(target_key, &candidate, sizeof(candidate)) ==
      sizeof(candidate);
  PersistedProfile verified{};
  const bool verify_ok =
      write_ok && readSlot(preferences, target_key, verified) &&
      verified.generation == next_generation;
  preferences.end();
  checkpoint_ok_ = verify_ok;
  if (!verify_ok) {
    return false;
  }

  active_slot_ = target_slot;
  profile_generation_ = next_generation;
  updates_since_checkpoint_ = 0U;
  profile_dirty_ = false;
  persisted_profile_loaded_ = true;
  recovered_from_single_slot_ = false;
  last_checkpoint_at_us_ = observed_at_us;
  return true;
}

BathroomContextNullCalibrationUpdateStatus
BathroomContextNullCalibration::update(
    const BathroomCsiEvidence& csi,
    const BathroomSafetyEvidence& safety,
    const BathroomRespirationEvidence& respiration,
    const BathroomBathingSessionEvidence& session,
    const BathroomIntegratedSafetyEvidence& integrated,
    BathroomContextNullCalibrationEvidence& output) {
  if (!csi.evidence_ready || !safety.evidence_ready ||
      !respiration.evidence_ready || !session.evidence_ready ||
      !integrated.evidence_ready) {
    return BathroomContextNullCalibrationUpdateStatus::NotReady;
  }
  if (has_current_probe_ &&
      integrated.probe_sequence != current_probe_sequence_ &&
      integrated.observed_at_us < last_observed_at_us_) {
    return BathroomContextNullCalibrationUpdateStatus::StaleObservation;
  }

  ensureStorageLoaded();
  const bool replacing_current_probe =
      has_current_probe_ &&
      integrated.probe_sequence == current_probe_sequence_;
  if (!replacing_current_probe) {
    checkpointIfDue(integrated.observed_at_us);
  }

  if (has_current_probe_ &&
      integrated.probe_sequence == current_probe_sequence_) {
    copyProfiles(profiles_, before_current_probe_);
    copyProfiles(shadow_profiles_, before_current_probe_shadow_);
    for (size_t index = 0U; index < kContextCount; ++index) {
      drift_states_[index] = before_current_probe_drift_[index];
    }
    updates_since_checkpoint_ = before_current_probe_updates_;
    profile_dirty_ = before_current_probe_dirty_;
  } else {
    copyProfiles(before_current_probe_, profiles_);
    copyProfiles(before_current_probe_shadow_, shadow_profiles_);
    for (size_t index = 0U; index < kContextCount; ++index) {
      before_current_probe_drift_[index] = drift_states_[index];
    }
    before_current_probe_updates_ = updates_since_checkpoint_;
    before_current_probe_dirty_ = profile_dirty_;
    current_probe_sequence_ = integrated.probe_sequence;
    has_current_probe_ = true;
  }

  float context_confidence = 0.0F;
  const BathroomNullContext context =
      selectContext(csi, respiration, session, context_confidence);
  const float update_quality = minimum(
      integrated.analysis_quality,
      minimum(csi.analysis_quality,
              minimum(respiration.analysis_quality, session.analysis_quality)));
  const bool danger_lock =
      dangerRank(integrated.safety_level) >=
          dangerRank(BathroomIntegratedSafetyLevel::Urgent) ||
      integrated.stale_danger_memory >= config_.danger_lock_memory;
  const bool base_update_allowed =
      safeUpdateGate(context, context_confidence, csi, safety, respiration,
                     session, integrated, update_quality);

  const float raw_values[kFeatureCount] = {
      integrated.floor_fall_evidence,
      integrated.bathtub_immobility_evidence,
      integrated.dangerous_posture_evidence,
      integrated.respiratory_motion_loss_evidence,
      integrated.occupied_unknown_evidence,
  };
  const size_t selected_context = contextIndex(context);
  DriftContextState& drift_state = drift_states_[selected_context];
  const bool drift_monitor_allowed =
      driftObservationGate(context, context_confidence, csi, safety,
                           respiration, session, integrated, update_quality) &&
      profileMature(profiles_[selected_context],
                    config_.mature_sample_count) &&
      !danger_lock;
  bool entered_quarantine = false;
  if (drift_monitor_allowed) {
    const float distance =
        profileDistance(raw_values, profiles_[selected_context]);
    drift_state.drift_score = clamp01(
        drift_state.drift_score +
        config_.drift_ewma_alpha *
            (minimum(distance, 1.0F) - drift_state.drift_score));
    const float scaled_drift_score =
        drift_state.drift_score * 12.0F;
    if (scaled_drift_score >= config_.drift_quarantine_score) {
      if (drift_state.consecutive_drift_samples < UINT32_MAX) {
        ++drift_state.consecutive_drift_samples;
      }
    } else if (scaled_drift_score < config_.drift_warning_score) {
      drift_state.consecutive_drift_samples = 0U;
    }
    if (!drift_state.quarantined &&
        drift_state.consecutive_drift_samples >=
            config_.drift_quarantine_samples) {
      drift_state.quarantined = true;
      entered_quarantine = true;
      clearShadowContext(selected_context);
    }
  } else if (!danger_lock && !drift_state.quarantined) {
    drift_state.drift_score *= 0.995F;
    if (drift_state.drift_score * 12.0F <
        config_.drift_warning_score) {
      drift_state.consecutive_drift_samples = 0U;
    }
  }

  const float reported_drift_score =
      drift_state.drift_score * 12.0F;
  const bool drift_warning =
      reported_drift_score >= config_.drift_warning_score;
  const bool calibration_suspended =
      drift_warning || drift_state.quarantined;
  const bool update_allowed =
      base_update_allowed && !calibration_suspended;
  if (update_allowed) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      updateStatistic(profiles_[selected_context][feature],
                      raw_values[feature]);
    }
    if (updates_since_checkpoint_ < UINT32_MAX) {
      ++updates_since_checkpoint_;
    }
    profile_dirty_ = true;
  }

  const bool shadow_update_allowed =
      drift_state.quarantined && drift_monitor_allowed &&
      shadowUpdateGate(context, csi, safety, respiration, session,
                       integrated);
  bool rebaseline_accepted = false;
  uint32_t shadow_samples =
      profileMinimumSamples(shadow_profiles_[selected_context]);
  uint32_t shadow_stable_samples =
      drift_state.shadow_stable_samples;
  if (shadow_update_allowed) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      updateStatistic(shadow_profiles_[selected_context][feature],
                      raw_values[feature]);
    }
    shadow_samples =
        profileMinimumSamples(shadow_profiles_[selected_context]);
    if (profileMature(shadow_profiles_[selected_context],
                      config_.shadow_rebaseline_samples)) {
      const float shadow_distance =
          profileDistance(raw_values,
                          shadow_profiles_[selected_context]);
      if (shadow_distance <= config_.shadow_stability_score) {
        if (drift_state.shadow_stable_samples < UINT32_MAX) {
          ++drift_state.shadow_stable_samples;
        }
      } else {
        drift_state.shadow_stable_samples = 0U;
      }
    }
    shadow_stable_samples = drift_state.shadow_stable_samples;
    if (shadow_samples >= config_.shadow_rebaseline_samples &&
        drift_state.shadow_stable_samples >=
            config_.shadow_stability_samples) {
      for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
        profiles_[selected_context][feature] =
            shadow_profiles_[selected_context][feature];
      }
      drift_state = {};
      clearShadowContext(selected_context);
      updates_since_checkpoint_ = std::max(
          updates_since_checkpoint_,
          config_.minimum_checkpoint_updates);
      profile_dirty_ = true;
      rebaseline_accepted = true;
    }
  }

  output = {};
  output.probe_sequence = integrated.probe_sequence;
  output.observed_at_us = integrated.observed_at_us;
  output.context = context;
  output.raw_level = integrated.safety_level;
  output.floor_fall = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::FloorFall)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::FloorFall)],
      danger_lock || calibration_suspended);
  output.bathtub_immobility = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::BathtubImmobility)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::BathtubImmobility)],
      danger_lock || calibration_suspended);
  output.dangerous_posture = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::DangerousPosture)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::DangerousPosture)],
      danger_lock || calibration_suspended);
  output.respiratory_motion_loss = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::RespiratoryMotionLoss)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::RespiratoryMotionLoss)],
      danger_lock || calibration_suspended);
  output.occupied_unknown = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::OccupiedUnknown)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::OccupiedUnknown)],
      danger_lock || calibration_suspended);
  output.prolonged_bathing_evidence =
      integrated.prolonged_bathing_evidence;
  output.raw_overall_risk = integrated.overall_risk;
  output.context_confidence = context_confidence;
  output.profile_maturity = minimum(
      output.floor_fall.maturity,
      minimum(output.bathtub_immobility.maturity,
              minimum(output.dangerous_posture.maturity,
                      minimum(output.respiratory_motion_loss.maturity,
                              output.occupied_unknown.maturity))));
  output.calibration_applied =
      output.profile_maturity >= 1.0F &&
      context != BathroomNullContext::Unknown && !danger_lock &&
      !calibration_suspended;

  const float calibrated_instant = maximum(
      output.floor_fall.calibrated_evidence * 0.95F,
      maximum(output.bathtub_immobility.calibrated_evidence * 0.95F,
              maximum(output.dangerous_posture.calibrated_evidence * 0.78F,
                      maximum(
                          output.respiratory_motion_loss.calibrated_evidence *
                              0.82F,
                          maximum(
                              output.occupied_unknown.calibrated_evidence *
                                  0.66F,
                              output.prolonged_bathing_evidence)))));
  const float retained_raw_risk =
      integrated.overall_risk *
      (1.0F - 0.25F * output.profile_maturity);
  output.calibrated_overall_risk =
      (danger_lock || calibration_suspended)
          ? integrated.overall_risk
          : clamp01(maximum(calibrated_instant, retained_raw_risk));
  output.explained_context_risk =
      clamp01(integrated.overall_risk - output.calibrated_overall_risk);
  output.recommended_level =
      recommendLevel(integrated.safety_level,
                     output.calibrated_overall_risk,
                     output.calibration_applied,
                     danger_lock || calibration_suspended);
  output.update_quality = update_quality;
  output.profile_drift_score =
      rebaseline_accepted ? 0.0F : reported_drift_score;
  output.shadow_profile_samples = shadow_samples;
  output.shadow_profile_maturity =
      config_.shadow_rebaseline_samples == 0U
          ? 1.0F
          : clamp01(static_cast<float>(shadow_samples) /
                    static_cast<float>(
                        config_.shadow_rebaseline_samples));
  output.drift_consecutive_samples =
      drift_state.consecutive_drift_samples;
  output.shadow_stable_samples = shadow_stable_samples;
  output.profile_generation = profile_generation_;
  output.updates_since_checkpoint = updates_since_checkpoint_;
  output.checkpoint_age_ms =
      elapsedMilliseconds(integrated.observed_at_us,
                          last_checkpoint_at_us_);
  output.null_update_allowed = update_allowed;
  output.null_profile_updated = update_allowed;
  output.danger_lock = danger_lock;
  output.physically_observable = integrated.physically_observable;
  output.drift_monitor_allowed = drift_monitor_allowed;
  output.drift_warning =
      !rebaseline_accepted && drift_warning;
  output.context_quarantined =
      !rebaseline_accepted && drift_state.quarantined;
  output.shadow_update_allowed = shadow_update_allowed;
  output.rebaseline_accepted = rebaseline_accepted;
  output.storage_available = storage_available_;
  output.persisted_profile_loaded = persisted_profile_loaded_;
  output.recovered_from_single_slot = recovered_from_single_slot_;
  output.profile_dirty = profile_dirty_;
  output.checkpoint_ok = checkpoint_ok_;
  output.evidence_ready = true;

  last_observed_at_us_ = integrated.observed_at_us;
  emitIfDue(output);
  return BathroomContextNullCalibrationUpdateStatus::Accepted;
}

void BathroomContextNullCalibration::emitIfDue(
    const BathroomContextNullCalibrationEvidence& evidence) {
  const uint64_t interval_us =
      static_cast<uint64_t>(config_.emit_interval_ms) * kUsPerMs;
  if (last_emit_at_us_ != 0U &&
      evidence.observed_at_us < last_emit_at_us_ + interval_us) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;

  Serial.printf(
      "{\"type\":\"bathroom_context_null_calibration\",\"probe\":%lu,"
      "\"context\":\"%s\",\"context_confidence\":%.3f,"
      "\"raw_level\":\"%s\",\"recommended_level\":\"%s\","
      "\"floor\":{\"raw\":%.3f,\"mean\":%.3f,\"scale\":%.3f,"
      "\"excess\":%.3f,\"calibrated\":%.3f,\"samples\":%lu},"
      "\"bathtub\":{\"raw\":%.3f,\"mean\":%.3f,\"scale\":%.3f,"
      "\"excess\":%.3f,\"calibrated\":%.3f,\"samples\":%lu},"
      "\"posture\":{\"raw\":%.3f,\"mean\":%.3f,\"scale\":%.3f,"
      "\"excess\":%.3f,\"calibrated\":%.3f,\"samples\":%lu},"
      "\"respiration_loss\":{\"raw\":%.3f,\"mean\":%.3f,"
      "\"scale\":%.3f,\"excess\":%.3f,\"calibrated\":%.3f,"
      "\"samples\":%lu},"
      "\"occupied_unknown\":{\"raw\":%.3f,\"mean\":%.3f,"
      "\"scale\":%.3f,\"excess\":%.3f,\"calibrated\":%.3f,"
      "\"samples\":%lu},"
      "\"raw_risk\":%.3f,\"calibrated_risk\":%.3f,"
      "\"explained_context_risk\":%.3f,\"maturity\":%.3f,"
      "\"profile_generation\":%lu,\"updates_since_checkpoint\":%lu,"
      "\"checkpoint_age_ms\":%lu,\"storage_available\":%s,"
      "\"persisted_profile_loaded\":%s,"
      "\"recovered_from_single_slot\":%s,\"profile_dirty\":%s,"
      "\"checkpoint_ok\":%s,"
      "\"drift_score\":%.3f,\"drift_consecutive\":%lu,"
      "\"drift_monitor_allowed\":%s,\"drift_warning\":%s,"
      "\"context_quarantined\":%s,\"shadow_update_allowed\":%s,"
      "\"shadow_samples\":%lu,\"shadow_maturity\":%.3f,"
      "\"shadow_stable_samples\":%lu,\"rebaseline_accepted\":%s,"
      "\"update_allowed\":%s,\"updated\":%s,\"applied\":%s,"
      "\"danger_lock\":%s,\"quality\":%.3f,\"ready\":true}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      contextName(evidence.context), evidence.context_confidence,
      levelName(evidence.raw_level), levelName(evidence.recommended_level),
      evidence.floor_fall.raw_evidence,
      evidence.floor_fall.baseline_mean,
      evidence.floor_fall.baseline_scale,
      evidence.floor_fall.standardized_excess,
      evidence.floor_fall.calibrated_evidence,
      static_cast<unsigned long>(evidence.floor_fall.samples),
      evidence.bathtub_immobility.raw_evidence,
      evidence.bathtub_immobility.baseline_mean,
      evidence.bathtub_immobility.baseline_scale,
      evidence.bathtub_immobility.standardized_excess,
      evidence.bathtub_immobility.calibrated_evidence,
      static_cast<unsigned long>(evidence.bathtub_immobility.samples),
      evidence.dangerous_posture.raw_evidence,
      evidence.dangerous_posture.baseline_mean,
      evidence.dangerous_posture.baseline_scale,
      evidence.dangerous_posture.standardized_excess,
      evidence.dangerous_posture.calibrated_evidence,
      static_cast<unsigned long>(evidence.dangerous_posture.samples),
      evidence.respiratory_motion_loss.raw_evidence,
      evidence.respiratory_motion_loss.baseline_mean,
      evidence.respiratory_motion_loss.baseline_scale,
      evidence.respiratory_motion_loss.standardized_excess,
      evidence.respiratory_motion_loss.calibrated_evidence,
      static_cast<unsigned long>(
          evidence.respiratory_motion_loss.samples),
      evidence.occupied_unknown.raw_evidence,
      evidence.occupied_unknown.baseline_mean,
      evidence.occupied_unknown.baseline_scale,
      evidence.occupied_unknown.standardized_excess,
      evidence.occupied_unknown.calibrated_evidence,
      static_cast<unsigned long>(evidence.occupied_unknown.samples),
      evidence.raw_overall_risk, evidence.calibrated_overall_risk,
      evidence.explained_context_risk, evidence.profile_maturity,
      static_cast<unsigned long>(evidence.profile_generation),
      static_cast<unsigned long>(evidence.updates_since_checkpoint),
      static_cast<unsigned long>(evidence.checkpoint_age_ms),
      evidence.storage_available ? "true" : "false",
      evidence.persisted_profile_loaded ? "true" : "false",
      evidence.recovered_from_single_slot ? "true" : "false",
      evidence.profile_dirty ? "true" : "false",
      evidence.checkpoint_ok ? "true" : "false",
      evidence.profile_drift_score,
      static_cast<unsigned long>(
          evidence.drift_consecutive_samples),
      evidence.drift_monitor_allowed ? "true" : "false",
      evidence.drift_warning ? "true" : "false",
      evidence.context_quarantined ? "true" : "false",
      evidence.shadow_update_allowed ? "true" : "false",
      static_cast<unsigned long>(evidence.shadow_profile_samples),
      evidence.shadow_profile_maturity,
      static_cast<unsigned long>(evidence.shadow_stable_samples),
      evidence.rebaseline_accepted ? "true" : "false",
      evidence.null_update_allowed ? "true" : "false",
      evidence.null_profile_updated ? "true" : "false",
      evidence.calibration_applied ? "true" : "false",
      evidence.danger_lock ? "true" : "false",
      evidence.update_quality);
}

void BathroomContextNullCalibration::reset() {
  for (size_t context = 0U; context < kContextCount; ++context) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      profiles_[context][feature] = {};
      before_current_probe_[context][feature] = {};
      shadow_profiles_[context][feature] = {};
      before_current_probe_shadow_[context][feature] = {};
    }
    drift_states_[context] = {};
    before_current_probe_drift_[context] = {};
  }
  before_current_probe_updates_ = 0U;
  before_current_probe_dirty_ = false;
  current_probe_sequence_ = 0U;
  last_observed_at_us_ = 0U;
  last_emit_at_us_ = 0U;
  last_checkpoint_at_us_ = 0U;
  last_checkpoint_attempt_at_us_ = 0U;
  profile_generation_ = 0U;
  updates_since_checkpoint_ = 0U;
  active_slot_ = 0U;
  has_current_probe_ = false;
  storage_initialized_ = false;
  storage_available_ = false;
  persisted_profile_loaded_ = false;
  recovered_from_single_slot_ = false;
  profile_dirty_ = false;
  checkpoint_ok_ = false;
}

}  // namespace atom::radar
