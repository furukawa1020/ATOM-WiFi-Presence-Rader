#include "BathroomRoomCalibration.hpp"

#include <Arduino.h>
#include <Preferences.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace atom::radar {

namespace {

constexpr const char *kPreferencesNamespace = "bath-room-cal";
constexpr const char *kProfileSlot0Key = "profile0";
constexpr const char *kProfileSlot1Key = "profile1";
constexpr const char *kActiveSlotKey = "active";

}  // namespace

const char *bathroomCalibrationCategoryName(
    BathroomCalibrationCategory category) {
  switch (category) {
    case BathroomCalibrationCategory::EmptyRoom:
      return "empty_room";
    case BathroomCalibrationCategory::Bathtub:
      return "bathtub";
    case BathroomCalibrationCategory::Boundary:
      return "boundary";
    case BathroomCalibrationCategory::WashFloor:
      return "wash_floor";
    case BathroomCalibrationCategory::Standing:
      return "standing";
    case BathroomCalibrationCategory::Seated:
      return "seated";
    case BathroomCalibrationCategory::StableRespiration:
      return "stable_respiration";
    case BathroomCalibrationCategory::PeriodicNuisance:
      return "periodic_nuisance";
    case BathroomCalibrationCategory::None:
    default:
      return "none";
  }
}

BathroomRoomCalibration::BathroomRoomCalibration(
    BathroomRoomCalibrationConfig config)
    : config_(config) {
  initializeProfile();
}

float BathroomRoomCalibration::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomRoomCalibration::maximum(float first, float second) {
  return first > second ? first : second;
}

float BathroomRoomCalibration::absolute(float value) {
  return value < 0.0F ? -value : value;
}

uint32_t BathroomRoomCalibration::crc32(const uint8_t *data,
                                        std::size_t length) {
  uint32_t crc = 0xFFFFFFFFU;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = -(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return ~crc;
}

void BathroomRoomCalibration::initializeProfile() {
  profile_ = {};
  profile_.magic = kProfileMagic;
  profile_.version = kProfileVersion;
  profile_.feature_count = static_cast<uint16_t>(kFeatureCount);
  profile_.category_count = static_cast<uint16_t>(kCategoryCount);
  for (std::size_t category = 0; category < kCategoryCount; ++category) {
    for (std::size_t feature = 0; feature < kFeatureCount; ++feature) {
      profile_.prototypes[category].variance[feature] = 0.04F;
    }
  }
}

void BathroomRoomCalibration::resetRuntime() {
  has_probe_ = false;
  last_probe_sequence_ = 0;
  updates_since_checkpoint_ = 0;
  last_updated_category_ = BathroomCalibrationCategory::None;
  last_emit_at_us_ = -1;
}

bool BathroomRoomCalibration::validateProfile(
    const PersistedProfile &profile) const {
  if (profile.magic != kProfileMagic || profile.version != kProfileVersion ||
      profile.feature_count != kFeatureCount ||
      profile.category_count != kCategoryCount) {
    return false;
  }
  const uint32_t expected = crc32(
      reinterpret_cast<const uint8_t *>(&profile),
      offsetof(PersistedProfile, crc32));
  return expected == profile.crc32;
}

bool BathroomRoomCalibration::loadProfile() {
  Preferences preferences;
  if (!preferences.begin(kPreferencesNamespace, true)) {
    storage_available_ = false;
    return false;
  }
  storage_available_ = true;
  PersistedProfile slot0{};
  PersistedProfile slot1{};
  const bool slot0_read =
      preferences.getBytesLength(kProfileSlot0Key) == sizeof(slot0) &&
      preferences.getBytes(kProfileSlot0Key, &slot0, sizeof(slot0)) ==
          sizeof(slot0);
  const bool slot1_read =
      preferences.getBytesLength(kProfileSlot1Key) == sizeof(slot1) &&
      preferences.getBytes(kProfileSlot1Key, &slot1, sizeof(slot1)) ==
          sizeof(slot1);
  preferences.end();

  const bool slot0_valid = slot0_read && validateProfile(slot0);
  const bool slot1_valid = slot1_read && validateProfile(slot1);
  if (!slot0_valid && !slot1_valid) {
    return false;
  }
  if (slot0_valid &&
      (!slot1_valid || static_cast<int32_t>(slot0.generation -
                                            slot1.generation) > 0)) {
    profile_ = slot0;
  } else {
    profile_ = slot1;
  }
  persisted_profile_loaded_ = true;
  dirty_ = false;
  return true;
}

void BathroomRoomCalibration::ensureLoaded() {
  if (load_attempted_) {
    return;
  }
  load_attempted_ = true;
  if (!loadProfile()) {
    initializeProfile();
  }
}

bool BathroomRoomCalibration::saveProfile() {
  if (!dirty_) {
    last_checkpoint_ok_ = true;
    return true;
  }
  PersistedProfile candidate = profile_;
  candidate.generation = profile_.generation + 1U;
  candidate.crc32 = 0;
  candidate.crc32 = crc32(
      reinterpret_cast<const uint8_t *>(&candidate),
      offsetof(PersistedProfile, crc32));
  const uint8_t target_slot = static_cast<uint8_t>(candidate.generation & 1U);
  const char *target_key =
      target_slot == 0 ? kProfileSlot0Key : kProfileSlot1Key;

  Preferences preferences;
  if (!preferences.begin(kPreferencesNamespace, false)) {
    storage_available_ = false;
    last_checkpoint_ok_ = false;
    return false;
  }
  storage_available_ = true;
  const bool profile_written =
      preferences.putBytes(target_key, &candidate, sizeof(candidate)) ==
      sizeof(candidate);
  const bool active_written =
      profile_written && preferences.putUChar(kActiveSlotKey, target_slot) == 1;
  preferences.end();
  if (!active_written) {
    last_checkpoint_ok_ = false;
    return false;
  }
  profile_ = candidate;
  dirty_ = false;
  updates_since_checkpoint_ = 0;
  last_checkpoint_ok_ = true;
  return true;
}

bool BathroomRoomCalibration::checkpoint() {
  ensureLoaded();
  return saveProfile();
}

void BathroomRoomCalibration::buildFeatures(
    const FusedCsiObservation &observation,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSpatialEvidence &spatial_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    float features[kFeatureCount]) const {
  features[0] = clamp01(observation.amplitude_motion);
  features[1] = clamp01(observation.differential_phase_motion);
  features[2] = clamp01(observation.complex_ratio_motion);
  features[3] = clamp01(observation.phase_coherence);
  features[4] = clamp01(observation.subcarrier_reliability);
  features[5] = clamp01(observation.delay_domain_motion);
  features[6] = clamp01(observation.delay_spread);
  features[7] = clamp01(observation.dynamic_tap_concentration);
  features[8] = clamp01(observation.background_explained_ratio);
  features[9] = clamp01(observation.innovation_motion);
  features[10] = clamp01(observation.doppler_energy);
  features[11] = clamp01(observation.doppler_bandwidth);
  features[12] = clamp01(observation.respiration_power);
  features[13] = clamp01(observation.respiration_rate_normalized);
  features[14] = clamp01(observation.respiration_spectral_snr);
  features[15] = clamp01(observation.baseline_shift);
  features[16] = clamp01(observation.broadband_nuisance);
  features[17] = clamp01(bathroom_evidence.nuisance_reduced_human_motion);
  features[18] = clamp01(spatial_evidence.bathtub_position_evidence);
  features[19] = clamp01(spatial_evidence.boundary_position_evidence);
  features[20] = clamp01(spatial_evidence.wash_floor_position_evidence);
  features[21] = clamp01(spatial_evidence.standing_posture_evidence);
  features[22] = clamp01(spatial_evidence.seated_posture_evidence);
  features[23] =
      clamp01(respiration_evidence.respiration_micro_motion_evidence);
}

bool BathroomRoomCalibration::updatePrototype(
    std::size_t category, const float features[kFeatureCount], float quality) {
  if (category >= kCategoryCount || quality < config_.minimum_update_quality) {
    return false;
  }
  Prototype &prototype = profile_.prototypes[category];
  if (prototype.samples == 0) {
    for (std::size_t feature = 0; feature < kFeatureCount; ++feature) {
      prototype.mean[feature] = features[feature];
      prototype.variance[feature] = 0.04F;
    }
    prototype.samples = 1;
    prototype.update_quality = quality;
    dirty_ = true;
    return true;
  }

  const float bootstrap_alpha = clamp01(
      1.0F / static_cast<float>(prototype.samples + 1U));
  const float alpha =
      prototype.samples < config_.minimum_mature_samples
          ? maximum(0.02F, bootstrap_alpha)
          : config_.mature_adaptation_alpha;
  float drift_sum = 0.0F;
  for (std::size_t feature = 0; feature < kFeatureCount; ++feature) {
    const float residual = features[feature] - prototype.mean[feature];
    const float sigma = std::sqrt(maximum(
        prototype.variance[feature], config_.variance_floor));
    const float limit = config_.residual_clip_sigma * sigma;
    const float clipped = std::clamp(residual, -limit, limit);
    prototype.mean[feature] =
        clamp01(prototype.mean[feature] + alpha * clipped);
    prototype.variance[feature] = std::clamp(
        (1.0F - alpha) * prototype.variance[feature] +
            alpha * clipped * clipped,
        config_.variance_floor, 0.25F);
    drift_sum += absolute(residual);
  }
  prototype.drift =
      0.98F * prototype.drift +
      0.02F * drift_sum / static_cast<float>(kFeatureCount);
  prototype.update_quality =
      0.95F * prototype.update_quality + 0.05F * quality;
  if (prototype.samples < UINT32_MAX) {
    ++prototype.samples;
  }
  dirty_ = true;
  return true;
}

float BathroomRoomCalibration::maturity(std::size_t category) const {
  if (category >= kCategoryCount) {
    return 0.0F;
  }
  return clamp01(
      static_cast<float>(profile_.prototypes[category].samples) /
      static_cast<float>(config_.minimum_mature_samples));
}

float BathroomRoomCalibration::similarity(
    std::size_t category, const float features[kFeatureCount]) const {
  if (category >= kCategoryCount ||
      profile_.prototypes[category].samples == 0) {
    return 0.5F;
  }
  const Prototype &prototype = profile_.prototypes[category];
  float distance = 0.0F;
  for (std::size_t feature = 0; feature < kFeatureCount; ++feature) {
    const float residual = features[feature] - prototype.mean[feature];
    distance += residual * residual /
                maximum(prototype.variance[feature],
                        config_.variance_floor);
  }
  distance /= static_cast<float>(kFeatureCount);
  return clamp01(1.0F / (1.0F + distance));
}

void BathroomRoomCalibration::buildEvidence(
    uint32_t probe_sequence, int64_t observed_at_us,
    const float features[kFeatureCount],
    const BathroomSpatialEvidence &spatial_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    bool normal_update_frozen,
    BathroomRoomCalibrationEvidence &evidence) const {
  evidence = {};
  evidence.probe_sequence = probe_sequence;
  evidence.observed_at_us = observed_at_us;
  evidence.profile_generation = profile_.generation;
  evidence.last_updated_category = last_updated_category_;

  const float empty_similarity = similarity(0, features);
  const float zone_similarity[3]{similarity(1, features),
                                 similarity(2, features),
                                 similarity(3, features)};
  const float zone_maturity[3]{maturity(1), maturity(2), maturity(3)};
  const float raw_zone[3]{spatial_evidence.bathtub_position_evidence,
                          spatial_evidence.boundary_position_evidence,
                          spatial_evidence.wash_floor_position_evidence};
  float calibrated_zone[3]{};
  float zone_sum = 0.0F;
  for (std::size_t index = 0; index < 3; ++index) {
    const float correction =
        (1.0F - zone_maturity[index]) +
        zone_maturity[index] * (0.55F + 0.90F * zone_similarity[index]);
    calibrated_zone[index] = raw_zone[index] * correction;
    zone_sum += calibrated_zone[index];
  }
  if (zone_sum > 0.001F) {
    for (float &value : calibrated_zone) {
      value /= zone_sum;
    }
  } else {
    for (std::size_t index = 0; index < 3; ++index) {
      calibrated_zone[index] = raw_zone[index];
    }
  }

  const float standing_similarity = similarity(4, features);
  const float seated_similarity = similarity(5, features);
  const float standing_maturity = maturity(4);
  const float seated_maturity = maturity(5);
  const float normal_similarity =
      maximum(standing_similarity * standing_maturity,
              seated_similarity * seated_maturity);
  const float posture_anomaly = clamp01(1.0F - normal_similarity);
  evidence.calibrated_empty_room_evidence =
      clamp01(empty_similarity * maturity(0));
  evidence.calibrated_bathtub_evidence = calibrated_zone[0];
  evidence.calibrated_boundary_evidence = calibrated_zone[1];
  evidence.calibrated_wash_floor_evidence = calibrated_zone[2];
  evidence.calibrated_standing_evidence = clamp01(
      spatial_evidence.standing_posture_evidence *
      ((1.0F - standing_maturity) +
       standing_maturity * (0.65F + 0.70F * standing_similarity)));
  evidence.calibrated_seated_evidence = clamp01(
      spatial_evidence.seated_posture_evidence *
      ((1.0F - seated_maturity) +
       seated_maturity * (0.65F + 0.70F * seated_similarity)));
  evidence.calibrated_low_posture_evidence = clamp01(
      spatial_evidence.low_posture_evidence *
      (0.75F + 0.25F * posture_anomaly));
  evidence.calibrated_lying_evidence = clamp01(
      spatial_evidence.lying_posture_evidence *
      (0.70F + 0.30F * posture_anomaly));
  evidence.calibrated_dangerous_posture_evidence = clamp01(
      spatial_evidence.dangerous_posture_evidence *
      (0.70F + 0.30F * posture_anomaly));

  const float respiration_maturity = maturity(6);
  const float respiration_similarity = similarity(6, features);
  evidence.calibrated_respiration_evidence = clamp01(
      respiration_evidence.respiration_micro_motion_evidence *
      ((1.0F - respiration_maturity) +
       respiration_maturity * (0.65F + 0.70F * respiration_similarity)));
  const float nuisance_maturity = maturity(7);
  const float nuisance_similarity = similarity(7, features);
  evidence.calibrated_periodic_nuisance_evidence = clamp01(
      respiration_evidence.periodic_nuisance_alternative *
      ((1.0F - nuisance_maturity) +
       nuisance_maturity * (0.65F + 0.70F * nuisance_similarity)));

  float maturity_sum = 0.0F;
  float drift_sum = 0.0F;
  float update_quality_sum = 0.0F;
  uint8_t mature_categories = 0;
  for (std::size_t category = 0; category < kCategoryCount; ++category) {
    const float category_maturity = maturity(category);
    maturity_sum += category_maturity;
    drift_sum += profile_.prototypes[category].drift * category_maturity;
    update_quality_sum +=
        profile_.prototypes[category].update_quality * category_maturity;
    if (category_maturity >= 1.0F) {
      ++mature_categories;
    }
  }
  evidence.mature_categories = mature_categories;
  evidence.profile_maturity =
      maturity_sum / static_cast<float>(kCategoryCount);
  evidence.profile_drift =
      maturity_sum > 0.001F ? clamp01(drift_sum / maturity_sum) : 0.0F;
  evidence.update_quality =
      maturity_sum > 0.001F
          ? clamp01(update_quality_sum / maturity_sum)
          : 0.0F;
  evidence.analysis_quality = clamp01(
      0.35F * spatial_evidence.analysis_quality +
      0.35F * respiration_evidence.analysis_quality +
      0.20F * evidence.update_quality +
      0.10F * (1.0F - evidence.profile_drift));
  evidence.normal_update_frozen = normal_update_frozen;
  evidence.persisted_profile_loaded = persisted_profile_loaded_;
  evidence.storage_available = storage_available_;
  evidence.checkpoint_ok = last_checkpoint_ok_;
  evidence.profile_dirty = dirty_;
  evidence.evidence_ready = evidence.profile_maturity >= 0.25F;
}

BathroomRoomCalibrationUpdateStatus BathroomRoomCalibration::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSafetyEvidence &safety_evidence,
    const BathroomSpatialEvidence &spatial_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    BathroomRoomCalibrationEvidence &calibrated_evidence) {
  ensureLoaded();
  if (observed_at_us < 0 ||
      observation.anchor_probe_sequence != bathroom_evidence.probe_sequence ||
      observation.anchor_probe_sequence != safety_evidence.probe_sequence ||
      observation.anchor_probe_sequence != spatial_evidence.probe_sequence ||
      observation.anchor_probe_sequence != respiration_evidence.probe_sequence ||
      !std::isfinite(observation.quality)) {
    return BathroomRoomCalibrationUpdateStatus::InvalidObservation;
  }

  bool same_probe = false;
  if (has_probe_) {
    const int32_t sequence_delta = static_cast<int32_t>(
        observation.anchor_probe_sequence - last_probe_sequence_);
    if (sequence_delta < 0) {
      return BathroomRoomCalibrationUpdateStatus::NonMonotonicProbe;
    }
    same_probe = sequence_delta == 0;
  }

  float features[kFeatureCount]{};
  buildFeatures(observation, bathroom_evidence, spatial_evidence,
                respiration_evidence, features);
  const bool normal_update_frozen =
      safety_evidence.impact_evidence > 0.25F ||
      safety_evidence.fall_sequence_evidence > 0.20F ||
      safety_evidence.dangerous_immobility_evidence > 0.20F ||
      spatial_evidence.dangerous_posture_evidence > 0.18F ||
      respiration_evidence.respiration_loss_evidence > 0.30F;

  last_updated_category_ = BathroomCalibrationCategory::None;
  if (!same_probe) {
    has_probe_ = true;
    last_probe_sequence_ = observation.anchor_probe_sequence;
    const float common_quality = clamp01(
        0.35F * observation.quality +
        0.20F * bathroom_evidence.analysis_quality +
        0.20F * spatial_evidence.analysis_quality +
        0.25F * respiration_evidence.analysis_quality);
    bool updated = false;

    const bool empty_gate =
        !normal_update_frozen &&
        bathroom_evidence.nuisance_reduced_human_motion < 0.04F &&
        respiration_evidence.respiration_micro_motion_evidence < 0.10F &&
        observation.innovation_motion < 0.05F &&
        observation.baseline_shift < 0.05F &&
        bathroom_evidence.nuisance_confidence < 0.20F;
    if (empty_gate && updatePrototype(0, features, common_quality)) {
      last_updated_category_ = BathroomCalibrationCategory::EmptyRoom;
      updated = true;
    }

    const bool spatial_gate =
        !normal_update_frozen && spatial_evidence.active_links >= 2 &&
        spatial_evidence.role_coverage >= 2.0F / 3.0F &&
        spatial_evidence.spatial_separation >= 0.18F &&
        spatial_evidence.baseline_maturity >= 0.60F &&
        spatial_evidence.position_uncertain_evidence < 0.42F;
    if (spatial_gate) {
      std::size_t zone_category = kCategoryCount;
      BathroomCalibrationCategory zone_name =
          BathroomCalibrationCategory::None;
      float zone_confidence = 0.0F;
      if (spatial_evidence.bathtub_position_evidence >= 0.62F) {
        zone_category = 1;
        zone_name = BathroomCalibrationCategory::Bathtub;
        zone_confidence = spatial_evidence.bathtub_position_evidence;
      } else if (spatial_evidence.boundary_position_evidence >= 0.62F) {
        zone_category = 2;
        zone_name = BathroomCalibrationCategory::Boundary;
        zone_confidence = spatial_evidence.boundary_position_evidence;
      } else if (spatial_evidence.wash_floor_position_evidence >= 0.62F) {
        zone_category = 3;
        zone_name = BathroomCalibrationCategory::WashFloor;
        zone_confidence = spatial_evidence.wash_floor_position_evidence;
      }
      if (zone_category < kCategoryCount &&
          updatePrototype(zone_category, features,
                          common_quality * zone_confidence)) {
        last_updated_category_ = zone_name;
        updated = true;
      }
    }

    const bool normal_posture_gate =
        !normal_update_frozen &&
        spatial_evidence.dangerous_posture_evidence < 0.12F &&
        spatial_evidence.lying_posture_evidence < 0.16F &&
        safety_evidence.fall_sequence_evidence < 0.12F;
    if (normal_posture_gate &&
        spatial_evidence.standing_posture_evidence >= 0.65F &&
        updatePrototype(4, features,
                        common_quality *
                            spatial_evidence.standing_posture_evidence)) {
      last_updated_category_ = BathroomCalibrationCategory::Standing;
      updated = true;
    } else if (normal_posture_gate &&
               spatial_evidence.seated_posture_evidence >= 0.65F &&
               updatePrototype(5, features,
                               common_quality *
                                   spatial_evidence.seated_posture_evidence)) {
      last_updated_category_ = BathroomCalibrationCategory::Seated;
      updated = true;
    }

    const bool respiration_gate =
        !normal_update_frozen &&
        respiration_evidence.stable_respiration_evidence >= 0.62F &&
        respiration_evidence.rate_stability >= 0.70F &&
        respiration_evidence.measurement_loss_alternative < 0.25F &&
        respiration_evidence.periodic_nuisance_alternative < 0.35F;
    if (respiration_gate &&
        updatePrototype(
            6, features,
            common_quality *
                respiration_evidence.stable_respiration_evidence)) {
      last_updated_category_ =
          BathroomCalibrationCategory::StableRespiration;
      updated = true;
    }

    const bool nuisance_gate =
        !normal_update_frozen &&
        respiration_evidence.periodic_nuisance_alternative >= 0.62F &&
        bathroom_evidence.nuisance_reduced_human_motion < 0.10F &&
        safety_evidence.fall_sequence_evidence < 0.10F;
    if (nuisance_gate &&
        updatePrototype(
            7, features,
            common_quality *
                respiration_evidence.periodic_nuisance_alternative)) {
      last_updated_category_ = BathroomCalibrationCategory::PeriodicNuisance;
      updated = true;
    }

    if (updated && updates_since_checkpoint_ < UINT32_MAX) {
      ++updates_since_checkpoint_;
    }
    if (dirty_ && updates_since_checkpoint_ >=
                      config_.checkpoint_interval_updates) {
      saveProfile();
    }
  }

  buildEvidence(observation.anchor_probe_sequence, observed_at_us, features,
                spatial_evidence, respiration_evidence,
                normal_update_frozen, calibrated_evidence);
  emitJsonIfDue(calibrated_evidence);
  return same_probe ? BathroomRoomCalibrationUpdateStatus::ReplacedSameProbe
                    : BathroomRoomCalibrationUpdateStatus::Updated;
}

void BathroomRoomCalibration::emitJsonIfDue(
    const BathroomRoomCalibrationEvidence &evidence) {
  if (last_emit_at_us_ >= 0 &&
      evidence.observed_at_us - last_emit_at_us_ < 1000000) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_room_calibration\",\"probe\":%lu,"
      "\"generation\":%lu,\"mature_categories\":%u,"
      "\"last_update\":\"%s\",\"empty_room\":%.3f,"
      "\"bathtub\":%.3f,\"boundary\":%.3f,\"wash_floor\":%.3f,"
      "\"standing\":%.3f,\"seated\":%.3f,\"low_posture\":%.3f,"
      "\"lying\":%.3f,\"dangerous_posture\":%.3f,"
      "\"respiration\":%.3f,\"periodic_nuisance\":%.3f,"
      "\"profile_maturity\":%.3f,\"profile_drift\":%.3f,"
      "\"update_quality\":%.3f,\"quality\":%.3f,"
      "\"normal_update_frozen\":%s,\"profile_loaded\":%s,"
      "\"storage_available\":%s,\"checkpoint_ok\":%s,"
      "\"dirty\":%s,\"ready\":%s}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      static_cast<unsigned long>(evidence.profile_generation),
      evidence.mature_categories,
      bathroomCalibrationCategoryName(evidence.last_updated_category),
      evidence.calibrated_empty_room_evidence,
      evidence.calibrated_bathtub_evidence,
      evidence.calibrated_boundary_evidence,
      evidence.calibrated_wash_floor_evidence,
      evidence.calibrated_standing_evidence,
      evidence.calibrated_seated_evidence,
      evidence.calibrated_low_posture_evidence,
      evidence.calibrated_lying_evidence,
      evidence.calibrated_dangerous_posture_evidence,
      evidence.calibrated_respiration_evidence,
      evidence.calibrated_periodic_nuisance_evidence,
      evidence.profile_maturity, evidence.profile_drift,
      evidence.update_quality, evidence.analysis_quality,
      evidence.normal_update_frozen ? "true" : "false",
      evidence.persisted_profile_loaded ? "true" : "false",
      evidence.storage_available ? "true" : "false",
      evidence.checkpoint_ok ? "true" : "false",
      evidence.profile_dirty ? "true" : "false",
      evidence.evidence_ready ? "true" : "false");
}

}  // namespace atom::radar
