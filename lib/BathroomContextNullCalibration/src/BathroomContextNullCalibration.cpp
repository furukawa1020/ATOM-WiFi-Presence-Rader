#include "BathroomContextNullCalibration.hpp"

#include <algorithm>
#include <cmath>

#include <Arduino.h>

namespace atom::radar {
namespace {

constexpr uint64_t kUsPerMs = 1000ULL;

size_t contextIndex(BathroomNullContext context) {
  return static_cast<size_t>(context);
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

  if (has_current_probe_ &&
      integrated.probe_sequence == current_probe_sequence_) {
    copyProfiles(profiles_, before_current_probe_);
  } else {
    copyProfiles(before_current_probe_, profiles_);
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
  const bool update_allowed =
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
  if (update_allowed) {
    for (size_t feature = 0U; feature < kFeatureCount; ++feature) {
      updateStatistic(profiles_[selected_context][feature],
                      raw_values[feature]);
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
      danger_lock);
  output.bathtub_immobility = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::BathtubImmobility)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::BathtubImmobility)],
      danger_lock);
  output.dangerous_posture = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::DangerousPosture)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::DangerousPosture)],
      danger_lock);
  output.respiratory_motion_loss = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::RespiratoryMotionLoss)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::RespiratoryMotionLoss)],
      danger_lock);
  output.occupied_unknown = calibrateFeature(
      raw_values[static_cast<size_t>(FeatureIndex::OccupiedUnknown)],
      profiles_[selected_context]
               [static_cast<size_t>(FeatureIndex::OccupiedUnknown)],
      danger_lock);
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
      context != BathroomNullContext::Unknown && !danger_lock;

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
      danger_lock
          ? integrated.overall_risk
          : clamp01(maximum(calibrated_instant, retained_raw_risk));
  output.explained_context_risk =
      clamp01(integrated.overall_risk - output.calibrated_overall_risk);
  output.recommended_level =
      recommendLevel(integrated.safety_level,
                     output.calibrated_overall_risk,
                     output.calibration_applied, danger_lock);
  output.update_quality = update_quality;
  output.null_update_allowed = update_allowed;
  output.null_profile_updated = update_allowed;
  output.danger_lock = danger_lock;
  output.physically_observable = integrated.physically_observable;
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
    }
  }
  current_probe_sequence_ = 0U;
  last_observed_at_us_ = 0U;
  last_emit_at_us_ = 0U;
  has_current_probe_ = false;
}

}  // namespace atom::radar
