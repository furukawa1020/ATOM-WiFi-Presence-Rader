#include "BathroomIntegratedSafety.hpp"

#include <algorithm>

#include <Arduino.h>

namespace atom::radar {
namespace {

constexpr uint64_t kUsPerMs = 1000ULL;
constexpr float kFastTimeConstantSeconds = 2.0F;
constexpr float kMediumTimeConstantSeconds = 12.0F;
constexpr float kSlowTimeConstantSeconds = 60.0F;
constexpr float kObservableMemoryDecaySeconds = 120.0F;
constexpr float kUnobservableMemoryDecaySeconds = 600.0F;

float elapsedAlpha(float elapsed_seconds, float time_constant_seconds) {
  if (elapsed_seconds <= 0.0F) {
    return 0.0F;
  }
  return std::clamp(elapsed_seconds / time_constant_seconds, 0.0F, 1.0F);
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

BathroomIntegratedSafetyAnalyzer::BathroomIntegratedSafetyAnalyzer(
    const BathroomIntegratedSafetyConfig& config)
    : config_(config) {}

float BathroomIntegratedSafetyAnalyzer::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomIntegratedSafetyAnalyzer::maximum(float a, float b) {
  return a > b ? a : b;
}

uint8_t BathroomIntegratedSafetyAnalyzer::dangerRank(
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

const char* BathroomIntegratedSafetyAnalyzer::levelName(
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

float BathroomIntegratedSafetyAnalyzer::bathtubTimeContext(
    const BathroomBathingSessionEvidence& session) const {
  if (!session.session_active || session.bathtub_probability <= 0.0F ||
      session.bathtub_duration_ms <= config_.bathtub_time_context_start_ms) {
    return 0.0F;
  }
  if (config_.bathtub_time_context_full_ms <=
      config_.bathtub_time_context_start_ms) {
    return 1.0F;
  }
  const float elapsed = static_cast<float>(
      session.bathtub_duration_ms - config_.bathtub_time_context_start_ms);
  const float range = static_cast<float>(config_.bathtub_time_context_full_ms -
                                         config_.bathtub_time_context_start_ms);
  return clamp01(elapsed / range) * clamp01(session.bathtub_probability);
}

BathroomIntegratedSafetyLevel BathroomIntegratedSafetyAnalyzer::desiredLevel(
    float overall_risk,
    float floor_fall,
    float bathtub_immobility) const {
  if (overall_risk >= config_.emergency_threshold ||
      floor_fall >= config_.direct_emergency_threshold ||
      bathtub_immobility >= config_.direct_emergency_threshold) {
    return BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended;
  }
  if (overall_risk >= config_.urgent_threshold) {
    return BathroomIntegratedSafetyLevel::Urgent;
  }
  if (overall_risk >= config_.check_required_threshold) {
    return BathroomIntegratedSafetyLevel::CheckRequired;
  }
  if (overall_risk >= config_.attention_threshold) {
    return BathroomIntegratedSafetyLevel::Attention;
  }
  return BathroomIntegratedSafetyLevel::Normal;
}

uint32_t BathroomIntegratedSafetyAnalyzer::transitionConfirmationMs(
    BathroomIntegratedSafetyLevel current,
    BathroomIntegratedSafetyLevel desired) const {
  const uint8_t current_rank = dangerRank(current);
  const uint8_t desired_rank = dangerRank(desired);
  if (desired ==
          BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended &&
      desired_rank > current_rank) {
    return 0U;
  }
  if (desired_rank < current_rank) {
    return current ==
                   BathroomIntegratedSafetyLevel::EmergencyNotificationRecommended
               ? config_.emergency_downgrade_confirmation_ms
               : config_.downgrade_confirmation_ms;
  }
  switch (desired) {
    case BathroomIntegratedSafetyLevel::Attention:
      return config_.attention_confirmation_ms;
    case BathroomIntegratedSafetyLevel::CheckRequired:
      return config_.check_required_confirmation_ms;
    case BathroomIntegratedSafetyLevel::Urgent:
      return config_.urgent_confirmation_ms;
    default:
      return 0U;
  }
}

BathroomIntegratedSafetyUpdateStatus BathroomIntegratedSafetyAnalyzer::update(
    const FusedCsiObservation& fused,
    const BathroomCsiEvidence& csi,
    const BathroomSafetyEvidence& safety,
    const BathroomSpatialEvidence& spatial,
    const BathroomRespirationEvidence& respiration,
    const BathroomRoomCalibrationEvidence& calibration,
    const BathroomBathingSessionEvidence& session,
    BathroomIntegratedSafetyEvidence& output) {
  if (!csi.evidence_ready || !safety.evidence_ready ||
      !spatial.evidence_ready || !respiration.evidence_ready ||
      !calibration.evidence_ready || !session.evidence_ready) {
    return BathroomIntegratedSafetyUpdateStatus::NotReady;
  }

  const uint32_t probe_sequence = session.probe_sequence;
  const uint64_t observed_at_us = session.observed_at_us;
  if (has_current_probe_ && probe_sequence != current_probe_sequence_ &&
      observed_at_us < state_.observed_at_us) {
    return BathroomIntegratedSafetyUpdateStatus::StaleObservation;
  }

  TemporalState base = state_;
  if (has_current_probe_ && probe_sequence == current_probe_sequence_) {
    base = before_current_probe_;
  } else {
    before_current_probe_ = state_;
    current_probe_sequence_ = probe_sequence;
    has_current_probe_ = true;
  }

  const float floor_position = maximum(
      spatial.wash_floor_position_evidence,
      calibration.calibrated_wash_floor_evidence);
  const float bathtub_position = maximum(
      spatial.bathtub_position_evidence,
      calibration.calibrated_bathtub_evidence);
  const float dangerous_posture = maximum(
      spatial.dangerous_posture_evidence,
      calibration.calibrated_dangerous_posture_evidence);
  const float occupancy = maximum(
      session.occupancy_evidence,
      maximum(session.bathtub_probability, session.wash_floor_probability));

  const float floor_fall = clamp01(maximum(
      safety.fall_sequence_evidence * floor_position,
      safety.impact_evidence * safety.post_impact_immobility * floor_position));
  const float respiratory_loss = clamp01(
      respiration.respiration_loss_evidence *
      (1.0F - clamp01(respiration.measurement_loss_alternative)) * occupancy);
  const float bathtub_immobility = clamp01(maximum(
      session.bathtub_immobility_context,
      bathtub_position * maximum(safety.dangerous_immobility_evidence,
                                 respiratory_loss)));
  const float posture =
      clamp01(dangerous_posture * maximum(0.50F, occupancy));
  const float thermal_context = bathtubTimeContext(session);
  const float prolonged = clamp01(maximum(
      maximum(session.prolonged_bathtub_evidence,
              session.prolonged_session_evidence *
                  session.bathtub_probability),
      thermal_context));
  const float occupied_unknown = clamp01(
      session.occupied_unknown_probability *
      maximum(session.measurement_loss_alternative,
              respiration.measurement_loss_alternative));
  const float nuisance = clamp01(maximum(
      maximum(csi.nuisance_confidence,
              calibration.calibrated_periodic_nuisance_evidence),
      maximum(safety.door_or_object_alternative,
              safety.persistent_nuisance_alternative)));
  const float minimum_quality = std::min(
      {fused.quality, csi.analysis_quality, safety.analysis_quality,
       spatial.analysis_quality, respiration.analysis_quality,
       calibration.analysis_quality, session.analysis_quality});
  const float measurement_failure = clamp01(maximum(
      maximum(respiration.measurement_loss_alternative,
              session.measurement_loss_alternative),
      1.0F - minimum_quality));
  const bool physically_observable =
      fused.physically_observable && fused.active_links > 0U &&
      spatial.active_links > 0U && respiration.active_links > 0U &&
      measurement_failure < config_.physical_measurement_failure_threshold;

  float fast_input = maximum(
      floor_fall,
      maximum(bathtub_immobility,
              maximum(posture * 0.78F, respiratory_loss * 0.82F)));
  float medium_input = maximum(
      0.72F * bathtub_immobility + 0.28F * respiratory_loss,
      maximum(0.70F * posture + 0.30F * prolonged,
              0.78F * floor_fall));
  float slow_input = maximum(
      prolonged,
      maximum(0.66F * occupied_unknown,
              0.58F * bathtub_immobility + 0.42F * respiratory_loss));
  const float nuisance_attenuation = 1.0F - 0.15F * nuisance;
  fast_input = clamp01(fast_input * nuisance_attenuation);
  medium_input = clamp01(medium_input * nuisance_attenuation);
  slow_input = clamp01(slow_input * nuisance_attenuation);

  float elapsed_seconds = 1.0F;
  if (base.observed_at_us != 0U && observed_at_us > base.observed_at_us) {
    elapsed_seconds = static_cast<float>(observed_at_us - base.observed_at_us) /
                      1000000.0F;
  }
  TemporalState next = base;
  const float fast_alpha =
      elapsedAlpha(elapsed_seconds, kFastTimeConstantSeconds);
  const float medium_alpha =
      elapsedAlpha(elapsed_seconds, kMediumTimeConstantSeconds);
  const float slow_alpha =
      elapsedAlpha(elapsed_seconds, kSlowTimeConstantSeconds);
  next.fast_risk = clamp01(base.fast_risk +
                           fast_alpha * (fast_input - base.fast_risk));
  next.medium_risk = clamp01(base.medium_risk +
                             medium_alpha * (medium_input - base.medium_risk));
  next.slow_risk = clamp01(base.slow_risk +
                           slow_alpha * (slow_input - base.slow_risk));
  float overall_risk = clamp01(maximum(
      fast_input * 0.45F + next.fast_risk * 0.30F +
          next.medium_risk * 0.17F + next.slow_risk * 0.08F,
      maximum(floor_fall * 0.95F, bathtub_immobility * 0.95F)));

  const float memory_decay_seconds =
      physically_observable ? kObservableMemoryDecaySeconds
                            : kUnobservableMemoryDecaySeconds;
  const float decayed_memory = clamp01(
      base.stale_danger_memory - elapsed_seconds / memory_decay_seconds);
  next.stale_danger_memory = maximum(overall_risk, decayed_memory);
  if (!physically_observable) {
    overall_risk = maximum(overall_risk, next.stale_danger_memory);
  }

  if (overall_risk >= config_.attention_threshold) {
    next.risk_since_us = base.risk_since_us == 0U ? observed_at_us
                                                  : base.risk_since_us;
  } else {
    next.risk_since_us = 0U;
  }
  if (!physically_observable &&
      measurement_failure >= config_.physical_measurement_failure_threshold) {
    next.unavailable_since_us =
        base.unavailable_since_us == 0U ? observed_at_us
                                       : base.unavailable_since_us;
  } else {
    next.unavailable_since_us = 0U;
  }

  BathroomIntegratedSafetyLevel desired =
      desiredLevel(overall_risk, floor_fall, bathtub_immobility);
  const uint32_t unavailable_duration_ms =
      elapsedMilliseconds(observed_at_us, next.unavailable_since_us);
  const bool retained_danger =
      (!physically_observable &&
       dangerRank(base.level) >=
           dangerRank(BathroomIntegratedSafetyLevel::Urgent)) ||
      next.stale_danger_memory >= config_.retained_danger_threshold;
  if (retained_danger &&
      dangerRank(desired) < dangerRank(BathroomIntegratedSafetyLevel::Urgent)) {
    desired = BathroomIntegratedSafetyLevel::Urgent;
  } else if (!retained_danger &&
             unavailable_duration_ms >=
                 config_.measurement_unavailable_confirmation_ms) {
    desired = BathroomIntegratedSafetyLevel::MeasurementUnavailable;
  }

  if (desired == base.level) {
    next.candidate_level = desired;
    next.candidate_since_us = 0U;
  } else {
    if (base.candidate_level != desired || base.candidate_since_us == 0U) {
      next.candidate_level = desired;
      next.candidate_since_us = observed_at_us;
    } else {
      next.candidate_level = base.candidate_level;
      next.candidate_since_us = base.candidate_since_us;
    }
    const uint32_t confirmation_ms =
        transitionConfirmationMs(base.level, desired);
    if (elapsedMilliseconds(observed_at_us, next.candidate_since_us) >=
        confirmation_ms) {
      next.level = desired;
      next.level_since_us = observed_at_us;
      next.candidate_since_us = 0U;
      next.candidate_level = desired;
    }
  }
  if (next.level_since_us == 0U) {
    next.level_since_us = observed_at_us;
  }
  next.observed_at_us = observed_at_us;
  state_ = next;

  output = {};
  output.probe_sequence = probe_sequence;
  output.observed_at_us = observed_at_us;
  output.safety_level = next.level;
  output.floor_fall_evidence = floor_fall;
  output.bathtub_immobility_evidence = bathtub_immobility;
  output.dangerous_posture_evidence = posture;
  output.respiratory_motion_loss_evidence = respiratory_loss;
  output.prolonged_bathing_evidence = prolonged;
  output.occupied_unknown_evidence = occupied_unknown;
  output.nuisance_alternative = nuisance;
  output.measurement_failure_evidence = measurement_failure;
  output.fast_risk = next.fast_risk;
  output.medium_risk = next.medium_risk;
  output.slow_risk = next.slow_risk;
  output.overall_risk = overall_risk;
  output.stale_danger_memory = next.stale_danger_memory;
  output.bathing_time_thermal_context = thermal_context;
  output.analysis_quality = clamp01(
      (fused.quality + csi.analysis_quality + safety.analysis_quality +
       spatial.analysis_quality + respiration.analysis_quality +
       calibration.analysis_quality + session.analysis_quality) /
      7.0F);
  output.level_duration_ms =
      elapsedMilliseconds(observed_at_us, next.level_since_us);
  output.consecutive_risk_duration_ms =
      elapsedMilliseconds(observed_at_us, next.risk_since_us);
  output.measurement_unavailable_duration_ms = unavailable_duration_ms;
  output.room_temperature_available = false;
  output.water_temperature_available = false;
  output.heat_shock_assessment_available = false;
  output.thermal_measurement_unavailable = true;
  output.physically_observable = physically_observable;
  output.evidence_ready = true;

  emitIfDue(output);
  return BathroomIntegratedSafetyUpdateStatus::Accepted;
}

void BathroomIntegratedSafetyAnalyzer::emitIfDue(
    const BathroomIntegratedSafetyEvidence& evidence) {
  const uint64_t interval_us =
      static_cast<uint64_t>(config_.emit_interval_ms) * kUsPerMs;
  if (last_emit_at_us_ != 0U &&
      evidence.observed_at_us < last_emit_at_us_ + interval_us) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_integrated_safety\",\"probe\":%lu,"
      "\"level\":\"%s\",\"floor_fall\":%.3f,"
      "\"bathtub_immobility\":%.3f,\"dangerous_posture\":%.3f,"
      "\"respiratory_motion_loss\":%.3f,\"prolonged_bathing\":%.3f,"
      "\"occupied_unknown\":%.3f,\"nuisance_alternative\":%.3f,"
      "\"measurement_failure\":%.3f,\"fast_risk\":%.3f,"
      "\"medium_risk\":%.3f,\"slow_risk\":%.3f,"
      "\"overall_risk\":%.3f,\"stale_danger_memory\":%.3f,"
      "\"bathing_time_thermal_context\":%.3f,"
      "\"level_duration_ms\":%lu,\"risk_duration_ms\":%lu,"
      "\"measurement_unavailable_duration_ms\":%lu,"
      "\"room_temperature_available\":false,"
      "\"water_temperature_available\":false,"
      "\"heat_shock_assessment_available\":false,"
      "\"thermal_measurement_unavailable\":true,"
      "\"physically_observable\":%s,\"quality\":%.3f,\"ready\":true}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      levelName(evidence.safety_level), evidence.floor_fall_evidence,
      evidence.bathtub_immobility_evidence,
      evidence.dangerous_posture_evidence,
      evidence.respiratory_motion_loss_evidence,
      evidence.prolonged_bathing_evidence,
      evidence.occupied_unknown_evidence, evidence.nuisance_alternative,
      evidence.measurement_failure_evidence, evidence.fast_risk,
      evidence.medium_risk, evidence.slow_risk, evidence.overall_risk,
      evidence.stale_danger_memory,
      evidence.bathing_time_thermal_context,
      static_cast<unsigned long>(evidence.level_duration_ms),
      static_cast<unsigned long>(evidence.consecutive_risk_duration_ms),
      static_cast<unsigned long>(
          evidence.measurement_unavailable_duration_ms),
      evidence.physically_observable ? "true" : "false",
      evidence.analysis_quality);
}

void BathroomIntegratedSafetyAnalyzer::reset() {
  state_ = {};
  before_current_probe_ = {};
  current_probe_sequence_ = 0U;
  last_emit_at_us_ = 0U;
  has_current_probe_ = false;
}

}  // namespace atom::radar
