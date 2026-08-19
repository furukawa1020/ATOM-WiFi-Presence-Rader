#include "BathroomSafetyEvidence.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace atom::radar {

BathroomSafetyEvidenceAnalyzer::BathroomSafetyEvidenceAnalyzer(
    BathroomSafetyEvidenceConfig config)
    : config_(config) {
  config_.history_points =
      std::min<uint16_t>(config_.history_points, kHistoryCapacity);
  reset();
}

void BathroomSafetyEvidenceAnalyzer::reset() {
  history_head_ = 0;
  history_count_ = 0;
  last_emit_at_us_ = -1;
}

float BathroomSafetyEvidenceAnalyzer::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomSafetyEvidenceAnalyzer::maximum(float first, float second) {
  return first > second ? first : second;
}

float BathroomSafetyEvidenceAnalyzer::mean(float sum, std::size_t count) {
  return count == 0 ? 0.0F : sum / static_cast<float>(count);
}

const BathroomSafetyEvidenceAnalyzer::EvidencePoint &
BathroomSafetyEvidenceAnalyzer::fromNewest(std::size_t offset) const {
  const std::size_t index =
      (history_head_ + kHistoryCapacity - 1 - offset) % kHistoryCapacity;
  return history_[index];
}

BathroomSafetyEvidenceUpdateStatus BathroomSafetyEvidenceAnalyzer::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    const BathroomCsiEvidence &bathroom_evidence,
    BathroomSafetyEvidence &evidence) {
  if (observed_at_us < 0 ||
      bathroom_evidence.probe_sequence != observation.anchor_probe_sequence ||
      !std::isfinite(observation.quality) ||
      !std::isfinite(bathroom_evidence.analysis_quality)) {
    return BathroomSafetyEvidenceUpdateStatus::InvalidObservation;
  }

  EvidencePoint point{};
  point.probe_sequence = observation.anchor_probe_sequence;
  point.observed_at_us = observed_at_us;
  const float doppler_shape =
      clamp01(0.55F * observation.doppler_energy +
              0.45F * observation.doppler_bandwidth);
  point.impact =
      clamp01(0.30F * observation.impulse_score + 0.20F * doppler_shape +
              0.20F * observation.innovation_motion +
              0.18F * observation.delay_domain_motion +
              0.12F * observation.complex_ratio_motion);
  point.human_motion =
      clamp01(bathroom_evidence.nuisance_reduced_human_motion);
  point.stillness = clamp01(observation.stillness_score);
  point.baseline_shift = clamp01(observation.baseline_shift);
  point.delay_motion = clamp01(observation.delay_domain_motion);
  point.delay_spread = clamp01(observation.delay_spread);
  point.dynamic_tap_concentration =
      clamp01(observation.dynamic_tap_concentration);
  point.respiration =
      clamp01(0.22F * observation.respiration_power +
              0.22F * observation.respiration_spectral_snr +
              0.20F * observation.respiration_harmonicity +
              0.20F * observation.respiration_coherence +
              0.16F * observation.respiration_rate_agreement);
  point.door_alternative =
      clamp01(bathroom_evidence.door_transient_like_evidence);
  point.persistent_nuisance =
      clamp01(maximum(
          bathroom_evidence.nuisance_confidence,
          maximum(bathroom_evidence.fan_like_evidence,
                  maximum(bathroom_evidence.shower_like_evidence,
                          bathroom_evidence.water_drift_like_evidence))));
  point.quality =
      clamp01(0.50F * observation.quality +
              0.50F * bathroom_evidence.analysis_quality);
  if (!observation.physically_observable) {
    point.quality *= 0.20F;
  }

  BathroomSafetyEvidenceUpdateStatus status =
      BathroomSafetyEvidenceUpdateStatus::Updated;
  if (history_count_ > 0) {
    const std::size_t latest_index =
        (history_head_ + kHistoryCapacity - 1) % kHistoryCapacity;
    const int32_t sequence_delta = static_cast<int32_t>(
        point.probe_sequence - history_[latest_index].probe_sequence);
    if (sequence_delta == 0) {
      point.observed_at_us = history_[latest_index].observed_at_us;
      history_[latest_index] = point;
      status = BathroomSafetyEvidenceUpdateStatus::ReplacedSameProbe;
    } else if (sequence_delta < 0) {
      return BathroomSafetyEvidenceUpdateStatus::NonMonotonicProbe;
    }
  }

  if (status == BathroomSafetyEvidenceUpdateStatus::Updated) {
    history_[history_head_] = point;
    history_head_ = (history_head_ + 1) % kHistoryCapacity;
    history_count_ =
        std::min<std::size_t>(history_count_ + 1, config_.history_points);
  }

  buildEvidence(evidence);
  emitJsonIfDue(evidence);
  return status;
}

void BathroomSafetyEvidenceAnalyzer::buildEvidence(
    BathroomSafetyEvidence &evidence) const {
  evidence = {};
  if (history_count_ == 0) {
    return;
  }

  const EvidencePoint &newest = fromNewest(0);
  evidence.probe_sequence = newest.probe_sequence;
  evidence.evaluated_at_us = newest.observed_at_us;
  evidence.history_points = static_cast<uint16_t>(history_count_);
  evidence.short_history_ready = history_count_ >= config_.sample_rate_hz;
  evidence.medium_history_ready = history_count_ >= config_.sample_rate_hz * 5U;
  evidence.long_history_ready = history_count_ >= config_.sample_rate_hz * 15U;

  std::size_t impact_offset = 0;
  float best_impact = -1.0F;
  for (std::size_t offset = 0; offset < history_count_; ++offset) {
    const float recency =
        1.0F - 0.08F * static_cast<float>(offset) /
                   static_cast<float>(kHistoryCapacity);
    const float candidate = fromNewest(offset).impact * recency;
    if (candidate > best_impact) {
      best_impact = candidate;
      impact_offset = offset;
    }
  }

  const EvidencePoint &impact = fromNewest(impact_offset);
  evidence.impact_probe_sequence = impact.probe_sequence;
  const int64_t elapsed_us = newest.observed_at_us - impact.observed_at_us;
  evidence.milliseconds_since_impact = static_cast<uint32_t>(
      std::clamp<int64_t>(elapsed_us / 1000, 0, UINT32_MAX));
  evidence.impact_evidence = clamp01(impact.impact);

  float pre_human_sum = 0.0F;
  float pre_human_max = 0.0F;
  float pre_baseline_sum = 0.0F;
  float pre_delay_spread_sum = 0.0F;
  std::size_t pre_count = 0;
  const std::size_t pre_end = std::min<std::size_t>(
      history_count_, impact_offset + config_.pre_impact_points + 1U);
  for (std::size_t offset = impact_offset + 1; offset < pre_end; ++offset) {
    const EvidencePoint &point = fromNewest(offset);
    pre_human_sum += point.human_motion;
    pre_human_max = maximum(pre_human_max, point.human_motion);
    pre_baseline_sum += point.baseline_shift;
    pre_delay_spread_sum += point.delay_spread;
    ++pre_count;
  }
  evidence.pre_impact_human_motion =
      clamp01(0.60F * mean(pre_human_sum, pre_count) +
              0.40F * pre_human_max);

  float post_quality_sum = impact.quality;
  float nuisance_sum = 0.0F;
  std::size_t post_count = 0;
  for (std::size_t offset = 0; offset < impact_offset; ++offset) {
    const EvidencePoint &point = fromNewest(offset);
    post_quality_sum += point.quality;
    nuisance_sum += point.persistent_nuisance;
    ++post_count;
  }

  const std::size_t relocation_start =
      impact_offset > config_.pre_impact_points
          ? impact_offset - config_.pre_impact_points
          : 0;
  float post_baseline_sum = 0.0F;
  float post_delay_motion_sum = 0.0F;
  float post_delay_spread_sum = 0.0F;
  float post_tap_sum = 0.0F;
  std::size_t relocation_count = 0;
  for (std::size_t offset = relocation_start; offset < impact_offset; ++offset) {
    const EvidencePoint &point = fromNewest(offset);
    post_baseline_sum += point.baseline_shift;
    post_delay_motion_sum += point.delay_motion;
    post_delay_spread_sum += point.delay_spread;
    post_tap_sum += point.dynamic_tap_concentration;
    ++relocation_count;
  }
  const float baseline_delta =
      std::fabs(mean(post_baseline_sum, relocation_count) -
                mean(pre_baseline_sum, pre_count));
  const float delay_spread_delta =
      std::fabs(mean(post_delay_spread_sum, relocation_count) -
                mean(pre_delay_spread_sum, pre_count));
  evidence.relocation_evidence =
      clamp01(0.35F * baseline_delta + 0.20F * delay_spread_delta +
              0.28F * mean(post_delay_motion_sum, relocation_count) +
              0.17F * mean(post_tap_sum, relocation_count));

  float immobility_sum = 0.0F;
  float recovery_sum = 0.0F;
  float recovery_max = 0.0F;
  float respiration_sum = 0.0F;
  std::size_t settled_count = 0;
  std::size_t recovery_count = 0;
  std::size_t respiration_count = 0;
  if (impact_offset > config_.impact_settling_points) {
    const std::size_t settled_end =
        impact_offset - config_.impact_settling_points;
    for (std::size_t offset = 0; offset < settled_end; ++offset) {
      const EvidencePoint &point = fromNewest(offset);
      immobility_sum +=
          clamp01(0.65F * point.stillness +
                  0.35F * (1.0F - point.human_motion));
      respiration_sum += point.respiration;
      ++settled_count;
      ++respiration_count;
      if (offset < config_.recovery_window_points) {
        recovery_sum += point.human_motion;
        recovery_max = maximum(recovery_max, point.human_motion);
        ++recovery_count;
      }
    }
  }

  const float post_maturity =
      clamp01(static_cast<float>(settled_count) /
              static_cast<float>(config_.immobility_maturity_points));
  evidence.post_impact_immobility =
      clamp01(mean(immobility_sum, settled_count) * post_maturity);
  evidence.recovery_motion =
      clamp01(0.65F * mean(recovery_sum, recovery_count) +
              0.35F * recovery_max);
  evidence.respiration_after_impact =
      clamp01(mean(respiration_sum, respiration_count));

  float door_max = impact.door_alternative;
  const std::size_t door_start = impact_offset > 5 ? impact_offset - 5 : 0;
  const std::size_t door_end =
      std::min<std::size_t>(history_count_, impact_offset + 6);
  for (std::size_t offset = door_start; offset < door_end; ++offset) {
    door_max = maximum(door_max, fromNewest(offset).door_alternative);
  }
  const float object_like =
      clamp01(evidence.impact_evidence *
              (1.0F - evidence.pre_impact_human_motion) *
              (1.0F - evidence.relocation_evidence));
  evidence.door_or_object_alternative =
      clamp01(0.65F * door_max + 0.35F * object_like);
  evidence.persistent_nuisance_alternative =
      clamp01(mean(nuisance_sum, post_count));
  evidence.analysis_quality =
      clamp01(post_quality_sum / static_cast<float>(post_count + 1));

  const float motion_context =
      0.35F + 0.65F * maximum(evidence.pre_impact_human_motion,
                              evidence.relocation_evidence);
  const float immobility_context =
      0.25F + 0.75F * evidence.post_impact_immobility;
  const float recovery_attenuation =
      1.0F - 0.45F * evidence.recovery_motion;
  const float alternative_attenuation =
      (1.0F - 0.18F * evidence.door_or_object_alternative) *
      (1.0F - 0.12F * evidence.persistent_nuisance_alternative);
  evidence.fall_sequence_evidence =
      clamp01(evidence.impact_evidence * motion_context *
              immobility_context * post_maturity *
              evidence.analysis_quality * recovery_attenuation *
              alternative_attenuation);
  evidence.dangerous_immobility_evidence =
      clamp01(evidence.fall_sequence_evidence *
              evidence.post_impact_immobility *
              (1.0F - 0.35F * evidence.recovery_motion));
  evidence.temporal_sequence_ready = pre_count >= 3 && settled_count >= 3;
  evidence.evidence_ready = evidence.short_history_ready && post_count >= 5;
}

void BathroomSafetyEvidenceAnalyzer::emitJsonIfDue(
    const BathroomSafetyEvidence &evidence) {
  if (last_emit_at_us_ >= 0 &&
      evidence.evaluated_at_us - last_emit_at_us_ < 1000000) {
    return;
  }
  last_emit_at_us_ = evidence.evaluated_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_safety_evidence\","
      "\"probe\":%lu,\"impact_probe\":%lu,\"since_impact_ms\":%lu,"
      "\"history\":%u,\"impact\":%.3f,\"pre_impact_human\":%.3f,"
      "\"relocation\":%.3f,\"post_impact_immobility\":%.3f,"
      "\"recovery\":%.3f,\"post_impact_respiration\":%.3f,"
      "\"door_or_object_alternative\":%.3f,"
      "\"persistent_nuisance_alternative\":%.3f,"
      "\"fall_sequence\":%.3f,\"dangerous_immobility\":%.3f,"
      "\"quality\":%.3f,\"short_ready\":%s,\"medium_ready\":%s,"
      "\"long_ready\":%s,\"temporal_ready\":%s,\"ready\":%s}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      static_cast<unsigned long>(evidence.impact_probe_sequence),
      static_cast<unsigned long>(evidence.milliseconds_since_impact),
      evidence.history_points, evidence.impact_evidence,
      evidence.pre_impact_human_motion, evidence.relocation_evidence,
      evidence.post_impact_immobility, evidence.recovery_motion,
      evidence.respiration_after_impact,
      evidence.door_or_object_alternative,
      evidence.persistent_nuisance_alternative,
      evidence.fall_sequence_evidence,
      evidence.dangerous_immobility_evidence, evidence.analysis_quality,
      evidence.short_history_ready ? "true" : "false",
      evidence.medium_history_ready ? "true" : "false",
      evidence.long_history_ready ? "true" : "false",
      evidence.temporal_sequence_ready ? "true" : "false",
      evidence.evidence_ready ? "true" : "false");
}

}  // namespace atom::radar
