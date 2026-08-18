#include "BathroomCsiEvidence.hpp"

#include <cmath>
#include <cstring>

#include <protocol.hpp>

namespace atom::radar {

BathroomCsiEvidenceAnalyzer::BathroomCsiEvidenceAnalyzer(BathroomCsiEvidenceConfig config)
    : config_(config) {
  reset();
}

void BathroomCsiEvidenceAnalyzer::reset() {
  std::memset(history_, 0, sizeof(history_));
  history_head_ = 0;
  history_count_ = 0;
}

BathroomCsiEvidenceUpdateStatus BathroomCsiEvidenceAnalyzer::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    BathroomCsiEvidence &evidence) {
  evidence = {};
  if (observation.active_links == 0U || observed_at_us <= 0 ||
      !std::isfinite(observation.quality) || !std::isfinite(observation.innovation_motion)) {
    return BathroomCsiEvidenceUpdateStatus::InvalidObservation;
  }

  const EvidencePoint point{
      observation.anchor_probe_sequence,
      observed_at_us,
      clamp01(observation.quality),
      clamp01(observation.link_agreement),
      clamp01(observation.synchronization_quality),
      clamp01(observation.amplitude_motion),
      clamp01(observation.phase_coherence),
      clamp01(observation.doppler_energy),
      clamp01(observation.doppler_centroid),
      clamp01(observation.doppler_bandwidth),
      clamp01(observation.doppler_asymmetry),
      clamp01(observation.delay_domain_motion),
      clamp01(observation.delay_spread),
      clamp01(observation.dynamic_tap_concentration),
      clamp01(observation.baseline_shift),
      clamp01(observation.broadband_nuisance),
      clamp01(observation.background_explained_ratio),
      clamp01(observation.innovation_motion),
      clamp01(observation.impulse_score),
      clamp01(observation.stillness_score),
  };

  BathroomCsiEvidenceUpdateStatus status = BathroomCsiEvidenceUpdateStatus::Updated;
  if (history_count_ > 0U) {
    EvidencePoint &newest =
        history_[(history_head_ + kHistoryCapacity - 1U) % kHistoryCapacity];
    if (point.probe_sequence == newest.probe_sequence) {
      newest = point;
      status = BathroomCsiEvidenceUpdateStatus::ReplacedSameProbe;
      buildEvidence(evidence);
      return status;
    }
    if (!protocol::isSequenceNewer(point.probe_sequence, newest.probe_sequence)) {
      return BathroomCsiEvidenceUpdateStatus::NonMonotonicProbe;
    }
  }

  history_[history_head_] = point;
  history_head_ = (history_head_ + 1U) % kHistoryCapacity;
  if (history_count_ < kHistoryCapacity) {
    ++history_count_;
  }
  buildEvidence(evidence);
  return status;
}

void BathroomCsiEvidenceAnalyzer::buildEvidence(BathroomCsiEvidence &evidence) const {
  if (history_count_ == 0U) {
    evidence = {};
    return;
  }

  const std::size_t short_points =
      history_count_ < config_.short_window_points ? history_count_ : config_.short_window_points;
  const std::size_t medium_points =
      history_count_ < config_.medium_window_points ? history_count_ : config_.medium_window_points;
  const std::size_t long_points =
      history_count_ < config_.long_window_points ? history_count_ : config_.long_window_points;
  const float short_maturity =
      clamp01(static_cast<float>(short_points) /
              static_cast<float>(config_.short_window_points > 0U
                                     ? config_.short_window_points
                                     : 1U));
  const float medium_maturity =
      clamp01(static_cast<float>(medium_points) /
              static_cast<float>(config_.medium_window_points > 0U
                                     ? config_.medium_window_points
                                     : 1U));
  const float long_maturity =
      clamp01(static_cast<float>(long_points) /
              static_cast<float>(config_.long_window_points > 0U
                                     ? config_.long_window_points
                                     : 1U));

  const float medium_doppler = average(medium_points, &EvidencePoint::doppler_energy);
  const float medium_bandwidth = average(medium_points, &EvidencePoint::doppler_bandwidth);
  const float medium_centroid = average(medium_points, &EvidencePoint::doppler_centroid);
  const float centroid_variance =
      variance(medium_points, &EvidencePoint::doppler_centroid, medium_centroid);
  const float centroid_stability =
      clamp01(1.0F - std::sqrt(centroid_variance) / 0.12F);
  const float periodic_persistence =
      persistence(medium_points, &EvidencePoint::doppler_energy,
                  config_.periodic_motion_threshold);
  const float narrowband = clamp01(1.0F - medium_bandwidth);
  const float medium_background =
      average(medium_points, &EvidencePoint::background_explained);
  const float medium_innovation = average(medium_points, &EvidencePoint::innovation_motion);
  const float periodic_gate =
      std::sqrt(clamp01(medium_doppler) * clamp01(periodic_persistence));
  const float fan_like =
      medium_maturity * periodic_gate *
      clamp01(0.38F * narrowband + 0.27F * centroid_stability +
              0.25F * medium_background + 0.10F * (1.0F - medium_innovation));

  const float medium_broadband = average(medium_points, &EvidencePoint::broadband_nuisance);
  const float medium_delay_spread = average(medium_points, &EvidencePoint::delay_spread);
  const float broadband_persistence =
      persistence(medium_points, &EvidencePoint::broadband_nuisance,
                  config_.broadband_threshold);
  const float short_impulse = maximum(short_points, &EvidencePoint::impulse);
  const float shower_core =
      0.34F * medium_broadband + 0.22F * medium_bandwidth +
      0.19F * medium_delay_spread + 0.15F * medium_background +
      0.10F * medium_doppler;
  const float shower_like =
      medium_maturity * clamp01(shower_core) * std::sqrt(clamp01(broadband_persistence)) *
      (1.0F - 0.35F * short_impulse);

  const float long_baseline = average(long_points, &EvidencePoint::baseline_shift);
  const float baseline_slope = trend(long_points, &EvidencePoint::baseline_shift);
  const float accumulated_drift =
      clamp01(std::fabs(baseline_slope) * static_cast<float>(long_points > 0U
                                                                 ? long_points - 1U
                                                                 : 0U) /
              0.25F);
  const float baseline_persistence =
      persistence(long_points, &EvidencePoint::baseline_shift,
                  config_.baseline_shift_threshold);
  const float long_delay_motion = average(long_points, &EvidencePoint::delay_motion);
  const float long_background = average(long_points, &EvidencePoint::background_explained);
  const float low_doppler = clamp01(1.0F - average(long_points, &EvidencePoint::doppler_energy) /
                                               0.35F);
  const float water_like =
      long_maturity * low_doppler *
      clamp01(0.36F * accumulated_drift + 0.20F * long_baseline +
              0.18F * baseline_persistence + 0.14F * long_delay_motion +
              0.12F * long_background);

  const float short_innovation = maximum(short_points, &EvidencePoint::innovation_motion);
  const float short_tap_concentration =
      maximum(short_points, &EvidencePoint::dynamic_tap_concentration);
  const float recent_baseline = average(short_points, &EvidencePoint::baseline_shift);
  const std::size_t prior_points = medium_points > short_points ? medium_points - short_points : 0U;
  const float prior_baseline =
      prior_points > 0U
          ? rangeAverage(short_points, prior_points, &EvidencePoint::baseline_shift)
          : recent_baseline;
  const float baseline_step = clamp01(std::fabs(recent_baseline - prior_baseline) / 0.30F);
  const float medium_impulse = average(medium_points, &EvidencePoint::impulse);
  const float impulse_transience = clamp01((short_impulse - medium_impulse) / 0.55F);
  const float door_like =
      short_maturity *
      clamp01(0.34F * short_impulse + 0.26F * short_innovation +
              0.20F * baseline_step + 0.20F * short_tap_concentration) *
      (0.45F + 0.55F * impulse_transience) *
      (1.0F - 0.30F * broadband_persistence);

  const float short_phase_coherence = average(short_points, &EvidencePoint::phase_coherence);
  const float short_delay_motion = average(short_points, &EvidencePoint::delay_motion);
  const float short_link_agreement = average(short_points, &EvidencePoint::link_agreement);
  const float short_synchronization = average(short_points, &EvidencePoint::synchronization);
  const float short_doppler = average(short_points, &EvidencePoint::doppler_energy);
  const float short_background = average(short_points, &EvidencePoint::background_explained);
  const float human_motion =
      short_maturity *
      clamp01(0.32F * average(short_points, &EvidencePoint::innovation_motion) +
              0.18F * short_phase_coherence + 0.20F * short_delay_motion +
              0.15F * short_link_agreement + 0.15F * short_doppler) *
      (0.60F + 0.40F * short_synchronization) *
      (1.0F - 0.35F * short_background);

  const float persistent_nuisance = fan_like > shower_like ? fan_like : shower_like;
  const float environmental_nuisance =
      persistent_nuisance > water_like ? persistent_nuisance : water_like;
  const float nuisance_reduced_human =
      clamp01(human_motion * (1.0F - 0.65F * environmental_nuisance));
  const float nuisance_confidence =
      maximumOfFour(fan_like, shower_like, water_like, door_like);
  const float unexplained_innovation =
      clamp01(short_innovation * (1.0F - 0.70F * nuisance_confidence));

  const float quality = average(medium_points, &EvidencePoint::quality);
  const float agreement = average(medium_points, &EvidencePoint::link_agreement);
  const float synchronization = average(medium_points, &EvidencePoint::synchronization);
  evidence.probe_sequence = fromNewest(0).probe_sequence;
  evidence.observed_at_us = fromNewest(0).observed_at_us;
  evidence.history_points = static_cast<uint16_t>(history_count_);
  evidence.fan_like_evidence = clamp01(fan_like);
  evidence.shower_like_evidence = clamp01(shower_like);
  evidence.water_drift_like_evidence = clamp01(water_like);
  evidence.door_transient_like_evidence = clamp01(door_like);
  evidence.human_motion_evidence = clamp01(human_motion);
  evidence.nuisance_reduced_human_motion = nuisance_reduced_human;
  evidence.unexplained_innovation = unexplained_innovation;
  evidence.nuisance_confidence = nuisance_confidence;
  evidence.analysis_quality =
      clamp01(quality * (0.60F + 0.20F * agreement + 0.20F * synchronization));
  evidence.short_window_ready = history_count_ >= config_.short_window_points;
  evidence.medium_window_ready = history_count_ >= config_.medium_window_points;
  evidence.long_window_ready = history_count_ >= config_.long_window_points;
  evidence.evidence_ready = history_count_ >= config_.minimum_ready_points;
}

const BathroomCsiEvidenceAnalyzer::EvidencePoint &BathroomCsiEvidenceAnalyzer::fromNewest(
    std::size_t offset) const {
  const std::size_t index =
      (history_head_ + kHistoryCapacity - 1U - offset) % kHistoryCapacity;
  return history_[index];
}

float BathroomCsiEvidenceAnalyzer::average(std::size_t points,
                                           float EvidencePoint::*member) const {
  return rangeAverage(0U, points, member);
}

float BathroomCsiEvidenceAnalyzer::rangeAverage(std::size_t newest_offset,
                                                std::size_t points,
                                                float EvidencePoint::*member) const {
  if (points == 0U || newest_offset >= history_count_) {
    return 0.0F;
  }
  const std::size_t available = history_count_ - newest_offset;
  const std::size_t count = points < available ? points : available;
  float sum = 0.0F;
  for (std::size_t offset = 0; offset < count; ++offset) {
    sum += fromNewest(newest_offset + offset).*member;
  }
  return sum / static_cast<float>(count);
}

float BathroomCsiEvidenceAnalyzer::variance(std::size_t points,
                                            float EvidencePoint::*member,
                                            float center) const {
  if (points == 0U) {
    return 0.0F;
  }
  const std::size_t count = points < history_count_ ? points : history_count_;
  float sum = 0.0F;
  for (std::size_t offset = 0; offset < count; ++offset) {
    const float delta = fromNewest(offset).*member - center;
    sum += delta * delta;
  }
  return count > 0U ? sum / static_cast<float>(count) : 0.0F;
}

float BathroomCsiEvidenceAnalyzer::maximum(std::size_t points,
                                           float EvidencePoint::*member) const {
  const std::size_t count = points < history_count_ ? points : history_count_;
  float result = 0.0F;
  for (std::size_t offset = 0; offset < count; ++offset) {
    const float value = fromNewest(offset).*member;
    if (value > result) {
      result = value;
    }
  }
  return result;
}

float BathroomCsiEvidenceAnalyzer::persistence(std::size_t points,
                                               float EvidencePoint::*member,
                                               float threshold) const {
  const std::size_t count = points < history_count_ ? points : history_count_;
  if (count == 0U) {
    return 0.0F;
  }
  std::size_t active = 0;
  for (std::size_t offset = 0; offset < count; ++offset) {
    if (fromNewest(offset).*member >= threshold) {
      ++active;
    }
  }
  return static_cast<float>(active) / static_cast<float>(count);
}

float BathroomCsiEvidenceAnalyzer::trend(std::size_t points,
                                         float EvidencePoint::*member) const {
  const std::size_t count = points < history_count_ ? points : history_count_;
  if (count < 2U) {
    return 0.0F;
  }
  float sum_x = 0.0F;
  float sum_y = 0.0F;
  float sum_xx = 0.0F;
  float sum_xy = 0.0F;
  for (std::size_t chronological = 0; chronological < count; ++chronological) {
    const float x = static_cast<float>(chronological);
    const float y = fromNewest(count - 1U - chronological).*member;
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }
  const float denominator = static_cast<float>(count) * sum_xx - sum_x * sum_x;
  return std::fabs(denominator) > 1.0e-6F
             ? (static_cast<float>(count) * sum_xy - sum_x * sum_y) / denominator
             : 0.0F;
}

float BathroomCsiEvidenceAnalyzer::clamp01(float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

float BathroomCsiEvidenceAnalyzer::maximumOfFour(float first, float second, float third,
                                                 float fourth) {
  float result = first > second ? first : second;
  result = result > third ? result : third;
  return result > fourth ? result : fourth;
}

}  // namespace atom::radar
