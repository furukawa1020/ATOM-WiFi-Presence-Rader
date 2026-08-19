#include "BathroomRespirationEvidence.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace atom::radar {

namespace {

constexpr float kMetricScale = 1.0F / 65535.0F;

}  // namespace

BathroomRespirationEvidenceAnalyzer::BathroomRespirationEvidenceAnalyzer(
    BathroomRespirationEvidenceConfig config)
    : config_(config) {
  config_.history_points =
      std::min<uint16_t>(config_.history_points, kHistoryCapacity);
  reset();
}

void BathroomRespirationEvidenceAnalyzer::reset() {
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    links_[index] = {};
  }
  history_head_ = 0;
  history_count_ = 0;
  last_emit_at_us_ = -1;
}

float BathroomRespirationEvidenceAnalyzer::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomRespirationEvidenceAnalyzer::maximum(float first,
                                                    float second) {
  return first > second ? first : second;
}

float BathroomRespirationEvidenceAnalyzer::absolute(float value) {
  return value < 0.0F ? -value : value;
}

BathroomRespirationEvidenceAnalyzer::LinkState *
BathroomRespirationEvidenceAnalyzer::findOrCreateLink(uint8_t receiver_id) {
  LinkState *empty = nullptr;
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    if (links_[index].allocated && links_[index].receiver_id == receiver_id) {
      return &links_[index];
    }
    if (!links_[index].allocated && empty == nullptr) {
      empty = &links_[index];
    }
  }
  if (empty != nullptr) {
    empty->allocated = true;
    empty->receiver_id = receiver_id;
  }
  return empty;
}

float BathroomRespirationEvidenceAnalyzer::metric(
    const CsiObservationPacket &packet, CsiObservationMetric index) const {
  return static_cast<float>(packet.metrics[static_cast<std::size_t>(index)]) *
         kMetricScale;
}

BathroomRespirationPacketStatus
BathroomRespirationEvidenceAnalyzer::ingestPacket(
    const uint8_t *data, std::size_t length, uint32_t expected_system_id,
    int64_t received_at_us) {
  CsiObservationPacket packet{};
  if (!decodeCsiObservationPacket(data, length, packet) ||
      packet.packet_type != kCsiObservationPacketType) {
    return BathroomRespirationPacketStatus::NotCsiObservation;
  }
  if (packet.system_id != expected_system_id) {
    return BathroomRespirationPacketStatus::WrongSystem;
  }

  LinkState *link = findOrCreateLink(packet.receiver_id);
  if (link == nullptr) {
    return BathroomRespirationPacketStatus::LinkCapacityExceeded;
  }
  BathroomRespirationPacketStatus status =
      BathroomRespirationPacketStatus::Accepted;
  if (link->has_packet) {
    const int32_t sequence_delta =
        static_cast<int32_t>(packet.probe_sequence - link->probe_sequence);
    if (sequence_delta < 0) {
      return BathroomRespirationPacketStatus::NonMonotonicProbe;
    }
    if (sequence_delta == 0) {
      status = BathroomRespirationPacketStatus::ReplacedSameProbe;
    }
  }

  link->has_packet = true;
  link->probe_sequence = packet.probe_sequence;
  link->received_at_us = received_at_us;
  link->power = metric(packet, CsiObservationMetric::RespirationPower);
  link->rate_normalized =
      metric(packet, CsiObservationMetric::RespirationRateNormalized);
  link->spectral_snr =
      metric(packet, CsiObservationMetric::RespirationSpectralSnr);
  link->harmonicity =
      metric(packet, CsiObservationMetric::RespirationHarmonicity);
  link->coherence =
      metric(packet, CsiObservationMetric::RespirationCoherence);
  link->subcarrier_reliability =
      metric(packet, CsiObservationMetric::SubcarrierReliability);
  link->quality = metric(packet, CsiObservationMetric::Quality);
  return status;
}

const BathroomRespirationEvidenceAnalyzer::HistoryPoint &
BathroomRespirationEvidenceAnalyzer::fromNewest(std::size_t offset) const {
  const std::size_t index =
      (history_head_ + kHistoryCapacity - 1 - offset) % kHistoryCapacity;
  return history_[index];
}

bool BathroomRespirationEvidenceAnalyzer::trackedRateBeforeCurrent(
    bool replacing_latest, float &rate) const {
  const std::size_t start = replacing_latest && history_count_ > 0 ? 1 : 0;
  float weighted_rate = 0.0F;
  float weight_sum = 0.0F;
  for (std::size_t offset = start; offset < history_count_; ++offset) {
    const HistoryPoint &point = fromNewest(offset);
    const float recency =
        1.0F - 0.70F * static_cast<float>(offset) /
                   static_cast<float>(kHistoryCapacity);
    const float weight = point.confidence * recency;
    weighted_rate += point.rate_normalized * weight;
    weight_sum += weight;
  }
  if (weight_sum <= 0.001F) {
    return false;
  }
  rate = clamp01(weighted_rate / weight_sum);
  return true;
}

BathroomRespirationEvidenceUpdateStatus
BathroomRespirationEvidenceAnalyzer::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSafetyEvidence &safety_evidence,
    const BathroomSpatialEvidence &spatial_evidence,
    BathroomRespirationEvidence &respiration_evidence) {
  if (observed_at_us < 0 ||
      observation.anchor_probe_sequence != bathroom_evidence.probe_sequence ||
      observation.anchor_probe_sequence != safety_evidence.probe_sequence ||
      observation.anchor_probe_sequence != spatial_evidence.probe_sequence ||
      !std::isfinite(observation.respiration_rate_normalized) ||
      !std::isfinite(observation.quality)) {
    return BathroomRespirationEvidenceUpdateStatus::InvalidObservation;
  }

  bool replacing_latest = false;
  if (history_count_ > 0) {
    const HistoryPoint &latest = fromNewest(0);
    const int32_t sequence_delta = static_cast<int32_t>(
        observation.anchor_probe_sequence - latest.probe_sequence);
    if (sequence_delta < 0) {
      return BathroomRespirationEvidenceUpdateStatus::NonMonotonicProbe;
    }
    replacing_latest = sequence_delta == 0;
  }

  float link_rates[kMaximumLinks]{};
  float link_weights[kMaximumLinks]{};
  std::size_t candidate_count = 0;
  uint8_t active_links = 0;
  float link_confidence_sum = 0.0F;
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    const LinkState &link = links_[index];
    if (!link.allocated || !link.has_packet) {
      continue;
    }
    const int32_t sequence_offset = static_cast<int32_t>(
        observation.anchor_probe_sequence - link.probe_sequence);
    const int64_t age_us = observed_at_us - link.received_at_us;
    if (absolute(static_cast<float>(sequence_offset)) >
            static_cast<float>(config_.maximum_sequence_skew) ||
        age_us < 0 || age_us > config_.link_freshness_us) {
      continue;
    }
    ++active_links;
    const float confidence = clamp01(
        0.23F * link.power + 0.20F * link.spectral_snr +
        0.17F * link.harmonicity + 0.17F * link.coherence +
        0.12F * link.subcarrier_reliability + 0.11F * link.quality);
    link_confidence_sum += confidence;
    if (confidence >= config_.minimum_link_confidence &&
        candidate_count < kMaximumLinks) {
      link_rates[candidate_count] = link.rate_normalized;
      link_weights[candidate_count] = confidence;
      ++candidate_count;
    }
  }

  for (std::size_t outer = 1; outer < candidate_count; ++outer) {
    const float rate = link_rates[outer];
    const float weight = link_weights[outer];
    std::size_t inner = outer;
    while (inner > 0 && link_rates[inner - 1] > rate) {
      link_rates[inner] = link_rates[inner - 1];
      link_weights[inner] = link_weights[inner - 1];
      --inner;
    }
    link_rates[inner] = rate;
    link_weights[inner] = weight;
  }

  float total_candidate_weight = 0.0F;
  for (std::size_t index = 0; index < candidate_count; ++index) {
    total_candidate_weight += link_weights[index];
  }
  float median_rate = observation.respiration_rate_normalized;
  if (candidate_count > 0) {
    float cumulative = 0.0F;
    for (std::size_t index = 0; index < candidate_count; ++index) {
      cumulative += link_weights[index];
      if (cumulative >= total_candidate_weight * 0.5F) {
        median_rate = link_rates[index];
        break;
      }
    }
  }

  float robust_rate_sum = 0.0F;
  float robust_weight_sum = 0.0F;
  for (std::size_t index = 0; index < candidate_count; ++index) {
    if (absolute(link_rates[index] - median_rate) <= 0.18F) {
      robust_rate_sum += link_rates[index] * link_weights[index];
      robust_weight_sum += link_weights[index];
    }
  }
  const float link_rate = robust_weight_sum > 0.001F
                              ? robust_rate_sum / robust_weight_sum
                              : observation.respiration_rate_normalized;

  float pair_agreement_sum = 0.0F;
  std::size_t pair_count = 0;
  for (std::size_t first = 0; first < candidate_count; ++first) {
    for (std::size_t second = first + 1; second < candidate_count; ++second) {
      pair_agreement_sum +=
          clamp01(1.0F - absolute(link_rates[first] - link_rates[second]) /
                               0.25F);
      ++pair_count;
    }
  }
  const float link_rate_agreement =
      pair_count > 0
          ? pair_agreement_sum / static_cast<float>(pair_count)
          : observation.respiration_rate_agreement;
  const float average_link_confidence =
      active_links == 0
          ? 0.0F
          : link_confidence_sum / static_cast<float>(active_links);
  const float link_coverage =
      static_cast<float>(active_links) /
      static_cast<float>(kMaximumM5AtomCsiReceivers);
  const float multi_link_support =
      clamp01(link_coverage * (0.35F + 0.65F * link_rate_agreement) *
              (0.35F + 0.65F * average_link_confidence));

  const float periodic_nuisance = clamp01(
      0.28F * bathroom_evidence.fan_like_evidence +
      0.24F * bathroom_evidence.water_drift_like_evidence +
      0.20F * bathroom_evidence.shower_like_evidence +
      0.16F * observation.broadband_nuisance +
      0.12F * observation.doppler_bandwidth);
  float measurement_loss = clamp01(
      1.0F -
      (0.28F * observation.quality + 0.20F * link_coverage +
       0.18F * observation.synchronization_quality +
       0.16F * observation.subcarrier_reliability +
       0.10F * spatial_evidence.analysis_quality +
       0.08F * observation.link_agreement));
  if (!observation.physically_observable) {
    measurement_loss = maximum(measurement_loss, 0.90F);
  }
  const float position_robustness = clamp01(
      (0.45F * multi_link_support +
       0.30F * observation.respiration_coherence +
       0.25F * observation.respiration_rate_agreement) *
      (1.0F - 0.20F * spatial_evidence.position_uncertain_evidence));
  const float spectral_evidence = clamp01(
      0.22F * observation.respiration_power +
      0.20F * observation.respiration_spectral_snr +
      0.17F * observation.respiration_harmonicity +
      0.17F * observation.respiration_coherence +
      0.14F * observation.respiration_rate_agreement +
      0.10F * average_link_confidence);
  const float body_context = clamp01(
      maximum(0.40F,
              maximum(spatial_evidence.seated_posture_evidence,
                      maximum(spatial_evidence.low_posture_evidence,
                              spatial_evidence.lying_posture_evidence))) +
      0.15F * safety_evidence.respiration_after_impact);
  const float motion_attenuation = clamp01(
      1.0F -
      0.42F * bathroom_evidence.nuisance_reduced_human_motion -
      0.18F * safety_evidence.recovery_motion);
  const float candidate_confidence = clamp01(
      spectral_evidence * (0.40F + 0.60F * observation.quality) *
      (0.55F + 0.45F * motion_attenuation) *
      (0.55F + 0.45F * body_context) *
      (1.0F - 0.55F * periodic_nuisance));

  float candidate_rate =
      candidate_count > 0
          ? clamp01(0.65F * link_rate +
                    0.35F * observation.respiration_rate_normalized)
          : clamp01(observation.respiration_rate_normalized);
  float prior_rate = candidate_rate;
  if (trackedRateBeforeCurrent(replacing_latest, prior_rate)) {
    const float position_transition = clamp01(
        0.40F * spatial_evidence.boundary_position_evidence +
        0.30F * safety_evidence.relocation_evidence +
        0.30F * bathroom_evidence.nuisance_reduced_human_motion);
    const float allowed_step = maximum(
        0.015F,
        (0.025F +
         0.12F * link_rate_agreement * candidate_confidence) *
            (1.0F - 0.45F * position_transition));
    const float rate_delta = candidate_rate - prior_rate;
    if (absolute(rate_delta) > allowed_step) {
      candidate_rate =
          clamp01(prior_rate + (rate_delta < 0.0F ? -allowed_step
                                                   : allowed_step));
    }
  }

  HistoryPoint point{};
  point.probe_sequence = observation.anchor_probe_sequence;
  point.observed_at_us = observed_at_us;
  point.rate_normalized = candidate_rate;
  point.confidence = candidate_confidence;
  point.power = clamp01(observation.respiration_power);
  point.periodic_nuisance = periodic_nuisance;
  point.measurement_loss = measurement_loss;
  point.multi_link_support = multi_link_support;
  point.position_robustness = position_robustness;
  point.quality = clamp01(
      0.35F * observation.quality + 0.20F * bathroom_evidence.analysis_quality +
      0.15F * safety_evidence.analysis_quality +
      0.15F * spatial_evidence.analysis_quality +
      0.15F * (1.0F - measurement_loss));

  BathroomRespirationEvidenceUpdateStatus status =
      BathroomRespirationEvidenceUpdateStatus::Updated;
  if (replacing_latest) {
    const std::size_t latest_index =
        (history_head_ + kHistoryCapacity - 1) % kHistoryCapacity;
    point.observed_at_us = history_[latest_index].observed_at_us;
    history_[latest_index] = point;
    status = BathroomRespirationEvidenceUpdateStatus::ReplacedSameProbe;
  } else {
    history_[history_head_] = point;
    history_head_ = (history_head_ + 1) % kHistoryCapacity;
    history_count_ =
        std::min<std::size_t>(history_count_ + 1, config_.history_points);
  }

  buildEvidence(respiration_evidence);
  emitJsonIfDue(respiration_evidence);
  return status;
}

void BathroomRespirationEvidenceAnalyzer::buildEvidence(
    BathroomRespirationEvidence &evidence) const {
  evidence = {};
  if (history_count_ == 0) {
    return;
  }

  const HistoryPoint &newest = fromNewest(0);
  evidence.probe_sequence = newest.probe_sequence;
  evidence.observed_at_us = newest.observed_at_us;
  evidence.history_points = static_cast<uint16_t>(history_count_);

  float tracked_rate_sum = 0.0F;
  float tracked_weight_sum = 0.0F;
  for (std::size_t offset = 0; offset < history_count_; ++offset) {
    const HistoryPoint &point = fromNewest(offset);
    const float recency =
        1.0F - 0.70F * static_cast<float>(offset) /
                   static_cast<float>(kHistoryCapacity);
    const float weight = point.confidence * recency;
    tracked_rate_sum += point.rate_normalized * weight;
    tracked_weight_sum += weight;
  }
  evidence.tracked_rate_normalized =
      tracked_weight_sum > 0.001F
          ? clamp01(tracked_rate_sum / tracked_weight_sum)
          : newest.rate_normalized;

  float rate_variance_sum = 0.0F;
  for (std::size_t offset = 0; offset < history_count_; ++offset) {
    const HistoryPoint &point = fromNewest(offset);
    const float recency =
        1.0F - 0.70F * static_cast<float>(offset) /
                   static_cast<float>(kHistoryCapacity);
    const float weight = point.confidence * recency;
    const float delta =
        point.rate_normalized - evidence.tracked_rate_normalized;
    rate_variance_sum += weight * delta * delta;
  }
  const float rate_deviation =
      tracked_weight_sum > 0.001F
          ? std::sqrt(rate_variance_sum / tracked_weight_sum)
          : 1.0F;
  evidence.rate_stability = clamp01(1.0F - rate_deviation / 0.18F);

  const std::size_t short_points = std::min<std::size_t>(
      history_count_, config_.short_window_points);
  float short_confidence_sum = 0.0F;
  float short_persistence_sum = 0.0F;
  float long_confidence_sum = 0.0F;
  float long_persistence_sum = 0.0F;
  float quality_sum = 0.0F;
  float nuisance_sum = 0.0F;
  float measurement_loss_sum = 0.0F;
  for (std::size_t offset = 0; offset < history_count_; ++offset) {
    const HistoryPoint &point = fromNewest(offset);
    long_confidence_sum += point.confidence;
    long_persistence_sum +=
        point.confidence >= config_.minimum_presence_confidence ? 1.0F : 0.0F;
    quality_sum += point.quality;
    nuisance_sum += point.periodic_nuisance;
    measurement_loss_sum += point.measurement_loss;
    if (offset < short_points) {
      short_confidence_sum += point.confidence;
      short_persistence_sum +=
          point.confidence >= config_.minimum_presence_confidence ? 1.0F
                                                                  : 0.0F;
    }
  }
  evidence.short_continuity = clamp01(
      0.60F * short_confidence_sum / static_cast<float>(short_points) +
      0.40F * short_persistence_sum / static_cast<float>(short_points));
  evidence.long_continuity = clamp01(
      0.60F * long_confidence_sum / static_cast<float>(history_count_) +
      0.40F * long_persistence_sum / static_cast<float>(history_count_));

  std::size_t missing_points = 0;
  while (missing_points < history_count_ &&
         fromNewest(missing_points).confidence <
             config_.minimum_presence_confidence) {
    ++missing_points;
  }
  float prior_confidence_sum = 0.0F;
  std::size_t prior_count = 0;
  const std::size_t prior_end = std::min<std::size_t>(
      history_count_, missing_points + config_.short_window_points);
  for (std::size_t offset = missing_points; offset < prior_end; ++offset) {
    prior_confidence_sum += fromNewest(offset).confidence;
    ++prior_count;
  }
  const float prior_strength =
      prior_count == 0
          ? 0.0F
          : prior_confidence_sum / static_cast<float>(prior_count);
  const float loss_maturity = clamp01(
      static_cast<float>(missing_points) /
      static_cast<float>(config_.loss_maturity_points));

  evidence.respiration_micro_motion_evidence = clamp01(
      0.55F * newest.confidence + 0.30F * evidence.short_continuity +
      0.15F * evidence.long_continuity);
  evidence.stable_respiration_evidence = clamp01(
      evidence.respiration_micro_motion_evidence * evidence.rate_stability *
      (0.40F + 0.60F * evidence.short_continuity));
  evidence.weak_respiration_evidence = clamp01(
      maximum(newest.confidence, evidence.short_continuity) *
      evidence.rate_stability * (1.0F - 0.65F * newest.power) *
      (0.35F + 0.65F * newest.confidence) *
      (1.0F - newest.measurement_loss));
  evidence.respiration_loss_evidence = clamp01(
      prior_strength * loss_maturity *
      (1.0F - newest.measurement_loss) *
      (1.0F - newest.periodic_nuisance) * newest.quality);
  evidence.measurement_loss_alternative = clamp01(
      0.70F * newest.measurement_loss +
      0.30F * measurement_loss_sum / static_cast<float>(history_count_));
  evidence.periodic_nuisance_alternative = clamp01(
      0.70F * newest.periodic_nuisance +
      0.30F * nuisance_sum / static_cast<float>(history_count_));
  evidence.active_links = 0;
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    if (links_[index].allocated && links_[index].has_packet) {
      const int32_t sequence_offset = static_cast<int32_t>(
          newest.probe_sequence - links_[index].probe_sequence);
      const int64_t age_us = newest.observed_at_us - links_[index].received_at_us;
      if (absolute(static_cast<float>(sequence_offset)) <=
              static_cast<float>(config_.maximum_sequence_skew) &&
          age_us >= 0 && age_us <= config_.link_freshness_us) {
        ++evidence.active_links;
      }
    }
  }
  evidence.multi_link_support = newest.multi_link_support;
  evidence.position_robustness = newest.position_robustness;
  evidence.analysis_quality =
      clamp01(quality_sum / static_cast<float>(history_count_));
  evidence.multi_link_ready = evidence.active_links >= 2;
  evidence.short_history_ready =
      history_count_ >= config_.short_window_points;
  evidence.long_history_ready = history_count_ >= config_.history_points;
  evidence.evidence_ready = history_count_ >= 20;
}

void BathroomRespirationEvidenceAnalyzer::emitJsonIfDue(
    const BathroomRespirationEvidence &evidence) {
  if (last_emit_at_us_ >= 0 &&
      evidence.observed_at_us - last_emit_at_us_ < 1000000) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_respiration_evidence\",\"probe\":%lu,"
      "\"history\":%u,\"active_links\":%u,\"tracked_rate\":%.3f,"
      "\"rate_stability\":%.3f,\"respiration_micro_motion\":%.3f,"
      "\"stable_respiration\":%.3f,\"weak_respiration\":%.3f,"
      "\"respiration_loss\":%.3f,\"measurement_loss_alternative\":%.3f,"
      "\"periodic_nuisance_alternative\":%.3f,"
      "\"short_continuity\":%.3f,\"long_continuity\":%.3f,"
      "\"multi_link_support\":%.3f,\"position_robustness\":%.3f,"
      "\"quality\":%.3f,\"multi_link_ready\":%s,"
      "\"short_ready\":%s,\"long_ready\":%s,\"ready\":%s}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      evidence.history_points, evidence.active_links,
      evidence.tracked_rate_normalized, evidence.rate_stability,
      evidence.respiration_micro_motion_evidence,
      evidence.stable_respiration_evidence,
      evidence.weak_respiration_evidence,
      evidence.respiration_loss_evidence,
      evidence.measurement_loss_alternative,
      evidence.periodic_nuisance_alternative, evidence.short_continuity,
      evidence.long_continuity, evidence.multi_link_support,
      evidence.position_robustness, evidence.analysis_quality,
      evidence.multi_link_ready ? "true" : "false",
      evidence.short_history_ready ? "true" : "false",
      evidence.long_history_ready ? "true" : "false",
      evidence.evidence_ready ? "true" : "false");
}

}  // namespace atom::radar
