#include "BathroomSpatialEvidence.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace atom::radar {

namespace {

constexpr float kMetricScale = 1.0F / 65535.0F;

}  // namespace

const char *bathroomSpatialZoneName(BathroomSpatialZone zone) {
  switch (zone) {
    case BathroomSpatialZone::Bathtub:
      return "bathtub";
    case BathroomSpatialZone::Boundary:
      return "boundary";
    case BathroomSpatialZone::WashFloor:
      return "wash_floor";
    case BathroomSpatialZone::Unknown:
    default:
      return "unknown";
  }
}

BathroomSpatialEvidenceAnalyzer::BathroomSpatialEvidenceAnalyzer(
    BathroomSpatialEvidenceConfig config)
    : config_(config) {
  reset();
}

void BathroomSpatialEvidenceAnalyzer::reset() {
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    links_[index] = {};
  }
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    zone_probability_[index] = 1.0F / 3.0F;
    zone_before_probe_[index] = 1.0F / 3.0F;
  }
  for (std::size_t index = 0; index < kPostureCount; ++index) {
    posture_probability_[index] = 0.0F;
    posture_before_probe_[index] = 0.0F;
  }
  has_evaluated_probe_ = false;
  last_evaluated_probe_ = 0;
  last_emit_at_us_ = -1;
}

float BathroomSpatialEvidenceAnalyzer::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomSpatialEvidenceAnalyzer::maximum(float first, float second) {
  return first > second ? first : second;
}

float BathroomSpatialEvidenceAnalyzer::absolute(float value) {
  return value < 0.0F ? -value : value;
}

BathroomSpatialEvidenceAnalyzer::LinkState *
BathroomSpatialEvidenceAnalyzer::findOrCreateLink(uint8_t receiver_id) {
  LinkState *empty = nullptr;
  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    if (links_[index].valid && links_[index].receiver_id == receiver_id) {
      return &links_[index];
    }
    if (!links_[index].valid && empty == nullptr) {
      empty = &links_[index];
    }
  }
  if (empty != nullptr) {
    empty->valid = true;
    empty->receiver_id = receiver_id;
  }
  return empty;
}

float BathroomSpatialEvidenceAnalyzer::metric(
    const CsiObservationPacket &packet, CsiObservationMetric index) const {
  return static_cast<float>(packet.metrics[static_cast<std::size_t>(index)]) *
         kMetricScale;
}

BathroomSpatialPacketStatus BathroomSpatialEvidenceAnalyzer::ingestPacket(
    const uint8_t *data, std::size_t length, uint32_t expected_system_id,
    int64_t received_at_us) {
  CsiObservationPacket packet{};
  if (!decodeCsiObservationPacket(data, length, packet) ||
      packet.packet_type != kCsiObservationPacketType) {
    return BathroomSpatialPacketStatus::NotCsiObservation;
  }
  if (packet.system_id != expected_system_id) {
    return BathroomSpatialPacketStatus::WrongSystem;
  }

  LinkState *link = findOrCreateLink(packet.receiver_id);
  if (link == nullptr) {
    return BathroomSpatialPacketStatus::LinkCapacityExceeded;
  }
  BathroomSpatialPacketStatus status = BathroomSpatialPacketStatus::Accepted;
  if (link->probe_sequence != 0) {
    const int32_t sequence_delta =
        static_cast<int32_t>(packet.probe_sequence - link->probe_sequence);
    if (sequence_delta < 0) {
      return BathroomSpatialPacketStatus::NonMonotonicProbe;
    }
    if (sequence_delta == 0) {
      status = BathroomSpatialPacketStatus::ReplacedSameProbe;
    }
  }

  link->probe_sequence = packet.probe_sequence;
  link->received_at_us = received_at_us;
  link->snr_db = static_cast<float>(packet.snr_db_x10) * 0.1F;
  link->subcarrier_reliability =
      metric(packet, CsiObservationMetric::SubcarrierReliability);
  link->differential_phase_motion =
      metric(packet, CsiObservationMetric::DifferentialPhaseMotion);
  link->complex_ratio_motion =
      metric(packet, CsiObservationMetric::ComplexRatioMotion);
  link->phase_coherence = metric(packet, CsiObservationMetric::PhaseCoherence);
  link->delay_motion = metric(packet, CsiObservationMetric::DelayDomainMotion);
  link->delay_spread = metric(packet, CsiObservationMetric::DelaySpread);
  link->dynamic_tap_concentration =
      metric(packet, CsiObservationMetric::DynamicTapConcentration);
  link->background_explained =
      metric(packet, CsiObservationMetric::BackgroundExplainedRatio);
  link->innovation_motion =
      metric(packet, CsiObservationMetric::InnovationMotion);
  link->doppler_energy = metric(packet, CsiObservationMetric::DopplerEnergy);
  link->doppler_bandwidth =
      metric(packet, CsiObservationMetric::DopplerBandwidth);
  link->baseline_shift = metric(packet, CsiObservationMetric::BaselineShift);
  link->broadband_nuisance =
      metric(packet, CsiObservationMetric::BroadbandNuisance);
  link->receiver_baseline_maturity =
      metric(packet, CsiObservationMetric::BaselineMaturity);
  link->quality = metric(packet, CsiObservationMetric::Quality);
  return status;
}

float BathroomSpatialEvidenceAnalyzer::instantaneousResponse(
    const LinkState &link) const {
  if (!link.baseline_initialized) {
    return clamp01(0.24F * link.baseline_shift +
                   0.20F * link.delay_motion +
                   0.16F * link.dynamic_tap_concentration +
                   0.12F * link.complex_ratio_motion +
                   0.10F * link.differential_phase_motion +
                   0.10F * link.innovation_motion +
                   0.08F * link.doppler_energy);
  }
  const float snr_delta =
      clamp01(absolute(link.snr_db - link.baseline_snr_db) / 12.0F);
  const float delay_delta =
      clamp01(absolute(link.delay_spread - link.baseline_delay_spread) * 3.0F);
  const float tap_delta = clamp01(
      absolute(link.dynamic_tap_concentration -
               link.baseline_tap_concentration) *
      2.0F);
  return clamp01(0.30F * snr_delta + 0.16F * delay_delta +
                 0.10F * tap_delta + 0.12F * link.baseline_shift +
                 0.10F * link.delay_motion +
                 0.08F * link.complex_ratio_motion +
                 0.06F * link.differential_phase_motion +
                 0.05F * link.doppler_energy +
                 0.03F * link.innovation_motion);
}

int BathroomSpatialEvidenceAnalyzer::roleIndex(uint8_t receiver_id) const {
  if (receiver_id == config_.bathtub_receiver_id) {
    return 0;
  }
  if (receiver_id == config_.boundary_receiver_id) {
    return 1;
  }
  if (receiver_id == config_.wash_floor_receiver_id) {
    return 2;
  }
  return -1;
}

BathroomSpatialEvidenceUpdateStatus BathroomSpatialEvidenceAnalyzer::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSafetyEvidence &safety_evidence,
    BathroomSpatialEvidence &spatial_evidence) {
  if (observed_at_us < 0 ||
      observation.anchor_probe_sequence != bathroom_evidence.probe_sequence ||
      observation.anchor_probe_sequence != safety_evidence.probe_sequence ||
      !std::isfinite(observation.quality) ||
      !std::isfinite(bathroom_evidence.analysis_quality) ||
      !std::isfinite(safety_evidence.analysis_quality)) {
    return BathroomSpatialEvidenceUpdateStatus::InvalidObservation;
  }

  bool same_probe = false;
  if (has_evaluated_probe_) {
    const int32_t sequence_delta = static_cast<int32_t>(
        observation.anchor_probe_sequence - last_evaluated_probe_);
    if (sequence_delta < 0) {
      return BathroomSpatialEvidenceUpdateStatus::NonMonotonicProbe;
    }
    same_probe = sequence_delta == 0;
  }
  if (!same_probe) {
    for (std::size_t index = 0; index < kZoneCount; ++index) {
      zone_before_probe_[index] = zone_probability_[index];
    }
    for (std::size_t index = 0; index < kPostureCount; ++index) {
      posture_before_probe_[index] = posture_probability_[index];
    }
    has_evaluated_probe_ = true;
    last_evaluated_probe_ = observation.anchor_probe_sequence;
  }

  const bool empty_reference =
      bathroom_evidence.human_motion_evidence < 0.04F &&
      observation.respiration_power < 0.04F &&
      observation.baseline_shift < 0.05F &&
      observation.innovation_motion < 0.05F &&
      safety_evidence.impact_evidence < 0.05F;
  float role_score[kZoneCount]{0.0F, 0.0F, 0.0F};
  float role_quality[kZoneCount]{0.0F, 0.0F, 0.0F};
  bool role_present[kZoneCount]{false, false, false};
  float quality_sum = 0.0F;
  float maturity_sum = 0.0F;
  float delay_delta_sum = 0.0F;
  float tap_delta_sum = 0.0F;
  uint8_t active_links = 0;

  for (std::size_t index = 0; index < kMaximumLinks; ++index) {
    LinkState &link = links_[index];
    if (!link.valid) {
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

    if (!link.baseline_initialized) {
      link.baseline_snr_db = link.snr_db;
      link.baseline_delay_spread = link.delay_spread;
      link.baseline_tap_concentration = link.dynamic_tap_concentration;
      link.baseline_samples = 1;
      link.baseline_initialized = true;
    }
    const float response = instantaneousResponse(link);
    if (link.last_scored_probe_sequence != link.probe_sequence) {
      link.response_ema =
          link.baseline_samples <= 1
              ? response
              : 0.78F * link.response_ema + 0.22F * response;
      if (empty_reference) {
        const float alpha = config_.baseline_adaptation_alpha;
        link.baseline_snr_db += alpha * (link.snr_db - link.baseline_snr_db);
        link.baseline_delay_spread +=
            alpha * (link.delay_spread - link.baseline_delay_spread);
        link.baseline_tap_concentration +=
            alpha * (link.dynamic_tap_concentration -
                     link.baseline_tap_concentration);
        if (link.baseline_samples < UINT16_MAX) {
          ++link.baseline_samples;
        }
      }
      link.last_scored_probe_sequence = link.probe_sequence;
    }

    const float link_quality =
        clamp01(0.45F * link.quality + 0.30F * link.subcarrier_reliability +
                0.25F * link.phase_coherence);
    const float weighted_response =
        clamp01((0.55F * response + 0.45F * link.response_ema) *
                (0.35F + 0.65F * link_quality));
    const int role = roleIndex(link.receiver_id);
    if (role >= 0) {
      role_score[role] = maximum(role_score[role], weighted_response);
      role_quality[role] = maximum(role_quality[role], link_quality);
      role_present[role] = true;
    }
    const float local_maturity = clamp01(maximum(
        link.receiver_baseline_maturity,
        static_cast<float>(link.baseline_samples) /
            static_cast<float>(config_.baseline_maturity_samples)));
    quality_sum += link_quality;
    maturity_sum += local_maturity;
    delay_delta_sum += clamp01(
        absolute(link.delay_spread - link.baseline_delay_spread) * 3.0F);
    tap_delta_sum += clamp01(
        absolute(link.dynamic_tap_concentration -
                 link.baseline_tap_concentration) *
        2.0F);
    ++active_links;
  }

  uint8_t present_roles = 0;
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    if (role_present[index]) {
      ++present_roles;
    }
  }
  const float role_coverage =
      static_cast<float>(present_roles) / static_cast<float>(kZoneCount);
  const float average_link_quality =
      active_links == 0 ? 0.0F : quality_sum / static_cast<float>(active_links);
  const float baseline_maturity =
      active_links == 0 ? 0.0F : maturity_sum / static_cast<float>(active_links);

  float observation_likelihood[kZoneCount]{};
  float likelihood_sum = 0.0F;
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    observation_likelihood[index] =
        0.10F + (role_present[index] ? role_score[index] : 0.0F);
    likelihood_sum += observation_likelihood[index];
  }
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    observation_likelihood[index] /= likelihood_sum;
  }
  float highest_likelihood = 0.0F;
  float second_likelihood = 0.0F;
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    const float value = observation_likelihood[index];
    if (value >= highest_likelihood) {
      second_likelihood = highest_likelihood;
      highest_likelihood = value;
    } else if (value > second_likelihood) {
      second_likelihood = value;
    }
  }
  const float spatial_separation =
      clamp01((highest_likelihood - second_likelihood) * 2.0F);
  const float link_consistency =
      clamp01(0.45F * observation.link_agreement +
              0.30F * observation.synchronization_quality +
              0.25F * average_link_quality);
  const float analysis_quality =
      clamp01(0.30F * observation.quality +
              0.22F * bathroom_evidence.analysis_quality +
              0.18F * safety_evidence.analysis_quality +
              0.20F * link_consistency + 0.10F * role_coverage);

  const float impact_transition = clamp01(safety_evidence.impact_evidence);
  const float stay_probability = 0.84F - 0.12F * impact_transition;
  const float adjacent_probability = 0.14F - 0.04F * impact_transition;
  const float direct_probability = 0.02F + 0.16F * impact_transition;
  float predicted[kZoneCount]{};
  predicted[0] = zone_before_probe_[0] * stay_probability +
                 zone_before_probe_[1] * 0.15F +
                 zone_before_probe_[2] * direct_probability;
  predicted[1] = zone_before_probe_[0] * adjacent_probability +
                 zone_before_probe_[1] * 0.70F +
                 zone_before_probe_[2] * adjacent_probability;
  predicted[2] = zone_before_probe_[0] * direct_probability +
                 zone_before_probe_[1] * 0.15F +
                 zone_before_probe_[2] * stay_probability;
  const float observation_weight = clamp01(
      (0.15F + 0.50F * spatial_separation + 0.35F * role_coverage) *
      analysis_quality);
  float posterior_sum = 0.0F;
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    const float neutralized_likelihood =
        (1.0F - observation_weight) / 3.0F +
        observation_weight * observation_likelihood[index];
    zone_probability_[index] =
        predicted[index] * (0.20F + 2.40F * neutralized_likelihood);
    posterior_sum += zone_probability_[index];
  }
  for (std::size_t index = 0; index < kZoneCount; ++index) {
    zone_probability_[index] /= posterior_sum;
  }

  const float average_delay_delta =
      active_links == 0 ? 0.0F : delay_delta_sum / static_cast<float>(active_links);
  const float average_tap_delta =
      active_links == 0 ? 0.0F : tap_delta_sum / static_cast<float>(active_links);
  const float lower_link_dominance = clamp01(
      maximum(role_score[0], role_score[2]) - 0.45F * role_score[1]);
  const float low_posture_raw =
      clamp01(0.20F * safety_evidence.relocation_evidence +
              0.18F * average_delay_delta + 0.14F * average_tap_delta +
              0.16F * observation.baseline_shift +
              0.18F * safety_evidence.post_impact_immobility +
              0.14F * lower_link_dominance);
  const float human_motion =
      clamp01(bathroom_evidence.nuisance_reduced_human_motion);
  const float stillness = clamp01(observation.stillness_score);
  const float standing_raw =
      clamp01((1.0F - low_posture_raw) *
              (0.45F + 0.30F * human_motion + 0.25F * (1.0F - stillness)));
  const float seated_raw = clamp01(
      low_posture_raw * (0.35F + 0.65F * stillness) *
      (0.45F + 0.40F * zone_probability_[0] +
       0.15F * safety_evidence.respiration_after_impact) *
      (1.0F - 0.35F * safety_evidence.fall_sequence_evidence));
  const float lying_raw = clamp01(
      low_posture_raw * (0.45F + 0.55F * stillness) *
      (0.40F * zone_probability_[2] + 0.20F * zone_probability_[0] +
       0.25F * safety_evidence.relocation_evidence +
       0.15F * safety_evidence.fall_sequence_evidence));
  const float floor_danger =
      zone_probability_[2] * lying_raw *
      (0.25F + 0.75F * safety_evidence.fall_sequence_evidence);
  const float bathtub_danger =
      zone_probability_[0] * low_posture_raw *
      safety_evidence.post_impact_immobility *
      (0.25F + 0.75F * safety_evidence.dangerous_immobility_evidence);
  const float dangerous_raw =
      clamp01(maximum(floor_danger, bathtub_danger) *
              (1.0F - 0.35F * safety_evidence.recovery_motion));
  const float posture_raw[kPostureCount]{standing_raw, seated_raw,
                                         low_posture_raw, lying_raw,
                                         dangerous_raw};
  for (std::size_t index = 0; index < kPostureCount; ++index) {
    posture_probability_[index] =
        clamp01(0.78F * posture_before_probe_[index] +
                0.22F * posture_raw[index]);
  }

  spatial_evidence = {};
  spatial_evidence.probe_sequence = observation.anchor_probe_sequence;
  spatial_evidence.observed_at_us = observed_at_us;
  spatial_evidence.active_links = active_links;
  spatial_evidence.bathtub_position_evidence = zone_probability_[0];
  spatial_evidence.boundary_position_evidence = zone_probability_[1];
  spatial_evidence.wash_floor_position_evidence = zone_probability_[2];
  spatial_evidence.position_uncertain_evidence = clamp01(
      0.45F * (1.0F - spatial_separation) +
      0.25F * (1.0F - role_coverage) +
      0.20F * (1.0F - baseline_maturity) +
      0.10F * (1.0F - analysis_quality));
  spatial_evidence.standing_posture_evidence = posture_probability_[0];
  spatial_evidence.seated_posture_evidence = posture_probability_[1];
  spatial_evidence.low_posture_evidence = posture_probability_[2];
  spatial_evidence.lying_posture_evidence = posture_probability_[3];
  spatial_evidence.dangerous_posture_evidence = posture_probability_[4];
  spatial_evidence.link_consistency = link_consistency;
  spatial_evidence.spatial_separation = spatial_separation;
  spatial_evidence.role_coverage = role_coverage;
  spatial_evidence.baseline_maturity = baseline_maturity;
  spatial_evidence.analysis_quality = analysis_quality;
  spatial_evidence.spatial_links_ready =
      active_links >= config_.minimum_spatial_links;
  spatial_evidence.baseline_ready = baseline_maturity >= 0.75F;
  spatial_evidence.evidence_ready =
      spatial_evidence.spatial_links_ready && role_coverage >= 2.0F / 3.0F;

  std::size_t dominant_index = 0;
  float dominant_value = zone_probability_[0];
  float runner_up = 0.0F;
  for (std::size_t index = 1; index < kZoneCount; ++index) {
    if (zone_probability_[index] > dominant_value) {
      runner_up = dominant_value;
      dominant_value = zone_probability_[index];
      dominant_index = index;
    } else {
      runner_up = maximum(runner_up, zone_probability_[index]);
    }
  }
  if (dominant_value - runner_up < 0.05F) {
    spatial_evidence.dominant_zone = BathroomSpatialZone::Unknown;
  } else if (dominant_index == 0) {
    spatial_evidence.dominant_zone = BathroomSpatialZone::Bathtub;
  } else if (dominant_index == 1) {
    spatial_evidence.dominant_zone = BathroomSpatialZone::Boundary;
  } else {
    spatial_evidence.dominant_zone = BathroomSpatialZone::WashFloor;
  }

  emitJsonIfDue(spatial_evidence);
  return same_probe ? BathroomSpatialEvidenceUpdateStatus::ReplacedSameProbe
                    : BathroomSpatialEvidenceUpdateStatus::Updated;
}

void BathroomSpatialEvidenceAnalyzer::emitJsonIfDue(
    const BathroomSpatialEvidence &evidence) {
  if (last_emit_at_us_ >= 0 &&
      evidence.observed_at_us - last_emit_at_us_ < 1000000) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_spatial_evidence\",\"probe\":%lu,"
      "\"active_links\":%u,\"dominant_zone\":\"%s\","
      "\"bathtub\":%.3f,\"boundary\":%.3f,\"wash_floor\":%.3f,"
      "\"position_uncertain\":%.3f,\"standing\":%.3f,"
      "\"seated\":%.3f,\"low_posture\":%.3f,\"lying\":%.3f,"
      "\"dangerous_posture\":%.3f,\"link_consistency\":%.3f,"
      "\"spatial_separation\":%.3f,\"role_coverage\":%.3f,"
      "\"baseline_maturity\":%.3f,\"quality\":%.3f,"
      "\"links_ready\":%s,\"baseline_ready\":%s,\"ready\":%s}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      evidence.active_links, bathroomSpatialZoneName(evidence.dominant_zone),
      evidence.bathtub_position_evidence,
      evidence.boundary_position_evidence,
      evidence.wash_floor_position_evidence,
      evidence.position_uncertain_evidence,
      evidence.standing_posture_evidence,
      evidence.seated_posture_evidence, evidence.low_posture_evidence,
      evidence.lying_posture_evidence,
      evidence.dangerous_posture_evidence, evidence.link_consistency,
      evidence.spatial_separation, evidence.role_coverage,
      evidence.baseline_maturity, evidence.analysis_quality,
      evidence.spatial_links_ready ? "true" : "false",
      evidence.baseline_ready ? "true" : "false",
      evidence.evidence_ready ? "true" : "false");
}

}  // namespace atom::radar
