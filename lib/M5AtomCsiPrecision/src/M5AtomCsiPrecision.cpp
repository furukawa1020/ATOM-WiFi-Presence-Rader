#include "M5AtomCsiPrecision.hpp"

#include <cmath>
#include <cstring>
#include <limits>

#include <protocol.hpp>

namespace atom::radar {

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;
constexpr uint8_t kBaselineReadyFlag = 0x01U;

uint16_t unitToWire(float value) {
  if (value <= 0.0F) {
    return 0U;
  }
  if (value >= 1.0F) {
    return 65535U;
  }
  return static_cast<uint16_t>(std::lround(value * 65535.0F));
}

float wireToUnit(uint16_t value) { return static_cast<float>(value) / 65535.0F; }

std::size_t metricIndex(CsiObservationMetric metric) {
  return static_cast<std::size_t>(metric);
}

uint32_t observationPacketCrc(const CsiObservationPacket &packet) {
  return protocol::crc32(reinterpret_cast<const uint8_t *>(&packet),
                         offsetof(CsiObservationPacket, crc32));
}

float packetMetric(const CsiObservationPacket &packet, CsiObservationMetric metric) {
  return wireToUnit(packet.metrics[metricIndex(metric)]);
}

CsiLinkObservation observationFromPacket(const CsiObservationPacket &packet) {
  CsiLinkObservation observation{};
  observation.receiver_id = packet.receiver_id;
  observation.channel = packet.channel;
  observation.bandwidth_mhz = packet.bandwidth_mhz;
  observation.probe_sequence = packet.probe_sequence;
  observation.observation_sequence = packet.observation_sequence;
  observation.tx_uptime_us = packet.tx_uptime_us;
  observation.frames_in_summary = packet.frames_in_summary;
  observation.sequence_gap_count = packet.sequence_gap_count;
  observation.snr_db = static_cast<float>(packet.snr_db_x10) * 0.1F;
  observation.valid_ratio = packetMetric(packet, CsiObservationMetric::ValidRatio);
  observation.baseline_maturity = packetMetric(packet, CsiObservationMetric::BaselineMaturity);
  observation.amplitude_motion = packetMetric(packet, CsiObservationMetric::AmplitudeMotion);
  observation.differential_phase_motion =
      packetMetric(packet, CsiObservationMetric::DifferentialPhaseMotion);
  observation.complex_ratio_motion =
      packetMetric(packet, CsiObservationMetric::ComplexRatioMotion);
  observation.phase_coherence = packetMetric(packet, CsiObservationMetric::PhaseCoherence);
  observation.doppler_energy = packetMetric(packet, CsiObservationMetric::DopplerEnergy);
  observation.respiration_power = packetMetric(packet, CsiObservationMetric::RespirationPower);
  observation.respiration_coherence =
      packetMetric(packet, CsiObservationMetric::RespirationCoherence);
  observation.impulse_score = packetMetric(packet, CsiObservationMetric::ImpulseScore);
  observation.stillness_score = packetMetric(packet, CsiObservationMetric::StillnessScore);
  observation.baseline_shift = packetMetric(packet, CsiObservationMetric::BaselineShift);
  observation.broadband_nuisance =
      packetMetric(packet, CsiObservationMetric::BroadbandNuisance);
  observation.quality = packetMetric(packet, CsiObservationMetric::Quality);
  observation.baseline_ready = (packet.flags & kBaselineReadyFlag) != 0U;
  return observation;
}

}  // namespace

CsiObservationPacket makeCsiObservationPacket(uint32_t system_id,
                                              const CsiLinkObservation &observation) {
  CsiObservationPacket packet{};
  packet.magic = protocol::kPacketMagic;
  packet.protocol_version = protocol::kProtocolVersion;
  packet.payload_length = sizeof(packet);
  packet.system_id = system_id;
  packet.packet_type = kCsiObservationPacketType;
  packet.receiver_id = observation.receiver_id;
  packet.channel = observation.channel;
  packet.bandwidth_mhz = observation.bandwidth_mhz;
  packet.probe_sequence = observation.probe_sequence;
  packet.tx_uptime_us = observation.tx_uptime_us;
  packet.observation_sequence = observation.observation_sequence;
  packet.frames_in_summary = observation.frames_in_summary;
  packet.sequence_gap_count = observation.sequence_gap_count;
  const float snr_x10 = observation.snr_db * 10.0F;
  packet.snr_db_x10 = static_cast<int16_t>(
      snr_x10 < -32768.0F ? -32768.0F : (snr_x10 > 32767.0F ? 32767.0F : snr_x10));
  packet.flags = observation.baseline_ready ? kBaselineReadyFlag : 0U;
  packet.metrics[metricIndex(CsiObservationMetric::ValidRatio)] =
      unitToWire(observation.valid_ratio);
  packet.metrics[metricIndex(CsiObservationMetric::BaselineMaturity)] =
      unitToWire(observation.baseline_maturity);
  packet.metrics[metricIndex(CsiObservationMetric::AmplitudeMotion)] =
      unitToWire(observation.amplitude_motion);
  packet.metrics[metricIndex(CsiObservationMetric::DifferentialPhaseMotion)] =
      unitToWire(observation.differential_phase_motion);
  packet.metrics[metricIndex(CsiObservationMetric::ComplexRatioMotion)] =
      unitToWire(observation.complex_ratio_motion);
  packet.metrics[metricIndex(CsiObservationMetric::PhaseCoherence)] =
      unitToWire(observation.phase_coherence);
  packet.metrics[metricIndex(CsiObservationMetric::DopplerEnergy)] =
      unitToWire(observation.doppler_energy);
  packet.metrics[metricIndex(CsiObservationMetric::RespirationPower)] =
      unitToWire(observation.respiration_power);
  packet.metrics[metricIndex(CsiObservationMetric::RespirationCoherence)] =
      unitToWire(observation.respiration_coherence);
  packet.metrics[metricIndex(CsiObservationMetric::ImpulseScore)] =
      unitToWire(observation.impulse_score);
  packet.metrics[metricIndex(CsiObservationMetric::StillnessScore)] =
      unitToWire(observation.stillness_score);
  packet.metrics[metricIndex(CsiObservationMetric::BaselineShift)] =
      unitToWire(observation.baseline_shift);
  packet.metrics[metricIndex(CsiObservationMetric::BroadbandNuisance)] =
      unitToWire(observation.broadband_nuisance);
  packet.metrics[metricIndex(CsiObservationMetric::Quality)] = unitToWire(observation.quality);
  packet.crc32 = observationPacketCrc(packet);
  return packet;
}

bool decodeCsiObservationPacket(const uint8_t *data, std::size_t length,
                                CsiObservationPacket &packet) {
  if (data == nullptr || length != sizeof(packet)) {
    return false;
  }
  std::memcpy(&packet, data, sizeof(packet));
  return packet.magic == protocol::kPacketMagic &&
         packet.protocol_version == protocol::kProtocolVersion &&
         packet.payload_length == sizeof(packet) &&
         packet.packet_type == kCsiObservationPacketType && packet.receiver_id >= 1U &&
         packet.receiver_id <= kMaximumM5AtomCsiReceivers &&
         packet.crc32 == observationPacketCrc(packet);
}

M5AtomCsiLinkProcessor::M5AtomCsiLinkProcessor(M5AtomCsiPrecisionConfig config)
    : config_(config) {
  reset();
}

void M5AtomCsiLinkProcessor::reset() {
  std::memset(probe_history_, 0, sizeof(probe_history_));
  probe_head_ = 0;
  probe_count_ = 0;
  last_matched_probe_ = 0;
  has_last_matched_probe_ = false;
  observation_sequence_ = 0;
  resetSignalState(ParsedCsiFrame{});
}

void M5AtomCsiLinkProcessor::resetSignalState(const ParsedCsiFrame &frame) {
  std::memset(carriers_, 0, sizeof(carriers_));
  std::memset(fast_history_, 0, sizeof(fast_history_));
  std::memset(slow_amplitude_history_, 0, sizeof(slow_amplitude_history_));
  std::memset(slow_phase_history_, 0, sizeof(slow_phase_history_));
  carrier_count_ = frame.sample_count;
  channel_ = frame.channel;
  bandwidth_mhz_ = frame.bandwidth_mhz;
  layout_ready_ = carrier_count_ > 0U;
  for (std::size_t index = 0; index < carrier_count_; ++index) {
    CarrierState &state = carriers_[index];
    state.subcarrier = frame.samples[index].subcarrier;
    state.phase_noise_variance =
        config_.phase_noise_floor_rad * config_.phase_noise_floor_rad;
    state.ratio_noise_variance = config_.ratio_noise_floor * config_.ratio_noise_floor;
    state.principal_weight =
        std::sin((static_cast<float>(index) + 1.0F) * 1.61803398875F);
  }
  float norm = 0.0F;
  for (std::size_t index = 0; index < carrier_count_; ++index) {
    norm += carriers_[index].principal_weight * carriers_[index].principal_weight;
  }
  norm = std::sqrt(norm > 1.0e-6F ? norm : 1.0F);
  for (std::size_t index = 0; index < carrier_count_; ++index) {
    carriers_[index].principal_weight /= norm;
  }

  baseline_frame_count_ = 0;
  fast_head_ = 0;
  fast_count_ = 0;
  slow_head_ = 0;
  slow_count_ = 0;
  summary_frames_ = 0;
  summary_sequence_gaps_ = 0;
  summary_amplitude_motion_ = 0.0F;
  summary_phase_motion_ = 0.0F;
  summary_ratio_motion_ = 0.0F;
  summary_phase_coherence_ = 0.0F;
  summary_impulse_ = 0.0F;
  summary_baseline_shift_ = 0.0F;
  summary_quality_ = 0.0F;
  summary_valid_ratio_ = 0.0F;
  summary_snr_db_ = 0.0F;
  summary_amplitude_projection_ = 0.0F;
  summary_phase_projection_ = 0.0F;
  previous_projection_ = 0.0F;
  motion_ewma_ = 0.0F;
  impulse_peak_ = 0.0F;
  baseline_shift_ewma_ = 0.0F;
  broadband_ewma_ = 0.0F;
  has_previous_projection_ = false;
}

void M5AtomCsiLinkProcessor::noteProbe(uint32_t sequence, uint64_t tx_uptime_us,
                                       int64_t received_at_us) {
  probe_history_[probe_head_] = {sequence, tx_uptime_us, received_at_us};
  probe_head_ = (probe_head_ + 1U) % kProbeHistoryCapacity;
  if (probe_count_ < kProbeHistoryCapacity) {
    ++probe_count_;
  }
}

M5AtomCsiLinkProcessor::ProbeMatchStatus M5AtomCsiLinkProcessor::matchProbe(
    int64_t received_at_us, MatchedProbe &matched) {
  if (probe_count_ == 0U) {
    return ProbeMatchStatus::Missing;
  }
  int64_t best_delta = std::numeric_limits<int64_t>::max();
  const ProbeStamp *best = nullptr;
  for (std::size_t offset = 0; offset < probe_count_; ++offset) {
    const std::size_t index =
        (probe_head_ + kProbeHistoryCapacity - 1U - offset) % kProbeHistoryCapacity;
    const ProbeStamp &candidate = probe_history_[index];
    const int64_t signed_delta = received_at_us - candidate.received_at_us;
    const int64_t delta = signed_delta < 0 ? -signed_delta : signed_delta;
    if (delta < best_delta) {
      best_delta = delta;
      best = &candidate;
    }
  }
  if (best == nullptr || best_delta > config_.probe_match_tolerance_us) {
    return ProbeMatchStatus::Missing;
  }
  if (has_last_matched_probe_ &&
      (best->sequence == last_matched_probe_ ||
       !protocol::isSequenceNewer(best->sequence, last_matched_probe_))) {
    return ProbeMatchStatus::NonMonotonic;
  }

  uint16_t gap = 0;
  if (has_last_matched_probe_) {
    const uint32_t distance = best->sequence - last_matched_probe_;
    const uint32_t missing = distance > 1U ? distance - 1U : 0U;
    gap = static_cast<uint16_t>(missing > 65535U ? 65535U : missing);
  }
  last_matched_probe_ = best->sequence;
  has_last_matched_probe_ = true;
  matched = {best->sequence, best->tx_uptime_us, gap};
  return ProbeMatchStatus::Matched;
}

CsiPrecisionUpdateStatus M5AtomCsiLinkProcessor::update(const ParsedCsiFrame &frame,
                                                        uint8_t receiver_id,
                                                        CsiLinkObservation &observation) {
  observation = {};
  if (receiver_id == 0U || receiver_id > kMaximumM5AtomCsiReceivers || frame.sample_count < 8U ||
      frame.sample_count > kMaximumHtSubcarrierCount) {
    return CsiPrecisionUpdateStatus::InvalidFrame;
  }

  MatchedProbe probe{};
  const ProbeMatchStatus match_status = matchProbe(frame.received_at_us, probe);
  if (match_status == ProbeMatchStatus::Missing) {
    return CsiPrecisionUpdateStatus::UnmatchedProbe;
  }
  if (match_status == ProbeMatchStatus::NonMonotonic) {
    return CsiPrecisionUpdateStatus::NonMonotonicProbe;
  }

  bool layout_changed = !layout_ready_ || carrier_count_ != frame.sample_count ||
                        channel_ != frame.channel || bandwidth_mhz_ != frame.bandwidth_mhz;
  if (!layout_changed) {
    for (std::size_t index = 0; index < frame.sample_count; ++index) {
      if (carriers_[index].subcarrier != frame.samples[index].subcarrier) {
        layout_changed = true;
        break;
      }
    }
  }
  if (layout_changed) {
    resetSignalState(frame);
  }

  float log_amplitude[kMaximumHtSubcarrierCount]{};
  float centered_amplitude[kMaximumHtSubcarrierCount]{};
  float wrapped_phase[kMaximumHtSubcarrierCount]{};
  float unwrapped_phase[kMaximumHtSubcarrierCount]{};
  float phase_residual[kMaximumHtSubcarrierCount]{};
  float amplitude_component[kMaximumHtSubcarrierCount]{};
  float phase_component[kMaximumHtSubcarrierCount]{};
  float phase_change[kMaximumHtSubcarrierCount]{};
  float ratio_phase_change[kMaximumHtSubcarrierCount]{};
  bool ratio_change_valid[kMaximumHtSubcarrierCount]{};
  bool valid[kMaximumHtSubcarrierCount]{};
  float scratch[kMaximumHtSubcarrierCount]{};

  std::size_t valid_count = 0;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    const CsiComplexSample &sample = frame.samples[index];
    if (!sample.valid_for_features) {
      continue;
    }
    const float real = static_cast<float>(sample.real);
    const float imaginary = static_cast<float>(sample.imaginary);
    const float power = real * real + imaginary * imaginary;
    if (power < 1.0F) {
      continue;
    }
    valid[index] = true;
    log_amplitude[index] = 0.5F * std::log(power);
    wrapped_phase[index] = std::atan2(imaginary, real);
    scratch[valid_count++] = log_amplitude[index];
  }
  if (valid_count < 8U) {
    return CsiPrecisionUpdateStatus::InvalidFrame;
  }
  const float frame_median = median(scratch, valid_count);

  for (int sign = -1; sign <= 1; sign += 2) {
    bool has_previous = false;
    float previous_wrapped = 0.0F;
    float previous_unwrapped = 0.0F;
    float sum_x = 0.0F;
    float sum_y = 0.0F;
    float sum_xx = 0.0F;
    float sum_xy = 0.0F;
    std::size_t count = 0;
    for (std::size_t index = 0; index < frame.sample_count; ++index) {
      const int carrier_sign = frame.samples[index].subcarrier < 0 ? -1 : 1;
      if (!valid[index] || carrier_sign != sign) {
        continue;
      }
      unwrapped_phase[index] =
          has_previous
              ? previous_unwrapped + wrapPhase(wrapped_phase[index] - previous_wrapped)
              : wrapped_phase[index];
      has_previous = true;
      previous_wrapped = wrapped_phase[index];
      previous_unwrapped = unwrapped_phase[index];
      const float x = static_cast<float>(frame.samples[index].subcarrier);
      const float y = unwrapped_phase[index];
      sum_x += x;
      sum_y += y;
      sum_xx += x * x;
      sum_xy += x * y;
      ++count;
    }
    const float denominator = static_cast<float>(count) * sum_xx - sum_x * sum_x;
    const float slope = count >= 2U && std::fabs(denominator) > 1.0e-6F
                            ? (static_cast<float>(count) * sum_xy - sum_x * sum_y) / denominator
                            : 0.0F;
    const float intercept =
        count > 0U ? (sum_y - slope * sum_x) / static_cast<float>(count) : 0.0F;
    for (std::size_t index = 0; index < frame.sample_count; ++index) {
      const int carrier_sign = frame.samples[index].subcarrier < 0 ? -1 : 1;
      if (valid[index] && carrier_sign == sign) {
        phase_residual[index] =
            wrapPhase(unwrapped_phase[index] -
                      intercept - slope * static_cast<float>(frame.samples[index].subcarrier));
      }
    }
  }

  float amplitude_motion_sum = 0.0F;
  float phase_motion_sum = 0.0F;
  float ratio_motion_sum = 0.0F;
  float baseline_shift_sum = 0.0F;
  float phase_vector_real = 0.0F;
  float phase_vector_imaginary = 0.0F;
  std::size_t temporal_count = 0;
  std::size_t ratio_count = 0;
  int previous_valid_index = -1;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    if (!valid[index]) {
      continue;
    }
    CarrierState &state = carriers_[index];
    centered_amplitude[index] = log_amplitude[index] - frame_median;
    const float variance = state.baseline_count > 1U
                               ? state.amplitude_m2 / static_cast<float>(state.baseline_count - 1U)
                               : config_.amplitude_std_floor * config_.amplitude_std_floor;
    const float amplitude_std =
        std::sqrt(variance > config_.amplitude_std_floor * config_.amplitude_std_floor
                      ? variance
                      : config_.amplitude_std_floor * config_.amplitude_std_floor);
    const float baseline_z = state.baseline_count > 8U
                                 ? (centered_amplitude[index] - state.amplitude_mean) / amplitude_std
                                 : 0.0F;
    baseline_shift_sum += clamp(std::fabs(baseline_z) / 6.0F, 0.0F, 1.0F);

    if (state.previous_valid) {
      const float amplitude_delta =
          (centered_amplitude[index] - state.previous_amplitude) / amplitude_std;
      phase_change[index] = wrapPhase(phase_residual[index] - state.previous_phase);
      const float phase_std =
          std::sqrt(state.phase_noise_variance >
                            config_.phase_noise_floor_rad * config_.phase_noise_floor_rad
                        ? state.phase_noise_variance
                        : config_.phase_noise_floor_rad * config_.phase_noise_floor_rad);
      amplitude_component[index] = clamp(amplitude_delta, -6.0F, 6.0F);
      phase_component[index] = clamp(phase_change[index] / phase_std, -6.0F, 6.0F);
      amplitude_motion_sum += clamp(std::fabs(amplitude_delta) / 4.0F, 0.0F, 1.0F);
      phase_motion_sum +=
          clamp(std::fabs(phase_change[index]) / (4.0F * phase_std), 0.0F, 1.0F);
      phase_vector_real += std::cos(phase_change[index]);
      phase_vector_imaginary += std::sin(phase_change[index]);
      ++temporal_count;
    }

    if (previous_valid_index >= 0) {
      const std::size_t previous_index = static_cast<std::size_t>(previous_valid_index);
      int carrier_distance = frame.samples[index].subcarrier - frame.samples[previous_index].subcarrier;
      if (carrier_distance < 0) {
        carrier_distance = -carrier_distance;
      }
      const bool same_side = (frame.samples[index].subcarrier < 0) ==
                             (frame.samples[previous_index].subcarrier < 0);
      if (same_side && carrier_distance <= 3) {
        const float ratio_phase = wrapPhase(phase_residual[index] - phase_residual[previous_index]);
        const float ratio_amplitude = centered_amplitude[index] - centered_amplitude[previous_index];
        if (state.previous_ratio_valid) {
          ratio_phase_change[index] = wrapPhase(ratio_phase - state.previous_ratio_phase);
          ratio_change_valid[index] = true;
          const float amplitude_change = ratio_amplitude - state.previous_ratio_amplitude;
          const float ratio_std =
              std::sqrt(state.ratio_noise_variance >
                                config_.ratio_noise_floor * config_.ratio_noise_floor
                            ? state.ratio_noise_variance
                            : config_.ratio_noise_floor * config_.ratio_noise_floor);
          const float normalized =
              std::sqrt((ratio_phase_change[index] / ratio_std) *
                            (ratio_phase_change[index] / ratio_std) +
                        (amplitude_change / config_.amplitude_std_floor) *
                            (amplitude_change / config_.amplitude_std_floor));
          ratio_motion_sum += clamp(normalized / 6.0F, 0.0F, 1.0F);
          phase_component[index] =
              0.65F * phase_component[index] +
              0.35F * clamp(ratio_phase_change[index] / ratio_std, -6.0F, 6.0F);
          ++ratio_count;
        }
        state.previous_ratio_phase = ratio_phase;
        state.previous_ratio_amplitude = ratio_amplitude;
        state.previous_ratio_valid = true;
      } else {
        state.previous_ratio_valid = false;
      }
    }
    previous_valid_index = static_cast<int>(index);
  }

  const float amplitude_motion = temporal_count > 0U
                                     ? amplitude_motion_sum / static_cast<float>(temporal_count)
                                     : 0.0F;
  const float phase_motion = temporal_count > 0U
                                 ? phase_motion_sum / static_cast<float>(temporal_count)
                                 : 0.0F;
  const float ratio_motion =
      ratio_count > 0U ? ratio_motion_sum / static_cast<float>(ratio_count) : 0.0F;
  const float baseline_shift = baseline_shift_sum / static_cast<float>(valid_count);
  const float phase_coherence =
      temporal_count > 0U
          ? std::sqrt(phase_vector_real * phase_vector_real +
                      phase_vector_imaginary * phase_vector_imaginary) /
                static_cast<float>(temporal_count)
          : 0.0F;
  const float frame_motion =
      clamp(0.38F * amplitude_motion + 0.34F * phase_motion + 0.28F * ratio_motion, 0.0F, 1.0F);

  float amplitude_projection = 0.0F;
  float phase_projection = 0.0F;
  float weight_energy = 0.0F;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    if (!valid[index] || !carriers_[index].previous_valid) {
      continue;
    }
    const float weight = carriers_[index].principal_weight;
    amplitude_projection += weight * amplitude_component[index];
    phase_projection += weight * phase_component[index];
    weight_energy += weight * weight;
  }
  const float projection_scale = std::sqrt(weight_energy > 1.0e-6F ? weight_energy : 1.0F);
  amplitude_projection /= projection_scale;
  phase_projection /= projection_scale;
  const float combined_projection =
      clamp(0.58F * amplitude_projection + 0.42F * phase_projection, -6.0F, 6.0F);

  if (temporal_count >= 8U) {
    constexpr float kOjaRate = 0.0008F;
    float norm = 0.0F;
    for (std::size_t index = 0; index < frame.sample_count; ++index) {
      if (!valid[index] || !carriers_[index].previous_valid) {
        continue;
      }
      const float component =
          clamp(0.58F * amplitude_component[index] + 0.42F * phase_component[index], -6.0F, 6.0F);
      carriers_[index].principal_weight +=
          kOjaRate * combined_projection *
          (component - combined_projection * carriers_[index].principal_weight);
      norm += carriers_[index].principal_weight * carriers_[index].principal_weight;
    }
    norm = std::sqrt(norm > 1.0e-6F ? norm : 1.0F);
    for (std::size_t index = 0; index < frame.sample_count; ++index) {
      if (valid[index]) {
        carriers_[index].principal_weight /= norm;
      }
    }
  }

  const bool baseline_ready = baseline_frame_count_ >= config_.baseline_min_frames;
  const bool baseline_learning =
      !baseline_ready && (baseline_frame_count_ < 25U || frame_motion < config_.quiet_motion_limit);
  const bool quiet_update = baseline_ready && frame_motion < config_.quiet_motion_limit;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    if (!valid[index]) {
      continue;
    }
    CarrierState &state = carriers_[index];
    if (baseline_learning && state.baseline_count < config_.baseline_min_frames) {
      ++state.baseline_count;
      const float delta = centered_amplitude[index] - state.amplitude_mean;
      state.amplitude_mean += delta / static_cast<float>(state.baseline_count);
      state.amplitude_m2 += delta * (centered_amplitude[index] - state.amplitude_mean);
    } else if (quiet_update && state.baseline_count > 1U) {
      const float variance = state.amplitude_m2 / static_cast<float>(state.baseline_count - 1U);
      const float standard_deviation =
          std::sqrt(variance > config_.amplitude_std_floor * config_.amplitude_std_floor
                        ? variance
                        : config_.amplitude_std_floor * config_.amplitude_std_floor);
      const float residual = clamp(centered_amplitude[index] - state.amplitude_mean,
                                   -3.0F * standard_deviation, 3.0F * standard_deviation);
      state.amplitude_mean += config_.baseline_adaptation_alpha * residual;
      state.amplitude_m2 =
          (1.0F - config_.baseline_adaptation_alpha) * state.amplitude_m2 +
          config_.baseline_adaptation_alpha * residual * residual *
              static_cast<float>(state.baseline_count - 1U);
    }
    if ((baseline_learning || quiet_update) && state.previous_valid) {
      state.phase_noise_variance =
          (1.0F - config_.noise_adaptation_alpha) * state.phase_noise_variance +
          config_.noise_adaptation_alpha * phase_change[index] * phase_change[index];
      if (ratio_change_valid[index]) {
        state.ratio_noise_variance =
            (1.0F - config_.noise_adaptation_alpha) * state.ratio_noise_variance +
            config_.noise_adaptation_alpha * ratio_phase_change[index] *
                ratio_phase_change[index];
      }
    }
    state.previous_amplitude = centered_amplitude[index];
    state.previous_phase = phase_residual[index];
    state.previous_valid = true;
  }
  if (baseline_learning) {
    ++baseline_frame_count_;
  }

  pushFast(combined_projection);
  const float impulse =
      has_previous_projection_
          ? clamp(std::fabs(combined_projection - previous_projection_) / 3.5F, 0.0F, 1.0F)
          : 0.0F;
  previous_projection_ = combined_projection;
  has_previous_projection_ = true;
  impulse_peak_ = impulse > impulse_peak_ * 0.88F ? impulse : impulse_peak_ * 0.88F;
  motion_ewma_ = 0.92F * motion_ewma_ + 0.08F * frame_motion;
  baseline_shift_ewma_ = 0.985F * baseline_shift_ewma_ + 0.015F * baseline_shift;

  const float noise_floor = frame.noise_floor < -10 ? static_cast<float>(frame.noise_floor) : -96.0F;
  const float snr_db = static_cast<float>(frame.rssi) - noise_floor;
  const float snr_quality = clamp((snr_db - 5.0F) / 25.0F, 0.0F, 1.0F);
  const float valid_ratio = static_cast<float>(valid_count) / static_cast<float>(frame.sample_count);
  const uint16_t baseline_target = config_.baseline_min_frames > 0U ? config_.baseline_min_frames : 1U;
  const float baseline_maturity =
      clamp(static_cast<float>(baseline_frame_count_) / static_cast<float>(baseline_target), 0.0F,
            1.0F);
  const float continuity = 1.0F / (1.0F + 0.12F * static_cast<float>(probe.sequence_gap));
  const float quality =
      clamp(0.32F * valid_ratio + 0.25F * snr_quality + 0.28F * baseline_maturity +
                0.15F * continuity,
            0.0F, 1.0F);

  ++summary_frames_;
  summary_sequence_gaps_ += probe.sequence_gap;
  summary_amplitude_motion_ += amplitude_motion;
  summary_phase_motion_ += phase_motion;
  summary_ratio_motion_ += ratio_motion;
  summary_phase_coherence_ += phase_coherence;
  summary_impulse_ += impulse_peak_;
  summary_baseline_shift_ += baseline_shift_ewma_;
  summary_quality_ += quality;
  summary_valid_ratio_ += valid_ratio;
  summary_snr_db_ += snr_db;
  summary_amplitude_projection_ += amplitude_projection;
  summary_phase_projection_ += phase_projection;

  const uint16_t frames_per_summary = static_cast<uint16_t>(
      config_.summary_rate_hz > 0U
          ? (config_.target_rate_hz + config_.summary_rate_hz - 1U) / config_.summary_rate_hz
          : 1U);
  if (summary_frames_ < (frames_per_summary > 0U ? frames_per_summary : 1U)) {
    return baseline_ready ? CsiPrecisionUpdateStatus::Collecting
                          : CsiPrecisionUpdateStatus::CollectingBaseline;
  }

  const float divisor = static_cast<float>(summary_frames_);
  pushSlow(summary_amplitude_projection_ / divisor, summary_phase_projection_ / divisor);

  constexpr float kDopplerFrequencies[] = {1.5F, 3.0F, 5.0F, 8.0F,
                                           12.0F, 18.0F, 26.0F, 34.0F};
  float doppler_power = 0.0F;
  float strongest_doppler_bin = 0.0F;
  for (float frequency : kDopplerFrequencies) {
    const float power = spectralPower(fast_history_, kFastHistoryCapacity, fast_head_, fast_count_,
                                      static_cast<float>(config_.target_rate_hz), frequency);
    doppler_power += power;
    if (power > strongest_doppler_bin) {
      strongest_doppler_bin = power;
    }
  }
  const float doppler_energy = clamp(std::sqrt(doppler_power / 8.0F) / 1.6F, 0.0F, 1.0F);
  const float doppler_concentration =
      doppler_power > 1.0e-6F ? strongest_doppler_bin / doppler_power : 1.0F;
  const float broadband =
      clamp(doppler_energy * (1.0F - doppler_concentration) * 1.8F, 0.0F, 1.0F);
  broadband_ewma_ = 0.85F * broadband_ewma_ + 0.15F * broadband;

  float respiration_power = 0.0F;
  float respiration_coherence = 0.0F;
  if (slow_count_ >= config_.respiration_min_summaries) {
    constexpr float kRespirationFrequencies[] = {0.12F, 0.16F, 0.20F, 0.25F,
                                                 0.30F, 0.36F, 0.42F, 0.50F};
    constexpr float kNoiseFrequencies[] = {0.65F, 0.80F, 1.00F, 1.25F};
    float amplitude_peak = 0.0F;
    float phase_peak = 0.0F;
    float amplitude_sum = 0.0F;
    float phase_sum = 0.0F;
    float amplitude_frequency = 0.0F;
    float phase_frequency = 0.0F;
    for (float frequency : kRespirationFrequencies) {
      const float amplitude_power =
          spectralPower(slow_amplitude_history_, kSlowHistoryCapacity, slow_head_, slow_count_,
                        static_cast<float>(config_.summary_rate_hz), frequency);
      const float phase_power =
          spectralPower(slow_phase_history_, kSlowHistoryCapacity, slow_head_, slow_count_,
                        static_cast<float>(config_.summary_rate_hz), frequency);
      amplitude_sum += amplitude_power;
      phase_sum += phase_power;
      if (amplitude_power > amplitude_peak) {
        amplitude_peak = amplitude_power;
        amplitude_frequency = frequency;
      }
      if (phase_power > phase_peak) {
        phase_peak = phase_power;
        phase_frequency = frequency;
      }
    }
    float amplitude_noise = 0.0F;
    float phase_noise = 0.0F;
    for (float frequency : kNoiseFrequencies) {
      amplitude_noise +=
          spectralPower(slow_amplitude_history_, kSlowHistoryCapacity, slow_head_, slow_count_,
                        static_cast<float>(config_.summary_rate_hz), frequency);
      phase_noise +=
          spectralPower(slow_phase_history_, kSlowHistoryCapacity, slow_head_, slow_count_,
                        static_cast<float>(config_.summary_rate_hz), frequency);
    }
    amplitude_noise *= 0.25F;
    phase_noise *= 0.25F;
    const float best_snr =
        amplitude_peak / (amplitude_noise + 1.0e-6F) > phase_peak / (phase_noise + 1.0e-6F)
            ? amplitude_peak / (amplitude_noise + 1.0e-6F)
            : phase_peak / (phase_noise + 1.0e-6F);
    const float peak_strength =
        std::sqrt(amplitude_peak > phase_peak ? amplitude_peak : phase_peak);
    const float window_maturity =
        clamp(static_cast<float>(slow_count_) /
                  (20.0F * static_cast<float>(config_.summary_rate_hz)),
              0.0F, 1.0F);
    respiration_power =
        clamp((std::log10(1.0F + best_snr) / 1.4F) *
                  clamp(peak_strength / 0.55F, 0.0F, 1.0F) * window_maturity,
              0.0F, 1.0F);
    const float amplitude_concentration =
        amplitude_sum > 1.0e-6F ? amplitude_peak / amplitude_sum : 0.0F;
    const float phase_concentration = phase_sum > 1.0e-6F ? phase_peak / phase_sum : 0.0F;
    const float frequency_agreement =
        std::exp(-std::fabs(amplitude_frequency - phase_frequency) / 0.08F);
    respiration_coherence =
        clamp(frequency_agreement *
                  std::sqrt(clamp(amplitude_concentration, 0.0F, 1.0F) *
                            clamp(phase_concentration, 0.0F, 1.0F)),
              0.0F, 1.0F);
  }

  observation.receiver_id = receiver_id;
  observation.channel = frame.channel;
  observation.bandwidth_mhz = frame.bandwidth_mhz;
  observation.probe_sequence = probe.sequence;
  observation.observation_sequence = observation_sequence_++;
  observation.tx_uptime_us = probe.tx_uptime_us;
  observation.frames_in_summary = summary_frames_;
  observation.sequence_gap_count = static_cast<uint16_t>(
      summary_sequence_gaps_ > 65535U ? 65535U : summary_sequence_gaps_);
  observation.snr_db = summary_snr_db_ / divisor;
  observation.valid_ratio = summary_valid_ratio_ / divisor;
  observation.baseline_maturity = baseline_maturity;
  observation.amplitude_motion = summary_amplitude_motion_ / divisor;
  observation.differential_phase_motion = summary_phase_motion_ / divisor;
  observation.complex_ratio_motion = summary_ratio_motion_ / divisor;
  observation.phase_coherence = summary_phase_coherence_ / divisor;
  observation.doppler_energy = doppler_energy;
  observation.respiration_power = respiration_power;
  observation.respiration_coherence = respiration_coherence;
  observation.impulse_score = clamp(summary_impulse_ / divisor, 0.0F, 1.0F);
  observation.stillness_score = clamp(1.0F - motion_ewma_ * 1.7F, 0.0F, 1.0F);
  observation.baseline_shift = clamp(summary_baseline_shift_ / divisor, 0.0F, 1.0F);
  observation.broadband_nuisance = broadband_ewma_;
  observation.quality = summary_quality_ / divisor;
  observation.baseline_ready = baseline_frame_count_ >= config_.baseline_min_frames;

  summary_frames_ = 0;
  summary_sequence_gaps_ = 0;
  summary_amplitude_motion_ = 0.0F;
  summary_phase_motion_ = 0.0F;
  summary_ratio_motion_ = 0.0F;
  summary_phase_coherence_ = 0.0F;
  summary_impulse_ = 0.0F;
  summary_baseline_shift_ = 0.0F;
  summary_quality_ = 0.0F;
  summary_valid_ratio_ = 0.0F;
  summary_snr_db_ = 0.0F;
  summary_amplitude_projection_ = 0.0F;
  summary_phase_projection_ = 0.0F;
  return CsiPrecisionUpdateStatus::Updated;
}

void M5AtomCsiLinkProcessor::pushFast(float value) {
  fast_history_[fast_head_] = value;
  fast_head_ = (fast_head_ + 1U) % kFastHistoryCapacity;
  if (fast_count_ < kFastHistoryCapacity) {
    ++fast_count_;
  }
}

void M5AtomCsiLinkProcessor::pushSlow(float amplitude, float phase) {
  slow_amplitude_history_[slow_head_] = amplitude;
  slow_phase_history_[slow_head_] = phase;
  slow_head_ = (slow_head_ + 1U) % kSlowHistoryCapacity;
  if (slow_count_ < kSlowHistoryCapacity) {
    ++slow_count_;
  }
}

float M5AtomCsiLinkProcessor::historyValue(const float *history, std::size_t capacity,
                                           std::size_t head, std::size_t count,
                                           std::size_t chronological_index) const {
  const std::size_t oldest = (head + capacity - count) % capacity;
  return history[(oldest + chronological_index) % capacity];
}

float M5AtomCsiLinkProcessor::spectralPower(const float *history, std::size_t capacity,
                                            std::size_t head, std::size_t count,
                                            float sample_rate_hz, float frequency_hz) const {
  if (count < 16U || sample_rate_hz <= 0.0F || frequency_hz <= 0.0F) {
    return 0.0F;
  }
  float mean = 0.0F;
  for (std::size_t index = 0; index < count; ++index) {
    mean += historyValue(history, capacity, head, count, index);
  }
  mean /= static_cast<float>(count);
  float real = 0.0F;
  float imaginary = 0.0F;
  float window_sum = 0.0F;
  for (std::size_t index = 0; index < count; ++index) {
    const float phase = kTwoPi * frequency_hz * static_cast<float>(index) / sample_rate_hz;
    const float window = count > 1U
                             ? 0.5F - 0.5F *
                                          std::cos(kTwoPi * static_cast<float>(index) /
                                                   static_cast<float>(count - 1U))
                             : 1.0F;
    const float centered = historyValue(history, capacity, head, count, index) - mean;
    real += centered * window * std::cos(phase);
    imaginary -= centered * window * std::sin(phase);
    window_sum += window;
  }
  const float normalization = window_sum > 1.0e-6F ? 2.0F / window_sum : 0.0F;
  return (real * real + imaginary * imaginary) * normalization * normalization;
}

float M5AtomCsiLinkProcessor::clamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

float M5AtomCsiLinkProcessor::wrapPhase(float value) {
  while (value > kPi) {
    value -= kTwoPi;
  }
  while (value < -kPi) {
    value += kTwoPi;
  }
  return value;
}

float M5AtomCsiLinkProcessor::median(float *values, std::size_t count) {
  if (count == 0U) {
    return 0.0F;
  }
  for (std::size_t index = 1; index < count; ++index) {
    const float value = values[index];
    std::size_t position = index;
    while (position > 0U && values[position - 1U] > value) {
      values[position] = values[position - 1U];
      --position;
    }
    values[position] = value;
  }
  const std::size_t middle = count / 2U;
  return count % 2U == 0U ? 0.5F * (values[middle - 1U] + values[middle]) : values[middle];
}

M5AtomCsiFusion::M5AtomCsiFusion(M5AtomCsiPrecisionConfig config) : config_(config) { reset(); }

void M5AtomCsiFusion::reset() { std::memset(links_, 0, sizeof(links_)); }

bool M5AtomCsiFusion::ingestPacket(const uint8_t *data, std::size_t length,
                                  uint32_t expected_system_id, int64_t received_at_us) {
  CsiObservationPacket packet{};
  if (!decodeCsiObservationPacket(data, length, packet) ||
      (expected_system_id != 0U && packet.system_id != expected_system_id)) {
    return false;
  }
  LinkSlot &slot = links_[static_cast<std::size_t>(packet.receiver_id - 1U)];
  if (slot.occupied &&
      (packet.observation_sequence == slot.observation.observation_sequence ||
       !protocol::isSequenceNewer(packet.observation_sequence,
                                  slot.observation.observation_sequence))) {
    return false;
  }
  slot.observation = observationFromPacket(packet);
  slot.received_at_us = received_at_us;
  slot.occupied = true;
  return true;
}

bool M5AtomCsiFusion::snapshot(int64_t now_us, FusedCsiObservation &observation) const {
  observation = {};
  uint64_t newest_tx_uptime = 0;
  for (const LinkSlot &slot : links_) {
    if (slot.occupied && now_us - slot.received_at_us <= config_.fusion_freshness_us &&
        slot.observation.tx_uptime_us > newest_tx_uptime) {
      newest_tx_uptime = slot.observation.tx_uptime_us;
      observation.anchor_probe_sequence = slot.observation.probe_sequence;
    }
  }
  if (newest_tx_uptime == 0U) {
    return false;
  }

  std::size_t indices[kMaximumM5AtomCsiReceivers]{};
  std::size_t count = 0;
  for (std::size_t index = 0; index < kMaximumM5AtomCsiReceivers; ++index) {
    const LinkSlot &slot = links_[index];
    if (!slot.occupied || now_us - slot.received_at_us > config_.fusion_freshness_us) {
      continue;
    }
    const uint64_t age = newest_tx_uptime - slot.observation.tx_uptime_us;
    if (age <= static_cast<uint64_t>(config_.fusion_alignment_us)) {
      indices[count++] = index;
    }
  }
  if (count == 0U) {
    return false;
  }

  observation.anchor_tx_uptime_us = newest_tx_uptime;
  observation.active_links = static_cast<uint8_t>(count);
  observation.amplitude_motion =
      robustAggregate(&CsiLinkObservation::amplitude_motion, indices, count);
  observation.differential_phase_motion =
      robustAggregate(&CsiLinkObservation::differential_phase_motion, indices, count);
  observation.complex_ratio_motion =
      robustAggregate(&CsiLinkObservation::complex_ratio_motion, indices, count);
  observation.phase_coherence =
      robustAggregate(&CsiLinkObservation::phase_coherence, indices, count);
  observation.doppler_energy = robustAggregate(&CsiLinkObservation::doppler_energy, indices, count);
  observation.respiration_power =
      robustAggregate(&CsiLinkObservation::respiration_power, indices, count);
  observation.respiration_coherence =
      robustAggregate(&CsiLinkObservation::respiration_coherence, indices, count);
  observation.impulse_score = robustAggregate(&CsiLinkObservation::impulse_score, indices, count);
  observation.stillness_score =
      robustAggregate(&CsiLinkObservation::stillness_score, indices, count);
  observation.baseline_shift =
      robustAggregate(&CsiLinkObservation::baseline_shift, indices, count);
  observation.broadband_nuisance =
      robustAggregate(&CsiLinkObservation::broadband_nuisance, indices, count);

  float disagreement = 0.0F;
  float quality_weight = 0.0F;
  for (std::size_t position = 0; position < count; ++position) {
    const CsiLinkObservation &link = links_[indices[position]].observation;
    const float link_motion =
        (link.amplitude_motion + link.differential_phase_motion + link.complex_ratio_motion) / 3.0F;
    const float fused_motion =
        (observation.amplitude_motion + observation.differential_phase_motion +
         observation.complex_ratio_motion) /
        3.0F;
    const float delta = link_motion - fused_motion;
    disagreement += link.quality * delta * delta;
    quality_weight += link.quality;
  }
  const float spread =
      quality_weight > 1.0e-6F ? std::sqrt(disagreement / quality_weight) : 1.0F;
  observation.link_agreement = clamp01(1.0F - spread / 0.35F);
  const float base_quality = robustAggregate(&CsiLinkObservation::quality, indices, count);
  const float link_factor =
      0.75F + 0.25F * static_cast<float>(count) /
                  static_cast<float>(kMaximumM5AtomCsiReceivers);
  observation.quality = clamp01(base_quality * link_factor *
                                (0.70F + 0.30F * observation.link_agreement));
  observation.physically_observable = observation.quality >= 0.12F;
  return true;
}

float M5AtomCsiFusion::robustAggregate(float CsiLinkObservation::*member,
                                      const std::size_t *indices, std::size_t count) const {
  float ordered[kMaximumM5AtomCsiReceivers]{};
  for (std::size_t position = 0; position < count; ++position) {
    ordered[position] = links_[indices[position]].observation.*member;
  }
  for (std::size_t index = 1; index < count; ++index) {
    const float value = ordered[index];
    std::size_t position = index;
    while (position > 0U && ordered[position - 1U] > value) {
      ordered[position] = ordered[position - 1U];
      --position;
    }
    ordered[position] = value;
  }
  const float center = count % 2U == 0U
                           ? 0.5F * (ordered[count / 2U - 1U] + ordered[count / 2U])
                           : ordered[count / 2U];
  float weighted_sum = 0.0F;
  float weight_sum = 0.0F;
  for (std::size_t position = 0; position < count; ++position) {
    const CsiLinkObservation &link = links_[indices[position]].observation;
    const float value = link.*member;
    const float deviation = std::fabs(value - center) / 0.20F;
    const float weight = link.quality / (1.0F + deviation * deviation);
    weighted_sum += weight * value;
    weight_sum += weight;
  }
  return clamp01(weight_sum > 1.0e-6F ? weighted_sum / weight_sum : center);
}

float M5AtomCsiFusion::clamp01(float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

}  // namespace atom::radar
