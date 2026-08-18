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
  observation.window_start_probe_sequence = packet.window_start_probe_sequence;
  observation.observation_sequence = packet.observation_sequence;
  observation.tx_uptime_us = packet.tx_uptime_us;
  observation.frames_in_summary = packet.frames_in_summary;
  observation.expected_frames_in_summary = packet.expected_frames_in_summary;
  observation.sequence_gap_count = packet.sequence_gap_count;
  observation.snr_db = static_cast<float>(packet.snr_db_x10) * 0.1F;
  observation.valid_ratio = packetMetric(packet, CsiObservationMetric::ValidRatio);
  observation.window_completeness =
      packetMetric(packet, CsiObservationMetric::WindowCompleteness);
  observation.subcarrier_reliability =
      packetMetric(packet, CsiObservationMetric::SubcarrierReliability);
  observation.baseline_maturity = packetMetric(packet, CsiObservationMetric::BaselineMaturity);
  observation.amplitude_motion = packetMetric(packet, CsiObservationMetric::AmplitudeMotion);
  observation.differential_phase_motion =
      packetMetric(packet, CsiObservationMetric::DifferentialPhaseMotion);
  observation.complex_ratio_motion =
      packetMetric(packet, CsiObservationMetric::ComplexRatioMotion);
  observation.phase_coherence = packetMetric(packet, CsiObservationMetric::PhaseCoherence);
  observation.delay_domain_motion =
      packetMetric(packet, CsiObservationMetric::DelayDomainMotion);
  observation.delay_spread = packetMetric(packet, CsiObservationMetric::DelaySpread);
  observation.dynamic_tap_concentration =
      packetMetric(packet, CsiObservationMetric::DynamicTapConcentration);
  observation.doppler_energy = packetMetric(packet, CsiObservationMetric::DopplerEnergy);
  observation.doppler_centroid = packetMetric(packet, CsiObservationMetric::DopplerCentroid);
  observation.doppler_bandwidth = packetMetric(packet, CsiObservationMetric::DopplerBandwidth);
  observation.doppler_asymmetry = packetMetric(packet, CsiObservationMetric::DopplerAsymmetry);
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
  packet.window_start_probe_sequence = observation.window_start_probe_sequence;
  packet.tx_uptime_us = observation.tx_uptime_us;
  packet.observation_sequence = observation.observation_sequence;
  packet.frames_in_summary = observation.frames_in_summary;
  packet.expected_frames_in_summary = observation.expected_frames_in_summary;
  packet.sequence_gap_count = observation.sequence_gap_count;
  const float snr_x10 = observation.snr_db * 10.0F;
  packet.snr_db_x10 = static_cast<int16_t>(
      snr_x10 < -32768.0F ? -32768.0F : (snr_x10 > 32767.0F ? 32767.0F : snr_x10));
  packet.flags = observation.baseline_ready ? kBaselineReadyFlag : 0U;
  packet.metrics[metricIndex(CsiObservationMetric::ValidRatio)] =
      unitToWire(observation.valid_ratio);
  packet.metrics[metricIndex(CsiObservationMetric::WindowCompleteness)] =
      unitToWire(observation.window_completeness);
  packet.metrics[metricIndex(CsiObservationMetric::SubcarrierReliability)] =
      unitToWire(observation.subcarrier_reliability);
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
  packet.metrics[metricIndex(CsiObservationMetric::DelayDomainMotion)] =
      unitToWire(observation.delay_domain_motion);
  packet.metrics[metricIndex(CsiObservationMetric::DelaySpread)] =
      unitToWire(observation.delay_spread);
  packet.metrics[metricIndex(CsiObservationMetric::DynamicTapConcentration)] =
      unitToWire(observation.dynamic_tap_concentration);
  packet.metrics[metricIndex(CsiObservationMetric::DopplerEnergy)] =
      unitToWire(observation.doppler_energy);
  packet.metrics[metricIndex(CsiObservationMetric::DopplerCentroid)] =
      unitToWire(observation.doppler_centroid);
  packet.metrics[metricIndex(CsiObservationMetric::DopplerBandwidth)] =
      unitToWire(observation.doppler_bandwidth);
  packet.metrics[metricIndex(CsiObservationMetric::DopplerAsymmetry)] =
      unitToWire(observation.doppler_asymmetry);
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
  std::memset(delay_taps_, 0, sizeof(delay_taps_));
  std::memset(fast_history_, 0, sizeof(fast_history_));
  std::memset(fast_complex_real_history_, 0, sizeof(fast_complex_real_history_));
  std::memset(fast_complex_imaginary_history_, 0, sizeof(fast_complex_imaginary_history_));
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
    state.validity_ewma = 1.0F;
    state.continuity_ewma = 1.0F;
  }
  for (DelayTapState &tap : delay_taps_) {
    tap.noise_variance = config_.delay_noise_floor * config_.delay_noise_floor;
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
  summary_epoch_ = 0;
  summary_first_probe_sequence_ = 0;
  has_summary_epoch_ = false;
  summary_sequence_gaps_ = 0;
  summary_amplitude_motion_ = 0.0F;
  summary_phase_motion_ = 0.0F;
  summary_ratio_motion_ = 0.0F;
  summary_phase_coherence_ = 0.0F;
  summary_impulse_ = 0.0F;
  summary_baseline_shift_ = 0.0F;
  summary_quality_ = 0.0F;
  summary_valid_ratio_ = 0.0F;
  summary_subcarrier_reliability_ = 0.0F;
  summary_snr_db_ = 0.0F;
  summary_delay_motion_ = 0.0F;
  summary_delay_spread_ = 0.0F;
  summary_dynamic_tap_concentration_ = 0.0F;
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
  float carrier_reliability[kMaximumHtSubcarrierCount]{};
  float phase_change[kMaximumHtSubcarrierCount]{};
  float ratio_phase_change[kMaximumHtSubcarrierCount]{};
  bool ratio_change_valid[kMaximumHtSubcarrierCount]{};
  bool valid[kMaximumHtSubcarrierCount]{};
  float scratch[kMaximumHtSubcarrierCount]{};

  std::size_t valid_count = 0;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    const CsiComplexSample &sample = frame.samples[index];
    const float real = static_cast<float>(sample.real);
    const float imaginary = static_cast<float>(sample.imaginary);
    const float power = real * real + imaginary * imaginary;
    const bool saturated = std::fabs(real) >= 126.0F || std::fabs(imaginary) >= 126.0F;
    const bool usable = sample.valid_for_features && power >= 1.0F;
    CarrierState &state = carriers_[index];
    state.validity_ewma =
        (1.0F - config_.reliability_adaptation_alpha) * state.validity_ewma +
        config_.reliability_adaptation_alpha * (usable ? 1.0F : 0.0F);
    state.saturation_ewma =
        (1.0F - config_.reliability_adaptation_alpha) * state.saturation_ewma +
        config_.reliability_adaptation_alpha * (saturated ? 1.0F : 0.0F);
    state.continuity_ewma =
        (1.0F - config_.reliability_adaptation_alpha) * state.continuity_ewma +
        config_.reliability_adaptation_alpha * (usable && state.previous_valid ? 1.0F : 0.0F);
    if (!usable) {
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
  float temporal_weight_sum = 0.0F;
  float ratio_weight_sum = 0.0F;
  float reliability_sum = 0.0F;
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
    const float phase_std =
        std::sqrt(state.phase_noise_variance >
                          config_.phase_noise_floor_rad * config_.phase_noise_floor_rad
                      ? state.phase_noise_variance
                      : config_.phase_noise_floor_rad * config_.phase_noise_floor_rad);
    const float phase_stability = 1.0F / (1.0F + phase_std / 0.25F);
    carrier_reliability[index] =
        clamp(state.validity_ewma * (1.0F - state.saturation_ewma) *
                  (0.40F + 0.60F * state.continuity_ewma) * phase_stability,
              0.05F, 1.0F);
    reliability_sum += carrier_reliability[index];
    const float baseline_z = state.baseline_count > 8U
                                 ? (centered_amplitude[index] - state.amplitude_mean) / amplitude_std
                                 : 0.0F;
    baseline_shift_sum += clamp(std::fabs(baseline_z) / 6.0F, 0.0F, 1.0F);

    if (state.previous_valid) {
      const float amplitude_delta =
          (centered_amplitude[index] - state.previous_amplitude) / amplitude_std;
      phase_change[index] = wrapPhase(phase_residual[index] - state.previous_phase);
      amplitude_component[index] = clamp(amplitude_delta, -6.0F, 6.0F);
      phase_component[index] = clamp(phase_change[index] / phase_std, -6.0F, 6.0F);
      amplitude_motion_sum += carrier_reliability[index] *
                              clamp(std::fabs(amplitude_delta) / 4.0F, 0.0F, 1.0F);
      phase_motion_sum += carrier_reliability[index] *
                          clamp(std::fabs(phase_change[index]) / (4.0F * phase_std), 0.0F, 1.0F);
      phase_vector_real += carrier_reliability[index] * std::cos(phase_change[index]);
      phase_vector_imaginary += carrier_reliability[index] * std::sin(phase_change[index]);
      temporal_weight_sum += carrier_reliability[index];
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
          const float pair_reliability =
              carrier_reliability[index] < carrier_reliability[previous_index]
                  ? carrier_reliability[index]
                  : carrier_reliability[previous_index];
          ratio_motion_sum += pair_reliability * clamp(normalized / 6.0F, 0.0F, 1.0F);
          ratio_weight_sum += pair_reliability;
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

  const float amplitude_motion = temporal_weight_sum > 1.0e-6F
                                     ? amplitude_motion_sum / temporal_weight_sum
                                     : 0.0F;
  const float phase_motion = temporal_weight_sum > 1.0e-6F
                                 ? phase_motion_sum / temporal_weight_sum
                                 : 0.0F;
  const float ratio_motion =
      ratio_weight_sum > 1.0e-6F ? ratio_motion_sum / ratio_weight_sum : 0.0F;
  const float baseline_shift = baseline_shift_sum / static_cast<float>(valid_count);
  const float phase_coherence =
      temporal_weight_sum > 1.0e-6F
          ? std::sqrt(phase_vector_real * phase_vector_real +
                      phase_vector_imaginary * phase_vector_imaginary) /
                temporal_weight_sum
          : 0.0F;
  const float frequency_motion =
      clamp(0.38F * amplitude_motion + 0.34F * phase_motion + 0.28F * ratio_motion, 0.0F, 1.0F);

  float amplitude_projection = 0.0F;
  float phase_projection = 0.0F;
  float weight_energy = 0.0F;
  for (std::size_t index = 0; index < frame.sample_count; ++index) {
    if (!valid[index] || !carriers_[index].previous_valid) {
      continue;
    }
    const float weight =
        carriers_[index].principal_weight * std::sqrt(carrier_reliability[index]);
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

  float delay_real[kDelayTapCount]{};
  float delay_imaginary[kDelayTapCount]{};
  float delay_energy[kDelayTapCount]{};
  const float transform_size = frame.bandwidth_mhz == 40U ? 128.0F : 64.0F;
  for (std::size_t tap_index = 0; tap_index < kDelayTapCount; ++tap_index) {
    float weight_sum = 0.0F;
    for (std::size_t carrier_index = 0; carrier_index < frame.sample_count; ++carrier_index) {
      if (!valid[carrier_index]) {
        continue;
      }
      const float weight = carrier_reliability[carrier_index];
      const float magnitude = std::exp(clamp(centered_amplitude[carrier_index], -4.0F, 4.0F));
      const float phase = phase_residual[carrier_index] +
                          kTwoPi * static_cast<float>(frame.samples[carrier_index].subcarrier) *
                              static_cast<float>(tap_index) / transform_size;
      delay_real[tap_index] += weight * magnitude * std::cos(phase);
      delay_imaginary[tap_index] += weight * magnitude * std::sin(phase);
      weight_sum += weight;
    }
    if (weight_sum > 1.0e-6F) {
      delay_real[tap_index] /= weight_sum;
      delay_imaginary[tap_index] /= weight_sum;
    }
  }

  float delay_energy_sum = 0.0F;
  float delay_peak = 0.0F;
  float delay_index_sum = 0.0F;
  for (std::size_t tap_index = 0; tap_index < kDelayTapCount; ++tap_index) {
    const DelayTapState &tap = delay_taps_[tap_index];
    if (!tap.previous_valid) {
      continue;
    }
    const float temporal_real = delay_real[tap_index] - tap.previous_real;
    const float temporal_imaginary = delay_imaginary[tap_index] - tap.previous_imaginary;
    const float temporal_distance =
        std::sqrt(temporal_real * temporal_real + temporal_imaginary * temporal_imaginary);
    float static_distance = 0.0F;
    if (tap.baseline_count > 8U) {
      const float static_real = delay_real[tap_index] - tap.baseline_real;
      const float static_imaginary = delay_imaginary[tap_index] - tap.baseline_imaginary;
      static_distance = std::sqrt(static_real * static_real + static_imaginary * static_imaginary);
    }
    const float delay_std =
        std::sqrt(tap.noise_variance > config_.delay_noise_floor * config_.delay_noise_floor
                      ? tap.noise_variance
                      : config_.delay_noise_floor * config_.delay_noise_floor);
    delay_energy[tap_index] =
        clamp((0.58F * temporal_distance + 0.42F * static_distance) /
                  (4.0F * delay_std),
              0.0F, 1.0F);
    delay_energy_sum += delay_energy[tap_index];
    delay_index_sum += delay_energy[tap_index] * static_cast<float>(tap_index);
    if (delay_energy[tap_index] > delay_peak) {
      delay_peak = delay_energy[tap_index];
    }
  }
  const float delay_domain_motion = delay_energy_sum / static_cast<float>(kDelayTapCount);
  const float delay_center = delay_energy_sum > 1.0e-6F ? delay_index_sum / delay_energy_sum : 0.0F;
  float delay_variance = 0.0F;
  for (std::size_t tap_index = 0; tap_index < kDelayTapCount; ++tap_index) {
    const float offset = static_cast<float>(tap_index) - delay_center;
    delay_variance += delay_energy[tap_index] * offset * offset;
  }
  const float delay_spread =
      delay_energy_sum > 1.0e-6F
          ? clamp(std::sqrt(delay_variance / delay_energy_sum) /
                      (0.5F * static_cast<float>(kDelayTapCount)),
                  0.0F, 1.0F)
          : 0.0F;
  const float dynamic_tap_concentration =
      delay_energy_sum > 1.0e-6F ? clamp(delay_peak / delay_energy_sum, 0.0F, 1.0F) : 0.0F;
  const float frame_motion =
      clamp(0.30F * amplitude_motion + 0.26F * phase_motion + 0.20F * ratio_motion +
                0.24F * delay_domain_motion,
            0.0F, 1.0F);

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
  for (std::size_t tap_index = 0; tap_index < kDelayTapCount; ++tap_index) {
    DelayTapState &tap = delay_taps_[tap_index];
    if (baseline_learning && tap.baseline_count < config_.baseline_min_frames) {
      ++tap.baseline_count;
      tap.baseline_real +=
          (delay_real[tap_index] - tap.baseline_real) / static_cast<float>(tap.baseline_count);
      tap.baseline_imaginary +=
          (delay_imaginary[tap_index] - tap.baseline_imaginary) /
          static_cast<float>(tap.baseline_count);
    } else if (quiet_update) {
      tap.baseline_real +=
          config_.baseline_adaptation_alpha * (delay_real[tap_index] - tap.baseline_real);
      tap.baseline_imaginary += config_.baseline_adaptation_alpha *
                                (delay_imaginary[tap_index] - tap.baseline_imaginary);
    }
    if ((baseline_learning || quiet_update) && tap.previous_valid) {
      const float delta_real = delay_real[tap_index] - tap.previous_real;
      const float delta_imaginary = delay_imaginary[tap_index] - tap.previous_imaginary;
      const float delta_power = delta_real * delta_real + delta_imaginary * delta_imaginary;
      tap.noise_variance =
          (1.0F - config_.noise_adaptation_alpha) * tap.noise_variance +
          config_.noise_adaptation_alpha * delta_power;
    }
    tap.previous_real = delay_real[tap_index];
    tap.previous_imaginary = delay_imaginary[tap_index];
    tap.previous_valid = true;
  }

  const float phase_increment_real =
      temporal_weight_sum > 1.0e-6F ? phase_vector_real / temporal_weight_sum : 1.0F;
  const float phase_increment_imaginary =
      temporal_weight_sum > 1.0e-6F ? phase_vector_imaginary / temporal_weight_sum : 0.0F;
  pushFast(combined_projection, phase_increment_real, phase_increment_imaginary);
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

  const uint16_t frames_per_summary = static_cast<uint16_t>(
      config_.summary_rate_hz > 0U
          ? (config_.target_rate_hz + config_.summary_rate_hz - 1U) / config_.summary_rate_hz
          : 1U);
  const uint16_t expected_summary_frames = frames_per_summary > 0U ? frames_per_summary : 1U;
  const uint32_t current_epoch = probe.sequence / expected_summary_frames;
  if (!has_summary_epoch_) {
    summary_epoch_ = current_epoch;
    has_summary_epoch_ = true;
  }
  if (summary_frames_ == 0U) {
    summary_first_probe_sequence_ = probe.sequence;
  }

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
  summary_subcarrier_reliability_ += reliability_sum / static_cast<float>(valid_count);
  summary_snr_db_ += snr_db;
  summary_delay_motion_ += delay_domain_motion;
  summary_delay_spread_ += delay_spread;
  summary_dynamic_tap_concentration_ += dynamic_tap_concentration;
  summary_amplitude_projection_ += amplitude_projection;
  summary_phase_projection_ += phase_projection;

  if (current_epoch == summary_epoch_) {
    return baseline_ready ? CsiPrecisionUpdateStatus::Collecting
                          : CsiPrecisionUpdateStatus::CollectingBaseline;
  }
  summary_epoch_ = current_epoch;

  const float divisor = static_cast<float>(summary_frames_);
  pushSlow(summary_amplitude_projection_ / divisor, summary_phase_projection_ / divisor);

  constexpr float kDopplerFrequencies[] = {1.5F, 3.0F, 5.0F, 8.0F,
                                           12.0F, 18.0F, 26.0F, 34.0F};
  float scalar_doppler_power = 0.0F;
  float complex_doppler_power = 0.0F;
  float positive_doppler_power = 0.0F;
  float negative_doppler_power = 0.0F;
  float weighted_frequency = 0.0F;
  float weighted_frequency_squared = 0.0F;
  float strongest_doppler_bin = 0.0F;
  for (float frequency : kDopplerFrequencies) {
    const float scalar_power =
        spectralPower(fast_history_, kFastHistoryCapacity, fast_head_, fast_count_,
                      static_cast<float>(config_.target_rate_hz), frequency);
    const float positive_power = complexSpectralPower(frequency);
    const float negative_power = complexSpectralPower(-frequency);
    const float complex_power = positive_power + negative_power;
    const float combined_power = 0.45F * scalar_power + 0.55F * complex_power;
    scalar_doppler_power += scalar_power;
    complex_doppler_power += complex_power;
    positive_doppler_power += positive_power;
    negative_doppler_power += negative_power;
    weighted_frequency += frequency * combined_power;
    weighted_frequency_squared += frequency * frequency * combined_power;
    if (combined_power > strongest_doppler_bin) {
      strongest_doppler_bin = combined_power;
    }
  }
  const float doppler_power = 0.45F * scalar_doppler_power + 0.55F * complex_doppler_power;
  const float doppler_energy = clamp(std::sqrt(doppler_power / 8.0F) / 1.6F, 0.0F, 1.0F);
  const float centroid_hz =
      doppler_power > 1.0e-6F ? weighted_frequency / doppler_power : 0.0F;
  const float frequency_variance =
      doppler_power > 1.0e-6F
          ? weighted_frequency_squared / doppler_power - centroid_hz * centroid_hz
          : 0.0F;
  const float doppler_centroid = clamp(centroid_hz / 34.0F, 0.0F, 1.0F);
  const float doppler_bandwidth =
      clamp(std::sqrt(frequency_variance > 0.0F ? frequency_variance : 0.0F) / 17.0F,
            0.0F, 1.0F);
  const float directional_power = positive_doppler_power + negative_doppler_power;
  const float doppler_asymmetry =
      directional_power > 1.0e-6F
          ? clamp(std::fabs(positive_doppler_power - negative_doppler_power) /
                      directional_power,
                  0.0F, 1.0F)
          : 0.0F;
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
  observation.window_start_probe_sequence = summary_first_probe_sequence_;
  observation.observation_sequence = observation_sequence_++;
  observation.tx_uptime_us = probe.tx_uptime_us;
  observation.frames_in_summary = summary_frames_;
  observation.expected_frames_in_summary = expected_summary_frames;
  observation.sequence_gap_count = static_cast<uint16_t>(
      summary_sequence_gaps_ > 65535U ? 65535U : summary_sequence_gaps_);
  observation.snr_db = summary_snr_db_ / divisor;
  observation.valid_ratio = summary_valid_ratio_ / divisor;
  const uint32_t sequence_span = probe.sequence - summary_first_probe_sequence_ + 1U;
  const float captured_ratio =
      clamp(static_cast<float>(summary_frames_) / static_cast<float>(expected_summary_frames),
            0.0F, 1.0F);
  const float span_ratio =
      sequence_span > expected_summary_frames
          ? static_cast<float>(expected_summary_frames) / static_cast<float>(sequence_span)
          : 1.0F;
  observation.window_completeness = clamp(captured_ratio * span_ratio, 0.0F, 1.0F);
  observation.subcarrier_reliability = summary_subcarrier_reliability_ / divisor;
  observation.baseline_maturity = baseline_maturity;
  observation.amplitude_motion = summary_amplitude_motion_ / divisor;
  observation.differential_phase_motion = summary_phase_motion_ / divisor;
  observation.complex_ratio_motion = summary_ratio_motion_ / divisor;
  observation.phase_coherence = summary_phase_coherence_ / divisor;
  observation.delay_domain_motion = summary_delay_motion_ / divisor;
  observation.delay_spread = summary_delay_spread_ / divisor;
  observation.dynamic_tap_concentration =
      summary_dynamic_tap_concentration_ / divisor;
  observation.doppler_energy = doppler_energy;
  observation.doppler_centroid = doppler_centroid;
  observation.doppler_bandwidth = doppler_bandwidth;
  observation.doppler_asymmetry = doppler_asymmetry;
  observation.respiration_power = respiration_power;
  observation.respiration_coherence = respiration_coherence;
  const float temporal_impulse = clamp(summary_impulse_ / divisor, 0.0F, 1.0F);
  observation.impulse_score =
      clamp(0.70F * temporal_impulse +
                0.30F * doppler_energy * doppler_bandwidth,
            0.0F, 1.0F);
  observation.stillness_score = clamp(1.0F - motion_ewma_ * 1.7F, 0.0F, 1.0F);
  observation.baseline_shift = clamp(summary_baseline_shift_ / divisor, 0.0F, 1.0F);
  observation.broadband_nuisance = broadband_ewma_;
  observation.quality =
      clamp((summary_quality_ / divisor) * (0.65F + 0.35F * observation.window_completeness),
            0.0F, 1.0F);
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
  summary_subcarrier_reliability_ = 0.0F;
  summary_snr_db_ = 0.0F;
  summary_delay_motion_ = 0.0F;
  summary_delay_spread_ = 0.0F;
  summary_dynamic_tap_concentration_ = 0.0F;
  summary_amplitude_projection_ = 0.0F;
  summary_phase_projection_ = 0.0F;
  return CsiPrecisionUpdateStatus::Updated;
}

void M5AtomCsiLinkProcessor::pushFast(float value, float complex_real,
                                      float complex_imaginary) {
  fast_history_[fast_head_] = value;
  fast_complex_real_history_[fast_head_] = complex_real;
  fast_complex_imaginary_history_[fast_head_] = complex_imaginary;
  fast_head_ = (fast_head_ + 1U) % kFastHistoryCapacity;
  if (fast_count_ < kFastHistoryCapacity) {
    ++fast_count_;
  }
}

float M5AtomCsiLinkProcessor::complexSpectralPower(float frequency_hz) const {
  if (fast_count_ < 16U || config_.target_rate_hz == 0U || frequency_hz == 0.0F) {
    return 0.0F;
  }
  float mean_real = 0.0F;
  float mean_imaginary = 0.0F;
  for (std::size_t index = 0; index < fast_count_; ++index) {
    mean_real += historyValue(fast_complex_real_history_, kFastHistoryCapacity, fast_head_,
                              fast_count_, index);
    mean_imaginary += historyValue(fast_complex_imaginary_history_, kFastHistoryCapacity,
                                   fast_head_, fast_count_, index);
  }
  mean_real /= static_cast<float>(fast_count_);
  mean_imaginary /= static_cast<float>(fast_count_);

  float spectrum_real = 0.0F;
  float spectrum_imaginary = 0.0F;
  float window_sum = 0.0F;
  for (std::size_t index = 0; index < fast_count_; ++index) {
    const float sample_real =
        historyValue(fast_complex_real_history_, kFastHistoryCapacity, fast_head_, fast_count_,
                     index) -
        mean_real;
    const float sample_imaginary =
        historyValue(fast_complex_imaginary_history_, kFastHistoryCapacity, fast_head_,
                     fast_count_, index) -
        mean_imaginary;
    const float phase =
        kTwoPi * frequency_hz * static_cast<float>(index) /
        static_cast<float>(config_.target_rate_hz);
    const float cosine = std::cos(phase);
    const float sine = std::sin(phase);
    const float window =
        fast_count_ > 1U
            ? 0.5F - 0.5F *
                         std::cos(kTwoPi * static_cast<float>(index) /
                                  static_cast<float>(fast_count_ - 1U))
            : 1.0F;
    spectrum_real += window * (sample_real * cosine + sample_imaginary * sine);
    spectrum_imaginary += window * (sample_imaginary * cosine - sample_real * sine);
    window_sum += window;
  }
  const float normalization = window_sum > 1.0e-6F ? 2.0F / window_sum : 0.0F;
  return (spectrum_real * spectrum_real + spectrum_imaginary * spectrum_imaginary) *
         normalization * normalization;
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
  const CsiLinkObservation *anchor = nullptr;
  for (const LinkSlot &slot : links_) {
    if (slot.occupied && now_us - slot.received_at_us <= config_.fusion_freshness_us &&
        slot.observation.tx_uptime_us > newest_tx_uptime) {
      newest_tx_uptime = slot.observation.tx_uptime_us;
      observation.anchor_probe_sequence = slot.observation.probe_sequence;
      anchor = &slot.observation;
    }
  }
  if (newest_tx_uptime == 0U || anchor == nullptr) {
    return false;
  }

  std::size_t indices[kMaximumM5AtomCsiReceivers]{};
  std::size_t count = 0;
  for (std::size_t index = 0; index < kMaximumM5AtomCsiReceivers; ++index) {
    const LinkSlot &slot = links_[index];
    if (!slot.occupied || now_us - slot.received_at_us > config_.fusion_freshness_us) {
      continue;
    }
    int32_t sequence_skew = static_cast<int32_t>(observation.anchor_probe_sequence -
                                                 slot.observation.probe_sequence);
    if (sequence_skew < 0) {
      sequence_skew = -sequence_skew;
    }
    if (sequence_skew > static_cast<int32_t>(config_.maximum_sequence_skew)) {
      continue;
    }
    const uint32_t overlap_start =
        anchor->window_start_probe_sequence > slot.observation.window_start_probe_sequence
            ? anchor->window_start_probe_sequence
            : slot.observation.window_start_probe_sequence;
    const uint32_t overlap_end =
        anchor->probe_sequence < slot.observation.probe_sequence
            ? anchor->probe_sequence
            : slot.observation.probe_sequence;
    const uint32_t anchor_span =
        anchor->probe_sequence - anchor->window_start_probe_sequence + 1U;
    const uint32_t candidate_span =
        slot.observation.probe_sequence - slot.observation.window_start_probe_sequence + 1U;
    const uint32_t shorter_span = anchor_span < candidate_span ? anchor_span : candidate_span;
    const uint32_t overlap_span = overlap_end >= overlap_start ? overlap_end - overlap_start + 1U : 0U;
    const float overlap = shorter_span > 0U
                              ? static_cast<float>(overlap_span) /
                                    static_cast<float>(shorter_span)
                              : 0.0F;
    if (overlap >= config_.minimum_window_overlap) {
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
  observation.subcarrier_reliability =
      robustAggregate(&CsiLinkObservation::subcarrier_reliability, indices, count);
  observation.delay_domain_motion =
      robustAggregate(&CsiLinkObservation::delay_domain_motion, indices, count);
  observation.delay_spread = robustAggregate(&CsiLinkObservation::delay_spread, indices, count);
  observation.dynamic_tap_concentration =
      robustAggregate(&CsiLinkObservation::dynamic_tap_concentration, indices, count);
  observation.doppler_energy = robustAggregate(&CsiLinkObservation::doppler_energy, indices, count);
  observation.doppler_centroid =
      robustAggregate(&CsiLinkObservation::doppler_centroid, indices, count);
  observation.doppler_bandwidth =
      robustAggregate(&CsiLinkObservation::doppler_bandwidth, indices, count);
  observation.doppler_asymmetry =
      robustAggregate(&CsiLinkObservation::doppler_asymmetry, indices, count);
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

  float synchronization_sum = 0.0F;
  float synchronization_weight = 0.0F;
  for (std::size_t position = 0; position < count; ++position) {
    const CsiLinkObservation &link = links_[indices[position]].observation;
    int32_t sequence_skew =
        static_cast<int32_t>(observation.anchor_probe_sequence - link.probe_sequence);
    if (sequence_skew < 0) {
      sequence_skew = -sequence_skew;
    }
    const float sequence_alignment =
        1.0F - clamp01(static_cast<float>(sequence_skew) /
                       static_cast<float>(config_.maximum_sequence_skew + 1U));
    synchronization_sum += link.quality * link.window_completeness * sequence_alignment;
    synchronization_weight += link.quality;
  }
  observation.synchronization_quality =
      clamp01(synchronization_weight > 1.0e-6F
                  ? synchronization_sum / synchronization_weight
                  : 0.0F);

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
  observation.quality =
      clamp01(base_quality * link_factor * (0.70F + 0.30F * observation.link_agreement) *
              (0.70F + 0.30F * observation.synchronization_quality));
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
    const float weight =
        link.quality * link.window_completeness / (1.0F + deviation * deviation);
    weighted_sum += weight * value;
    weight_sum += weight;
  }
  return clamp01(weight_sum > 1.0e-6F ? weighted_sum / weight_sum : center);
}

float M5AtomCsiFusion::clamp01(float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

}  // namespace atom::radar
