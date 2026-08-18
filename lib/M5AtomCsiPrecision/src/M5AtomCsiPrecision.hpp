#pragma once

#include <cstddef>
#include <cstdint>

#include <CsiFrameParser.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class M5AtomCsiLinkProcessor final {
 public:
  explicit M5AtomCsiLinkProcessor(M5AtomCsiPrecisionConfig config = {});

  void noteProbe(uint32_t sequence, uint64_t tx_uptime_us, int64_t received_at_us);
  CsiPrecisionUpdateStatus update(const ParsedCsiFrame &frame, uint8_t receiver_id,
                                  CsiLinkObservation &observation);
  void reset();

 private:
  static constexpr std::size_t kProbeHistoryCapacity = 16;
  static constexpr std::size_t kFastHistoryCapacity = 128;
  static constexpr std::size_t kSlowHistoryCapacity = 256;
  static constexpr std::size_t kDelayTapCount = 16;

  enum class ProbeMatchStatus : uint8_t {
    Matched = 0,
    Missing,
    NonMonotonic,
  };

  struct ProbeStamp {
    uint32_t sequence;
    uint64_t tx_uptime_us;
    int64_t received_at_us;
  };

  struct CarrierState {
    int16_t subcarrier;
    uint16_t baseline_count;
    float amplitude_mean;
    float amplitude_m2;
    float phase_noise_variance;
    float ratio_noise_variance;
    float previous_amplitude;
    float previous_phase;
    float previous_ratio_phase;
    float previous_ratio_amplitude;
    float principal_weight;
    float validity_ewma;
    float saturation_ewma;
    float continuity_ewma;
    bool previous_valid;
    bool previous_ratio_valid;
  };

  struct DelayTapState {
    uint16_t baseline_count;
    float baseline_real;
    float baseline_imaginary;
    float noise_variance;
    float previous_real;
    float previous_imaginary;
    bool previous_valid;
  };

  struct MatchedProbe {
    uint32_t sequence;
    uint64_t tx_uptime_us;
    uint16_t sequence_gap;
  };

  ProbeMatchStatus matchProbe(int64_t received_at_us, MatchedProbe &matched);
  void resetSignalState(const ParsedCsiFrame &frame);
  void pushFast(float value, float complex_real, float complex_imaginary);
  void pushSlow(float amplitude, float phase);
  float spectralPower(const float *history, std::size_t capacity, std::size_t head,
                      std::size_t count, float sample_rate_hz, float frequency_hz) const;
  float complexSpectralPower(float frequency_hz) const;
  float historyValue(const float *history, std::size_t capacity, std::size_t head,
                     std::size_t count, std::size_t chronological_index) const;
  static float clamp(float value, float lower, float upper);
  static float wrapPhase(float value);
  static float median(float *values, std::size_t count);

  M5AtomCsiPrecisionConfig config_;
  ProbeStamp probe_history_[kProbeHistoryCapacity]{};
  std::size_t probe_head_{0};
  std::size_t probe_count_{0};
  uint32_t last_matched_probe_{0};
  bool has_last_matched_probe_{false};

  CarrierState carriers_[kMaximumHtSubcarrierCount]{};
  DelayTapState delay_taps_[kDelayTapCount]{};
  uint16_t carrier_count_{0};
  uint8_t channel_{0};
  uint8_t bandwidth_mhz_{0};
  uint32_t baseline_frame_count_{0};
  uint32_t observation_sequence_{0};
  bool layout_ready_{false};

  float fast_history_[kFastHistoryCapacity]{};
  float fast_complex_real_history_[kFastHistoryCapacity]{};
  float fast_complex_imaginary_history_[kFastHistoryCapacity]{};
  std::size_t fast_head_{0};
  std::size_t fast_count_{0};
  float slow_amplitude_history_[kSlowHistoryCapacity]{};
  float slow_phase_history_[kSlowHistoryCapacity]{};
  std::size_t slow_head_{0};
  std::size_t slow_count_{0};

  uint16_t summary_frames_{0};
  uint32_t summary_epoch_{0};
  uint32_t summary_first_probe_sequence_{0};
  bool has_summary_epoch_{false};
  uint32_t summary_sequence_gaps_{0};
  float summary_amplitude_motion_{0.0F};
  float summary_phase_motion_{0.0F};
  float summary_ratio_motion_{0.0F};
  float summary_phase_coherence_{0.0F};
  float summary_impulse_{0.0F};
  float summary_baseline_shift_{0.0F};
  float summary_quality_{0.0F};
  float summary_valid_ratio_{0.0F};
  float summary_subcarrier_reliability_{0.0F};
  float summary_snr_db_{0.0F};
  float summary_delay_motion_{0.0F};
  float summary_delay_spread_{0.0F};
  float summary_dynamic_tap_concentration_{0.0F};
  float summary_amplitude_projection_{0.0F};
  float summary_phase_projection_{0.0F};

  float previous_projection_{0.0F};
  float motion_ewma_{0.0F};
  float impulse_peak_{0.0F};
  float baseline_shift_ewma_{0.0F};
  float broadband_ewma_{0.0F};
  bool has_previous_projection_{false};
};

class M5AtomCsiFusion final {
 public:
  explicit M5AtomCsiFusion(M5AtomCsiPrecisionConfig config = {});

  bool ingestPacket(const uint8_t *data, std::size_t length, uint32_t expected_system_id,
                    int64_t received_at_us);
  bool snapshot(int64_t now_us, FusedCsiObservation &observation) const;
  void reset();

 private:
  struct LinkSlot {
    CsiLinkObservation observation;
    int64_t received_at_us;
    bool occupied;
  };

  float robustAggregate(float CsiLinkObservation::*member, const std::size_t *indices,
                        std::size_t count) const;
  static float clamp01(float value);

  M5AtomCsiPrecisionConfig config_;
  LinkSlot links_[kMaximumM5AtomCsiReceivers]{};
};

}  // namespace atom::radar
