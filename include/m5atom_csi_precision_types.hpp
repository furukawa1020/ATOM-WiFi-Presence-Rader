#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

#include <protocol.hpp>

namespace atom::radar {

constexpr std::size_t kMaximumM5AtomCsiReceivers = 3;
constexpr std::size_t kCsiObservationMetricCount = 27;
constexpr uint8_t kCsiObservationPacketType = 0x43U;

enum class CsiPrecisionUpdateStatus : uint8_t {
  Updated = 0,
  Collecting,
  CollectingBaseline,
  UnmatchedProbe,
  NonMonotonicProbe,
  InvalidFrame,
};

struct M5AtomCsiPrecisionConfig {
  uint16_t target_rate_hz{100};
  uint16_t summary_rate_hz{10};
  uint16_t baseline_min_frames{500};
  uint16_t respiration_min_summaries{192};
  int64_t probe_match_tolerance_us{6500};
  int64_t fusion_freshness_us{500000};
  int64_t fusion_alignment_us{250000};
  uint8_t maximum_sequence_skew{2};
  float minimum_window_overlap{0.60F};
  float amplitude_std_floor{0.025F};
  float phase_noise_floor_rad{0.035F};
  float ratio_noise_floor{0.040F};
  float quiet_motion_limit{0.22F};
  float baseline_adaptation_alpha{0.0015F};
  float noise_adaptation_alpha{0.010F};
  float reliability_adaptation_alpha{0.020F};
  float delay_noise_floor{0.020F};
};

struct CsiLinkObservation {
  uint8_t receiver_id;
  uint8_t channel;
  uint8_t bandwidth_mhz;
  uint32_t probe_sequence;
  uint32_t window_start_probe_sequence;
  uint32_t observation_sequence;
  uint64_t tx_uptime_us;
  uint16_t frames_in_summary;
  uint16_t expected_frames_in_summary;
  uint16_t sequence_gap_count;
  float snr_db;
  float valid_ratio;
  float window_completeness;
  float subcarrier_reliability;
  float baseline_maturity;
  float amplitude_motion;
  float differential_phase_motion;
  float complex_ratio_motion;
  float phase_coherence;
  float delay_domain_motion;
  float delay_spread;
  float dynamic_tap_concentration;
  float background_explained_ratio;
  float innovation_motion;
  float doppler_energy;
  float doppler_centroid;
  float doppler_bandwidth;
  float doppler_asymmetry;
  float respiration_power;
  float respiration_rate_normalized;
  float respiration_spectral_snr;
  float respiration_harmonicity;
  float respiration_coherence;
  float impulse_score;
  float stillness_score;
  float baseline_shift;
  float broadband_nuisance;
  float quality;
  bool baseline_ready;
};

struct FusedCsiObservation {
  uint32_t anchor_probe_sequence;
  uint64_t anchor_tx_uptime_us;
  uint8_t active_links;
  float amplitude_motion;
  float differential_phase_motion;
  float complex_ratio_motion;
  float phase_coherence;
  float subcarrier_reliability;
  float delay_domain_motion;
  float delay_spread;
  float dynamic_tap_concentration;
  float background_explained_ratio;
  float innovation_motion;
  float doppler_energy;
  float doppler_centroid;
  float doppler_bandwidth;
  float doppler_asymmetry;
  float respiration_power;
  float respiration_rate_normalized;
  float respiration_spectral_snr;
  float respiration_harmonicity;
  float respiration_coherence;
  float respiration_rate_agreement;
  float impulse_score;
  float stillness_score;
  float baseline_shift;
  float broadband_nuisance;
  float link_agreement;
  float synchronization_quality;
  float quality;
  bool physically_observable;
};

enum class CsiObservationMetric : std::size_t {
  ValidRatio = 0,
  WindowCompleteness,
  SubcarrierReliability,
  BaselineMaturity,
  AmplitudeMotion,
  DifferentialPhaseMotion,
  ComplexRatioMotion,
  PhaseCoherence,
  DelayDomainMotion,
  DelaySpread,
  DynamicTapConcentration,
  BackgroundExplainedRatio,
  InnovationMotion,
  DopplerEnergy,
  DopplerCentroid,
  DopplerBandwidth,
  DopplerAsymmetry,
  RespirationPower,
  RespirationRateNormalized,
  RespirationSpectralSnr,
  RespirationHarmonicity,
  RespirationCoherence,
  ImpulseScore,
  StillnessScore,
  BaselineShift,
  BroadbandNuisance,
  Quality,
};

#pragma pack(push, 1)
struct CsiObservationPacket {
  uint32_t magic;
  uint16_t protocol_version;
  uint16_t payload_length;
  uint32_t system_id;
  uint8_t packet_type;
  uint8_t receiver_id;
  uint8_t channel;
  uint8_t bandwidth_mhz;
  uint32_t probe_sequence;
  uint32_t window_start_probe_sequence;
  uint64_t tx_uptime_us;
  uint32_t observation_sequence;
  uint16_t frames_in_summary;
  uint16_t expected_frames_in_summary;
  uint16_t sequence_gap_count;
  int16_t snr_db_x10;
  uint8_t flags;
  uint8_t reserved;
  uint16_t metrics[kCsiObservationMetricCount];
  uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(CsiObservationPacket) <= protocol::kMaximumPacketBytes,
              "CSI observation exceeds ESP-NOW transport limit");
static_assert(std::is_trivially_copyable_v<CsiObservationPacket>,
              "CSI observation packet must remain trivially copyable");

CsiObservationPacket makeCsiObservationPacket(uint32_t system_id,
                                              const CsiLinkObservation &observation);
bool decodeCsiObservationPacket(const uint8_t *data, std::size_t length,
                                CsiObservationPacket &packet);

}  // namespace atom::radar
