#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_csi_evidence_types.hpp>
#include <bathroom_safety_evidence_types.hpp>
#include <bathroom_spatial_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomSpatialEvidenceAnalyzer final {
 public:
  explicit BathroomSpatialEvidenceAnalyzer(BathroomSpatialEvidenceConfig config = {});

  BathroomSpatialPacketStatus ingestPacket(const uint8_t *data, std::size_t length,
                                            uint32_t expected_system_id,
                                            int64_t received_at_us);
  BathroomSpatialEvidenceUpdateStatus update(
      const FusedCsiObservation &observation, int64_t observed_at_us,
      const BathroomCsiEvidence &bathroom_evidence,
      const BathroomSafetyEvidence &safety_evidence,
      BathroomSpatialEvidence &spatial_evidence);
  void reset();

 private:
  struct LinkState {
    bool valid;
    bool baseline_initialized;
    uint8_t receiver_id;
    uint32_t probe_sequence;
    uint32_t last_scored_probe_sequence;
    int64_t received_at_us;
    uint16_t baseline_samples;
    float snr_db;
    float subcarrier_reliability;
    float differential_phase_motion;
    float complex_ratio_motion;
    float phase_coherence;
    float delay_motion;
    float delay_spread;
    float dynamic_tap_concentration;
    float background_explained;
    float innovation_motion;
    float doppler_energy;
    float doppler_bandwidth;
    float baseline_shift;
    float broadband_nuisance;
    float receiver_baseline_maturity;
    float quality;
    float baseline_snr_db;
    float baseline_delay_spread;
    float baseline_tap_concentration;
    float response_ema;
  };

  static constexpr std::size_t kMaximumLinks = kMaximumM5AtomCsiReceivers;
  static constexpr std::size_t kZoneCount = 3;
  static constexpr std::size_t kPostureCount = 5;

  LinkState *findOrCreateLink(uint8_t receiver_id);
  float metric(const CsiObservationPacket &packet, CsiObservationMetric index) const;
  float instantaneousResponse(const LinkState &link) const;
  int roleIndex(uint8_t receiver_id) const;
  void emitJsonIfDue(const BathroomSpatialEvidence &evidence);
  static float clamp01(float value);
  static float maximum(float first, float second);
  static float absolute(float value);

  BathroomSpatialEvidenceConfig config_;
  LinkState links_[kMaximumLinks]{};
  float zone_probability_[kZoneCount]{1.0F / 3.0F, 1.0F / 3.0F, 1.0F / 3.0F};
  float zone_before_probe_[kZoneCount]{1.0F / 3.0F, 1.0F / 3.0F, 1.0F / 3.0F};
  float posture_probability_[kPostureCount]{};
  float posture_before_probe_[kPostureCount]{};
  bool has_evaluated_probe_{false};
  uint32_t last_evaluated_probe_{0};
  int64_t last_emit_at_us_{-1};
};

}  // namespace atom::radar
