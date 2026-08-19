#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_csi_evidence_types.hpp>
#include <bathroom_respiration_evidence_types.hpp>
#include <bathroom_safety_evidence_types.hpp>
#include <bathroom_spatial_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomRespirationEvidenceAnalyzer final {
 public:
  explicit BathroomRespirationEvidenceAnalyzer(
      BathroomRespirationEvidenceConfig config = {});

  BathroomRespirationPacketStatus ingestPacket(const uint8_t *data,
                                                std::size_t length,
                                                uint32_t expected_system_id,
                                                int64_t received_at_us);
  BathroomRespirationEvidenceUpdateStatus update(
      const FusedCsiObservation &observation, int64_t observed_at_us,
      const BathroomCsiEvidence &bathroom_evidence,
      const BathroomSafetyEvidence &safety_evidence,
      const BathroomSpatialEvidence &spatial_evidence,
      BathroomRespirationEvidence &respiration_evidence);
  void reset();

 private:
  static constexpr std::size_t kMaximumLinks = kMaximumM5AtomCsiReceivers;
  static constexpr std::size_t kHistoryCapacity = 300;

  struct LinkState {
    bool allocated;
    bool has_packet;
    uint8_t receiver_id;
    uint32_t probe_sequence;
    int64_t received_at_us;
    float power;
    float rate_normalized;
    float spectral_snr;
    float harmonicity;
    float coherence;
    float subcarrier_reliability;
    float quality;
  };

  struct HistoryPoint {
    uint32_t probe_sequence;
    int64_t observed_at_us;
    float rate_normalized;
    float confidence;
    float power;
    float periodic_nuisance;
    float measurement_loss;
    float multi_link_support;
    float position_robustness;
    float quality;
  };

  LinkState *findOrCreateLink(uint8_t receiver_id);
  const HistoryPoint &fromNewest(std::size_t offset) const;
  float metric(const CsiObservationPacket &packet,
               CsiObservationMetric index) const;
  bool trackedRateBeforeCurrent(bool replacing_latest, float &rate) const;
  void buildEvidence(BathroomRespirationEvidence &evidence) const;
  void emitJsonIfDue(const BathroomRespirationEvidence &evidence);
  static float clamp01(float value);
  static float maximum(float first, float second);
  static float absolute(float value);

  BathroomRespirationEvidenceConfig config_;
  LinkState links_[kMaximumLinks]{};
  HistoryPoint history_[kHistoryCapacity]{};
  std::size_t history_head_{0};
  std::size_t history_count_{0};
  int64_t last_emit_at_us_{-1};
};

}  // namespace atom::radar
