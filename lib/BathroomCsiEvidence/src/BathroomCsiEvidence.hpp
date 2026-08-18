#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_csi_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomCsiEvidenceAnalyzer final {
 public:
  explicit BathroomCsiEvidenceAnalyzer(BathroomCsiEvidenceConfig config = {});

  BathroomCsiEvidenceUpdateStatus update(const FusedCsiObservation &observation,
                                         int64_t observed_at_us,
                                         BathroomCsiEvidence &evidence);
  void reset();

 private:
  static constexpr std::size_t kHistoryCapacity = 300;

  struct EvidencePoint {
    uint32_t probe_sequence;
    int64_t observed_at_us;
    float quality;
    float link_agreement;
    float synchronization;
    float amplitude_motion;
    float phase_coherence;
    float doppler_energy;
    float doppler_centroid;
    float doppler_bandwidth;
    float doppler_asymmetry;
    float delay_motion;
    float delay_spread;
    float dynamic_tap_concentration;
    float baseline_shift;
    float broadband_nuisance;
    float background_explained;
    float innovation_motion;
    float impulse;
    float stillness;
  };

  const EvidencePoint &fromNewest(std::size_t offset) const;
  float average(std::size_t points, float EvidencePoint::*member) const;
  float rangeAverage(std::size_t newest_offset, std::size_t points,
                     float EvidencePoint::*member) const;
  float variance(std::size_t points, float EvidencePoint::*member, float center) const;
  float maximum(std::size_t points, float EvidencePoint::*member) const;
  float persistence(std::size_t points, float EvidencePoint::*member, float threshold) const;
  float trend(std::size_t points, float EvidencePoint::*member) const;
  void buildEvidence(BathroomCsiEvidence &evidence) const;
  static float clamp01(float value);
  static float maximumOfFour(float first, float second, float third, float fourth);

  BathroomCsiEvidenceConfig config_;
  EvidencePoint history_[kHistoryCapacity]{};
  std::size_t history_head_{0};
  std::size_t history_count_{0};
};

}  // namespace atom::radar
