#pragma once

#include <cstddef>
#include <cstdint>

#include <bathroom_csi_evidence_types.hpp>
#include <bathroom_safety_evidence_types.hpp>
#include <m5atom_csi_precision_types.hpp>

namespace atom::radar {

class BathroomSafetyEvidenceAnalyzer final {
 public:
  explicit BathroomSafetyEvidenceAnalyzer(BathroomSafetyEvidenceConfig config = {});

  BathroomSafetyEvidenceUpdateStatus update(
      const FusedCsiObservation &observation, int64_t observed_at_us,
      const BathroomCsiEvidence &bathroom_evidence,
      BathroomSafetyEvidence &evidence);
  void reset();

 private:
  static constexpr std::size_t kHistoryCapacity = 300;

  struct EvidencePoint {
    uint32_t probe_sequence;
    int64_t observed_at_us;
    float impact;
    float human_motion;
    float stillness;
    float baseline_shift;
    float delay_motion;
    float delay_spread;
    float dynamic_tap_concentration;
    float respiration;
    float door_alternative;
    float persistent_nuisance;
    float quality;
  };

  const EvidencePoint &fromNewest(std::size_t offset) const;
  void buildEvidence(BathroomSafetyEvidence &evidence) const;
  void emitJsonIfDue(const BathroomSafetyEvidence &evidence);
  static float clamp01(float value);
  static float maximum(float first, float second);
  static float mean(float sum, std::size_t count);

  BathroomSafetyEvidenceConfig config_;
  EvidencePoint history_[kHistoryCapacity]{};
  std::size_t history_head_{0};
  std::size_t history_count_{0};
  int64_t last_emit_at_us_{-1};
};

}  // namespace atom::radar
