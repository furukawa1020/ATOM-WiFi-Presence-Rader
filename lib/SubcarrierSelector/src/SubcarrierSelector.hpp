#pragma once

#include <cstddef>
#include <cstdint>

#include <CsiFrameParser.hpp>

namespace atom::radar {

constexpr std::size_t kMinimumSelectedSubcarriers = 24;
constexpr std::size_t kDefaultSelectedSubcarriers = 32;
constexpr std::size_t kMaximumSelectedSubcarriers = 48;

struct SubcarrierTrainingMetrics {
  int16_t subcarrier;
  float empty_mad;
  float still_separation;
  float motion_separation;
  float missing_ratio;
  float outlier_ratio;
  float temporal_noise;
  float validation_reproducibility;
  float maximum_correlation;
  bool eligible;
};

struct SubcarrierSelection {
  uint8_t receiver_id;
  uint16_t count;
  int16_t subcarriers[kMaximumSelectedSubcarriers];
  float scores[kMaximumSelectedSubcarriers];
};

struct SubcarrierSelectorConfig {
  uint16_t target_count{kDefaultSelectedSubcarriers};
  uint16_t minimum_count{kMinimumSelectedSubcarriers};
  uint16_t maximum_count{kMaximumSelectedSubcarriers};
  float maximum_missing_ratio{0.20F};
  float maximum_outlier_ratio{0.15F};
  float minimum_reproducibility{0.50F};
};

enum class SubcarrierSelectionStatus : uint8_t {
  Ok = 0,
  InvalidConfiguration,
  InvalidMetrics,
  InsufficientCandidates,
};

class SubcarrierSelector final {
 public:
  explicit SubcarrierSelector(SubcarrierSelectorConfig config = {});

  SubcarrierSelectionStatus select(uint8_t receiver_id, const SubcarrierTrainingMetrics *metrics,
                                   std::size_t metrics_count,
                                   SubcarrierSelection &selection) const;

 private:
  float score(const SubcarrierTrainingMetrics &metrics) const;
  static float clamp01(float value);

  SubcarrierSelectorConfig config_;
};

}  // namespace atom::radar
