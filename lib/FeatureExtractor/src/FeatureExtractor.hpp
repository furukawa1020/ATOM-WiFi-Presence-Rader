#pragma once

#include <cstddef>
#include <cstdint>

#include <CsiPreprocessor.hpp>
#include <SubcarrierSelector.hpp>

namespace atom::radar {

constexpr std::size_t kFeatureHistoryCapacity = 300;

struct FeatureExtractorConfig {
  uint16_t aggregation_rate_hz{10};
  uint16_t short_window_points{10};
  uint16_t medium_window_points{40};
  uint16_t long_window_points{200};
  uint16_t quality_window_points{50};
  float changed_subcarrier_threshold{1.5F};
  float motion_persistence_threshold{0.75F};
  float persistent_bias_threshold{1.5F};
};

struct DetectionFeatures {
  int64_t timestamp_us;
  float motion_energy_short;
  float motion_mad_short;
  float variance_short;
  float changed_subcarrier_ratio_short;
  float synchronized_change_ratio_short;
  float high_frequency_energy_short;
  float motion_persistence_seconds;
  float baseline_distance_instant;
  float rssi_delta_short;
  float covariance_change_proxy_short;
  float slow_motion_energy_medium;
  float baseline_distance_long;
  float long_median_shift;
  float persistent_bias_ratio_long;
  float low_frequency_variation_long;
  float quality_valid_ratio;
  bool short_window_ready;
  bool medium_window_ready;
  bool long_window_ready;
  bool quality_window_ready;
};

enum class FeatureUpdateStatus : uint8_t {
  Updated = 0,
  Collecting,
  InvalidSelection,
  InsufficientData,
  NonMonotonicTimestamp,
};

class FeatureExtractor final {
 public:
  explicit FeatureExtractor(FeatureExtractorConfig config = {});

  FeatureUpdateStatus update(const PreprocessedCsiFrame &frame,
                             const SubcarrierSelection &selection, DetectionFeatures &features);
  void reset();

 private:
  struct HistoryPoint {
    int64_t timestamp_us;
    float motion_energy;
    float motion_mad;
    float variance;
    float changed_ratio;
    float synchronized_ratio;
    float high_frequency_energy;
    float baseline_distance;
    float rssi_delta;
    float valid_ratio;
  };

  bool selectionMatches(const SubcarrierSelection &selection) const;
  void applySelection(const SubcarrierSelection &selection);
  void pushHistory(const HistoryPoint &point);
  const HistoryPoint &historyFromNewest(std::size_t offset) const;
  float historyAverage(std::size_t points, float HistoryPoint::*member) const;
  float historyMedian(std::size_t points, float HistoryPoint::*member) const;
  float historyMad(std::size_t points, float HistoryPoint::*member, float center) const;
  void buildFeatures(DetectionFeatures &features) const;
  static float arrayMedian(float *values, std::size_t count);

  FeatureExtractorConfig config_;
  HistoryPoint history_[kFeatureHistoryCapacity]{};
  std::size_t history_head_{0};
  std::size_t history_count_{0};
  int16_t selected_subcarriers_[kMaximumSelectedSubcarriers]{};
  float previous_values_[kMaximumSelectedSubcarriers]{};
  float previous_deltas_[kMaximumSelectedSubcarriers]{};
  bool previous_valid_[kMaximumSelectedSubcarriers]{};
  uint16_t selected_count_{0};
  int64_t last_input_us_{0};
  int64_t last_history_us_{0};
  int8_t previous_rssi_{0};
  bool has_previous_rssi_{false};
};

}  // namespace atom::radar
