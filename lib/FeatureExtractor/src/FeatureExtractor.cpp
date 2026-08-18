#include "FeatureExtractor.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace atom::radar {

FeatureExtractor::FeatureExtractor(FeatureExtractorConfig config) : config_(config) {}

FeatureUpdateStatus FeatureExtractor::update(const PreprocessedCsiFrame &frame,
                                             const SubcarrierSelection &selection,
                                             DetectionFeatures &features) {
  features = {};
  if (selection.count < kMinimumSelectedSubcarriers ||
      selection.count > kMaximumSelectedSubcarriers) {
    return FeatureUpdateStatus::InvalidSelection;
  }
  if (!selectionMatches(selection)) {
    applySelection(selection);
  }
  if (last_input_us_ != 0 && frame.received_at_us <= last_input_us_) {
    return FeatureUpdateStatus::NonMonotonicTimestamp;
  }
  last_input_us_ = frame.received_at_us;

  float values[kMaximumSelectedSubcarriers]{};
  float deltas[kMaximumSelectedSubcarriers]{};
  float absolute_deltas[kMaximumSelectedSubcarriers]{};
  float absolute_values[kMaximumSelectedSubcarriers]{};
  bool current_valid[kMaximumSelectedSubcarriers]{};
  std::size_t valid_count = 0;
  std::size_t delta_count = 0;

  for (std::size_t selected_index = 0; selected_index < selected_count_; ++selected_index) {
    for (std::size_t sample_index = 0; sample_index < frame.sample_count; ++sample_index) {
      const PreprocessedCsiSample &sample = frame.samples[sample_index];
      if (sample.subcarrier != selected_subcarriers_[selected_index]) {
        continue;
      }
      if (sample.valid && std::isfinite(sample.robust_z)) {
        values[selected_index] = sample.robust_z;
        absolute_values[valid_count++] = std::fabs(sample.robust_z);
        current_valid[selected_index] = true;
        if (previous_valid_[selected_index]) {
          const float delta = sample.robust_z - previous_values_[selected_index];
          deltas[delta_count] = delta;
          absolute_deltas[delta_count++] = std::fabs(delta);
        }
      }
      break;
    }
  }

  if (valid_count == 0U) {
    std::memset(previous_valid_, 0, sizeof(previous_valid_));
    return FeatureUpdateStatus::InsufficientData;
  }

  HistoryPoint point{};
  point.timestamp_us = frame.received_at_us;
  point.valid_ratio = static_cast<float>(valid_count) / static_cast<float>(selected_count_);
  point.baseline_distance = arrayMedian(absolute_values, valid_count);

  float mean = 0.0F;
  for (std::size_t index = 0; index < selected_count_; ++index) {
    if (current_valid[index]) {
      mean += values[index];
    }
  }
  mean /= static_cast<float>(valid_count);
  for (std::size_t index = 0; index < selected_count_; ++index) {
    if (current_valid[index]) {
      const float centered = values[index] - mean;
      point.variance += centered * centered;
    }
  }
  point.variance /= static_cast<float>(valid_count);

  if (delta_count > 0U) {
    point.motion_energy = arrayMedian(absolute_deltas, delta_count);
    float motion_deviations[kMaximumSelectedSubcarriers]{};
    float signed_delta_mean = 0.0F;
    std::size_t changed_count = 0;
    for (std::size_t index = 0; index < delta_count; ++index) {
      motion_deviations[index] = std::fabs(absolute_deltas[index] - point.motion_energy);
      signed_delta_mean += deltas[index];
      if (absolute_deltas[index] >= config_.changed_subcarrier_threshold) {
        ++changed_count;
      }
    }
    point.motion_mad = arrayMedian(motion_deviations, delta_count);
    point.changed_ratio = static_cast<float>(changed_count) / static_cast<float>(delta_count);
    signed_delta_mean /= static_cast<float>(delta_count);

    std::size_t synchronized_count = 0;
    std::size_t high_frequency_count = 0;
    for (std::size_t index = 0; index < selected_count_; ++index) {
      if (!current_valid[index] || !previous_valid_[index]) {
        continue;
      }
      const float delta = values[index] - previous_values_[index];
      if ((delta >= 0.0F) == (signed_delta_mean >= 0.0F)) {
        ++synchronized_count;
      }
      point.high_frequency_energy += std::fabs(delta - previous_deltas_[index]);
      previous_deltas_[index] = delta;
      ++high_frequency_count;
    }
    point.synchronized_ratio =
        static_cast<float>(synchronized_count) / static_cast<float>(delta_count);
    if (high_frequency_count > 0U) {
      point.high_frequency_energy /= static_cast<float>(high_frequency_count);
    }
  }

  point.rssi_delta =
      has_previous_rssi_ ? std::fabs(static_cast<float>(frame.rssi - previous_rssi_)) : 0.0F;
  previous_rssi_ = frame.rssi;
  has_previous_rssi_ = true;
  for (std::size_t index = 0; index < selected_count_; ++index) {
    previous_values_[index] = values[index];
    previous_valid_[index] = current_valid[index];
  }

  const int64_t aggregation_interval_us = 1000000LL / config_.aggregation_rate_hz;
  if (last_history_us_ != 0 && frame.received_at_us - last_history_us_ < aggregation_interval_us) {
    return FeatureUpdateStatus::Collecting;
  }
  last_history_us_ = frame.received_at_us;
  pushHistory(point);
  buildFeatures(features);
  return FeatureUpdateStatus::Updated;
}

void FeatureExtractor::reset() {
  history_head_ = 0;
  history_count_ = 0;
  last_input_us_ = 0;
  last_history_us_ = 0;
  previous_rssi_ = 0;
  has_previous_rssi_ = false;
  std::memset(history_, 0, sizeof(history_));
  std::memset(previous_values_, 0, sizeof(previous_values_));
  std::memset(previous_deltas_, 0, sizeof(previous_deltas_));
  std::memset(previous_valid_, 0, sizeof(previous_valid_));
}

bool FeatureExtractor::selectionMatches(const SubcarrierSelection &selection) const {
  if (selected_count_ != selection.count) {
    return false;
  }
  for (std::size_t index = 0; index < selection.count; ++index) {
    if (selected_subcarriers_[index] != selection.subcarriers[index]) {
      return false;
    }
  }
  return true;
}

void FeatureExtractor::applySelection(const SubcarrierSelection &selection) {
  selected_count_ = selection.count;
  std::memcpy(selected_subcarriers_, selection.subcarriers,
              selected_count_ * sizeof(selected_subcarriers_[0]));
  reset();
}

void FeatureExtractor::pushHistory(const HistoryPoint &point) {
  history_[history_head_] = point;
  history_head_ = (history_head_ + 1U) % kFeatureHistoryCapacity;
  if (history_count_ < kFeatureHistoryCapacity) {
    ++history_count_;
  }
}

const FeatureExtractor::HistoryPoint &FeatureExtractor::historyFromNewest(std::size_t offset) const {
  const std::size_t index =
      (history_head_ + kFeatureHistoryCapacity - 1U - offset) % kFeatureHistoryCapacity;
  return history_[index];
}

float FeatureExtractor::historyAverage(std::size_t points, float HistoryPoint::*member) const {
  const std::size_t count = std::min(points, history_count_);
  if (count == 0U) {
    return 0.0F;
  }
  float total = 0.0F;
  for (std::size_t offset = 0; offset < count; ++offset) {
    total += historyFromNewest(offset).*member;
  }
  return total / static_cast<float>(count);
}

float FeatureExtractor::historyMedian(std::size_t points, float HistoryPoint::*member) const {
  const std::size_t count = std::min(points, history_count_);
  float values[kFeatureHistoryCapacity]{};
  for (std::size_t offset = 0; offset < count; ++offset) {
    values[offset] = historyFromNewest(offset).*member;
  }
  return count == 0U ? 0.0F : arrayMedian(values, count);
}

float FeatureExtractor::historyMad(std::size_t points, float HistoryPoint::*member,
                                   float center) const {
  const std::size_t count = std::min(points, history_count_);
  float deviations[kFeatureHistoryCapacity]{};
  for (std::size_t offset = 0; offset < count; ++offset) {
    deviations[offset] = std::fabs(historyFromNewest(offset).*member - center);
  }
  return count == 0U ? 0.0F : arrayMedian(deviations, count);
}

void FeatureExtractor::buildFeatures(DetectionFeatures &features) const {
  const std::size_t short_count = std::min<std::size_t>(config_.short_window_points, history_count_);
  const std::size_t medium_count =
      std::min<std::size_t>(config_.medium_window_points, history_count_);
  const std::size_t long_count = std::min<std::size_t>(config_.long_window_points, history_count_);
  const std::size_t quality_count =
      std::min<std::size_t>(config_.quality_window_points, history_count_);

  features.timestamp_us = historyFromNewest(0).timestamp_us;
  features.motion_energy_short =
      historyMedian(short_count, &HistoryPoint::motion_energy);
  features.motion_mad_short =
      historyMad(short_count, &HistoryPoint::motion_energy, features.motion_energy_short);
  features.variance_short = historyAverage(short_count, &HistoryPoint::variance);
  features.changed_subcarrier_ratio_short =
      historyAverage(short_count, &HistoryPoint::changed_ratio);
  features.synchronized_change_ratio_short =
      historyAverage(short_count, &HistoryPoint::synchronized_ratio);
  features.high_frequency_energy_short =
      historyAverage(short_count, &HistoryPoint::high_frequency_energy);
  features.baseline_distance_instant = historyFromNewest(0).baseline_distance;
  features.rssi_delta_short = historyAverage(short_count, &HistoryPoint::rssi_delta);
  features.covariance_change_proxy_short = features.variance_short;
  features.slow_motion_energy_medium =
      historyMedian(medium_count, &HistoryPoint::motion_energy);
  features.baseline_distance_long =
      historyMedian(long_count, &HistoryPoint::baseline_distance);
  features.quality_valid_ratio = historyAverage(quality_count, &HistoryPoint::valid_ratio);

  std::size_t persistence_count = 0;
  std::size_t biased_count = 0;
  float low_frequency_values[kFeatureHistoryCapacity]{};
  for (std::size_t offset = 0; offset < short_count; ++offset) {
    if (historyFromNewest(offset).motion_energy >= config_.motion_persistence_threshold) {
      ++persistence_count;
    }
  }
  features.motion_persistence_seconds =
      static_cast<float>(persistence_count) / static_cast<float>(config_.aggregation_rate_hz);

  for (std::size_t offset = 0; offset < long_count; ++offset) {
    if (historyFromNewest(offset).baseline_distance >= config_.persistent_bias_threshold) {
      ++biased_count;
    }
    if (offset + 1U < long_count) {
      low_frequency_values[offset] =
          std::fabs(historyFromNewest(offset).baseline_distance -
                    historyFromNewest(offset + 1U).baseline_distance);
    }
  }
  features.persistent_bias_ratio_long =
      long_count == 0U ? 0.0F : static_cast<float>(biased_count) / static_cast<float>(long_count);
  features.low_frequency_variation_long =
      long_count < 2U ? 0.0F : arrayMedian(low_frequency_values, long_count - 1U);

  if (long_count >= 2U) {
    const std::size_t half = long_count / 2U;
    float recent[kFeatureHistoryCapacity]{};
    float older[kFeatureHistoryCapacity]{};
    for (std::size_t index = 0; index < half; ++index) {
      recent[index] = historyFromNewest(index).baseline_distance;
      older[index] = historyFromNewest(index + half).baseline_distance;
    }
    features.long_median_shift =
        std::fabs(arrayMedian(recent, half) - arrayMedian(older, half));
  }

  features.short_window_ready = history_count_ >= config_.short_window_points;
  features.medium_window_ready = history_count_ >= config_.medium_window_points;
  features.long_window_ready = history_count_ >= config_.long_window_points;
  features.quality_window_ready = history_count_ >= config_.quality_window_points;
}

float FeatureExtractor::arrayMedian(float *values, std::size_t count) {
  std::sort(values, values + count);
  const std::size_t middle = count / 2U;
  return (count % 2U) != 0U ? values[middle]
                            : (values[middle - 1U] + values[middle]) * 0.5F;
}

}  // namespace atom::radar
