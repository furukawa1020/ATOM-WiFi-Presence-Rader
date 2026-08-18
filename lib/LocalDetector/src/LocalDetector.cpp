#include "LocalDetector.hpp"

#include <algorithm>
#include <cmath>

namespace atom::radar {

LocalDetector::LocalDetector(LocalDetectorConfig config) : config_(config) {}

LocalDetectionResult LocalDetector::evaluate(const DetectionFeatures &features, bool baseline_valid,
                                             const LogisticModel *motion_model,
                                             const LogisticModel *presence_model) const {
  const float motion_statistic_logit =
      (0.75F * features.motion_energy_short) + (0.35F * features.motion_mad_short) +
      (0.20F * features.variance_short) +
      (1.10F * features.changed_subcarrier_ratio_short) +
      (0.45F * features.synchronized_change_ratio_short) +
      (0.20F * features.high_frequency_energy_short) +
      (0.30F * features.slow_motion_energy_medium) - config_.motion_statistic_bias;
  const float presence_statistic_logit =
      (0.55F * features.baseline_distance_instant) +
      (0.65F * features.baseline_distance_long) + (0.35F * features.long_median_shift) +
      (1.20F * features.persistent_bias_ratio_long) +
      (0.25F * features.low_frequency_variation_long) - config_.presence_statistic_bias;

  const float motion_statistic = sigmoid(motion_statistic_logit);
  const float presence_statistic = sigmoid(presence_statistic_logit);

  const float motion_features[] = {
      features.motion_energy_short,
      features.motion_mad_short,
      features.variance_short,
      features.changed_subcarrier_ratio_short,
      features.synchronized_change_ratio_short,
      features.high_frequency_energy_short,
      features.motion_persistence_seconds,
      features.rssi_delta_short,
      features.covariance_change_proxy_short,
      features.slow_motion_energy_medium,
  };
  const float presence_features[] = {
      features.baseline_distance_instant,
      features.baseline_distance_long,
      features.long_median_shift,
      features.persistent_bias_ratio_long,
      features.low_frequency_variation_long,
      features.quality_valid_ratio,
      features.slow_motion_energy_medium,
      features.motion_energy_short,
      features.variance_short,
      features.changed_subcarrier_ratio_short,
  };

  const LogisticPrediction motion_prediction =
      motion_model == nullptr
          ? LogisticPrediction{0.5F, 0.0F, false}
          : classifier_.predict(*motion_model, motion_features,
                                sizeof(motion_features) / sizeof(motion_features[0]));
  const LogisticPrediction presence_prediction =
      presence_model == nullptr
          ? LogisticPrediction{0.5F, 0.0F, false}
          : classifier_.predict(*presence_model, presence_features,
                                sizeof(presence_features) / sizeof(presence_features[0]));

  const auto combine = [this](float statistic, const LogisticPrediction &prediction) {
    if (!prediction.valid) {
      return statistic;
    }
    const float total_weight = config_.statistic_weight + config_.classifier_weight;
    return total_weight <= 0.0F
               ? statistic
               : ((config_.statistic_weight * statistic) +
                  (config_.classifier_weight * prediction.probability)) /
                     total_weight;
  };

  LocalDetectionResult result{};
  result.motion_probability = clamp01(combine(motion_statistic, motion_prediction));
  result.presence_probability = clamp01(combine(presence_statistic, presence_prediction));
  result.baseline_distance = features.baseline_distance_long;
  result.anomaly_score = clamp01(
      (0.45F * (1.0F - clamp01(features.quality_valid_ratio))) +
      (0.20F * clamp01(features.rssi_delta_short / 12.0F)) +
      (0.20F * clamp01(features.high_frequency_energy_short / 6.0F)) +
      (0.15F * clamp01(features.long_median_shift / 4.0F)));
  result.model_confidence =
      0.5F * (motion_prediction.confidence + presence_prediction.confidence);
  result.baseline_valid = baseline_valid;
  result.methods_disagree =
      (motion_prediction.valid &&
       std::fabs(motion_prediction.probability - motion_statistic) >=
           config_.disagreement_threshold) ||
      (presence_prediction.valid &&
       std::fabs(presence_prediction.probability - presence_statistic) >=
           config_.disagreement_threshold);
  result.motion_active = result.motion_probability >= config_.motion_active_threshold;
  result.presence_active = result.presence_probability >= config_.presence_active_threshold;

  if (!std::isfinite(result.motion_probability) || !std::isfinite(result.presence_probability) ||
      !std::isfinite(result.anomaly_score) || !std::isfinite(result.baseline_distance)) {
    result = {0.5F, 0.5F, 1.0F, 0.0F, 0.0F, false, false, baseline_valid, true};
  }
  return result;
}

float LocalDetector::sigmoid(float value) {
  return 1.0F / (1.0F + std::exp(-std::clamp(value, -16.0F, 16.0F)));
}

float LocalDetector::clamp01(float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

}  // namespace atom::radar
