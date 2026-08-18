#include "LogisticClassifier.hpp"

#include <algorithm>
#include <cmath>

namespace atom::radar {

LogisticPrediction LogisticClassifier::predict(const LogisticModel &model, const float *features,
                                                std::size_t feature_count) const {
  LogisticPrediction prediction{0.5F, 0.0F, false};
  if (!model.trained || model.model_version != APP_MODEL_VERSION || features == nullptr ||
      feature_count == 0U || feature_count != model.feature_count ||
      feature_count > kMaximumClassifierFeatures || !std::isfinite(model.bias) ||
      !std::isfinite(model.l2_strength) || model.l2_strength < 0.0F) {
    return prediction;
  }

  float logit = model.bias;
  for (std::size_t index = 0; index < feature_count; ++index) {
    if (!std::isfinite(features[index]) || !std::isfinite(model.weights[index]) ||
        !std::isfinite(model.feature_mean[index]) || !std::isfinite(model.feature_scale[index]) ||
        model.feature_scale[index] <= 0.0F) {
      return prediction;
    }
    const float normalized =
        (features[index] - model.feature_mean[index]) / model.feature_scale[index];
    logit += normalized * model.weights[index];
  }

  if (!std::isfinite(logit)) {
    return prediction;
  }
  logit = std::clamp(logit, -16.0F, 16.0F);
  prediction.probability = sigmoid(logit);
  prediction.confidence =
      std::fabs((prediction.probability * 2.0F) - 1.0F) / (1.0F + model.l2_strength);
  prediction.valid = std::isfinite(prediction.probability) && std::isfinite(prediction.confidence);
  return prediction;
}

float LogisticClassifier::sigmoid(float value) { return 1.0F / (1.0F + std::exp(-value)); }

}  // namespace atom::radar
