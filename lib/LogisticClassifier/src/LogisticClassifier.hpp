#pragma once

#include <cstddef>
#include <cstdint>

#ifndef APP_MODEL_VERSION
#define APP_MODEL_VERSION 1
#endif

namespace atom::radar {

constexpr std::size_t kMaximumClassifierFeatures = 16;

struct LogisticModel {
  uint16_t model_version;
  uint16_t feature_count;
  float bias;
  float weights[kMaximumClassifierFeatures];
  float feature_mean[kMaximumClassifierFeatures];
  float feature_scale[kMaximumClassifierFeatures];
  float l2_strength;
  bool trained;
};

struct LogisticPrediction {
  float probability;
  float confidence;
  bool valid;
};

class LogisticClassifier final {
 public:
  LogisticPrediction predict(const LogisticModel &model, const float *features,
                             std::size_t feature_count) const;

 private:
  static float sigmoid(float value);
};

}  // namespace atom::radar
