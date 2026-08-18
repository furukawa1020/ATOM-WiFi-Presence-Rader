#pragma once

#include <cstdint>

#include <FeatureExtractor.hpp>
#include <LogisticClassifier.hpp>

namespace atom::radar {

struct LocalDetectionResult {
  float motion_probability;
  float presence_probability;
  float anomaly_score;
  float baseline_distance;
  float model_confidence;
  bool motion_active;
  bool presence_active;
  bool baseline_valid;
  bool methods_disagree;
};

struct LocalDetectorConfig {
  float statistic_weight{0.55F};
  float classifier_weight{0.45F};
  float disagreement_threshold{0.40F};
  float motion_active_threshold{0.65F};
  float presence_active_threshold{0.65F};
  float motion_statistic_bias{1.25F};
  float presence_statistic_bias{1.25F};
};

class LocalDetector final {
 public:
  explicit LocalDetector(LocalDetectorConfig config = {});

  LocalDetectionResult evaluate(const DetectionFeatures &features, bool baseline_valid,
                                const LogisticModel *motion_model,
                                const LogisticModel *presence_model) const;

 private:
  static float sigmoid(float value);
  static float clamp01(float value);

  LocalDetectorConfig config_;
  LogisticClassifier classifier_;
};

}  // namespace atom::radar
