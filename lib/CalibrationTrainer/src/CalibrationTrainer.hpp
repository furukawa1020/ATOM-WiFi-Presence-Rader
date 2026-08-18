#pragma once

#include <stdint.h>

#include "calibration_model_types.hpp"

struct CalibrationTrainerConfig {
  uint16_t max_samples = 512;
  uint16_t epochs = 300;
  uint16_t min_samples_per_partition = 8;
  float learning_rate = 0.05F;
  float l2_regularization = 0.001F;
  float min_feature_scale = 0.001F;
  float threshold_min = 0.10F;
  float threshold_max = 0.90F;
  float threshold_step = 0.05F;
};

class CalibrationTrainer {
 public:
  static constexpr uint8_t kMaxRuns = 24;
  static constexpr uint16_t kMaxSamples = 512;

  CalibrationTrainer();
  explicit CalibrationTrainer(const CalibrationTrainerConfig& config);

  void configure(const CalibrationTrainerConfig& config);
  void reset();

  CalibrationTrainingStatus bindFeatureSchema(
      const CalibrationFeatureSchema& schema);
  CalibrationTrainingStatus registerRun(
      const CalibrationRunDescriptor& descriptor);
  CalibrationTrainingStatus appendSample(
      uint32_t session_id,
      const float* features,
      uint8_t feature_count,
      CalibrationBinaryTarget occupied_target,
      CalibrationBinaryTarget motion_target);
  CalibrationTrainingStatus train(CalibratedDetectionModel& output);

  const CalibrationTrainingProgress& progress() const;
  uint8_t runCount() const;
  bool runSummary(uint8_t index, CalibrationRunSummary& output) const;

  static bool standardTargets(
      CalibrationLabel label,
      CalibrationBinaryTarget& occupied_target,
      CalibrationBinaryTarget& motion_target);

 private:
  struct FeatureSample {
    uint8_t run_index = 0;
    CalibrationBinaryTarget occupied_target =
        CalibrationBinaryTarget::Ignore;
    CalibrationBinaryTarget motion_target =
        CalibrationBinaryTarget::Ignore;
    float values[kCalibrationMaxFeatures]{};
  };

  struct TargetCounts {
    uint16_t occupied_positive = 0;
    uint16_t occupied_negative = 0;
    uint16_t motion_positive = 0;
    uint16_t motion_negative = 0;
    uint16_t samples = 0;
  };

  int8_t findRun(uint32_t session_id) const;
  static bool isValidPartition(CalibrationPartition partition);
  static bool isValidTarget(CalibrationBinaryTarget target);
  static bool targetsMatchLabel(
      CalibrationLabel label,
      CalibrationBinaryTarget occupied_target,
      CalibrationBinaryTarget motion_target);
  TargetCounts countTargets(CalibrationPartition partition) const;
  bool hasCoverage(const TargetCounts& counts) const;
  void updateRunCounts(
      CalibrationRunSummary& run,
      CalibrationBinaryTarget occupied_target,
      CalibrationBinaryTarget motion_target);
  bool computeNormalization(CalibratedDetectionModel& output) const;
  bool fitHead(CalibratedDetectionModel& output, bool motion_head) const;
  bool selectThresholds(CalibratedDetectionModel& output) const;
  CalibrationMetrics evaluate(
      CalibrationPartition partition,
      const CalibratedDetectionModel& model,
      float occupied_threshold,
      float motion_threshold) const;
  float predict(
      const FeatureSample& sample,
      const CalibratedDetectionModel& model,
      const CalibrationLogisticHead& head) const;
  static float sigmoid(float value);
  static float safeRate(uint16_t numerator, uint16_t denominator);

  CalibrationTrainerConfig config_{};
  CalibrationTrainingProgress progress_{};
  CalibrationRunSummary runs_[kMaxRuns]{};
  FeatureSample samples_[kMaxSamples]{};
  uint8_t run_count_ = 0;
  uint16_t sample_count_ = 0;
  uint8_t feature_count_ = 0;
  CalibrationFeatureSchema feature_schema_{};
};
