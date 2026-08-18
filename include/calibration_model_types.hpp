#pragma once

#include <stdint.h>

#include "calibration_types.hpp"

constexpr uint8_t kCalibrationMaxFeatures = 16;

enum class CalibrationPartition : uint8_t {
  Training = 0,
  ThresholdTuning,
  FinalValidation,
};

struct CalibrationFeatureSchema {
  uint32_t schema_id = 0;
  uint16_t version = 0;
  uint8_t feature_count = 0;
};

enum class CalibrationBinaryTarget : int8_t {
  Ignore = -1,
  Negative = 0,
  Positive = 1,
};

enum class CalibrationTrainingStatus : uint8_t {
  Ok = 0,
  InvalidArgument,
  FeatureSchemaRequired,
  FeatureSchemaLocked,
  DuplicateSessionId,
  RunCapacityReached,
  RunNotFound,
  SampleCapacityReached,
  FeatureCountMismatch,
  NonFiniteFeature,
  InvalidTarget,
  MissingTrainingCoverage,
  MissingTuningCoverage,
  MissingFinalCoverage,
  NumericalFailure,
};

struct CalibrationRunDescriptor {
  uint32_t session_id = 0;
  CalibrationPartition partition = CalibrationPartition::Training;
  CalibrationLabel label = CalibrationLabel::Empty;
  uint16_t repetition = 0;
  uint16_t time_block = 0;
};

struct CalibrationRunSummary {
  CalibrationRunDescriptor descriptor{};
  uint16_t sample_count = 0;
  uint16_t occupied_positive = 0;
  uint16_t occupied_negative = 0;
  uint16_t motion_positive = 0;
  uint16_t motion_negative = 0;
};

struct CalibrationTrainingProgress {
  uint8_t feature_count = 0;
  uint8_t run_count = 0;
  uint16_t sample_count = 0;
  uint16_t training_samples = 0;
  uint16_t tuning_samples = 0;
  uint16_t final_validation_samples = 0;
};

struct CalibrationMetrics {
  bool ready = false;
  uint16_t evaluated_samples = 0;
  uint16_t occupied_true_positive = 0;
  uint16_t occupied_true_negative = 0;
  uint16_t occupied_false_positive = 0;
  uint16_t occupied_false_negative = 0;
  uint16_t motion_true_positive = 0;
  uint16_t motion_true_negative = 0;
  uint16_t motion_false_positive = 0;
  uint16_t motion_false_negative = 0;
  float occupied_recall = 0.0F;
  float motion_recall = 0.0F;
  float precision = 0.0F;
  float balanced_accuracy = 0.0F;
  float false_alarm_rate = 0.0F;
  float objective = 0.0F;
};

struct CalibrationLogisticHead {
  float bias = 0.0F;
  float weights[kCalibrationMaxFeatures]{};
};

struct CalibratedDetectionModel {
  bool ready = false;
  CalibrationFeatureSchema feature_schema{};
  uint8_t feature_count = 0;
  uint16_t training_samples = 0;
  float feature_mean[kCalibrationMaxFeatures]{};
  float feature_scale[kCalibrationMaxFeatures]{};
  CalibrationLogisticHead occupied{};
  CalibrationLogisticHead motion{};
  float occupied_threshold = 0.5F;
  float motion_threshold = 0.5F;
  CalibrationMetrics tuning_metrics{};
  CalibrationMetrics final_metrics{};
};
