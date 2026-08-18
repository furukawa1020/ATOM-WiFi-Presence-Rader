#pragma once

#include <atomic>
#include <stdint.h>

#include "calibration_model_types.hpp"

enum class CalibratedRuntimeStatus : uint8_t {
  Ok = 0,
  InvalidSchema,
  FeatureIndexOutOfRange,
  DuplicateFeature,
  NonFiniteFeature,
  IncompleteFeatureVector,
  InvalidModel,
  NoActiveModel,
  SchemaMismatch,
  FeatureCountMismatch,
  NumericalFailure,
};

struct CalibrationFeatureVector {
  bool ready = false;
  CalibrationFeatureSchema schema{};
  float values[kCalibrationMaxFeatures]{};
};

struct CalibratedInference {
  bool ready = false;
  CalibrationFeatureSchema schema{};
  float occupied_probability = 0.0F;
  float motion_probability = 0.0F;
  bool occupied = false;
  bool motion = false;
};

class CalibrationFeatureVectorBuilder {
 public:
  CalibratedRuntimeStatus begin(const CalibrationFeatureSchema& schema);
  CalibratedRuntimeStatus set(uint8_t feature_index, float value);
  CalibratedRuntimeStatus finish(CalibrationFeatureVector& output);
  void reset();

 private:
  CalibrationFeatureVector vector_{};
  uint32_t populated_mask_ = 0;
};

class CalibratedModelRuntime {
 public:
  CalibratedModelRuntime();

  void reset();
  CalibratedRuntimeStatus activate(
      const CalibratedDetectionModel& candidate);
  CalibratedRuntimeStatus infer(
      const CalibrationFeatureVector& features,
      CalibratedInference& output) const;
  bool hasActiveModel() const;
  bool activeSchema(CalibrationFeatureSchema& output) const;

 private:
  static bool validSchema(const CalibrationFeatureSchema& schema);
 public:
  static bool validateModel(const CalibratedDetectionModel& model);

 private:
  static float sigmoid(float value);
  static float score(
      const CalibrationFeatureVector& features,
      const CalibratedDetectionModel& model,
      const CalibrationLogisticHead& head);

  CalibratedDetectionModel models_[2]{};
  std::atomic<uint8_t> active_index_{0};
  std::atomic<bool> active_ready_{false};
};
