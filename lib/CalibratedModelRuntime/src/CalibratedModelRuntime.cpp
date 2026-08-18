#include "CalibratedModelRuntime.hpp"

#include <math.h>

CalibratedRuntimeStatus CalibrationFeatureVectorBuilder::begin(
    const CalibrationFeatureSchema& schema) {
  reset();
  if (schema.schema_id == 0 || schema.version == 0 ||
      schema.feature_count == 0 ||
      schema.feature_count > kCalibrationMaxFeatures) {
    return CalibratedRuntimeStatus::InvalidSchema;
  }

  vector_.schema = schema;
  return CalibratedRuntimeStatus::Ok;
}

CalibratedRuntimeStatus CalibrationFeatureVectorBuilder::set(
    uint8_t feature_index,
    float value) {
  if (vector_.schema.schema_id == 0) {
    return CalibratedRuntimeStatus::InvalidSchema;
  }
  if (feature_index >= vector_.schema.feature_count) {
    return CalibratedRuntimeStatus::FeatureIndexOutOfRange;
  }
  if (!isfinite(value)) {
    return CalibratedRuntimeStatus::NonFiniteFeature;
  }

  const uint32_t feature_mask = 1UL << feature_index;
  if ((populated_mask_ & feature_mask) != 0) {
    return CalibratedRuntimeStatus::DuplicateFeature;
  }

  vector_.values[feature_index] = value;
  populated_mask_ |= feature_mask;
  return CalibratedRuntimeStatus::Ok;
}

CalibratedRuntimeStatus CalibrationFeatureVectorBuilder::finish(
    CalibrationFeatureVector& output) {
  output = CalibrationFeatureVector{};
  if (vector_.schema.schema_id == 0) {
    return CalibratedRuntimeStatus::InvalidSchema;
  }

  const uint32_t required_mask =
      (1UL << vector_.schema.feature_count) - 1UL;
  if (populated_mask_ != required_mask) {
    return CalibratedRuntimeStatus::IncompleteFeatureVector;
  }

  vector_.ready = true;
  output = vector_;
  return CalibratedRuntimeStatus::Ok;
}

void CalibrationFeatureVectorBuilder::reset() {
  vector_ = CalibrationFeatureVector{};
  populated_mask_ = 0;
}

CalibratedModelRuntime::CalibratedModelRuntime() {
  reset();
}

void CalibratedModelRuntime::reset() {
  models_[0] = CalibratedDetectionModel{};
  models_[1] = CalibratedDetectionModel{};
  active_index_.store(0, std::memory_order_relaxed);
  active_ready_.store(false, std::memory_order_release);
}

CalibratedRuntimeStatus CalibratedModelRuntime::activate(
    const CalibratedDetectionModel& candidate) {
  if (!validModel(candidate)) {
    return CalibratedRuntimeStatus::InvalidModel;
  }

  const uint8_t current =
      active_index_.load(std::memory_order_acquire);
  const uint8_t next = current ^ 1U;
  models_[next] = candidate;
  std::atomic_thread_fence(std::memory_order_release);
  active_index_.store(next, std::memory_order_release);
  active_ready_.store(true, std::memory_order_release);
  return CalibratedRuntimeStatus::Ok;
}

CalibratedRuntimeStatus CalibratedModelRuntime::infer(
    const CalibrationFeatureVector& features,
    CalibratedInference& output) const {
  output = CalibratedInference{};
  if (!features.ready) {
    return CalibratedRuntimeStatus::IncompleteFeatureVector;
  }
  if (!active_ready_.load(std::memory_order_acquire)) {
    return CalibratedRuntimeStatus::NoActiveModel;
  }

  const uint8_t active =
      active_index_.load(std::memory_order_acquire);
  const CalibratedDetectionModel& model = models_[active];
  if (features.schema.schema_id != model.feature_schema.schema_id ||
      features.schema.version != model.feature_schema.version) {
    return CalibratedRuntimeStatus::SchemaMismatch;
  }
  if (features.schema.feature_count != model.feature_count ||
      features.schema.feature_count !=
          model.feature_schema.feature_count) {
    return CalibratedRuntimeStatus::FeatureCountMismatch;
  }
  for (uint8_t i = 0; i < features.schema.feature_count; ++i) {
    if (!isfinite(features.values[i])) {
      return CalibratedRuntimeStatus::NonFiniteFeature;
    }
  }

  const float occupied_probability =
      score(features, model, model.occupied);
  const float motion_probability =
      score(features, model, model.motion);
  if (!isfinite(occupied_probability) ||
      !isfinite(motion_probability)) {
    return CalibratedRuntimeStatus::NumericalFailure;
  }

  output.ready = true;
  output.schema = model.feature_schema;
  output.occupied_probability = occupied_probability;
  output.motion_probability = motion_probability;
  output.occupied =
      occupied_probability >= model.occupied_threshold;
  output.motion =
      motion_probability >= model.motion_threshold;
  return CalibratedRuntimeStatus::Ok;
}

bool CalibratedModelRuntime::hasActiveModel() const {
  return active_ready_.load(std::memory_order_acquire);
}

bool CalibratedModelRuntime::activeSchema(
    CalibrationFeatureSchema& output) const {
  output = CalibrationFeatureSchema{};
  if (!active_ready_.load(std::memory_order_acquire)) {
    return false;
  }
  const uint8_t active =
      active_index_.load(std::memory_order_acquire);
  output = models_[active].feature_schema;
  return true;
}

bool CalibratedModelRuntime::validSchema(
    const CalibrationFeatureSchema& schema) {
  return schema.schema_id != 0 &&
      schema.version != 0 &&
      schema.feature_count != 0 &&
      schema.feature_count <= kCalibrationMaxFeatures;
}

bool CalibratedModelRuntime::validModel(
    const CalibratedDetectionModel& model) {
  if (!model.ready || !validSchema(model.feature_schema) ||
      model.feature_count != model.feature_schema.feature_count) {
    return false;
  }
  if (!isfinite(model.occupied.bias) ||
      !isfinite(model.motion.bias) ||
      !isfinite(model.occupied_threshold) ||
      !isfinite(model.motion_threshold) ||
      model.occupied_threshold < 0.0F ||
      model.occupied_threshold > 1.0F ||
      model.motion_threshold < 0.0F ||
      model.motion_threshold > 1.0F) {
    return false;
  }

  for (uint8_t i = 0; i < model.feature_count; ++i) {
    if (!isfinite(model.feature_mean[i]) ||
        !isfinite(model.feature_scale[i]) ||
        !(model.feature_scale[i] > 0.0F) ||
        !isfinite(model.occupied.weights[i]) ||
        !isfinite(model.motion.weights[i])) {
      return false;
    }
  }
  return true;
}

float CalibratedModelRuntime::sigmoid(float value) {
  if (value > 20.0F) {
    value = 20.0F;
  } else if (value < -20.0F) {
    value = -20.0F;
  }
  return 1.0F / (1.0F + expf(-value));
}

float CalibratedModelRuntime::score(
    const CalibrationFeatureVector& features,
    const CalibratedDetectionModel& model,
    const CalibrationLogisticHead& head) {
  float linear = head.bias;
  for (uint8_t i = 0; i < model.feature_count; ++i) {
    const float normalized =
        (features.values[i] - model.feature_mean[i]) /
        model.feature_scale[i];
    linear += head.weights[i] * normalized;
  }
  return sigmoid(linear);
}
