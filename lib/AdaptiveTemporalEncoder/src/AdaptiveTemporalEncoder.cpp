#include "AdaptiveTemporalEncoder.hpp"

#include <math.h>
#include <string.h>

AdaptiveTemporalEncoder::AdaptiveTemporalEncoder() {
  reset();
}

void AdaptiveTemporalEncoder::reset() {
  models_[0] = AdaptiveTemporalEncoderModel{};
  models_[1] = AdaptiveTemporalEncoderModel{};
  active_index_.store(0, std::memory_order_relaxed);
  active_ready_.store(false, std::memory_order_release);
  memset(state_, 0, sizeof(state_));
  last_timestamp_us_ = 0;
  warmup_count_ = 0;
  diagnostics_ = TemporalEncoderDiagnostics{};
}

void AdaptiveTemporalEncoder::resetState(TemporalEncoderResetReason reason) {
  resetStateInternal(reason);
}

TemporalEncoderStatus AdaptiveTemporalEncoder::activate(
    const AdaptiveTemporalEncoderModel& candidate) {
  if (!validateModel(candidate)) {
    return TemporalEncoderStatus::InvalidModel;
  }

  const uint8_t current =
      active_index_.load(std::memory_order_acquire);
  const uint8_t next = current ^ 1U;
  models_[next] = candidate;
  std::atomic_thread_fence(std::memory_order_release);
  active_index_.store(next, std::memory_order_release);
  active_ready_.store(true, std::memory_order_release);
  resetStateInternal(TemporalEncoderResetReason::ModelActivated);
  return TemporalEncoderStatus::Ok;
}

TemporalEncoderStatus AdaptiveTemporalEncoder::step(
    const CalibrationFeatureVector& input,
    const TemporalEncoderStepContext& context,
    CalibrationFeatureVector& output) {
  output = CalibrationFeatureVector{};
  if (!active_ready_.load(std::memory_order_acquire)) {
    return TemporalEncoderStatus::NoActiveModel;
  }
  if (!input.ready || context.timestamp_us <= 0 ||
      !isfinite(context.environment_drift_score)) {
    ++diagnostics_.rejected_steps;
    return TemporalEncoderStatus::InvalidInput;
  }

  const uint8_t active =
      active_index_.load(std::memory_order_acquire);
  const AdaptiveTemporalEncoderModel& model = models_[active];
  if (input.schema.schema_id != model.input_schema.schema_id ||
      input.schema.version != model.input_schema.version) {
    ++diagnostics_.rejected_steps;
    return TemporalEncoderStatus::SchemaMismatch;
  }
  if (input.schema.feature_count !=
      model.input_schema.feature_count) {
    ++diagnostics_.rejected_steps;
    return TemporalEncoderStatus::FeatureCountMismatch;
  }

  if (!context.link_healthy) {
    ++diagnostics_.rejected_steps;
    resetStateInternal(TemporalEncoderResetReason::LinkLoss);
    return TemporalEncoderStatus::QualityRejected;
  }
  if (context.queue_drop_detected) {
    ++diagnostics_.rejected_steps;
    resetStateInternal(TemporalEncoderResetReason::QueueDrop);
    return TemporalEncoderStatus::StateReset;
  }
  if (context.device_moved) {
    ++diagnostics_.rejected_steps;
    resetStateInternal(TemporalEncoderResetReason::DeviceMovement);
    return TemporalEncoderStatus::StateReset;
  }
  if (model.drift_reset_threshold > 0.0F &&
      context.environment_drift_score >
          model.drift_reset_threshold) {
    ++diagnostics_.rejected_steps;
    resetStateInternal(TemporalEncoderResetReason::EnvironmentDrift);
    return TemporalEncoderStatus::StateReset;
  }

  float interval_ratio = 1.0F;
  if (last_timestamp_us_ > 0) {
    if (context.timestamp_us <= last_timestamp_us_) {
      ++diagnostics_.rejected_steps;
      resetStateInternal(TemporalEncoderResetReason::TimeRollback);
      return TemporalEncoderStatus::StateReset;
    }
    const int64_t interval_us =
        context.timestamp_us - last_timestamp_us_;
    if (interval_us > model.max_gap_us) {
      resetStateInternal(TemporalEncoderResetReason::FrameGap);
    } else {
      interval_ratio = clamp(
          static_cast<float>(interval_us) /
              static_cast<float>(model.nominal_interval_us),
          0.25F,
          4.0F);
    }
  }

  float normalized[kCalibrationMaxFeatures]{};
  for (uint8_t feature = 0;
       feature < model.input_schema.feature_count;
       ++feature) {
    if (!isfinite(input.values[feature])) {
      ++diagnostics_.rejected_steps;
      return TemporalEncoderStatus::InvalidInput;
    }
    normalized[feature] = clamp(
        (input.values[feature] - model.input_mean[feature]) /
            model.input_scale[feature],
        -model.input_clip,
        model.input_clip);
  }

  float gate_sum = 0.0F;
  float state_energy = 0.0F;
  for (uint8_t state = 0;
       state < model.state_count;
       ++state) {
    float projected = model.input_bias[state];
    for (uint8_t feature = 0;
         feature < model.input_schema.feature_count;
         ++feature) {
      projected +=
          model.input_projection[state][feature] *
          normalized[feature];
    }
    projected = tanhf(projected);
    const float update_gate = softplus(
        model.update_gate_gain[state] * projected +
        model.update_gate_bias[state]);
    const float retention = expf(
        -model.decay_rate[state] *
        update_gate *
        interval_ratio);
    state_[state] =
        retention * state_[state] +
        (1.0F - retention) * projected;
    if (!isfinite(state_[state])) {
      ++diagnostics_.rejected_steps;
      resetStateInternal(TemporalEncoderResetReason::NumericalFailure);
      return TemporalEncoderStatus::NumericalFailure;
    }
    gate_sum += update_gate;
    state_energy += state_[state] * state_[state];
  }

  output.schema = model.output_schema;
  for (uint8_t feature = 0;
       feature < model.output_count;
       ++feature) {
    float value = model.output_bias[feature];
    for (uint8_t state = 0;
         state < model.state_count;
         ++state) {
      value +=
          model.output_projection[feature][state] *
          state_[state];
    }
    if (feature < model.input_schema.feature_count) {
      value +=
          model.skip_scale[feature] * normalized[feature];
    }
    output.values[feature] = tanhf(value);
    if (!isfinite(output.values[feature])) {
      output = CalibrationFeatureVector{};
      ++diagnostics_.rejected_steps;
      resetStateInternal(TemporalEncoderResetReason::NumericalFailure);
      return TemporalEncoderStatus::NumericalFailure;
    }
  }

  last_timestamp_us_ = context.timestamp_us;
  if (warmup_count_ < model.warmup_steps) {
    ++warmup_count_;
  }
  ++diagnostics_.processed_steps;
  diagnostics_.last_mean_gate =
      gate_sum / static_cast<float>(model.state_count);
  diagnostics_.last_state_norm = sqrtf(state_energy);
  diagnostics_.warmup_remaining =
      warmup_count_ >= model.warmup_steps
          ? 0
          : model.warmup_steps - warmup_count_;
  if (warmup_count_ < model.warmup_steps) {
    return TemporalEncoderStatus::Warmup;
  }

  output.ready = true;
  ++diagnostics_.emitted_steps;
  return TemporalEncoderStatus::Ok;
}

bool AdaptiveTemporalEncoder::hasActiveModel() const {
  return active_ready_.load(std::memory_order_acquire);
}

const TemporalEncoderDiagnostics& AdaptiveTemporalEncoder::diagnostics() const {
  return diagnostics_;
}

bool AdaptiveTemporalEncoder::validateModel(
    const AdaptiveTemporalEncoderModel& model) {
  if (!model.ready ||
      model.model_id == 0 ||
      model.model_version == 0 ||
      !validSchema(model.input_schema) ||
      !validSchema(model.output_schema) ||
      model.state_count == 0 ||
      model.state_count > kTemporalEncoderMaxStates ||
      model.output_count == 0 ||
      model.output_count > kCalibrationMaxFeatures ||
      model.output_count != model.output_schema.feature_count ||
      model.warmup_steps == 0 ||
      model.nominal_interval_us <= 0 ||
      model.max_gap_us <= model.nominal_interval_us ||
      !isfinite(model.input_clip) ||
      !(model.input_clip > 0.0F) ||
      !isfinite(model.drift_reset_threshold) ||
      model.drift_reset_threshold < 0.0F) {
    return false;
  }

  for (uint8_t feature = 0;
       feature < model.input_schema.feature_count;
       ++feature) {
    if (!isfinite(model.input_mean[feature]) ||
        !isfinite(model.input_scale[feature]) ||
        !(model.input_scale[feature] > 0.0F)) {
      return false;
    }
  }
  for (uint8_t state = 0;
       state < model.state_count;
       ++state) {
    if (!isfinite(model.input_bias[state]) ||
        !isfinite(model.update_gate_gain[state]) ||
        !isfinite(model.update_gate_bias[state]) ||
        !isfinite(model.decay_rate[state]) ||
        !(model.decay_rate[state] > 0.0F)) {
      return false;
    }
    for (uint8_t feature = 0;
         feature < model.input_schema.feature_count;
         ++feature) {
      if (!isfinite(
              model.input_projection[state][feature])) {
        return false;
      }
    }
  }
  for (uint8_t feature = 0;
       feature < model.output_count;
       ++feature) {
    if (!isfinite(model.skip_scale[feature]) ||
        !isfinite(model.output_bias[feature])) {
      return false;
    }
    for (uint8_t state = 0;
         state < model.state_count;
         ++state) {
      if (!isfinite(
              model.output_projection[feature][state])) {
        return false;
      }
    }
  }
  return true;
}

bool AdaptiveTemporalEncoder::initializeDefaultModel(
    uint32_t model_id,
    const CalibrationFeatureSchema& input_schema,
    const CalibrationFeatureSchema& output_schema,
    AdaptiveTemporalEncoderModel& output) {
  output = AdaptiveTemporalEncoderModel{};
  if (model_id == 0 ||
      !validSchema(input_schema) ||
      !validSchema(output_schema)) {
    return false;
  }

  output.model_id = model_id;
  output.model_version = 1;
  output.input_schema = input_schema;
  output.output_schema = output_schema;
  output.state_count =
      input_schema.feature_count < kTemporalEncoderMaxStates
          ? input_schema.feature_count
          : kTemporalEncoderMaxStates;
  output.output_count = output_schema.feature_count;
  output.warmup_steps = 8;
  output.nominal_interval_us = 100000;
  output.max_gap_us = 750000;
  output.input_clip = 8.0F;
  output.drift_reset_threshold = 6.0F;

  uint8_t group_size[kTemporalEncoderMaxStates]{};
  for (uint8_t feature = 0;
       feature < input_schema.feature_count;
       ++feature) {
    ++group_size[feature % output.state_count];
    output.input_scale[feature] = 1.0F;
  }
  for (uint8_t state = 0;
       state < output.state_count;
       ++state) {
    output.update_gate_gain[state] = 1.0F;
    output.update_gate_bias[state] = -0.5F;
    output.decay_rate[state] =
        0.03F * powf(2.0F, static_cast<float>(state));
  }
  for (uint8_t feature = 0;
       feature < input_schema.feature_count;
       ++feature) {
    const uint8_t state = feature % output.state_count;
    output.input_projection[state][feature] =
        1.0F / static_cast<float>(group_size[state]);
  }
  for (uint8_t feature = 0;
       feature < output.output_count;
       ++feature) {
    output.output_projection[
        feature][feature % output.state_count] = 1.0F;
    output.skip_scale[feature] =
        feature < input_schema.feature_count ? 0.25F : 0.0F;
  }

  output.ready = true;
  if (!validateModel(output)) {
    output = AdaptiveTemporalEncoderModel{};
    return false;
  }
  return true;
}

bool AdaptiveTemporalEncoder::validSchema(
    const CalibrationFeatureSchema& schema) {
  return schema.schema_id != 0 &&
      schema.version != 0 &&
      schema.feature_count != 0 &&
      schema.feature_count <= kCalibrationMaxFeatures;
}

float AdaptiveTemporalEncoder::softplus(float value) {
  if (value > 20.0F) {
    return value;
  }
  if (value < -20.0F) {
    return expf(value);
  }
  return log1pf(expf(value));
}

float AdaptiveTemporalEncoder::clamp(
    float value,
    float minimum,
    float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

void AdaptiveTemporalEncoder::resetStateInternal(
    TemporalEncoderResetReason reason) {
  memset(state_, 0, sizeof(state_));
  last_timestamp_us_ = 0;
  warmup_count_ = 0;
  diagnostics_.last_mean_gate = 0.0F;
  diagnostics_.last_state_norm = 0.0F;
  diagnostics_.last_reset_reason = reason;
  if (reason == TemporalEncoderResetReason::None) {
    return;
  }

  ++diagnostics_.reset_count;
  switch (reason) {
    case TemporalEncoderResetReason::FrameGap:
      ++diagnostics_.frame_gap_resets;
      break;
    case TemporalEncoderResetReason::TimeRollback:
      ++diagnostics_.time_rollback_resets;
      break;
    case TemporalEncoderResetReason::LinkLoss:
      ++diagnostics_.link_loss_resets;
      break;
    case TemporalEncoderResetReason::QueueDrop:
      ++diagnostics_.queue_drop_resets;
      break;
    case TemporalEncoderResetReason::DeviceMovement:
      ++diagnostics_.device_movement_resets;
      break;
    case TemporalEncoderResetReason::EnvironmentDrift:
      ++diagnostics_.environment_drift_resets;
      break;
    case TemporalEncoderResetReason::NumericalFailure:
      ++diagnostics_.numerical_resets;
      break;
    case TemporalEncoderResetReason::Manual:
    case TemporalEncoderResetReason::ModelActivated:
    case TemporalEncoderResetReason::None:
    default:
      break;
  }
}
