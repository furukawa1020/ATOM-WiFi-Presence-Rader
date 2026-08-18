#include "CalibrationTrainer.hpp"

#include <float.h>
#include <math.h>
#include <string.h>

CalibrationTrainer::CalibrationTrainer() {
  configure(CalibrationTrainerConfig{});
  reset();
}

CalibrationTrainer::CalibrationTrainer(
    const CalibrationTrainerConfig& config) {
  configure(config);
  reset();
}

void CalibrationTrainer::configure(
    const CalibrationTrainerConfig& config) {
  config_ = config;
  if (config_.max_samples == 0 || config_.max_samples > kMaxSamples) {
    config_.max_samples = kMaxSamples;
  }
  if (config_.epochs == 0) {
    config_.epochs = 1;
  }
  if (config_.min_samples_per_partition < 4) {
    config_.min_samples_per_partition = 4;
  }
  if (!(config_.learning_rate > 0.0F)) {
    config_.learning_rate = 0.05F;
  }
  if (config_.l2_regularization < 0.0F) {
    config_.l2_regularization = 0.0F;
  }
  if (!(config_.min_feature_scale > 0.0F)) {
    config_.min_feature_scale = 0.001F;
  }
  if (config_.threshold_min < 0.0F) {
    config_.threshold_min = 0.0F;
  }
  if (config_.threshold_max > 1.0F) {
    config_.threshold_max = 1.0F;
  }
  if (!(config_.threshold_max > config_.threshold_min)) {
    config_.threshold_min = 0.10F;
    config_.threshold_max = 0.90F;
  }
  if (!(config_.threshold_step > 0.0F) ||
      config_.threshold_step >
          config_.threshold_max - config_.threshold_min) {
    config_.threshold_step = 0.05F;
  }
}

void CalibrationTrainer::reset() {
  memset(&progress_, 0, sizeof(progress_));
  memset(runs_, 0, sizeof(runs_));
  run_count_ = 0;
  sample_count_ = 0;
  feature_count_ = 0;
  feature_schema_ = CalibrationFeatureSchema{};
}

CalibrationTrainingStatus CalibrationTrainer::bindFeatureSchema(
    const CalibrationFeatureSchema& schema) {
  if (schema.schema_id == 0 || schema.version == 0 ||
      schema.feature_count == 0 ||
      schema.feature_count > kCalibrationMaxFeatures) {
    return CalibrationTrainingStatus::InvalidArgument;
  }
  if (sample_count_ != 0) {
    return CalibrationTrainingStatus::FeatureSchemaLocked;
  }

  feature_schema_ = schema;
  feature_count_ = schema.feature_count;
  progress_.feature_count = feature_count_;
  return CalibrationTrainingStatus::Ok;
}

CalibrationTrainingStatus CalibrationTrainer::registerRun(
    const CalibrationRunDescriptor& descriptor) {
  if (descriptor.session_id == 0 ||
      !isValidPartition(descriptor.partition)) {
    return CalibrationTrainingStatus::InvalidArgument;
  }
  if (findRun(descriptor.session_id) >= 0) {
    return CalibrationTrainingStatus::DuplicateSessionId;
  }
  if (run_count_ >= kMaxRuns) {
    return CalibrationTrainingStatus::RunCapacityReached;
  }

  runs_[run_count_] = CalibrationRunSummary{};
  runs_[run_count_].descriptor = descriptor;
  ++run_count_;
  progress_.run_count = run_count_;
  return CalibrationTrainingStatus::Ok;
}

CalibrationTrainingStatus CalibrationTrainer::appendSample(
    uint32_t session_id,
    const float* features,
    uint8_t feature_count,
    CalibrationBinaryTarget occupied_target,
    CalibrationBinaryTarget motion_target) {
  const int8_t run_index = findRun(session_id);
  if (run_index < 0) {
    return CalibrationTrainingStatus::RunNotFound;
  }
  if (features == nullptr || feature_count == 0 ||
      feature_count > kCalibrationMaxFeatures) {
    return CalibrationTrainingStatus::InvalidArgument;
  }
  if (feature_schema_.schema_id == 0) {
    return CalibrationTrainingStatus::FeatureSchemaRequired;
  }
  if (feature_count != feature_schema_.feature_count) {
    return CalibrationTrainingStatus::FeatureCountMismatch;
  }
  if (!isValidTarget(occupied_target) || !isValidTarget(motion_target) ||
      (occupied_target == CalibrationBinaryTarget::Ignore &&
       motion_target == CalibrationBinaryTarget::Ignore)) {
    return CalibrationTrainingStatus::InvalidTarget;
  }
  if (!targetsMatchLabel(
          runs_[run_index].descriptor.label,
          occupied_target,
          motion_target)) {
    return CalibrationTrainingStatus::InvalidTarget;
  }
  if (feature_count_ != 0 && feature_count != feature_count_) {
    return CalibrationTrainingStatus::FeatureCountMismatch;
  }
  for (uint8_t i = 0; i < feature_count; ++i) {
    if (!isfinite(features[i])) {
      return CalibrationTrainingStatus::NonFiniteFeature;
    }
  }
  if (sample_count_ >= config_.max_samples) {
    return CalibrationTrainingStatus::SampleCapacityReached;
  }

  if (feature_count_ == 0) {
    feature_count_ = feature_count;
    progress_.feature_count = feature_count_;
  }

  FeatureSample& sample = samples_[sample_count_];
  sample = FeatureSample{};
  sample.run_index = static_cast<uint8_t>(run_index);
  sample.occupied_target = occupied_target;
  sample.motion_target = motion_target;
  for (uint8_t i = 0; i < feature_count_; ++i) {
    sample.values[i] = features[i];
  }

  CalibrationRunSummary& run = runs_[run_index];
  ++run.sample_count;
  updateRunCounts(run, occupied_target, motion_target);
  ++sample_count_;
  progress_.sample_count = sample_count_;
  switch (run.descriptor.partition) {
    case CalibrationPartition::Training:
      ++progress_.training_samples;
      break;
    case CalibrationPartition::ThresholdTuning:
      ++progress_.tuning_samples;
      break;
    case CalibrationPartition::FinalValidation:
      ++progress_.final_validation_samples;
      break;
    default:
      break;
  }

  return CalibrationTrainingStatus::Ok;
}

CalibrationTrainingStatus CalibrationTrainer::train(
    CalibratedDetectionModel& output) {
  output = CalibratedDetectionModel{};
  if (feature_schema_.schema_id == 0) {
    return CalibrationTrainingStatus::FeatureSchemaRequired;
  }
  if (feature_count_ == 0) {
    return CalibrationTrainingStatus::MissingTrainingCoverage;
  }

  const TargetCounts training =
      countTargets(CalibrationPartition::Training);
  if (!hasCoverage(training)) {
    return CalibrationTrainingStatus::MissingTrainingCoverage;
  }
  const TargetCounts tuning =
      countTargets(CalibrationPartition::ThresholdTuning);
  if (!hasCoverage(tuning)) {
    return CalibrationTrainingStatus::MissingTuningCoverage;
  }
  const TargetCounts final_validation =
      countTargets(CalibrationPartition::FinalValidation);
  if (!hasCoverage(final_validation)) {
    return CalibrationTrainingStatus::MissingFinalCoverage;
  }

  output.feature_schema = feature_schema_;
  output.feature_count = feature_count_;
  output.training_samples = training.samples;
  if (!computeNormalization(output) ||
      !fitHead(output, false) ||
      !fitHead(output, true)) {
    output = CalibratedDetectionModel{};
    return CalibrationTrainingStatus::NumericalFailure;
  }
  if (!selectThresholds(output)) {
    output = CalibratedDetectionModel{};
    return CalibrationTrainingStatus::MissingTuningCoverage;
  }

  output.final_metrics = evaluate(
      CalibrationPartition::FinalValidation,
      output,
      output.occupied_threshold,
      output.motion_threshold);
  if (!output.final_metrics.ready) {
    output = CalibratedDetectionModel{};
    return CalibrationTrainingStatus::MissingFinalCoverage;
  }

  output.ready = true;
  return CalibrationTrainingStatus::Ok;
}

const CalibrationTrainingProgress& CalibrationTrainer::progress() const {
  return progress_;
}

uint8_t CalibrationTrainer::runCount() const {
  return run_count_;
}

bool CalibrationTrainer::runSummary(
    uint8_t index,
    CalibrationRunSummary& output) const {
  if (index >= run_count_) {
    return false;
  }
  output = runs_[index];
  return true;
}

bool CalibrationTrainer::standardTargets(
    CalibrationLabel label,
    CalibrationBinaryTarget& occupied_target,
    CalibrationBinaryTarget& motion_target) {
  switch (label) {
    case CalibrationLabel::Empty:
      occupied_target = CalibrationBinaryTarget::Negative;
      motion_target = CalibrationBinaryTarget::Negative;
      return true;
    case CalibrationLabel::Still:
      occupied_target = CalibrationBinaryTarget::Positive;
      motion_target = CalibrationBinaryTarget::Negative;
      return true;
    case CalibrationLabel::Motion:
      occupied_target = CalibrationBinaryTarget::Positive;
      motion_target = CalibrationBinaryTarget::Positive;
      return true;
    case CalibrationLabel::Nuisance:
    default:
      occupied_target = CalibrationBinaryTarget::Ignore;
      motion_target = CalibrationBinaryTarget::Ignore;
      return false;
  }
}

int8_t CalibrationTrainer::findRun(uint32_t session_id) const {
  for (uint8_t i = 0; i < run_count_; ++i) {
    if (runs_[i].descriptor.session_id == session_id) {
      return static_cast<int8_t>(i);
    }
  }
  return -1;
}

bool CalibrationTrainer::isValidPartition(
    CalibrationPartition partition) {
  return partition == CalibrationPartition::Training ||
      partition == CalibrationPartition::ThresholdTuning ||
      partition == CalibrationPartition::FinalValidation;
}

bool CalibrationTrainer::isValidTarget(
    CalibrationBinaryTarget target) {
  return target == CalibrationBinaryTarget::Ignore ||
      target == CalibrationBinaryTarget::Negative ||
      target == CalibrationBinaryTarget::Positive;
}

bool CalibrationTrainer::targetsMatchLabel(
    CalibrationLabel label,
    CalibrationBinaryTarget occupied_target,
    CalibrationBinaryTarget motion_target) {
  switch (label) {
    case CalibrationLabel::Empty:
      return occupied_target == CalibrationBinaryTarget::Negative &&
          motion_target == CalibrationBinaryTarget::Negative;
    case CalibrationLabel::Still:
      return occupied_target == CalibrationBinaryTarget::Positive &&
          motion_target == CalibrationBinaryTarget::Negative;
    case CalibrationLabel::Motion:
      return occupied_target == CalibrationBinaryTarget::Positive &&
          motion_target == CalibrationBinaryTarget::Positive;
    case CalibrationLabel::Nuisance:
      return true;
    default:
      return false;
  }
}

CalibrationTrainer::TargetCounts CalibrationTrainer::countTargets(
    CalibrationPartition partition) const {
  TargetCounts counts{};
  for (uint16_t i = 0; i < sample_count_; ++i) {
    const FeatureSample& sample = samples_[i];
    if (runs_[sample.run_index].descriptor.partition != partition) {
      continue;
    }
    ++counts.samples;
    if (sample.occupied_target == CalibrationBinaryTarget::Positive) {
      ++counts.occupied_positive;
    } else if (
        sample.occupied_target == CalibrationBinaryTarget::Negative) {
      ++counts.occupied_negative;
    }
    if (sample.motion_target == CalibrationBinaryTarget::Positive) {
      ++counts.motion_positive;
    } else if (sample.motion_target == CalibrationBinaryTarget::Negative) {
      ++counts.motion_negative;
    }
  }
  return counts;
}

bool CalibrationTrainer::hasCoverage(const TargetCounts& counts) const {
  return counts.samples >= config_.min_samples_per_partition &&
      counts.occupied_positive > 0 &&
      counts.occupied_negative > 0 &&
      counts.motion_positive > 0 &&
      counts.motion_negative > 0;
}

void CalibrationTrainer::updateRunCounts(
    CalibrationRunSummary& run,
    CalibrationBinaryTarget occupied_target,
    CalibrationBinaryTarget motion_target) {
  if (occupied_target == CalibrationBinaryTarget::Positive) {
    ++run.occupied_positive;
  } else if (occupied_target == CalibrationBinaryTarget::Negative) {
    ++run.occupied_negative;
  }
  if (motion_target == CalibrationBinaryTarget::Positive) {
    ++run.motion_positive;
  } else if (motion_target == CalibrationBinaryTarget::Negative) {
    ++run.motion_negative;
  }
}

bool CalibrationTrainer::computeNormalization(
    CalibratedDetectionModel& output) const {
  uint16_t training_count = 0;
  for (uint16_t i = 0; i < sample_count_; ++i) {
    const FeatureSample& sample = samples_[i];
    if (runs_[sample.run_index].descriptor.partition !=
        CalibrationPartition::Training) {
      continue;
    }
    for (uint8_t feature = 0; feature < feature_count_; ++feature) {
      output.feature_mean[feature] += sample.values[feature];
    }
    ++training_count;
  }
  if (training_count == 0) {
    return false;
  }

  for (uint8_t feature = 0; feature < feature_count_; ++feature) {
    output.feature_mean[feature] /= static_cast<float>(training_count);
  }
  for (uint16_t i = 0; i < sample_count_; ++i) {
    const FeatureSample& sample = samples_[i];
    if (runs_[sample.run_index].descriptor.partition !=
        CalibrationPartition::Training) {
      continue;
    }
    for (uint8_t feature = 0; feature < feature_count_; ++feature) {
      const float centered =
          sample.values[feature] - output.feature_mean[feature];
      output.feature_scale[feature] += centered * centered;
    }
  }
  for (uint8_t feature = 0; feature < feature_count_; ++feature) {
    const float variance =
        output.feature_scale[feature] / static_cast<float>(training_count);
    const float scale = sqrtf(variance);
    output.feature_scale[feature] =
        scale >= config_.min_feature_scale ? scale : 1.0F;
    if (!isfinite(output.feature_mean[feature]) ||
        !isfinite(output.feature_scale[feature])) {
      return false;
    }
  }
  return true;
}

bool CalibrationTrainer::fitHead(
    CalibratedDetectionModel& output,
    bool motion_head) const {
  CalibrationLogisticHead& head =
      motion_head ? output.motion : output.occupied;
  const TargetCounts counts =
      countTargets(CalibrationPartition::Training);
  const uint16_t positive_count =
      motion_head ? counts.motion_positive : counts.occupied_positive;
  const uint16_t negative_count =
      motion_head ? counts.motion_negative : counts.occupied_negative;
  const float labeled_count =
      static_cast<float>(positive_count + negative_count);
  const float positive_weight =
      labeled_count / (2.0F * static_cast<float>(positive_count));
  const float negative_weight =
      labeled_count / (2.0F * static_cast<float>(negative_count));

  for (uint16_t epoch = 0; epoch < config_.epochs; ++epoch) {
    float gradient[kCalibrationMaxFeatures]{};
    float bias_gradient = 0.0F;
    float weight_total = 0.0F;

    for (uint16_t i = 0; i < sample_count_; ++i) {
      const FeatureSample& sample = samples_[i];
      if (runs_[sample.run_index].descriptor.partition !=
          CalibrationPartition::Training) {
        continue;
      }
      const CalibrationBinaryTarget target = motion_head
          ? sample.motion_target
          : sample.occupied_target;
      if (target == CalibrationBinaryTarget::Ignore) {
        continue;
      }

      float linear = head.bias;
      for (uint8_t feature = 0; feature < feature_count_; ++feature) {
        const float normalized =
            (sample.values[feature] - output.feature_mean[feature]) /
            output.feature_scale[feature];
        linear += head.weights[feature] * normalized;
      }
      const float probability = sigmoid(linear);
      const bool positive = target == CalibrationBinaryTarget::Positive;
      const float sample_weight =
          positive ? positive_weight : negative_weight;
      const float error =
          (probability - (positive ? 1.0F : 0.0F)) * sample_weight;
      bias_gradient += error;
      weight_total += sample_weight;
      for (uint8_t feature = 0; feature < feature_count_; ++feature) {
        const float normalized =
            (sample.values[feature] - output.feature_mean[feature]) /
            output.feature_scale[feature];
        gradient[feature] += error * normalized;
      }
    }

    if (!(weight_total > 0.0F)) {
      return false;
    }
    const float learning_rate =
        config_.learning_rate /
        (1.0F + 0.01F * static_cast<float>(epoch));
    head.bias -= learning_rate * bias_gradient / weight_total;
    for (uint8_t feature = 0; feature < feature_count_; ++feature) {
      const float regularized_gradient =
          gradient[feature] / weight_total +
          config_.l2_regularization * head.weights[feature];
      head.weights[feature] -= learning_rate * regularized_gradient;
      if (!isfinite(head.weights[feature])) {
        return false;
      }
    }
    if (!isfinite(head.bias)) {
      return false;
    }
  }
  return true;
}

bool CalibrationTrainer::selectThresholds(
    CalibratedDetectionModel& output) const {
  const uint16_t threshold_steps = static_cast<uint16_t>(floorf(
      (config_.threshold_max - config_.threshold_min) /
          config_.threshold_step +
      0.5F));
  float best_objective = -FLT_MAX;
  float best_threshold_sum = -1.0F;
  CalibrationMetrics best_metrics{};

  for (uint16_t occupied_index = 0;
       occupied_index <= threshold_steps;
       ++occupied_index) {
    float occupied_threshold =
        config_.threshold_min +
        config_.threshold_step * static_cast<float>(occupied_index);
    if (occupied_threshold > config_.threshold_max) {
      occupied_threshold = config_.threshold_max;
    }
    for (uint16_t motion_index = 0;
         motion_index <= threshold_steps;
         ++motion_index) {
      float motion_threshold =
          config_.threshold_min +
          config_.threshold_step * static_cast<float>(motion_index);
      if (motion_threshold > config_.threshold_max) {
        motion_threshold = config_.threshold_max;
      }

      const CalibrationMetrics metrics = evaluate(
          CalibrationPartition::ThresholdTuning,
          output,
          occupied_threshold,
          motion_threshold);
      if (!metrics.ready) {
        continue;
      }
      const float threshold_sum =
          occupied_threshold + motion_threshold;
      if (metrics.objective > best_objective + 0.000001F ||
          (fabsf(metrics.objective - best_objective) <= 0.000001F &&
           threshold_sum > best_threshold_sum)) {
        best_objective = metrics.objective;
        best_threshold_sum = threshold_sum;
        output.occupied_threshold = occupied_threshold;
        output.motion_threshold = motion_threshold;
        best_metrics = metrics;
      }
    }
  }

  if (!best_metrics.ready) {
    return false;
  }
  output.tuning_metrics = best_metrics;
  return true;
}

CalibrationMetrics CalibrationTrainer::evaluate(
    CalibrationPartition partition,
    const CalibratedDetectionModel& model,
    float occupied_threshold,
    float motion_threshold) const {
  CalibrationMetrics metrics{};
  for (uint16_t i = 0; i < sample_count_; ++i) {
    const FeatureSample& sample = samples_[i];
    if (runs_[sample.run_index].descriptor.partition != partition) {
      continue;
    }

    bool evaluated = false;
    if (sample.occupied_target != CalibrationBinaryTarget::Ignore) {
      const bool predicted = predict(sample, model, model.occupied) >=
          occupied_threshold;
      const bool positive =
          sample.occupied_target == CalibrationBinaryTarget::Positive;
      if (predicted && positive) {
        ++metrics.occupied_true_positive;
      } else if (predicted) {
        ++metrics.occupied_false_positive;
      } else if (positive) {
        ++metrics.occupied_false_negative;
      } else {
        ++metrics.occupied_true_negative;
      }
      evaluated = true;
    }
    if (sample.motion_target != CalibrationBinaryTarget::Ignore) {
      const bool predicted =
          predict(sample, model, model.motion) >= motion_threshold;
      const bool positive =
          sample.motion_target == CalibrationBinaryTarget::Positive;
      if (predicted && positive) {
        ++metrics.motion_true_positive;
      } else if (predicted) {
        ++metrics.motion_false_positive;
      } else if (positive) {
        ++metrics.motion_false_negative;
      } else {
        ++metrics.motion_true_negative;
      }
      evaluated = true;
    }
    if (evaluated) {
      ++metrics.evaluated_samples;
    }
  }

  const uint16_t occupied_positive =
      metrics.occupied_true_positive +
      metrics.occupied_false_negative;
  const uint16_t occupied_negative =
      metrics.occupied_true_negative +
      metrics.occupied_false_positive;
  const uint16_t motion_positive =
      metrics.motion_true_positive +
      metrics.motion_false_negative;
  const uint16_t motion_negative =
      metrics.motion_true_negative +
      metrics.motion_false_positive;
  if (occupied_positive == 0 || occupied_negative == 0 ||
      motion_positive == 0 || motion_negative == 0) {
    return metrics;
  }

  metrics.occupied_recall = safeRate(
      metrics.occupied_true_positive, occupied_positive);
  metrics.motion_recall = safeRate(
      metrics.motion_true_positive, motion_positive);
  const uint16_t combined_true_positive =
      metrics.occupied_true_positive + metrics.motion_true_positive;
  const uint16_t combined_predicted_positive =
      combined_true_positive +
      metrics.occupied_false_positive +
      metrics.motion_false_positive;
  metrics.precision = safeRate(
      combined_true_positive, combined_predicted_positive);

  const float occupied_specificity = safeRate(
      metrics.occupied_true_negative, occupied_negative);
  const float motion_specificity = safeRate(
      metrics.motion_true_negative, motion_negative);
  metrics.balanced_accuracy = 0.25F * (
      metrics.occupied_recall +
      occupied_specificity +
      metrics.motion_recall +
      motion_specificity);
  metrics.false_alarm_rate = 0.5F * (
      safeRate(metrics.occupied_false_positive, occupied_negative) +
      safeRate(metrics.motion_false_positive, motion_negative));
  metrics.objective =
      0.35F * metrics.occupied_recall +
      0.25F * metrics.motion_recall +
      0.20F * metrics.precision +
      0.15F * metrics.balanced_accuracy -
      0.05F * metrics.false_alarm_rate;
  metrics.ready = true;
  return metrics;
}

float CalibrationTrainer::predict(
    const FeatureSample& sample,
    const CalibratedDetectionModel& model,
    const CalibrationLogisticHead& head) const {
  float linear = head.bias;
  for (uint8_t feature = 0; feature < model.feature_count; ++feature) {
    const float normalized =
        (sample.values[feature] - model.feature_mean[feature]) /
        model.feature_scale[feature];
    linear += head.weights[feature] * normalized;
  }
  return sigmoid(linear);
}

float CalibrationTrainer::sigmoid(float value) {
  if (value > 20.0F) {
    value = 20.0F;
  } else if (value < -20.0F) {
    value = -20.0F;
  }
  return 1.0F / (1.0F + expf(-value));
}

float CalibrationTrainer::safeRate(
    uint16_t numerator,
    uint16_t denominator) {
  if (denominator == 0) {
    return 0.0F;
  }
  return static_cast<float>(numerator) /
      static_cast<float>(denominator);
}
