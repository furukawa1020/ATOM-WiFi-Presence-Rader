#include "CalibrationStartupController.hpp"

CalibrationStartupState CalibrationStartupController::begin(
    const CalibrationRuntimeFingerprint& runtime,
    CalibrationStorage& storage,
    AdaptiveTemporalEncoder& temporal_encoder,
    CalibratedModelRuntime& detection_runtime,
    const uint32_t now_ms,
    const CalibrationRetryPolicy& policy) {
  runtime_ = runtime;
  storage_ = &storage;
  temporal_encoder_ = &temporal_encoder;
  detection_runtime_ = &detection_runtime;
  policy_ = normalizePolicy(policy);
  snapshot_ = CalibrationStartupSnapshot{};
  next_delay_ms_ = policy_.initial_delay_ms;
  started_ = true;
  return attempt(now_ms);
}

CalibrationStartupState CalibrationStartupController::poll(
    const uint32_t now_ms) {
  if (!started_ || !snapshot_.automatic_retry_armed ||
      !deadlineReached(now_ms, snapshot_.next_retry_at_ms)) {
    return snapshot_.state;
  }

  snapshot_.automatic_retry_armed = false;
  ++snapshot_.automatic_retry_attempts;
  return attempt(now_ms);
}

CalibrationStartupState CalibrationStartupController::reload(
    const uint32_t now_ms) {
  if (!started_) {
    return snapshot_.state;
  }

  stopRetry();
  snapshot_.automatic_retry_attempts = 0;
  next_delay_ms_ = policy_.initial_delay_ms;
  return attempt(now_ms);
}

bool CalibrationStartupController::started() const { return started_; }

bool CalibrationStartupController::ready() const { return bootstrap_.ready(); }

const CalibrationStartupSnapshot& CalibrationStartupController::snapshot()
    const {
  return snapshot_;
}

const CalibrationBootstrap& CalibrationStartupController::bootstrap() const {
  return bootstrap_;
}

CalibrationStartupState CalibrationStartupController::attempt(
    const uint32_t now_ms) {
  snapshot_.state = CalibrationStartupState::Loading;
  ++snapshot_.total_attempts;

  CalibrationBootstrapReport report{};
  const CalibrationBootstrapStatus status = bootstrap_.loadAndActivate(
      runtime_, *storage_, *temporal_encoder_, *detection_runtime_, report);
  snapshot_.last_report = report;
  updateActiveSnapshot();

  if (status == CalibrationBootstrapStatus::Ok) {
    stopRetry();
    snapshot_.state = CalibrationStartupState::Active;
    snapshot_.consecutive_failures = 0;
    snapshot_.automatic_retry_attempts = 0;
    snapshot_.current_retry_delay_ms = 0;
    next_delay_ms_ = policy_.initial_delay_ms;
    ++snapshot_.successful_activations;
    return snapshot_.state;
  }

  ++snapshot_.consecutive_failures;
  if (status == CalibrationBootstrapStatus::StorageLoadFailed &&
      autoRetryAvailable()) {
    scheduleRetry(now_ms);
    snapshot_.state = bootstrap_.ready()
                          ? CalibrationStartupState::ActiveRetryWait
                          : CalibrationStartupState::RetryWait;
    return snapshot_.state;
  }

  stopRetry();
  snapshot_.state = bootstrap_.ready()
                        ? CalibrationStartupState::ActiveDegraded
                        : CalibrationStartupState::Degraded;
  return snapshot_.state;
}

void CalibrationStartupController::scheduleRetry(const uint32_t now_ms) {
  snapshot_.current_retry_delay_ms = next_delay_ms_;
  snapshot_.next_retry_at_ms = now_ms + next_delay_ms_;
  snapshot_.automatic_retry_armed = true;

  if (next_delay_ms_ >= policy_.maximum_delay_ms ||
      next_delay_ms_ > policy_.maximum_delay_ms / 2U) {
    next_delay_ms_ = policy_.maximum_delay_ms;
  } else {
    next_delay_ms_ *= 2U;
  }
}

void CalibrationStartupController::stopRetry() {
  snapshot_.automatic_retry_armed = false;
  snapshot_.next_retry_at_ms = 0;
}

void CalibrationStartupController::updateActiveSnapshot() {
  snapshot_.active_calibration_id = bootstrap_.activeCalibrationId();
  snapshot_.active_generation = bootstrap_.activeGeneration();
}

bool CalibrationStartupController::autoRetryAvailable() const {
  return policy_.maximum_auto_attempts == 0 ||
         snapshot_.automatic_retry_attempts < policy_.maximum_auto_attempts;
}

bool CalibrationStartupController::deadlineReached(const uint32_t now_ms,
                                                    const uint32_t deadline_ms) {
  return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

CalibrationRetryPolicy CalibrationStartupController::normalizePolicy(
    const CalibrationRetryPolicy& policy) {
  CalibrationRetryPolicy normalized = policy;
  if (normalized.initial_delay_ms == 0) {
    normalized.initial_delay_ms = 1;
  }
  if (normalized.maximum_delay_ms < normalized.initial_delay_ms) {
    normalized.maximum_delay_ms = normalized.initial_delay_ms;
  }
  return normalized;
}
