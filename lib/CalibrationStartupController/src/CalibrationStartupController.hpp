#pragma once

#include <cstdint>

#include <CalibrationBootstrap.hpp>
#include <calibration_startup_types.hpp>

struct CalibrationStartupSnapshot {
  CalibrationStartupState state = CalibrationStartupState::Idle;
  CalibrationBootstrapReport last_report{};
  uint32_t total_attempts = 0;
  uint32_t successful_activations = 0;
  uint32_t consecutive_failures = 0;
  uint16_t automatic_retry_attempts = 0;
  uint32_t current_retry_delay_ms = 0;
  uint32_t next_retry_at_ms = 0;
  uint32_t active_calibration_id = 0;
  uint32_t active_generation = 0;
  bool automatic_retry_armed = false;
};

class CalibrationStartupController {
 public:
  CalibrationStartupState begin(
      const CalibrationRuntimeFingerprint& runtime,
      CalibrationStorage& storage,
      AdaptiveTemporalEncoder& temporal_encoder,
      CalibratedModelRuntime& detection_runtime,
      uint32_t now_ms,
      const CalibrationRetryPolicy& policy = CalibrationRetryPolicy{});

  CalibrationStartupState poll(uint32_t now_ms);
  CalibrationStartupState reload(uint32_t now_ms);

  bool started() const;
  bool ready() const;
  const CalibrationStartupSnapshot& snapshot() const;
  const CalibrationBootstrap& bootstrap() const;

 private:
  CalibrationStartupState attempt(uint32_t now_ms);
  void scheduleRetry(uint32_t now_ms);
  void stopRetry();
  void updateActiveSnapshot();
  bool autoRetryAvailable() const;
  static bool deadlineReached(uint32_t now_ms, uint32_t deadline_ms);
  static CalibrationRetryPolicy normalizePolicy(
      const CalibrationRetryPolicy& policy);

  CalibrationBootstrap bootstrap_{};
  CalibrationRuntimeFingerprint runtime_{};
  CalibrationStorage* storage_ = nullptr;
  AdaptiveTemporalEncoder* temporal_encoder_ = nullptr;
  CalibratedModelRuntime* detection_runtime_ = nullptr;
  CalibrationRetryPolicy policy_{};
  CalibrationStartupSnapshot snapshot_{};
  uint32_t next_delay_ms_ = 0;
  bool started_ = false;
};
