#pragma once

#include <stdint.h>

#include "CsiPreprocessor.hpp"
#include "calibration_types.hpp"

struct CalibrationConfig {
  uint16_t min_empty_frames = 120;
  uint16_t max_empty_frames = 192;
  uint16_t min_usable_subcarriers = 16;
  float min_valid_ratio = 0.80F;
  float min_packet_rate_hz = 50.0F;
  float max_empty_change = 0.35F;
  float mad_floor = 0.01F;
  int64_t empty_sample_interval_us = 100000;
};

class CalibrationManager {
 public:
  static constexpr uint16_t kMaxSubcarriers = 114;
  static constexpr uint16_t kMaxEmptyFrames = 192;
  static constexpr uint8_t kMaxSessions = 16;

  CalibrationManager();
  explicit CalibrationManager(const CalibrationConfig& config);

  void configure(const CalibrationConfig& config);
  void reset();

  CalibrationStatus beginCalibration(uint32_t calibration_id);
  CalibrationStatus transitionTo(CalibrationStage next_stage);
  CalibrationStatus startSession(
      const CalibrationSessionDescriptor& descriptor,
      int64_t started_at_us);
  CalibrationStatus ingestEmptyFrame(
      const PreprocessedCsiFrame& frame,
      const CalibrationQualitySnapshot& quality);
  CalibrationStatus finishSession(int64_t finished_at_us);
  CalibrationStatus cancelSession(int64_t finished_at_us);

  const CalibrationProgress& progress() const;
  const RobustBaseline* baseline() const;
  uint8_t sessionCount() const;
  bool sessionSummary(uint8_t index, CalibrationSessionSummary& output) const;

 private:
  static bool isAllowedTransition(
      CalibrationStage current,
      CalibrationStage next);
  static CalibrationStage stageForSession(
      const CalibrationSessionDescriptor& descriptor);
  static void sortAscending(float* values, uint16_t count);
  static float median(float* values, uint16_t count);

  CalibrationStatus reject(
      CalibrationRejectReason reason,
      CalibrationStatus status);
  CalibrationStatus buildBaseline();
  void resetEmptyCollection();
  void closeActiveSession(
      CalibrationSessionState state,
      int64_t finished_at_us);
  bool hasSessionId(uint32_t session_id) const;

  CalibrationConfig config_{};
  CalibrationProgress progress_{};
  CalibrationSessionSummary sessions_[kMaxSessions]{};
  int8_t active_session_index_ = -1;
  uint8_t session_count_ = 0;

  RobustBaseline baseline_{};
  float empty_samples_[kMaxEmptyFrames][kMaxSubcarriers]{};
  int16_t subcarrier_order_[kMaxSubcarriers]{};
  float previous_empty_frame_[kMaxSubcarriers]{};
  bool previous_valid_[kMaxSubcarriers]{};
  float scratch_[kMaxEmptyFrames]{};
  uint16_t empty_frame_count_ = 0;
  uint16_t subcarrier_count_ = 0;
  bool previous_frame_ready_ = false;
  int64_t last_accepted_at_us_ = 0;
  bool queue_drop_count_ready_ = false;
  uint32_t last_queue_drop_count_ = 0;
};
