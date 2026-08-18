#pragma once

#include <stdint.h>

#include "CsiPreprocessor.hpp"
#include "calibration_model_types.hpp"

constexpr uint8_t kCalibrationMaxReceivers = 3;
constexpr uint8_t kCalibrationMaxNodes = 4;
constexpr uint8_t kCalibrationMaxSelectedSubcarriers = 24;
constexpr uint16_t kCalibrationMaxCovarianceValues =
    (kCalibrationMaxSelectedSubcarriers *
     (kCalibrationMaxSelectedSubcarriers + 1)) /
    2;

enum class CalibrationNodeOrientation : uint8_t {
  Unknown = 0,
  DisplayUp,
  DisplayDown,
  UsbUp,
  UsbDown,
  Custom,
};

struct CalibrationReceiverRadioSnapshot {
  uint8_t node_id = 0;
  int8_t median_rssi = 0;
  int8_t noise_floor = 0;
  float valid_csi_ratio = 0.0F;
  float packet_interval_ms = 0.0F;
};

struct CalibrationRadioProfile {
  uint8_t channel = 0;
  uint8_t bandwidth_mhz = 0;
  uint8_t receiver_count = 0;
  CalibrationReceiverRadioSnapshot
      receivers[kCalibrationMaxReceivers]{};
};

struct CalibrationArtifact {
  uint16_t artifact_version = 1;
  uint32_t calibration_id = 0;
  uint64_t created_at_unix_s = 0;
  uint32_t firmware_compatibility_version = 0;
  uint32_t feature_pipeline_version = 0;
  uint32_t protocol_version = 0;
  uint8_t node_count = 0;
  CalibrationNodeOrientation
      node_orientations[kCalibrationMaxNodes]{};
  CalibrationRadioProfile radio{};
  RobustBaseline empty_baseline{};
  uint8_t selected_subcarrier_count = 0;
  int16_t selected_subcarriers[
      kCalibrationMaxSelectedSubcarriers]{};
  uint16_t covariance_value_count = 0;
  float covariance_lower_triangle[
      kCalibrationMaxCovarianceValues]{};
  CalibratedDetectionModel model{};
};
