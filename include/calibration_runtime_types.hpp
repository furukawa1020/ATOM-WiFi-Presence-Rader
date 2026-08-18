#pragma once

#include <cstdint>

#include <calibration_artifact.hpp>

enum class CalibrationCompatibilityStatus : uint8_t {
  Ok = 0,
  InvalidRuntimeFingerprint,
  FirmwareVersionMismatch,
  FeaturePipelineVersionMismatch,
  ProtocolVersionMismatch,
  RadioChannelMismatch,
  RadioBandwidthMismatch,
  ReceiverCountMismatch,
  ReceiverNodeMismatch,
  NodeCountMismatch,
  NodeOrientationMismatch,
  FeatureSchemaChainMismatch,
};

struct CalibrationRuntimeFingerprint {
  uint32_t firmware_compatibility_version = 0;
  uint32_t feature_pipeline_version = 0;
  uint32_t protocol_version = 0;
  uint8_t channel = 0;
  uint8_t bandwidth_mhz = 0;
  uint8_t receiver_count = 0;
  uint8_t receiver_node_ids[kCalibrationMaxReceivers]{};
  uint8_t node_count = 0;
  CalibrationNodeOrientation node_orientations[kCalibrationMaxNodes]{};
};
