#include "CalibrationBootstrap.hpp"

namespace {

template <typename Status>
uint8_t statusCode(const Status status) {
  return static_cast<uint8_t>(status);
}

template <typename Status>
bool statusIsOk(const Status status) {
  return statusCode(status) == 0;
}

bool sameSchema(const CalibrationFeatureSchema& lhs,
                const CalibrationFeatureSchema& rhs) {
  return lhs.schema_id == rhs.schema_id && lhs.version == rhs.version &&
         lhs.feature_count == rhs.feature_count;
}

bool hasUniqueReceiverIds(const CalibrationRuntimeFingerprint& runtime) {
  for (uint8_t i = 0; i < runtime.receiver_count; ++i) {
    if (runtime.receiver_node_ids[i] == 0) {
      return false;
    }
    for (uint8_t j = static_cast<uint8_t>(i + 1);
         j < runtime.receiver_count; ++j) {
      if (runtime.receiver_node_ids[i] == runtime.receiver_node_ids[j]) {
        return false;
      }
    }
  }
  return true;
}

bool containsReceiver(const CalibrationRadioProfile& radio,
                      const uint8_t node_id) {
  for (uint8_t i = 0; i < radio.receiver_count; ++i) {
    if (radio.receivers[i].node_id == node_id) {
      return true;
    }
  }
  return false;
}

}  // namespace

CalibrationBootstrapStatus CalibrationBootstrap::loadAndActivate(
    const CalibrationRuntimeFingerprint& runtime,
    CalibrationStorage& storage,
    AdaptiveTemporalEncoder& temporal_encoder,
    CalibratedModelRuntime& detection_runtime,
    CalibrationBootstrapReport& report) {
  report = CalibrationBootstrapReport{};
  publishActiveState(report);

  if (!validateRuntimeFingerprint(runtime)) {
    report.status = CalibrationBootstrapStatus::InvalidRuntimeFingerprint;
    return report.status;
  }

  CalibrationStorageInfo loaded_info{};
  const auto storage_status = storage.load(candidate_, &loaded_info);
  report.storage_status_code = statusCode(storage_status);
  report.storage_info = loaded_info;
  if (!statusIsOk(storage_status)) {
    report.status = CalibrationBootstrapStatus::StorageLoadFailed;
    return report.status;
  }

  report.candidate_calibration_id = candidate_.calibration_id;
  report.compatibility_status = checkCompatibility(runtime, candidate_);
  if (report.compatibility_status != CalibrationCompatibilityStatus::Ok) {
    report.status = CalibrationBootstrapStatus::CompatibilityFailed;
    return report.status;
  }

  if (!AdaptiveTemporalEncoder::validateModel(candidate_.temporal_encoder)) {
    report.status = CalibrationBootstrapStatus::TemporalModelInvalid;
    return report.status;
  }
  if (!CalibratedModelRuntime::validateModel(candidate_.model)) {
    report.status = CalibrationBootstrapStatus::DetectionModelInvalid;
    return report.status;
  }

  const auto temporal_status =
      temporal_encoder.activate(candidate_.temporal_encoder);
  report.temporal_status_code = statusCode(temporal_status);
  if (!statusIsOk(temporal_status)) {
    report.status = CalibrationBootstrapStatus::TemporalActivationFailed;
    return report.status;
  }

  const auto detection_status = detection_runtime.activate(candidate_.model);
  report.detection_status_code = statusCode(detection_status);
  if (!statusIsOk(detection_status)) {
    if (active_) {
      temporal_encoder.activate(active_artifact_.temporal_encoder);
    }
    report.status = CalibrationBootstrapStatus::DetectionActivationFailed;
    return report.status;
  }

  active_artifact_ = candidate_;
  active_generation_ = loaded_info.generation;
  active_ = true;
  report.status = CalibrationBootstrapStatus::Ok;
  publishActiveState(report);
  return report.status;
}

bool CalibrationBootstrap::validateRuntimeFingerprint(
    const CalibrationRuntimeFingerprint& runtime) {
  if (runtime.firmware_compatibility_version == 0 ||
      runtime.feature_pipeline_version == 0 || runtime.protocol_version == 0 ||
      runtime.channel == 0 || runtime.channel > 14 ||
      (runtime.bandwidth_mhz != 20 && runtime.bandwidth_mhz != 40) ||
      runtime.receiver_count == 0 ||
      runtime.receiver_count > kCalibrationMaxReceivers || runtime.node_count == 0 ||
      runtime.node_count > kCalibrationMaxNodes ||
      runtime.receiver_count > runtime.node_count ||
      !hasUniqueReceiverIds(runtime)) {
    return false;
  }

  if (runtime.channel == 14 && runtime.bandwidth_mhz != 20) {
    return false;
  }
  for (uint8_t i = 0; i < runtime.node_count; ++i) {
    if (runtime.node_orientations[i] == CalibrationNodeOrientation::Unknown) {
      return false;
    }
  }
  return true;
}

CalibrationCompatibilityStatus CalibrationBootstrap::checkCompatibility(
    const CalibrationRuntimeFingerprint& runtime,
    const CalibrationArtifact& artifact) {
  if (!validateRuntimeFingerprint(runtime)) {
    return CalibrationCompatibilityStatus::InvalidRuntimeFingerprint;
  }
  if (artifact.firmware_compatibility_version !=
      runtime.firmware_compatibility_version) {
    return CalibrationCompatibilityStatus::FirmwareVersionMismatch;
  }
  if (artifact.feature_pipeline_version != runtime.feature_pipeline_version) {
    return CalibrationCompatibilityStatus::FeaturePipelineVersionMismatch;
  }
  if (artifact.protocol_version != runtime.protocol_version) {
    return CalibrationCompatibilityStatus::ProtocolVersionMismatch;
  }
  if (artifact.radio.channel != runtime.channel) {
    return CalibrationCompatibilityStatus::RadioChannelMismatch;
  }
  if (artifact.radio.bandwidth_mhz != runtime.bandwidth_mhz) {
    return CalibrationCompatibilityStatus::RadioBandwidthMismatch;
  }
  if (artifact.radio.receiver_count != runtime.receiver_count) {
    return CalibrationCompatibilityStatus::ReceiverCountMismatch;
  }
  for (uint8_t i = 0; i < runtime.receiver_count; ++i) {
    if (!containsReceiver(artifact.radio, runtime.receiver_node_ids[i])) {
      return CalibrationCompatibilityStatus::ReceiverNodeMismatch;
    }
  }
  if (artifact.node_count != runtime.node_count) {
    return CalibrationCompatibilityStatus::NodeCountMismatch;
  }
  for (uint8_t i = 0; i < runtime.node_count; ++i) {
    if (artifact.node_orientations[i] != runtime.node_orientations[i]) {
      return CalibrationCompatibilityStatus::NodeOrientationMismatch;
    }
  }
  if (!sameSchema(artifact.temporal_encoder.output_schema,
                  artifact.model.feature_schema)) {
    return CalibrationCompatibilityStatus::FeatureSchemaChainMismatch;
  }
  return CalibrationCompatibilityStatus::Ok;
}

bool CalibrationBootstrap::ready() const { return active_; }

uint32_t CalibrationBootstrap::activeCalibrationId() const {
  return active_ ? active_artifact_.calibration_id : 0;
}

uint32_t CalibrationBootstrap::activeGeneration() const {
  return active_ ? active_generation_ : 0;
}

const CalibrationArtifact* CalibrationBootstrap::artifact() const {
  return active_ ? &active_artifact_ : nullptr;
}

const RobustBaseline* CalibrationBootstrap::baseline() const {
  return active_ ? &active_artifact_.empty_baseline : nullptr;
}

uint8_t CalibrationBootstrap::selectedSubcarrierCount() const {
  return active_ ? active_artifact_.selected_subcarrier_count : 0;
}

const int16_t* CalibrationBootstrap::selectedSubcarriers() const {
  return active_ ? active_artifact_.selected_subcarriers : nullptr;
}

uint16_t CalibrationBootstrap::covarianceValueCount() const {
  return active_ ? active_artifact_.covariance_value_count : 0;
}

const float* CalibrationBootstrap::covarianceLowerTriangle() const {
  return active_ ? active_artifact_.covariance_lower_triangle : nullptr;
}

void CalibrationBootstrap::publishActiveState(
    CalibrationBootstrapReport& report) const {
  report.active = active_;
  report.active_calibration_id = activeCalibrationId();
  report.active_generation = activeGeneration();
}
