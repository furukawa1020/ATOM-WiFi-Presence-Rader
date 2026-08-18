#include "CalibrationStorage.hpp"

#include <Preferences.h>
#include <math.h>

namespace {

constexpr char kNamespace[] = "atom_cal";
constexpr char kSlotAKey[] = "slot_a";
constexpr char kSlotBKey[] = "slot_b";

bool validMetricSet(const CalibrationMetrics& metrics) {
  return metrics.ready &&
      isfinite(metrics.occupied_recall) &&
      isfinite(metrics.motion_recall) &&
      isfinite(metrics.precision) &&
      isfinite(metrics.balanced_accuracy) &&
      isfinite(metrics.false_alarm_rate) &&
      isfinite(metrics.objective);
}

bool validModel(const CalibratedDetectionModel& model) {
  if (!model.ready ||
      model.feature_schema.schema_id == 0 ||
      model.feature_schema.version == 0 ||
      model.feature_schema.feature_count == 0 ||
      model.feature_schema.feature_count > kCalibrationMaxFeatures ||
      model.feature_count != model.feature_schema.feature_count ||
      model.training_samples == 0 ||
      !validMetricSet(model.tuning_metrics) ||
      !validMetricSet(model.final_metrics) ||
      !isfinite(model.occupied.bias) ||
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

bool baselineContainsUsable(
    const RobustBaseline& baseline,
    int16_t subcarrier) {
  for (uint16_t i = 0; i < baseline.sample_count; ++i) {
    if (baseline.subcarriers[i] == subcarrier) {
      return baseline.usable[i];
    }
  }
  return false;
}

}  // namespace

CalibrationStorageStatus CalibrationStorage::save(
    const CalibrationArtifact& artifact,
    uint32_t* saved_generation) {
  if (saved_generation != nullptr) {
    *saved_generation = 0;
  }
  if (!validateArtifact(artifact)) {
    return CalibrationStorageStatus::InvalidArtifact;
  }

  Preferences preferences;
  if (!preferences.begin(kNamespace, false)) {
    return CalibrationStorageStatus::NamespaceUnavailable;
  }

  const CalibrationStorageSlotState slot_a_state =
      readSlot(preferences, kSlotAKey, slot_a_);
  const CalibrationStorageSlotState slot_b_state =
      readSlot(preferences, kSlotBKey, slot_b_);
  const CalibrationStorageSlot latest = chooseLatest(
      slot_a_state, slot_a_, slot_b_state, slot_b_);

  CalibrationStorageSlot target = CalibrationStorageSlot::SlotA;
  uint32_t latest_generation = 0;
  if (latest == CalibrationStorageSlot::SlotA) {
    target = CalibrationStorageSlot::SlotB;
    latest_generation = slot_a_.header.generation;
  } else if (latest == CalibrationStorageSlot::SlotB) {
    target = CalibrationStorageSlot::SlotA;
    latest_generation = slot_b_.header.generation;
  }

  uint32_t next_generation = latest_generation + 1U;
  if (next_generation == 0) {
    next_generation = 1;
  }

  working_ = CalibrationStorageBlob{};
  working_.header.magic = kMagic;
  working_.header.storage_version = kStorageVersion;
  working_.header.artifact_version = kArtifactVersion;
  working_.header.blob_size = sizeof(CalibrationStorageBlob);
  working_.header.generation = next_generation;
  working_.artifact = artifact;
  working_.header.crc32 = calculateCrc(working_);

  const char* target_key =
      target == CalibrationStorageSlot::SlotA ? kSlotAKey : kSlotBKey;
  const size_t written = preferences.putBytes(
      target_key, &working_, sizeof(working_));
  if (written != sizeof(working_)) {
    preferences.end();
    return CalibrationStorageStatus::WriteFailed;
  }

  CalibrationStorageBlob& verified =
      target == CalibrationStorageSlot::SlotA ? slot_a_ : slot_b_;
  const CalibrationStorageSlotState verified_state =
      readSlot(preferences, target_key, verified);
  preferences.end();
  if (verified_state != CalibrationStorageSlotState::Valid ||
      verified.header.generation != next_generation ||
      verified.header.crc32 != working_.header.crc32) {
    return CalibrationStorageStatus::VerifyFailed;
  }

  if (saved_generation != nullptr) {
    *saved_generation = next_generation;
  }
  return CalibrationStorageStatus::Ok;
}

CalibrationStorageStatus CalibrationStorage::load(
    CalibrationArtifact& output,
    CalibrationStorageInfo* info) {
  output = CalibrationArtifact{};
  if (info != nullptr) {
    *info = CalibrationStorageInfo{};
  }

  Preferences preferences;
  if (!preferences.begin(kNamespace, true)) {
    return CalibrationStorageStatus::NotFound;
  }
  const CalibrationStorageSlotState slot_a_state =
      readSlot(preferences, kSlotAKey, slot_a_);
  const CalibrationStorageSlotState slot_b_state =
      readSlot(preferences, kSlotBKey, slot_b_);
  preferences.end();

  CalibrationStorageInfo local_info{};
  fillInfo(
      local_info,
      slot_a_state,
      slot_a_,
      slot_b_state,
      slot_b_);
  const CalibrationStorageSlot selected = chooseLatest(
      slot_a_state, slot_a_, slot_b_state, slot_b_);
  if (selected == CalibrationStorageSlot::None) {
    if (info != nullptr) {
      *info = local_info;
    }
    return failureStatus(slot_a_state, slot_b_state);
  }

  const CalibrationStorageBlob& selected_blob =
      selected == CalibrationStorageSlot::SlotA ? slot_a_ : slot_b_;
  output = selected_blob.artifact;
  local_info.selected_slot = selected;
  local_info.selected_generation = selected_blob.header.generation;
  if (info != nullptr) {
    *info = local_info;
  }
  return CalibrationStorageStatus::Ok;
}

CalibrationStorageStatus CalibrationStorage::inspect(
    CalibrationStorageInfo& output) {
  output = CalibrationStorageInfo{};
  Preferences preferences;
  if (!preferences.begin(kNamespace, true)) {
    return CalibrationStorageStatus::NotFound;
  }
  const CalibrationStorageSlotState slot_a_state =
      readSlot(preferences, kSlotAKey, slot_a_);
  const CalibrationStorageSlotState slot_b_state =
      readSlot(preferences, kSlotBKey, slot_b_);
  preferences.end();

  fillInfo(
      output,
      slot_a_state,
      slot_a_,
      slot_b_state,
      slot_b_);
  const CalibrationStorageSlot selected = chooseLatest(
      slot_a_state, slot_a_, slot_b_state, slot_b_);
  if (selected == CalibrationStorageSlot::None) {
    return failureStatus(slot_a_state, slot_b_state);
  }
  output.selected_slot = selected;
  output.selected_generation =
      selected == CalibrationStorageSlot::SlotA
          ? slot_a_.header.generation
          : slot_b_.header.generation;
  return CalibrationStorageStatus::Ok;
}

bool CalibrationStorage::validateArtifact(
    const CalibrationArtifact& artifact) {
  if (artifact.artifact_version != kArtifactVersion ||
      artifact.calibration_id == 0 ||
      artifact.firmware_compatibility_version == 0 ||
      artifact.feature_pipeline_version == 0 ||
      artifact.protocol_version == 0 ||
      artifact.node_count == 0 ||
      artifact.node_count > kCalibrationMaxNodes ||
      artifact.radio.channel < 1 ||
      artifact.radio.channel > 13 ||
      (artifact.radio.bandwidth_mhz != 20 &&
       artifact.radio.bandwidth_mhz != 40) ||
      artifact.radio.receiver_count == 0 ||
      artifact.radio.receiver_count > kCalibrationMaxReceivers ||
      !artifact.empty_baseline.ready ||
      artifact.empty_baseline.sample_count == 0 ||
      artifact.empty_baseline.sample_count > 114 ||
      artifact.selected_subcarrier_count == 0 ||
      artifact.selected_subcarrier_count >
          kCalibrationMaxSelectedSubcarriers ||
      !validModel(artifact.model)) {
    return false;
  }

  for (uint8_t i = 0; i < artifact.node_count; ++i) {
    if (artifact.node_orientations[i] ==
        CalibrationNodeOrientation::Unknown) {
      return false;
    }
  }
  for (uint8_t i = 0; i < artifact.radio.receiver_count; ++i) {
    const CalibrationReceiverRadioSnapshot& receiver =
        artifact.radio.receivers[i];
    if (receiver.node_id == 0 ||
        !isfinite(receiver.valid_csi_ratio) ||
        receiver.valid_csi_ratio < 0.0F ||
        receiver.valid_csi_ratio > 1.0F ||
        !isfinite(receiver.packet_interval_ms) ||
        !(receiver.packet_interval_ms > 0.0F)) {
      return false;
    }
    for (uint8_t other = 0; other < i; ++other) {
      if (artifact.radio.receivers[other].node_id ==
          receiver.node_id) {
        return false;
      }
    }
  }

  for (uint16_t i = 0;
       i < artifact.empty_baseline.sample_count;
       ++i) {
    for (uint16_t other = 0; other < i; ++other) {
      if (artifact.empty_baseline.subcarriers[other] ==
          artifact.empty_baseline.subcarriers[i]) {
        return false;
      }
    }
    if (artifact.empty_baseline.usable[i] &&
        (!isfinite(artifact.empty_baseline.median[i]) ||
         !isfinite(artifact.empty_baseline.mad[i]) ||
         !(artifact.empty_baseline.mad[i] > 0.0F))) {
      return false;
    }
  }

  for (uint8_t i = 0;
       i < artifact.selected_subcarrier_count;
       ++i) {
    const int16_t subcarrier = artifact.selected_subcarriers[i];
    if (!baselineContainsUsable(
            artifact.empty_baseline, subcarrier)) {
      return false;
    }
    for (uint8_t other = 0; other < i; ++other) {
      if (artifact.selected_subcarriers[other] == subcarrier) {
        return false;
      }
    }
  }

  const uint16_t expected_covariance_count =
      (artifact.selected_subcarrier_count *
       (artifact.selected_subcarrier_count + 1U)) /
      2U;
  if (artifact.covariance_value_count !=
      expected_covariance_count) {
    return false;
  }
  for (uint16_t i = 0;
       i < artifact.covariance_value_count;
       ++i) {
    if (!isfinite(artifact.covariance_lower_triangle[i])) {
      return false;
    }
  }
  for (uint8_t row = 0;
       row < artifact.selected_subcarrier_count;
       ++row) {
    const uint16_t diagonal_index =
        (row * (row + 1U)) / 2U + row;
    if (!(artifact.covariance_lower_triangle[
            diagonal_index] >= 0.0F)) {
      return false;
    }
  }

  return true;
}

bool CalibrationStorage::generationNewer(
    uint32_t candidate,
    uint32_t reference) {
  return static_cast<int32_t>(candidate - reference) > 0;
}

uint32_t CalibrationStorage::calculateCrc(
    const CalibrationStorageBlob& blob) {
  const uint8_t* bytes =
      reinterpret_cast<const uint8_t*>(&blob);
  const size_t crc_offset =
      offsetof(CalibrationStorageBlob, header) +
      offsetof(CalibrationStorageHeader, crc32);
  uint32_t crc = 0xFFFFFFFFU;
  crc = updateCrc(crc, bytes, crc_offset);
  const uint32_t zero = 0;
  crc = updateCrc(
      crc,
      reinterpret_cast<const uint8_t*>(&zero),
      sizeof(zero));
  const size_t suffix_offset =
      crc_offset + sizeof(CalibrationStorageHeader::crc32);
  crc = updateCrc(
      crc,
      bytes + suffix_offset,
      sizeof(CalibrationStorageBlob) - suffix_offset);
  return ~crc;
}

uint32_t CalibrationStorage::updateCrc(
    uint32_t crc,
    const uint8_t* data,
    size_t length) {
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask =
          0U - static_cast<uint32_t>(crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320U & mask);
    }
  }
  return crc;
}

CalibrationStorageSlotState CalibrationStorage::validateBlob(
    const CalibrationStorageBlob& blob) {
  if (blob.header.magic != kMagic ||
      blob.header.storage_version != kStorageVersion ||
      blob.header.artifact_version != kArtifactVersion ||
      blob.header.blob_size != sizeof(CalibrationStorageBlob) ||
      blob.artifact.artifact_version != kArtifactVersion) {
    return CalibrationStorageSlotState::VersionMismatch;
  }
  if (blob.header.crc32 != calculateCrc(blob)) {
    return CalibrationStorageSlotState::CrcMismatch;
  }
  if (!validateArtifact(blob.artifact)) {
    return CalibrationStorageSlotState::InvalidArtifact;
  }
  return CalibrationStorageSlotState::Valid;
}

CalibrationStorageStatus CalibrationStorage::failureStatus(
    CalibrationStorageSlotState slot_a,
    CalibrationStorageSlotState slot_b) {
  if (slot_a == CalibrationStorageSlotState::ReadError ||
      slot_b == CalibrationStorageSlotState::ReadError) {
    return CalibrationStorageStatus::ReadFailed;
  }
  if (slot_a == CalibrationStorageSlotState::CrcMismatch ||
      slot_b == CalibrationStorageSlotState::CrcMismatch) {
    return CalibrationStorageStatus::CrcMismatch;
  }
  if (slot_a == CalibrationStorageSlotState::VersionMismatch ||
      slot_b == CalibrationStorageSlotState::VersionMismatch) {
    return CalibrationStorageStatus::VersionMismatch;
  }
  if (slot_a == CalibrationStorageSlotState::InvalidArtifact ||
      slot_b == CalibrationStorageSlotState::InvalidArtifact) {
    return CalibrationStorageStatus::InvalidArtifact;
  }
  return CalibrationStorageStatus::NotFound;
}

void CalibrationStorage::fillInfo(
    CalibrationStorageInfo& output,
    CalibrationStorageSlotState slot_a_state,
    const CalibrationStorageBlob& slot_a,
    CalibrationStorageSlotState slot_b_state,
    const CalibrationStorageBlob& slot_b) {
  output = CalibrationStorageInfo{};
  output.slot_a_state = slot_a_state;
  output.slot_b_state = slot_b_state;
  if (slot_a_state == CalibrationStorageSlotState::Valid) {
    output.slot_a_generation = slot_a.header.generation;
  }
  if (slot_b_state == CalibrationStorageSlotState::Valid) {
    output.slot_b_generation = slot_b.header.generation;
  }
}

CalibrationStorageSlot CalibrationStorage::chooseLatest(
    CalibrationStorageSlotState slot_a_state,
    const CalibrationStorageBlob& slot_a,
    CalibrationStorageSlotState slot_b_state,
    const CalibrationStorageBlob& slot_b) {
  const bool slot_a_valid =
      slot_a_state == CalibrationStorageSlotState::Valid;
  const bool slot_b_valid =
      slot_b_state == CalibrationStorageSlotState::Valid;
  if (slot_a_valid && slot_b_valid) {
    return generationNewer(
               slot_b.header.generation,
               slot_a.header.generation)
        ? CalibrationStorageSlot::SlotB
        : CalibrationStorageSlot::SlotA;
  }
  if (slot_a_valid) {
    return CalibrationStorageSlot::SlotA;
  }
  if (slot_b_valid) {
    return CalibrationStorageSlot::SlotB;
  }
  return CalibrationStorageSlot::None;
}

CalibrationStorageSlotState CalibrationStorage::readSlot(
    Preferences& preferences,
    const char* key,
    CalibrationStorageBlob& output) {
  output = CalibrationStorageBlob{};
  const size_t stored_size = preferences.getBytesLength(key);
  if (stored_size == 0) {
    return CalibrationStorageSlotState::Missing;
  }
  if (stored_size != sizeof(output)) {
    return CalibrationStorageSlotState::ReadError;
  }
  const size_t read =
      preferences.getBytes(key, &output, sizeof(output));
  if (read != sizeof(output)) {
    output = CalibrationStorageBlob{};
    return CalibrationStorageSlotState::ReadError;
  }
  return validateBlob(output);
}
