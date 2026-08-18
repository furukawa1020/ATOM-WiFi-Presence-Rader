#pragma once

#include <stddef.h>
#include <stdint.h>

#include "calibration_artifact.hpp"

class Preferences;

enum class CalibrationStorageStatus : uint8_t {
  Ok = 0,
  InvalidArtifact,
  NamespaceUnavailable,
  NotFound,
  ReadFailed,
  VersionMismatch,
  CrcMismatch,
  WriteFailed,
  VerifyFailed,
};

enum class CalibrationStorageSlot : uint8_t {
  None = 0,
  SlotA,
  SlotB,
};

enum class CalibrationStorageSlotState : uint8_t {
  Missing = 0,
  Valid,
  ReadError,
  VersionMismatch,
  CrcMismatch,
  InvalidArtifact,
};

struct CalibrationStorageInfo {
  CalibrationStorageSlotState slot_a_state =
      CalibrationStorageSlotState::Missing;
  CalibrationStorageSlotState slot_b_state =
      CalibrationStorageSlotState::Missing;
  uint32_t slot_a_generation = 0;
  uint32_t slot_b_generation = 0;
  CalibrationStorageSlot selected_slot =
      CalibrationStorageSlot::None;
  uint32_t selected_generation = 0;
};

struct CalibrationStorageHeader {
  uint32_t magic = 0;
  uint16_t storage_version = 0;
  uint16_t artifact_version = 0;
  uint32_t blob_size = 0;
  uint32_t generation = 0;
  uint32_t crc32 = 0;
};

struct CalibrationStorageBlob {
  CalibrationStorageHeader header{};
  CalibrationArtifact artifact{};
};

class CalibrationStorage {
 public:
  static constexpr uint32_t kMagic = 0x41544F4DU;
  static constexpr uint16_t kStorageVersion = 1;
  static constexpr uint16_t kArtifactVersion = 1;

  CalibrationStorageStatus save(
      const CalibrationArtifact& artifact,
      uint32_t* saved_generation = nullptr);
  CalibrationStorageStatus load(
      CalibrationArtifact& output,
      CalibrationStorageInfo* info = nullptr);
  CalibrationStorageStatus inspect(CalibrationStorageInfo& output);

  static bool validateArtifact(const CalibrationArtifact& artifact);

 private:
  static bool generationNewer(uint32_t candidate, uint32_t reference);
  static uint32_t calculateCrc(const CalibrationStorageBlob& blob);
  static uint32_t updateCrc(
      uint32_t crc,
      const uint8_t* data,
      size_t length);
  static CalibrationStorageSlotState validateBlob(
      const CalibrationStorageBlob& blob);
  static CalibrationStorageStatus failureStatus(
      CalibrationStorageSlotState slot_a,
      CalibrationStorageSlotState slot_b);
  static void fillInfo(
      CalibrationStorageInfo& output,
      CalibrationStorageSlotState slot_a_state,
      const CalibrationStorageBlob& slot_a,
      CalibrationStorageSlotState slot_b_state,
      const CalibrationStorageBlob& slot_b);
  static CalibrationStorageSlot chooseLatest(
      CalibrationStorageSlotState slot_a_state,
      const CalibrationStorageBlob& slot_a,
      CalibrationStorageSlotState slot_b_state,
      const CalibrationStorageBlob& slot_b);
  CalibrationStorageSlotState readSlot(
      Preferences& preferences,
      const char* key,
      CalibrationStorageBlob& output);

  CalibrationStorageBlob slot_a_{};
  CalibrationStorageBlob slot_b_{};
  CalibrationStorageBlob working_{};
};
