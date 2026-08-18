#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#ifndef APP_PROTOCOL_VERSION
#define APP_PROTOCOL_VERSION 1
#endif

namespace atom::radar::protocol {

constexpr uint32_t kPacketMagic = 0x41544F4DUL;
constexpr uint16_t kProtocolVersion = APP_PROTOCOL_VERSION;
constexpr std::size_t kMaximumPacketBytes = 192;

enum class WireRole : uint8_t {
  TxCoordinator = 0,
  Receiver = 1,
};

enum class WireBandwidth : uint8_t {
  Ht20 = 20,
  Ht40 = 40,
};

enum class ControlCommand : uint8_t {
  Pair = 0,
  SetChannel,
  SetBandwidth,
  StartRadioSurvey,
  StartEmptyTraining,
  StartStillTraining,
  StartMotionTraining,
  StartNuisanceTraining,
  StartValidation,
  CommitModel,
  AbortCalibration,
  ResetConfiguration,
  RequestStatus,
};

#pragma pack(push, 1)
struct ProbePacket {
  uint32_t magic;
  uint16_t protocol_version;
  uint16_t payload_length;
  uint32_t system_id;
  uint32_t sequence;
  uint64_t tx_uptime_us;
  uint8_t channel;
  uint8_t bandwidth;
  uint8_t role;
  uint8_t reserved;
  uint32_t crc32;
};

struct ControlPacket {
  uint32_t magic;
  uint16_t protocol_version;
  uint16_t payload_length;
  uint32_t system_id;
  uint32_t sequence;
  uint64_t tx_uptime_us;
  uint8_t command;
  uint8_t channel;
  uint8_t bandwidth;
  uint8_t reserved;
  uint32_t argument;
  uint32_t crc32;
};
#pragma pack(pop)

static_assert(sizeof(ProbePacket) == 32, "ProbePacket wire size changed");
static_assert(sizeof(ControlPacket) == 36, "ControlPacket wire size changed");
static_assert(sizeof(ProbePacket) < kMaximumPacketBytes, "ProbePacket exceeds transport limit");
static_assert(sizeof(ControlPacket) < kMaximumPacketBytes, "ControlPacket exceeds transport limit");
static_assert(std::is_trivially_copyable_v<ProbePacket>, "ProbePacket must remain trivially copyable");
static_assert(std::is_trivially_copyable_v<ControlPacket>, "ControlPacket must remain trivially copyable");

inline uint32_t crc32(const uint8_t *data, std::size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (std::size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      const uint32_t mask = 0U - (crc & 1U);
      crc = (crc >> 1U) ^ (0xEDB88320UL & mask);
    }
  }
  return ~crc;
}

inline uint32_t packetCrc(const ProbePacket &packet) {
  return crc32(reinterpret_cast<const uint8_t *>(&packet), offsetof(ProbePacket, crc32));
}

inline uint32_t packetCrc(const ControlPacket &packet) {
  return crc32(reinterpret_cast<const uint8_t *>(&packet), offsetof(ControlPacket, crc32));
}

inline ProbePacket makeProbePacket(uint32_t system_id, uint32_t sequence, uint64_t tx_uptime_us,
                                   uint8_t channel, WireBandwidth bandwidth) {
  ProbePacket packet{};
  packet.magic = kPacketMagic;
  packet.protocol_version = kProtocolVersion;
  packet.payload_length = sizeof(ProbePacket);
  packet.system_id = system_id;
  packet.sequence = sequence;
  packet.tx_uptime_us = tx_uptime_us;
  packet.channel = channel;
  packet.bandwidth = static_cast<uint8_t>(bandwidth);
  packet.role = static_cast<uint8_t>(WireRole::TxCoordinator);
  packet.crc32 = packetCrc(packet);
  return packet;
}

inline bool decodeProbePacket(const uint8_t *data, std::size_t length, ProbePacket &packet) {
  if (data == nullptr || length != sizeof(ProbePacket)) {
    return false;
  }

  std::memcpy(&packet, data, sizeof(packet));
  return packet.magic == kPacketMagic && packet.protocol_version == kProtocolVersion &&
         packet.payload_length == sizeof(ProbePacket) &&
         packet.role == static_cast<uint8_t>(WireRole::TxCoordinator) &&
         packet.crc32 == packetCrc(packet);
}

inline bool isSequenceNewer(uint32_t candidate, uint32_t previous) {
  return static_cast<int32_t>(candidate - previous) > 0;
}

}  // namespace atom::radar::protocol
