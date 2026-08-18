#pragma once

#include <cstddef>
#include <cstdint>

#include <CsiCapture.hpp>

namespace atom::radar {

constexpr std::size_t kHt20SubcarrierCount = 56;
constexpr std::size_t kHt40SubcarrierCount = 114;
constexpr std::size_t kMaximumHtSubcarrierCount = kHt40SubcarrierCount;

enum class CsiParseStatus : uint8_t {
  Ok = 0,
  NonHtFrame,
  BandwidthMismatch,
  LengthMismatch,
  NoUsableSubcarriers,
};

struct CsiComplexSample {
  int16_t subcarrier;
  int8_t imaginary;
  int8_t real;
  bool valid_for_features;
};

struct ParsedCsiFrame {
  int64_t received_at_us;
  int8_t rssi;
  int8_t noise_floor;
  uint8_t channel;
  uint8_t bandwidth_mhz;
  uint16_t sample_count;
  uint16_t valid_sample_count;
  CsiComplexSample samples[kMaximumHtSubcarrierCount];
};

class CsiFrameParser final {
 public:
  CsiParseStatus parse(const CsiFrame &frame, wifi_bandwidth_t expected_bandwidth,
                       ParsedCsiFrame &output) const;

 private:
  static bool isPilot(int16_t subcarrier, wifi_bandwidth_t bandwidth);
};

}  // namespace atom::radar
