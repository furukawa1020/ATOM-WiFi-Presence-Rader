#include "CsiFrameParser.hpp"

#include <cstdlib>

namespace atom::radar {

namespace {

constexpr uint8_t kHtSignalMode = 1;

}  // namespace

CsiParseStatus CsiFrameParser::parse(const CsiFrame &frame, wifi_bandwidth_t expected_bandwidth,
                                     ParsedCsiFrame &output) const {
  output = {};

  if (frame.rx_ctrl.sig_mode != kHtSignalMode) {
    return CsiParseStatus::NonHtFrame;
  }

  const bool received_ht40 = frame.rx_ctrl.cwb != 0;
  const bool expected_ht40 = expected_bandwidth == WIFI_BW_HT40;
  if (received_ht40 != expected_ht40) {
    return CsiParseStatus::BandwidthMismatch;
  }

  const std::size_t raw_sample_count = expected_ht40 ? 128U : 64U;
  const std::size_t expected_bytes = raw_sample_count * 2U;
  if (frame.length != expected_bytes) {
    return CsiParseStatus::LengthMismatch;
  }

  const int16_t maximum_subcarrier = expected_ht40 ? 57 : 28;
  output.received_at_us = frame.received_at_us;
  output.rssi = frame.rx_ctrl.rssi;
  output.noise_floor = frame.rx_ctrl.noise_floor;
  output.channel = frame.rx_ctrl.channel;
  output.bandwidth_mhz = expected_ht40 ? 40U : 20U;

  for (int16_t subcarrier = -maximum_subcarrier; subcarrier <= maximum_subcarrier; ++subcarrier) {
    if (subcarrier == 0) {
      continue;
    }

    const std::size_t raw_position =
        subcarrier > 0 ? static_cast<std::size_t>(subcarrier)
                       : raw_sample_count + static_cast<std::size_t>(subcarrier);
    CsiComplexSample &sample = output.samples[output.sample_count++];
    sample.subcarrier = subcarrier;
    sample.imaginary = frame.data[raw_position * 2U];
    sample.real = frame.data[(raw_position * 2U) + 1U];
    sample.valid_for_features =
        !(frame.first_word_invalid && raw_position < 2U) &&
        !isPilot(subcarrier, expected_bandwidth);
    if (sample.valid_for_features) {
      ++output.valid_sample_count;
    }
  }

  const std::size_t required_count = expected_ht40 ? kHt40SubcarrierCount : kHt20SubcarrierCount;
  if (output.sample_count != required_count) {
    output = {};
    return CsiParseStatus::LengthMismatch;
  }
  if (output.valid_sample_count == 0U) {
    return CsiParseStatus::NoUsableSubcarriers;
  }
  return CsiParseStatus::Ok;
}

bool CsiFrameParser::isPilot(int16_t subcarrier, wifi_bandwidth_t bandwidth) {
  const int16_t absolute = static_cast<int16_t>(std::abs(subcarrier));
  if (bandwidth == WIFI_BW_HT40) {
    return absolute == 11 || absolute == 25 || absolute == 53;
  }
  return absolute == 7 || absolute == 21;
}

}  // namespace atom::radar
