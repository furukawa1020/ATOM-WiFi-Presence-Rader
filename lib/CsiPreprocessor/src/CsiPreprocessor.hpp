#pragma once

#include <cstddef>
#include <cstdint>

#include <CsiFrameParser.hpp>

namespace atom::radar {

enum class CsiPreprocessStatus : uint8_t {
  Ok = 0,
  BaselineRequired,
  BaselineMismatch,
  NoUsableSubcarriers,
};

struct CsiPreprocessConfig {
  float epsilon{1.0e-4F};
  float mad_scale{1.4826F};
  float robust_z_limit{6.0F};
};

struct RobustBaseline {
  bool ready;
  uint16_t sample_count;
  int16_t subcarriers[kMaximumHtSubcarrierCount];
  float median[kMaximumHtSubcarrierCount];
  float mad[kMaximumHtSubcarrierCount];
  bool usable[kMaximumHtSubcarrierCount];
};

struct PreprocessedCsiSample {
  int16_t subcarrier;
  float log_amplitude;
  float centered_amplitude;
  float robust_z;
  bool valid;
};

struct PreprocessedCsiFrame {
  int64_t received_at_us;
  int8_t rssi;
  int8_t noise_floor;
  uint8_t channel;
  uint8_t bandwidth_mhz;
  uint16_t sample_count;
  uint16_t valid_sample_count;
  float frame_median;
  PreprocessedCsiSample samples[kMaximumHtSubcarrierCount];
};

class CsiPreprocessor final {
 public:
  explicit CsiPreprocessor(CsiPreprocessConfig config = {});

  CsiPreprocessStatus process(const ParsedCsiFrame &input, const RobustBaseline *baseline,
                              PreprocessedCsiFrame &output) const;

 private:
  static float median(float *values, std::size_t count);
  static float clamp(float value, float lower, float upper);
  bool baselineMatches(const ParsedCsiFrame &input, const RobustBaseline &baseline) const;

  CsiPreprocessConfig config_;
};

}  // namespace atom::radar
