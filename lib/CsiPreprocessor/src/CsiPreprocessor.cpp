#include "CsiPreprocessor.hpp"

#include <algorithm>
#include <cmath>

namespace atom::radar {

CsiPreprocessor::CsiPreprocessor(CsiPreprocessConfig config) : config_(config) {}

CsiPreprocessStatus CsiPreprocessor::process(const ParsedCsiFrame &input,
                                             const RobustBaseline *baseline,
                                             PreprocessedCsiFrame &output) const {
  output = {};
  output.received_at_us = input.received_at_us;
  output.rssi = input.rssi;
  output.noise_floor = input.noise_floor;
  output.channel = input.channel;
  output.bandwidth_mhz = input.bandwidth_mhz;
  output.sample_count = input.sample_count;

  float median_scratch[kMaximumHtSubcarrierCount]{};
  std::size_t median_count = 0;

  for (std::size_t index = 0; index < input.sample_count; ++index) {
    const CsiComplexSample &source = input.samples[index];
    PreprocessedCsiSample &destination = output.samples[index];
    destination.subcarrier = source.subcarrier;
    destination.valid = source.valid_for_features;
    if (!destination.valid) {
      continue;
    }

    const float real = static_cast<float>(source.real);
    const float imaginary = static_cast<float>(source.imaginary);
    const float magnitude = std::sqrt((real * real) + (imaginary * imaginary));
    destination.log_amplitude = std::log(magnitude + config_.epsilon);
    if (!std::isfinite(destination.log_amplitude)) {
      destination.valid = false;
      continue;
    }
    median_scratch[median_count++] = destination.log_amplitude;
  }

  if (median_count == 0U) {
    return CsiPreprocessStatus::NoUsableSubcarriers;
  }

  output.frame_median = median(median_scratch, median_count);
  for (std::size_t index = 0; index < input.sample_count; ++index) {
    PreprocessedCsiSample &sample = output.samples[index];
    if (sample.valid) {
      sample.centered_amplitude = sample.log_amplitude - output.frame_median;
    }
  }

  if (baseline == nullptr || !baseline->ready) {
    return CsiPreprocessStatus::BaselineRequired;
  }
  if (!baselineMatches(input, *baseline)) {
    return CsiPreprocessStatus::BaselineMismatch;
  }

  for (std::size_t index = 0; index < input.sample_count; ++index) {
    PreprocessedCsiSample &sample = output.samples[index];
    if (!sample.valid || !baseline->usable[index] || !std::isfinite(baseline->median[index]) ||
        !std::isfinite(baseline->mad[index]) || baseline->mad[index] < 0.0F) {
      sample.valid = false;
      continue;
    }

    const float denominator = (config_.mad_scale * baseline->mad[index]) + config_.epsilon;
    const float robust_z = (sample.centered_amplitude - baseline->median[index]) / denominator;
    if (!std::isfinite(robust_z)) {
      sample.valid = false;
      continue;
    }
    sample.robust_z = clamp(robust_z, -config_.robust_z_limit, config_.robust_z_limit);
    ++output.valid_sample_count;
  }

  return output.valid_sample_count == 0U ? CsiPreprocessStatus::NoUsableSubcarriers
                                         : CsiPreprocessStatus::Ok;
}

float CsiPreprocessor::median(float *values, std::size_t count) {
  std::sort(values, values + count);
  const std::size_t middle = count / 2U;
  if ((count % 2U) != 0U) {
    return values[middle];
  }
  return (values[middle - 1U] + values[middle]) * 0.5F;
}

float CsiPreprocessor::clamp(float value, float lower, float upper) {
  return value < lower ? lower : (value > upper ? upper : value);
}

bool CsiPreprocessor::baselineMatches(const ParsedCsiFrame &input,
                                      const RobustBaseline &baseline) const {
  if (baseline.sample_count != input.sample_count) {
    return false;
  }
  for (std::size_t index = 0; index < input.sample_count; ++index) {
    if (baseline.subcarriers[index] != input.samples[index].subcarrier) {
      return false;
    }
  }
  return true;
}

}  // namespace atom::radar
