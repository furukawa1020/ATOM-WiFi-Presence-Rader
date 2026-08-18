#include "SubcarrierSelector.hpp"

#include <algorithm>
#include <cmath>

namespace atom::radar {

namespace {

struct RankedSubcarrier {
  int16_t subcarrier;
  float score;
};

}  // namespace

SubcarrierSelector::SubcarrierSelector(SubcarrierSelectorConfig config) : config_(config) {}

SubcarrierSelectionStatus SubcarrierSelector::select(
    uint8_t receiver_id, const SubcarrierTrainingMetrics *metrics, std::size_t metrics_count,
    SubcarrierSelection &selection) const {
  selection = {};
  if (metrics == nullptr || metrics_count == 0U || metrics_count > kMaximumHtSubcarrierCount) {
    return SubcarrierSelectionStatus::InvalidMetrics;
  }
  if (config_.minimum_count < kMinimumSelectedSubcarriers ||
      config_.maximum_count > kMaximumSelectedSubcarriers ||
      config_.minimum_count > config_.target_count || config_.target_count > config_.maximum_count) {
    return SubcarrierSelectionStatus::InvalidConfiguration;
  }

  RankedSubcarrier ranked[kMaximumHtSubcarrierCount]{};
  std::size_t ranked_count = 0;
  for (std::size_t index = 0; index < metrics_count; ++index) {
    const SubcarrierTrainingMetrics &candidate = metrics[index];
    if (!candidate.eligible || !std::isfinite(candidate.empty_mad) ||
        !std::isfinite(candidate.still_separation) ||
        !std::isfinite(candidate.motion_separation) ||
        !std::isfinite(candidate.missing_ratio) || !std::isfinite(candidate.outlier_ratio) ||
        !std::isfinite(candidate.temporal_noise) ||
        !std::isfinite(candidate.validation_reproducibility) ||
        !std::isfinite(candidate.maximum_correlation) ||
        candidate.missing_ratio > config_.maximum_missing_ratio ||
        candidate.outlier_ratio > config_.maximum_outlier_ratio ||
        candidate.validation_reproducibility < config_.minimum_reproducibility) {
      continue;
    }

    const float candidate_score = score(candidate);
    if (candidate_score <= 0.0F || !std::isfinite(candidate_score)) {
      continue;
    }
    ranked[ranked_count++] = {candidate.subcarrier, candidate_score};
  }

  if (ranked_count < config_.minimum_count) {
    return SubcarrierSelectionStatus::InsufficientCandidates;
  }

  std::sort(ranked, ranked + ranked_count,
            [](const RankedSubcarrier &left, const RankedSubcarrier &right) {
              return left.score == right.score ? left.subcarrier < right.subcarrier
                                               : left.score > right.score;
            });

  selection.receiver_id = receiver_id;
  selection.count = static_cast<uint16_t>(
      std::min<std::size_t>(config_.target_count, std::min<std::size_t>(ranked_count,
                                                                        config_.maximum_count)));
  for (std::size_t index = 0; index < selection.count; ++index) {
    selection.subcarriers[index] = ranked[index].subcarrier;
    selection.scores[index] = ranked[index].score;
  }
  return SubcarrierSelectionStatus::Ok;
}

float SubcarrierSelector::score(const SubcarrierTrainingMetrics &metrics) const {
  const float separation =
      0.5F * (std::max(0.0F, metrics.still_separation) +
              std::max(0.0F, metrics.motion_separation));
  const float stability =
      1.0F / (1.0F + std::max(0.0F, metrics.empty_mad) +
              std::max(0.0F, metrics.temporal_noise));
  const float availability =
      (1.0F - clamp01(metrics.missing_ratio)) * (1.0F - clamp01(metrics.outlier_ratio));
  const float reproducibility = clamp01(metrics.validation_reproducibility);
  const float non_redundancy = 1.0F - clamp01(std::fabs(metrics.maximum_correlation));
  return separation * stability * availability * reproducibility * non_redundancy;
}

float SubcarrierSelector::clamp01(float value) {
  return value < 0.0F ? 0.0F : (value > 1.0F ? 1.0F : value);
}

}  // namespace atom::radar
