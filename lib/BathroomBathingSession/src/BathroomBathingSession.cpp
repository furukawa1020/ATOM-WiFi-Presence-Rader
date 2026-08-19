#include "BathroomBathingSession.hpp"

#include <Arduino.h>

#include <algorithm>
#include <cmath>

namespace atom::radar {

const char *bathroomSessionStateName(BathroomSessionState state) {
  switch (state) {
    case BathroomSessionState::Vacant:
      return "vacant";
    case BathroomSessionState::Entering:
      return "entering";
    case BathroomSessionState::WashFloor:
      return "wash_floor";
    case BathroomSessionState::Boundary:
      return "boundary";
    case BathroomSessionState::Bathtub:
      return "bathtub";
    case BathroomSessionState::Exiting:
      return "exiting";
    case BathroomSessionState::OccupiedUnknown:
    default:
      return "occupied_unknown";
  }
}

BathroomBathingSessionTracker::BathroomBathingSessionTracker(
    BathroomBathingSessionConfig config)
    : config_(config) {
  reset();
}

void BathroomBathingSessionTracker::reset() {
  for (std::size_t index = 0; index < kStateCount; ++index) {
    state_probability_[index] = 0.0F;
    state_before_probe_[index] = 0.0F;
  }
  state_probability_[0] = 1.0F;
  state_before_probe_[0] = 1.0F;
  has_probe_ = false;
  last_probe_sequence_ = 0;
  last_observed_at_us_ = -1;
  observations_ = 0;
  session_active_ = false;
  bathtub_interval_active_ = false;
  session_id_ = 0;
  session_started_at_us_ = -1;
  bathtub_interval_started_at_us_ = -1;
  entry_candidate_at_us_ = -1;
  exit_candidate_at_us_ = -1;
  bathtub_entry_candidate_at_us_ = -1;
  bathtub_exit_candidate_at_us_ = -1;
  accumulated_bathtub_us_ = 0;
  wash_floor_duration_us_ = 0;
  boundary_duration_us_ = 0;
  unobserved_duration_us_ = 0;
  last_completed_session_us_ = 0;
  last_completed_bathtub_us_ = 0;
  bathtub_intervals_ = 0;
  bathtub_reentries_ = 0;
  latest_entry_evidence_ = 0.0F;
  latest_exit_evidence_ = 0.0F;
  latest_state_confidence_ = 0.0F;
  latest_transition_confidence_ = 0.0F;
  last_emit_at_us_ = -1;
}

float BathroomBathingSessionTracker::clamp01(float value) {
  return std::clamp(value, 0.0F, 1.0F);
}

float BathroomBathingSessionTracker::maximum(float first, float second) {
  return first > second ? first : second;
}

uint32_t BathroomBathingSessionTracker::milliseconds(uint64_t microseconds) {
  return static_cast<uint32_t>(
      std::min<uint64_t>(microseconds / 1000ULL, UINT32_MAX));
}

float BathroomBathingSessionTracker::durationRamp(uint32_t value,
                                                  uint32_t start,
                                                  uint32_t full) {
  if (value <= start) {
    return 0.0F;
  }
  if (value >= full || full <= start) {
    return 1.0F;
  }
  return static_cast<float>(value - start) /
         static_cast<float>(full - start);
}

void BathroomBathingSessionTracker::updateStateProbabilities(
    const FusedCsiObservation &observation,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSafetyEvidence &safety_evidence,
    const BathroomSpatialEvidence &spatial_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    const BathroomRoomCalibrationEvidence &calibrated_evidence,
    float measurement_loss) {
  static constexpr float kTransition[kStateCount][kStateCount]{
      {0.930F, 0.055F, 0.000F, 0.000F, 0.000F, 0.000F, 0.015F},
      {0.020F, 0.620F, 0.250F, 0.070F, 0.000F, 0.020F, 0.020F},
      {0.005F, 0.010F, 0.870F, 0.080F, 0.010F, 0.015F, 0.010F},
      {0.000F, 0.010F, 0.140F, 0.620F, 0.200F, 0.010F, 0.020F},
      {0.000F, 0.000F, 0.010F, 0.100F, 0.860F, 0.000F, 0.030F},
      {0.160F, 0.020F, 0.100F, 0.040F, 0.000F, 0.660F, 0.020F},
      {0.050F, 0.040F, 0.160F, 0.140F, 0.160F, 0.050F, 0.400F},
  };

  const float human_motion =
      clamp01(bathroom_evidence.nuisance_reduced_human_motion);
  const float respiration = clamp01(
      calibrated_evidence.calibrated_respiration_evidence);
  const float door =
      clamp01(bathroom_evidence.door_transient_like_evidence);
  const float danger_context = maximum(
      safety_evidence.fall_sequence_evidence,
      maximum(calibrated_evidence.calibrated_dangerous_posture_evidence,
              respiration_evidence.respiration_loss_evidence));

  float likelihood[kStateCount]{};
  likelihood[0] = clamp01(
      (0.50F * calibrated_evidence.calibrated_empty_room_evidence +
       0.20F * (1.0F - human_motion) +
       0.15F * (1.0F - respiration) +
       0.15F * (1.0F - observation.innovation_motion)) *
      (1.0F - 0.80F * measurement_loss));
  likelihood[1] = clamp01(
      0.34F * door + 0.26F * human_motion +
      0.20F * calibrated_evidence.calibrated_boundary_evidence +
      0.12F * calibrated_evidence.calibrated_wash_floor_evidence +
      0.08F * respiration);
  likelihood[2] = clamp01(
      0.43F * calibrated_evidence.calibrated_wash_floor_evidence +
      0.20F * calibrated_evidence.calibrated_standing_evidence +
      0.15F * human_motion + 0.12F * bathroom_evidence.shower_like_evidence +
      0.10F * respiration);
  likelihood[3] = clamp01(
      0.52F * calibrated_evidence.calibrated_boundary_evidence +
      0.18F * human_motion + 0.15F * door +
      0.15F * spatial_evidence.position_uncertain_evidence);
  likelihood[4] = clamp01(
      0.42F * calibrated_evidence.calibrated_bathtub_evidence +
      0.20F * calibrated_evidence.calibrated_seated_evidence +
      0.14F * calibrated_evidence.calibrated_low_posture_evidence +
      0.12F * respiration +
      0.12F * bathroom_evidence.water_drift_like_evidence);
  likelihood[5] = clamp01(
      0.34F * door + 0.24F * calibrated_evidence.calibrated_boundary_evidence +
      0.20F * (1.0F - human_motion) + 0.12F * human_motion +
      0.10F * (1.0F - respiration));
  likelihood[6] = clamp01(
      0.38F * spatial_evidence.position_uncertain_evidence +
      0.34F * measurement_loss + 0.28F * danger_context);
  float likelihood_sum = 0.0F;
  for (float value : likelihood) {
    likelihood_sum += value + 0.02F;
  }
  for (float &value : likelihood) {
    value = (value + 0.02F) / likelihood_sum;
  }

  float predicted[kStateCount]{};
  for (std::size_t from = 0; from < kStateCount; ++from) {
    for (std::size_t to = 0; to < kStateCount; ++to) {
      float transition = kTransition[from][to];
      if (from == 4) {
        const float impact = clamp01(safety_evidence.impact_evidence);
        if (to == 2) {
          transition += 0.18F * impact;
        } else if (to == 6) {
          transition += 0.08F * impact;
        } else if (to == 4) {
          transition -= 0.26F * impact;
        }
      }
      predicted[to] += state_before_probe_[from] * transition;
    }
  }

  const float observation_quality = clamp01(
      0.25F * observation.quality +
      0.20F * spatial_evidence.analysis_quality +
      0.20F * respiration_evidence.analysis_quality +
      0.20F * calibrated_evidence.analysis_quality +
      0.15F * safety_evidence.analysis_quality);
  const float observation_weight =
      clamp01((0.15F + 0.75F * observation_quality) *
              (1.0F - 0.55F * measurement_loss));
  float posterior_sum = 0.0F;
  for (std::size_t state = 0; state < kStateCount; ++state) {
    const float neutral_likelihood =
        (1.0F - observation_weight) / static_cast<float>(kStateCount) +
        observation_weight * likelihood[state];
    state_probability_[state] =
        predicted[state] *
        (0.20F + static_cast<float>(kStateCount) * 0.80F *
                     neutral_likelihood);
    posterior_sum += state_probability_[state];
  }
  for (float &value : state_probability_) {
    value /= posterior_sum;
  }

  latest_entry_evidence_ =
      clamp01(0.55F * likelihood[1] * static_cast<float>(kStateCount) +
              0.45F * state_probability_[1]);
  latest_exit_evidence_ =
      clamp01(0.55F * likelihood[5] * static_cast<float>(kStateCount) +
              0.45F * state_probability_[5]);
  float highest = 0.0F;
  float second = 0.0F;
  for (float value : state_probability_) {
    if (value >= highest) {
      second = highest;
      highest = value;
    } else if (value > second) {
      second = value;
    }
  }
  latest_state_confidence_ = clamp01((highest - second) * 2.0F);
  latest_transition_confidence_ = clamp01(
      (1.0F - state_probability_[6]) * (1.0F - measurement_loss) *
      (0.40F + 0.60F * observation_quality));
}

void BathroomBathingSessionTracker::updateDurations(
    int64_t observed_at_us, float measurement_loss) {
  if (last_observed_at_us_ < 0 || observed_at_us <= last_observed_at_us_) {
    last_observed_at_us_ = observed_at_us;
    return;
  }
  const uint64_t elapsed_us =
      static_cast<uint64_t>(observed_at_us - last_observed_at_us_);
  last_observed_at_us_ = observed_at_us;
  if (!session_active_) {
    return;
  }
  if (elapsed_us > static_cast<uint64_t>(config_.maximum_observed_gap_us) ||
      measurement_loss > 0.55F) {
    unobserved_duration_us_ += elapsed_us;
    return;
  }
  wash_floor_duration_us_ +=
      static_cast<uint64_t>(static_cast<double>(elapsed_us) *
                            state_probability_[2]);
  boundary_duration_us_ +=
      static_cast<uint64_t>(static_cast<double>(elapsed_us) *
                            state_probability_[3]);
}

uint64_t BathroomBathingSessionTracker::currentBathtubDurationUs(
    int64_t observed_at_us) const {
  uint64_t duration = accumulated_bathtub_us_;
  if (bathtub_interval_active_ && bathtub_interval_started_at_us_ >= 0 &&
      observed_at_us > bathtub_interval_started_at_us_) {
    duration += static_cast<uint64_t>(observed_at_us -
                                      bathtub_interval_started_at_us_);
  }
  return duration;
}

void BathroomBathingSessionTracker::updateSessionLifecycle(
    int64_t observed_at_us, float measurement_loss, float danger_context) {
  const float occupancy = clamp01(
      state_probability_[1] + state_probability_[2] + state_probability_[3] +
      state_probability_[4] + state_probability_[5] +
      0.50F * state_probability_[6]);
  if (!session_active_) {
    if (occupancy >= 0.62F && measurement_loss < 0.55F &&
        state_probability_[6] < 0.55F) {
      if (entry_candidate_at_us_ < 0) {
        entry_candidate_at_us_ = observed_at_us;
      } else if (observed_at_us - entry_candidate_at_us_ >=
                 config_.entry_confirmation_us) {
        session_active_ = true;
        ++session_id_;
        session_started_at_us_ = entry_candidate_at_us_;
        accumulated_bathtub_us_ = 0;
        wash_floor_duration_us_ = 0;
        boundary_duration_us_ = 0;
        unobserved_duration_us_ = 0;
        bathtub_intervals_ = 0;
        bathtub_reentries_ = 0;
        entry_candidate_at_us_ = -1;
      }
    } else {
      entry_candidate_at_us_ = -1;
    }
    return;
  }

  if (state_probability_[0] >= 0.72F && measurement_loss < 0.35F &&
      danger_context < 0.22F) {
    if (exit_candidate_at_us_ < 0) {
      exit_candidate_at_us_ = observed_at_us;
    } else if (observed_at_us - exit_candidate_at_us_ >=
               config_.exit_confirmation_us) {
      const int64_t completed_at_us = exit_candidate_at_us_;
      if (bathtub_interval_active_) {
        accumulated_bathtub_us_ += static_cast<uint64_t>(
            maximum(0.0F,
                    static_cast<float>(completed_at_us -
                                       bathtub_interval_started_at_us_)));
        bathtub_interval_active_ = false;
        bathtub_interval_started_at_us_ = -1;
      }
      last_completed_session_us_ =
          completed_at_us > session_started_at_us_
              ? static_cast<uint64_t>(completed_at_us -
                                      session_started_at_us_)
              : 0;
      last_completed_bathtub_us_ = accumulated_bathtub_us_;
      session_active_ = false;
      session_started_at_us_ = -1;
      exit_candidate_at_us_ = -1;
      bathtub_entry_candidate_at_us_ = -1;
      bathtub_exit_candidate_at_us_ = -1;
      return;
    }
  } else {
    exit_candidate_at_us_ = -1;
  }

  if (!bathtub_interval_active_) {
    if (state_probability_[4] >= 0.56F && measurement_loss < 0.50F) {
      if (bathtub_entry_candidate_at_us_ < 0) {
        bathtub_entry_candidate_at_us_ = observed_at_us;
      } else if (observed_at_us - bathtub_entry_candidate_at_us_ >=
                 config_.bathtub_confirmation_us) {
        bathtub_interval_active_ = true;
        bathtub_interval_started_at_us_ = bathtub_entry_candidate_at_us_;
        if (bathtub_intervals_ > 0 && bathtub_reentries_ < UINT16_MAX) {
          ++bathtub_reentries_;
        }
        if (bathtub_intervals_ < UINT16_MAX) {
          ++bathtub_intervals_;
        }
        bathtub_entry_candidate_at_us_ = -1;
      }
    } else {
      bathtub_entry_candidate_at_us_ = -1;
    }
  } else {
    const bool left_bathtub =
        state_probability_[4] < 0.25F &&
        state_probability_[2] + state_probability_[3] > 0.50F &&
        measurement_loss < 0.40F;
    if (left_bathtub) {
      if (bathtub_exit_candidate_at_us_ < 0) {
        bathtub_exit_candidate_at_us_ = observed_at_us;
      } else if (observed_at_us - bathtub_exit_candidate_at_us_ >=
                 config_.bathtub_confirmation_us) {
        accumulated_bathtub_us_ += static_cast<uint64_t>(
            bathtub_exit_candidate_at_us_ - bathtub_interval_started_at_us_);
        bathtub_interval_active_ = false;
        bathtub_interval_started_at_us_ = -1;
        bathtub_exit_candidate_at_us_ = -1;
      }
    } else {
      bathtub_exit_candidate_at_us_ = -1;
    }
  }
}

void BathroomBathingSessionTracker::buildEvidence(
    uint32_t probe_sequence, int64_t observed_at_us,
    const BathroomSafetyEvidence &safety_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    const BathroomRoomCalibrationEvidence &calibrated_evidence,
    float measurement_loss, BathroomBathingSessionEvidence &evidence) const {
  evidence = {};
  evidence.probe_sequence = probe_sequence;
  evidence.observed_at_us = observed_at_us;
  evidence.session_id = session_id_;
  evidence.observations = observations_;
  evidence.vacant_probability = state_probability_[0];
  evidence.entering_probability = state_probability_[1];
  evidence.wash_floor_probability = state_probability_[2];
  evidence.boundary_probability = state_probability_[3];
  evidence.bathtub_probability = state_probability_[4];
  evidence.exiting_probability = state_probability_[5];
  evidence.occupied_unknown_probability = state_probability_[6];
  evidence.occupancy_evidence = clamp01(
      state_probability_[1] + state_probability_[2] + state_probability_[3] +
      state_probability_[4] + state_probability_[5] +
      0.50F * state_probability_[6]);
  evidence.entry_evidence = latest_entry_evidence_;
  evidence.exit_evidence = latest_exit_evidence_;
  evidence.state_confidence = latest_state_confidence_;
  evidence.transition_confidence = latest_transition_confidence_;
  evidence.measurement_loss_alternative = measurement_loss;

  std::size_t dominant = 0;
  for (std::size_t index = 1; index < kStateCount; ++index) {
    if (state_probability_[index] > state_probability_[dominant]) {
      dominant = index;
    }
  }
  evidence.state = static_cast<BathroomSessionState>(dominant);
  const uint64_t bathtub_us =
      session_active_ ? currentBathtubDurationUs(observed_at_us) : 0;
  const uint64_t session_us =
      session_active_ && session_started_at_us_ >= 0 &&
              observed_at_us > session_started_at_us_
          ? static_cast<uint64_t>(observed_at_us - session_started_at_us_)
          : 0;
  evidence.session_duration_ms = milliseconds(session_us);
  evidence.bathtub_duration_ms = milliseconds(bathtub_us);
  evidence.wash_floor_duration_ms =
      session_active_ ? milliseconds(wash_floor_duration_us_) : 0;
  evidence.boundary_duration_ms =
      session_active_ ? milliseconds(boundary_duration_us_) : 0;
  evidence.unobserved_duration_ms =
      session_active_ ? milliseconds(unobserved_duration_us_) : 0;
  evidence.last_completed_session_duration_ms =
      milliseconds(last_completed_session_us_);
  evidence.last_completed_bathtub_duration_ms =
      milliseconds(last_completed_bathtub_us_);
  evidence.prolonged_bathtub_evidence = durationRamp(
      evidence.bathtub_duration_ms, config_.prolonged_bathtub_start_ms,
      config_.prolonged_bathtub_full_ms);
  evidence.prolonged_session_evidence = durationRamp(
      evidence.session_duration_ms, config_.prolonged_session_start_ms,
      config_.prolonged_session_full_ms);
  const float immobility = maximum(
      safety_evidence.dangerous_immobility_evidence,
      maximum(calibrated_evidence.calibrated_dangerous_posture_evidence,
              respiration_evidence.respiration_loss_evidence));
  evidence.bathtub_immobility_context = clamp01(
      state_probability_[4] * immobility *
      (0.40F + 0.60F * evidence.prolonged_bathtub_evidence));
  evidence.analysis_quality = clamp01(
      0.30F * calibrated_evidence.analysis_quality +
      0.25F * respiration_evidence.analysis_quality +
      0.25F * safety_evidence.analysis_quality +
      0.20F * latest_transition_confidence_);
  evidence.bathtub_intervals = bathtub_intervals_;
  evidence.bathtub_reentries = bathtub_reentries_;
  evidence.session_active = session_active_;
  evidence.bathtub_interval_active = bathtub_interval_active_;
  evidence.entry_pending = entry_candidate_at_us_ >= 0;
  evidence.exit_pending = exit_candidate_at_us_ >= 0;
  evidence.evidence_ready = observations_ >= 20;
}

BathroomBathingSessionUpdateStatus BathroomBathingSessionTracker::update(
    const FusedCsiObservation &observation, int64_t observed_at_us,
    const BathroomCsiEvidence &bathroom_evidence,
    const BathroomSafetyEvidence &safety_evidence,
    const BathroomSpatialEvidence &spatial_evidence,
    const BathroomRespirationEvidence &respiration_evidence,
    const BathroomRoomCalibrationEvidence &calibrated_evidence,
    BathroomBathingSessionEvidence &session_evidence) {
  if (observed_at_us < 0 ||
      observation.anchor_probe_sequence != bathroom_evidence.probe_sequence ||
      observation.anchor_probe_sequence != safety_evidence.probe_sequence ||
      observation.anchor_probe_sequence != spatial_evidence.probe_sequence ||
      observation.anchor_probe_sequence != respiration_evidence.probe_sequence ||
      observation.anchor_probe_sequence != calibrated_evidence.probe_sequence ||
      !std::isfinite(observation.quality)) {
    return BathroomBathingSessionUpdateStatus::InvalidObservation;
  }

  bool same_probe = false;
  if (has_probe_) {
    const int32_t sequence_delta = static_cast<int32_t>(
        observation.anchor_probe_sequence - last_probe_sequence_);
    if (sequence_delta < 0) {
      return BathroomBathingSessionUpdateStatus::NonMonotonicProbe;
    }
    same_probe = sequence_delta == 0;
  }
  if (!same_probe) {
    for (std::size_t index = 0; index < kStateCount; ++index) {
      state_before_probe_[index] = state_probability_[index];
    }
    has_probe_ = true;
    last_probe_sequence_ = observation.anchor_probe_sequence;
    if (observations_ < UINT32_MAX) {
      ++observations_;
    }
  }

  float measurement_loss = clamp01(
      0.65F * respiration_evidence.measurement_loss_alternative +
      0.20F * spatial_evidence.position_uncertain_evidence +
      0.15F * (1.0F - observation.quality));
  if (!observation.physically_observable) {
    measurement_loss = maximum(measurement_loss, 0.92F);
  }
  updateStateProbabilities(
      observation, bathroom_evidence, safety_evidence, spatial_evidence,
      respiration_evidence, calibrated_evidence, measurement_loss);

  if (!same_probe) {
    updateDurations(observed_at_us, measurement_loss);
    const float danger_context = maximum(
        safety_evidence.fall_sequence_evidence,
        maximum(safety_evidence.dangerous_immobility_evidence,
                maximum(
                    calibrated_evidence.calibrated_dangerous_posture_evidence,
                    respiration_evidence.respiration_loss_evidence)));
    updateSessionLifecycle(observed_at_us, measurement_loss, danger_context);
  }
  buildEvidence(observation.anchor_probe_sequence, observed_at_us,
                safety_evidence, respiration_evidence, calibrated_evidence,
                measurement_loss, session_evidence);
  emitJsonIfDue(session_evidence);
  return same_probe ? BathroomBathingSessionUpdateStatus::ReplacedSameProbe
                    : BathroomBathingSessionUpdateStatus::Updated;
}

void BathroomBathingSessionTracker::emitJsonIfDue(
    const BathroomBathingSessionEvidence &evidence) {
  if (last_emit_at_us_ >= 0 &&
      evidence.observed_at_us - last_emit_at_us_ < 1000000) {
    return;
  }
  last_emit_at_us_ = evidence.observed_at_us;
  Serial.printf(
      "{\"type\":\"bathroom_bathing_session\",\"probe\":%lu,"
      "\"session_id\":%lu,\"state\":\"%s\",\"vacant\":%.3f,"
      "\"entering\":%.3f,\"wash_floor\":%.3f,\"boundary\":%.3f,"
      "\"bathtub\":%.3f,\"exiting\":%.3f,\"occupied_unknown\":%.3f,"
      "\"occupancy\":%.3f,\"entry\":%.3f,\"exit\":%.3f,"
      "\"state_confidence\":%.3f,\"transition_confidence\":%.3f,"
      "\"measurement_loss\":%.3f,\"session_ms\":%lu,"
      "\"bathtub_ms\":%lu,\"wash_floor_ms\":%lu,"
      "\"boundary_ms\":%lu,\"unobserved_ms\":%lu,"
      "\"last_session_ms\":%lu,\"last_bathtub_ms\":%lu,"
      "\"bathtub_intervals\":%u,\"bathtub_reentries\":%u,"
      "\"prolonged_bathtub\":%.3f,\"prolonged_session\":%.3f,"
      "\"bathtub_immobility\":%.3f,\"quality\":%.3f,"
      "\"session_active\":%s,\"bathtub_active\":%s,"
      "\"entry_pending\":%s,\"exit_pending\":%s,\"ready\":%s}\r\n",
      static_cast<unsigned long>(evidence.probe_sequence),
      static_cast<unsigned long>(evidence.session_id),
      bathroomSessionStateName(evidence.state), evidence.vacant_probability,
      evidence.entering_probability, evidence.wash_floor_probability,
      evidence.boundary_probability, evidence.bathtub_probability,
      evidence.exiting_probability, evidence.occupied_unknown_probability,
      evidence.occupancy_evidence, evidence.entry_evidence,
      evidence.exit_evidence, evidence.state_confidence,
      evidence.transition_confidence,
      evidence.measurement_loss_alternative,
      static_cast<unsigned long>(evidence.session_duration_ms),
      static_cast<unsigned long>(evidence.bathtub_duration_ms),
      static_cast<unsigned long>(evidence.wash_floor_duration_ms),
      static_cast<unsigned long>(evidence.boundary_duration_ms),
      static_cast<unsigned long>(evidence.unobserved_duration_ms),
      static_cast<unsigned long>(evidence.last_completed_session_duration_ms),
      static_cast<unsigned long>(evidence.last_completed_bathtub_duration_ms),
      evidence.bathtub_intervals, evidence.bathtub_reentries,
      evidence.prolonged_bathtub_evidence,
      evidence.prolonged_session_evidence,
      evidence.bathtub_immobility_context, evidence.analysis_quality,
      evidence.session_active ? "true" : "false",
      evidence.bathtub_interval_active ? "true" : "false",
      evidence.entry_pending ? "true" : "false",
      evidence.exit_pending ? "true" : "false",
      evidence.evidence_ready ? "true" : "false");
}

}  // namespace atom::radar
