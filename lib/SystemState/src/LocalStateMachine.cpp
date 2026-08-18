#include "LocalStateMachine.hpp"

#include <cmath>

namespace atom::radar {

LocalStateMachine::LocalStateMachine(LocalStateMachineConfig config) : config_(config) {}

LocalStateSnapshot LocalStateMachine::update(const LocalStateInput &input, int64_t now_us) {
  if (last_update_us_ != 0 && now_us <= last_update_us_) {
    return snapshot();
  }
  last_update_us_ = now_us;

  if (input.device_moved || !input.calibration_valid) {
    occupancy_memory_ = false;
    forceState(LocalState::CalibrationRequired, now_us);
    return snapshot();
  }
  if (!input.detection.baseline_valid || !std::isfinite(input.health_score) ||
      input.health_score < config_.minimum_health || input.recent_data_gap) {
    forceState(LocalState::LocalDegraded, now_us);
    return snapshot();
  }

  if (input.detection.motion_probability >= config_.motion_enter ||
      input.detection.presence_probability >= config_.presence_enter) {
    last_occupancy_evidence_us_ = now_us;
  }

  const LocalState requested = requestedState(input, now_us);
  if (requested == state_) {
    candidate_state_ = state_;
    candidate_since_us_ = now_us;
    return snapshot();
  }
  if (requested != candidate_state_) {
    candidate_state_ = requested;
    candidate_since_us_ = now_us;
    return snapshot();
  }
  if (now_us - candidate_since_us_ < requiredDuration(requested)) {
    return snapshot();
  }

  state_ = requested;
  state_since_us_ = now_us;
  candidate_state_ = state_;
  candidate_since_us_ = now_us;
  if (state_ == LocalState::LocalMotion || state_ == LocalState::LocalPresent) {
    occupancy_memory_ = true;
    last_occupancy_evidence_us_ = now_us;
  } else if (state_ == LocalState::LocalEmpty) {
    occupancy_memory_ = false;
  }
  return snapshot();
}

LocalStateSnapshot LocalStateMachine::snapshot() const {
  return {state_, candidate_state_, state_since_us_, candidate_since_us_, occupancy_memory_};
}

void LocalStateMachine::reset(int64_t now_us) {
  state_ = LocalState::Initializing;
  candidate_state_ = LocalState::Initializing;
  state_since_us_ = now_us;
  candidate_since_us_ = now_us;
  last_update_us_ = now_us;
  last_occupancy_evidence_us_ = 0;
  occupancy_memory_ = false;
}

LocalState LocalStateMachine::requestedState(const LocalStateInput &input, int64_t now_us) {
  if (input.detection.methods_disagree) {
    return LocalState::LocalUncertain;
  }

  const float motion_threshold =
      state_ == LocalState::LocalMotion ? config_.motion_exit : config_.motion_enter;
  if (input.detection.motion_probability >= motion_threshold) {
    return LocalState::LocalMotion;
  }

  if (state_ == LocalState::LocalMotion && occupancy_memory_) {
    return LocalState::LocalPresent;
  }

  const float presence_threshold =
      state_ == LocalState::LocalPresent ? config_.presence_exit : config_.presence_enter;
  if (input.detection.presence_probability >= presence_threshold) {
    return LocalState::LocalPresent;
  }

  if (occupancy_memory_) {
    if (last_occupancy_evidence_us_ != 0 &&
        now_us - last_occupancy_evidence_us_ > config_.occupancy_memory_limit_us) {
      occupancy_memory_ = false;
      return LocalState::LocalUncertain;
    }
    return LocalState::LocalEmpty;
  }
  return LocalState::LocalEmpty;
}

int64_t LocalStateMachine::requiredDuration(LocalState requested) const {
  switch (requested) {
    case LocalState::LocalMotion:
      return config_.motion_enter_us;
    case LocalState::LocalPresent:
      return state_ == LocalState::LocalMotion ? config_.motion_exit_us
                                               : config_.presence_enter_us;
    case LocalState::LocalEmpty:
      return config_.empty_enter_us;
    case LocalState::LocalUncertain:
      return config_.uncertain_enter_us;
    case LocalState::Initializing:
    case LocalState::CalibrationRequired:
    case LocalState::LocalDegraded:
      return 0;
  }
  return 0;
}

void LocalStateMachine::forceState(LocalState state, int64_t now_us) {
  state_ = state;
  candidate_state_ = state;
  state_since_us_ = now_us;
  candidate_since_us_ = now_us;
}

const char *localStateName(LocalState state) {
  switch (state) {
    case LocalState::Initializing:
      return "INITIALIZING";
    case LocalState::CalibrationRequired:
      return "CALIBRATION_REQUIRED";
    case LocalState::LocalEmpty:
      return "LOCAL_EMPTY";
    case LocalState::LocalMotion:
      return "LOCAL_MOTION";
    case LocalState::LocalPresent:
      return "LOCAL_PRESENT";
    case LocalState::LocalUncertain:
      return "LOCAL_UNCERTAIN";
    case LocalState::LocalDegraded:
      return "LOCAL_DEGRADED";
  }
  return "LOCAL_UNCERTAIN";
}

}  // namespace atom::radar
