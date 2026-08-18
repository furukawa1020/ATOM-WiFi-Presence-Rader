#pragma once

#include <cstdint>

#include <LocalDetector.hpp>

namespace atom::radar {

enum class LocalState : uint8_t {
  Initializing = 0,
  CalibrationRequired,
  LocalEmpty,
  LocalMotion,
  LocalPresent,
  LocalUncertain,
  LocalDegraded,
};

struct LocalStateInput {
  LocalDetectionResult detection;
  float health_score;
  bool calibration_valid;
  bool device_moved;
  bool recent_data_gap;
};

struct LocalStateMachineConfig {
  float motion_enter{0.70F};
  float motion_exit{0.45F};
  float presence_enter{0.70F};
  float presence_exit{0.45F};
  float minimum_health{0.60F};
  int64_t motion_enter_us{300000};
  int64_t motion_exit_us{1000000};
  int64_t presence_enter_us{2000000};
  int64_t empty_enter_us{10000000};
  int64_t uncertain_enter_us{500000};
  int64_t occupancy_memory_limit_us{1800000000LL};
};

struct LocalStateSnapshot {
  LocalState state;
  LocalState candidate_state;
  int64_t state_since_us;
  int64_t candidate_since_us;
  bool occupancy_memory;
};

class LocalStateMachine final {
 public:
  explicit LocalStateMachine(LocalStateMachineConfig config = {});

  LocalStateSnapshot update(const LocalStateInput &input, int64_t now_us);
  LocalStateSnapshot snapshot() const;
  void reset(int64_t now_us = 0);

 private:
  LocalState requestedState(const LocalStateInput &input, int64_t now_us);
  int64_t requiredDuration(LocalState requested) const;
  void forceState(LocalState state, int64_t now_us);

  LocalStateMachineConfig config_;
  LocalState state_{LocalState::Initializing};
  LocalState candidate_state_{LocalState::Initializing};
  int64_t state_since_us_{0};
  int64_t candidate_since_us_{0};
  int64_t last_update_us_{0};
  int64_t last_occupancy_evidence_us_{0};
  bool occupancy_memory_{false};
};

const char *localStateName(LocalState state);

}  // namespace atom::radar
