#pragma once

#include <cstdint>

enum class CalibrationStartupState : uint8_t {
  Idle = 0,
  Loading,
  Active,
  RetryWait,
  ActiveRetryWait,
  Degraded,
  ActiveDegraded,
};

struct CalibrationRetryPolicy {
  uint32_t initial_delay_ms = 1000;
  uint32_t maximum_delay_ms = 60000;
  uint16_t maximum_auto_attempts = 0;
};
