#pragma once

#include <cstdint>

extern "C" {
#include "esp_err.h"
#include "esp_wifi.h"
}

namespace atom::radar {

struct RadioConfig {
  uint8_t channel;
  wifi_bandwidth_t bandwidth;
  wifi_second_chan_t secondary_channel;
};

class RadioController final {
 public:
  esp_err_t begin(const RadioConfig &config);
  bool started() const;
  const RadioConfig &config() const;

 private:
  RadioConfig config_{};
  bool started_{false};
};

}  // namespace atom::radar
