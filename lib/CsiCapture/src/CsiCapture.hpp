#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern "C" {
#include "esp_err.h"
#include "esp_wifi.h"
}

#if !defined(CONFIG_ESP_WIFI_CSI_ENABLED) || !CONFIG_ESP_WIFI_CSI_ENABLED
#error "The selected Arduino-ESP32 core was built without Wi-Fi CSI support."
#endif

namespace atom::radar {

constexpr std::size_t kWifiMacLength = 6;
constexpr std::size_t kMaxCsiBytes = 612;
constexpr UBaseType_t kCsiQueueDepth = 48;

struct CsiFrame {
  int64_t received_at_us;
  wifi_pkt_rx_ctrl_t rx_ctrl;
  uint8_t source_mac[kWifiMacLength];
  bool first_word_invalid;
  uint16_t length;
  int8_t data[kMaxCsiBytes];
};

struct CsiCaptureCounters {
  uint32_t accepted_frames;
  uint32_t null_frames;
  uint32_t filtered_frames;
  uint32_t invalid_length_frames;
  uint32_t invalid_radio_frames;
  uint32_t queue_drops;
  uint32_t queued_frames;
};

class CsiCapture final {
 public:
  CsiCapture() = default;

  CsiCapture(const CsiCapture &) = delete;
  CsiCapture &operator=(const CsiCapture &) = delete;

  esp_err_t begin(const uint8_t *coordinator_mac);
  bool receive(CsiFrame &frame, TickType_t wait_ticks = 0);
  bool started() const;
  bool hasCoordinatorFilter() const;
  CsiCaptureCounters counters() const;

 private:
  static void onCsiFrame(void *context, wifi_csi_info_t *info);
  void capture(const wifi_csi_info_t *info);
  bool matchesCoordinator(const uint8_t *source_mac) const;

  alignas(4) uint8_t queue_storage_[kCsiQueueDepth * sizeof(CsiFrame)]{};
  StaticQueue_t queue_control_{};
  QueueHandle_t queue_{nullptr};
  uint8_t coordinator_mac_[kWifiMacLength]{};
  bool has_coordinator_filter_{false};
  bool started_{false};

  std::atomic<uint32_t> accepted_frames_{0};
  std::atomic<uint32_t> null_frames_{0};
  std::atomic<uint32_t> filtered_frames_{0};
  std::atomic<uint32_t> invalid_length_frames_{0};
  std::atomic<uint32_t> invalid_radio_frames_{0};
  std::atomic<uint32_t> queue_drops_{0};
};

}  // namespace atom::radar
