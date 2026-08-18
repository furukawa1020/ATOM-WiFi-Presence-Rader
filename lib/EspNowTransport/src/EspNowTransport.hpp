#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

extern "C" {
#include "esp_err.h"
#include "esp_now.h"
#include "esp_wifi.h"
}

#include <protocol.hpp>

namespace atom::radar {

constexpr UBaseType_t kEspNowReceiveQueueDepth = 32;

struct EspNowFrame {
  int64_t received_at_us;
  uint8_t source_mac[ESP_NOW_ETH_ALEN];
  int8_t rssi;
  uint8_t channel;
  uint16_t length;
  uint8_t data[protocol::kMaximumPacketBytes];
};

struct EspNowCounters {
  uint32_t received_frames;
  uint32_t null_frames;
  uint32_t filtered_frames;
  uint32_t invalid_length_frames;
  uint32_t receive_queue_drops;
  uint32_t send_requests;
  uint32_t send_successes;
  uint32_t send_failures;
  uint32_t send_timeouts;
  uint32_t immediate_send_errors;
  uint32_t queued_frames;
};

class EspNowTransport final {
 public:
  EspNowTransport() = default;

  EspNowTransport(const EspNowTransport &) = delete;
  EspNowTransport &operator=(const EspNowTransport &) = delete;

  esp_err_t begin(uint8_t channel, wifi_bandwidth_t bandwidth, const uint8_t *allowed_source_mac,
                  bool add_broadcast_peer);
  esp_err_t send(const uint8_t *destination_mac, const void *payload, std::size_t length,
                 int64_t now_us);
  bool receive(EspNowFrame &frame, TickType_t wait_ticks = 0);
  bool started() const;
  bool sendPending() const;
  EspNowCounters counters() const;

  static const uint8_t *broadcastAddress();

 private:
  static void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int data_length);
  static void onSend(const esp_now_send_info_t *info, esp_now_send_status_t status);

  esp_err_t addPeer(const uint8_t *peer_mac, uint8_t channel, wifi_bandwidth_t bandwidth);
  void captureReceived(const esp_now_recv_info_t *info, const uint8_t *data, int data_length);
  bool matchesAllowedSource(const uint8_t *source_mac) const;

  static EspNowTransport *active_instance_;

  alignas(4) uint8_t queue_storage_[kEspNowReceiveQueueDepth * sizeof(EspNowFrame)]{};
  StaticQueue_t queue_control_{};
  QueueHandle_t queue_{nullptr};
  uint8_t allowed_source_mac_[ESP_NOW_ETH_ALEN]{};
  bool has_allowed_source_{false};
  bool started_{false};
  int64_t send_started_at_us_{0};
  std::atomic<bool> send_pending_{false};

  std::atomic<uint32_t> received_frames_{0};
  std::atomic<uint32_t> null_frames_{0};
  std::atomic<uint32_t> filtered_frames_{0};
  std::atomic<uint32_t> invalid_length_frames_{0};
  std::atomic<uint32_t> receive_queue_drops_{0};
  std::atomic<uint32_t> send_requests_{0};
  std::atomic<uint32_t> send_successes_{0};
  std::atomic<uint32_t> send_failures_{0};
  std::atomic<uint32_t> send_timeouts_{0};
  std::atomic<uint32_t> immediate_send_errors_{0};
};

}  // namespace atom::radar
