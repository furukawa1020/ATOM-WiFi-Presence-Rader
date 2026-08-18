#include "EspNowTransport.hpp"

#include <cstring>

extern "C" {
#include "esp_timer.h"
}

namespace atom::radar {

namespace {

constexpr uint8_t kBroadcastMac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
constexpr int64_t kSendCompletionTimeoutUs = 20000;

}  // namespace

EspNowTransport *EspNowTransport::active_instance_ = nullptr;

esp_err_t EspNowTransport::begin(uint8_t channel, wifi_bandwidth_t bandwidth,
                                 const uint8_t *allowed_source_mac, bool add_broadcast_peer) {
  if (started_ || active_instance_ != nullptr) {
    return ESP_ERR_INVALID_STATE;
  }

  queue_ = xQueueCreateStatic(kEspNowReceiveQueueDepth, sizeof(EspNowFrame), queue_storage_,
                              &queue_control_);
  if (queue_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  has_allowed_source_ = allowed_source_mac != nullptr;
  if (has_allowed_source_) {
    std::memcpy(allowed_source_mac_, allowed_source_mac, ESP_NOW_ETH_ALEN);
  }

  esp_err_t result = esp_now_init();
  if (result != ESP_OK) {
    return result;
  }

  active_instance_ = this;
  result = esp_now_register_recv_cb(&EspNowTransport::onReceive);
  if (result != ESP_OK) {
    active_instance_ = nullptr;
    return result;
  }

  result = esp_now_register_send_cb(&EspNowTransport::onSend);
  if (result != ESP_OK) {
    active_instance_ = nullptr;
    return result;
  }

  if (add_broadcast_peer) {
    result = addPeer(kBroadcastMac, channel, bandwidth);
    if (result != ESP_OK) {
      return result;
    }
  }

  if (has_allowed_source_) {
    result = addPeer(allowed_source_mac_, channel, bandwidth);
    if (result != ESP_OK) {
      return result;
    }
  }

  started_ = true;
  return ESP_OK;
}

esp_err_t EspNowTransport::send(const uint8_t *destination_mac, const void *payload,
                                std::size_t length, int64_t now_us) {
  if (!started_) {
    return ESP_ERR_INVALID_STATE;
  }
  if (destination_mac == nullptr || payload == nullptr || length == 0U ||
      length > protocol::kMaximumPacketBytes) {
    return ESP_ERR_INVALID_ARG;
  }

  if (send_pending_.load(std::memory_order_acquire)) {
    if (now_us - send_started_at_us_ < kSendCompletionTimeoutUs) {
      return ESP_ERR_INVALID_STATE;
    }
    send_pending_.store(false, std::memory_order_release);
    send_timeouts_.fetch_add(1, std::memory_order_relaxed);
  }

  send_started_at_us_ = now_us;
  send_pending_.store(true, std::memory_order_release);
  const esp_err_t result = esp_now_send(destination_mac, static_cast<const uint8_t *>(payload), length);
  if (result != ESP_OK) {
    send_pending_.store(false, std::memory_order_release);
    immediate_send_errors_.fetch_add(1, std::memory_order_relaxed);
    return result;
  }

  send_requests_.fetch_add(1, std::memory_order_relaxed);
  return ESP_OK;
}

bool EspNowTransport::receive(EspNowFrame &frame, TickType_t wait_ticks) {
  return queue_ != nullptr && xQueueReceive(queue_, &frame, wait_ticks) == pdTRUE;
}

bool EspNowTransport::started() const { return started_; }

bool EspNowTransport::sendPending() const {
  return send_pending_.load(std::memory_order_acquire);
}

EspNowCounters EspNowTransport::counters() const {
  return {
      received_frames_.load(std::memory_order_relaxed),
      null_frames_.load(std::memory_order_relaxed),
      filtered_frames_.load(std::memory_order_relaxed),
      invalid_length_frames_.load(std::memory_order_relaxed),
      receive_queue_drops_.load(std::memory_order_relaxed),
      send_requests_.load(std::memory_order_relaxed),
      send_successes_.load(std::memory_order_relaxed),
      send_failures_.load(std::memory_order_relaxed),
      send_timeouts_.load(std::memory_order_relaxed),
      immediate_send_errors_.load(std::memory_order_relaxed),
      queue_ == nullptr ? 0U : static_cast<uint32_t>(uxQueueMessagesWaiting(queue_)),
  };
}

const uint8_t *EspNowTransport::broadcastAddress() { return kBroadcastMac; }

void EspNowTransport::onReceive(const esp_now_recv_info_t *info, const uint8_t *data,
                                int data_length) {
  if (active_instance_ != nullptr) {
    active_instance_->captureReceived(info, data, data_length);
  }
}

void EspNowTransport::onSend(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  (void)info;
  if (active_instance_ == nullptr) {
    return;
  }

  active_instance_->send_pending_.store(false, std::memory_order_release);
  if (status == ESP_NOW_SEND_SUCCESS) {
    active_instance_->send_successes_.fetch_add(1, std::memory_order_relaxed);
  } else {
    active_instance_->send_failures_.fetch_add(1, std::memory_order_relaxed);
  }
}

esp_err_t EspNowTransport::addPeer(const uint8_t *peer_mac, uint8_t channel,
                                   wifi_bandwidth_t bandwidth) {
  if (!esp_now_is_peer_exist(peer_mac)) {
    esp_now_peer_info_t peer{};
    std::memcpy(peer.peer_addr, peer_mac, ESP_NOW_ETH_ALEN);
    peer.channel = channel;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;

    const esp_err_t add_result = esp_now_add_peer(&peer);
    if (add_result != ESP_OK) {
      return add_result;
    }
  }

  esp_now_rate_config_t rate{};
  rate.phymode = bandwidth == WIFI_BW_HT40 ? WIFI_PHY_MODE_HT40 : WIFI_PHY_MODE_HT20;
  rate.rate = WIFI_PHY_RATE_MCS0_LGI;
  rate.ersu = false;
  rate.dcm = false;
  return esp_now_set_peer_rate_config(peer_mac, &rate);
}

void EspNowTransport::captureReceived(const esp_now_recv_info_t *info, const uint8_t *data,
                                      int data_length) {
  if (info == nullptr || info->src_addr == nullptr || data == nullptr) {
    null_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (!matchesAllowedSource(info->src_addr)) {
    filtered_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  if (data_length <= 0 || static_cast<std::size_t>(data_length) > protocol::kMaximumPacketBytes) {
    invalid_length_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  EspNowFrame frame{};
  frame.received_at_us = esp_timer_get_time();
  std::memcpy(frame.source_mac, info->src_addr, ESP_NOW_ETH_ALEN);
  if (info->rx_ctrl != nullptr) {
    frame.rssi = info->rx_ctrl->rssi;
    frame.channel = info->rx_ctrl->channel;
  }
  frame.length = static_cast<uint16_t>(data_length);
  std::memcpy(frame.data, data, static_cast<std::size_t>(data_length));

  if (xQueueSend(queue_, &frame, 0) != pdTRUE) {
    receive_queue_drops_.fetch_add(1, std::memory_order_relaxed);
    return;
  }
  received_frames_.fetch_add(1, std::memory_order_relaxed);
}

bool EspNowTransport::matchesAllowedSource(const uint8_t *source_mac) const {
  return !has_allowed_source_ ||
         (source_mac != nullptr &&
          std::memcmp(source_mac, allowed_source_mac_, ESP_NOW_ETH_ALEN) == 0);
}

}  // namespace atom::radar
