#include "CsiCapture.hpp"

#include <cstring>

extern "C" {
#include "esp_timer.h"
}

namespace atom::radar {

namespace {

constexpr uint16_t kMinimumCsiBytes = 2;

wifi_csi_config_t makeCsiConfig() {
  wifi_csi_config_t config{};
  config.lltf_en = false;
  config.htltf_en = true;
  config.stbc_htltf2_en = false;
  config.ltf_merge_en = false;
  config.channel_filter_en = false;
  config.manu_scale = false;
  config.shift = 0;
  config.dump_ack_en = false;
  return config;
}

}  // namespace

esp_err_t CsiCapture::begin(const uint8_t *coordinator_mac) {
  if (started_) {
    return ESP_ERR_INVALID_STATE;
  }

  queue_ = xQueueCreateStatic(kCsiQueueDepth, sizeof(CsiFrame), queue_storage_, &queue_control_);
  if (queue_ == nullptr) {
    return ESP_ERR_NO_MEM;
  }

  has_coordinator_filter_ = coordinator_mac != nullptr;
  if (has_coordinator_filter_) {
    std::memcpy(coordinator_mac_, coordinator_mac, kWifiMacLength);
  }

  esp_err_t result = esp_wifi_set_promiscuous(true);
  if (result != ESP_OK) {
    return result;
  }

  const wifi_csi_config_t config = makeCsiConfig();
  result = esp_wifi_set_csi_config(&config);
  if (result != ESP_OK) {
    return result;
  }

  result = esp_wifi_set_csi_rx_cb(&CsiCapture::onCsiFrame, this);
  if (result != ESP_OK) {
    return result;
  }

  result = esp_wifi_set_csi(true);
  if (result != ESP_OK) {
    return result;
  }

  started_ = true;
  return ESP_OK;
}

bool CsiCapture::receive(CsiFrame &frame, TickType_t wait_ticks) {
  return queue_ != nullptr && xQueueReceive(queue_, &frame, wait_ticks) == pdTRUE;
}

bool CsiCapture::started() const { return started_; }

bool CsiCapture::hasCoordinatorFilter() const { return has_coordinator_filter_; }

CsiCaptureCounters CsiCapture::counters() const {
  return {
      accepted_frames_.load(std::memory_order_relaxed),
      null_frames_.load(std::memory_order_relaxed),
      filtered_frames_.load(std::memory_order_relaxed),
      invalid_length_frames_.load(std::memory_order_relaxed),
      invalid_radio_frames_.load(std::memory_order_relaxed),
      queue_drops_.load(std::memory_order_relaxed),
      queue_ == nullptr ? 0U : static_cast<uint32_t>(uxQueueMessagesWaiting(queue_)),
  };
}

void CsiCapture::onCsiFrame(void *context, wifi_csi_info_t *info) {
  if (context == nullptr) {
    return;
  }
  static_cast<CsiCapture *>(context)->capture(info);
}

void CsiCapture::capture(const wifi_csi_info_t *info) {
  if (info == nullptr || info->buf == nullptr) {
    null_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (!matchesCoordinator(info->mac)) {
    filtered_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (info->rx_ctrl.rx_state != 0) {
    invalid_radio_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (info->len < kMinimumCsiBytes || info->len > kMaxCsiBytes || (info->len % 2U) != 0U ||
      (info->first_word_invalid && info->len < 4U)) {
    invalid_length_frames_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  CsiFrame frame{};
  frame.received_at_us = esp_timer_get_time();
  frame.rx_ctrl = info->rx_ctrl;
  std::memcpy(frame.source_mac, info->mac, kWifiMacLength);
  frame.first_word_invalid = info->first_word_invalid;
  frame.length = info->len;
  std::memcpy(frame.data, info->buf, info->len);

  if (xQueueSend(queue_, &frame, 0) != pdTRUE) {
    queue_drops_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  accepted_frames_.fetch_add(1, std::memory_order_relaxed);
}

bool CsiCapture::matchesCoordinator(const uint8_t *source_mac) const {
  return has_coordinator_filter_ && source_mac != nullptr &&
         std::memcmp(source_mac, coordinator_mac_, kWifiMacLength) == 0;
}

}  // namespace atom::radar
