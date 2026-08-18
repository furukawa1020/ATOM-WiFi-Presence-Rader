#include "RadioController.hpp"

#include <Arduino.h>
#include <WiFi.h>

namespace atom::radar {

esp_err_t RadioController::begin(const RadioConfig &config) {
  if (started_) {
    return ESP_ERR_INVALID_STATE;
  }
  if (config.channel < 1U || config.channel > 13U ||
      (config.bandwidth != WIFI_BW_HT20 && config.bandwidth != WIFI_BW_HT40)) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!WiFi.mode(WIFI_STA)) {
    return ESP_FAIL;
  }
  WiFi.disconnect(false, false);
  vTaskDelay(pdMS_TO_TICKS(20));

  esp_err_t result = esp_wifi_set_ps(WIFI_PS_NONE);
  if (result != ESP_OK) {
    return result;
  }

  result = esp_wifi_set_protocol(
      WIFI_IF_STA, static_cast<uint8_t>(WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
  if (result != ESP_OK) {
    return result;
  }

  result = esp_wifi_set_bandwidth(WIFI_IF_STA, config.bandwidth);
  if (result != ESP_OK) {
    return result;
  }

  result = esp_wifi_set_channel(config.channel, config.secondary_channel);
  if (result != ESP_OK) {
    return result;
  }

  config_ = config;
  started_ = true;
  return ESP_OK;
}

bool RadioController::started() const { return started_; }

const RadioConfig &RadioController::config() const { return config_; }

}  // namespace atom::radar
