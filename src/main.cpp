#include <Arduino.h>
#include <CsiCapture.hpp>
#include <M5Unified.h>
#include <Preferences.h>
#include <WiFi.h>

extern "C" {
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
}

#ifndef APP_PROTOCOL_VERSION
#define APP_PROTOCOL_VERSION 1
#endif

#ifndef NODE_ROLE
#define NODE_ROLE 1
#endif

#ifndef NODE_ID
#define NODE_ID 1
#endif

enum class NodeRole : uint8_t {
  TxCoordinator = 0,
  Receiver = 1
};

struct RuntimeConfig {
  NodeRole role;
  uint8_t node_id;
  uint32_t app_version;
};

static constexpr const char *kRoleNames[] = {"TX_COORDINATOR", "RECEIVER"};
static atom::radar::CsiCapture g_csi_capture;
static bool g_pairing_ready = false;

static RuntimeConfig buildRuntimeConfig() {
  RuntimeConfig cfg{};
  cfg.role = (NODE_ROLE == 0) ? NodeRole::TxCoordinator : NodeRole::Receiver;
  cfg.node_id = static_cast<uint8_t>(NODE_ID);
  cfg.app_version = APP_PROTOCOL_VERSION;
  return cfg;
}

static void printBootInfo(const RuntimeConfig &cfg) {
  Serial.printf("ATOM Wi-Fi Presence Radar bootstrap\r\n");
  Serial.printf("role=%s id=%u protocol=%u\r\n", kRoleNames[static_cast<uint8_t>(cfg.role)],
                cfg.node_id, cfg.app_version);
}

static void showStatus(const char *line_one, const char *line_two, uint32_t color) {
  M5.Display.clear(TFT_BLACK);
  M5.Display.setTextColor(color, TFT_BLACK);
  M5.Display.setTextSize(1);
  M5.Display.setCursor(8, 48);
  M5.Display.println(line_one);
  M5.Display.setCursor(8, 66);
  M5.Display.println(line_two);
}

static esp_err_t initNetwork() {
  if (!WiFi.mode(WIFI_STA)) {
    return ESP_FAIL;
  }
  WiFi.disconnect(false, false);
  delay(20);
  return esp_wifi_set_ps(WIFI_PS_NONE);
}

static bool loadCoordinatorMac(uint8_t *coordinator_mac) {
  Preferences prefs;
  if (!prefs.begin("pairing", true)) {
    return false;
  }

  const bool valid_length = prefs.getBytesLength("coord_mac") == atom::radar::kWifiMacLength;
  const bool loaded = valid_length &&
                      prefs.getBytes("coord_mac", coordinator_mac, atom::radar::kWifiMacLength) ==
                          atom::radar::kWifiMacLength;
  prefs.end();
  return loaded;
}

static void setupTxRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing transmitter role...");
  const esp_err_t result = initNetwork();
  if (result != ESP_OK) {
    Serial.printf("Wi-Fi initialization failed: %s\r\n", esp_err_to_name(result));
    showStatus("WIFI", "INITIALIZATION FAILED", TFT_RED);
    return;
  }
  showStatus("TX / COORDINATOR", "READY", TFT_WHITE);
  Serial.println("TX role runtime skeleton ready.");
}

static void setupReceiverRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing receiver role...");
  const esp_err_t network_result = initNetwork();
  if (network_result != ESP_OK) {
    Serial.printf("Wi-Fi initialization failed: %s\r\n", esp_err_to_name(network_result));
    showStatus("WIFI", "INITIALIZATION FAILED", TFT_RED);
    return;
  }

  uint8_t coordinator_mac[atom::radar::kWifiMacLength]{};
  g_pairing_ready = loadCoordinatorMac(coordinator_mac);

  const esp_err_t csi_result = g_csi_capture.begin(g_pairing_ready ? coordinator_mac : nullptr);
  if (csi_result != ESP_OK) {
    Serial.printf("CSI initialization failed: %s\r\n", esp_err_to_name(csi_result));
    showStatus("CSI", "UNSUPPORTED", TFT_RED);
    return;
  }

  if (!g_pairing_ready) {
    Serial.println("CSI capture ready; Coordinator MAC is not paired.");
    showStatus("PAIRING", "REQUIRED", TFT_YELLOW);
    return;
  }

  Serial.printf("CSI capture ready for Coordinator %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                coordinator_mac[0], coordinator_mac[1], coordinator_mac[2], coordinator_mac[3],
                coordinator_mac[4], coordinator_mac[5]);
  showStatus("CSI CAPTURE", "READY", TFT_WHITE);
}

static void serviceReceiverCapture() {
  atom::radar::CsiFrame frame{};
  while (g_csi_capture.receive(frame)) {
    // Parsing and signal processing intentionally belong to later pipeline stages.
  }

  static int64_t last_report_us = 0;
  const int64_t now_us = esp_timer_get_time();
  if (now_us - last_report_us < 1000000) {
    return;
  }
  last_report_us = now_us;

  const atom::radar::CsiCaptureCounters counters = g_csi_capture.counters();
  Serial.printf(
      "{\"type\":\"csi_capture\",\"accepted\":%lu,\"filtered\":%lu,"
      "\"invalid_length\":%lu,\"invalid_radio\":%lu,\"queue_drops\":%lu,"
      "\"queued\":%lu,\"paired\":%s}\r\n",
      static_cast<unsigned long>(counters.accepted_frames),
      static_cast<unsigned long>(counters.filtered_frames),
      static_cast<unsigned long>(counters.invalid_length_frames),
      static_cast<unsigned long>(counters.invalid_radio_frames),
      static_cast<unsigned long>(counters.queue_drops),
      static_cast<unsigned long>(counters.queued_frames), g_pairing_ready ? "true" : "false");
}

void setup() {
  Serial.begin(115200);
  delay(100);
  auto cfg = buildRuntimeConfig();

  M5.begin();
  showStatus("INITIALIZING", "PLEASE WAIT", TFT_WHITE);
  printBootInfo(cfg);

  if (cfg.role == NodeRole::TxCoordinator) {
    setupTxRole(cfg);
  } else {
    setupReceiverRole(cfg);
  }
}

void loop() {
  M5.update();
  if (NODE_ROLE == static_cast<uint8_t>(NodeRole::Receiver) && g_csi_capture.started()) {
    serviceReceiverCapture();
  }
  delay(2);
}
