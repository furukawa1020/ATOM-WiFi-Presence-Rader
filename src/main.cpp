#include <Arduino.h>
#include <M5Unified.h>
#include <WiFi.h>
#include <Preferences.h>

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

enum class OccupancyState : uint8_t {
  Empty = 0,
  Movement,
  PresentStill,
  Uncertain,
  Degraded
};

struct RuntimeConfig {
  NodeRole role;
  uint8_t node_id;
  uint32_t app_version;
};

static constexpr const char *kRoleNames[] = {"TX_COORDINATOR", "RECEIVER"};
static constexpr const char *kStateNames[] = {"EMPTY", "MOVEMENT", "PRESENT_STILL", "UNCERTAIN", "DEGRADED"};

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

#ifndef CONFIG_ESP_WIFI_CSI_ENABLED
#warning "Wi-Fi CSI may be disabled in this ESP32-COMPONENT build."
#endif

static void initNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  delay(20);
}

static void initEmptyTelemetry() {
  Preferences prefs;
  bool ok = prefs.begin("radar", true);
  if (!ok) {
    Serial.println("Preferences open for read failed.");
  } else {
    Serial.println("Preferences opened for read.");
  }
  prefs.end();
}

static void setupTxRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing transmitter role...");
  initNetwork();
  Serial.println("TX role runtime skeleton ready.");
}

static void setupReceiverRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing receiver role...");
  initNetwork();
  Serial.println("RX role runtime skeleton ready.");
}

static OccupancyState computePlaceholderState(uint32_t tick_count) {
  const uint32_t cycle = tick_count % 1000;
  if (cycle < 850) {
    return OccupancyState::Empty;
  }
  if (cycle < 900) {
    return OccupancyState::Movement;
  }
  if (cycle < 950) {
    return OccupancyState::PresentStill;
  }
  return OccupancyState::Uncertain;
}

void setup() {
  Serial.begin(115200);
  delay(100);
  auto cfg = buildRuntimeConfig();

  M5.begin();
  printBootInfo(cfg);
  initEmptyTelemetry();

  if (cfg.role == NodeRole::TxCoordinator) {
    setupTxRole(cfg);
  } else {
    setupReceiverRole(cfg);
  }
}

void loop() {
  static uint32_t tick = 0;
  static uint64_t last_ms = 0;
  uint64_t now_ms = esp_timer_get_time() / 1000;
  if (now_ms - last_ms >= 1000) {
    last_ms = now_ms;
    OccupancyState state = computePlaceholderState(tick++);
    Serial.printf("tick=%u state=%s\r\n", tick, kStateNames[static_cast<uint8_t>(state)]);
  }
  delay(10);
}
