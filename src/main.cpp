#include <Arduino.h>
#include <cstring>
#include <CsiCapture.hpp>
#include <CsiFrameParser.hpp>
#include <CsiPreprocessor.hpp>
#include <CalibrationManager.hpp>
#include <CalibrationTrainer.hpp>
#include <CalibratedModelRuntime.hpp>
#include <CalibrationStorage.hpp>
#include <CalibrationBootstrap.hpp>
#include <CalibrationStartupController.hpp>
#include <AdaptiveTemporalEncoder.hpp>
#include <EspNowTransport.hpp>
#include <FeatureExtractor.hpp>
#include <LocalDetector.hpp>
#include <LocalStateMachine.hpp>
#include <M5AtomCsiPrecision.hpp>
#include <M5Unified.h>
#include <Preferences.h>
#include <RadioController.hpp>
#include <SubcarrierSelector.hpp>
#include <WiFi.h>
#include <protocol.hpp>

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

#ifndef RADIO_CHANNEL
#define RADIO_CHANNEL 6
#endif

#ifndef RADIO_BANDWIDTH_MHZ
#define RADIO_BANDWIDTH_MHZ 20
#endif

static_assert(RADIO_CHANNEL >= 1 && RADIO_CHANNEL <= 13, "RADIO_CHANNEL must be 1-13");
static_assert(RADIO_BANDWIDTH_MHZ == 20 || RADIO_BANDWIDTH_MHZ == 40,
              "RADIO_BANDWIDTH_MHZ must be 20 or 40");

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
static atom::radar::CsiFrameParser g_csi_parser;
static atom::radar::CsiPreprocessor g_csi_preprocessor;
static atom::radar::EspNowTransport g_esp_now;
static atom::radar::FeatureExtractor g_feature_extractor;
static atom::radar::LocalDetector g_local_detector;
static atom::radar::LocalStateMachine g_local_state_machine;
static atom::radar::M5AtomCsiLinkProcessor g_m5atom_csi_processor;
static atom::radar::M5AtomCsiFusion g_m5atom_csi_fusion;
static atom::radar::RadioController g_radio;
static atom::radar::SubcarrierSelection g_subcarrier_selection{};
static bool g_pairing_ready = false;
static uint32_t g_system_id = 0;
static uint8_t g_coordinator_mac[atom::radar::kWifiMacLength]{};
static atom::radar::CsiObservationPacket g_pending_csi_observation{};
static int64_t g_pending_csi_observation_at_us = 0;
static bool g_has_pending_csi_observation = false;

struct PairingConfig {
  uint32_t system_id;
  uint8_t coordinator_mac[atom::radar::kWifiMacLength];
  bool has_coordinator_mac;
};

struct ProbeReceiveCounters {
  uint32_t accepted;
  uint32_t invalid;
  uint32_t duplicates;
  uint32_t sequence_regressions;
};

static ProbeReceiveCounters g_probe_receive_counters{};
static bool g_has_probe_sequence = false;
static uint32_t g_last_probe_sequence = 0;

struct CsiProcessingCounters {
  uint32_t parsed_frames;
  uint32_t parse_rejects;
  uint32_t preprocessed_frames;
  uint32_t baseline_required_frames;
  uint32_t preprocess_rejects;
  uint32_t feature_updates;
  uint32_t selection_required_frames;
  uint32_t local_detection_updates;
  uint32_t precision_updates;
  uint32_t precision_collecting_frames;
  uint32_t precision_unmatched_frames;
  uint32_t observation_packets_queued;
  uint32_t observation_packets_replaced;
  uint32_t observation_packets_sent;
  uint32_t observation_packet_send_errors;
  uint32_t fused_observations;
  uint32_t fusion_rejects;
};

static CsiProcessingCounters g_csi_processing_counters{};

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

static atom::radar::RadioConfig buildRadioConfig() {
  atom::radar::RadioConfig config{};
  config.channel = RADIO_CHANNEL;
#if RADIO_BANDWIDTH_MHZ == 40
  config.bandwidth = WIFI_BW_HT40;
  config.secondary_channel = RADIO_CHANNEL <= 7 ? WIFI_SECOND_CHAN_ABOVE : WIFI_SECOND_CHAN_BELOW;
#else
  config.bandwidth = WIFI_BW_HT20;
  config.secondary_channel = WIFI_SECOND_CHAN_NONE;
#endif
  return config;
}

static PairingConfig loadPairingConfig() {
  PairingConfig config{};
  Preferences prefs;
  if (!prefs.begin("pairing", true)) {
    return config;
  }

  config.system_id = prefs.getUInt("system_id", 0);
  config.has_coordinator_mac =
      prefs.getBytesLength("coord_mac") == atom::radar::kWifiMacLength &&
      prefs.getBytes("coord_mac", config.coordinator_mac, atom::radar::kWifiMacLength) ==
          atom::radar::kWifiMacLength;
  prefs.end();
  return config;
}

static void setupTxRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing transmitter role...");
  const PairingConfig pairing = loadPairingConfig();
  g_system_id = pairing.system_id;

  const atom::radar::RadioConfig radio_config = buildRadioConfig();
  esp_err_t result = g_radio.begin(radio_config);
  if (result != ESP_OK) {
    Serial.printf("Radio initialization failed: %s\r\n", esp_err_to_name(result));
    showStatus("RADIO", "INITIALIZATION FAILED", TFT_RED);
    return;
  }

  result = g_esp_now.begin(radio_config.channel, radio_config.bandwidth, nullptr, true);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW initialization failed: %s\r\n", esp_err_to_name(result));
    showStatus("ESP-NOW", "INITIALIZATION FAILED", TFT_RED);
    return;
  }
  showStatus("TX / COORDINATOR", "PROBES READY", TFT_WHITE);
  Serial.printf("TX probe transport ready: channel=%u bandwidth=%uMHz system=%lu\r\n",
                radio_config.channel, RADIO_BANDWIDTH_MHZ,
                static_cast<unsigned long>(g_system_id));
}

static void setupReceiverRole(const RuntimeConfig &cfg) {
  (void)cfg;
  Serial.println("Initializing receiver role...");
  const PairingConfig pairing = loadPairingConfig();
  g_system_id = pairing.system_id;
  g_pairing_ready = pairing.has_coordinator_mac;
  if (g_pairing_ready) {
    std::memcpy(g_coordinator_mac, pairing.coordinator_mac, atom::radar::kWifiMacLength);
  }

  const atom::radar::RadioConfig radio_config = buildRadioConfig();
  esp_err_t result = g_radio.begin(radio_config);
  if (result != ESP_OK) {
    Serial.printf("Radio initialization failed: %s\r\n", esp_err_to_name(result));
    showStatus("RADIO", "INITIALIZATION FAILED", TFT_RED);
    return;
  }

  result = g_esp_now.begin(radio_config.channel, radio_config.bandwidth,
                           g_pairing_ready ? pairing.coordinator_mac : nullptr, false);
  if (result != ESP_OK) {
    Serial.printf("ESP-NOW initialization failed: %s\r\n", esp_err_to_name(result));
    showStatus("ESP-NOW", "INITIALIZATION FAILED", TFT_RED);
    return;
  }

  const esp_err_t csi_result =
      g_csi_capture.begin(g_pairing_ready ? pairing.coordinator_mac : nullptr);
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
                pairing.coordinator_mac[0], pairing.coordinator_mac[1], pairing.coordinator_mac[2],
                pairing.coordinator_mac[3], pairing.coordinator_mac[4], pairing.coordinator_mac[5]);
  showStatus("CALIBRATION", "REQUIRED", TFT_RED);
}

static void serviceCoordinatorProbes() {
  constexpr int64_t kProbeIntervalUs = 1000000LL / CSI_TARGET_SAMPLE_RATE_HZ;
  static int64_t next_probe_us = 0;
  static uint32_t sequence = 0;

  const int64_t now_us = esp_timer_get_time();
  if (next_probe_us == 0) {
    next_probe_us = now_us;
  }
  if (now_us < next_probe_us) {
    return;
  }

  const atom::radar::protocol::WireBandwidth bandwidth =
      RADIO_BANDWIDTH_MHZ == 40 ? atom::radar::protocol::WireBandwidth::Ht40
                                : atom::radar::protocol::WireBandwidth::Ht20;
  const atom::radar::protocol::ProbePacket packet = atom::radar::protocol::makeProbePacket(
      g_system_id, sequence, static_cast<uint64_t>(now_us), RADIO_CHANNEL, bandwidth);

  const esp_err_t result = g_esp_now.send(atom::radar::EspNowTransport::broadcastAddress(), &packet,
                                          sizeof(packet), now_us);
  if (result == ESP_OK) {
    ++sequence;
    next_probe_us += kProbeIntervalUs;
    if (now_us - next_probe_us > kProbeIntervalUs * 4) {
      next_probe_us = now_us + kProbeIntervalUs;
    }
  } else if (result != ESP_ERR_INVALID_STATE) {
    next_probe_us = now_us + kProbeIntervalUs;
  }

  static int64_t last_report_us = 0;
  if (now_us - last_report_us >= 1000000) {
    last_report_us = now_us;
    const atom::radar::EspNowCounters counters = g_esp_now.counters();
    Serial.printf(
        "{\"type\":\"probe_tx\",\"requested\":%lu,\"success\":%lu,\"failed\":%lu,"
        "\"timeouts\":%lu,\"immediate_errors\":%lu,\"pending\":%s}\r\n",
        static_cast<unsigned long>(counters.send_requests),
        static_cast<unsigned long>(counters.send_successes),
        static_cast<unsigned long>(counters.send_failures),
        static_cast<unsigned long>(counters.send_timeouts),
        static_cast<unsigned long>(counters.immediate_send_errors),
        g_esp_now.sendPending() ? "true" : "false");
  }
}

static void serviceReceiverPackets() {
  atom::radar::EspNowFrame frame{};
  while (g_esp_now.receive(frame)) {
    atom::radar::protocol::ProbePacket probe{};
    if (!atom::radar::protocol::decodeProbePacket(frame.data, frame.length, probe) ||
        (g_system_id != 0U && probe.system_id != g_system_id) || probe.channel != RADIO_CHANNEL ||
        probe.bandwidth != RADIO_BANDWIDTH_MHZ) {
      ++g_probe_receive_counters.invalid;
      continue;
    }

    if (g_has_probe_sequence) {
      if (probe.sequence == g_last_probe_sequence) {
        ++g_probe_receive_counters.duplicates;
        continue;
      }
      if (!atom::radar::protocol::isSequenceNewer(probe.sequence, g_last_probe_sequence)) {
        ++g_probe_receive_counters.sequence_regressions;
        continue;
      }
    }

    g_has_probe_sequence = true;
    g_last_probe_sequence = probe.sequence;
    g_m5atom_csi_processor.noteProbe(probe.sequence, probe.tx_uptime_us, frame.received_at_us);
    ++g_probe_receive_counters.accepted;
  }

  static int64_t last_report_us = 0;
  const int64_t now_us = esp_timer_get_time();
  if (now_us - last_report_us >= 1000000) {
    last_report_us = now_us;
    const atom::radar::EspNowCounters counters = g_esp_now.counters();
    Serial.printf(
        "{\"type\":\"probe_rx\",\"accepted\":%lu,\"invalid\":%lu,"
        "\"duplicates\":%lu,\"sequence_regressions\":%lu,\"filtered\":%lu,"
        "\"queue_drops\":%lu}\r\n",
        static_cast<unsigned long>(g_probe_receive_counters.accepted),
        static_cast<unsigned long>(g_probe_receive_counters.invalid),
        static_cast<unsigned long>(g_probe_receive_counters.duplicates),
        static_cast<unsigned long>(g_probe_receive_counters.sequence_regressions),
        static_cast<unsigned long>(counters.filtered_frames),
        static_cast<unsigned long>(counters.receive_queue_drops));
  }
}

static void serviceReceiverCapture() {
  atom::radar::CsiFrame frame{};
  while (g_csi_capture.receive(frame)) {
    atom::radar::ParsedCsiFrame parsed{};
    const atom::radar::CsiParseStatus parse_status =
        g_csi_parser.parse(frame, g_radio.config().bandwidth, parsed);
    if (parse_status != atom::radar::CsiParseStatus::Ok) {
      ++g_csi_processing_counters.parse_rejects;
      continue;
    }
    ++g_csi_processing_counters.parsed_frames;

    atom::radar::CsiLinkObservation precision_observation{};
    const atom::radar::CsiPrecisionUpdateStatus precision_status =
        g_m5atom_csi_processor.update(parsed, static_cast<uint8_t>(NODE_ID), precision_observation);
    if (precision_status == atom::radar::CsiPrecisionUpdateStatus::Updated) {
      ++g_csi_processing_counters.precision_updates;
      if (g_pairing_ready) {
        if (g_has_pending_csi_observation) {
          ++g_csi_processing_counters.observation_packets_replaced;
        }
        g_pending_csi_observation =
            atom::radar::makeCsiObservationPacket(g_system_id, precision_observation);
        g_pending_csi_observation_at_us =
            parsed.received_at_us + 1000LL + static_cast<int64_t>(NODE_ID) * 2000LL;
        g_has_pending_csi_observation = true;
        ++g_csi_processing_counters.observation_packets_queued;
      }
    } else if (precision_status == atom::radar::CsiPrecisionUpdateStatus::Collecting ||
               precision_status == atom::radar::CsiPrecisionUpdateStatus::CollectingBaseline) {
      ++g_csi_processing_counters.precision_collecting_frames;
    } else if (precision_status == atom::radar::CsiPrecisionUpdateStatus::UnmatchedProbe ||
               precision_status == atom::radar::CsiPrecisionUpdateStatus::NonMonotonicProbe) {
      ++g_csi_processing_counters.precision_unmatched_frames;
    }

    atom::radar::PreprocessedCsiFrame preprocessed{};
    const atom::radar::CsiPreprocessStatus preprocess_status =
        g_csi_preprocessor.process(parsed, nullptr, preprocessed);
    if (preprocess_status == atom::radar::CsiPreprocessStatus::BaselineRequired) {
      ++g_csi_processing_counters.baseline_required_frames;
    } else if (preprocess_status == atom::radar::CsiPreprocessStatus::Ok) {
      ++g_csi_processing_counters.preprocessed_frames;
      if (g_subcarrier_selection.count < atom::radar::kMinimumSelectedSubcarriers) {
        ++g_csi_processing_counters.selection_required_frames;
      } else {
        atom::radar::DetectionFeatures features{};
        if (g_feature_extractor.update(preprocessed, g_subcarrier_selection, features) ==
            atom::radar::FeatureUpdateStatus::Updated) {
          ++g_csi_processing_counters.feature_updates;
          const atom::radar::LocalDetectionResult detection =
              g_local_detector.evaluate(features, true, nullptr, nullptr);
          const atom::radar::LocalStateInput state_input{
              detection,
              features.quality_valid_ratio,
              true,
              false,
              false,
          };
          g_local_state_machine.update(state_input, features.timestamp_us);
          ++g_csi_processing_counters.local_detection_updates;
        }
      }
    } else {
      ++g_csi_processing_counters.preprocess_rejects;
    }
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
      "\"queued\":%lu,\"parsed\":%lu,\"parse_rejects\":%lu,"
      "\"baseline_required\":%lu,\"preprocess_rejects\":%lu,"
      "\"feature_updates\":%lu,\"selection_required\":%lu,"
      "\"local_detection_updates\":%lu,\"local_state\":\"%s\",\"paired\":%s}\r\n",
      static_cast<unsigned long>(counters.accepted_frames),
      static_cast<unsigned long>(counters.filtered_frames),
      static_cast<unsigned long>(counters.invalid_length_frames),
      static_cast<unsigned long>(counters.invalid_radio_frames),
      static_cast<unsigned long>(counters.queue_drops),
      static_cast<unsigned long>(counters.queued_frames),
      static_cast<unsigned long>(g_csi_processing_counters.parsed_frames),
      static_cast<unsigned long>(g_csi_processing_counters.parse_rejects),
      static_cast<unsigned long>(g_csi_processing_counters.baseline_required_frames),
      static_cast<unsigned long>(g_csi_processing_counters.preprocess_rejects),
      static_cast<unsigned long>(g_csi_processing_counters.feature_updates),
      static_cast<unsigned long>(g_csi_processing_counters.selection_required_frames),
      static_cast<unsigned long>(g_csi_processing_counters.local_detection_updates),
      atom::radar::localStateName(g_local_state_machine.snapshot().state),
      g_pairing_ready ? "true" : "false");
}

static void serviceReceiverObservationTx() {
  if (!g_has_pending_csi_observation || !g_pairing_ready) {
    return;
  }
  const int64_t now_us = esp_timer_get_time();
  if (now_us < g_pending_csi_observation_at_us || g_esp_now.sendPending()) {
    return;
  }
  const esp_err_t result =
      g_esp_now.send(g_coordinator_mac, &g_pending_csi_observation,
                     sizeof(g_pending_csi_observation), now_us);
  if (result == ESP_OK) {
    g_has_pending_csi_observation = false;
    ++g_csi_processing_counters.observation_packets_sent;
  } else if (result != ESP_ERR_INVALID_STATE) {
    g_has_pending_csi_observation = false;
    ++g_csi_processing_counters.observation_packet_send_errors;
  }
}

static void serviceCoordinatorObservations() {
  atom::radar::EspNowFrame frame{};
  while (g_esp_now.receive(frame)) {
    if (g_m5atom_csi_fusion.ingestPacket(frame.data, frame.length, g_system_id,
                                         frame.received_at_us)) {
      ++g_csi_processing_counters.fused_observations;
    } else {
      ++g_csi_processing_counters.fusion_rejects;
    }
  }

  static int64_t last_report_us = 0;
  const int64_t now_us = esp_timer_get_time();
  if (now_us - last_report_us < 1000000) {
    return;
  }
  last_report_us = now_us;

  atom::radar::FusedCsiObservation observation{};
  if (!g_m5atom_csi_fusion.snapshot(now_us, observation)) {
    Serial.printf("{\"type\":\"csi_fusion\",\"active_links\":0,\"accepted\":%lu,"
                  "\"rejected\":%lu}\r\n",
                  static_cast<unsigned long>(g_csi_processing_counters.fused_observations),
                  static_cast<unsigned long>(g_csi_processing_counters.fusion_rejects));
    return;
  }

  Serial.printf(
      "{\"type\":\"csi_fusion\",\"active_links\":%u,\"quality\":%.3f,"
      "\"agreement\":%.3f,\"synchronization\":%.3f,"
      "\"amplitude\":%.3f,\"phase\":%.3f,"
      "\"complex_ratio\":%.3f,\"subcarrier_reliability\":%.3f,"
      "\"delay_motion\":%.3f,\"delay_spread\":%.3f,"
      "\"dynamic_tap_concentration\":%.3f,\"doppler\":%.3f,"
      "\"doppler_centroid\":%.3f,\"doppler_bandwidth\":%.3f,"
      "\"doppler_asymmetry\":%.3f,\"respiration\":%.3f,"
      "\"respiration_coherence\":%.3f,\"impulse\":%.3f,\"stillness\":%.3f,"
      "\"baseline_shift\":%.3f,\"broadband_nuisance\":%.3f,"
      "\"physically_observable\":%s}\r\n",
      observation.active_links, observation.quality, observation.link_agreement,
      observation.synchronization_quality,
      observation.amplitude_motion, observation.differential_phase_motion,
      observation.complex_ratio_motion, observation.subcarrier_reliability,
      observation.delay_domain_motion, observation.delay_spread,
      observation.dynamic_tap_concentration, observation.doppler_energy,
      observation.doppler_centroid, observation.doppler_bandwidth,
      observation.doppler_asymmetry,
      observation.respiration_power, observation.respiration_coherence,
      observation.impulse_score, observation.stillness_score, observation.baseline_shift,
      observation.broadband_nuisance, observation.physically_observable ? "true" : "false");
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
  if (NODE_ROLE == static_cast<uint8_t>(NodeRole::TxCoordinator)) {
    if (g_esp_now.started()) {
      serviceCoordinatorObservations();
      serviceCoordinatorProbes();
    }
  } else {
    if (g_esp_now.started()) {
      serviceReceiverPackets();
    }
    if (g_csi_capture.started()) {
      serviceReceiverCapture();
    }
    if (g_esp_now.started()) {
      serviceReceiverObservationTx();
    }
  }
  delay(2);
}
