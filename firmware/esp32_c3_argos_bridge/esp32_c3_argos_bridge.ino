#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

#include "wifi_secrets.h"

// ESP32-C3 Super Mini: D7/GPIO20 is RX, D6/GPIO21 is TX.
// ESP D7 RX <- Nucleo D1 / PA2 / USART2_TX
// ESP D6 TX -> Nucleo D0 / PA3 / USART2_RX
constexpr int STM32_RX_PIN = 20;
constexpr int STM32_TX_PIN = 21;
constexpr uint32_t STM32_BAUD = 115200;

constexpr char FALLBACK_AP_NAME[] = "ARGOS";
constexpr uint16_t TCP_PORT = 3333;
constexpr uint16_t DISCOVERY_PORT = 3334;
constexpr uint16_t UPDATE_PORT = 3335;
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 12000U;
constexpr uint32_t STA_RECOVERY_TIMEOUT_MS = 15000U;
constexpr uint32_t STA_RECONNECT_PERIOD_MS = 5000U;
constexpr uint32_t BRIDGE_STATUS_PERIOD_MS = 5000U;
constexpr uint32_t REPEATED_WIFI_LOG_PERIOD_MS = 60000U;
constexpr size_t EVENT_LOG_LIMIT = 3072U;

struct WifiDisconnectDiagnostic {
  uint8_t reason;
};

HardwareSerial Stm32Serial(1);
WiFiServer server(TCP_PORT);
WebServer updateServer(UPDATE_PORT);
WiFiUDP discovery;
WiFiClient panel;
Preferences preferences;
QueueHandle_t wifiDiagnosticQueue = nullptr;

bool panelActive = false;
bool stm32Seen = false;
bool stm32OnlineLogPending = false;
uint32_t lastStm32ReceiveMs = 0U;
bool fallbackApActive = false;
wl_status_t lastWifiStatus = WL_NO_SHIELD;
uint32_t wifiDisconnectedSinceMs = 0U;
uint32_t lastReconnectAttemptMs = 0U;
uint32_t lastProbeMs = 0U;
uint32_t lastWaitingReportMs = 0U;
uint32_t lastBridgeStatusMs = 0U;
uint32_t stm32RxBytes = 0U;
uint32_t lastWifiStateLogMs = 0U;
uint32_t bootId = 0U;
uint32_t nextEventId = 1U;
uint32_t disconnectReasonLogMs[256] = {0U};
uint16_t disconnectReasonSuppressed[256] = {0U};
String pendingEvents;
String panelLine;
String stm32TelemetryLine;
bool stm32TelemetryOverflow = false;
File firmwareUploadFile;
bool firmwareUploadReady = false;
String firmwareUploadError;
uint32_t firmwareExpectedSize = 0U;
uint32_t firmwareExpectedCrc = 0U;
uint32_t firmwareBuildId = 0U;
uint32_t firmwareUploadBytes = 0U;
uint32_t firmwareUploadRunningCrc = 0xFFFFFFFFU;

uint32_t crc32Update(uint32_t crc, const uint8_t *data, size_t length) {
  while (length-- > 0U) {
    crc ^= *data++;
    for (uint8_t bit = 0U; bit < 8U; bit++) {
      crc = (crc >> 1U) ^ ((crc & 1U) ? 0xEDB88320U : 0U);
    }
  }
  return crc;
}

uint32_t crc32Bytes(const uint8_t *data, size_t length) {
  return ~crc32Update(0xFFFFFFFFU, data, length);
}

String activeIp() {
  return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString()
                                         : WiFi.softAPIP().toString();
}

const char *activeMode() {
  return (WiFi.status() == WL_CONNECTED) ? "STA" : "AP";
}

const char *resetReasonName(esp_reset_reason_t reason) {
  switch (reason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXTERNAL";
    case ESP_RST_SW: return "SOFTWARE";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    case ESP_RST_USB: return "USB";
    case ESP_RST_JTAG: return "JTAG";
    case ESP_RST_EFUSE: return "EFUSE";
    case ESP_RST_PWR_GLITCH: return "POWER_GLITCH";
    case ESP_RST_CPU_LOCKUP: return "CPU_LOCKUP";
    default: return "UNKNOWN";
  }
}

const char *wifiDisconnectReasonName(uint8_t reason) {
  switch (reason) {
    case 2: return "AUTH_EXPIRE";
    case 4: return "ASSOC_EXPIRE";
    case 8: return "ASSOC_LEAVE";
    case 15: return "HANDSHAKE_TIMEOUT";
    case 36: return "STA_LEAVING";
    case 200: return "BEACON_TIMEOUT";
    case 201: return "NO_AP_FOUND";
    case 202: return "AUTH_FAIL";
    case 203: return "ASSOC_FAIL";
    case 204: return "HANDSHAKE_TIMEOUT";
    case 205: return "CONNECTION_FAIL";
    default: return "OTHER";
  }
}

void onWifiDisconnected(WiFiEvent_t event, WiFiEventInfo_t info) {
  (void)event;
  if (wifiDiagnosticQueue == nullptr) {
    return;
  }
  WifiDisconnectDiagnostic diagnostic;
  diagnostic.reason = info.wifi_sta_disconnected.reason;
  (void)xQueueSend(wifiDiagnosticQueue, &diagnostic, 0U);
}

void persistEvents() {
  preferences.putString("events", pendingEvents);
  preferences.putUInt("next_event", nextEventId);
}

void trimEventLog() {
  while (pendingEvents.length() > EVENT_LOG_LIMIT) {
    int newline = pendingEvents.indexOf('\n');
    if (newline < 0) {
      pendingEvents = "";
      return;
    }
    pendingEvents.remove(0, newline + 1);
  }
}

void logEvent(const char *level, const char *code, const String &detail) {
  String clean = detail;
  clean.replace("\r", " ");
  clean.replace("\n", " ");
  clean.replace("|", "/");
  pendingEvents += String(nextEventId++) + "|" + String(bootId) + "|" +
                   String(millis()) + "|" + level + "|" + code + "|" + clean + "\n";
  trimEventLog();
  persistEvents();
}

void drainWifiDiagnostics() {
  if (wifiDiagnosticQueue == nullptr) {
    return;
  }

  WifiDisconnectDiagnostic diagnostic;
  while (xQueueReceive(wifiDiagnosticQueue, &diagnostic, 0U) == pdTRUE) {
    uint32_t now = millis();
    uint32_t lastLog = disconnectReasonLogMs[diagnostic.reason];
    if ((lastLog != 0U) && ((now - lastLog) < REPEATED_WIFI_LOG_PERIOD_MS)) {
      if (disconnectReasonSuppressed[diagnostic.reason] < UINT16_MAX) {
        disconnectReasonSuppressed[diagnostic.reason]++;
      }
      continue;
    }
    uint16_t repeated = disconnectReasonSuppressed[diagnostic.reason];
    disconnectReasonSuppressed[diagnostic.reason] = 0U;
    disconnectReasonLogMs[diagnostic.reason] = now;
    logEvent("WARN", "WIFI_DISCONNECT",
             "reason=" + String(diagnostic.reason) + " " +
             wifiDisconnectReasonName(diagnostic.reason) +
             " repeated=" + String(repeated));
  }
}

void flushSuppressedWifiDisconnects(const char *context) {
  for (uint16_t reason = 0U; reason < 256U; reason++) {
    uint16_t suppressed = disconnectReasonSuppressed[reason];
    if (suppressed > 0U) {
      logEvent("INFO", "WIFI_DISCONNECT_SUMMARY",
               "reason=" + String(reason) + " " + wifiDisconnectReasonName((uint8_t)reason) +
               " suppressed=" + String(suppressed) + " " + context);
    }
    disconnectReasonSuppressed[reason] = 0U;
    disconnectReasonLogMs[reason] = 0U;
  }
}

void acknowledgeEvents(uint32_t acknowledgedId) {
  String retained;
  int start = 0;
  while (start < (int)pendingEvents.length()) {
    int end = pendingEvents.indexOf('\n', start);
    if (end < 0) {
      end = pendingEvents.length();
    }
    String row = pendingEvents.substring(start, end);
    uint32_t eventId = (uint32_t)row.toInt();
    if ((eventId == 0U) || (eventId > acknowledgedId)) {
      retained += row + "\n";
    }
    start = end + 1;
  }
  pendingEvents = retained;
  persistEvents();
}

void flushEventsToPanel() {
  if (!panelActive || !panel.connected() || (pendingEvents.length() == 0U)) {
    return;
  }

  int start = 0;
  while (start < (int)pendingEvents.length()) {
    int end = pendingEvents.indexOf('\n', start);
    if (end < 0) {
      end = pendingEvents.length();
    }
    String row = pendingEvents.substring(start, end);
    if (row.length() > 0U) {
      panel.print("ESP LOG | ");
      panel.println(row);
    }
    start = end + 1;
  }
}

void sendBridgeStatus(const char *prefix) {
  if (!panelActive || !panel.connected()) {
    return;
  }
  int32_t rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  panel.printf("%s | uptime:%lu | heap:%lu | rssi:%ld | net:%s | ip:%s | stm:%s\r\n",
               prefix,
               (unsigned long)millis(),
               (unsigned long)ESP.getFreeHeap(),
               (long)rssi,
               activeMode(),
               activeIp().c_str(),
               stm32Seen ? "ONLINE" : "WAITING");
}

void notifyStm32(bool connected) {
  Stm32Serial.printf("W %s %s\r\n", activeMode(), activeIp().c_str());
  Stm32Serial.print(connected ? "N 1\r\n" : "N 0\r\n");
}

void startFallbackAp() {
  if (fallbackApActive) {
    return;
  }
  WiFi.mode(WIFI_AP_STA);
  fallbackApActive = WiFi.softAP(FALLBACK_AP_NAME, ARGOS_FALLBACK_AP_PASSWORD, 6, false, 1);
  logEvent(fallbackApActive ? "WARN" : "ERROR", "AP_FALLBACK",
           fallbackApActive ? WiFi.softAPIP().toString() : "START_FAILED");
}

void stopFallbackAp() {
  if (!fallbackApActive) {
    return;
  }
  WiFi.softAPdisconnect(true);
  fallbackApActive = false;
  WiFi.mode(WIFI_STA);
  logEvent("INFO", "AP_FALLBACK_STOP", WiFi.localIP().toString());
}

void handleWifiState() {
  wl_status_t status = WiFi.status();
  if (status != lastWifiStatus) {
    uint32_t now = millis();
    bool shouldLog = (status == WL_CONNECTED) || (lastWifiStatus == WL_CONNECTED) ||
                     ((now - lastWifiStateLogMs) >= REPEATED_WIFI_LOG_PERIOD_MS);
    if (shouldLog) {
      logEvent((status == WL_CONNECTED) ? "INFO" : "WARN", "WIFI_STATE",
               String((int)lastWifiStatus) + "->" + String((int)status));
      lastWifiStateLogMs = now;
    }
    lastWifiStatus = status;

    if (status == WL_CONNECTED) {
      flushSuppressedWifiDisconnects("RECOVERED");
      wifiDisconnectedSinceMs = 0U;
      stopFallbackAp();
      notifyStm32(panelActive);
    } else if (wifiDisconnectedSinceMs == 0U) {
      wifiDisconnectedSinceMs = millis();
    }
  }

  if (status != WL_CONNECTED) {
    if ((millis() - lastReconnectAttemptMs) >= STA_RECONNECT_PERIOD_MS) {
      WiFi.reconnect();
      lastReconnectAttemptMs = millis();
    }
    if ((wifiDisconnectedSinceMs != 0U) &&
        ((millis() - wifiDisconnectedSinceMs) >= STA_RECOVERY_TIMEOUT_MS)) {
      startFallbackAp();
    }
  }
}

void handleDiscovery() {
  int packetSize = discovery.parsePacket();
  if (packetSize <= 0) {
    return;
  }

  char request[32] = {0};
  int length = discovery.read(request, sizeof(request) - 1);
  if ((length > 0) && (strncmp(request, "ARGOS_DISCOVER", 14) == 0)) {
    discovery.beginPacket(discovery.remoteIP(), discovery.remotePort());
    discovery.printf("ARGOS_BRIDGE|%u|%s|%s|%u", TCP_PORT, activeMode(),
                     activeIp().c_str(), UPDATE_PORT);
    discovery.endPacket();
  }
}

void reportFirmwareProgress(const String &message) {
  logEvent("INFO", "STM32_UPDATE", message);
  if (panelActive && panel.connected()) {
    panel.println("ARGOS UPDATE | " + message);
  }
}

bool readStm32Line(String &line, uint32_t timeoutMs) {
  line = "";
  uint32_t startedAt = millis();
  while ((millis() - startedAt) < timeoutMs) {
    while (Stm32Serial.available()) {
      char value = (char)Stm32Serial.read();
      if (value == '\n') {
        line.trim();
        if (line.length() > 0U) {
          return true;
        }
      } else if ((value != '\r') && (line.length() < 120U)) {
        line += value;
      }
    }
    delay(1);
  }
  line.trim();
  return line.length() > 0U;
}

bool waitForStm32Prefix(const char *prefix, uint32_t timeoutMs, String &matched) {
  uint32_t startedAt = millis();
  while ((millis() - startedAt) < timeoutMs) {
    String line;
    uint32_t remaining = timeoutMs - (millis() - startedAt);
    uint32_t slice = (remaining < 500U) ? remaining : 500U;
    if (!readStm32Line(line, slice)) {
      continue;
    }
    if (panelActive && panel.connected()) {
      panel.println("ARGOS UPDATE STM | " + line);
    }
    if (line.startsWith(prefix)) {
      matched = line;
      return true;
    }
    if (line.startsWith("BL ERROR")) {
      matched = line;
      return false;
    }
  }
  matched = "TIMEOUT waiting for " + String(prefix);
  return false;
}

void clearStm32Input() {
  while (Stm32Serial.available()) {
    (void)Stm32Serial.read();
  }
}

bool enterStm32Bootloader(String &detail) {
  Stm32Serial.print("x\r\n");
  delay(80);
  clearStm32Input();
  Stm32Serial.print("U ARGOS_OTA\r\n");
  delay(180);

  uint32_t startedAt = millis();
  while ((millis() - startedAt) < 8000U) {
    Stm32Serial.print("BL SYNC\r\n");
    if (waitForStm32Prefix("BL READY", 450U, detail)) {
      return true;
    }
    delay(50);
  }
  detail = "bootloader did not answer; install bootstrap once through ST-LINK";
  return false;
}

bool programStm32FromCache(String &result) {
  stm32TelemetryLine = "";
  stm32TelemetryOverflow = false;
  File image = LittleFS.open("/stm32.bin", "r");
  if (!image) {
    result = "cached image missing";
    return false;
  }
  const uint32_t imageSize = (uint32_t)image.size();
  if ((imageSize == 0U) || (imageSize != firmwareExpectedSize)) {
    image.close();
    result = "cached image size mismatch";
    return false;
  }

  reportFirmwareProgress("BOOTLOADER REQUEST");
  String reply;
  if (!enterStm32Bootloader(reply)) {
    image.close();
    result = reply;
    logEvent("ERROR", "STM32_UPDATE_BOOT", result);
    return false;
  }

  Stm32Serial.printf("BL BEGIN %lu %lu %lu\r\n",
                     (unsigned long)imageSize,
                     (unsigned long)firmwareExpectedCrc,
                     (unsigned long)firmwareBuildId);
  if (!waitForStm32Prefix("BL ERASED", 30000U, reply)) {
    image.close();
    result = reply;
    logEvent("ERROR", "STM32_UPDATE_ERASE", result);
    return false;
  }

  uint8_t data[256];
  uint32_t offset = 0U;
  uint8_t lastPercent = 0U;
  while (offset < imageSize) {
    uint32_t remaining = imageSize - offset;
    size_t request = (remaining < sizeof(data)) ? remaining : sizeof(data);
    size_t count = image.read(data, request);
    if (count == 0U) {
      image.close();
      result = "cache read failed";
      return false;
    }
    uint32_t chunkCrc = crc32Bytes(data, count);
    Stm32Serial.printf("BL DATA %lu %u %lu\r\n",
                       (unsigned long)offset, (unsigned int)count,
                       (unsigned long)chunkCrc);
    Stm32Serial.write(data, count);

    String expectedAck = "BL ACK " + String(offset + count);
    if (!waitForStm32Prefix(expectedAck.c_str(), 4000U, reply)) {
      image.close();
      result = reply;
      logEvent("ERROR", "STM32_UPDATE_CHUNK", "offset=" + String(offset) + " " + result);
      return false;
    }
    offset += (uint32_t)count;
    uint8_t percent = (uint8_t)((offset * 100UL) / imageSize);
    if ((percent >= (uint8_t)(lastPercent + 10U)) || (percent == 100U)) {
      lastPercent = percent;
      reportFirmwareProgress("WRITE " + String(percent) + "%");
    }
  }
  image.close();

  Stm32Serial.print("BL END\r\n");
  if (!waitForStm32Prefix("BL OK", 15000U, reply)) {
    result = reply;
    logEvent("ERROR", "STM32_UPDATE_VERIFY", result);
    return false;
  }

  result = "PASS size=" + String(imageSize) + " crc=" + String(firmwareExpectedCrc, HEX);
  reportFirmwareProgress("PASS - STM32 RESTARTING");
  logEvent("INFO", "STM32_UPDATE_PASS", result);
  stm32Seen = false;
  return true;
}

void handleFirmwareUpload() {
  HTTPUpload &upload = updateServer.upload();
  if (upload.status == UPLOAD_FILE_START) {
    firmwareUploadReady = false;
    firmwareUploadError = "";
    firmwareUploadBytes = 0U;
    firmwareUploadRunningCrc = 0xFFFFFFFFU;
    firmwareExpectedSize = (uint32_t)strtoul(
        updateServer.header("X-ARGOS-Size").c_str(), nullptr, 10);
    firmwareExpectedCrc = (uint32_t)strtoul(
        updateServer.header("X-ARGOS-CRC32").c_str(), nullptr, 10);
    firmwareBuildId = (uint32_t)strtoul(
        updateServer.header("X-ARGOS-Build").c_str(), nullptr, 10);
    if (updateServer.header("X-ARGOS-Confirm") != "STOPPED") {
      firmwareUploadError = "missing safety confirmation";
      return;
    }
    LittleFS.remove("/stm32.bin");
    firmwareUploadFile = LittleFS.open("/stm32.bin", "w");
    if (!firmwareUploadFile) {
      firmwareUploadError = "LittleFS open failed";
    }
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (firmwareUploadError.length() == 0U) {
      size_t written = firmwareUploadFile.write(upload.buf, upload.currentSize);
      if (written != upload.currentSize) {
        firmwareUploadError = "LittleFS write failed";
      } else {
        firmwareUploadBytes += (uint32_t)written;
        firmwareUploadRunningCrc = crc32Update(
            firmwareUploadRunningCrc, upload.buf, upload.currentSize);
      }
    }
  } else if (upload.status == UPLOAD_FILE_END) {
    if (firmwareUploadFile) {
      firmwareUploadFile.close();
    }
    uint32_t actualCrc = ~firmwareUploadRunningCrc;
    if ((firmwareUploadError.length() == 0U) &&
        (firmwareUploadBytes == firmwareExpectedSize) &&
        (actualCrc == firmwareExpectedCrc)) {
      firmwareUploadReady = true;
      logEvent("INFO", "STM32_IMAGE_CACHED",
               "size=" + String(firmwareUploadBytes) + " crc=" + String(actualCrc, HEX));
    } else if (firmwareUploadError.length() == 0U) {
      firmwareUploadError = "upload size/CRC mismatch";
    }
  } else if (upload.status == UPLOAD_FILE_ABORTED) {
    if (firmwareUploadFile) {
      firmwareUploadFile.close();
    }
    firmwareUploadError = "upload aborted";
    LittleFS.remove("/stm32.bin");
  }
}

void finishFirmwarePost() {
  if (!firmwareUploadReady) {
    updateServer.send(400, "text/plain", "CACHE FAIL: " + firmwareUploadError);
    return;
  }
  String result;
  bool ok = programStm32FromCache(result);
  updateServer.send(ok ? 200 : 500, "text/plain", ok ? "UPDATE PASS: " + result
                                                       : "UPDATE FAIL: " + result);
  firmwareUploadReady = false;
}

void handlePanelControl(const String &line) {
  if (line == "@ESP PING") {
    sendBridgeStatus("ESP PONG");
    return;
  }
  if (line == "@ESP LOGS") {
    flushEventsToPanel();
    return;
  }
  if (line.startsWith("@ESP ACK ")) {
    acknowledgeEvents((uint32_t)line.substring(9).toInt());
    return;
  }

  Stm32Serial.print(line);
  Stm32Serial.print("\r\n");
}

void readPanelCommands() {
  while (panel.available()) {
    char value = (char)panel.read();
    if ((value == '\r') || (value == '\n')) {
      if (panelLine.length() > 0U) {
        handlePanelControl(panelLine);
        panelLine = "";
      }
    } else if (panelLine.length() < 95U) {
      panelLine += value;
    } else {
      panelLine = "";
      logEvent("ERROR", "PANEL_LINE", "TOO_LONG");
    }
  }
}

void setup() {
  // Keep startup/status bursts while connection logs are persisted and sent.
  Stm32Serial.setRxBufferSize(4096U);
  Stm32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

  preferences.begin("argos", false);
  pendingEvents = preferences.getString("events", "");
  nextEventId = preferences.getUInt("next_event", 1U);
  bootId = preferences.getUInt("boot_id", 0U) + 1U;
  preferences.putUInt("boot_id", bootId);
  logEvent("INFO", "BOOT", resetReasonName(esp_reset_reason()));

  WiFi.persistent(false);
  wifiDiagnosticQueue = xQueueCreate(8U, sizeof(WifiDisconnectDiagnostic));
  WiFi.onEvent(onWifiDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  bool sleepApplied = WiFi.setSleep(true);
  bool txPowerApplied = WiFi.setTxPower(WIFI_POWER_8_5dBm);
  logEvent("INFO", "RADIO_CONFIG",
           "tx=8.5dBm applied=" + String(txPowerApplied ? 1 : 0) +
           " sleep=" + String(sleepApplied ? 1 : 0));
  WiFi.begin(ARGOS_WIFI_SSID, ARGOS_WIFI_PASSWORD);

  uint32_t startedAt = millis();
  while ((WiFi.status() != WL_CONNECTED) &&
         ((millis() - startedAt) < STA_CONNECT_TIMEOUT_MS)) {
    delay(100);
  }

  drainWifiDiagnostics();
  lastWifiStatus = WiFi.status();
  if (lastWifiStatus == WL_CONNECTED) {
    flushSuppressedWifiDisconnects("STARTUP_RECOVERED");
    logEvent("INFO", "WIFI_CONNECTED",
             WiFi.localIP().toString() + " RSSI=" + String(WiFi.RSSI()));
  } else {
    wifiDisconnectedSinceMs = millis();
    logEvent("WARN", "WIFI_TIMEOUT", String((int)lastWifiStatus));
    startFallbackAp();
  }

  bool littleFsReady = LittleFS.begin(true);
  logEvent(littleFsReady ? "INFO" : "ERROR", "LITTLEFS",
           littleFsReady ? "READY" : "MOUNT_FAILED");
  const char *updateHeaders[] = {
      "X-ARGOS-Size", "X-ARGOS-CRC32", "X-ARGOS-Build", "X-ARGOS-Confirm"};
  updateServer.collectHeaders(updateHeaders, 4U);
  updateServer.on("/stm32", HTTP_POST, finishFirmwarePost, handleFirmwareUpload);
  updateServer.on("/status", HTTP_GET, []() {
    updateServer.send(200, "text/plain",
                      "ARGOS UPDATE V1 | STM:" + String(stm32Seen ? "ONLINE" : "WAITING"));
  });
  updateServer.begin();

  server.begin();
  server.setNoDelay(true);
  discovery.begin(DISCOVERY_PORT);
  MDNS.begin("argos");
  MDNS.addService("argos", "tcp", TCP_PORT);
  notifyStm32(false);
}

void loop() {
  drainWifiDiagnostics();
  handleWifiState();
  handleDiscovery();
  updateServer.handleClient();

  if (panelActive && !panel.connected()) {
    panel.stop();
    panelActive = false;
    panelLine = "";
    Stm32Serial.print("x\r\n");
    notifyStm32(false);
    logEvent("WARN", "TCP_DISCONNECTED",
             "rx=" + String(stm32RxBytes) + " stm=" + String(stm32Seen ? 1 : 0));
    stm32Seen = false;
    stm32OnlineLogPending = false;
  }

  if (!panelActive) {
    WiFiClient candidate = server.available();
    if (candidate) {
      panel = candidate;
      panelActive = true;
      stm32Seen = false;
      stm32RxBytes = 0U;
      stm32OnlineLogPending = false;
      panelLine = "";
      stm32TelemetryLine = "";
      stm32TelemetryOverflow = false;
      panel.setNoDelay(true);
      // Persist/send connection history before soliciting the UART response.
      logEvent("INFO", "TCP_CONNECTED", panel.remoteIP().toString());
      panel.printf("ARGOS BRIDGE | TCP:CONNECTED | NET:%s %s | STM:WAITING\r\n",
                   activeMode(), activeIp().c_str());
      sendBridgeStatus("ESP STATUS");
      flushEventsToPanel();
      notifyStm32(true);
      Stm32Serial.print("E\r\n");
      lastProbeMs = millis();
      lastWaitingReportMs = millis();
      lastBridgeStatusMs = millis();
    }
  }

  if (panelActive && panel.connected()) {
    readPanelCommands();

    uint8_t buffer[256];
    while (Stm32Serial.available()) {
      size_t count = 0U;
      while (Stm32Serial.available() && count < sizeof(buffer)) {
        buffer[count++] = (uint8_t)Stm32Serial.read();
      }
      if (!stm32Seen && count > 0U) {
        stm32Seen = true;
        stm32OnlineLogPending = true;
        panel.println("ARGOS BRIDGE | STM:ONLINE");
      }
      if (count > 0U) {
        lastStm32ReceiveMs = millis();
        stm32RxBytes += count;
        // Send complete records so ESP diagnostics cannot split a MOTOR line.
        for (size_t index = 0U; index < count; ++index) {
          char value = (char)buffer[index];
          if (!stm32TelemetryOverflow) {
            stm32TelemetryLine += value;
          }
          if (value == '\n') {
            if (!stm32TelemetryOverflow) {
              panel.print(stm32TelemetryLine);
            }
            stm32TelemetryLine = "";
            stm32TelemetryOverflow = false;
          } else if (stm32TelemetryLine.length() >= 1024U) {
            stm32TelemetryLine = "";
            stm32TelemetryOverflow = true;
          }
        }
      }
    }

    // NVS writes can stall UART handling. Wait for the initial burst to finish.
    if (stm32OnlineLogPending && (millis() - lastStm32ReceiveMs) >= 100U) {
      stm32OnlineLogPending = false;
      logEvent("INFO", "STM_ONLINE", "UART_RX");
    }

    if (!stm32Seen && (millis() - lastProbeMs) >= 1000U) {
      notifyStm32(true);
      Stm32Serial.print("E\r\n");
      lastProbeMs = millis();
    }
    if (!stm32Seen && (millis() - lastWaitingReportMs) >= 2000U) {
      panel.printf("ARGOS BRIDGE | STM:WAITING | RX=D7/GPIO20 TX=D6/GPIO21 | RX_BYTES:%lu\r\n",
                   (unsigned long)stm32RxBytes);
      lastWaitingReportMs = millis();
    }
    if ((millis() - lastBridgeStatusMs) >= BRIDGE_STATUS_PERIOD_MS) {
      sendBridgeStatus("ESP STATUS");
      lastBridgeStatusMs = millis();
    }
  } else {
    while (Stm32Serial.available()) {
      (void)Stm32Serial.read();
    }
  }

  delay(1);
}
