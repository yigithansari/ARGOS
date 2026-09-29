#include <ESPmDNS.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include "wifi_secrets.h"

// ESP32-C3 Super Mini: D7/GPIO20 is native UART0 RX,
// D6/GPIO21 is native UART0 TX.
// ESP D7 RX <- Nucleo D1 / PA2 / USART2_TX
// ESP D6 TX -> Nucleo D0 / PA3 / USART2_RX
constexpr int STM32_RX_PIN = 20;
constexpr int STM32_TX_PIN = 21;
constexpr uint32_t STM32_BAUD = 115200;

constexpr char FALLBACK_AP_NAME[] = "ARGOS";
constexpr uint16_t TCP_PORT = 3333;
constexpr uint16_t DISCOVERY_PORT = 3334;
constexpr uint32_t STA_CONNECT_TIMEOUT_MS = 12000U;

// UART1 is routed to the labelled pins through the ESP32-C3 GPIO matrix. This
// keeps UART0 boot-ROM messages away from the STM32 command parser.
HardwareSerial Stm32Serial(1);
WiFiServer server(TCP_PORT);
WiFiUDP discovery;
WiFiClient panel;
bool panelActive = false;
bool stm32Seen = false;
uint32_t lastProbeMs = 0;
uint32_t lastWaitingReportMs = 0;
uint32_t stm32RxBytes = 0;

String activeIp() {
  return (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString()
                                         : WiFi.softAPIP().toString();
}

const char *activeMode() {
  return (WiFi.status() == WL_CONNECTED) ? "STA" : "AP";
}

void notifyStm32(bool connected) {
  Stm32Serial.printf("W %s %s\r\n", activeMode(), activeIp().c_str());
  Stm32Serial.print(connected ? "N 1\r\n" : "N 0\r\n");
}

void startFallbackAp() {
  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(FALLBACK_AP_NAME, ARGOS_FALLBACK_AP_PASSWORD, 6, false, 1);
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
    discovery.printf("ARGOS_BRIDGE|%u|%s|%s", TCP_PORT, activeMode(), activeIp().c_str());
    discovery.endPacket();
  }
}

void setup() {
  Stm32Serial.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.setSleep(true);
  WiFi.begin(ARGOS_WIFI_SSID, ARGOS_WIFI_PASSWORD);

  uint32_t startedAt = millis();
  while ((WiFi.status() != WL_CONNECTED) &&
         ((millis() - startedAt) < STA_CONNECT_TIMEOUT_MS)) {
    delay(100);
  }

  if (WiFi.status() != WL_CONNECTED) {
    startFallbackAp();
  }

  server.begin();
  server.setNoDelay(true);
  discovery.begin(DISCOVERY_PORT);
  MDNS.begin("argos");
  MDNS.addService("argos", "tcp", TCP_PORT);

  notifyStm32(false);
}

void loop() {
  handleDiscovery();

  if (panelActive && !panel.connected()) {
    panel.stop();
    panelActive = false;
    Stm32Serial.print("x\r\n");
    notifyStm32(false);
    stm32Seen = false;
  }

  if (!panelActive) {
    WiFiClient candidate = server.available();
    if (candidate) {
      panel = candidate;
      panelActive = true;
      stm32Seen = false;
      stm32RxBytes = 0;
      panel.setNoDelay(false);
      notifyStm32(true);
      Stm32Serial.print("E\r\n");
      lastProbeMs = millis();
      lastWaitingReportMs = millis();
      panel.printf("ARGOS BRIDGE | TCP:CONNECTED | NET:%s %s | STM:WAITING\r\n",
                   activeMode(), activeIp().c_str());
    }
  }

  if (panelActive && panel.connected()) {
    uint8_t buffer[256];
    size_t count = 0;

    while (panel.available()) {
      count = 0;
      while (panel.available() && count < sizeof(buffer)) {
        buffer[count++] = (uint8_t)panel.read();
      }
      if (count > 0) {
        Stm32Serial.write(buffer, count);
      }
    }

    while (Stm32Serial.available()) {
      count = 0;
      while (Stm32Serial.available() && count < sizeof(buffer)) {
        buffer[count++] = (uint8_t)Stm32Serial.read();
      }
      if (!stm32Seen) {
        stm32Seen = true;
        panel.println("ARGOS BRIDGE | STM:ONLINE");
      }
      if (count > 0) {
        stm32RxBytes += count;
        panel.write(buffer, count);
      }
    }

    if (!stm32Seen && millis() - lastProbeMs >= 1000U) {
      notifyStm32(true);
      Stm32Serial.print("E\r\n");
      lastProbeMs = millis();
    }
    if (!stm32Seen && millis() - lastWaitingReportMs >= 2000U) {
      panel.printf("ARGOS BRIDGE | STM:WAITING | RX=D7/GPIO20 TX=D6/GPIO21 | RX_BYTES:%lu\r\n",
                   (unsigned long)stm32RxBytes);
      lastWaitingReportMs = millis();
    }
  } else {
    // Drain startup telemetry while no panel is attached so stale data does not
    // burst into the next session.
    while (Stm32Serial.available()) {
      (void)Stm32Serial.read();
    }
  }

  delay(1);
}
