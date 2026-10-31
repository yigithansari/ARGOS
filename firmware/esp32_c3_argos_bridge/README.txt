ARGOS ESP32-C3 Wi-Fi Bridge
===========================

Arduino IDE:
1. Board package: esp32 by Espressif Systems
2. Board: ESP32C3 Dev Module
3. Open esp32_c3_argos_bridge/esp32_c3_argos_bridge.ino
4. Upload to ESP32-C3
5. Create wifi_secrets.h from wifi_secrets.example.h and set local credentials.
   The private header is intentionally excluded from the publication snapshot.

Wi-Fi:
- Primary: configured Windows hotspot (STA); address is assigned by DHCP.
- The following ARGOS network/address is the fallback AP, not the fixed STA address.
- SSID: ARGOS
- Password: configured locally in wifi_secrets.h
- ESP address: 192.168.4.1
- TCP port: 3333

Connections:
- ESP 5V/VBUS -> Nucleo 5V
- ESP GND     -> Nucleo GND
- ESP D7 / GPIO20 / RX <- Nucleo D1 / PA2 / USART2_TX
- ESP D6 / GPIO21 / TX -> Nucleo D0 / PA3 / USART2_RX

NETWORK AND RECOVERY
--------------------
- Primary mode: joins the hotspot configured in wifi_secrets.h.
- Indoor TX power: 8.5 dBm with modem sleep enabled.
- Recovery: retries STA and starts the ARGOS fallback AP after 15 seconds offline.
- Discovery: UDP 3334, TCP control/telemetry: 3333, STM update HTTP: 3335.
- Persistent diagnostics: reset, Wi-Fi, TCP and STM link events are queued in NVS.
- The desktop acknowledges queued events after writing them to
  argos_connection_logbook.jsonl.
- Bridge heartbeat commands are handled by ESP and are not forwarded to STM32.

NUCLEO-F446RE UART routing for D0/D1:
- SB13 and SB14: OFF (disconnect ST-LINK virtual COM from PA2/PA3)
- SB62 and SB63: ON  (connect PA2/PA3 to Arduino D1/D0)
- ST-LINK USB is still usable for firmware upload/debug, but its virtual COM
  channel is intentionally disconnected.

STM32 WIRELESS UPDATE
---------------------
- The ESP receives the complete relocated STM32 image on HTTP port 3335 and
  stores it in LittleFS before requesting bootloader mode.
- Required upload headers include image size, CRC32, build ID, and the stopped
  robot confirmation.
- ESP sends 256-byte verified chunks over USART2 and reports progress to the
  active desktop panel connection.
- Install Debug/ARGOS_BOOTSTRAP.bin once through ST-LINK before using this path.
- Routine updates use Debug/ARGOS_OTA.bin from the desktop panel. Do not flash
  ARGOS_OTA.bin by itself at 0x08000000.

OLED is controlled by STM32:
- VCC -> 3V3
- GND -> GND
- SCL -> Nucleo D12 / PA6
- SDA -> Nucleo D11 / PA7
- Expected controller/address: SSD1306, 128x64, I2C 0x3C

Operation:
1. Connect the computer to ARGOS Wi-Fi.
   For normal hotspot use, enable the configured hotspot instead; use UDP discovery.
2. Start argos_control_panel.py.
3. The panel automatically connects to 192.168.4.1:3333.
4. If TCP disconnects, ESP sends STOP to STM32 before accepting a new client.
5. The panel shows STM UART WAITING until STM32 replies. Motor controls remain
   locked until the complete PC -> Wi-Fi -> ESP -> UART -> STM32 path responds.

2026-10-31 source snapshot:
- UART RX buffer 4096 bytes; connection logging precedes initial UART requests.
- STM-online NVS logging waits for a 100 ms UART idle interval.
- Complete newline records are forwarded, preventing ESP diagnostics from
  splitting motor telemetry; overlong records are discarded through newline.
- TCP_NODELAY enabled. These changes received targeted telemetry tests;
  long-duration motion/link acceptance remains open.
