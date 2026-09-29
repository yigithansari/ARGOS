ARGOS ESP32-C3 Wi-Fi Bridge
===========================

Arduino IDE:
1. Board package: esp32 by Espressif Systems
2. Board: ESP32C3 Dev Module
3. Open esp32_c3_argos_bridge/esp32_c3_argos_bridge.ino
4. Upload to ESP32-C3

Wi-Fi configuration:
1. Copy wifi_secrets.example.h to wifi_secrets.h.
2. Set the infrastructure-network SSID and password.
3. Choose a fallback ARGOS access-point password.
4. Keep wifi_secrets.h local; it is excluded by .gitignore.

Fallback access point:
- SSID: ARGOS
- ESP address: 192.168.4.1
- TCP port: 3333

Connections:
- ESP 5V/VBUS -> Nucleo 5V
- ESP GND     -> Nucleo GND
- ESP D7 / GPIO20 / RX <- Nucleo D1 / PA2 / USART2_TX
- ESP D6 / GPIO21 / TX -> Nucleo D0 / PA3 / USART2_RX

NUCLEO-F446RE UART routing for D0/D1:
- SB13 and SB14: OFF (disconnect ST-LINK virtual COM from PA2/PA3)
- SB62 and SB63: ON  (connect PA2/PA3 to Arduino D1/D0)
- ST-LINK USB is still usable for firmware upload/debug, but its virtual COM
  channel is intentionally disconnected.

OLED is controlled by STM32:
- VCC -> 3V3
- GND -> GND
- SCL -> Nucleo D12 / PA6
- SDA -> Nucleo D11 / PA7
- Expected controller/address: SSD1306, 128x64, I2C 0x3C

Operation:
1. Connect the computer to ARGOS Wi-Fi.
2. Start argos_control_panel.py.
3. The panel automatically connects to 192.168.4.1:3333.
4. If TCP disconnects, ESP sends STOP to STM32 before accepting a new client.
5. The panel shows STM UART WAITING until STM32 replies. Motor controls remain
   locked until the complete PC -> Wi-Fi -> ESP -> UART -> STM32 path responds.
