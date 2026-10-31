# ARGOS Field Link Regression

Use this run after flashing the field-integration ESP and STM32 firmware.

## Latest Accepted Run

**2026-10-08: PASS.** The ESP remained online for more than 20 minutes, then
survived an intentional hotspot outage and restored Wi-Fi, TCP, and STM UART
automatically without resetting. The archived evidence is available at
`tests/field-integration/2026-10-08-field-link-regression-report.md`.

The final five-minute post-recovery dwell was not captured. Repeat that dwell
opportunistically during the next subsystem run; it does not reopen the accepted
endurance and recovery result.

## Simdi Yapacagin Kisa Kontrol

Bu asamada `SYSTEM TEST` calistirmak zorunda degilsin. Once baglantiyi dogrula:

1. Guncel ESP bridge sketch'ini ESP32-C3'e yukle.
2. Windows hotspot ag bandini `2.4 GHz` olarak sabitle ve hotspot'u ESP'ye
   guc vermeden once ac. Varsa "bagli cihaz yokken kapat" tasarrufunu kapat.
3. Bilgisayarin da ayni hotspot aginda oldugunu kontrol et.
4. Robot hareket etmesin diye tekerleri yerden kaldir veya sasenin altina takoz koy.
5. Robota guc ver, sonra kontrol panelini ac.
6. Ust durumun `ARGOS ONLINE - WIFI + STM32` olmasini bekle.
7. Bir dakika bosta bekle, sonra yon tuslariyla 20-30 saniye kisa hareket dene.
8. Robot dururken hotspot'u 15 saniye kapatip tekrar ac.
9. Panelin kendiliginden yeniden baglanmasini bekle ve `LINK LOGBOOK` tusuna bas.

Bu kisa kontrol temiz gecerse asagidaki uzun kabul kosusuna gec.

## Preparation

- Keep the robot wheels clear of the floor for the first run. This means lifting
  the chassis or placing it on blocks so the wheels can rotate without moving it.
- Start the configured computer hotspot.
- Power the complete robot from its normal supply.
- Start the control panel with host set to `AUTO`.
- Open `LINK LOGBOOK` and confirm a new `BOOT` and `RADIO_CONFIG` event arrive.
- `RADIO_CONFIG` should report `applied=1` and `sleep=1`.

## Run

1. Leave the robot connected and idle for 10 minutes.
2. Drive for 3 minutes in short bursts, including forward turns and reverse turns.
3. Release every key and confirm both requested motor values return to zero.
4. Keep the stopped robot connected until the session reaches 20 minutes.
5. Disable the hotspot for 15 seconds.
6. Confirm the panel reports link loss and the STM32 stops the motors.
7. Re-enable the hotspot and wait for automatic reconnection.
8. Confirm the queued ESP events appear in `LINK LOGBOOK` after reconnect.
9. Keep the recovered link open for another 5 minutes.

## Interpretation

- `BOOT | POWERON` after an intentional power cycle is normal.
- `BOOT | BROWNOUT` points to the ESP supply, wiring, grounding, or decoupling.
- `BOOT | PANIC`, `INT_WDT`, `TASK_WDT`, or `WDT` points to firmware execution.
- `WIFI_DISCONNECT` without a new `BOOT` points to RF or hotspot behavior.
- `HEARTBEAT_TIMEOUT` without an ESP reset means TCP stayed unusable long enough
  for the panel watchdog to close and rebuild the link.
- `TCP_DISCONNECTED` followed by `TCP_CONNECTED` confirms recovery completed.
- Repeated `NO_AP_FOUND` means the hotspot was not visible to the ESP.
- Repeated `AUTH_FAIL` or handshake errors mean credentials/security negotiation.

## Power Follow-up

If a brownout is recorded, inspect the ESP rail before changing the software:

- Keep ESP supply and ground wiring short.
- Keep the antenna end clear of metal and high-current motor wiring.
- Use a stable 5 V source for the module board and a common ground with STM32.
- Add local decoupling near the ESP board if it is not already present.
- Keep motor-current return paths away from the ESP/STM signal-ground route.

Preserve `argos_connection_logbook.jsonl` after the run. It is cumulative; do not
rename it between retries unless a clean test epoch is intentionally required.
