# ARGOS Field Link Regression Report

**Date:** 2026-10-08
**Result:** PASS
**Scope:** ESP32-C3 Wi-Fi, TCP bridge, STM32 UART visibility, persistent diagnostics, and automatic recovery

## Test Configuration

- Network: Windows mobile hotspot, `192.168.137.0/24`
- ESP radio: `8.5 dBm`, modem sleep enabled
- Bridge endpoint: TCP port `3333`
- ESP boot session: `boot_id=6`
- Control-panel logbook: `ARGOS/ARGOS/Tools/argos_connection_logbook.jsonl`

## Acceptance Results

| Check | Result | Evidence |
| --- | --- | --- |
| Radio configuration applied | PASS | `RADIO_CONFIG tx=8.5dBm applied=1 sleep=1` |
| Initial hotspot association | PASS | Connected in `2.359 s`, RSSI `-40 dBm` |
| Initial TCP bridge connection | PASS | Connected at ESP uptime `7.645 s` |
| STM32 UART detected | PASS | `STM_ONLINE` at ESP uptime `7.656 s` |
| Sustained link | PASS | Link remained online for about `21 min 49 s` before the intentional outage |
| ESP reset safety | PASS | No new boot ID, brownout, panic, watchdog, CPU lockup, or power-glitch event occurred |
| Link-loss detection | PASS | Panel recorded `LINK_LOST`; ESP recorded Wi-Fi and TCP disconnect events |
| Fallback access point | PASS | `AP_FALLBACK 192.168.4.1` was started during the outage |
| Automatic hotspot recovery | PASS | STA recovered without an ESP restart |
| Automatic TCP recovery | PASS | TCP restored `20.795 s` after the first disconnect event |
| Automatic STM32 recovery | PASS | STM UART restored `20.819 s` after the first disconnect event |
| Persistent diagnostic delivery | PASS | Queued ESP events `141-153` were delivered after reconnection |
| Disconnect-log throttling | PASS | Repeated causes were summarized with `suppressed=4` entries |

## Timeline

- `01:51:44`: panel connected to `192.168.137.30:3333`.
- ESP uptime `0.012 s`: `BOOT POWERON`.
- ESP uptime `2.359 s`: hotspot connected at `-40 dBm`.
- ESP uptime `7.645 s`: TCP client connected.
- ESP uptime `7.656 s`: STM32 UART traffic detected.
- `02:13:33`: panel detected the intentional link interruption.
- ESP uptime `1316.875 s`: Wi-Fi disconnect sequence began.
- ESP uptime `1331.885 s`: fallback AP started.
- ESP uptime `1336.467 s`: Windows hotspot connection recovered.
- ESP uptime `1336.479 s`: fallback AP stopped after recovery.
- ESP uptime `1337.670 s`: TCP client reconnected.
- ESP uptime `1337.694 s`: STM32 UART traffic returned.
- `02:13:54`: control panel reconnected automatically.

## Observation

The prescribed five-minute post-recovery dwell was not present in the captured
log because the run ended immediately after recovery. This does not invalidate
the link endurance and automatic-recovery result: the same ESP boot session
survived more than 20 minutes, the deliberate outage, fallback operation, and a
complete Wi-Fi/TCP/STM recovery. The extra dwell should be repeated
opportunistically during the next subsystem run rather than reopening this test.

## Verdict

The field communication path is accepted for the next integration stage. The
result applies to the ESP32-C3 Wi-Fi/TCP/STM transport and diagnostics. Phase 00
motor and sensor certification remains covered by the existing Phase 00 report;
this run did not repeat that full system test.
