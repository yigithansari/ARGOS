# Stationary OTA acceptance - 2026-10-30

Result: all automated tests PASS. Physical interruption and full field acceptance pending.

Target: NUCLEO-F446RE through ESP32-C3, Wi-Fi 192.168.137.178.
Image: ARGOS_OTA.bin, 41424 bytes, CRC32 12954AFE, same firmware reinstalled.
No movement commands issued. USB removed per prior operator report.

## Passed

- Initial STOP, zero motor PWM and speed targets.
- One payload byte modified while retaining expected original CRC: ESP returned
  HTTP 400 CACHE FAIL: upload size/CRC mismatch. No bootloader request observed;
  original application still answered clean STOP status queries.
- Multipart upload socket closed halfway: application remained accessible,
  no bootloader request observed, HTTP update status reported STM:ONLINE.
- Ten consecutive complete wireless updates without ST-LINK intervention.
  Every attempt returned UPDATE PASS, bootloader BL OK with expected CRC,
  application startup banner, and clean STOP/zero PWM and target status afterward.
- The actual active panel _wireless_update_worker was extracted with Python AST
  and run with a headless completion adapter. GUI interaction was not tested.

## Limits and remaining work

- Closing an upload TCP socket does not test physical Wi-Fi loss or power loss.
- Operator-assisted power interruption during STM programming and recovery remain.
- Actual Wi-Fi outage before erase, sensor/drive regression, and GUI interaction remain.
- Telemetry confirms commanded state; no electrical motor-pin measurements were taken.
- Earlier startup command corruption remains a follow-up; these ten runs passed
  using continuous TCP draining, an eight-second startup allowance, and bounded
  status retries. This does not establish that the earlier issue was fixed.

## Evidence

- results.json: per-test outcomes (13 checks including initial status).
- telemetry.log: captured bridge, bootloader, and application responses.
- ota_acceptance.py: exact harness used. Contains live flashing actions; rerun
  only intentionally with a stationary robot. Requires the active tree files.
