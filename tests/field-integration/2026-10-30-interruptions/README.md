# Operator-assisted OTA interruptions - 2026-10-30

## Wi-Fi loss before erase: PASS

- Initial STOP/zero PWM confirmed.
- Original image uploaded slowly; operator disabled hotspot for ten seconds and
  restored it without cutting robot power.
- Upload connection reset after 6656 bytes queued; no final multipart boundary
  or complete application was sent. No new STM bootloader request in event history.
- ESP Wi-Fi disconnect/reconnect recorded; uptime continued. DHCP changed address
  from 192.168.137.178 to 192.168.137.226, found by UDP discovery.
- Application remained accessible and returned clean STOP/zero PWM status.

## Power loss during programming and wireless retry: PASS

- Operator confirmed wheels clear, USB absent, power switch ready.
- Same application padded with FF to 491520 bytes (CRC ACA3D122) to provide a
  practical interruption window. Application code itself unchanged.
- BL ERASED observed; operator instructed to cut power, wait ten seconds, restore.
- Last acknowledged offset 303104 of 491520; no BL OK or successful HTTP completion.
  ESP event history reached WRITE 60%, then a new POWERON boot.
- Operator reported no motor movement during power cycling.
- After restart application telemetry was absent; bootloader accepted the retry.
- Original ARGOS_OTA.bin recovered through Wi-Fi only: 41424 bytes, CRC 12954AFE,
  HTTP UPDATE PASS and BL OK, application startup, clean E response with SAFE:STOP
  and zero PWM/targets. No ST-LINK or USB recovery used.
- Immediate post-recovery query was echoed as WE; retry succeeded. This remains
  an open startup UART/command-integrity issue, not a passed behavior.

Logs from previous sessions are retained in ESP history; distinguish by uptime,
boot counter and action boundaries. The recovery harness's completion_seen flag
can be false when HTTP returns between polling iterations; its separate recovery
PASS checks BL OK in full captured telemetry, HTTP result, and STOP response.

Electrical motor pin levels were not measured. Full field/sensor/drive acceptance
and panel GUI interaction are separate from these interruption outcomes.

## Post-recovery stationary sensor check

Clean m/t/E replies: IMU data available through compatible reads, WHO_AM_I 0x72
(sensor identity not established as MPU6050); ToF left/right STATUS:OK and ranges
20/71 mm; STOP and zero PWM/targets. This is a communication/readout check, not
distance accuracy, IMU calibration, or drive validation.
