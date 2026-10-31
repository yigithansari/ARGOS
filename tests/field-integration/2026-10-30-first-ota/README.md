# First wireless STM32 update - 2026-10-30

Result: normal OTA transfer PASS; full field acceptance pending.

- USB cables removed per operator; ESP reached through hotspot at 192.168.137.178.
- Existing ARGOS_OTA.bin reinstalled: 41424 bytes, CRC32 12954AFE.
- STOP and zero motor PWM/targets confirmed before programming.
- Raw application/octet-stream upload failed with HTTP 400 CACHE FAIL before
  STM erase. Desktop worker changed to multipart/form-data; syntax checks passed.
- Equivalent multipart upload through diagnostic script returned HTTP 200
  UPDATE PASS, BL OK 311773950; application restarted, both ToF addresses assigned.
- Immediate E query was received as WE; separate follow-up returned clean x/E
  responses with SAFE:STOP, zero PWM/targets, STM:ONLINE.
- No drive commands or physical motor safety measurements performed.
- Pending: panel GUI path, startup command integrity, sensor/drive regression,
  corrupt-image rejection, interrupted Wi-Fi/power recovery, ten consecutive updates.

Logs include both the initial failed raw upload and successful multipart upload.
