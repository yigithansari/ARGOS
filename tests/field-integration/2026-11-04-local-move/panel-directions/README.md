# Experimental local-move panel — 2026-11-04

Installed STM32 OTA: 44736 bytes, CRC `0C70506B`.

The preceding forward100 mm screening looked straight to the operator. Encoder
travel was L96.235/R97.017 mm; external distance was not precisely measured.
This is not repeated position acceptance or completed PI tuning.

Panel Drive > DISTANCE / DIRECTION now accepts forward/backward30..150 mm and
in-place left/right15..90 degrees. Geometry comes from the calibration JSON.
New firmware syntax `G F/B/L/R left_counts right_counts` preserves legacy forward
syntax and existing motor protections. Signed progress/speed follow requested
direction;15 RPM,5 s and one-revolution limits remain. PI gains are unchanged.

Verification executed:

- Normal, OTA, bootloader and package builds passed.
-9 new panel tests passed: planner limits/directions, confirmed-stop start gate,
  command exclusion, feedback timeout, terminal/STOP lifecycle, widget construction
  without a connection, and write failure during disconnect.
-7 existing panel OTA tests and4 probe tests passed.
- Wireless upload CRC and zero-output STOP passed (`ota_verify_20261104_210437`).
-12 live invalid-command rejections passed with zero encoder movement and final
  STOP (`mcu_move_20261104_210523`), including invalid new directional syntax.

After operator confirmed30 cm clearance in every direction and power-switch access,
six bounded jobs were executed sequentially. Every job had final zero-output STOP.

| Job | Encoder result | Terminal |
| --- | --- | --- |
| Forward100 mm | mean95.693 mm; wheel difference0.257 mm | DONE |
| Backward100 mm | mean-95.489 mm; wheel difference0.473 mm | DONE |
| Left30 degrees | model+28.607 degrees | DONE |
| Right30 degrees | model-28.641 degrees | DONE |
| Forward150 mm, no keepalive | mean84.271 mm | ABORT WATCHDOG |
| Backward100 mm, x at~0.35 s | mean-18.223 mm | CANCELLED |

The95% encoder cutoff accounts for conservative undershoot. Do not adjust nominal
wheel geometry from these encoder-derived distances. External distance/turn
measurements and repeated accuracy acceptance remain pending; model angles do
not measure real yaw or wheel slip. See summary.json and individual logs.

Robot remains stopped on floor. Await operator direction observations and secured
wheels-clear readiness before a long PI speed suite. Persistent left instantaneous
speed variation remains provisional, not repaired. No further motion is scheduled.

These are explicit experimental snapshots; the publication source tree is not
an automatic mirror. Numeric/unit/build checks do not establish physical distance
or turn accuracy. The live jobs above specifically screen local motor behavior,
watchdog and cancellation; they are not a comprehensive safety qualification.
