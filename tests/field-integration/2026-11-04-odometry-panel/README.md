# Odometry panel milestone — 2026-11-04

Operator stopped further PI tuning/mechanical-isolation loops to advance Phase02.
Full hub exchange cancelled. Installed firmware remains44736 bytes, CRC0C70506B;
no gain, hardware mapping, geometry, timeout or flash change. Mean tracking is
provisionally accepted; left instantaneous ripple remains a documented limitation.
Last hardware report: original hubs, exchanged outer rings, unchanged electrical
mapping, raised/secured, last motor test ended with confirmed STOP.

Implemented in the desktop panel:

- Distance / Direction window shows relative X/Y, CCW heading and centre-path
  distance, with STOP-gated RESET POSITION that sends no hardware command.
- Encoder-only exact differential-drive arc integration using central calibrated
  geometry; signed32-bit encoder rollover and STM tick rollover handled.
- New origin after disconnect, MCU reboot/backward tick, counter reset or geometry
  change, avoiding false displacement across those boundaries. Position is a
  local estimate, not a global localization or slip-compensated measurement.
- Current T_MS-tagged STM RPM is already calibrated and now displayed directly.
  Removed erroneous second scaling/sign conversion for current frames. Legacy
  conversion retained only for older untagged frames.

Executed offline:9 odometry/parser/replay tests,9 local-move safety/UI tests,
7 OTA tests and Python syntax checks passed. Widget construction is included;
network/discovery timers suppressed during GUI tests. Replay uses existing
100 mm floor-job telemetry from mcu_move_20261104_210838, not a new hardware run.
Normal/reverse, in-place turn, curved arc, initial nonzero counters, wrap, reboot,
geometry change and STOP-gated local reset are covered.

No robot commands, new motor test, firmware build or flash were performed for
this software milestone. Live field accuracy and external distance/yaw checks
remain pending. Restart the panel to load the changes. Do not restart PI or
mechanical isolation merely because these recorded limitations still exist.
