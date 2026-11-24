# Supervised scan / experiment software — 2026-11-05

Implemented operator-run experiments without remote robot driving or new PI work.
Firmware, pin mappings, calibration and installed CRC0C70506B are unchanged.

Panel SCAN / EXPERIMENT provides session setup, START/STOP LOG, MARK NOTE,
EXPORT / REPLAY, a bounded endpoint cloud/encoder path and wall candidate display.
SUGGEST STEP evaluates fresh ranges and confirmed STOP before proposing a short
forward/scan step; it does not transmit. MANUAL MOVE opens existing bounded drive
controls and keeps their operator confirmation and STOP behavior.

JSONL logs contain calibration/setup, UTC/host-monotonic time, successful motor
and query commands (including keepalives/disconnect stop), raw RX, parsed motor/ToF
state, pose, origin changes, notes and proposed actions. Replay exports JSON with
separate coordinate-origin segments, motion terminal messages and wall candidates;
a truncated/malformed record does not discard preceding valid records. Logging
failures visibly stop recording without transmitting or disconnecting the robot.

Verified offline:7 exploration tests,9 odometry/parser/replay tests,9 existing
local-move safety/UI tests and7 OTA tests passed; Python syntax checks passed.
GUI tests construct the experiment window and exercise geometry/recording/advice
with fake transport and scheduled discovery suppressed. No hardware test, firmware
build or flash was performed. No claim of autonomous or field mapping acceptance.

Remaining limits:

- Current STM G jobs pause ToF streaming. In-motion STM-owned obstacle/range-age
  stop is required before continuous autonomous operation; this version is advisory.
- Forward sensors do not observe the full side/swept footprint. Optional left-side
  range wall-follow policy is tested software only; ultrasonic driver/wiring absent.
- Nominal telemetry yaw/tilt/baseline are projected with ToF midpoint at robot pose
  origin; physical sensor offsets are not calibrated. Host ToF receipt timestamps
  are not synchronized sensor measurement times.
- Endpoint cloud and robust line candidate are not occupancy/SLAM or automatic
  heading correction. Wall association and actual distance/slip remain unverified.
- PI freeze remains in force. Logged scan errors do not automatically reopen tuning.

Operator workflow: Tools/SCAN_EXPERIMENTS.md in the active source tree.
