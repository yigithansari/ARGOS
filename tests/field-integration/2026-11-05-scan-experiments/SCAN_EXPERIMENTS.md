# Supervised scan experiments

This version records and proposes steps; it never starts an autonomous drive.
Existing STM G moves pause ToF streaming. Continuous autonomous exploration needs
a separately validated STM obstacle/freshness stop during movement first.

1. Open the revised panel, connect, and confirm current ToF telemetry is arriving.
2. Open **SCAN / EXPERIMENT**. Enter the setup/room/wheel arrangement and start
   **START LOG**. A JSONL file is created under `Tools/experiment_logs/`.
3. Use **MANUAL MOVE** for existing bounded distance/angle commands after checking
   the path and swept clearance. The user starts every movement. **SUGGEST STEP**
   shows a proposed30 mm forward or15 degree scan step, or HOLD; it sends nothing.
4. At each stop, allow fresh measurements. **MARK NOTE** records the entry text
   and current pose; use it for external distance/angle measurements, slip,
   contact, missed ranges or other observations. Stop if physical clearance is uncertain.
5. **STOP LOG** closes the recording. **EXPORT / REPLAY** also writes a summary
   JSON with projected endpoints, separate origin segments, motion outcomes and
   a robust candidate wall fit. Send the original `.jsonl` for analysis.

Logs include UTC/host-monotonic receipt times, successful panel motor/query TX,
raw incoming lines, parsed motor/ToF data, encoder pose, metadata and notes.
STM `T_MS` is retained in raw motor frames; ToF receipt time is not a synchronized
hardware measurement timestamp. Logger errors are visible and disable recording;
they do not issue motor commands. Restart a session after correcting a file error.

Replay without a robot:
`python argos_exploration.py --replay experiment_logs/<session>.jsonl`

Geometry uses nominal telemetry yaw/tilt/baseline: positive yaw points RIGHT,
while odometry heading is positive CCW. ToF midpoint is currently assumed at the
robot pose origin; physical forward/lateral offsets are not calibrated. The view
is an endpoint cloud with an encoder path, not a corrected occupancy/SLAM map.
Wall candidates do not automatically correct heading/pose or establish clearance.
Unknown areas are not certified free space. Front sensors cannot fully observe
the side or swept robot body. The optional left-side policy input is tested in
software but has no ultrasonic hardware/transport integration yet.

PI remains frozen at the accepted baseline. Do not reopen PI tuning from every
scan mismatch; first inspect the logged ranges, geometry and motion outcomes.
