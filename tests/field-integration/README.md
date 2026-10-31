# Field Integration Evidence

## Current Result

- Phase 01 mobile platform integration: **COMPLETE**
- Phase 02 motion control: **ACTIVE - PI tuning**
- Basic hotspot-controlled motion and safe stop: **OPERATIONAL**
- Raised-wheel forward runs at 25/35/45 RPM: mean tracking target met in single runs
- Open: repeated runs, reverse, disconnect-stop latency, floor tuning and odometry

## Field Link Baseline

- ESP32-C3 Wi-Fi/TCP/STM link regression: **PASS**
- Validation date: `2026-10-08`
- Evidence report: `2026-10-08-field-link-regression-report.md`

The accepted run demonstrated more than 20 minutes of continuous operation,
intentional hotspot loss, fallback access-point operation, and automatic
Wi-Fi/TCP/STM recovery without an ESP reset.

This evidence validates the field communication path only. Phase 00 motor,
encoder, IMU, and ToF certification remains archived under `tests/phase-00/`.

## Drive Balance Follow-up

- Latest precheck: `2026-10-29-drive-balance-precheck.md`
- Live paired result: `2026-10-29-live-drive-balance-report.md`
- Result: encoder comparison passed; open-loop balance requires correction.

## Sensor Health Follow-up

- Live sensor report: `2026-10-29-live-sensor-health-report.md`
- Result: both ToF sensors PASS; MPU communication PASS with gyro calibration pending.

## Closed-loop Drive Update

- Implementation record: `2026-10-29-closed-loop-drive-update.md`
- Result: firmware flash PASS; qualitative floor test reported straight driving.

## Phase 02 Motion Preparation

- Geometry and software record: `2026-10-29-phase-02-geometry-prep.md`
- Result: geometry and manual 10-turn encoder scale measurement PASS. Next test
  was calibrated PWM-to-wheel-RPM characterization.
- Characterization: `2026-10-30-pwm-rpm-characterization.md`
- Result: motor and encoder paths PASS; a nonlinear 50-60% transition requires
  a focused low-band test before PI gain selection.
- Focused result: `2026-10-30-lowband-rpm-characterization.md`
- Result: PASS; side/direction-specific feed-forward anchors are available and
  PI controller implementation may begin.

## Phase 02 PI Progress

- Implementation record: `2026-10-30-pi-speed-control-implementation.md`
- Latest evidence: `2026-10-31-motion-progress/`
- Result: feed-forward plus independent PI loops are operational. Single forward
  raised-wheel 25/35/45 RPM runs completed with STOP confirmation. Acceptance is
  still pending repeated, reverse, disconnect-stop latency and floor tests.
