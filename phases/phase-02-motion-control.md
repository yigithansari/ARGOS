# Phase 02 - Motion Control

**Document ID:** PHASE-002
**Status:** Active - PI Tuning
**Start Date:** 2026-10-29
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

## 1. Purpose

Phase 02 develops measured wheel-speed control, repeatable straight-line motion, differential-drive odometry, and distance/angle commands on the assembled ARGOS mobile platform.

## 2. Current Status

The robot is assembled and can move and stop safely under computer control through the Windows hotspot and ESP32-C3 bridge. The STM32 command timeout and link-loss stop behavior remain the final motion safety layer.

Current controller work:

- Wheel geometry and encoder scale calibration are complete.
- Direction-specific low-band PWM/RPM characterization is complete.
- Independent feed-forward plus PI wheel-speed loops are implemented.
- Raised-wheel forward runs at 25, 35, and 45 RPM completed with mean tracking inside the initial +/-5% target.
- STOP was confirmed after each recorded run.
- PI gain tuning and repeatability acceptance remain in progress.

## 3. Calibrated Parameters

- Wheel circumference: `260.0 mm`
- Track width: `198.0 mm`
- Left encoder scale: `48097.2 counts/wheel revolution`, forward sign `-1`
- Right encoder scale: `48036.6 counts/wheel revolution`, forward sign `+1`

The active calibration file is `software/control_panel/argos_motion_calibration.json`.

## 4. Open Acceptance Work

- Complete three repeated runs at each accepted target speed.
- Improve low-speed variation at 25 and 35 RPM.
- Validate reverse PI control and in-place turning.
- Measure disconnect-to-physical-stop latency.
- Complete controlled floor tuning and straight-line repeatability tests.
- Implement and validate odometry.
- Add distance and angle commands with IMU comparison.
- Add battery-voltage monitoring; charge level is not currently available.

## 5. Safety Rules

- Initial gain changes shall be tested with the wheels raised.
- Floor testing requires a clear lane and immediate access to power isolation.
- Link loss shall continue to stop motor output through the STM32 timeout.
- No PI tuning result is accepted from a single run.

## 6. Evidence

- `docs/08_phase_02_motion_control.md`
- `tests/field-integration/2026-10-30-lowband-rpm-characterization.md`
- `tests/field-integration/2026-10-30-pi-speed-control-implementation.md`
- `tests/field-integration/2026-10-31-motion-progress/`

## 7. Next Milestone

Complete repeated forward/reverse raised-wheel tests, then perform controlled floor tuning for straight-line motion.

