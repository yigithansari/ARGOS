# Phase 02 - Motion Control

**Document ID:** PHASE-002
**Status:** Complete with carry-over
**Closed:** 2026-11-25
**Start Date:** 2026-10-29
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

## 1. Purpose

Phase 02 develops measured wheel-speed control, repeatable straight-line motion, differential-drive odometry, and distance/angle commands on the assembled ARGOS mobile platform.

## 2. Current Status

The 2026-11-24 acceptance run passed all nine floor trials and the mean wheel-speed criterion in all valid raised-wheel trials. Instantaneous speed tracking and precise verification of the 1200 ms watchdog cutoff remain open. See the [acceptance results](../tests/field-integration/2026-11-24-phase-02-acceptance/README.md). The early development results below predate this run.

The robot is assembled and can move and stop safely under computer control through the Windows hotspot and ESP32-C3 bridge. The STM32 command timeout and link-loss stop behavior remain the final motion safety layer.

Current controller work:

- Wheel geometry and encoder scale calibration are complete.
- Direction-specific low-band PWM/RPM characterization is complete.
- Independent feed-forward plus PI wheel-speed loops are implemented.
- Raised-wheel forward runs at 25, 35, and 45 RPM completed with mean tracking inside the initial +/-5% target.
- STOP was confirmed after each recorded run.
- PI gain tuning and repeatability acceptance remain in progress.

## 3. Calibrated Parameters

- Wheel diameter: `87.0 mm` measured unloaded. Effective rolling circumference remains to be verified.
- Track width: `255.0 mm` between wheel centre planes
- Left encoder scale: `48097.2 counts/wheel revolution`, forward sign `-1`
- Right encoder scale: `48036.6 counts/wheel revolution`, forward sign `+1`

The active calibration file is `software/control_panel/argos_motion_calibration.json`.

## 4. Exit Decision

- Floor distance and turns passed all nine recorded trials. During straight driving the operator observed a small lateral offset despite preserved heading. A mechanical cause is suspected and remains unconfirmed.
- Link-loss stop is accepted as PASS based on the operator observing motor rotation cease immediately after disconnect. The archived telemetry brackets the 1200 ms threshold. Precise cutoff timing remains a revalidation item.
- Instantaneous wheel-speed tracking within +/-5% remains unmet. Revalidate it along with lateral drift and watchdog timing after the new chassis, motors and PCB are installed.

### Deferred Work

| Item | Current state | Resume when / next action |
| --- | --- | --- |
| Wheel-speed oscillation | The left-side variation also appeared at fixed PWM. Motor or wheel mechanics are suspected. Root cause is unconfirmed. | After fitting the new motors, check wheel clearance, remeasure encoder scale and repeat fixed-PWM and PI runs. Recheck floor distance and turns after calibration. |
| Battery percentage | No charge percentage is available. | Add pack-voltage sensing, calibrate it against a multimeter and validate the charge estimate for the installed pack before displaying it in the panel or OLED. |
| Exploration and mapping | Dual ToF and side ultrasonic sensing produced prototype maps. Corner handling, repeatable coverage and map accuracy remain open. | Revisit after motion is repeatable. Keep LiDAR as a future evaluation. |
| Shared ToF I2C dropout | Both ToF devices disappeared in one observed event and recovered after a power cycle. | Capture bus and power evidence on recurrence and verify recovery behavior. |
| Raspberry Pi remote driving | UART and live telemetry were demonstrated. Keyboard driving through Pi Connect showed intermittent stutter. | Compare local and remote input timing, command refresh and STOP behavior. |
| ToF test-report false PASS | A substring check can read `YOK` as `OK`. | Match explicit status tokens and cover missing-sensor responses in regression tests. |

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

Continue under [Phase 03](phase-03-localization-and-basic-autonomy.md).

