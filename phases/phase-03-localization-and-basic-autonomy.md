# Phase 03 - Localization and Basic Autonomy

**Status:** Active as of 2026-11-25

## Current Baseline

The mobile platform supports encoder odometry, distance and turn commands, and operator-controlled motion. Phase 02 closed with hardware-dependent revalidation carried forward. See the [Phase 02 closeout](phase-02-motion-control.md).

Two front ToF sensors and a left HC-SR04 feed guarded exploration and prototype 2D point maps. The panel exports JSONL traces and SVG maps. The three latest exploration traces contain two `CURVE_TURN_CLEARANCE` stops and one `CURVE_SIDE_CLEARANCE` stop. Repeatable corner handling, room coverage and measured map accuracy remain open.

Pi 5 direct STM32 UART at 115200 baud, Pi Connect panel launch and live telemetry were demonstrated by the operator. Remote keyboard stutter, shared ToF I2C loss and Pi mounting remain open. See the [Pi setup](../software/RaspberryPi/README.md) and [scan experiments](../software/control_panel/SCAN_EXPERIMENTS.md).

## Work Queue

- Correct ToF status parsing so `YOK` cannot be reported as PASS. Capture shared I2C dropout and recovery evidence.
- Validate sustained Pi telemetry and compare local versus remote command timing.
- Repeat guarded corner and wall-following trials. Compare maps against a measured reference layout before accepting room coverage.
- Add calibrated battery-voltage measurement and validate the displayed battery percentage.
- After the new chassis, motors and PCB are installed, repeat wheel calibration, instantaneous speed tracking, lateral drift measurements and watchdog timing. Recheck distance, turns and STOP on the revised platform.

## Evidence

The working copy under `argos ekstralar/ARGOS/ARGOS SOFTWARE/Tools/experiment_logs/` contains 68 JSONL traces, 60 SVG maps and 57 JSON files. The published repo keeps selected [scan evidence](../tests/field-integration/2026-11-05-scan-experiments/README.md) and the [November 24 acceptance record](../tests/field-integration/2026-11-24-phase-02-acceptance/README.md).
