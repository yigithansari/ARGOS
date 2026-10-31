# Phase 01 - Mobile Platform

**Document ID:** PHASE-001
**Status:** Complete
**Start Date:** 2026-09-24
**Completion Date:** 2026-10-31
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

## 1. Purpose

Phase 01 converts the certified Phase 00 bench platform into the first integrated mobile ARGOS prototype.

The phase focuses on mechanical assembly, subsystem placement, wiring, differential-drive integration, motion calibration, and controlled mobile testing.

## 2. Current Status

Phase 01 is complete. The first mobile ARGOS platform is assembled and operational.

- Chassis V1 has been designed.
- The Fusion 360 source model and DWG export are archived under `hardware/mechanical/chassis/v1/`.
- Wheel V5 manufacturing files are archived under `hardware/mechanical/wheels/v5/`.
- Motors, electronics, power module, sensors, ESP32-C3, and OLED are installed on the chassis.
- The robot can move and stop safely under computer control through the hotspot connection.
- Field communication recovery and command-timeout behavior have been validated.
- The Phase 00 bench platform remains the validated electrical and firmware baseline.

## 3. Current Deliverables

| ID | Deliverable | Status | Evidence |
| --- | --- | --- | --- |
| P01-001 | Chassis V1 design | Complete | `hardware/mechanical/chassis/v1/ARGOS_Chassis_V1.f3d` |
| P01-002 | Chassis drawing/export | Complete | `hardware/mechanical/chassis/v1/ARGOS_Chassis_V1.dwg` |
| P01-003 | Mechanical fit and clearance review | Complete | Components installed on the physical chassis. |
| P01-004 | Mobile-platform assembly | Complete | Mobile platform photographs archived under `images/hardware/phase-01/`. |
| P01-005 | Wiring integration | Complete for prototype | STM32, ESP32-C3, motor drivers, sensors, OLED, and power system integrated. |
| P01-006 | Initial motion and safe-stop test | Complete | Basic drive, command timeout, and link recovery demonstrated. |

## 4. Phase 02 Carry-Over

- Tune independent left/right PI wheel-speed controllers.
- Complete repeated forward/reverse and floor tests.
- Implement differential-drive odometry and distance/angle commands.
- Add and validate battery-voltage monitoring.
- Complete thermal and oscilloscope-based power checks when equipment is available.

## 5. Phase Boundary

Phase 00 is closed and shall not be reopened for Phase 01 calibration or mechanical-integration work. Changes affecting the certified bench baseline shall instead receive targeted regression testing and be documented in Phase 01.

## 6. Exit Decision

Phase 01 exit is approved. Motion-control tuning and odometry continue under `phase-02-motion-control.md` without reopening the mechanical-integration phase.

