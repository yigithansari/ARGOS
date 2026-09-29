# Phase 01 - Mobile Platform

**Document ID:** PHASE-001
**Status:** Active
**Start Date:** 2026-09-24
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

## 1. Purpose

Phase 01 converts the certified Phase 00 bench platform into the first integrated mobile ARGOS prototype.

The phase focuses on mechanical assembly, subsystem placement, wiring, differential-drive integration, motion calibration, and controlled mobile testing.

## 2. Current Status

Phase 01 is active.

The first mechanical milestone is complete:

- Chassis V1 has been designed.
- The Fusion 360 source model and DWG export are archived under `hardware/mechanical/chassis/v1/`.
- The Phase 00 bench platform remains the validated electrical and firmware baseline.

## 3. Current Deliverables

| ID | Deliverable | Status | Evidence |
| --- | --- | --- | --- |
| P01-001 | Chassis V1 design | Complete | `hardware/mechanical/chassis/v1/ARGOS_Chassis_V1.f3d` |
| P01-002 | Chassis drawing/export | Complete | `hardware/mechanical/chassis/v1/ARGOS_Chassis_V1.dwg` |
| P01-003 | Mechanical fit and clearance review | Planned | Validate component placement before final assembly. |
| P01-004 | Mobile-platform assembly | Planned | Install motors, electronics, sensors, and power system. |
| P01-005 | Wiring integration | Planned | Create controlled and serviceable platform wiring. |
| P01-006 | Initial motion test | Planned | Verify forward, reverse, turning, and safe stop on the assembled chassis. |

## 4. Engineering Work

- Validate chassis dimensions, mounting points, wheel alignment, and clearances.
- Define final placement for the STM32, ESP32-C3, OLED, motor drivers, sensors, and Power Module V1.
- Assemble the differential-drive platform.
- Characterize left and right motor deadband under mechanical load.
- Calibrate encoder CPR, gearbox ratio, wheel RPM, and odometry constants.
- Validate ToF geometry and IMU orientation after installation.
- Add and validate battery-voltage monitoring.
- Complete thermal and oscilloscope-based power checks when equipment is available.

## 5. Phase Boundary

Phase 00 is closed and shall not be reopened for Phase 01 calibration or mechanical-integration work. Changes affecting the certified bench baseline shall instead receive targeted regression testing and be documented in Phase 01.

## 6. Next Milestone

Perform the Chassis V1 fit and clearance review, then begin physical assembly of the first mobile platform.

