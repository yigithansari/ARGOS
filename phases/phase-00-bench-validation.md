# Phase 00 - Bench Validation

**Document ID:** PHASE-000
**Document Title:** Phase 00 - Bench Validation
**Version:** 1.1
**Status:** Active Bench Validation
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

---

## 1. Purpose

Phase 00 is the foundation of the ARGOS project.

The objective of this phase is to validate every critical hardware and firmware subsystem individually before integrating them into a mobile robotic platform.

No chassis is required during this phase. The focus is controlled bench validation, subsystem bring-up, firmware development, wiring verification, and early software tooling.

---

## 2. Current Phase Status

ARGOS is currently in **Phase 00 - Bench Validation**.

The first bench validation experiments have been performed. Several subsystems are already working, while a few issues remain under investigation before mechanical integration.

Current confirmed progress:

- STM32 NUCLEO-F446RE firmware project has been created.
- UART-based command and telemetry path is available.
- Python control panel has been added for bench testing.
- Power Module V1 has passed available electrical validation checks.
- VL53L0X ToF sensors have been tested.
- MPU6050 IMU has been tested.
- Motor drivers can drive the motors.

Current known issues:

- Motor encoder behavior is not fully reliable yet.
- Low-PWM motor operation is problematic.
- Encoder wiring information from the vendor page appears to be incorrect.
- Low-PWM motor startup is likely affected by internal motor losses and the motor's difficulty overcoming static friction and inertia.

Open measurements:

- Detailed thermal validation has not been performed.
- Oscilloscope-based ripple and transient measurements have not been performed.

---

## 3. Objectives

The primary objectives of Phase 00 are:

- Establish the embedded development environment.
- Validate all selected hardware components.
- Develop reusable firmware modules.
- Verify communication interfaces.
- Understand the behavior of every subsystem.
- Create reliable test procedures.
- Build confidence before mechanical integration.

---

## 4. Required Hardware

### Main Controller

- STM32 NUCLEO-F446RE

### Prototype Interface

- Arduino prototype shield for STM32 NUCLEO expansion and temporary wiring

### Motion System

- 2 x 12V encoder DC motors
- BTS7960 motor driver modules

### Sensors

- MPU6050 IMU
- VL53L0X ToF sensors

### Power

- Power Module V1
- 3S Li-ion battery pack
- 3S BMS
- Main switch
- 5A blade fuse
- LM2596 buck converter
- Planned battery level monitoring circuit

### Communication and User Feedback

- USB serial / UART connection
- ESP32-C3 Super Mini for planned wireless control
- 0.96 inch OLED display for planned local status indication

### Tools

- Multimeter
- USB cable
- Logic analyzer, optional
- Oscilloscope, optional

---

## 5. Firmware Modules

During Phase 00 the firmware is being divided into reusable modules.

Current implementation lives under:

- `firmware/argos_f446re/`

Current firmware responsibilities include:

- UART command console
- I2C device communication
- MPU6050 acquisition
- VL53L0X ranging
- PWM motor control
- Encoder counting and RPM telemetry
- Motor safety timeout
- Direction-change stop window

Planned module split:

- GPIO driver
- PWM driver
- Encoder driver
- UART driver
- I2C driver
- Motor driver
- IMU driver
- ToF driver
- PID controller
- Debug utilities
- Safety utilities

---

## 6. Validation Tasks

| ID | Task | Status | Notes |
| --- | --- | --- | --- |
| P00-001 | STM32 development environment | Passed | Firmware project exists and bench firmware is active. |
| P00-002 | LED / basic firmware operation | Passed | Basic firmware execution confirmed. |
| P00-003 | UART communication | Passed | Command and telemetry path available. |
| P00-004 | Python control panel | Passed | Control panel copied into main ARGOS repository. |
| P00-005 | Power Module V1 validation | Passed with open items | Electrical checks passed; thermal and oscilloscope checks remain open. |
| P00-006 | PWM generation | Passed | PWM motor control is available. |
| P00-007 | Motor driver operation | Partially Passed | Motors can be driven, but low-PWM operation is not yet satisfactory. |
| P00-008 | Encoder interface | Partially Passed | Encoder wiring and signal behavior require further investigation. |
| P00-009 | MPU6050 IMU | Passed | IMU has been tested. |
| P00-010 | VL53L0X ToF sensors | Passed | ToF sensors have been tested. |
| P00-011 | ESP32-C3 wireless control expansion | Planned | Planned for wireless control experiments. |
| P00-012 | OLED status display | Planned | Planned for local status indication. |
| P00-013 | Battery level monitoring | Planned | Planned addition to the power module. |

---

## 7. Known Technical Issues

### Encoder Wiring

The encoder pinout information from the vendor page appears to be incorrect or incomplete. This can cause swapped channels, incorrect direction detection, unreliable pulse counting, or unexpected RPM telemetry.

Next steps:

- Identify encoder VCC, GND, A, and B lines directly.
- Verify channel behavior manually.
- Confirm direction sign during forward and reverse motor motion.
- Update wiring diagrams after the corrected pinout is confirmed.

### Low-PWM Motor Drive

The motors are difficult to drive reliably at low PWM values.

The likely cause is that the motor cannot overcome static friction, gearbox friction, and internal losses at very low duty cycles. This behavior should be characterized as motor deadband rather than treated only as a firmware problem.

Next steps:

- Measure minimum PWM duty cycle required for left motor startup.
- Measure minimum PWM duty cycle required for right motor startup.
- Add a startup kick or minimum PWM threshold if required.
- Document deadband values before PID tuning.

---

## 8. Planned Hardware Additions

The following hardware additions are planned during the remaining Phase 00 bench validation work:

- Arduino prototype shield for cleaner STM32 NUCLEO wiring
- ESP32-C3 Super Mini for wireless control experiments
- 0.96 inch OLED display for local status and debug information
- Battery level monitoring circuit for the power module

These additions are intended to improve bench usability and prepare the platform for cleaner integration in later phases.

---

## 9. Exit Criteria

Phase 00 shall be considered complete only if all of the following conditions are satisfied:

- STM32 development environment is fully operational.
- UART debugging and telemetry are functional.
- Power Module V1 is validated for bench use.
- PWM generation has been verified.
- Motor driver operation is reliable across the intended PWM range.
- Encoder data is reliable and direction-correct.
- IMU communication is stable.
- ToF sensors are operational.
- Firmware modules are organized and reusable.
- Known wiring corrections are documented.
- All validation tests have been documented.

No chassis integration shall begin before these conditions are met.

---

## 10. Completion Statement

Phase 00 is not about building the final robot.

It is about building confidence in each subsystem before integration. Current bench validation has confirmed meaningful progress, but encoder behavior and low-PWM motor control must be resolved before ARGOS moves into full mobile platform integration.
