# Phase 00 - Bench Validation

**Document ID:** PHASE-000
**Document Title:** Phase 00 - Bench Validation
**Version:** 2.1
**Status:** Complete - Certified
**Completion Date:** 2026-09-17
**Certification Date:** 2026-09-29
**Project:** ARGOS - Autonomous Robotic Guard & Observation System

---

## 1. Purpose

Phase 00 is the foundation of the ARGOS project.

The objective of this phase is to validate every critical hardware and firmware subsystem individually before integrating them into a mobile robotic platform.

No chassis is required during this phase. The focus is controlled bench validation, subsystem bring-up, firmware development, wiring verification, and early software tooling.

---

## 2. Current Phase Status

ARGOS completed **Phase 00 - Bench Validation** on 2026-09-17. The integrated bench platform passed its final automated system test on 2026-09-29 and is certified to proceed to Phase 01 mechanical and mobile-platform integration.

Current confirmed progress:

- STM32 NUCLEO-F446RE firmware project has been created.
- UART-based command and telemetry path is available.
- Python control panel has been added for bench testing.
- Power Module V1 has passed available electrical validation checks.
- VL53L0X ToF sensors have been tested.
- MPU6050 IMU has been tested.
- Motor drivers can drive the motors independently.
- Motor direction, encoder count, and RPM telemetry passed the bench ramp test.
- ESP32-C3 wireless control and an SSD1306 OLED status display have been integrated.

Accepted Phase 01 carry-over items:

- Encoder CPR and gearbox ratio require calibration for accurate wheel RPM and odometry.
- Low-PWM motor deadband requires characterization under the installed mechanical load.
- ToF distance and IMU angle accuracy require reference measurements.
- Battery-level monitoring remains to be added.

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
- ESP32-C3 Super Mini for wireless control
- 0.96 inch OLED display for local status indication

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
| P00-003 | UART communication | Passed | Final system test confirmed command and telemetry exchange. |
| P00-004 | Python control panel | Passed | Generated and archived the final integrated system-test report. |
| P00-005 | Power Module V1 validation | Passed with open items | Electrical checks passed; thermal and oscilloscope checks remain open. |
| P00-006 | PWM generation | Passed | PWM motor control is available. |
| P00-007 | Motor driver operation | Passed | Independent forward/reverse ramps at 50, 60, 70, and 80 percent passed; low-PWM deadband is carried into Phase 01. |
| P00-008 | Encoder interface | Passed with calibration pending | Both encoders responded during the final ramp test; CPR and gearbox ratio require calibration. |
| P00-009 | MPU6050 IMU | Passed | IMU has been tested. |
| P00-010 | VL53L0X ToF sensors | Passed | ToF sensors have been tested. |
| P00-011 | ESP32-C3 wireless control expansion | Passed | SoftAP and TCP-to-UART control path implemented with disconnect stop behavior. |
| P00-012 | OLED status display | Passed | STM32-driven SSD1306 status display integrated. |
| P00-013 | Battery level monitoring | Carried to Phase 01 | Planned addition to the power module. |

---

## 7. Known Technical Issues

### Encoder Calibration

The vendor pinout information required bench verification. Encoder direction and pulse counting now operate, but the physical scale is not final.

Next steps:

- Measure motor-shaft encoder CPR.
- Confirm the gearbox ratio.
- Calibrate wheel RPM and odometry constants.

### Low-PWM Motor Drive

The motors are difficult to drive reliably at low PWM values.

The likely cause is that the motor cannot overcome static friction, gearbox friction, and internal losses at very low duty cycles. This behavior should be characterized as motor deadband rather than treated only as a firmware problem.

Next steps:

- Measure minimum PWM duty cycle required for left motor startup.
- Measure minimum PWM duty cycle required for right motor startup.
- Add a startup kick or minimum PWM threshold if required.
- Document deadband values before PID tuning.

---

## 8. Hardware Expansion Status

The Phase 00 expansion status is:

- Arduino prototype shield used for cleaner STM32 NUCLEO wiring
- ESP32-C3 Super Mini wireless bridge integrated
- 0.96 inch SSD1306 OLED status display integrated
- Battery-level monitoring circuit carried into Phase 01

These additions improve bench usability and provide the communication and status foundation for Phase 01.

---

## 9. Exit Criteria

Phase 00 was assessed against the following conditions:

- STM32 development environment is fully operational.
- UART debugging and telemetry are functional.
- Power Module V1 is validated for bench use.
- PWM generation has been verified.
- Motor-driver direction, isolation, stop, and ramp operation are reliable in the validated range.
- Encoder count and direction data are reliable; physical scaling is explicitly carried forward.
- IMU communication is stable.
- ToF sensors are operational.
- Firmware modules are organized and reusable.
- Known wiring corrections are documented.
- All validation tests have been documented.

All required exit conditions are met or have an accepted, documented carry-over action. See `tests/reports/2026-09-17_Phase00_Exit_Report.md`.

Final functional evidence is archived at `tests/phase-00/2026-09-29-system-test-report.txt`.

---

## 10. Completion Statement

Phase 00 is not about building the final robot.

It is about building confidence in each subsystem before integration. Phase 00 established that foundation, documented its limitations, and authorized ARGOS to proceed to Phase 01.
