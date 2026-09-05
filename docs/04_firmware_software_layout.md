# ARGOS Firmware and Software Layout

**Document ID:** DOC-004
**Status:** Active Draft
**Scope:** Phase 00 bench validation implementation

## Purpose

This document records where the current working Phase 00 implementation lives in the repository and how it maps to the ARGOS architecture principles.

## Repository Layout

- `firmware/argos_f446re/`
  - STM32CubeIDE project for the NUCLEO-F446RE bench-validation firmware.
  - Contains the generated STM32 HAL project files, `Core/`, `Drivers/`, linker scripts, and `.ioc`.
- `software/control_panel/`
  - Python desktop control panel for serial telemetry, manual commands, sensor visualization, and motor/encoder tests.

## Firmware Module Boundary

The firmware now keeps CubeMX-generated board startup responsibilities in `Core/Src/main.c`.

ARGOS application behavior is routed through:

- `Core/Inc/argos_platform.h`
  - Public ARGOS platform interface.
  - Phase 00 pin map shared by generated GPIO setup and application code.
- `Core/Src/argos_platform.c`
  - UART command console.
  - Register-level I2C bus access.
  - IMU acquisition and gyro calibration.
  - VL53L0X address assignment and ranging.
  - BTS7960 PWM motor control.
  - Quadrature encoder counting and RPM telemetry.
  - Motor safety timeout and direction-change stop window.

This is the first modularization step: generated STM32 startup code is separated from ARGOS application behavior. The next split should extract the internal sections of `argos_platform.c` into dedicated modules:

- `argos_i2c`
- `argos_imu`
- `argos_tof`
- `argos_motor`
- `argos_encoder`
- `argos_console`
- `argos_safety`

## Phase 00 Compliance

Current implementation supports these Phase 00 validation requirements:

- STM32 build/upload/debug workflow.
- UART command and telemetry path.
- PWM motor control.
- Safe startup and stop behavior.
- Motor command timeout.
- Direction-change stop window.
- Signed encoder count.
- Encoder delta and RPM telemetry.
- IMU raw/scaled telemetry.
- ToF address assignment and range telemetry.
- Desktop validation panel with encoder self-test.

## Current Bench Validation Status

The first bench-validation experiments have been performed.

Confirmed working areas:

- VL53L0X ToF sensors have been tested.
- MPU6050 IMU has been tested.
- Motor drivers can drive the motors.
- UART telemetry and the Python control panel are available for bench observation.
- Power Module V1 is suitable for controlled Phase 00 bench use, with thermal and oscilloscope-based checks still open.

Known open issues:

- Encoder behavior is not fully reliable yet.
- Encoder pinout information from the vendor page appears to be incorrect.
- Low-PWM motor operation is not yet reliable.
- Low-PWM startup difficulty is likely caused by motor internal losses, gearbox friction, static friction, and inertia.

Planned hardware additions:

- Arduino prototype shield for cleaner STM32 NUCLEO wiring.
- ESP32-C3 Super Mini for wireless control experiments.
- 0.96 inch OLED display for local status indication.
- Battery level monitoring circuit for the power module.

## Remaining Work Before Phase 00 Exit

- Run encoder self-test on hardware and record PASS/DIR WARN/FAIL result.
- Confirm encoder direction sign for forward motion.
- Measure motor deadband for left and right motors.
- Record ToF test results against known distances.
- Record MPU6050 test results against known orientations.
- Validate the planned battery level monitoring circuit.
- Integrate ESP32-C3 wireless control after wired bench behavior is stable.
- Integrate OLED status display after telemetry fields are finalized.
- Split `argos_platform.c` internals into dedicated `.c/.h` modules after hardware behavior is confirmed stable.
