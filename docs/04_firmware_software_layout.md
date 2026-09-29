# ARGOS Firmware and Software Layout

**Document ID:** DOC-004
**Status:** Phase 00 Certified Baseline
**Scope:** Phase 00 certified implementation

## Purpose

This document records the firmware and software structure certified by the 2026-09-29 integrated bench test and identifies the work carried into Phase 01.

## Repository Layout

- `firmware/argos_f446re/`
  - STM32CubeIDE project for the NUCLEO-F446RE.
  - Contains the STM32 HAL project, board startup, ARGOS application code, and SSD1306 OLED driver.
- `firmware/esp32_c3_argos_bridge/`
  - ESP32-C3 Wi-Fi client/bridge and TCP-to-UART transport.
  - Sends a motor stop command when the control client disconnects.
  - Keeps local network credentials in ignored `wifi_secrets.h`; only `wifi_secrets.example.h` is versioned.
- `software/control_panel/`
  - Python desktop control panel for automatic Wi-Fi or COM connection, driving, telemetry, sensor visualization, and motor/encoder tests.

## STM32 Application Boundary

CubeMX-generated board startup remains in `Core/Src/main.c`. ARGOS application behavior is routed through:

- `Core/Inc/argos_platform.h`
  - Public platform interface and shared Phase 00 pin map.
- `Core/Src/argos_platform.c`
  - UART command console
  - Register-level I2C access
  - MPU6050 acquisition and gyro calibration
  - Dual VL53L0X address assignment and ranging
  - BTS7960 PWM motor control
  - Quadrature encoder counting and RPM telemetry
  - Motor timeout and direction-change safety behavior
- `Core/Inc/argos_oled.h` and `Core/Src/argos_oled.c`
  - Software-I2C SSD1306 driver
  - Connection, safety, motor, ToF, and IMU status display

## Phase 00 Certified Baseline

The implementation supports:

- STM32 build, upload, and debug workflow
- USB/UART commands and telemetry
- Independent left and right PWM motor control
- Safe startup, emergency stop, command timeout, and direction-change stop window
- Signed encoder count, direction, delta, and RPM telemetry
- MPU6050 raw and scaled telemetry
- Dual VL53L0X address assignment and distance telemetry
- ESP32-C3 Wi-Fi control through a TCP-to-UART bridge
- STM32-driven OLED status display
- Python desktop control and validation interface

Motor isolation, direction, encoder count, and RPM telemetry passed bench testing. Power Module V1 is approved for controlled development use with its instrumentation-dependent measurements still open.

The final automated system test passed UART, I2C discovery, IMU, both ToF sensors, motor safe-stop behavior, encoder response, and independent bidirectional motor ramps. Evidence is archived at `tests/phase-00/2026-09-29-system-test-report.txt`.

## Phase 01 Carry-Over Work

- Measure left and right motor deadband under installed mechanical load.
- Calibrate encoder CPR and gearbox ratio for accurate wheel RPM and odometry.
- Validate ToF distances and IMU angles against known references.
- Add and validate battery-level monitoring.
- Complete thermal and oscilloscope-based power-module checks when equipment is available.
- Split `argos_platform.c` into dedicated modules after integration behavior stabilizes.

Battery charge level is not currently measured or displayed. The formal decision and limitations are recorded in `tests/reports/2026-09-17_Phase00_Exit_Report.md`.
