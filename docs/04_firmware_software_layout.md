# ARGOS Firmware and Software Layout

**Document ID:** DOC-004
**Status:** Phase 00 certified; mobile chassis operational; Phase 02 acceptance in progress
**Scope:** Published source snapshot updated 2026-10-31

## Purpose

This document maps the current published STM32, ESP32 and desktop sources. Recent hardware results are summarized in the root README and archived under `tests/field-integration/`.

## Repository Layout

- `firmware/argos_f446re/`
  - STM32CubeIDE project for the NUCLEO-F446RE bench-validation firmware.
  - Contains the generated STM32 HAL project files, `Core/`, `Drivers/`, linker scripts, and `.ioc`.

- `firmware/esp32_c3_argos_bridge/`
  - ESP32-C3 Arduino sketch providing hotspot STA, fallback `ARGOS` AP, TCP/UART transport, discovery and cached STM32 OTA transfer.
  - Sends a motor stop command when the control client disconnects.
  - Keeps infrastructure and fallback-AP credentials in ignored `wifi_secrets.h`.

- `software/control_panel/`
  - Python desktop control panel for automatic Wi-Fi/COM connection, manual driving, serial telemetry, sensor visualization, and motor/encoder tests.

## Firmware Module Boundary

For this publication layout, import `firmware/argos_f446re` into STM32CubeIDE
and generate/build its Debug configuration first. The bundle script expects that
generated Debug makefile and the installed Windows CubeIDE/Python paths; it is
not a clean-checkout portable build system. Run its `Tools/build_wireless_bundle.ps1`
with `-SkipEspCompile`, then compile the separate `firmware/esp32_c3_argos_bridge`
sketch in Arduino IDE as ESP32C3 Dev Module. Copy the example Wi-Fi header to a
private local `wifi_secrets.h`. Select the resulting `Debug/ARGOS_OTA.bin` explicitly
in the published panel; its default image folder follows the active workspace layout.

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

- `Core/Inc/argos_oled.h` and `Core/Src/argos_oled.c`
  - Software-I2C SSD1306 driver on PA6/PA7.
  - Displays ARGOS identity, Wi-Fi client state, safety state, motor commands, ToF ranges, and IMU state.

Generated startup is separated from application behavior. The following are possible future module boundaries, not a required refactor before validation:

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
- ESP32-C3 Wi-Fi control path with automatic desktop connection.
- On-robot OLED status display.

## Current Implementation and Remaining Validation

- Wheel encoder scale/geometry and low-band PWM characterization are recorded in DOC-008.
- Per-wheel feed-forward/PI, command timeout and extended telemetry are implemented.
- `Bootloader/`, OTA linker and `Tools/` provide the persistent USART2 updater and packaging.
- Normal image links at `0x08000000`; OTA application at `0x08008000`.
- UART reception starts before slow OLED initialization; OLED refresh is limited to 500 ms.
- ESP sends complete telemetry lines and keeps startup NVS logging outside the first UART burst.
- Panel uses multipart OTA upload and rejects incompatible application vectors locally.
- Three-repeat PI acceptance, low-speed variation, reverse/disconnect-stop and floor tests remain.
- Validate ToF distances against known ruler distances.
- Validate IMU roll/pitch against known angles.
- Split modules only with a specific need and appropriate regression coverage.

Phase 00 functional evidence is archived at `tests/phase-00/2026-09-29-system-test-report.txt`.
