# Phase 00 - Bench Validation Status Update

**Date:** 2026-09-06
**Phase:** Phase 00 - Bench Validation
**Topic:** First bench validation results and remaining issues

## Summary

The first ARGOS bench validation work has been performed. The project is no longer only in planning; it is now actively validating Phase 00 hardware, firmware, sensors, motor control, and support software on the bench.

## Completed or Partially Completed Work

- Power Module V1 passed the available electrical validation checks.
- STM32 NUCLEO-F446RE firmware project was added to the main ARGOS repository.
- Python control panel was added to the main ARGOS repository.
- VL53L0X ToF sensors were tested.
- MPU6050 IMU was tested.
- Motor drivers are able to drive the motors.

## Open Measurement Items

- Detailed thermal validation was not performed.
- Oscilloscope-based ripple and transient measurements were not performed.

These checks remain open and should be completed when the required equipment is available.

## Known Issues

### Encoder Wiring

The motor encoder wiring information from the vendor page appears to be incorrect. This is likely contributing to unreliable encoder behavior and direction/counting uncertainty.

### Low-PWM Motor Operation

The motors are difficult to drive reliably at low PWM values. The likely cause is that the motors have enough internal losses, gearbox friction, static friction, and inertia that very low duty cycles cannot reliably start rotation.

This should be characterized as motor deadband before PID tuning.

## Planned Hardware Additions

- Arduino prototype shield for cleaner STM32 NUCLEO wiring.
- ESP32-C3 Super Mini for wireless control experiments.
- 0.96 inch OLED display for local status indication.
- Battery level monitoring circuit for the power module.

## Decision

Continue Phase 00 bench validation before chassis integration.

The next engineering focus is to correct encoder wiring, characterize motor deadband, and document stable operating ranges for the current motor and driver combination.

## Next Actions

- Identify the correct encoder pinout experimentally.
- Confirm encoder A/B channel behavior.
- Confirm direction sign for forward and reverse motion.
- Measure minimum usable PWM duty cycle for each motor.
- Evaluate whether firmware should include a startup kick or minimum PWM threshold.
- Add battery level monitoring to the power module.
- Update diagrams after wiring changes are confirmed.
