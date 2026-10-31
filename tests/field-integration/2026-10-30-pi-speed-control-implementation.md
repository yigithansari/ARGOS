# Phase 02 PI Wheel-Speed Control - Implementation Record

**Date:** 2026-10-30
**Status:** Software build PASS; hardware acceptance pending

## Inputs

- Wheel circumference: `260.0 mm`
- Track width: `198.0 mm`
- Left encoder scale: `48097.2 count/wheel rev`, forward sign `-1`
- Right encoder scale: `48036.6 count/wheel rev`, forward sign `+1`
- Feed-forward source: accepted low-band PWM/RPM characterization from
  `2026-10-30-lowband-rpm-characterization.md`

## Implemented

- Signed wheel-speed command: `V <left_rpm> <right_rpm>`
- Independent left/right fixed-point PI loops at 100 ms
- Direction-specific feed-forward interpolation
- Filtered calibrated wheel RPM from the motor-shaft encoders and measured
  gearbox/output-wheel count scale
- Target slew limiting and reversal through zero
- Integral anti-windup and `75%` output ceiling
- Existing 1200 ms STM32 command timeout retained for PI mode
- Extended motor telemetry containing target, ramped target, measured RPM,
  integral contribution and actual output
- Control-panel PWM/RPM selector, RPM-aware arrow-key drive and PI telemetry

## Initial Controller Values

- Target range: `15..55 rpm` for non-zero commands
- Ramp: `3 rpm / 100 ms`
- Proportional gain: approximately `0.18 %PWM / rpm`
- Integral gain: approximately `0.12 %PWM / (rpm*s)`
- Integral correction limit: `+/-12% PWM`
- Output ceiling: `75% PWM`

These are conservative first-pass values. They are not accepted tuning values
until the raised-wheel and floor tests are complete.

## Software Verification

- STM32 GNU build: PASS
- Firmware image: `ARGOS/ARGOS/Debug/ARGOS.elf`
- Image size: text `41036`, data `148`, bss `2988` bytes
- Python syntax compile: PASS
- PI telemetry parser smoke test: PASS

## Required Hardware Acceptance

1. Raise both wheels and select RPM mode in the panel.
2. Command `25 rpm` forward for a short interval; verify both measured RPM signs
   are positive and STOP removes output.
3. Repeat at `35 rpm` and `45 rpm`.
4. Repeat reverse and one in-place turn.
5. Verify link loss stops both outputs within the STM32 timeout.
6. Run a short floor test and record steady-state error, overshoot, left/right
   mismatch and any oscillation.

Target acceptance is steady-state tracking within `+/-5%` after settling, with
no sustained oscillation and reliable timeout stop.
