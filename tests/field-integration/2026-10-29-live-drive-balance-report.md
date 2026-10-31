# ARGOS Live Drive Balance Report

**Date:** 2026-10-29
**Connection:** ESP32-C3 Wi-Fi bridge at `192.168.137.33:3333`
**Wheel state:** Driven wheels unloaded
**Measurement result:** PASS
**Open-loop balance result:** CHECK

## Method

The existing STM32 firmware was exercised directly over the Wi-Fi/UART bridge.
Both motors received the same command simultaneously. Encoder counters were
reset before every point, the command was refreshed within the motor safety
timeout, and an explicit stop was sent after each approximately one-second
sample. MPU and ToF streams were paused during the run and restored afterward.

Raw encoder count per second is used because absolute wheel-RPM calibration is
still pending. Positive mismatch means the left side was faster; negative means
the right side was faster.

## Results

| Direction | Command | Left count/s | Right count/s | Mismatch |
| --- | ---: | ---: | ---: | ---: |
| Forward | 50% | 15944.2 | 13801.5 | +14.41% |
| Forward | 60% | 52370.7 | 49805.3 | +5.02% |
| Forward | 70% | 57167.5 | 57645.6 | -0.83% |
| Forward | 80% | 51668.4 | 52570.2 | -1.73% |
| Reverse | 50% | 16658.7 | 15799.1 | +5.30% |
| Reverse | 60% | 46541.1 | 52904.4 | -12.80% |
| Reverse | 70% | 40996.5 | 43476.2 | -5.87% |
| Reverse | 80% | 49553.9 | 52369.5 | -5.53% |

## Interpretation

- Encoder measurement is repeatable enough for relative wheel-speed control.
- At the control panel's common 60% forward command, the left side is about 5%
  faster. This can produce a visible turn toward the right.
- Forward 70%-80% operation is close to balanced while unloaded.
- Reverse operation has a direction-dependent asymmetry: the right side is
  about 5.5%-12.8% faster from 60% to 80%.
- The 50% region is close to drivetrain deadband and is not suitable for a
  single fixed calibration value.
- Because mismatch changes with direction and PWM, a global static trim is not
  an adequate final correction.

## Decision

Do not assign the pull to chassis mechanics yet. Implement encoder-based
left/right speed synchronization first, then repeat a short on-floor straight
run. Absolute wheel RPM can be calibrated separately from motor-shaft encoder
CPR and gearbox ratio; synchronization can already use raw count-rate feedback.

The motor safety timeout and explicit stop behavior both operated during this
test. No ESP or STM reset was observed.
