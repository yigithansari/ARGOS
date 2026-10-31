# Phase 02 PWM-to-Wheel-RPM Characterization

**Date:** 2026-10-30
**Test window:** 01:01:31-01:02:07 Europe/Istanbul
**Link:** ESP32-C3 Wi-Fi bridge to STM32
**Result:** PASS WITH FOLLOW-UP

## Safety and Platform Health

- Final motor state: `SAFE:STOP`
- UART: PASS
- I2C: PASS, three devices (`0x30`, `0x31`, `0x68`)
- MPU6050: PASS
- Left ToF: PASS, 677 mm
- Right ToF: PASS, 853 mm
- Encoder activity: PASS, eight active samples per side
- Wi-Fi/STM link remained online throughout the test

## Calibrated No-load Wheel RPM

| Side | Direction | 50% | 60% | 70% | 80% |
| --- | --- | ---: | ---: | ---: | ---: |
| Left | Forward | 22.32 | 61.30 | 66.98 | 67.15 |
| Left | Reverse | 22.40 | 57.25 | 64.40 | 64.47 |
| Right | Forward | 20.38 | 60.82 | 68.20 | 68.44 |
| Right | Reverse | 19.15 | 64.95 | 69.42 | 69.48 |

The values use the accepted wheel encoder scales:

- Left: `48097.2 counts/wheel revolution`, forward sign `-1`
- Right: `48036.6 counts/wheel revolution`, forward sign `+1`

## Findings

1. Both motors and both encoder directions are working correctly.
2. The useful no-load speed range is approximately 20-69 wheel RPM.
3. A strong nonlinear transition exists between 50% and 60% PWM.
4. Both motors nearly saturate by 70%; increasing PWM from 70% to 80% produces
   almost no additional no-load speed.
5. Left forward/reverse maximum-speed asymmetry is about 4%; right-side maximum
   asymmetry is about 1.5%.
6. Paired-wheel mismatch from 60-80% averaged 4.4% with a maximum of 7.6%.
7. The 50% paired samples are less reliable for controller tuning because the
   motors are operating near the startup/deadband transition.

## Decision

Do not select PI gains from the four-point test alone. Run a focused transition
test at 50, 52, 54, 56, 58 and 60 percent with a settling interval separated
from the measurement interval. Use that result to build direction-specific
feed-forward values before enabling integral action.

## Next Step

Add a dedicated Phase 02 low-band characterization command and report. It must:

- test one motor at a time with wheels raised;
- keep the STM32 command-timeout safety active;
- settle before counting the measurement window;
- report calibrated wheel RPM;
- stop between samples and finish in `SAFE:STOP`;
- preserve the existing system-test workflow.
