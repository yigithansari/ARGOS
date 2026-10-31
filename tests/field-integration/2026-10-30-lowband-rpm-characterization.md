# Phase 02 Low-band Wheel-RPM Characterization

**Date:** 2026-10-30
**Test window:** 01:20:38-01:22:17 Europe/Istanbul
**Result:** PASS
**Final safety state:** `SAFE:STOP`

## Method

- Wheels raised; no-load test.
- One motor energized at a time.
- PWM commands: 50, 52, 54, 56, 58 and 60 percent.
- Both directions tested on both motors.
- Settling interval: 1400 ms.
- Measurement interval: nominally 1600 ms; the report records actual elapsed
  time after link/telemetry latency.
- Panel keepalive remained active, preserving the STM32 1200 ms link-loss
  timeout as the final safety layer.

## Results

| PWM | Left FWD | Left REV | Right FWD | Right REV |
| ---: | ---: | ---: | ---: | ---: |
| 50% | +20.40 | -18.92 | +21.39 | -12.67 |
| 52% | +23.50 | -27.59 | +33.85 | -32.34 |
| 54% | +27.00 | -31.13 | +34.34 | -34.68 |
| 56% | +50.47 | -39.54 | +40.99 | -59.86 |
| 58% | +64.54 | -53.37 | +65.31 | -66.71 |
| 60% | +60.65 | -60.22 | +61.78 | -67.82 |

Values are calibrated output-wheel RPM. Positive means robot-forward wheel
direction; reverse samples are negative.

## Findings

1. Every motor/direction pair produced valid, correctly signed encoder data.
2. Static-friction/deadband transition is direction-specific:
   - Left forward: strongest transition around 54-58%.
   - Left reverse: progressive transition around 56-60%.
   - Right forward: strongest transition around 56-58%.
   - Right reverse: strongest transition around 52-56%.
3. The no-load maximum is approximately 61-68 RPM in this supply/mechanical
   configuration.
4. The 58-to-60% non-monotonic forward samples are treated as saturation and
   short-window measurement variation, not as an inverse motor model.
5. A single shared PWM-to-RPM line is unsuitable. Initial speed control needs
   side- and direction-specific feed-forward plus closed-loop PI correction.

## Initial Feed-forward Anchors

These are conservative starting points, not final floor-load calibration:

| Wheel direction | ~20 RPM | ~30 RPM | ~40 RPM | ~50 RPM | ~60 RPM |
| --- | ---: | ---: | ---: | ---: | ---: |
| Left forward | 50 | 54 | 55 | 56 | 58 |
| Left reverse | 50 | 53 | 56 | 58 | 60 |
| Right forward | 50 | 52 | 56 | 57 | 58 |
| Right reverse | 51 | 52 | 54 | 55 | 56 |

Interpolation must be monotonic and output-limited. Floor-load testing may
shift all anchors upward.

## PI Implementation Decision

- Run independent left/right wheel loops.
- Use calibrated wheel RPM and direction-specific feed-forward.
- Start with conservative proportional/integral gains and anti-windup.
- Apply a target-speed ramp instead of a step command.
- Preserve immediate stop, direction-change dead time and command timeout.
- Initially limit commanded wheel speed to approximately 55 RPM until loaded
  floor tests validate stability.
