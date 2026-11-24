# Raised-wheel PI gain comparison — 2026-11-04

Operator explicitly secured the robot with both wheels fully clear. Two35 RPM,
20 s runs used the same existing feedback/stall/overspeed and STOP protections.

| Final4 s metric | Baseline left Kp0.18 | Candidate left Kp0.08 |
| --- | --- | --- |
| Left mean RPM |35.585 |35.527 |
| Right mean RPM |35.100 |35.081 |
| Left RPM range |30.83–41.30 |30.67–41.67 |
| Left target-error RMS, RPM |3.763 |4.040 |
| Left samples within±5% |27.3% |18.2% |
| Final STOP | Confirmed | Confirmed |

Right Kp0.18, Ki, motor limits, timeout and motion guards were unchanged. The
candidate did not improve the left ripple and was rejected. Candidate CRC05F55FF3
was replaced by baseline CRC0C70506B (44736 bytes); rebuilt source matched the
saved baseline byte-for-byte, and wireless CRC plus STOP verification passed.
No PI gain change remains in the active firmware.

`phase_analysis.json` fits filtered telemetered RPM against encoder phase over
samples at least8 s after the first PI status timestamp. Left first-harmonic
R² was0.984 baseline and0.985 candidate, amplitudes5.10/5.28 RPM. Right R² was
0.031/0.015. This is diagnostic evidence of a strong once-per-wheel-revolution
component unaffected by this gain reduction. It does not distinguish actual
shaft-speed ripple from encoder nonuniformity, nor prove a mechanical cause.
`pi_left_gain_trial_20261104/analyze.py` preserves the calculation and was run
from its original Tools/installation_backups location using central geometry.

PI remains provisional: mean tracking passes, left instantaneous tracking fails.
Next useful isolation: power off, remove only the left wheel, preserve motor and
encoder wiring, secure with right wheel clear, then repeat after explicit ready.
No wheel-removed run has been authorized as ready or executed yet.
