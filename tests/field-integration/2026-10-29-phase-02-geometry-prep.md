# Phase 02 Geometry Preparation

**Date:** 2026-10-29
**Result:** PASS

## Recorded Measurements

- Wheel circumference, left: `260.0 mm`
- Wheel circumference, right: `260.0 mm`
- Track width, wheel centre plane to wheel centre plane: `198.0 mm`
- Ideal per-wheel travel for an in-place 90-degree turn: `155.5 mm`

## Software Preparation

- Added a `PHASE 02 CALIBRATION` view to the desktop panel.
- Calibration explicitly stops the drive and never commands motor movement.
- Added guided zero, left 10-turn capture and right 10-turn capture actions.
- Added per-side counts-per-wheel-revolution and encoder-forward-sign capture.
- Added persistent `argos_motion_calibration.json` output.
- Calibrated telemetry converts encoder data to signed wheel RPM and centimetres.
- Python syntax verification passed.

## Encoder Scale Result

- Left: `-480972 counts / 10 turns` = `48097.2 counts/wheel revolution`
- Right: `+480366 counts / 10 turns` = `48036.6 counts/wheel revolution`
- Forward signs: left `-1`, right `+1`
- Scale difference: approximately `0.126%`
- Calibration persisted successfully at `2026-10-29T23:35:23+03:00`.

The magnitude is consistent with the encoder being mounted before the gearbox
and with the earlier observed count rates near the motor's nominal output speed.

## Next Evidence Required

Run the calibrated PWM-to-wheel-RPM characterization at 50, 60, 70 and 80
percent in both directions. Use those results to select PI gains and deadband
feed-forward values.
