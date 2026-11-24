# ARGOS Phase 02 - Motion Control

**Started:** 2026-10-29  
**Status:** Phase 02 closed with carry-over on 2026-11-25. See the [phase closeout](../phases/phase-02-motion-control.md).

**Latest review:** 2026-11-24. Three 1 m runs measured 987.5/972.5/982.5 mm
at the wheel-center midpoint. Left 90 degree turns measured 88/88/88 degrees;
right turns measured 87/89/89 degrees. All met the floor criteria and confirmed
STOP. Raised speed means passed, while instantaneous +/-5% tracking did not.
Three watchdog trials confirmed output stop but their sampled intervals crossed
1200 ms. [Evidence and raw records](../tests/field-integration/2026-11-24-phase-02-acceptance/README.md)

The ESP panel includes a repeatable acceptance screen and external measurement
entry. Its supervised floor tests run with operator-confirmed clear space and
range clearance disabled. Raspberry Pi integration and prototype mapping are
also available. These results record the 2026-11-24 acceptance checkpoint.

## Earlier results

**Updated:** 2026-10-31. Single forward raised-wheel runs completed at 25/35/45 RPM:

| Target RPM | Mean left/right RPM | Settled samples within +/-5% (L/R) |
| --- | --- | --- |
| 25 | 25.58 / 25.64 | 73% / 45% |
| 35 | 36.11 / 34.64 | 55% / 45% |
| 45 | 44.79 / 45.27 | 100% / 100% |

STOP passed after each run. Single-command timeout returned TIMEOUT_STOP;
exact physical stop latency was not measured. All means meet the initial target,
but low-speed variation and three-repeat acceptance remain open. Reverse,
disconnect-stop and floor tests are still pending. Earlier intermittent feedback
gaps are not conclusively resolved by these single successful runs.

Evidence: `tests/field-integration/2026-10-31-motion-progress/`.
Follow-up: `tests/field-integration/2026-10-31-repeated-motion/` records nine
completed forward runs, all mean targets met, and three 45 RPM runs with all
settled samples in band. The 25/35 RPM instantaneous requirement remains open.
Short reverse/timeout and post-disconnect STOP checks passed. A fractional-PWM
trial did not establish improvement and was rolled back to CRC `6C6BC21F`.
OLED refresh is bounded at 500 ms; test capture now buffers logs in memory rather
than reopening a OneDrive file for every received line. Motor gains/scaling unchanged.

## Objective

Operator decision on 2026-11-04: continue with periodic left-side variation as a
provisional limitation. Normal wiring was independently reverified; firmware/gains
remain unchanged. Use 20 s repeated runs and final 4 s mean +/-5%, keeping
instantaneous tracking visible and safety aborts intact. This is not full tracking
or floor acceptance. Evidence: `tests/field-integration/2026-11-04-provisional-operation/`.

New-wheel raised validation on 2026-11-04: twelve principal repeated runs at
+25/+35/+45/-25 RPM completed with STOP; final means within band after the documented
10/20 s windows. Initial 35 RPM/10 s tolerance failure retained. Right tracking
improved substantially; left instantaneous tracking remains unaccepted, including
PI-off single-wheel variation. Timeout/disconnect screening passed. Firmware/gains
unchanged. Full results and physical-isolation next step:
`tests/field-integration/2026-11-04-new-wheels/`.

Latest diagnosis: constant PWM also varies, including single-motor runs (CV about
5.3-5.6% over overlapping one-second windows). PI is not established as the sole
cause. See `tests/field-integration/2026-10-31-open-loop-comparison/`; supply and
external-speed measurements are pending before further gain changes.

Move from percentage-based motor commands to measured wheel speed, distance and
heading control. Phase 02 will add per-wheel speed control and differential-drive
odometry without weakening the existing command-timeout safety layer.

## Measured Robot Geometry

- Updated 2026-11-04: operator replaced the rubbing wooden wheels; zero contact
  reported, new-wheel motion validation pending. Earlier tests above refer to old wheels.
- Nominal diameter: `87 mm` at the outside of the tread (both wheels).
- Derived circumference: `273.31856 mm`; loaded rolling measurement pending.
- Track width: `255.0 mm` (operator confirmed centre-plane measurement).
- Track-width definition: distance between the left and right wheel centre planes

The 90-degree in-place-turn model therefore requires each wheel to travel about
`200.28 mm` in opposite directions. This is only the geometric starting point;
floor slip and the wooden-wheel contact geometry will require an empirical turn
correction later.

## Why Encoder Scale Must Be Measured

The encoders are mounted on the motor shafts before the gearbox. Wheel RPM cannot
be derived reliably from a motor-shaft pulse specification alone unless encoder
resolution, quadrature multiplier and exact gearbox ratio are all confirmed.

The control panel therefore measures each side independently:

1. Lift the wheels and stop both motors.
2. Mark a repeatable point on each wheel.
3. Zero the encoder counters.
4. Rotate the left wheel exactly 10 output-wheel turns in its forward-driving
   direction and capture the count.
5. Repeat for the right wheel.
6. Save the resulting counts per wheel revolution and forward sign.

The saved file is `Tools/argos_motion_calibration.json`. Version 2 defines diameter
and track centrally, deriving circumference; legacy circumference files still load.
The panel's measured-circumference input saves an equivalent diameter. Once both sides are
measured, the desktop telemetry converts encoder data into signed wheel RPM and
travel distance.

## Accepted Encoder Scale

**Measurement date:** 2026-10-29  
**Method:** 10 manual output-wheel turns per side, in the forward-driving direction

- Left total delta: `-480972 counts`
- Left scale: `48097.2 counts/wheel revolution`
- Left forward sign: `-1`
- Left nominal distance scale with current wheels: `0.00568263 mm/count`
- Right total delta: `+480366 counts`
- Right scale: `48036.6 counts/wheel revolution`
- Right forward sign: `+1`
- Right nominal distance scale with current wheels: `0.00568980 mm/count`
- Left/right scale difference: approximately `0.126%`

Derived open-loop odometry starting values:

- 1000 mm straight with new nominal geometry: left `175975` counts, right `175753` counts
- Ideal 90-degree in-place turn: left `35244` counts, right `35199` counts,
  in opposite directions

The small left/right difference is credible and will be retained rather than
forcing both sides to a shared average.

## Implementation Sequence

1. Geometry and counts-per-wheel-revolution calibration. **COMPLETE**
2. PWM-to-wheel-RPM characterization at 50, 60, 70 and 80 percent in both
   directions. **COMPLETE; focused 50-60% follow-up required**
   - Dedicated panel-driven low-band test implemented at 50, 52, 54, 56, 58
     and 60 percent.
   - Motor commands are refreshed by the panel so loss of the control link
     still activates the STM32 1200 ms timeout.
   - Hardware run PASS on 2026-10-30; direction-specific feed-forward anchors
     recorded in `tests/field-integration/2026-10-30-lowband-rpm-characterization.md`.
3. Independent left/right wheel-speed PI loops. **IMPLEMENTED; HARDWARE TUNING PENDING**
   - New `V left_rpm right_rpm` command accepts signed wheel-speed targets.
   - Target range is limited to `+/-55 rpm`; non-zero requests below `15 rpm`
     are raised to the current reliable motor floor.
   - Direction-specific feed-forward values come from the accepted 2026-10-30
     low-band characterization.
   - Each wheel has its own fixed-point PI correction, filtered RPM feedback,
     integral anti-windup and output limit.
   - Direction changes ramp through zero. The existing independent STM32
     command timeout remains active.
4. Straight-line target speed using both PI loops. **THREE FLOOR DISTANCE TRIALS PASSED**
5. Differential-drive odometry. **IMPLEMENTED; ENCODER ESTIMATE**
6. Distance and angle commands. **SUPERVISED FLOOR TRIALS PASSED**
   - External turn measurements passed in both directions.
   - IMU heading comparison and fusion remain open.

## Initial Acceptance Targets

- Wheel-speed tracking: within `+/-5%` after settling.
- Straight 1000 mm command: within `+/-50 mm` during initial floor testing.
- 90-degree turn: within `+/-5 degrees` after empirical correction.
- Link loss: motor output stops within the existing STM32 `1200 ms` command
  timeout.
- Three repeated runs produce comparable results before a calibration is
  accepted.

## Safety Rule

Encoder calibration is manual and does not energize the motors. PWM/RPM and PI
tests must only be started after the operator explicitly confirms that the wheels
are raised or the robot has a clear test lane.

The first PI validation must begin at `25 rpm` with the wheels raised. Confirm
encoder direction, measured RPM and STOP behavior before any floor run. The
controller intentionally does not attempt hard braking; the geared motors and
wooden wheels may coast after PWM is removed.
