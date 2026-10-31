# ARGOS Phase 02 - Motion Control

**Started:** 2026-10-29
**Status:** First closed-loop wheel-speed controller implemented; hardware tuning pending

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
OLED refresh is bounded at 500 ms; test capture now buffers logs in memory rather
than reopening a OneDrive file for every received line. Motor gains/scaling unchanged.

## Objective

Move from percentage-based motor commands to measured wheel speed, distance and
heading control. Phase 02 will add per-wheel speed control and differential-drive
odometry without weakening the existing command-timeout safety layer.

## Measured Robot Geometry

- Left wheel circumference: `260.0 mm`
- Right wheel circumference: `260.0 mm`
- Track width: `198.0 mm`
- Track-width definition: distance between the left and right wheel centre planes

The 90-degree in-place-turn model therefore requires each wheel to travel about
`155.5 mm` in opposite directions. This is only the geometric starting point;
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

The saved file is `Tools/argos_motion_calibration.json`. Once both sides are
measured, the desktop telemetry converts encoder data into signed wheel RPM and
travel distance.

## Accepted Encoder Scale

**Measurement date:** 2026-10-29
**Method:** 10 manual output-wheel turns per side, in the forward-driving direction

- Left total delta: `-480972 counts`
- Left scale: `48097.2 counts/wheel revolution`
- Left forward sign: `-1`
- Left distance scale: `0.00540572 mm/count`
- Right total delta: `+480366 counts`
- Right scale: `48036.6 counts/wheel revolution`
- Right forward sign: `+1`
- Right distance scale: `0.00541254 mm/count`
- Left/right scale difference: approximately `0.126%`

Derived open-loop odometry starting values:

- 1000 mm straight: left `184989` counts, right `184756` counts
- Ideal 90-degree in-place turn: left `28767` counts, right `28731` counts,
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
4. Straight-line target speed using both PI loops. **PANEL SUPPORT IMPLEMENTED;
   FLOOR ACCEPTANCE PENDING**
5. Differential-drive odometry.
6. Distance and angle commands with IMU comparison.

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
