# ARGOS Drive Balance Precheck

**Date:** 2026-10-29
**Source:** `ARGOS/ARGOS/Tools/argos_system_test_report.txt`
**Status:** SUPERSEDED BY LIVE PAIRED TEST

The requested paired-wheel measurement was completed later the same day. See
`2026-10-29-live-drive-balance-report.md` for the accepted data and decision.

## Scope

This review checks whether the latest motor-ramp result explains the chassis
pulling to one side. The archived run tested each motor separately. It did not
measure both wheels under the same simultaneous chassis load.

## Equal-PWM Comparison

Positive mismatch means the left encoder rate was higher. Negative mismatch
means the right encoder rate was higher.

| PWM | Forward left | Forward right | Forward mismatch | Reverse left | Reverse right | Reverse mismatch |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 50% | 74121 | 64687 | +13.6% | 63660 | 54827 | +14.9% |
| 60% | 202830 | 196896 | +3.0% | 191208 | 216773 | -12.5% |
| 70% | 223090 | 226752 | -1.6% | 217901 | 231770 | -6.2% |
| 80% | 223885 | 227756 | -1.7% | 218881 | 232413 | -6.0% |

The values are the magnitude of the firmware's `RPMx100` speed index. Absolute
wheel RPM is not certified because motor-shaft encoder CPR and gearbox ratio
calibration are still pending.

## Findings

- Forward operation from 60% to 80% is close: average absolute mismatch is
  about 2.1%.
- Reverse operation from 60% to 80% is less symmetric: the right side is about
  6.0% to 12.5% faster.
- Both sides change behavior sharply around 50%, confirming that this region is
  near the drivetrain deadband and is unsuitable for precise open-loop motion.
- Encoder channels are active and direction signs are consistent. The pull
  diagnostic must use raw count rate until absolute wheel-RPM calibration is
  completed.
- The old report's cumulative encoder summary was invalid because it compared
  the final counter against a value captured before the firmware reset the
  counters. The ramp rows themselves remain usable.

## Software Correction

- The STM32 motor ramp now includes a paired-wheel balance section at 50%, 60%,
  70%, and 80% in both directions.
- The paired test reports left/right raw count per second and signed mismatch.
- The desktop report now evaluates encoder activity from ramp samples instead
  of the invalid pre-reset cumulative delta.
- The desktop report labels paired balance as PASS only when every 60%-80%
  sample remains within 8%.
- A motor-movement safety confirmation is shown before the system test starts.

## Next Acceptance Run

1. Flash the updated STM32 firmware and restart the updated control panel.
2. Lift the driven wheels or leave clear space in front of and behind the robot.
3. Run `SYSTEM TEST` once.
4. Review the new `MOTOR BALANCE TABLE` in the generated report.
5. If forward mismatch exceeds 5% consistently, add encoder-based wheel-speed
   synchronization before blaming chassis mechanics.
6. If forward mismatch stays below 5% while the robot still pulls, inspect wheel
   diameter, wheel alignment, motor mounting, caster drag, chassis twist, and
   left/right weight distribution.

This precheck does not certify straight-line motion. It defines the measurement
needed to separate control asymmetry from a mechanical fault.
