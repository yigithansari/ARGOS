# New-wheel raised validation - 2026-11-04

Operator confirmed robot raised, secured and connected. New non-rubbing wheels:
nominal 87 mm tread diameter, 255 mm centre-plane track. Firmware stayed 5E1584A0;
no control gains, scaling, geometry, motor timeout or firmware changed in this test.

## Speed screening

- 25 RPM: three 10 s forward runs completed; all final-four-second means within
  +/-5%. Left instantaneous tracking remains outside band; right improved.
- 35 RPM: initial 10 s run 2 failed mean tolerance (L36.708/R36.863 RPM);
  series stopped with confirmed STOP. This failed result was retained.
- To investigate settling rather than modify gains, three 20 s runs each at
  35/45 RPM evaluated seconds 16-20. All means within +/-5%, every STOP confirmed.
  Different evaluation windows must not be conflated with improved 10 s tracking.
- Reverse -25 RPM: three 20 s runs, all final means within +/-5% and STOP passed.

| Target RPM | Range of final means L | Range of final means R | Instantaneous band fraction L / R |
| --- | --- | --- | --- |
| +25 | 25.52-25.77 | 25.15-25.45 | 18-36% / 73-100% |
| +35 | 34.92-35.81 | 34.36-35.13 | 18-36% / 91-100% |
| +45 | 44.65-45.36 | 46.13-46.24 | 36-55% / 100% |
| -25 | -25.01 to -24.96 | -25.11 to -24.96 | 27-30% / 90-100% |

Twelve principal repeated speed runs completed, plus the two initial 35 RPM
10 s runs. This is mean/direction screening, not full instantaneous PI acceptance.
External shaft speed, loaded rolling diameter and floor behavior were not measured.
Reverse 35/45 and continuous direction transitions were not accepted by this test.

## Safety checks

- Unrefreshed single +25 command: TIMEOUT_STOP then final STOP observed.
  Thirty concurrent HTTP status probes succeeded; maximum 234 ms.
- Control socket closed after one short command: reconnect showed STOP/zero PWM.
- Exact electrical motor-pin levels/stop latency were not measured.
- Final isolated-left fixed P 52 0 / 10 s run also ended in confirmed STOP.

## Remaining left-side diagnosis

With PI disabled and right motor stopped, left one-second speed mean was
36.31 RPM, range 33.77-38.82, CV 5.06% (nine overlapping windows).
Exploratory first-wheel-harmonic fit: 17 interval samples, amplitude 6.49 RPM,
R-squared 0.979. This supports a wheel-phase-related left-side effect, but phase
and speed derive from the same encoder; no unique physical cause is established.
At +25 PI, left harmonic association remained, while right was negligible.

Next physical isolation: with power OFF, swap only compatible wheels left/right,
retaining motor/encoder wiring and shaft retention; then repeat the same low-speed
condition. An effect following the wheel implicates wheel/hub fit; an effect staying
left directs inspection to left motor/gear/mount or encoder. No swap performed yet.
Do not tune gains to conceal the unresolved periodic effect. Robot ended STOP.
