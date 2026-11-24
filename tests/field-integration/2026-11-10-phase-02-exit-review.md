# Phase 02 Exit Review

**Date:** 2026-11-10  
**Decision:** Acceptance remains open. Resume motion validation before Raspberry Pi integration.

Criteria are defined in [Phase 02 Motion Control](../../docs/08_phase_02_motion_control.md).

| Criterion | Evidence | Exit status |
| --- | --- | --- |
| Encoder scale and feed-forward characterization | Independent 10-turn encoder calibration and [low-band PWM characterization](2026-10-30-lowband-rpm-characterization.md) completed. | Passed for the documented baseline. Loaded rolling geometry still needs floor verification. |
| Settled wheel speed within ±5%, three repeats | [New-wheel runs](2026-11-04-new-wheels/) passed final-window mean at +25/+35/+45/−25 RPM. Left instantaneous tracking was outside the band in some runs, including variation under constant PWM. | Provisional mean-speed result. Full tracking open. |
| 1000 mm straight within ±50 mm, three repeats | Three supervised runs completed. Wheel-center measurements were about 990 mm, 950 mm, and 950 mm right / 920–930 mm left. | Open. The third run falls below tolerance at the midpoint between the wheels. Repeat the wheel-center measurements from the same reference line before adjusting calibration. |
| 90° turn within ±5°, three repeats | Short left/right turn screening passed with encoder estimates. | Open. Measure actual chassis angle independently of the encoder estimate and record IMU comparison where available. |
| Link loss stops motor output within 1200 ms | Prior watchdog/disconnect screening confirmed STOP. Exact output cutoff timing was not captured. | Functional screening passed. Timing acceptance open. |
| Software regression | 48 panel/tool tests passed on 2026-11-10. | Passed. |

## Acceptance Order

### 2026-11-10 raised-wheel execution

- Nine forward 20 s runs at +25/+35/+45 RPM met the final-window mean
  criterion and confirmed STOP. At +35 RPM the left wheel was within ±5%
  for 9–18% of settled samples, versus 91–100% on the right. Across the
  three +35 RPM runs, an encoder-angle fit yielded 4.92–5.20 RPM
  once-per-revolution amplitude (R² 0.953–0.964). This reproduces the
  earlier phase-linked variation and does not distinguish real speed
  variation from encoder measurement effects. Gains were unchanged.
- Two −25 RPM runs aborted on missing timely PI feedback and confirmed
  STOP. The single-command watchdog test returned TIMEOUT_STOP with both
  PWM outputs zero, and final STOP. The gap between received samples
  prevents a 1200 ms cutoff measurement.
- Raw evidence is under `2026-11-04-new-wheels/` in
  `pi_suite_20261110_225300`, `pi_suite_20261110_225801` and
  `single_shot_20261110_225855`. Floor acceptance was not started.
  With ESP disconnected, Windows enumerated ST-LINK as COM8, but the STM32
  did not return a motor-status frame to repeated STOP/status queries.
  No wired motion was started. Use the previously verified direct Pi UART
  as the wired transport for reverse and cutoff confirmation.
- ESP retest with the ToF stream paused completed all three −25 RPM runs.
  Both means passed ±5% and each STOP was confirmed. Settled in-band
  fractions were 19–27% left and 93–100% right. Watchdog samples bracketed
  the transition between 1188 ms (active) and 1269 ms (zero PWM), based on
  STM32 timestamps. The exact cutoff still needs a finer measurement.
  Evidence: [ESP retest](phase02_esp_20261110_231040/).
- First 1 m attempt stopped at zero travel on FRONT_INVALID. Both ToFs
  subsequently reported RAW 255 and failed I2C address assignment. A cold
  restart restored optical status 8/6, and the existing guarded 60 mm
  command returned MOVE DONE (10496/10018 counts) and STOP.
  Continuous cruise now accepts fresh optical no-return with the existing
  side-clearance guard. OTA build CRC 15E2891B, installation and continuous
  floor validation pending.
- On 2026-11-11, diagnostic OTA CRC 3DE6B247 confirmed a transient left
  RAW 255 alongside a fresh right no-return before cruise start. OTA CRC
  331B3DDE tolerates a recent no-return on one channel only while the other
  still reports a fresh no-return. The first continuous floor run reached
  the commanded cutoff and confirmed STOP, with encoder distances
  983.75/990.14 mm. The operator measured approximately 990 mm externally,
  within the 950–1050 mm acceptance band.
  The second run returned 989.33/990.47 mm by encoder; the operator measured
  approximately 950 mm, at the lower acceptance boundary. The third returned
  991.59/990.32 mm by encoder and confirmed STOP. The operator measured
  approximately 950 mm on the right and 920–930 mm on the left. The operator
  confirmed that all three runs were measured at the wheel centers. The
  estimated midpoint travel in the third run is 935–940 mm, below the
  950 mm acceptance limit.
  Runs two and three are under `phase02_floor_20261111_005442/` and
  `phase02_floor_20261111_010025/`. These are host-supervised cruise tests.
- Testing paused for the day after the third run, with STOP confirmed.
  Continue with external wheel-center measurements before changing calibration.
  Evidence: [floor log](phase02_cruise_20261111_001902/).

1. Keep current gains and wiring for the baseline. With wheels secured off the ground,
   repeat +25 RPM first. Record time series, last 4 s mean and in-band fraction for
   each wheel, and STOP. Use three 20 s runs for each target. Continue with +35,
   +45 and −25 RPM if the baseline and STOP permit. If variation is repeatable,
   compare with the existing constant-PWM baseline before changing gains.
2. Verify motor-output cutoff after loss of command refresh on the installed
   firmware. Measure from the last accepted command to zero output. Report any
   measurement uncertainty separately from wheel coast time.
3. With a marked, clear floor lane, run three continuous 1000 mm trials and three
   trials in each turn direction. Record external distance and angle, encoder
   estimate, end state and any abort. Use the external measurements for acceptance.

The three 2026-11-10 exploration runs produced map exports but ended on corner
clearance guards. Mapping reliability and LiDAR remain follow-on work. Direct Pi
UART was demonstrated, and sustained Pi integration is deferred until motion
acceptance is complete.
