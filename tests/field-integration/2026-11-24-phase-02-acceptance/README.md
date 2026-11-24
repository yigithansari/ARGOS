# Phase 02 Acceptance Results

**Date:** 2026-11-24
**Decision at test time:** Floor distance and angle criteria passed. The subsequent [Phase 02 closeout](../../../phases/phase-02-motion-control.md) records the operator's link-loss stop acceptance and hardware revalidation plan.

## Operator-run floor results

| Case | Trial | External measurement | Result |
| --- | --- | --- | --- |
| Straight 1000 mm | 1 | L 990 / R 985 mm; midpoint 987.5 mm | PASS |
| Straight 1000 mm | 2 | L 970 / R 975 mm; midpoint 972.5 mm | PASS |
| Straight 1000 mm | 3 | L 990 / R 975 mm; midpoint 982.5 mm | PASS |
| Left 90 deg | 1 | 88 degrees | PASS |
| Left 90 deg | 2 | 88 degrees | PASS |
| Left 90 deg | 3 | 88 degrees | PASS |
| Right 90 deg | 1 | 87 degrees | PASS |
| Right 90 deg | 2 | 89 degrees | PASS |
| Right 90 deg | 3 | 89 degrees | PASS |

## Raised-wheel speed results

| Target | Valid trials | Mean criterion | Settled in-band fraction L / R |
| --- | --- | --- | --- |
| PI +25 | 3 | PASS | 17.6–44.4% / 41.2–50.0% |
| PI +35 | 3 | PASS | 16.7–27.8% / 77.8–88.9% |
| PI +45 | 4 | PASS | 16.7–27.8% / 100.0–100.0% |
| PI -25 | 5 | PASS | 22.2–38.9% / 76.5–94.4% |

All valid speed runs passed the mean criterion and confirmed STOP. All
speed cases retain at least one wheel outside the instantaneous +/-5% band.
The first reverse run is marked invalid because the operator reported cable
contact with the wheel. Extra repetitions are retained.

## Watchdog

| Trial | Last active | First zero-PWM timeout | Decision |
| --- | --- | --- | --- |
| 1 | 1177 ms | 1412 ms | Measurement required |
| 2 | 958 ms | 1246 ms | Measurement required |
| 3 | 1026 ms | 1249 ms | Measurement required |

All three runs reached TIMEOUT_STOP and confirmed final STOP. Each sampled
interval spans the 1200 ms cutoff. A finer output measurement remains open.

## Scope and outstanding work

- Floor tests used operator-confirmed clear space with range clearance disabled.
  The 1 m job used 25 RPM with host encoder balancing and cutoff near 978 mm.
- Distances are operator-entered wheel-center measurements. Angles are
  operator-entered chassis measurements. Instrument uncertainty was not recorded.
- Earlier rejected floor trials remain in the active workspace's invalid archive
  and are excluded from this acceptance set.
- Finish settled instantaneous speed tracking and exact watchdog cutoff acceptance.
- Encoder odometry, mapping and nominal sensor mount estimates are prototypes.
  Room-map accuracy and reliable autonomous coverage remain unaccepted.
- Raspberry Pi UART, desktop launch through Pi Connect and live sensor telemetry
  were operator demonstrated. Remote keyboard stutter and intermittent I2C loss
  remain unresolved. The old system report can incorrectly mark YOK as ToF PASS.
- No new motor test or firmware upload was performed during publication.

Raw commands and received frames are in the adjacent JSONL files. Individual
JSON files retain calibration, STOP status and external measurements.

## Publication verification

The active software suite passed 57 tests. The published software suite ran
57 tests with 50 passed and 7 OTA tests skipped because locally built firmware
images are excluded. Eighteen STM32 C/header/startup/bootloader source files
matched the active tree byte for byte. Selected documentation links and Pi
Python syntax checks passed. No new firmware build was required for this
source publication.
