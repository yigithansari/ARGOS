# Phase 02 Acceptance via ESP

Use the active ESP panel and the **PHASE 02 ACCEPTANCE** button.
The existing STM32 firmware supports these commands. Launch the updated
panel from this Tools directory. Keep a physical STOP/power switch accessible.

## Test sequence

| Setup | Panel selection | Repeats | Measurement and target |
| --- | --- | --- | --- |
| Robot secured, both wheels fully raised | PI +25 | 3 x 20 s | Final 4 s, each wheel within +/-5%. Mean and instantaneous in-band fraction recorded separately. |
| Same raised setup | PI +35 | 3 x 20 s | Same speed criterion. |
| Same raised setup | PI +45 | 3 x 20 s | Same speed criterion. |
| Same raised setup | PI -25 | 3 x 20 s | Same speed criterion in reverse. |
| Same raised setup | Watchdog | 3 | A single +25 RPM command followed by status-only queries. Motor outputs must stop within 1200 ms. |
| Flat floor, 1.5 m clear lane | Straight 1000 mm | 3 | Align wheel centers with the same start line each time. Enter actual left and right forward travel in mm. Midpoint must be 950–1050 mm. |
| Flat floor, clear swept area | Left 90 deg | 3 | Mark initial orientation. Enter externally measured chassis angle magnitude, 85–95 degrees. |
| Same floor setup | Right 90 deg | 3 | Same angle criterion. |

Each START runs once. Before the next run, save the external measurement when
requested and reposition the robot by hand. Select the next test and acknowledge
its setup using the checkbox. Move through speed cases only when STOP is confirmed
and wheel direction and motion are normal. STOP cancels the active test.

## Records and interpretation

The phase02_results folder contains run_<timestamp>.jsonl commands and raw
telemetry, written as the test executes. The adjacent .json includes the case,
geometry, STOP confirmation, encoder travel and evaluation. Physical
measurements are saved into the same result. Records survive panel restarts.

A mean-speed pass does not close the instantaneous tracking requirement. The
previous left-wheel ripple remains an open item if the in-band fraction fails.
For watchdog tests, a sampled interval spanning 1200 ms is recorded as
**MEASUREMENT REQUIRED**. Output-cutoff timing then needs a finer independent
measurement. Mechanical coast time and PWM cutoff are different measurements.

The 1 m run uses manual 25 RPM targets with host encoder balance trim
(up to 2 RPM per wheel), an encoder cutoff near 978 mm and explicit STOP.
Distance-sensor clearance does not block these operator-supervised acceptance
tests. Existing manual turn commands also run without range guards. Watchdog,
feedback, direction and divergence checks still apply. The operator must keep
the test lane and swept area clear and have access to power/STOP.

Reports support the Phase 02 exit review. Full acceptance requires all three
comparable repetitions per criterion, adequate feedback and external floor
measurements. The nominal 87 mm diameter remains subject to loaded rolling
validation. Reverse floor distance is outside the current mandatory exit matrix.
