# First short floor screening - 2026-11-04

Operator confirmed ready in a 1-1.8 m clear area, robot oriented toward long axis,
initial marker requested. Firmware 5E1584A0; nominal 87 mm / 255 mm geometry.

Host probe: V25/25, target encoder mean 100 mm, maximum run 2.5 s. Feedback-age
limit 0.8 s, reverse/overtravel/speed/no-motion guards and >30 mm wheel mismatch
abort. Guard/distance synthetic checks and syntax passed before hardware run.
This is a bounded host diagnostic, not accepted autonomous distance control.

Aborted for wheel mismatch after 1.156 s. STOP/zero PWM confirmed. Latest pre-STOP
travel L43.61/R89.26 mm; after stop/coast snapshot L52.13/R107.63, mean79.88 mm.
Encoder heading estimate (right-left)/track = +12.47 degrees (leftward model).
Physical displacement/direction and rolling geometry not yet verified; operator
observation pending. No subsequent motion scheduled. No firmware or gain changes.

Do not infer floor acceptance or actual braking distance from these encoder-only
values. Robot remains on floor; raised-wheel speed suites must not be reused here.

## Operator observation and requested repeat

First observation: right wheel about 7-8 cm / left about 4 cm; robot skewed.
Operator reset robot to initial point and requested same short test again.

Second run: target_reached at 1.469 s, STOP confirmed. Last pre-STOP estimates
L124.47/R129.99 mm (mean127.23). Final L166.26/R174.07 mm (mean170.17), encoder
heading +1.76 degrees. Operator reported balanced travel and actual 16-17 cm.
This roughly corroborates nominal geometry over a short run; not precise wheel
calibration. No geometry/gain/firmware adjustment made.

Requested 100 mm was exceeded: target-trigger sampling/command latency and coast
must be addressed before claiming distance control. The second run is a positive
balance/readout screening result, not a passed 100 mm positioning test. Final state
STOP; operator has not confirmed reset to the marker for a further run.

## Third run: lower approach speed and anticipatory STOP

Operator reset to the line, front clear. Host script requested 25 RPM, lowering
to 15 RPM after estimated mean travel 30 mm. Stop projection uses measured filtered
RPM, nominal circumference, 0.30 s delay + 5 mm coast, bounded lead 8-45 mm.
All previous safety guards kept. Offline guard/projection checks passed.

Predicted stop triggered after 1.047 s: latest L64.87/R91.67 mm (mean78.27),
projected final111.50 mm. Actual final encoder L69.50/R101.34, mean85.42 mm,
heading estimate +7.15 degrees. STOP/zero PWM confirmed. Physical observation pending.
No firmware/gain change. Prediction did not accurately estimate stop travel;
neither 100 mm positioning nor heading is accepted. Timing/loaded startup variation
must be addressed; do not automatically repeat moves in the confined floor area.

Operator observed L about7-8/R10 cm, confirming the asymmetry. Host-only next
profile is prepared at constant 15 RPM from startup, retaining the existing
predictive STOP and guards. Offline checks passed; no hardware run yet. Wait
for reset/front-clear confirmation. No firmware or gains changed.

## Fourth run: constant 15 RPM

Operator confirmed reset/front clear. Constant15 RPM with the same predictive
STOP/guards executed; predicted_stop at1.047 s. Final encoder L60.75/R92.00 mm,
mean76.38, heading+7.02 degrees. STOP/zero PWM confirmed. No external physical
measurement yet. Speed reduction did not remove progress mismatch, so this is
not accepted 100 mm or straight-line positioning. Firmware/gains unchanged.

Do not continue identical floor trials. Next work should explicitly address
wheel-progress synchronization and distance-stop timing. Robot is stopped at
the fourth endpoint; wait for physical reset/clear confirmation before any next move.
