# Provisional operation decision - 2026-11-04

Operator requested one more check, then acceptance of the persistent variation
as a known limitation to continue development. No cause or physical repair claimed.

The attempted cross-driver repeat detected right encoder movement instead of
left and aborted at low PWM; STOP confirmed. Subsequent independent pulses
verified normal channel mapping restored (left -13450/right 0; left 0/right +18076).
Every pulse ended with STOP. No PI used before mapping verification.

Final normal-wiring left P52/0 / 10 s repeat: nearest-one-second mean 36.857 RPM,
range 33.468-40.999, CV 6.076%; STOP confirmed. Comparable periodic variation
persists, so retain existing firmware 5E1584A0/gains rather than claim a repair.

Repeated test default becomes 20 s, evaluating last 4 s for mean +/-5%. Report
provisional_mean_pass separately from instantaneous tracking; shorter runs remain
screening only. Previous completed 20 s runs support this settling choice; no new
PI run was made after the operator decision. Full floor acceptance remains open.

Feedback age, stall/direction/overspeed checks and STOP/firmware command timeout
were not loosened. Normal wiring is restored; temporary cross-wiring warnings
in earlier reports are historical. This decision can be revisited after hardware
changes or new observations. Last verified robot state STOP.
