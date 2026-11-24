# Temporary cross-driver test - 2026-11-04

Operator configuration: physical left motor on right driver; left driver output
and physical right motor disconnected. Encoders stayed in original positions.
Robot raised/secured. No firmware mapping, encoder sign or gain change.
Never run normal PI commands with this temporary actuator/feedback mismatch.

Open-loop stages P 0 +50 (1.5 s), P 0 +52 (10 s) confirmed only physical left
encoder motion, but left normalized speed was reverse. STOP passed. That run is
not a matched-direction comparison with the original forward baseline.

P 0 -50 (1.5 s), P 0 -52 (10 s) then matched physical left-forward. Physical right
encoder delta was zero throughout; zero PWM/STOP confirmed after each stage/end.
Only this -52 stage is filtered into stage_52.log for analysis; raw full log kept.

| Same physical left motor, forward | Original left driver | Right driver |
| --- | --- | --- |
| PWM magnitude | 52% | 52% |
| Nearest-one-second mean RPM | 36.31 | 33.03 |
| Window CV | 5.06% | 7.10% |
| Wheel-harmonic amplitude RPM | 6.49 | 6.13 |
| Exploratory harmonic R-squared | 0.979 | 0.986 |

Periodic effect persists across driver change. This does not support the original
left driver as sole cause. Different driver/bridge leg, supply state and separate
short runs limit comparison; encoder-derived phase/speed fit is not independent
external speed measurement. Motor/gear/wheel/hub/encoder remain unresolved.

Temporary wiring remains installed per latest operator report. Last verified state
STOP. Restore normal motor wiring before ordinary PI or drive tests. Next isolation
is a powered-off compatible wheel swap while retaining normal motor/encoder wiring.
