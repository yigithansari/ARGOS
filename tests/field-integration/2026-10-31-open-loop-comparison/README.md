# Constant PWM / PI comparison - 2026-10-31

Robot raised and secured per operator authorization. Firmware 5E1584A0; no
firmware or gain changes during this comparison. Commands refreshed with a
1 s feedback-age abort; every constant-PWM run ended with confirmed zero PWM/STOP.

Analysis uses nearest-one-second MCU timestamp/count windows (0.9-1.4 s allowed)
after 6 s of a 10 s run. Ten overlapping windows per constant-PWM run; descriptive
statistics, not independent repetitions or proof of physical shaft speed.

| Condition | Mean L/R RPM | CV L/R |
| --- | --- | --- |
| Previous PI 25 RPM | 25.68 / 25.20 | 3.43% / 5.98% |
| Fixed P 52 51 | 26.91 / 31.31 | 5.24% / 5.72% |
| Fixed P 52 0 | 27.31 / 0 | 5.34% / n/a |
| Fixed P 0 51 | 0 / 31.59 | n/a / 5.57% |

Fixed PWM is not expected to meet a 25 RPM target. PI baseline is an earlier
single run, not an interleaved experiment; supply/temperature drift is uncontrolled.
PI-loop timing fields are stale/non-applicable in PWM mode and are excluded.

Variation persists without PI and with only one motor energized. These data do
not support treating the controller as the sole cause or simultaneous motor
loading as the sole explanation. Supply ripple, motor/gear/wheel load and encoder
readout remain unresolved; voltage and external speed measurements are needed
before assigning a hardware cause or accepting further tuning.

No gain/encoder calibration, pin, timer or safety change. Latest state STOP.
Operator asked whether a multimeter or oscilloscope is available; no new motion
is scheduled until measurement coordination. Raw logs and analysis are archived.

## Operator voltage measurement and offline phase follow-up

Driver motor supply measured by DMM: approximately 11.3 V idle, minimum 11.24 V
during a separate P 52 51 / 10 s run. STOP confirmed. Approximate drop 60 mV;
no large slow sag detected. This does not exclude fast ripple, ground shifts or
local logic-rail problems. Baseline reading was approximate, not precision metrology.

Exploratory first-wheel-harmonic fits to individual-wheel interval speed versus
mid-interval encoder phase: 17 samples each, R-squared 0.886 left / 0.967 right,
amplitude 3.14 / 3.67 RPM. Synthetic known-coefficient check passed. Phase and
speed both derive from encoder counts; short-run correlation cannot establish
wheel imbalance, rubbing, gearing or encoder error as a physical cause.

Next: powered-off visual wobble/rubbing/uneven resistance check and independent
speed measurement. No gain change or additional motor run for this phase analysis.
