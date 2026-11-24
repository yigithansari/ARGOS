# MCU-local bounded straight move prototype - 2026-11-04

Implemented in active Core/Src/argos_platform.c and Core/Inc/argos_straight_guard.h.
Installed OTA image44364 bytes, CRC06082E16. Experimental; no valid movement yet.

## Protocol and limits

- G left_counts right_counts starts only with motor commands/targets/outputs zero.
  Positive counts5000..one calibrated output-wheel revolution; normalized goals
  must agree within2%. Host tool restricts initial distance30..150 mm and derives
  counts from the central geometry JSON. 100 mm ->17597/17575 counts.
- Fixed forward15 RPM; G KEEP refreshes the existing1200 ms command watchdog,
  without resetting origins/start time. Overall job bound5 s. x/X cancels.
- At >4% progress lead the faster wheel output is held at zero; release below
  1.5%. Held-side integral reset prevents windup. Stop when both reach95% target;
  this coast allowance still needs physical validation. No hard braking added.
- Abort on >25% progress mismatch, >115% individual travel, >3% reverse motion,
  >1.2 s unheld progress stall or wrong/overspeed feedback. Limits use encoder
  fractions and existing calibrated signs, not duplicated diameter constants.
- Sensor streams temporarily paused/restored around a job to bound blocking;
  active job rejects unrelated commands. E/network notifications and OTA stop
  path retained. UART lines over39 characters are rejected, not truncated/acted on.

## Verification actually performed

- STM bootloader/normal/OTA/package builds PASS; OTA/CRC/restart/STOP PASS.
- Eleven guard numeric cases and unequal-speed synchronization simulation PASS.
  Tests execute the C header function's unchanged numeric body after primitive
  type/signature translation to C# on host. This is not native HAL/MCU testing.
- Four host unit tests and five geometry tests PASS; host syntax PASS.
- Live reject-only probe: six invalid/malformed/overflow/unbalanced/overlong G
  commands rejected, STOP after each, encoder displacement zero. G KEEP when
  inactive reported MOVE IDLE. No valid G job or motor movement was requested.

Remaining hardware checks: first operator-ready short job, straightness/distance,
local cancel, watchdog/host disconnect, stall/hold behavior and regressions. Robot
still stopped at previous floor endpoint; reset/front-clear readiness requested.
Host tool defaults to reject-only; never invoke --execute without a prepared area.
Existing PI/PWM gains, pin mapping, encoder signs, flash layout and watchdog unchanged.

## First valid job - relative floor start

Operator explicitly confirmed front clear and authorized starting from current
position without resetting to marker. --execute requested100 mm (17597/17575
counts). MCU returned MOVE DONE COUNTS16718/16903. After stop/coast, encoder
travel L96.235/R97.017 mm, difference0.781 mm, mean96.626 mm. Final STOP/zero PWM
confirmed. Physical direction/distance observation pending.

This first successful completion does not establish repeated positioning accuracy,
external rolling scale, exact electrical stop timing or abort/stall behavior.
Robot remains at the new endpoint; no further motion scheduled until operator
observation and available space are checked. Earlier no-motion status above is
the chronology before this first authorized execution.
