# Repeated raised-wheel motion checks - 2026-10-31

Operator authorization: robot raised, secured, powered and connected.

## Baseline (retained firmware CRC 6C6BC21F)

Nine 10-second forward runs completed (three each at 25/35/45 RPM). No feedback
abort; STOP/zero PWM confirmed after every run. All run means within +/-5%.
At 45 RPM, all recorded settled samples in all three runs were within +/-5%.
At 25/35 RPM, instantaneous tracking was outside band in many samples: not full
PI acceptance. Results measure filtered telemetry in the last four seconds;
they are not external tachometer measurements or floor-load validation.

## Trial and rollback

The controller rounded its fixed-point PI result to integer percent before PWM.
A trial preserved fractional output down to timer resolution (CRC D180E343).
Build, OTA/CRC, forward single runs and short reverse/timeout worked, but the
trial did not establish an improvement; 45 RPM left tracking worsened.
Rolled source and firmware back to baseline; verified CRC 6C6BC21F and source
hash equality with the publication source. Restored file timestamps initially
left a stale object; forced recompilation before flashing. No trial code remains
in active/publication firmware. Trial source retained only as diagnostic evidence.

## Final baseline safety screening

Short reverse command without refresh: negative normalized wheel readings,
TIMEOUT_STOP and final STOP observed. Thirty simultaneous HTTP probes succeeded
(maximum 234 ms). This is direction/timeout screening, not sustained reverse
speed acceptance or an exact electrical measurement of the 1200 ms stop deadline.

Control socket deliberately closed after a short single command: reconnect status
confirmed STOP and zero PWM. Motor coast-down distance/time not measured.

Remaining: low-speed tracking, sustained reverse/differential operation and floor
acceptance. No calibration, gains, mappings, timeout or safety limits changed.
