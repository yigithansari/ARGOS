# Phase 02 Exit Report

**Decision date:** 2026-11-25

**Decision:** Complete with carry-over to Phase 03.

| Check | Result | Evidence |
| --- | --- | --- |
| 1 m straight drive | PASS, 3 of 3 | Midpoints 987.5, 972.5 and 982.5 mm against 950-1050 mm. |
| 90-degree turns | PASS, 6 of 6 | Left: 88, 88, 88 degrees. Right: 87, 89, 89 degrees. Target band: 85-95 degrees. |
| Raised-wheel mean speed and STOP | PASS, 15 valid trials | Signed targets +25, +35, +45 and -25 RPM. One reverse trial was invalidated after reported cable contact. |
| Instantaneous speed tracking | Carry-over | At least one wheel exceeded the settled +/-5% band in every speed case. Variation was also seen at fixed PWM. |
| Straight-path lateral offset | Carry-over | The operator observed a small sideways offset with heading preserved. Offset magnitude and root cause remain unmeasured. |
| Link-loss stop | Functional PASS | The operator observed motor rotation cease immediately after disconnect. Three recorded watchdog runs reached zero PWM and `TIMEOUT_STOP`. Their sample intervals straddled 1200 ms, leaving precise cutoff timing for revalidation. |

After the new chassis, motors and PCB are installed, recalibrate the wheels and repeat speed tracking, lateral offset and watchdog timing tests. Recheck distance, turns and STOP on the revised platform.

Recorded measurements and raw traces are in the [November 24 acceptance folder](../field-integration/2026-11-24-phase-02-acceptance/README.md). The closeout decision and carry-over list are in the [Phase 02 record](../../phases/phase-02-motion-control.md).
