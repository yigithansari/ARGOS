# Phase 03 Baseline Verification Report

**Milestone date:** 2026-11-25

This report summarizes the exploration logs and operator demonstrations available at Phase 03 entry.

| Check | Recorded result |
| --- | --- |
| Guarded exploration and cornering | The three reviewed traces ended at clearance guards: two `CURVE_TURN_CLEARANCE` and one `CURVE_SIDE_CLEARANCE`. Repeatable corner completion remains open. |
| Mapping export | JSONL traces and SVG point maps were produced using encoder pose, two front ToF sensors and a left HC-SR04. Coverage and map accuracy remain unverified. |
| Pi 5 interface | The operator demonstrated STM32 UART at 115200 baud, panel launch through Pi Connect and live telemetry. Sustained telemetry and remote keyboard timing remain open. |
| ToF recovery and reporting | Both ToF devices recovered after a power cycle following shared I2C loss. Root cause remains open. The old report parser can misclassify `YOK` as PASS. |
| Battery percentage | Pending voltage sensing, calibration and display verification. |

For the next run, record how many times the robot completes the same corner, compare mapped distances with a measured layout, and capture local and remote command timing when drive stutter occurs. Add a missing-ToF regression case for report parsing.

See the [scan implementation and replay notes](../field-integration/2026-11-05-scan-experiments/README.md), [example map](../../images/software/phase-03/2026-11-10_Exploration_Map.svg) and [Phase 03 work queue](../../phases/phase-03-localization-and-basic-autonomy.md).
