# Power Module V1 Validation Checklist

**Project:** ARGOS
**Phase:** Phase 00 - Bench Validation
**Module:** Power Module V1
**Report Date:** 2026-09-06
**Status:** Bench Validation Completed with Open Measurement Items

## Purpose

This document defines the validation checklist for the ARGOS Power Module V1.

The objective is to verify that the perfboard-based power distribution module is suitable for controlled bench validation of the STM32 controller, motor drivers, sensors, and other Phase 00 subsystems.

## Related Documentation

- `journal/19-07-2026_Phase00_Power_Module_V1.md`
- `images/diagrams/Phase00_System_Block_Diagram.png`
- `images/diagrams/Phase00_Wiring_Diagram.png`
- `images/hardware/19-07-2026-power_module_front.jpeg`
- `images/hardware/19-07-2026-power_module_back.jpeg`

## Power Architecture Under Test

The tested power architecture is:

```text
3S Li-ion Battery Pack
        |
        v
3S BMS
        |
        v
Main Power Switch
        |
        v
5A Blade Fuse
        |
        +----> 12V Rail ----> Motor Drivers
        |
        +----> LM2596 Buck Converter ----> 5V Rail ----> STM32 / Sensors / Low-voltage peripherals
```

## Result Definitions

Use the following result values during validation:

- `Passed` / `Gecti` - Test meets the acceptance criteria.
- `Partially Passed` / `Kismen Gecti` - Test works, but a limitation or concern was observed.
- `Failed` / `Kaldi` - Test does not meet the acceptance criteria.
- `Not Tested` / `Test Edilmedi` - Test has not been executed yet.

## Test Scope Note

The power module was validated with the available bench equipment. Electrical continuity, polarity, switching, fuse path, 12V rail behavior, 5V LM2596 output, STM32 power-up, motor-driver supply, and combined basic bench-load behavior passed.

Oscilloscope-based checks and detailed thermal measurement were not performed during this validation pass.

## Required Test Equipment

- Digital multimeter
- Known-good 3S Li-ion battery pack
- Appropriate dummy load or power resistors
- Fuse continuity tester or multimeter continuity mode
- USB-powered STM32 NUCLEO board for low-current validation
- Optional: oscilloscope for ripple observation
- Optional: thermal camera or contact thermometer

## Validation Checklist

| ID | Test | Method | Acceptance Criteria | Result | Notes |
| --- | --- | --- | --- | --- | --- |
| PMV1-001 | Visual inspection | Inspect solder joints, wire routing, insulation, connector polarity, fuse holder, switch, battery holder, and LM2596 module. | No loose wires, short risks, reversed polarity, damaged parts, or mechanically weak connections. | Passed |  |
| PMV1-002 | Continuity check - ground rail | Verify continuity between all intended GND points. | All intended GND points are connected with low resistance. | Passed |  |
| PMV1-003 | Short-circuit check - power to ground | Measure resistance between battery positive rail and GND before inserting battery. | No direct short circuit is present. | Passed |  |
| PMV1-004 | BMS output polarity | Connect battery pack and verify BMS output polarity. | Output polarity matches module input polarity. | Passed |  |
| PMV1-005 | Main switch operation | Measure voltage before and after the main switch in OFF and ON positions. | OFF isolates downstream rail; ON connects downstream rail. | Passed |  |
| PMV1-006 | Fuse path verification | Check continuity through the 5A blade fuse and verify downstream rail voltage after fuse. | Fuse path is continuous and downstream rail is powered only through the fuse. | Passed |  |
| PMV1-007 | 12V rail no-load voltage | Measure fused 12V rail with no external load. | Rail voltage is consistent with 3S battery voltage and has correct polarity. | Passed |  |
| PMV1-008 | LM2596 adjustment | Measure LM2596 output and adjust to 5.0V before connecting STM32 or sensors. | Output is adjusted to 5.0V within acceptable tolerance. | Passed |  |
| PMV1-009 | 5V rail no-load voltage | Measure 5V output after LM2596 adjustment. | 5V rail remains stable with no load. | Passed |  |
| PMV1-010 | 5V rail low-current load test | Apply a small load representative of STM32 and sensor current. | 5V rail remains stable and LM2596 does not overheat. | Passed |  |
| PMV1-011 | STM32 power validation | Power STM32 NUCLEO from the 5V rail under controlled conditions. | STM32 powers up reliably without reset or abnormal heating. | Passed |  |
| PMV1-012 | 12V motor driver supply validation | Connect motor driver supply input without motor load and verify voltage. | Motor driver receives correct 12V rail polarity and voltage. | Passed |  |
| PMV1-013 | 12V rail load test | Apply a controlled load to the 12V rail. | Rail remains stable, wiring does not heat abnormally, and fuse remains intact. | Passed |  |
| PMV1-014 | Combined 12V and 5V load test | Load both 12V and 5V rails at expected bench validation levels. | Both rails remain stable without abnormal voltage drop, reset, or overheating. | Passed |  |
| PMV1-015 | Switch-off behavior | Turn off main switch while module is loaded. | Downstream rails turn off safely and no unexpected behavior is observed. | Passed |  |
| PMV1-016 | Fuse protection behavior | Verify that the fuse is correctly placed in the main downstream power path. | The protected rail is interrupted when the fuse is removed. | Passed |  |
| PMV1-017 | Thermal observation | Run the module under expected bench load for a defined test period. | LM2596, wires, switch, fuse holder, and connectors remain within safe temperature limits. | Not Tested | no thermal camera |
| PMV1-018 | Documentation consistency | Compare actual module wiring with the journal entry, block diagram, and wiring diagram. | Documentation matches the implemented power architecture or differences are recorded. | Passed |  |
| PMV1-019 | Output ripple observation | Observe 5V rail and 12V rail behavior with an oscilloscope under load. | No excessive ripple, oscillation, or switching artifact is observed for the intended bench-validation use. | Not Tested | Oscilloscope not available during this validation pass. |

## Critical Tests

The following tests must pass before the module is considered suitable for use as the primary Phase 00 bench power source:

- PMV1-001 - Visual inspection
- PMV1-003 - Short-circuit check
- PMV1-004 - BMS output polarity
- PMV1-005 - Main switch operation
- PMV1-006 - Fuse path verification
- PMV1-008 - LM2596 adjustment
- PMV1-011 - STM32 power validation
- PMV1-014 - Combined 12V and 5V load test

## Validation Decision

Current decision:

```text
Approved for controlled Phase 00 bench validation use with open thermal and oscilloscope-based checks.
```

The Power Module V1 is considered suitable for controlled ARGOS Phase 00 bench use based on the completed electrical validation checks.

Thermal measurement and oscilloscope-based ripple observation remain open verification items. These checks should be completed before the module is treated as a mature or final power subsystem.

## Issues Found

- No physical validation issues have been recorded during the completed checks.
- Detailed thermal behavior has not been measured.
- Output ripple has not been measured with an oscilloscope.

## Next Actions

- Perform thermal observation during a longer motor-driver bench session.
- Measure 5V rail ripple with an oscilloscope when available.
- Measure 12V rail behavior during motor startup and low-PWM operation.
- Add battery level monitoring circuit to the power module.
- Update this report if the power module wiring or load profile changes.
