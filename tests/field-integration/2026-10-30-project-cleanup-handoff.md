# ARGOS Controlled Cleanup and Handoff

**Date:** 2026-10-30
**Result:** Software verification PASS; no hardware flashing performed

## Audit Result

- HAL UART/GPIO callbacks and IRQ handlers were preserved as indirect entry
  points.
- No demonstrably unused STM32 or ESP function, macro, or static state was
  found in the custom firmware sources.
- Two unused desktop-panel helper methods and their orphaned state variable were
  identified and removed.
- Large STM32 platform and desktop UI modules were deliberately preserved; a
  structural split would require dedicated regression coverage.
- Generated builds, caches, local credentials, runtime logs, and replaceable
  reports were classified for ignore rules rather than deleted.
- Git metadata was not present in the workspace, so no reset, commit, or history
  operation was attempted.

## Safe Changes

- Moved OTA vector relocation into a CubeMX `USER CODE` region without changing
  execution order.
- Moved custom ToF/motor startup output levels into a CubeMX `USER CODE` region.
- Removed `_wide_command`, `send_custom_command`, and `custom_command_var` from
  the panel because they had no construction or call path.
- Added token-efficient `AGENTS.md`, `PROJECT_STATUS.md`, and `PROJECT_MAP.md` at
  the shared workspace root.
- Added root hygiene rules and expanded the publication-tree `.gitignore`.
- Documented the one-time wired OTA bootstrap boundary and later Wi-Fi update
  procedure.

## Build Verification

- Bootloader: text `2740`, BSS `260` bytes.
- Normal STM32 image: text `41264`, data `148`, BSS `2988` bytes.
- Relocated OTA image: text `41264`, data `148`, BSS `2988` bytes.
- OTA binary: `41424 bytes`, CRC32 `12954AFE`.
- Combined bootstrap image: `74192 bytes`.
- STM32 compiler/linker warnings observed: none.
- Python syntax and bootstrap metadata/CRC checks: PASS.
- Existing compiled ESP map contains all STM update handlers: PASS.

## Remaining Hardware Work

1. Upload the current ESP sketch once over USB.
2. Install `ARGOS_BOOTSTRAP.bin` once at STM32 `0x08000000` through ST-LINK.
3. Run sensor, motor STOP, and drive regression.
4. Perform a panel-driven `ARGOS_OTA.bin` update and interruption recovery tests.
5. Continue raised-wheel and floor acceptance for Phase 02 PI speed control.

