# Wireless STM32 Update - Implementation Record

**Date:** 2026-10-30
**Status:** Software verification PASS; one-time installation and hardware acceptance pending

## Implemented Path

`Desktop panel -> Wi-Fi HTTP 3335 -> ESP32-C3 LittleFS -> USART2 115200 -> STM32 bootloader`

- The desktop panel stops the robot, calculates image CRC32, and uploads the
  relocated application with explicit size, CRC, build ID, and stopped-state
  headers.
- ESP verifies and caches the complete image before requesting STM32 reset.
- The bootloader keeps all motor PWM pins low, receives 256-byte chunks, checks
  each chunk, writes only the application sectors, and validates the final
  image before booting it.
- Interrupted or invalid updates leave the STM32 in a recoverable bootloader
  state instead of starting an incomplete application.

## Flash Layout

| Region | Address | Size |
| --- | --- | ---: |
| Bootloader | `0x08000000` | 16 KB reserved |
| Metadata | `0x08004000` | 16 KB reserved |
| ARGOS application | `0x08008000` | 480 KB maximum |

## Build Evidence

- Bootloader text: `2740 bytes`; BSS: `260 bytes`
- Relocated application binary: `41424 bytes`
- Application CRC32: `12954AFE`
- Initial stack pointer: `0x20020000`
- Reset vector: `0x0800D165`
- Combined first-install image: `74192 bytes`
- STM normal and relocated builds: **PASS**
- Python panel and packager syntax checks: **PASS**
- ESP compiled map contains upload handler, cache programmer, and completion
  handler: **PASS**

## Installation Boundary

The implementation cannot make its own first installation. Flash
`Debug/ARGOS_BOOTSTRAP.bin` once at `0x08000000` using ST-LINK and upload the
updated ESP sketch once by USB. After this bootstrap, normal STM application
updates use `Debug/ARGOS_OTA.bin` from the panel without ST-LINK.

## Remaining Acceptance

1. Verify a normal wireless update and complete sensor/motor safety regression.
2. Interrupt Wi-Fi before erase and confirm the running app is preserved.
3. Interrupt power during programming and confirm retry recovery.
4. Reject a deliberately corrupted image.
5. Complete ten consecutive wireless updates.

Until these tests pass, the feature is implemented but not field accepted.
