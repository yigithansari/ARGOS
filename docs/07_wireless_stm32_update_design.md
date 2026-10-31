# ARGOS Wireless STM32 Update Design

**Decision date:** 2026-10-29
**Implementation date:** 2026-10-30
**Status:** Wired installation and targeted OTA/interruption tests passed; full field regression pending (2026-10-31)
**Target:** STM32F446RE application update through the existing ESP32-C3 Wi-Fi bridge

## Decision

Use a small persistent custom STM32 bootloader over the existing USART2 link.
The ESP32-C3 will receive and cache the application image over Wi-Fi, then send
verified chunks to the STM32 bootloader through PA2/PA3.

This route preserves the current wiring and requires one final wired ST-LINK
installation. After that installation, normal STM32 application revisions can
be delivered from the ARGOS control panel.

## Why the Factory ROM Bootloader Is Not the Default

The STM32F446 factory system-memory bootloader supports USART1 on PA9/PA10 and
USART3 on PB10/PB11 or PC10/PC11. It does not list USART2 on PA2/PA3, which is
the link currently used between STM32 and ESP32-C3. See ST AN2606, section 42,
Table 93:

- [AN2606: STM32 system-memory boot mode](https://www.st.com/resource/en/application_note/an2606-stm32microcontroller-system-memory-boot-mode-stmicroelectronics.pdf)
- [AN3155: USART protocol used in the STM32 bootloader](https://www.st.com/resource/en/application_note/an3155-usart-protocol-used-in-the-stm32-bootloader-stmicroelectronics.pdf)

Using the ROM bootloader would require moving the bridge UART to USART3 and
adding ESP-controlled BOOT0 and NRST signals. It remains a valid fallback, but
it adds four hardware changes and conflicts with the goal of reducing service
wiring.

## Proposed Flash Layout

The STM32F446RE has 512 KB of flash. Its first four sectors are 16 KB each.
Reserve sector 0 for the immutable bootloader, sector 1 for update metadata,
and start the application at sector 2:

| Region | Address | Size | Purpose |
| --- | --- | ---: | --- |
| Bootloader | `0x08000000-0x08003FFF` | 16 KB | Immutable updater and recovery path |
| Metadata | `0x08004000-0x08007FFF` | 16 KB | Image state, size, build ID and CRC32 |
| Application | `0x08008000-0x0807FFFF` | 480 KB | ARGOS firmware image |

The current application map occupies only about 36 KB of flash, so the new
layout has ample margin. Flash organization is defined in the
[STM32F446 reference manual RM0390](https://www.st.com/resource/en/reference_manual/rm0390-stm32f446xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf).

## Update Flow

1. The control panel builds or selects an application-only `.bin` file.
2. The panel uploads the file to the ESP32-C3 with image length, version, and
   CRC32.
3. ESP stores the complete file before requesting an STM32 update.
4. ESP asks the running STM32 application to enter bootloader mode.
5. STM32 stops motors, records a bootloader-request marker, and resets.
6. The bootloader validates the request and announces readiness over USART2.
7. ESP sends numbered chunks with per-chunk CRC and acknowledgement.
8. The bootloader erases only application sectors, writes the image, and reads
   it back for verification.
9. The bootloader validates full-image length, stack pointer, reset vector, and
   CRC32 before marking the image valid.
10. The bootloader jumps to the application. ESP and the panel verify the new
    firmware version and archive the update result.

## Recovery Rules

- Motors remain disabled throughout bootloader mode.
- The bootloader sectors are never erased by a wireless update.
- A power or Wi-Fi loss leaves the bootloader available for retry.
- The running application is not erased until ESP has cached and CRC-checked
  the complete image.
- An incomplete or invalid application never boots.
- Update mode requires an explicit panel action and a robot-stopped state.
- Every attempt is recorded in the desktop connection logbook.
- ST-LINK remains the final recovery method during bootloader development.

## Implemented Software

- `Bootloader/`: a 3 KB custom STM32F446 bootloader using USART2 at 115200.
- `STM32F446RETX_OTA_APP.ld`: relocates the application to `0x08008000`.
- The application sets `SCB->VTOR` from its linked vector table and accepts the
  explicit `U ARGOS_OTA` reboot request only after stopping both motors.
- ESP caches `/stm32.bin` in LittleFS, verifies upload CRC32, transfers
  256-byte UART chunks with per-chunk acknowledgement, and verifies the final
  bootloader result.
- The desktop panel exposes `STM32 WIRELESS UPDATE` and requires a stopped-robot
  confirmation before uploading to ESP HTTP port `3335`.
- `Tools/build_wireless_bundle.ps1` builds the bootloader and both normal and
  relocated application images.
- `Tools/package_wireless_firmware.py` validates the application vector table,
  records CRC32 metadata, and creates the one-time bootstrap image.

Build verification on 2026-10-30:

- Bootloader: 2740 bytes text, 260 bytes BSS; **PASS** against 16 KB limit.
- Relocated application: 41424-byte binary; vector `0x0800D165`; **PASS**.
- Application CRC32: `12954AFE`; bootstrap size: 74192 bytes.
- Desktop Python syntax check: **PASS**.
- ESP image contains the upload, cache, and STM programming routines: **PASS**.

## One-Time Installation

1. Upload the updated `esp32_c3_argos_bridge.ino` to ESP32-C3 once by USB.
2. Flash `Debug/ARGOS_BOOTSTRAP.bin` to STM32 address `0x08000000` once by
   ST-LINK. This installs the bootloader, valid metadata, and relocated app.
3. Reboot the robot and verify normal sensors, STOP, and drive operation.
4. For later STM32 revisions, select `Debug/ARGOS_OTA.bin` from the panel's
   `STM32 WIRELESS UPDATE` button. ST-LINK is no longer needed for routine
   application updates.

Do not flash `ARGOS_OTA.bin` directly at `0x08000000`; it is linked for
`0x08008000`. The bootstrap image is only for the first wired installation or
full recovery.

## Acceptance Test

1. Install bootloader and relocated application once through ST-LINK.
2. Perform a normal wireless update and verify version, image CRC, sensors, and
   motor safety.
3. Interrupt Wi-Fi before STM erase and confirm the old application remains.
4. Interrupt power during STM programming and confirm the bootloader accepts a
   retry after restart.
5. Upload a corrupted image and confirm it is rejected before boot.
6. Complete ten consecutive wireless updates without ST-LINK intervention.

## Recorded Hardware Results (2026-10-30 / 2026-10-31)

Bootstrap installation/readback, normal OTA, cache CRC rejection, ten consecutive
updates, real hotspot loss before erase, and power loss during programming with
Wi-Fi-only retry all passed. Operator observed no motor movement during power
cycling; electrical motor-pin levels were not measured. Sensor readout and STOP
checks passed after recovery. Full drive regression and panel GUI interaction
remain pending; do not label the complete system field accepted.

Evidence: `tests/field-integration/2026-10-30-ota-acceptance/`,
`2026-10-30-interruptions/`, and `2026-10-31-motion-progress/`.
Earlier build figures above are historical. Latest installed STM image is
41444 bytes, CRC32 `6C6BC21F`; it includes early UART receive and bounded OLED
refresh. Older endurance/interruption results apply to the earlier firmware;
the latest image received targeted OTA, STOP, speed screening and timeout checks.

The panel now sends multipart/form-data; custom size/CRC headers describe the
binary payload, not the multipart envelope. Never select the bootstrap image for
routine OTA. Build binaries are local outputs and are not included in this snapshot.
