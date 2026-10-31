# ARGOS Field Integration Roadmap

**Started:** 2026-10-07
**Phase:** First mobile chassis integration

## Work Queue

1. **ESP32-C3 link reliability and diagnostics - VALIDATED (2026-10-08)**
   - Reduced Wi-Fi transmit power for indoor use.
   - Modem sleep enabled.
   - STA reconnect and fallback AP recovery.
   - Application heartbeat between control panel and bridge.
   - Persistent ESP event queue in NVS.
   - Desktop JSONL connection logbook with acknowledgement.
   - In-panel logbook viewer for the latest 500 connection events.
   - 2026-10-08 preflight: no reset observed; hotspot was not found initially,
     then STA, TCP and STM UART recovered successfully. Per-reason log throttling
     was corrected and the revised ESP firmware compiled successfully.
   - 2026-10-08 revised-firmware check: joined the hotspot in 1.9 s at -55 dBm;
     TCP and STM UART were online by 4.84 s. The following 56 s were clean, but
     the hotspot interruption and sustained-duration portions were not exercised.
   - 2026-10-08 acceptance run: remained online for about 21 min 49 s at
     -40 dBm, survived an intentional hotspot outage without resetting, started
     the fallback AP, and automatically restored Wi-Fi, TCP and STM UART in
     about 20.8 s. Result: PASS. See
     `tests/field-integration/2026-10-08-field-link-regression-report.md`.

2. **Operational OLED view - Implemented, hardware regression pending**
   - Network mode and IP address.
   - Control-panel link state.
   - Last accepted command.
   - Requested motor percentages and safety state.
   - UART receive count and overflow indication.

3. **Field-oriented desktop control - Implemented, hardware regression pending**
   - Direct arrow-key driving.
   - Simultaneous forward/reverse and steering input.
   - Differential inner-wheel reduction for moving turns.
   - Stop on key release and existing STM32 command timeout as final safety layer.
   - Immediate stop when the control window loses keyboard focus.
   - Discovery support for both Windows hotspot and fallback-AP subnets.

4. **Wireless firmware update - Software implemented; bootstrap and hardware acceptance pending**
   - Keep the current ESP-to-STM USART2 wiring.
   - Reserve STM32 sector 0 for a 16 KB persistent recovery bootloader and
     sector 1 for image metadata.
   - Cache and CRC-check the complete image on ESP before STM application erase.
   - Interrupted-update recovery, application validity checks, panel upload,
     ESP LittleFS cache and framed UART transfer are implemented.
   - STM bootloader, relocated application, desktop panel, and ESP bridge builds
     passed software verification on 2026-10-30.
   - One final ST-LINK installation of `ARGOS_BOOTSTRAP.bin` is required before
     routine Wi-Fi updates can be tested.
   - Design, installation, and acceptance criteria are recorded in
     `docs/07_wireless_stm32_update_design.md`.

5. **Mobile platform integration - COMPLETE (2026-10-31)**
   - Follow `docs/06_field_link_regression.md`.
   - Connection endurance and intentional-outage recovery: PASS (2026-10-08).
   - No brownout, panic, watchdog, CPU-lockup, power-glitch, or silent TCP-stall
     event was recorded in the accepted boot session.
   - First field link regression report archived under `tests/field-integration/`.
   - Chassis assembly, basic computer-controlled motion and safe stop are operational.
   - Precision closed-loop floor acceptance continues under Phase 02.

6. **Phase 02 motion control - Started (2026-10-29)**
   - Both wheel circumferences measured as 260.0 mm.
   - Track width measured as 198.0 mm between wheel centre planes.
   - Encoder balancing firmware produced a successful qualitative straight-drive
     floor test.
   - Guided manual counts-per-wheel-revolution calibration completed: left
     48097.2 and right 48036.6 counts per wheel revolution.
   - Calibrated PWM-to-wheel-RPM and focused 50-60% characterization passed.
   - Feed-forward plus independent PI loops are implemented.
   - Single forward raised-wheel runs at 25/35/45 RPM met the initial mean target.
   - Repeated, reverse, disconnect-stop latency and floor tuning remain open.
   - Detailed sequence and acceptance targets are in
     `docs/08_phase_02_motion_control.md`.

## Release Rule

Published status must distinguish completed basic safe motion from open Phase 02
PI repeatability, reverse, disconnect-stop latency and floor acceptance work.
