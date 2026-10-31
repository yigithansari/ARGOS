# UART startup follow-up - 2026-10-30

## STM32 change and hardware result

USART2 was enabled before platform initialization, but interrupt reception only
started after OLED initialization (including a 50 ms delay). This allows stale
data/overrun before command reception. RX now starts after Motor_Stop and before
OLED initialization; stale UART flags/data are cleared first. No pin mapping,
baud rate, command syntax, motor timeout, direction/scaling or flash layout changed.

Bootloader, normal/OTA apps and bootstrap package built successfully.
Installed application: 41452 bytes, CRC 4881C69F. Three OTA restarts tested with
early E queries instead of an eight-second startup delay: 36 intact motor status
lines, no WE echo or unknown command, restart and STOP verified in each cycle.
This bounds the observed result; commands sent while the MCU is resetting are
not guaranteed to be received. Prior source/image backed up under
Tools/installation_backups/uart_startup_before.

## Separate ESP issue

Five new TCP connections on the unchanged ESP firmware still showed incomplete
first motor telemetry. Subsequent E queries were clean in all five; no unknown
commands. This is distinct from the STM startup command concatenation.

Arduino HardwareSerial defaults to a 256-byte RX buffer. Connection handling
sends W/N/E, then synchronously persists and transmits event logs before draining
UART; first STM-online logging also persists while data arrives. Buffer pressure
is a plausible cause. ESP source now requests 4096 bytes before begin.
Compilation and USB upload/hash verification succeeded. Reconnect comparison after
buffer enlargement still failed initial telemetry in 5/5 sessions; later E queries
were clean, unknown commands zero. Buffer enlargement alone did not resolve it.

Follow-up source moves connection logging/history before W/N/E requests and defers
STM_ONLINE logging until a 100 ms UART-idle gap, avoiding synchronous NVS writes
inside the initial response burst. Build and USB upload/hash verification passed.
After reconnection, all five sessions had intact initial motor telemetry, clean
subsequent STOP replies, and zero unknown commands: targeted regression PASS.
Before/after result improved from 0/5 to 5/5 intact initial replies. This supports
the logging-timing diagnosis; interrupt latency was not directly measured.
Evidence: `uart_reconnect_after_log_timing/` (result recorded 2026-10-31).

No motor movement commands were sent. Test scripts and captured startup/reconnect
logs are archived beside this report. Older ten-update/interruption results apply
to the earlier image; the changed image received targeted startup regression.
