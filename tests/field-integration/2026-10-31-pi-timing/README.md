# PI timing diagnosis - 2026-10-31

MCU T_MS/LOOP_MS/LOOP_MAX_MS appended to motor telemetry. Original diagnostic
image 7056522C measured max PI interval 171 ms. Additional main-loop motor service
points alone (3329D341) measured 176 ms. OLED runtime rendering was then split
into one row per main-loop call, retaining 500 ms frame scheduling; final image
5E1584A0 measured 124 ms in the same single 25 RPM/10 s test.

Final mean filtered RPM 25.063/25.439, but settled band fractions 70%/10%.
Nearest-one-second encoder windows also varied (L 24.51-27.41, R 23.33-27.46 RPM).
Clock elapsed ratio MCU/host 1.0002. This demonstrates reduced scheduling stalls,
not solved wheel-speed tracking. Each intermediate/final run confirmed STOP.

Final build/OTA/CRC/startup/STOP checks passed; final single-command timeout
returned TIMEOUT_STOP and final STOP. Thirty HTTP probes succeeded (max 281 ms).
No exact electrical stop-latency measurement, floor test, gain or scaling change.
OLED physical display appearance remains to be observed by operator.

Three timing samples are sequential single runs, not statistical proof of a
maximum bound. Instrumentation adds UART bytes and encoder analysis uses MCU
timestamps; external tachometer validation remains separate. Next: constant PWM
versus closed-loop variation before choosing controller changes.
