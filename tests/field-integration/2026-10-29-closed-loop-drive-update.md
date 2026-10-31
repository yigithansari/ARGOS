# Closed-loop straight-drive update

Date: 2026-10-29

## Implemented

- Straight commands with equal left/right targets now use encoder feedback.
- Synchronization starts above 55% command after a 350 ms launch delay.
- The controller samples each encoder every 100 ms and limits correction to +/-8%.
- Pivot turns, differential steering and single-motor diagnostics are not modified.
- The existing 1.2 s motor command timeout and direction-change stop remain active.
- `S SYNC 0/1` disables or enables synchronization; it is enabled at startup.
- Motor telemetry now reports sync state, relative error, trim and applied outputs.
- The paired section of `MOTOR RAMP TEST` services the closed-loop controller.
- Resetting encoder counters also resets the synchronization baseline.

## Control-panel changes

- Motor telemetry visualizes sync state, error, trim and actual applied percentages.
- `CALIBRATE IMU` stops the drive, pauses the MPU stream, runs stationary gyro
  calibration and restores the stream.
- System Test explicitly enables synchronization before the motor ramp test.

## Verification

- Control-panel Python compilation: PASS.
- STM32CubeIDE incremental build: PASS; updated `ARGOS.elf`, object and map files
  were generated on 2026-10-29 20:49.
- The map contains `Motor_ApplyOutputs`, `Motor_ResetSyncState` and `Motor_Task`.
- Hardware flash: PASS. STM32CubeProgrammer v2.23.0 programmed and verified the
  37.79 KB ELF on NUCLEO-F446RE, ST-LINK `066BFF515086874967242707`.
- Post-flash safety telemetry: PASS (`SAFE:STOP`, `SYNC:IDLE`, applied `0/0`).
- Post-flash link and I2C: PASS (ESP bridge `192.168.137.33`, RSSI -33/-34 dBm;
  devices `0x30`, `0x31`, `0x68`; MPU communication OK).
- ToF identity: PASS for both sensors. Range validation needs a physical target:
  left repeatedly reported 20 mm and right reported no valid range during the
  unattended check.
- Loaded-wheel straight-drive validation: PENDING.

## Next hardware run

1. Raise the wheels or provide clear space.
2. Open the updated panel and run `CALIBRATE IMU` while the robot is still.
3. Put a matte target 20-60 cm in front of each ToF and confirm valid ranges.
4. Run System Test and save the report.
5. Drive straight at 60%, 70% and 80%; confirm `SYNC:ACTIVE`, bounded trim and
   visibly reduced heading drift.
