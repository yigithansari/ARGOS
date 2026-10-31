# ARGOS Live Sensor Health Report

**Date:** 2026-10-29
**Connection:** ESP32-C3 Wi-Fi bridge at `192.168.137.33:3333`
**Motor state:** Explicit STOP before and after test
**Result:** PASS WITH MPU CALIBRATION NOTE

## I2C Inventory

| Address | Device |
| --- | --- |
| `0x30` | Left VL53L0X |
| `0x31` | Right VL53L0X |
| `0x68` | MPU-compatible IMU |

The scan completed with three devices and no missing address.

## MPU Health

- Identity response: `WHO_AM_I 0x72`, accepted by the current compatible-driver path.
- Single sample: acceleration `+0.029 / +0.039 / +1.073 g`.
- Temperature: approximately `44.5 C`.
- Live samples captured: `27`.
- Data remained continuous with no read failure.
- Stationary gyro values clustered around approximately `X -3.1`, `Y +5.6`,
  `Z 0 deg/s`.

The IMU is communicating and producing coherent acceleration data. The nonzero
stationary gyro values indicate an offset-calibration requirement, not a link
or sensor-health failure. Run the existing gyro calibration command while the
chassis is completely still before using yaw-rate integration.

## ToF Health

- Address assignment: left `0x30 OK`, right `0x31 OK`.
- Both sensors returned model `0xEE`, module `0xAA`, revision `0x10`.
- Direct range test: left `648 mm`, right `907 mm`.
- Live range samples captured: `13`.
- Left live range: `611-640 mm`.
- Right live range: `904-939 mm`.
- No `YOK`, timeout, address loss, or invalid-range line was observed.

Both ToF sensors are healthy and updating. Different left/right distances are
expected because the sensors face 15 degrees left and right with a 70 mm
baseline and currently see different parts of the environment.

## Link Health During Test

- ESP reported STM `ONLINE` throughout.
- Wi-Fi RSSI ranged approximately from `-41` to `-49 dBm`.
- MPU and ToF streams were restored after the diagnostic sequence.

## Decision

No ToF hardware repair is indicated. A previous panel `WAITING` or stale ToF
display should be treated as a stream/link-state issue unless a future direct
test loses `0x30` or `0x31`. Calibrate the gyro before motion-estimation work.
