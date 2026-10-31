# Offline panel follow-up - 2026-10-30

## Change and validation

The active panel previously checked only binary size. It now validates the
initial stack and reset vectors before confirmation/upload, preventing accidental
selection of bootstrap/normal-address binaries for relocated OTA programming.
This is input screening, not proof of firmware authenticity or compatibility.

Seven offline tests passed using the actual panel validation/HTTP worker functions:
current OTA accepted; bootstrap rejected; invalid sizes/vectors rejected;
multipart binary and size/CRC headers preserved; HTTP failure surfaced;
unexpected HTTP 200 body rejected; timeout surfaced and connection closed.
Python syntax checks for panel and packager passed.

Command: run `ARGOS/ARGOS/Tools/test_ota_panel.py` with the installed Python.
Tests use existing build images and mock HTTP; they do not connect to the robot.
GUI interaction remains pending. Restart the panel to use the changes.

## Startup command corruption

Reviewed STM UART initialization, line queue, RX/error callbacks, and ESP
network notification/probing around startup. No root cause is established.
An earlier immediate query was echoed as WE. Ten later OTA runs passed with
continuous TCP reads, an eight-second startup allowance and bounded status retries.
Those conditions are diagnostic context, not a firmware fix. No hardware mapping,
protocol, timeout, encoder scaling, flash layout or firmware was changed here.

## Operator-assisted tests remaining

Keep robot stationary and wheels clear; retain USB/ST-LINK cables for recovery.
Do not perform power cuts without coordinated timing.

1. Real Wi-Fi outage before erase: deliberately slow the upload so its cache
   phase can be identified, interrupt hotspot before any bootloader request,
   restore Wi-Fi, verify old application and STOP, then retry normally.
2. Power loss during STM programming: arrange a controlled transfer pause/window
   after BL ERASED and before BL END so the operator can interrupt robot power.
   Confirm from logs that interruption actually occurred in that window.
3. Restore robot power and hotspot; confirm recovery bootloader, retry a valid
   OTA image, and verify CRC, application restart, sensors and STOP.
4. If wireless recovery fails, preserve logs and recover through the documented
   ST-LINK bootstrap procedure. Do not count a wired recovery as an OTA recovery pass.
5. Run sensor/drive regression and raised-wheel PI 25/35/45 RPM acceptance with
   operator present. These are separate from the completed stationary OTA tests.

Physical pin levels during interruption still require observation/measurement;
telemetry alone cannot establish electrical motor safety.
