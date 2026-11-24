# Explore / map

Use the rebuilt STM32 OTA firmware and the current control panel. HC-SR04 TRIG
uses D3/PB3, ECHO uses D4/PB5, with shared ground. PB3 is configured as GPIO
instead of SWO. The side sensor faces left and the software subtracts a 105 mm
inset from its range to estimate clearance from the left edge.

1. Verify front and side readings with the robot stationary, then verify STOP and
   obstacle/range-loss behavior with the wheels secured clear of the floor.
2. For a floor run, open **EXPLORE / MAP**, describe the setup and select
   **START EXPLORATION**. The panel requests STOP status and range telemetry
   from GUARD:10. Confirm rear clearance for recovery steps and keep
   access to STOP.
3. The STM samples ranges during motion. Manual drive has direct control.
   A wall seen by both front sensors at up to 450 mm can trigger an early
   right arc directly from cruise. The left wheel targets 25 RPM and the
   right wheel ramps toward zero. Cruise resumes after heading and fresh
   clearance checks. An interrupted arc reports the actual range guard.
   Autonomous forward cruises at 25 RPM with encoder trim during straight
   travel and uses 22 RPM for the inner wheel when steering. It stops when both fresh front
   ranges are at most 200 mm. After confirmed STOP, fresh ranges can lead to
   a 90 degree right pivot. Either front sensor below 150 mm inhibits the
   pivot. Recovery backs up by 30–90 mm according to measured clearance
   before resuming with a right-wheel reverse arc once the left front
   reads at least 190 mm and available left-side clearance is at least 80 mm.
   A close right-front return does not veto this arc. Within eight degrees
   of the target heading, fresh clearance can permit a forward right arc
   to leave recovery. The panel range field
   shows the left ultrasonic beam in purple.
   Fresh optical failures 4/6/9 with side clearance allow up to 30 cm
   of search in short steps. The side echo guides steering when available.
   Sessions remain capped at 90 s or 40 maneuvers.
4. **MARK NOTE** attaches the entry text and pose to the active log. On STOP or
   completion the panel saves a JSONL log, JSON summary and 2D SVG map under
   `Tools/experiment_logs/`. **START LOG** still supports manual experiments.

The SVG shows front ToF endpoints, purple left ultrasound returns and the encoder path. Stationary scans use
the settled pose and moving scans are projected between nearby encoder poses.
The side range guides wall following and inhibits autonomous turns below
80 mm estimated left-edge clearance. Map projection estimates the sensor at
x=0 and y=half track minus inset. Measured side_sensor_x_mm and side_sensor_y_mm
in motion calibration override that estimate.
Origin changes produce separate map panels. The map uses nominal front sensor
angles and encoder odometry, so field geometry accuracy is pending.

Front rays and one side beam leave unobserved areas. Rear driving has no rear
range coverage. Obstacle stopping thresholds and stopping distance require
hardware verification before routine floor use.

Replay JSONL without the robot:
`python argos_exploration.py --replay experiment_logs/<session>.jsonl`

Send the JSONL file for analysis. It contains raw frames, motor and range data,
movement outcomes, pose, timestamps and operator notes.
