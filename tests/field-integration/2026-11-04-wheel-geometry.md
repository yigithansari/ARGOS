# Wheel replacement and central geometry - 2026-11-04

Operator reported old wooden wheels rubbed the motor-mount screws. Replacement
wheels installed with zero rubbing reported. Outer tread diameter 87 mm; 255 mm
track explicitly confirmed between wheel centre planes. Nominal circumference
273.318560862312 mm; loaded rolling measurement and new-wheel motor acceptance pending.

Active source of truth: ARGOS/ARGOS/Tools/argos_motion_calibration.json.
Version 2 stores wheel_diameter_mm and track_width_mm. Panel derives circumference;
its existing measured-circumference UI saves diameter=circumference/pi. Old JSON
with circumference remains compatible. Missing/invalid geometry stays unset.
Encoder CPR/signs retained assuming unchanged motor/gearbox; no firmware geometry
consumer exists yet, so no STM build/flash or motion test was required/performed.

Five offline geometry checks and seven OTA panel regression checks passed; Python
syntax passed. Published panel/JSON snapshots intentionally synchronized and
published geometry tests/hash comparison passed. Restart panel to load changes.
Old calibration JSON backed up under active Tools/installation_backups/.

Earlier friction-affected motion results are historical evidence, not acceptance
of the replacement wheels. Recheck free rotation and raised-wheel speed behavior
before floor/odometry validation. No gains, pin maps or safety limits changed.
