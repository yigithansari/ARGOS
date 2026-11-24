# Left-wheel removal isolation — 2026-11-04

Operator powered off and removed only the left wheel, kept motor/encoder wiring
unchanged, secured the robot with right wheel fully clear, powered on and
explicitly reported ready. Firmware remained44736 bytes, CRC0C70506B; no gain
change was made. Two35 RPM/20 s runs used existing safety checks.

| Final4 s metric | Wheel fitted baseline | Wheel removed run1 | Wheel removed run2 |
| --- | --- | --- | --- |
| Left mean RPM |35.585 |35.596 |35.446 |
| Left range RPM |30.83–41.30 |33.41–36.51 |33.57–36.65 |
| Right mean RPM |35.100 |34.877 |35.385 |
| Both wheels' samples within±5% | No | Yes | Yes |
| Final zero-output STOP | Confirmed | Confirmed | Confirmed |

Filtered-speed versus encoder-phase analysis over STM-timed8+ s samples:
left first-harmonic amplitude5.096 RPM fitted,0.134/0.467 RPM removed;
R²0.984 fitted,0.011/0.098 removed. See phase_analysis.json. The calculation
script was run in its original Tools/installation_backups location; it reads the
central calibration JSON and the saved named runs. Baseline and rejected gain
trial evidence are in the sibling pi-gain-comparison archive.

The removal intervention strongly implicates a wheel-mounted load/attachment
interaction. It does not isolate wheel imbalance/eccentricity, hub alignment,
mounting contact or motor shaft/bearing behavior under load. It does not prove
the encoder intrinsically fault-free or complete loaded PI acceptance.

Next requested controlled isolation: power off, exchange only the two wheels,
preserve motor and encoder wiring, secure both wheels clear and power on. Await
operator readiness before repeating35 RPM. The swap has not yet been confirmed
or tested; last verified state is STOP with original left wheel removed.
