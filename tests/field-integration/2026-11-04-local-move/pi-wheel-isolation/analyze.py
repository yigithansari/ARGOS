"""Compare saved runs only. Harmonic fit is diagnostic, not a cause attribution."""
import json
import math
from pathlib import Path
import re


def solve(matrix, rhs):
    rows = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda index: abs(rows[index][col]))
        rows[col], rows[pivot] = rows[pivot], rows[col]
        divisor = rows[col][col]
        rows[col] = [value / divisor for value in rows[col]]
        for index in range(3):
            if index != col:
                factor = rows[index][col]
                rows[index] = [a - factor*b for a, b in zip(rows[index], rows[col])]
    return [row[-1] for row in rows]


root = Path(__file__).resolve().parent.parent
config = json.loads((root.parent/'argos_motion_calibration.json').read_text())
results = {}
for name in ('pi_20261104_local_move_regression_35', 'pi_20261104_left_kp008_35',
             'pi_20261104_left_wheel_removed_35',
             'pi_20261104_left_wheel_removed_35_repeat'):
    rows = []
    for line in (root/name/'telemetry.log').read_text().splitlines():
        if 'CTRL:PI TARGET_RPMx100:+3500/+3500' not in line:
            continue
        enc = re.search(r'LEFT:.*? ENC:(-?\d+).*?RIGHT:.*? ENC:(-?\d+)', line)
        speeds = re.search(r'MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)', line)
        tick = re.search(r'T_MS:(\d+)', line)
        if enc and speeds and tick:
            rows.append((int(tick[1]), *map(int, enc.groups()), *[int(x)/100 for x in speeds.groups()]))
    start = rows[0][0]
    rows = [row for row in rows if row[0]-start >= 8000]
    results[name] = {}
    for index, side in enumerate(('left', 'right'), 1):
        cpr = config[f'{side}_counts_per_wheel_rev']
        theta = [row[index]/cpr*2*math.pi for row in rows]
        design = [[1, math.sin(t), math.cos(t)] for t in theta]
        y = [row[index+2] for row in rows]
        matrix = [[sum(row[i]*row[j] for row in design) for j in range(3)] for i in range(3)]
        rhs = [sum(row[i]*v for row, v in zip(design, y)) for i in range(3)]
        fit = solve(matrix, rhs)
        mean = sum(y)/len(y)
        total = sum((v-mean)**2 for v in y)
        residual = sum((v-sum(a*b for a, b in zip(row, fit)))**2 for row, v in zip(design, y))
        results[name][side] = dict(samples=len(y), mean_rpm=mean,
            rms_error_rpm=math.sqrt(sum((v-35)**2 for v in y)/len(y)),
            first_harmonic_amplitude_rpm=math.hypot(fit[1], fit[2]),
            phase_fit_r_squared=1-residual/total)

Path(__file__).with_name('phase_analysis.json').write_text(json.dumps(results, indent=2))
print(json.dumps(results))
