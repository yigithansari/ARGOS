import json
from pathlib import Path
import re
import statistics
import sys

path=Path(sys.argv[1])
mode=sys.argv[2] if len(sys.argv)>2 else 'PI'
assert mode in ('PI','PWM')
rows=[]
for line in path.read_text(encoding='utf-8').splitlines():
    counts=re.search(r'LEFT:.*? ENC:(-?\d+).*?RIGHT:.*? ENC:(-?\d+)',line)
    timing=re.search(r'T_MS:(\d+) LOOP_MS:(\d+) LOOP_MAX_MS:(\d+)',line)
    measured=re.search(rf'CTRL:{mode} .*MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)',line)
    if counts and timing and measured and 'SAFE:RUN' in line:
        rows.append([float(line.split()[0]),*map(int,timing.groups()),*map(int,counts.groups()),*map(int,measured.groups())])
assert len(rows)>10, 'Missing complete timestamped PI records'
settled=[row for row in rows if row[1]-rows[0][1]>=6000]
window=[]
for current in settled:
    prior=[row for row in rows if 900<=current[1]-row[1]<=1400]
    if not prior:
        continue
    previous=min(prior,key=lambda row:abs(current[1]-row[1]-1000))
    dt=(current[1]-previous[1])/1000
    window.append([-(current[4]-previous[4])*60/48097.2/dt,
                   (current[5]-previous[5])*60/48036.6/dt])
result=dict(mode=mode,records=len(rows),loop_ms_min=min((row[2] for row in rows if row[2]),default=0) if mode=='PI' else None,
            loop_ms_max=max(row[3] for row in rows) if mode=='PI' else None,
            mcu_to_host_elapsed_ratio=(rows[-1][1]-rows[0][1])/1000/(rows[-1][0]-rows[0][0]),
            window_samples=len(window), window_definition='nearest 1 s, allowed 0.9-1.4 s, MCU timestamps',
            window_rpm_mean=[statistics.mean(row[i] for row in window) for i in (0,1)],
            window_rpm_min_max=[[min(row[i] for row in window),max(row[i] for row in window)] for i in (0,1)],
            window_rpm_std=[statistics.pstdev(row[i] for row in window) for i in (0,1)],
            window_rpm_cv=[statistics.pstdev(row[i] for row in window)/statistics.mean(row[i] for row in window) if statistics.mean(row[i] for row in window) else None for i in (0,1)],
            window_within_5_percent=[sum(abs(row[i]-25)<=1.25 for row in window)/len(window) for i in (0,1)] if mode=='PI' else None)
path.with_name('timing_analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result),flush=True)
