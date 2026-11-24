"""Exploratory phase correlation from existing constant-PWM logs; no hardware IO."""
import json
import math
from pathlib import Path
import re
import sys
def fit_three(x,y):
    matrix=[[sum(row[i]*row[j] for row in x) for j in range(3)]
            + [sum(row[i]*value for row,value in zip(x,y))] for i in range(3)]
    for col in range(3):
        pivot=max(range(col,3),key=lambda i:abs(matrix[i][col]))
        matrix[col],matrix[pivot]=matrix[pivot],matrix[col]
        scale=matrix[col][col]
        if abs(scale)<1e-12:
            raise ValueError('Insufficient phase coverage')
        matrix[col]=[value/scale for value in matrix[col]]
        for row in range(3):
            if row!=col:
                scale=matrix[row][col]
                matrix[row]=[a-scale*b for a,b in zip(matrix[row],matrix[col])]
    return [matrix[i][3] for i in range(3)]

path=Path(sys.argv[1])
rows=[]
for line in path.read_text(encoding='utf-8').splitlines():
    counts=re.search(r'LEFT:.*? ENC:(-?\d+).*RIGHT:.*? ENC:(-?\d+)',line)
    tick=re.search(r'T_MS:(\d+)',line)
    if counts and tick and 'CTRL:PWM' in line and 'SAFE:RUN' in line:
        rows.append((int(tick[1]),*map(int,counts.groups())))
assert len(rows)>10
results=[]
for side,cpr,sign in ((1,48097.2,-1),(2,48036.6,1)):
    data=[]
    for previous,current in zip(rows,rows[1:]):
        dt=(current[0]-previous[0])/1000
        if dt<.1 or current[0]-rows[0][0]<3000:
            continue
        rpm=sign*(current[side]-previous[side])*60/cpr/dt
        phase=2*math.pi*sign*(current[side]+previous[side])/2/cpr
        data.append((rpm,math.sin(phase),math.cos(phase)))
    if not data or max(abs(row[0]) for row in data)<1:
        results.append(dict(side=side,active=False))
        continue
    y=[row[0] for row in data]
    x=[[1,row[1],row[2]] for row in data]
    coef=fit_three(x,y)
    mean=sum(y)/len(y)
    total=sum((value-mean)**2 for value in y)
    r2=1-sum((value-sum(a*b for a,b in zip(row,coef)))**2 for row,value in zip(x,y))/total if total else None
    results.append(dict(side=side,active=True,samples=len(data),
                        wheel_frequency_amplitude_rpm=math.hypot(coef[1],coef[2]),
                        r_squared=r2))
result=dict(exploratory=True,source=str(path),results=results,
            limitation='Single short run; counts define both phase and speed. Correlation cannot identify physical cause.')
path.with_name('wheel_phase_analysis.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result),flush=True)
