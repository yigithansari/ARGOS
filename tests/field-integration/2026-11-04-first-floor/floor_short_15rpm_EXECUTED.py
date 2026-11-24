"""Operator-ready floor test: encoder estimate 100 mm, 2.5 s hard run limit.
Host-side diagnostic only, not an accepted autonomous distance controller.
"""
import ast
import json
import math
from pathlib import Path
import re
import socket
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
config=json.loads((ROOT/'Tools/argos_motion_calibration.json').read_text())
circumference=math.pi*float(config['wheel_diameter_mm'])

def distances(counts,origin):
    return tuple((counts[i]-origin[i])*config[f'{side}_forward_sign']*circumference/
                 config[f'{side}_counts_per_wheel_rev'] for i,side in enumerate(('left','right')))

def decision(travel,elapsed):
    if min(travel)<-5: return 'abort_reverse'
    if max(travel)>140: return 'abort_overtravel'
    if max(travel)>20 and abs(travel[0]-travel[1])>30: return 'abort_wheel_mismatch'
    if elapsed>1 and min(travel)<3: return 'abort_no_motion'
    if sum(travel)/2>=100: return 'target_reached'
    return None

def snapshot(lines):
    rows=[tuple(map(int,m.groups())) for line in lines
          if line.startswith('MOTOR |') and (m:=re.search(r'LEFT:.*? ENC:(-?\d+).*RIGHT:.*? ENC:(-?\d+)',line))]
    return rows[-1] if rows else None

def stopping_projection(travel, rpm):
    speed=max(0.0,sum(rpm)/2)*circumference/60
    lead=min(45.0,max(8.0,speed*.30+5.0))
    return sum(travel)/2+lead

def main(host):
    global sock,buffer,out
    out=Path(__file__).parent/('floor_short_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir()
    buffer=''
    tree=ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
    exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'<IO helpers>','exec'),globals())
    result=dict(target_estimated_mm=100,max_run_seconds=2.5,rpm_command=15,
                stop_delay_model_s=.30,
                coast_allowance_mm=5,geometry=config,scope='predictive host floor screening')
    origin=None
    with socket.create_connection((host,3333),timeout=5) as sock:
        sock.settimeout(.05)
        sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
        try:
            read(2)
            send('x')
            initial=read(.7)
            send('E')
            initial+=read(.5)
            assert stopped(initial), 'Initial zero PWM/STOP absent'
            origin=snapshot(initial)
            assert origin is not None
            result['origin_counts']=origin
            start=last_feedback=time.monotonic()
            last_send=start-1
            reason='time_limit'
            command_rpm=15
            print('START floor: constant 15 RPM, predictive STOP, estimated 100 mm, max 2.5s',flush=True)
            while time.monotonic()-start<2.5:
                now=time.monotonic()
                if now-last_feedback>.8:
                    reason='abort_feedback_age'
                    break
                if now-last_send>=.25:
                    send(f'V {command_rpm} {command_rpm}')
                    last_send=now
                lines=read(.05)
                current=snapshot(lines)
                if current is not None:
                    last_feedback=time.monotonic()
                    travel=distances(current,origin)
                    result['latest_estimated_mm']=travel
                    outcome=decision(travel,last_feedback-start)
                    if outcome:
                        reason=outcome
                        break
                    measured=[m for line in lines if (m:=re.search(r'MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)',line))]
                    if measured:
                        rpm=tuple(int(value)/100 for value in measured[-1].groups())
                        projection=stopping_projection(travel,rpm)
                        result['projected_stop_mm']=projection
                        if projection>=100:
                            reason='predicted_stop'
                            break
                speed=[m for line in lines if (m:=re.search(r'MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)',line))]
                if any(max(abs(int(v)) for v in m.groups())>4000 for m in speed):
                    reason='abort_overspeed'
                    break
            result.update(stop_reason=reason,run_seconds=time.monotonic()-start)
        finally:
            send('x')
            final=read(.6)
            send('E')
            final+=read(.7)
            result['stop_confirmed']=stopped(final)
            end_counts=snapshot(final)
            if origin and end_counts:
                travel=distances(end_counts,origin)
                result.update(final_estimated_mm=travel,mean_estimated_mm=sum(travel)/2,
                              estimated_heading_deg=math.degrees((travel[1]-travel[0])/config['track_width_mm']))
            (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
            (out/'result.json').write_text(json.dumps(result,indent=2))
            print(str(out),json.dumps(result),flush=True)
            assert result['stop_confirmed'], 'STOP not confirmed'

if __name__=='__main__':
    if sys.argv[1:] == ['--self-test']:
        assert decision((-6,0),.1)=='abort_reverse'
        assert decision((141,140),.5)=='abort_overtravel'
        assert decision((25,60),.5)=='abort_wheel_mismatch'
        assert decision((0,0),1.1)=='abort_no_motion'
        assert decision((99,101),1)=='target_reached'
        assert decision((10,12),.5) is None
        d=distances((-48097.2,48036.6),(0,0))
        assert all(abs(v-circumference)<1e-9 for v in d)
        assert stopping_projection((80,80),(15,15))>100
        assert stopping_projection((20,20),(25,25))<100
        assert stopping_projection((0,0),(-25,-25))==8
        print('Floor guard/distance checks: PASS')
    else:
        main(sys.argv[1])
