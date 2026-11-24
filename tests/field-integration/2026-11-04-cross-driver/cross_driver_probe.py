"""Temporary wiring: right driver -> physical left motor; encoders unchanged.
Open-loop P 0 N only. Do not use normal PI tests with this wiring.
"""
import ast
import json
from pathlib import Path
import re
import socket
import sys
import time

host=sys.argv[1]
command_sign=int(sys.argv[2]) if len(sys.argv)>2 else 1
assert command_sign in (-1,1)
out=Path(__file__).parent/('cross_driver_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
buffer=''
tree=ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'<helpers>','exec'))
results=[]
with socket.create_connection((host,3333),timeout=5) as sock:
    sock.settimeout(.1)
    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    try:
        read(2)
        send('x')
        assert stopped(read(1)), 'Initial STOP absent'
        for pwm,duration in ((50,1.5),(52,10)):
            command=command_sign*pwm
            rpwm=pwm*10 if command>0 else 0
            lpwm=pwm*10 if command<0 else 0
            previous=first=None
            speeds=[]
            start=last_feedback=time.monotonic()
            print(f'Right driver PWM {command}%, physical LEFT encoder feedback, {duration}s',flush=True)
            while time.monotonic()-start<duration:
                send(f'P 0 {command}')
                for line in read(.35):
                    match=re.search(rf'LEFT:0 RPWM:0 LPWM:0 ENC:(-?\d+).*RIGHT:{command} RPWM:{rpwm} LPWM:{lpwm} ENC:(-?\d+).*SAFE:RUN.*CTRL:PWM.*T_MS:(\d+)',line)
                    if not match:
                        continue
                    current=tuple(map(int,match.groups()))
                    if first is None:
                        first=current
                    if abs(current[1]-first[1])>200:
                        raise RuntimeError('Unexpected physical RIGHT encoder movement; wiring isolation not confirmed')
                    if previous and current[2]-previous[2]>=100:
                        dt=(current[2]-previous[2])/1000
                        rpm=-(current[0]-previous[0])*60/48097.2/dt
                        speeds.append(rpm)
                        if abs(rpm)>60 or (time.monotonic()-start>3 and abs(rpm)<5):
                            raise RuntimeError(f'Abnormal left speed/stall {rpm}')
                    previous=current
                    last_feedback=time.monotonic()
                if time.monotonic()-last_feedback>1:
                    raise RuntimeError('Feedback timeout')
            send('x')
            stop_lines=read(.5)
            send('E')
            assert stopped(stop_lines+read(1)), 'Stage STOP absent'
            assert first and previous and abs(previous[0]-first[0])>1000, 'Physical left motion not confirmed'
            result=dict(right_driver_pwm_percent=command,physical_motor='left',feedback_encoder='left',
                        left_delta=previous[0]-first[0],right_delta=previous[1]-first[1],
                        normalized_left_direction=1 if sum(speeds)>0 else -1,stop=True)
            results.append(result)
            print(json.dumps(result),flush=True)
            (out/'results.json').write_text(json.dumps(results,indent=2))
    finally:
        try:
            send('x')
            final=read(.5)
            send('E')
            assert stopped(final+read(1)), 'Final STOP absent'
        finally:
            (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
            print(str(out),flush=True)
