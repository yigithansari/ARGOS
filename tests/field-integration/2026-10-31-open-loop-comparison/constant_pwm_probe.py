"""Raised-wheel comparison at fixed 52/51 percent; no controller tuning."""
import ast
from pathlib import Path
import re
import socket
import sys
import time

side=sys.argv[1] if len(sys.argv)>1 else 'both'
assert side in ('both','left','right')
left_pwm=0 if side=='right' else 52
right_pwm=0 if side=='left' else 51
out=Path(__file__).parent/('constant_pwm_'+side+'_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
buffer=''
tree=ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'<helpers>','exec'))
with socket.create_connection(('192.168.137.234',3333),timeout=5) as sock:
    sock.settimeout(.1)
    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    previous=None
    try:
        read(2)
        send('x')
        assert stopped(read(1)), 'Initial STOP absent'
        start=last_feedback=time.monotonic()
        print(f'START fixed PWM left={left_pwm}%, right={right_pwm}%, duration=10s',flush=True)
        while time.monotonic()-start<10:
            send(f'P {left_pwm} {right_pwm}')
            for line in read(.35):
                counts=re.search(rf'LEFT:{left_pwm} RPWM:{left_pwm*10} LPWM:0 ENC:(-?\d+).*RIGHT:{right_pwm} RPWM:{right_pwm*10} LPWM:0 ENC:(-?\d+).*SAFE:RUN.*CTRL:PWM.*T_MS:(\d+)',line)
                if not counts:
                    continue
                left,right,tick=map(int,counts.groups())
                if previous and tick-previous[2]>=100:
                    dt=(tick-previous[2])/1000
                    rpm=[-(left-previous[0])*60/48097.2/dt,(right-previous[1])*60/48036.6/dt]
                    energized=[rpm[i] for i,pwm in enumerate((left_pwm,right_pwm)) if pwm]
                    if min(energized)<-2 or max(energized)>60:
                        raise RuntimeError(f'Unexpected direction/speed {rpm}')
                    if time.monotonic()-start>3 and min(energized)<5:
                        raise RuntimeError(f'Stall/feedback issue {rpm}')
                previous=(left,right,tick)
                last_feedback=time.monotonic()
            if time.monotonic()-last_feedback>1:
                raise RuntimeError('Feedback age exceeded 1s')
    finally:
        send('x')
        final=read(.5)
        send('E')
        final+=read(1)
        stop_ok=stopped(final)
        (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
        print(str(out),f'STOP={stop_ok}',flush=True)
        assert stop_ok, 'Final STOP unconfirmed'
