"""Short open-loop pulses verify restored motor/encoder mapping before PI."""
import ast
import json
from pathlib import Path
import re
import socket
import sys
import time

host=sys.argv[1]
out=Path(__file__).parent/('mapping_check_'+time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
buffer=''
tree=ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n,ast.FunctionDef)],type_ignores=[]),'<helpers>','exec'))
def counts(lines):
    rows=[tuple(map(int,m.groups())) for line in lines
          if (m:=re.search(r'LEFT:.*? ENC:(-?\d+).*RIGHT:.*? ENC:(-?\d+)',line))]
    assert rows, 'No encoder snapshot'
    return rows[-1]
with socket.create_connection((host,3333),timeout=5) as sock:
    sock.settimeout(.1)
    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    results=[]
    try:
        read(2)
        for side,command in ((0,'P 50 0'),(1,'P 0 50')):
            send('x')
            assert stopped(read(1))
            send('E')
            before=counts(read(.4))
            send(command)
            read(.7)
            send('x')
            final=read(.7)
            send('E')
            final+=read(.5)
            assert stopped(final), 'STOP absent'
            after=counts(final)
            delta=[b-a for a,b in zip(before,after)]
            ok=(delta[side]*(-1 if side==0 else 1)>1000 and abs(delta[1-side])<200)
            results.append(dict(side=side,encoder_delta=delta,matches_normal=ok,stop=True))
            print(json.dumps(results[-1]),flush=True)
            assert ok, 'Normal motor/encoder mapping not confirmed'
    finally:
        send('x')
        read(.5)
        (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
        (out/'result.json').write_text(json.dumps(results,indent=2))
        print(str(out),flush=True)
