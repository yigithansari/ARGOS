import ast
import json
from pathlib import Path
import socket
import time

out = Path(__file__).parent / ('disconnect_stop_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
buffer = ''
tree = ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)],type_ignores=[]),'<helpers>','exec'))
def connect():
    s=socket.create_connection(('192.168.137.234',3333),timeout=5)
    s.settimeout(.1)
    s.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    return s
sock=connect()
try:
    read(2)
    send('x')
    assert stopped(read(1)), 'Initial STOP absent'
    send('V 25 25')
    moving=read(.5)
    assert any('SAFE:RUN' in line and 'CTRL:PI' in line for line in moving)
    print('Closing control socket; single command, no refresh',flush=True)
    closed=time.monotonic()
    sock.close()
    buffer=''
    sock=connect()
    lines=read(2)
    send('E')
    lines+=read(1)
    result=dict(post_disconnect_stop=stopped(lines),
                note='Zero PWM/STOP after reconnect; exact electrical stop latency not measured')
    (out/'result.json').write_text(json.dumps(result,indent=2))
    print(str(out),json.dumps(result),flush=True)
    assert result['post_disconnect_stop']
finally:
    try:
        send('x')
        read(.5)
    finally:
        sock.close()
        (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
