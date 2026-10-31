import ast
import http.client
import json
from pathlib import Path
import socket
import threading
import time

out = Path(__file__).parent / ('single_shot_' + time.strftime('%Y%m%d_%H%M%S'))
out.mkdir()
buffer = ''
tree = ast.parse(Path(__file__).with_name('pi_raised_check.py').read_text())
exec(compile(ast.Module(body=[n for n in tree.body if isinstance(n, ast.FunctionDef)], type_ignores=[]), '<helpers>', 'exec'))
probes = []
done = threading.Event()
def probe():
    while not done.is_set():
        start = time.monotonic()
        c = http.client.HTTPConnection('192.168.137.234',3335,timeout=1)
        try:
            c.request('GET','/status')
            r = c.getresponse()
            detail = r.read().decode(errors='replace')
            probes.append([start,time.monotonic()-start,r.status,detail])
        except Exception as e:
            probes.append([start,time.monotonic()-start,str(e)])
        finally:
            c.close()
        done.wait(.2)

with socket.create_connection(('192.168.137.234',3333),timeout=5) as sock:
    sock.settimeout(.1)
    sock.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
    read(2)
    send('x')
    assert stopped(read(1))
    thread = threading.Thread(target=probe,daemon=True)
    thread.start()
    read(2)
    start = time.monotonic()
    print('Single V 25 25; no command refresh, existing 1200 ms timeout active',flush=True)
    try:
        send('V 25 25')
        lines = []
        while time.monotonic()-start < 5:
            send('E')
            lines.extend(read(.2))
    finally:
        send('x')
        final = read(1)
        send('E')
        final += read(1)
        done.set()
        thread.join(2)
        (out/'telemetry.log').write_text(''.join(read.__defaults__[0]),encoding='utf-8')
        (out/'http.json').write_text(json.dumps(probes,indent=2))
        result = dict(timeout_stop=any('SAFE:TIMEOUT_STOP' in line and 'RPWM:0 LPWM:0' in line for line in lines),
                      final_stop=stopped(final),http_probes=len(probes),
                      http_errors=sum(len(row)==3 for row in probes),max_http_seconds=max(row[1] for row in probes))
        (out/'result.json').write_text(json.dumps(result,indent=2))
        print(str(out),json.dumps(result),flush=True)
