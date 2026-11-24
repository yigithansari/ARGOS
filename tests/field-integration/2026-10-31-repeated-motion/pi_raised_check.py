"""Operator-authorized raised-wheel test; aborts on missing/abnormal feedback."""
import json
from pathlib import Path
import re
import socket
import sys
import time

target = int(sys.argv[1]) if len(sys.argv) > 1 else 25
assert target in (25, 35, 45)
out = Path(sys.argv[2]) if len(sys.argv) > 2 else Path(__file__).parent / f'pi_raised_20261031_{target}rpm'
out.mkdir(exist_ok=True)
samples = []
sock = socket.create_connection((sys.argv[3] if len(sys.argv) > 3 else '192.168.137.234', 3333), timeout=5)
sock.settimeout(0.1)
sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
buffer = ''

def send(command):
    sock.sendall((command + '\r\n').encode())

def read(seconds, records=[]):
    global buffer
    lines = []
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            data = sock.recv(16384)
            if not data:
                raise RuntimeError('Connection closed')
            buffer += data.decode(errors='replace')
            while '\n' in buffer:
                line, buffer = buffer.split('\n', 1)
                line = line.strip()
                if line:
                    lines.append(line)
                    records.append(f'{time.monotonic():.3f} {line}\n')
        except socket.timeout:
            pass
    return lines

def stopped(lines):
    return any(line.startswith('MOTOR | LEFT:0 RPWM:0 LPWM:0 ') and
               'RIGHT:0 RPWM:0 LPWM:0 ' in line and 'SAFE:STOP' in line for line in lines)

try:
    read(3)
    send('x')
    read(1)
    send('E')
    assert stopped(read(1)), 'Initial STOP not confirmed'
    print(f'START: {target} RPM both wheels, 10 seconds', flush=True)
    start = time.monotonic()
    last_feedback = start
    while time.monotonic() - start < 10:
        send(f'V {target} {target}')
        # V already returns full motor status; avoid duplicate telemetry traffic.
        lines = read(0.35)
        valid = []
        for line in lines:
            match = re.search(rf'CTRL:PI TARGET_RPMx100:\+{target*100}/\+{target*100} .*MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)', line)
            if match:
                elapsed = time.monotonic() - start
                left, right = (int(v)/100 for v in match.groups())
                valid.append((elapsed, left, right))
                if min(left, right) < -2 or max(left, right) > min(60, target * 1.6):
                    raise RuntimeError(f'Abnormal speed/direction: {left}/{right}')
                if elapsed > 3 and min(left, right) < 5:
                    raise RuntimeError(f'Stall/feedback failure: {left}/{right}')
        if valid:
            last_feedback = time.monotonic()
        if time.monotonic() - last_feedback > 1.0:
            raise RuntimeError('No fresh PI feedback; stopping')
        samples.extend(valid)
finally:
    try:
        send('x')
        stop_lines = read(0.5)
        send('E')
        stop_ok = stopped(stop_lines + read(1))
        print(f'STOP confirmed={stop_ok}', flush=True)
    finally:
        sock.close()
        (out / 'telemetry.log').write_text(''.join(read.__defaults__[0]), encoding='utf-8')
        (out / 'samples.json').write_text(json.dumps(samples, indent=2))

settled = [row for row in samples if row[0] >= 6]
assert stop_ok, 'STOP not confirmed; no further tests allowed'
assert settled, 'No settled samples'
means = [sum(row[i] for row in settled)/len(settled) for i in (1, 2)]
result = dict(target_rpm=target, mean_rpm=means, mean_within_5_percent=all(abs(v-target)<=target*.05 for v in means),
              within_5_percent_fraction=[sum(abs(row[i]-target)<=target*.05 for row in settled)/len(settled) for i in (1,2)],
              min_max_rpm=[[min(row[i] for row in settled),max(row[i] for row in settled)] for i in (1,2)],
              stop_confirmed=stop_ok, samples=len(samples), scope='single forward raised-wheel run')
(out / 'result.json').write_text(json.dumps(result, indent=2))
print(json.dumps(result), flush=True)
