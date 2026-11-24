"""MCU-local forward move diagnostic. Default checks reject-only commands.
Use --execute only after operator confirms start mark and clear floor area.
"""
import argparse
import json
import math
from pathlib import Path
import re
import socket
import time


def target_counts(distance_mm, config):
    if not math.isfinite(distance_mm) or not 30 <= distance_mm <= 150:
        raise ValueError('Initial floor diagnostic permits 30..150 mm only')
    if (config['left_forward_sign'], config['right_forward_sign']) != (-1, 1):
        raise ValueError('Firmware encoder direction convention must be validated first')
    circumference = math.pi * float(config['wheel_diameter_mm'])
    if not math.isfinite(circumference) or circumference <= 0:
        raise ValueError('Invalid wheel diameter')
    goals = tuple(round(distance_mm/circumference*float(config[f'{side}_counts_per_wheel_rev']))
                  for side in ('left', 'right'))
    if min(goals) < 5000:
        raise ValueError('Requested encoder distance below firmware limit')
    return goals


def snapshot(lines):
    rows = [tuple(map(int, m.groups())) for line in lines
            if line.startswith('MOTOR |') and
            (m := re.search(r'LEFT:.*? ENC:(-?\d+).*RIGHT:.*? ENC:(-?\d+)', line))]
    return rows[-1] if rows else None


def stopped(lines):
    return any(line.startswith('MOTOR | LEFT:0 RPWM:0 LPWM:0 ') and
               'RIGHT:0 RPWM:0 LPWM:0 ' in line and 'SAFE:STOP' in line for line in lines)


def run(args):
    config = json.loads(Path(__file__).with_name('argos_motion_calibration.json').read_text())
    goals = target_counts(args.distance_mm, config)
    out = Path(__file__).parent/'installation_backups'/('mcu_move_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True)
    records = []
    buffer = ''
    result = dict(execute=args.execute, requested_mm=args.distance_mm, goals=goals, geometry=config)
    with socket.create_connection((args.host, 3333), timeout=5) as sock:
        sock.settimeout(.05)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        def send(command):
            sock.sendall((command+'\r\n').encode('ascii'))

        def read(seconds):
            nonlocal buffer
            lines = []
            end = time.monotonic()+seconds
            while time.monotonic()<end:
                try:
                    data = sock.recv(16384)
                    if not data:
                        raise RuntimeError('Control connection closed')
                    buffer += data.decode(errors='replace')
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            records.append(f'{time.monotonic():.3f} {line}\n')
                            lines.append(line)
                except socket.timeout:
                    pass
            return lines

        origin = None
        try:
            read(2)
            send('x')
            initial = read(.7)
            send('E')
            initial += read(.4)
            assert stopped(initial), 'Initial STOP not confirmed'
            origin = snapshot(initial)
            send('G KEEP')
            assert any(line == 'MOVE IDLE' for line in read(.5)), 'Firmware MOVE capability absent'
            if not args.execute:
                checks = []
                invalid = ('G 0 0', 'G -10 10000', 'G 5000 5000 junk',
                           'G 999999999999999999999 6000', 'G 6000 10000',
                           'G 6000 6000'+' '*45+'BAD')
                for command in invalid:
                    send(command)
                    lines = read(.4)
                    send('E')
                    lines += read(.4)
                    rejected = any(line.startswith('MOVE ERROR') or 'LINE_TOO_LONG' in line for line in lines)
                    assert rejected and stopped(lines), 'Invalid command rejection/STOP failed'
                    assert snapshot(lines) == origin, 'Unexpected encoder movement in reject-only checks'
                    checks.append(dict(command=command, rejected=True, stopped=True))
                result['reject_checks'] = checks
            else:
                send(f'G {goals[0]} {goals[1]}')
                started = time.monotonic()
                last_keep = started
                end_marker = None
                while time.monotonic()-started < 6:
                    lines = read(.1)
                    end_marker = next((line for line in lines if line.startswith(('MOVE DONE', 'MOVE ABORT', 'MOVE CANCELLED'))), None)
                    if end_marker:
                        break
                    if any(line.startswith('MOVE ERROR') for line in lines):
                        raise RuntimeError('Firmware rejected move')
                    if time.monotonic()-last_keep >= .25:
                        send('G KEEP')
                        send('E')
                        last_keep = time.monotonic()
                result['completion'] = end_marker
                assert end_marker is not None, 'No completion within host observation bound'
        finally:
            send('x')
            final = read(.7)
            send('E')
            final += read(.5)
            result['stop_confirmed'] = stopped(final)
            end = snapshot(final)
            if origin and end:
                circumference = math.pi*config['wheel_diameter_mm']
                result['final_encoder_mm'] = [
                    (end[i]-origin[i])*config[f'{side}_forward_sign']*circumference/
                    config[f'{side}_counts_per_wheel_rev'] for i, side in enumerate(('left','right'))]
            (out/'telemetry.log').write_text(''.join(records), encoding='utf-8')
            (out/'result.json').write_text(json.dumps(result, indent=2), encoding='utf-8')
            print(str(out), json.dumps(result), flush=True)
            assert result['stop_confirmed'], 'Final STOP not confirmed'


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    parser.add_argument('--distance-mm', type=float, default=100)
    parser.add_argument('--execute', action='store_true')
    run(parser.parse_args())
