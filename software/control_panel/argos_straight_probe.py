"""MCU-local move diagnostic. Default checks reject-only commands.
Use --execute only after operator confirms start mark and clear floor area.
"""
import argparse
import json
import math
from pathlib import Path
import re
import socket
import time

from argos_control_panel import plan_local_move


def target_counts(distance_mm, config):
    return tuple(map(int, plan_local_move(config, 'F', distance_mm).split()[-2:]))


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
    direction = getattr(args, 'direction', 'F')
    value = args.angle_deg if direction in ('L', 'R') else args.distance_mm
    command = plan_local_move(config, direction, value)
    goals = tuple(map(int, command.split()[-2:]))
    out = Path(__file__).parent/'installation_backups'/('mcu_move_'+time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True)
    records = []
    buffer = ''
    result = dict(execute=args.execute, direction=direction, requested_value=value,
                  units='degrees' if direction in ('L', 'R') else 'mm', goals=goals, geometry=config)
    exercise = getattr(args, 'exercise', 'complete')
    result['exercise'] = exercise
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
                           'G 6000 6000'+' '*45+'BAD',
                           'G F 0 0', 'G B -6000 6000', 'G L 6000 10000',
                           'G R 6000 6000 junk', 'G Q 6000 6000', 'G F6000 6000')
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
                send(command)
                started = time.monotonic()
                last_keep = started
                end_marker = None
                cancelled = False
                while time.monotonic()-started < 6:
                    lines = read(.1)
                    end_marker = next((line for line in lines if line.startswith(('MOVE DONE', 'MOVE ABORT', 'MOVE CANCELLED'))), None)
                    if end_marker:
                        break
                    if any(line.startswith('MOVE ERROR') for line in lines):
                        raise RuntimeError('Firmware rejected move')
                    if exercise == 'cancel' and not cancelled and time.monotonic()-started >= .35:
                        send('x')
                        cancelled = True
                    if time.monotonic()-last_keep >= .25:
                        if exercise != 'watchdog' and not cancelled:
                            send('G KEEP')
                        send('E')
                        last_keep = time.monotonic()
                result['completion'] = end_marker
                assert end_marker is not None, 'No completion within host observation bound'
                expected = {'complete': 'MOVE DONE', 'cancel': 'MOVE CANCELLED',
                            'watchdog': 'MOVE ABORT WATCHDOG'}[exercise]
                assert end_marker.startswith(expected), f'Expected {expected}, received {end_marker}'
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
    parser.add_argument('--direction', choices=('F', 'B', 'L', 'R'), default='F')
    parser.add_argument('--angle-deg', type=float, default=30)
    parser.add_argument('--exercise', choices=('complete', 'cancel', 'watchdog'), default='complete')
    parser.add_argument('--execute', action='store_true')
    run(parser.parse_args())
