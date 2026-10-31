"""Capture five new TCP sessions; only E status queries, never motor movement."""
import ast
import json
from pathlib import Path
import socket
import threading
import time

OUT = Path(__file__).parent / 'uart_reconnect'
OUT.mkdir(exist_ok=True)
HOST = '192.168.137.226'
tree = ast.parse(Path(__file__).with_name('ota_acceptance.py').read_text())
node = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Link')
exec(compile(ast.Module(body=[node], type_ignores=[]), '<Link>', 'exec'))
results = []
for index in range(1, 6):
    with Link() as link:
        time.sleep(3)
        initial = link.text
        clean_initial = any(line.startswith('MOTOR | LEFT:0 RPWM:0 LPWM:0 ')
                            and 'RIGHT:0 RPWM:0 LPWM:0 ' in line
                            and 'SAFE:STOP' in line and 'I_x100:+0/+0' in line
                            for line in initial.splitlines())
        stopped = link.status()
        (OUT / f'connection_{index}.log').write_text(link.text, encoding='utf-8')
        result = dict(connection=index, clean_initial_motor=clean_initial,
                      later_stop=stopped, unknown_commands=link.text.count('Bilinmeyen komut'))
        results.append(result)
        print(json.dumps(result), flush=True)
        assert stopped, 'No STOP confirmation'
    time.sleep(1)
(OUT / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
