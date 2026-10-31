"""Stationary UART startup regression via three OTA restarts, no drive commands."""
import ast
import http.client
import json
from pathlib import Path
import socket
import threading
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent / 'uart_startup_after'
OUT.mkdir(exist_ok=True)
HOST = '192.168.137.226'
IMAGE = (ROOT / 'Debug/ARGOS_OTA.bin').read_bytes()
META = json.loads((ROOT / 'Debug/ARGOS_OTA_manifest.json').read_text())
CRC = zlib.crc32(IMAGE) & 0xffffffff
assert f'{CRC:08X}' == META['application_crc32']
helpers = ast.parse(Path(__file__).with_name('ota_acceptance.py').read_text())
definitions = [n for n in helpers.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
               and n.name in ('Link', 'PanelAdapter')]
exec(compile(ast.Module(body=definitions, type_ignores=[]), '<test helpers>', 'exec'))
tree = ast.parse((ROOT / 'Tools/argos_control_panel.py').read_text(encoding='utf-8-sig'))
worker = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_wireless_update_worker')
WIFI_UPDATE_PORT = 3335
exec(compile(ast.Module(body=[worker], type_ignores=[]), '<panel worker>', 'exec'))
results = []

with Link() as link:
    time.sleep(3)
    assert link.status(stop=True), 'STOP not confirmed'
    for cycle in range(1, 4):
        start = len(link.text)
        adapter = PanelAdapter()
        _wireless_update_worker(adapter, HOST, IMAGE, CRC, META['build_id'])
        assert adapter.ok, adapter.detail
        # Query during startup instead of hiding it behind the previous 8s wait.
        for query in range(12):
            link.sock.sendall(b'E\r\n')
            time.sleep(0.5)
        assert link.status(), 'Post-update STOP not confirmed'
        captured = link.text[start:]
        motor_lines = [line for line in captured.splitlines() if line.startswith('MOTOR |')]
        malformed = [line for line in motor_lines if not (
            line.startswith('MOTOR | LEFT:0 RPWM:0 LPWM:0 ') and
            'RIGHT:0 RPWM:0 LPWM:0 ' in line and 'SAFE:STOP' in line and
            'TARGET_RPMx100:+0/+0' in line and 'I_x100:+0/+0' in line)]
        item = dict(cycle=cycle, crc=f'{CRC:08X}', update=adapter.detail,
                    unknown_commands=captured.count('Bilinmeyen komut'),
                    echo_we=captured.count('> WE'), motor_lines=len(motor_lines),
                    malformed_motor_lines=len(malformed),
                    restarted='ARGOS test platform hazir.' in captured)
        results.append(item)
        (OUT / f'cycle_{cycle}.log').write_text(captured, encoding='utf-8')
        (OUT / 'results.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
        print(json.dumps(item), flush=True)
