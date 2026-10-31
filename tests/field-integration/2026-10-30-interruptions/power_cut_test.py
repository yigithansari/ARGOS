"""Operator-coordinated power interruption or original-image recovery. No drive commands."""
import ast
import http.client
import json
from pathlib import Path
import socket
import sys
import threading
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent / 'power_cut_20261030'
OUT.mkdir(exist_ok=True)
HOST = sys.argv[2] if len(sys.argv) > 2 else '192.168.137.226'
META = json.loads((ROOT / 'Debug/ARGOS_OTA_manifest.json').read_text())
original = (ROOT / 'Debug/ARGOS_OTA.bin').read_bytes()
assert f'{zlib.crc32(original):08X}' == META['application_crc32']
mode = sys.argv[1]
assert mode in ('cut', 'recover')
# Padding extends the operator's interruption window without changing application code.
IMAGE = original.ljust(480 * 1024, b'\xff') if mode == 'cut' else original
CRC = zlib.crc32(IMAGE) & 0xffffffff
helpers = ast.parse(Path(__file__).with_name('ota_acceptance.py').read_text())
selected = [n for n in helpers.body if isinstance(n, (ast.ClassDef, ast.FunctionDef))
            and n.name in ('Link', 'multipart')]
exec(compile(ast.Module(body=selected, type_ignores=[]), '<acceptance helpers>', 'exec'))

def report(text):
    print(text, flush=True)
    with (OUT / 'actions.log').open('a', encoding='utf-8') as f:
        f.write(f'{time.time():.3f} {text}\n')

outcome = {}
def upload():
    connection = http.client.HTTPConnection(HOST, 3335, timeout=150)
    try:
        body, headers = multipart(IMAGE)
        connection.request('POST', '/stm32', body=body, headers=headers)
        response = connection.getresponse()
        detail = response.read().decode(errors='replace')
        outcome.update(status=response.status, detail=detail)
    except Exception as exc:
        outcome.update(error=str(exc))
    finally:
        connection.close()

report(f'BEGIN mode={mode} bytes={len(IMAGE)} crc={CRC:08X}')
with Link() as link:
    time.sleep(3)
    report('INITIAL LINK\n' + link.text)
    if mode == 'cut' and not link.status(stop=True):
        raise RuntimeError('STOP not confirmed; no update attempted')
    worker = threading.Thread(target=upload, daemon=True)
    worker.start()
    start = time.monotonic()
    erase_announced = False
    finish_announced = False
    while worker.is_alive() and time.monotonic() - start < 160:
        if 'BL ERASED' in link.text and not erase_announced:
            erase_announced = True
            report('FLASH ERASED - PROGRAMMING ACTIVE - CUT ROBOT POWER NOW' if mode == 'cut'
                   else 'RECOVERY PROGRAMMING ACTIVE')
        if f'BL OK {CRC}' in link.text and not finish_announced:
            finish_announced = True
            report('FLASH COMPLETE - TOO LATE TO CUT POWER' if mode == 'cut' else 'RECOVERY CRC VERIFIED')
        if link.error:
            report('CONTROL LINK LOST: ' + link.error)
            break
        time.sleep(0.1)
    if mode == 'cut' and link.error:
        report('Await operator power restoration; separate recovery step required')
    else:
        worker.join(5)
        report('HTTP RESULT ' + json.dumps(outcome))
        if mode == 'recover':
            time.sleep(8)
            stopped = link.status()
            passed = (outcome.get('status') == 200 and
                      outcome.get('detail', '').startswith('UPDATE PASS') and
                      f'BL OK {CRC}' in link.text and stopped)
            report(f'RECOVERY PASS={passed}; STOP={stopped}')
            if not passed:
                raise RuntimeError('Recovery not confirmed; preserve logs')
    report(f'END mode={mode}; erase_seen={erase_announced}; completion_seen={finish_announced}')
