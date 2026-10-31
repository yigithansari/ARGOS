"""Stationary OTA acceptance. Never sends a drive command; stops on first failure."""
import ast
import http.client
import json
from pathlib import Path
import socket
import threading
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).parent / 'ota_acceptance_20261030'
OUT.mkdir(exist_ok=True)
HOST = '192.168.137.178'
IMAGE = (ROOT / 'Debug/ARGOS_OTA.bin').read_bytes()
META = json.loads((ROOT / 'Debug/ARGOS_OTA_manifest.json').read_text())
CRC = zlib.crc32(IMAGE) & 0xffffffff
assert len(IMAGE) == META['application_bytes'] and f'{CRC:08X}' == META['application_crc32']
RESULTS = []

def result(name, passed, detail):
    entry = dict(test=name, passed=passed, detail=detail)
    RESULTS.append(entry)
    (OUT / 'results.json').write_text(json.dumps(RESULTS, indent=2), encoding='utf-8')
    print(json.dumps(entry), flush=True)
    if not passed:
        raise RuntimeError(f'{name}: {detail}')

class Link:
    def __enter__(self):
        self.sock = socket.create_connection((HOST, 3333), timeout=5)
        self.sock.settimeout(0.3)
        self.text = ''
        self.error = None
        self.done = False
        self.thread = threading.Thread(target=self.read, daemon=True)
        self.thread.start()
        return self

    def read(self):
        with (OUT / 'telemetry.log').open('a', encoding='utf-8') as log:
            while not self.done:
                try:
                    data = self.sock.recv(16384)
                    if not data:
                        self.error = 'TCP closed'
                        return
                    text = data.decode('utf-8', errors='replace')
                    self.text += text
                    log.write(text)
                    log.flush()
                except socket.timeout:
                    pass
                except OSError as exc:
                    if not self.done:
                        self.error = str(exc)
                    return

    def status(self, stop=False):
        if stop:
            self.sock.sendall(b'\r\nx\r\n')
            time.sleep(1)
        for attempt in range(3):
            start = len(self.text)
            self.sock.sendall(b'\r\nE\r\n')
            time.sleep(2)
            recent = self.text[start:]
            for line in recent.splitlines():
                if (line.startswith('MOTOR | LEFT:0 RPWM:0 LPWM:0 ')
                    and 'RIGHT:0 RPWM:0 LPWM:0 ' in line
                    and 'SAFE:STOP' in line and 'TARGET_RPMx100:+0/+0' in line):
                    return True
            if self.error:
                return False
        return False

    def __exit__(self, *args):
        self.done = True
        self.sock.close()
        self.thread.join(2)

def multipart(image):
    boundary = f'ARGOS-{time.time_ns()}'
    while boundary.encode() in image:
        boundary += 'X'
    body = (f'--{boundary}\r\n'
            'Content-Disposition: form-data; name="firmware"; filename="ARGOS_OTA.bin"\r\n'
            'Content-Type: application/octet-stream\r\n\r\n').encode()
    body += image + f'\r\n--{boundary}--\r\n'.encode()
    headers = {'Content-Type': f'multipart/form-data; boundary={boundary}',
               'X-ARGOS-Size': str(len(IMAGE)), 'X-ARGOS-CRC32': str(CRC),
               'X-ARGOS-Build': str(META['build_id']), 'X-ARGOS-Confirm': 'STOPPED'}
    return body, headers

# Execute the actual panel worker without constructing a GUI. Its HTTP request,
# response checks, and completion callback run unchanged.
tree = ast.parse((ROOT / 'Tools/argos_control_panel.py').read_text(encoding='utf-8-sig'))
worker = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
              and n.name == '_wireless_update_worker')
namespace = {'http': __import__('http'), 'time': time, 'WIFI_UPDATE_PORT': 3335}
exec(compile(ast.Module(body=[worker], type_ignores=[]), '<panel OTA worker>', 'exec'), namespace)

class PanelAdapter:
    def after(self, delay, callback):
        callback()

    def _finish_wireless_update(self, ok, detail):
        self.ok, self.detail = ok, detail

with Link() as link:
    time.sleep(3)
    result('initial_stop', link.status(stop=True), 'STOP and zero PWM/targets')

    corrupted = bytearray(IMAGE)
    corrupted[128] ^= 1
    body, headers = multipart(bytes(corrupted))
    start = len(link.text)
    conn = http.client.HTTPConnection(HOST, 3335, timeout=20)
    try:
        conn.request('POST', '/stm32', body=body, headers=headers)
        response = conn.getresponse()
        detail = response.read().decode(errors='replace')
        rejected = response.status == 400 and 'size/CRC mismatch' in detail
    finally:
        conn.close()
    stopped = link.status()
    no_boot = 'BOOTLOADER REQUEST' not in link.text[start:]
    result('corrupt_crc_rejected', rejected and stopped and no_boot, detail)

    body, headers = multipart(IMAGE)
    start = len(link.text)
    with socket.create_connection((HOST, 3335), timeout=5) as upload:
        header = f'POST /stm32 HTTP/1.1\r\nHost: {HOST}:3335\r\nContent-Length: {len(body)}\r\n'
        header += ''.join(f'{key}: {value}\r\n' for key, value in headers.items())
        upload.sendall(header.encode() + b'\r\n' + body[:len(body)//2])
    time.sleep(7)
    stopped = link.status()
    conn = http.client.HTTPConnection(HOST, 3335, timeout=15)
    try:
        conn.request('GET', '/status')
        response = conn.getresponse()
        detail = response.read().decode(errors='replace')
        online = response.status == 200 and 'STM:ONLINE' in detail
    finally:
        conn.close()
    result('partial_upload_disconnect', stopped and online and
           'BOOTLOADER REQUEST' not in link.text[start:],
           'TCP upload closed halfway; application responds, no bootloader request; ' + detail)

    for index in range(1, 11):
        if not link.status():
            result(f'ota_{index:02d}', False, 'Pre-update STOP not confirmed')
        start = len(link.text)
        adapter = PanelAdapter()
        namespace['_wireless_update_worker'](adapter, HOST, IMAGE, CRC, META['build_id'])
        time.sleep(8)
        events = link.text[start:]
        stopped = link.status()
        boot_ok = f'BL OK {CRC}' in events
        restarted = 'ARGOS test platform hazir.' in events
        result(f'ota_{index:02d}', adapter.ok and boot_ok and restarted and stopped,
               f'{adapter.detail}; boot_crc={boot_ok}; restarted={restarted}; stop={stopped}')
    print('ALL AUTOMATED TESTS PASSED', flush=True)
