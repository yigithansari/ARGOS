"""Operator-assisted Wi-Fi cut during incomplete cache upload; never completes image."""
import json
from pathlib import Path
import socket
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).with_name('wifi_cut_20261030.log')
HOST = '192.168.137.178'

def log(text):
    print(text, flush=True)
    with OUT.open('a', encoding='utf-8') as f:
        f.write(f'{time.time():.3f} {text}\n')

def read(sock, duration):
    end = time.monotonic() + duration
    data = b''
    sock.settimeout(0.2)
    while time.monotonic() < end:
        try:
            part = sock.recv(16384)
            if not part:
                break
            data += part
        except socket.timeout:
            pass
    return data.decode(errors='replace')

image = (ROOT / 'Debug/ARGOS_OTA.bin').read_bytes()
meta = json.loads((ROOT / 'Debug/ARGOS_OTA_manifest.json').read_text())
crc = zlib.crc32(image) & 0xffffffff
assert f'{crc:08X}' == meta['application_crc32']
with socket.create_connection((HOST, 3333), timeout=5) as control:
    read(control, 2)
    control.sendall(b'\r\nx\r\n')
    read(control, 1)
    control.sendall(b'\r\nE\r\n')
    status = read(control, 2)
    log(status)
    assert 'SAFE:STOP' in status and 'TARGET_RPMx100:+0/+0' in status
    boundary = f'ARGOS-{time.time_ns()}'
    prefix = (f'--{boundary}\r\n'
              'Content-Disposition: form-data; name="firmware"; filename="ARGOS_OTA.bin"\r\n'
              'Content-Type: application/octet-stream\r\n\r\n').encode()
    suffix = f'\r\n--{boundary}--\r\n'.encode()
    headers = (f'POST /stm32 HTTP/1.1\r\nHost: {HOST}:3335\r\n'
               f'Content-Type: multipart/form-data; boundary={boundary}\r\n'
               f'Content-Length: {len(prefix)+len(image)+len(suffix)}\r\n'
               f'X-ARGOS-Size: {len(image)}\r\nX-ARGOS-CRC32: {crc}\r\n'
               f'X-ARGOS-Build: {meta["build_id"]}\r\nX-ARGOS-Confirm: STOPPED\r\n\r\n').encode()
    with socket.create_connection((HOST, 3335), timeout=3) as upload:
        upload.sendall(headers + prefix)
        log('CACHE UPLOAD ACTIVE: operator may disable hotspot now; robot power stays ON')
        sent = 0
        try:
            for offset in range(0, len(image)//2, 128):
                chunk = image[offset:min(offset+128, len(image)//2)]
                upload.sendall(chunk)
                sent += len(chunk)
                time.sleep(0.5)
        except OSError as exc:
            log(f'UPLOAD NETWORK ERROR after {sent} bytes: {exc}')
        else:
            log(f'Partial upload limit reached: {sent} bytes queued; operator outage still requires confirmation')
    log('PARTIAL UPLOAD CLOSED; no final boundary sent; full image cannot be programmed')
log('Restore hotspot; reconnect and application checks required before PASS')
