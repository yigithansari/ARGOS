"""ARGOS UART bridge and map export service for Raspberry Pi."""

import argparse
import json
import select
import signal
import socket
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "control_panel"))
from argos_exploration import replay, save_map_svg

CONTROL_PORT = 3333
MAP_PORT = 3336
MAX_LOG_BYTES = 16 * 1024 * 1024


def write_uart(uart, data):
    if uart.write(data) != len(data):
        raise OSError("Incomplete UART write")


def relay(client, uart, listener):
    """Forward one control session without buffering or replaying drive commands."""
    client.settimeout(0.25)
    uart.reset_input_buffer()
    pending = bytearray()
    try:
        write_uart(uart, b"x\r\nN 1\r\nE\r\n")
        while True:
            readable, _, _ = select.select([client, listener], [], [], 0.01)
            if listener in readable:
                extra, _ = listener.accept()
                extra.close()
            if client in readable:
                chunk = client.recv(4096)
                if not chunk:
                    return
                pending.extend(chunk)
                while b"\n" in pending:
                    end = pending.index(b"\n")
                    line = bytes(pending[:end]).removesuffix(b"\r")
                    del pending[:end + 1]
                    if not line:
                        continue
                    if len(line) > 39 or any(byte < 32 or byte > 126 for byte in line):
                        raise ValueError("Invalid STM32 command")
                    if line in (b"@ESP PING", b"@ESP LOGS"):
                        # The panel treats this as transport liveness. E probes STM32 separately.
                        client.sendall(b"ESP PONG | bridge:PI\r\n")
                    elif line.startswith(b"@ESP "):
                        continue
                    elif line.startswith(b"U "):
                        client.sendall(b"PI | STM32 OTA unavailable on this bridge\r\n")
                    else:
                        write_uart(uart, line + b"\r\n")
                if len(pending) > 40:
                    raise ValueError("Incomplete command exceeds STM32 line limit")
            data = uart.read(min(uart.in_waiting, 4096)) if uart.in_waiting else b""
            if data:
                client.sendall(data)
    finally:
        # Finish a potentially partial UART line before requesting STOP.
        write_uart(uart, b"\r\nx\r\nN 0\r\n")


class MapHandler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(5)
        super().setup()

    def do_POST(self):
        if self.path != "/map":
            self.send_error(404)
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
        except ValueError:
            self.send_error(400)
            return
        if not 0 < size <= MAX_LOG_BYTES:
            self.send_error(413)
            return
        try:
            data = self.rfile.read(size)
            if len(data) != size:
                self.send_error(400, "Incomplete log")
                return
            with tempfile.TemporaryDirectory() as directory:
                source = Path(directory) / "experiment.jsonl"
                target = source.with_suffix(".svg")
                source.write_bytes(data)
                summary = replay(source)
                if not summary["events"].get("session"):
                    self.send_error(400, "Missing ARGOS session")
                    return
                save_map_svg(summary, target)
                body = json.dumps({"summary": summary, "svg": target.read_text(encoding="utf-8")},
                                  allow_nan=False).encode("utf-8")
        except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError, RecursionError):
            self.send_error(422, "Cannot replay ARGOS log")
            return
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main():
    import serial

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--serial", default="/dev/ttyAMA0")
    args = parser.parse_args()

    def shutdown(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGHUP, shutdown)
    # SSH forwards these ports from the operator PC. Neither listens on the LAN.
    with HTTPServer(("127.0.0.1", MAP_PORT), MapHandler) as maps, socket.create_server(
            ("127.0.0.1", CONTROL_PORT), backlog=1) as listener:
        threading.Thread(target=maps.serve_forever, daemon=True).start()
        print(f"ARGOS Pi | {args.serial} | control {CONTROL_PORT} | map {MAP_PORT}", flush=True)
        try:
            while True:
                try:
                    with serial.Serial(args.serial, 115200, timeout=0, write_timeout=0.25,
                                       exclusive=True) as uart:
                        write_uart(uart, b"\r\nx\r\nN 0\r\n")
                        while True:
                            client, _ = listener.accept()
                            with client:
                                try:
                                    relay(client, uart, listener)
                                except (ConnectionError, TimeoutError, ValueError) as exc:
                                    print(f"Control session closed: {exc}", flush=True)
                except OSError as exc:
                    print(f"UART unavailable: {exc}", flush=True)
                    time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            maps.shutdown()


if __name__ == "__main__":
    main()
