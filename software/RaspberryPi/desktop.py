"""ARGOS desktop panel for the Raspberry Pi bridge."""

import argparse
import http.client
import json
import queue
import re
import subprocess
import sys
import threading
import tkinter as tk
from pathlib import Path
from tkinter import ttk

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "control_panel"))
from argos_control_panel import ArgosControlPanel


class SshTunnel:
    def __init__(self, target):
        self.target = target
        self.process = None

    def start(self):
        if self.process is None or self.process.poll() is not None:
            self.process = subprocess.Popen([
                "ssh", "-N", "-T", "-o", "BatchMode=yes", "-o", "ExitOnForwardFailure=yes",
                "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=5",
                "-o", "ServerAliveInterval=3", "-o", "ServerAliveCountMax=2",
                "-L", "127.0.0.1:43333:127.0.0.1:3333",
                "-L", "127.0.0.1:43336:127.0.0.1:3336", self.target,
            ], stdin=subprocess.DEVNULL)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=1)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()


class PiPanel(ArgosControlPanel):
    def __init__(self, target=None):
        self.tunnel = SshTunnel(target) if target else None
        super().__init__()
        self.title("ARGOS | Raspberry Pi")
        self.wifi_host_var.set("127.0.0.1")
        self.wifi_port_var.set("43333" if self.tunnel else "3333")
        self.map_port = 43336 if self.tunnel else 3336
        self.map_results = queue.Queue()
        self.map_pending = False

    def connect_serial(self, quiet=False):
        if self.tunnel and self.link_mode_var.get() == "Wi-Fi":
            try:
                self.tunnel.start()
            except OSError as exc:
                self._log(f"SSH start failed: {exc}\n", "error")
                return False
        return super().connect_serial(quiet=quiet)

    def on_close(self):
        try:
            super().on_close()
        finally:
            if self.tunnel:
                self.tunnel.close()

    def _build_sidebar(self, parent):
        super()._build_sidebar(parent)
        self._action_button(parent, "DISTANCE / TURN", self.open_motion, "#43b5ff").pack(
            fill="x", padx=16, pady=(0, 8))

    def _action_button(self, parent, text, command, color):
        button = super()._action_button(parent, text, command, color)
        if text == "STM32 WIRELESS UPDATE":
            button.configure(text="STM32 OTA VIA ESP", state="disabled")
        return button

    def _mark_robot_online(self):
        was_ready = self.robot_ready
        super()._mark_robot_online()
        if not was_ready:
            self.status_var.set("ARGOS ONLINE - PI + STM32")

    def open_motion(self):
        window = tk.Toplevel(self)
        window.title("ARGOS | Distance / Turn")
        window.transient(self)
        window.configure(bg="#0a141f")
        direction = tk.StringVar(window, "F")
        value = tk.StringVar(window, "100")
        ttk.Label(window, text="F/B 30–150 mm · L/R 15–90°").pack(padx=18, pady=(16, 8))
        ttk.Combobox(window, textvariable=direction, values=("F", "B", "L", "R"),
                     state="readonly", width=8).pack(pady=4)
        ttk.Entry(window, textvariable=value, width=12).pack(pady=4)
        ttk.Button(window, text="START",
                   command=lambda: self.start_local_move(direction.get(), value.get())).pack(pady=4)
        ttk.Button(window, text="STOP", command=self.stop_drive).pack(pady=4)
        ttk.Label(window, textvariable=self.motion_status_var, wraplength=360).pack(padx=18, pady=12)

    def _export_experiment(self):
        self._stop_experiment()
        path = self.experiment_last_path
        if path is None or self.map_pending:
            return
        self.map_pending = True
        self.experiment_status.set("Exporting map on Raspberry Pi…")
        threading.Thread(target=self._fetch_map, args=(path,), daemon=True).start()
        self.after(100, self._poll_map)

    def _fetch_map(self, path):
        connection = http.client.HTTPConnection("127.0.0.1", self.map_port, timeout=30)
        try:
            if path.stat().st_size > 16 * 1024 * 1024:
                raise ValueError("Log exceeds 16 MiB")
            connection.request("POST", "/map", body=path.read_bytes(),
                               headers={"Content-Type": "application/x-ndjson"})
            response = connection.getresponse()
            if response.status != 200:
                raise ValueError(f"Pi returned HTTP {response.status}")
            output = json.load(response)
            path.with_suffix(".summary.json").write_text(
                json.dumps(output["summary"], indent=2, ensure_ascii=False), encoding="utf-8")
            map_path = path.with_suffix(".map.svg")
            map_path.write_text(output["svg"], encoding="utf-8")
            status = f"Map saved: {map_path}"
        except (OSError, http.client.HTTPException, ValueError, KeyError, TypeError) as exc:
            status = f"Pi map export failed: {exc}"
        finally:
            connection.close()
        self.map_results.put(status)

    def _poll_map(self):
        try:
            status = self.map_results.get_nowait()
        except queue.Empty:
            self.after(100, self._poll_map)
        else:
            self.map_pending = False
            self.experiment_status.set(status)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh", metavar="USER@HOST", help="Open the Pi tunnel automatically")
    args = parser.parse_args()
    if args.ssh and not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*@[A-Za-z0-9][A-Za-z0-9.-]*", args.ssh):
        parser.error("--ssh requires USER@HOST (IPv4 address or hostname)")
    app = PiPanel(args.ssh)
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
