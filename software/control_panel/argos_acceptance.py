"""Operator-run Phase 02 acceptance with durable logs and external measurements."""
import json
import math
import re
import time
import tkinter as tk
from datetime import datetime, timezone
from pathlib import Path
from tkinter import messagebox, ttk

CASES = ("PI +25", "PI +35", "PI +45", "PI -25", "Straight 1000 mm",
         "Left 90 deg", "Right 90 deg", "Watchdog")
MOTOR = re.compile(r"^MOTOR \| LEFT:(-?\d+) RPWM:(\d+) LPWM:(\d+) ENC:(-?\d+).*?"
                   r"RIGHT:(-?\d+) RPWM:(\d+) LPWM:(\d+) ENC:(-?\d+).*?"
                   r"SAFE:(\w+).*?MEAS_RPMx100:([+-]?\d+)/([+-]?\d+).*?T_MS:(\d+)")

def motor_frame(line):
    match = MOTOR.search(line)
    if not match:
        return None
    values = match.groups()
    return dict(pwm=[int(values[i]) for i in (1, 2, 5, 6)],
                counts=[int(values[i]) for i in (3, 7)], safe=values[8],
                rpm=[int(values[i])/100 for i in (9, 10)], tick=int(values[11]))

def travel(origin, current, calibration):
    circumference = math.pi * calibration["wheel_diameter_mm"]
    return [((current[i]-origin[i]+2**31) % 2**32-2**31) *
            calibration[f"{side}_forward_sign"] * circumference /
            calibration[f"{side}_counts_per_wheel_rev"]
            for i, side in enumerate(("left", "right"))]

def straight_command(distances, calibration):
    counts_per_mm = calibration["left_counts_per_wheel_rev"] / (math.pi * calibration["wheel_diameter_mm"])
    trim = max(-2, min(2, round((distances[0]-distances[1]) * counts_per_mm / 1000)))
    return f"V {25-trim} {25+trim}"

def external_result(case, left, right):
    if not all(math.isfinite(v) for v in (left, right)):
        raise ValueError("Measurements must be finite.")
    if case == "Straight 1000 mm":
        if min(left, right) <= 0:
            raise ValueError("Enter positive wheel-center travel in mm.")
        midpoint = (left+right)/2
        return dict(left_mm=left, right_mm=right, midpoint_mm=midpoint,
                    external_pass=abs(midpoint-1000) <= 50)
    if not 0 < left <= 180:
        raise ValueError("Enter the measured turn magnitude in degrees.")
    return dict(angle_deg=left, external_pass=abs(left-90) <= 5)

class Acceptance:
    def __init__(self, app, planner):
        self.app, self.planner = app, planner
        self.running = self.sending = False
        self.pending = None
        self.window = tk.Toplevel(app)
        self.window.title("ARGOS | Phase 02 Acceptance")
        self.case = tk.StringVar(self.window, CASES[0])
        self.ready = tk.BooleanVar(self.window, False)
        self.left = tk.StringVar(self.window)
        self.right = tk.StringVar(self.window)
        self.status = tk.StringVar(self.window, "Choose a test. Each START runs once.")
        self.instruction = tk.StringVar(self.window)
        self.directory = Path(__file__).with_name("phase02_results")
        ttk.Combobox(self.window, textvariable=self.case, values=CASES,
                     state="readonly", width=32).pack(padx=14, pady=10)
        self.case.trace_add("write", self.describe)
        ttk.Label(self.window, textvariable=self.instruction, wraplength=600).pack(padx=14, pady=5)
        ttk.Checkbutton(self.window, text="Setup matches the instructions; power switch is within reach",
                        variable=self.ready).pack(padx=14, pady=5)
        ttk.Button(self.window, text="START ONE RUN", command=self.start).pack(pady=5)
        ttk.Button(self.window, text="STOP", command=self.cancel).pack(pady=5)
        ttk.Label(self.window, text="External measurement: straight L/R in mm; turn angle in left field").pack(padx=14)
        ttk.Entry(self.window, textvariable=self.left).pack(pady=3)
        ttk.Entry(self.window, textvariable=self.right).pack(pady=3)
        ttk.Button(self.window, text="SAVE MEASUREMENT", command=self.measure).pack(pady=5)
        ttk.Label(self.window, textvariable=self.status, wraplength=600).pack(padx=14, pady=10)
        self.window.protocol("WM_DELETE_WINDOW", self.close)
        ttk.Label(self.window, text=f"Logs: {self.directory}", wraplength=600).pack(padx=14, pady=5)
        self.describe()

    def describe(self, *_):
        self.ready.set(False)
        case = self.case.get()
        if case.startswith("PI"):
            text = "Wheels secured fully off ground. Three 20 s runs at each +25/+35/+45/-25 RPM. Evaluation uses final 4 s, mean and all samples within +/-5%."
        elif case == "Watchdog":
            text = "Wheels secured fully off ground. Three runs: a single +25 RPM command, then only status queries. Log brackets output cutoff using STM timestamps. Physical cutoff measurement remains required if the bracket crosses 1200 ms."
        elif case.startswith("Straight"):
            text = "Floor: 1.5 m clear lane; distance-sensor guard OFF for this test. Align both wheel centers to the same start line. Three 1000 mm runs, reposition by hand each time. Measure both wheel centers from the start line. Midpoint target 950-1050 mm."
        else:
            text = "Floor: clear the full swept area; distance-sensor guard OFF for this test. Mark chassis orientation. Three 90 degree runs per direction. Measure actual chassis angle with an external square/protractor. Target 85-95 degrees."
        self.instruction.set(text)

    def write(self, event, **data):
        self.log.write(json.dumps(dict(utc=datetime.now(timezone.utc).isoformat(),
                                      monotonic=time.monotonic(), event=event, **data),
                                  allow_nan=False)+"\n")
        self.log.flush()

    def send(self, command):
        try:
            self.write("tx", command=command)
        except OSError:
            if command != "x":
                raise
        self.sending = True
        try:
            if not self.app.send_command(command):
                raise RuntimeError("Command rejected or link unavailable")
        finally:
            self.sending = False

    def start(self):
        if self.running or self.pending:
            self.status.set("Finish the current run and save its external measurement first.")
            return
        app = self.app
        blockers = [
            (not self.ready.get(), "Confirm the setup checkbox."),
            (not app.robot_ready or app.serial_port is None or not app.serial_port.is_open,
             "Connect to STM32 and wait for telemetry."),
            (app.link_mode_var.get() != "Wi-Fi", "Select Wi-Fi for the ESP test."),
            (app.motion_active or app.drive_active, "Press STOP to finish active drive."),
            (app.explore_session, "Stop exploration first."),
            (app.system_test_active or app.lowband_test_active, "Finish the active system/RPM test."),
            (app.firmware_update_active, "Wait for firmware update completion."),
        ]
        for blocked, detail in blockers:
            if blocked:
                self.status.set(detail)
                return
        try:
            self.calibration = dict(app.motion_calibration)
            self.planner(self.calibration, "F", 100)
            self.directory.mkdir(exist_ok=True)
            self.path = self.directory / datetime.now().strftime("run_%Y%m%d_%H%M%S_%f")
            self.log = self.path.with_suffix(".jsonl").open("x", encoding="utf-8")
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self.status.set(str(exc))
            return
        self.test = self.case.get()
        self.result = dict(case=self.test, calibration=self.calibration, stop_confirmed=False,
                           range_guard=False, operator_setup_confirmed=True,
                           control_method="host RPM and encoder cutoff" if self.test.startswith("Straight")
                           else "existing manual STM32 command")
        self.samples = []
        self.origin = self.latest = self.last_tick = self.trigger_tick = None
        self.watchdog_last = self.watchdog_first = None
        self.phase = "prepare"
        self.started = self.feedback = self.sent_at = time.monotonic()
        self.running = True
        self.left.set("")
        self.right.set("")
        self.status.set(f"Preparing {self.test}: requesting fresh STOP confirmation")
        try:
            self.write("start", case=self.test, calibration=self.calibration, link=app.connection_label)
            self.send("x")
            self.send("E")
        except (OSError, RuntimeError) as exc:
            self.halt(str(exc))
        self.tick()

    def handle(self, text):
        if not self.running:
            return
        try:
            self.write("rx", text=text)
            for line in text.splitlines():
                frame = motor_frame(line)
                if frame:
                    if self.last_tick is not None and ((frame["tick"]-self.last_tick)&0xffffffff) > 0x7fffffff:
                        self.halt("STM32 reboot")
                        continue
                    self.last_tick = frame["tick"]
                    self.latest = frame
                    self.feedback = time.monotonic()
                    if self.phase == "prepare" and frame["safe"] == "STOP" and not any(frame["pwm"]):
                        self.launch(frame)
                    elif self.phase == "run":
                        self.observe(frame)
                    elif self.phase == "stopping" and frame["safe"] == "STOP" and not any(frame["pwm"]):
                        self.finish(True)
                if self.phase == "run":
                    if line.startswith("MOVE DONE |"):
                        self.halt("turn completed" if "90 deg" in self.test else "unexpected completion")
                    elif line.startswith(("MOVE ABORT", "MOVE ERROR", "MOVE IDLE", "DRIVE BLOCKED")):
                        self.halt(line)
        except (OSError, RuntimeError, ValueError) as exc:
            self.halt(str(exc))

    def launch(self, frame):
        self.origin = frame["counts"]
        self.trigger_tick = None
        self.phase = "run"
        self.started = self.feedback = time.monotonic()
        self.status.set(f"RUNNING {self.test}")
        if self.test.startswith("PI"):
            rpm = int(self.test.split()[1])
            self.send(f"V {rpm} {rpm}")
        elif self.test == "Watchdog":
            self.send("V 25 25")
        elif self.test.startswith("Straight"):
            self.send("V 25 25")
        else:
            self.send(self.planner(self.calibration, "L" if self.test.startswith("Left") else "R", 90))
        self.sent_at = time.monotonic()

    def observe(self, frame):
        if self.trigger_tick is None:
            if frame["safe"] != "RUN":
                return
            self.trigger_tick = frame["tick"]
        elapsed = ((frame["tick"]-self.trigger_tick)&0xffffffff)/1000
        self.samples.append(dict(elapsed=elapsed, **frame))
        speeds = frame["rpm"]
        if max(map(abs, speeds)) > 60:
            self.halt("excess wheel speed")
            return
        if self.test == "Watchdog":
            if frame["safe"] == "RUN" and any(frame["pwm"]):
                self.watchdog_last = elapsed*1000
            if frame["safe"] == "TIMEOUT_STOP" and not any(frame["pwm"]):
                self.watchdog_first = elapsed*1000
                self.halt("watchdog completed")
        elif frame["safe"] not in ("RUN",):
            self.halt("unexpected motor stop")
        elif self.test.startswith("PI"):
            sign = 1 if int(self.test.split()[1]) > 0 else -1
            if min(v*sign for v in speeds) < -2 or (elapsed > 3 and min(v*sign for v in speeds) < 5):
                self.halt("stall or wrong wheel direction")
            elif elapsed >= 20:
                self.halt("speed completed")
        elif self.test.startswith("Straight"):
            distances = travel(self.origin, frame["counts"], self.calibration)
            if min(distances) < -15 or abs(distances[0]-distances[1]) > 65:
                self.halt("wheel divergence")
            elif sum(distances)/2 >= 978:
                self.halt("distance cutoff")

    def halt(self, reason):
        if not self.running or self.phase == "stopping":
            return
        self.result["reason"] = reason
        self.phase = "stopping"
        self.stopping = time.monotonic()
        if self.app.serial_port is None or not self.app.serial_port.is_open:
            self.finish(False)
            return
        try:
            self.send("x")
        except (OSError, RuntimeError):
            self.finish(False)

    def cancel(self):
        self.halt("operator STOP")

    def tick(self):
        if not self.running:
            return
        now = time.monotonic()
        try:
            if self.app.serial_port is None or not self.app.serial_port.is_open or not self.app.robot_ready:
                self.halt("link lost")
            elif self.phase == "stopping" and now-self.stopping > 1.5:
                self.finish(False)
            elif self.phase == "prepare" and now-self.started > 2:
                self.halt("initial STOP feedback timeout")
            elif self.phase == "run":
                limit = 24 if self.test.startswith("PI") else 12 if self.test.startswith("Straight") else 6
                if now-self.feedback > 0.8 or now-self.started > limit:
                    self.halt("feedback or duration timeout")
            if self.running and now-self.sent_at >= 0.2:
                command = "E"
                if self.phase == "run" and self.test.startswith("PI"):
                    rpm = int(self.test.split()[1])
                    command = f"V {rpm} {rpm}"
                elif self.phase == "run" and self.test.startswith("Straight"):
                    distances = travel(self.origin, self.latest["counts"], self.calibration)
                    command = straight_command(distances, self.calibration)
                elif self.phase == "run" and self.test != "Watchdog":
                    command = "G KEEP"
                self.send(command)
                self.sent_at = now
        except (OSError, RuntimeError) as exc:
            self.halt(str(exc))
        if self.running:
            self.app.after(50, self.tick)

    def finish(self, stopped):
        self.running = False
        self.result["stop_confirmed"] = stopped
        if self.origin and self.latest:
            self.result["encoder_mm"] = travel(self.origin, self.latest["counts"], self.calibration)
        success = stopped and self.result.get("reason") in (
            "speed completed", "turn completed", "distance cutoff", "watchdog completed")
        self.result["execution_pass"] = success
        if self.test.startswith("PI"):
            target = int(self.test.split()[1])
            tail = [s for s in self.samples if 16 <= s["elapsed"] <= 20.5]
            sufficient = len(tail) >= 8 and tail[-1]["elapsed"]-tail[0]["elapsed"] >= 3
            self.result["settled_samples"] = len(tail)
            self.result["sufficient_settled_data"] = sufficient
            means = [sum(s["rpm"][i] for s in tail)/len(tail) for i in (0, 1)] if tail else []
            self.result["mean_rpm"] = means
            self.result["mean_pass"] = success and sufficient and all(abs(v-target)<=abs(target)*.05 for v in means)
            self.result["in_band_fraction"] = [sum(abs(s["rpm"][i]-target)<=abs(target)*.05 for s in tail)/len(tail) for i in (0,1)] if tail else []
            self.result["tracking_pass"] = success and sufficient and all(v==1 for v in self.result["in_band_fraction"])
        if self.test == "Watchdog":
            self.result.update(last_active_ms=self.watchdog_last, first_timeout_ms=self.watchdog_first)
            self.result["timing_status"] = ("PASS" if success and self.watchdog_first is not None and self.watchdog_first<=1200 else "MEASUREMENT REQUIRED")
        try:
            self.write("finished", result=self.result)
        except OSError:
            pass
        finally:
            self.log.close()
        try:
            self.save()
        except OSError as exc:
            self.status.set(f"Result write failed: {exc}. STOP confirmed: {stopped}")
            return
        if success and ("mm" in self.test or "deg" in self.test):
            self.pending = self.path
            self.status.set("STOP confirmed. Enter the external measurement and SAVE MEASUREMENT.")
        else:
            detail = f"Mean PASS {self.result.get('mean_pass')} | tracking PASS {self.result.get('tracking_pass')}" if self.test.startswith("PI") else self.result.get("timing_status", self.result.get("reason"))
            self.status.set(f"{detail} | STOP {stopped} | saved {self.path.name}")

    def save(self):
        temporary = self.path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(self.result, indent=2), encoding="utf-8")
        temporary.replace(self.path.with_suffix(".json"))

    def measure(self):
        if not self.pending:
            return
        try:
            left = float(self.left.get().replace(",", "."))
            right = float(self.right.get().replace(",", ".")) if self.test.startswith("Straight") else left
            measured = external_result(self.test, left, right)
            self.result.update(measured)
            self.save()
        except (OSError, ValueError) as exc:
            messagebox.showerror("Measurement", str(exc))
            return
        self.pending = None
        self.status.set(f"External {'PASS' if measured['external_pass'] else 'FAIL'} saved. Reposition before the next START.")

    def close(self):
        if self.running:
            self.cancel()
            return
        if self.pending:
            self.status.set("Save the external measurement before closing.")
            return
        self.window.destroy()
