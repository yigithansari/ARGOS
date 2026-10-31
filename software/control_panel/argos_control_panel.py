import json
import http.client
import math
import os
import queue
import re
import socket
import threading
import time
import tkinter as tk
import zlib
from collections import deque
from datetime import datetime
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 115200
READ_POLL_SECONDS = 0.03
WIFI_HOST = "192.168.4.1"
WIFI_PORT = 3333
DISCOVERY_PORT = 3334
WIFI_UPDATE_PORT = 3335
WINDOWS_HOTSPOT_IP = "192.168.137.1"
DEFAULT_WHEEL_CIRCUMFERENCE_MM = 260.0
DEFAULT_TRACK_WIDTH_MM = 198.0
ENCODER_CALIBRATION_TURNS = 10
FIRMWARE_ENCODER_COUNTS_PER_REV = 1440.0
LOWBAND_PWM_LEVELS = (50, 52, 54, 56, 58, 60)
LOWBAND_SETTLE_MS = 1400
LOWBAND_MEASURE_MS = 1600
LOWBAND_STOP_MS = 500


def validate_ota_image(image):
    if not 8 <= len(image) <= 480 * 1024:
        raise ValueError("Select an OTA application image between 8 bytes and 480 KiB.")
    stack = int.from_bytes(image[:4], "little")
    reset = int.from_bytes(image[4:8], "little")
    if not 0x20000000 <= stack <= 0x20020000:
        raise ValueError("The image has an invalid STM32 initial stack pointer.")
    if not (reset & 1) or not 0x08008000 <= (reset & ~1) < 0x08008000 + len(image):
        raise ValueError(
            "Select ARGOS_OTA.bin linked at 0x08008000. "
            "Bootstrap and normal application images cannot be uploaded here."
        )


class TcpLineTransport:
    def __init__(self, host, port):
        self.socket = socket.create_connection((host, port), timeout=0.8)
        self.socket.settimeout(0.08)
        self.buffer = bytearray()
        self.is_open = True

    def write(self, payload):
        self.socket.sendall(payload)
        return len(payload)

    def readline(self):
        newline = self.buffer.find(b"\n")
        if newline >= 0:
            line = bytes(self.buffer[:newline + 1])
            del self.buffer[:newline + 1]
            return line
        try:
            chunk = self.socket.recv(2048)
        except socket.timeout:
            return b""
        if not chunk:
            raise ConnectionError("ESP Wi-Fi baglantisi kapandi")
        self.buffer.extend(chunk)
        newline = self.buffer.find(b"\n")
        if newline < 0:
            return b""
        line = bytes(self.buffer[:newline + 1])
        del self.buffer[:newline + 1]
        return line

    def close(self):
        self.is_open = False
        try:
            self.socket.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        self.socket.close()


def discover_argos(timeout=0.6):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.settimeout(timeout)
        sock.bind(("", 0))
        for broadcast in ("255.255.255.255", "192.168.137.255", "192.168.4.255"):
            sock.sendto(b"ARGOS_DISCOVER", (broadcast, DISCOVERY_PORT))
        payload, address = sock.recvfrom(256)
        fields = payload.decode("ascii", errors="ignore").strip().split("|")
        if fields and fields[0] == "ARGOS_BRIDGE":
            port = int(fields[1]) if len(fields) > 1 else WIFI_PORT
            return address[0], port
    except (OSError, ValueError):
        return None
    finally:
        sock.close()
    return None


class ArgosControlPanel(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("ARGOS Robot Control Platform")
        self.geometry("1280x820")
        self.minsize(1040, 680)
        self.configure(bg="#071018")
        try:
            self.state("zoomed")
        except tk.TclError:
            pass

        self.serial_port = None
        self.reader_thread = None
        self.reader_alive = threading.Event()
        self.main_canvas = None
        self.main_window = None
        self.sidebar_canvas = None
        self.sidebar_window = None
        self.active_scroll_canvas = None
        self.rx_queue = queue.Queue()
        self.tx_count = 0
        self.rx_count = 0
        self.imu_samples = deque(maxlen=160)
        self.tof_samples = deque(maxlen=80)
        self.console_autoscroll = True
        self.system_test_active = False
        self.system_test_finishing = False
        self.system_test_timeout_job = None
        self.system_test_lines = []
        self.system_test_started_at = None
        self.system_test_report_path = Path(__file__).with_name("argos_system_test_report.txt")
        self.firmware_update_active = False
        self.lowband_report_path = Path(__file__).with_name("argos_lowband_rpm_report.txt")
        self.connection_logbook_path = Path(__file__).with_name("argos_connection_logbook.jsonl")
        self.motion_calibration_path = Path(__file__).with_name("argos_motion_calibration.json")
        self.motion_calibration = self._load_motion_calibration()
        self.encoder_calibration_zero = None
        self.encoder_calibration_measurements = {}
        self.phase02_window = None
        self.phase02_vars = {}
        self.lowband_test_active = False
        self.lowband_test_steps = []
        self.lowband_test_rows = []
        self.lowband_test_index = 0
        self.lowband_test_started_at = None
        self.lowband_current_step = None
        self.lowband_start_snapshot = None
        self.system_test_button = None
        self.motor_sample_seq = 0
        self.esp_last_event_id = self._load_last_esp_event_id()
        self.last_bridge_rx_time = time.monotonic()
        self.last_bridge_ping_time = 0.0
        self.pressed_drive_keys = set()
        self.keyboard_drive_active = False
        self.last_keyboard_drive = (0, 0)
        self.intentional_disconnect = False
        self.link_loss_handled = False

        self.port_var = tk.StringVar()
        self.link_mode_var = tk.StringVar(value="Wi-Fi")
        self.wifi_host_var = tk.StringVar(value="AUTO")
        self.wifi_port_var = tk.StringVar(value=str(WIFI_PORT))
        self.connection_label = "Wi-Fi disconnected"
        self.status_var = tk.StringVar(value="DISCONNECTED")
        self.baud_var = tk.StringVar(value=str(BAUD_RATE))
        self.last_i2c_var = tk.StringVar(value="-")
        self.mpu_var = tk.StringVar(value="WAITING")
        self.tof_left_var = tk.StringVar(value="WAITING")
        self.tof_right_var = tk.StringVar(value="WAITING")
        self.left_motor_var = tk.StringVar(value="0% / 0.0 rpm est.")
        self.right_motor_var = tk.StringVar(value="0% / 0.0 rpm est.")
        self.int_var = tk.StringVar(value="-")
        self.stream_var = tk.StringVar(value="OFF")
        self.tof_stream_var = tk.StringVar(value="OFF")
        self.wifi_rssi_var = tk.StringVar(value="-")
        self.bridge_health_var = tk.StringVar(value="WAITING")
        self.accel_var = tk.StringVar(value="0.00 g / 0.00 g / 0.00 g")
        self.gyro_var = tk.StringVar(value="0.00 / 0.00 / 0.00 deg/s")
        self.temp_var = tk.StringVar(value="-")
        self.tilt_var = tk.StringVar(value="roll 0.0 / pitch 0.0")
        self.tof_left_distance_var = tk.StringVar(value="LEFT -")
        self.tof_right_distance_var = tk.StringVar(value="RIGHT -")
        self.left_drive_var = tk.IntVar(value=0)
        self.right_drive_var = tk.IntVar(value=0)
        self.drive_speed_var = tk.IntVar(value=60)
        self.drive_mode_var = tk.StringVar(value="PWM")
        self.drive_speed_unit_var = tk.StringVar(value="%")
        self.drive_command_var = tk.StringVar(value="STOPPED")
        self.drive_active = False
        self.active_drive_mode = "PWM"
        self.drive_sliders = []
        self.main_speed_scale = None
        self.robot_ready = False
        self.auto_stream_started = False
        self.motor_state = {
            "left_pct": 0,
            "right_pct": 0,
            "left_pwm_r": 0,
            "left_pwm_l": 0,
            "right_pwm_r": 0,
            "right_pwm_l": 0,
            "left_enc": 0,
            "right_enc": 0,
            "left_delta": 0,
            "right_delta": 0,
            "left_rpm": 0.0,
            "right_rpm": 0.0,
            "left_distance_mm": 0.0,
            "right_distance_mm": 0.0,
            "safe": "STOP",
            "sync": "IDLE",
            "sync_error": 0.0,
            "sync_trim": 0,
            "left_applied": 0,
            "right_applied": 0,
            "control": "PWM",
            "left_target_rpm": 0.0,
            "right_target_rpm": 0.0,
            "left_ramped_rpm": 0.0,
            "right_ramped_rpm": 0.0,
            "left_pi_i": 0.0,
            "right_pi_i": 0.0,
        }

        self._configure_style()
        self._build_ui()
        self.bind_all("<KeyPress>", self._on_drive_key_press, add="+")
        self.bind_all("<KeyRelease>", self._on_drive_key_release, add="+")
        self.refresh_ports()
        self.after(40, self._drain_rx_queue)
        self.after(250, self._ensure_windows_mobile_hotspot)
        self.after(900, self._auto_connect_wifi)
        self.after(500, self._motor_keepalive)
        self.after(100, self._drive_focus_watchdog)
        self.after(2000, self._link_watchdog)

    def _windows_mobile_hotspot_active(self):
        if os.name != "nt":
            return False
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            probe.bind((WINDOWS_HOTSPOT_IP, 0))
            return True
        except OSError:
            return False
        finally:
            probe.close()

    def _ensure_windows_mobile_hotspot(self):
        if os.name != "nt" or self.link_mode_var.get() != "Wi-Fi":
            return
        if self._windows_mobile_hotspot_active():
            self._log("[HOTSPOT] Windows mobile hotspot is active.\n", "ok")
            return
        self.status_var.set("WINDOWS HOTSPOT REQUIRED")
        self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
        self._append_connection_event(
            "PANEL", "HOTSPOT_AUTO_START_UNAVAILABLE",
            "Windows hotspot is off; WinRT auto-start requires packaged wiFiControl capability",
        )
        self._log(
            "[HOTSPOT] Windows hotspot is off. Automatic start was tested but Windows denied "
            "wiFiControl access to this unpackaged desktop app; waiting without opening Settings.\n",
            "warn",
        )

    def _load_motion_calibration(self):
        calibration = {
            "version": 1,
            "wheel_circumference_mm": DEFAULT_WHEEL_CIRCUMFERENCE_MM,
            "track_width_mm": DEFAULT_TRACK_WIDTH_MM,
            "left_counts_per_wheel_rev": None,
            "right_counts_per_wheel_rev": None,
            "left_forward_sign": None,
            "right_forward_sign": None,
        }
        try:
            stored = json.loads(self.motion_calibration_path.read_text(encoding="utf-8"))
            if isinstance(stored, dict):
                calibration.update(stored)
        except (OSError, ValueError, TypeError):
            pass
        return calibration

    def _motion_calibration_ready(self):
        return all(
            self.motion_calibration.get(key) not in (None, 0, 0.0)
            for key in (
                "wheel_circumference_mm",
                "track_width_mm",
                "left_counts_per_wheel_rev",
                "right_counts_per_wheel_rev",
                "left_forward_sign",
                "right_forward_sign",
            )
        )

    def _calibrated_wheel_rpm(self, side, firmware_rpm_x100):
        if not self._motion_calibration_ready():
            return None
        counts_per_rev = float(self.motion_calibration[f"{side}_counts_per_wheel_rev"])
        forward_sign = int(self.motion_calibration[f"{side}_forward_sign"])
        return ((float(firmware_rpm_x100) / 100.0) * FIRMWARE_ENCODER_COUNTS_PER_REV /
                counts_per_rev * forward_sign)

    def _load_last_esp_event_id(self):
        highest = 0
        try:
            with self.connection_logbook_path.open("r", encoding="utf-8") as handle:
                for row in handle:
                    try:
                        record = json.loads(row)
                    except (ValueError, TypeError):
                        continue
                    if record.get("source") == "ESP":
                        highest = max(highest, int(record.get("event_id", 0)))
        except OSError:
            pass
        return highest

    def _append_connection_event(self, source, code, detail="", **fields):
        record = {
            "received_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "source": source,
            "code": code,
            "detail": detail,
            "link": self.connection_label,
        }
        record.update(fields)
        try:
            with self.connection_logbook_path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(record, ensure_ascii=True) + "\n")
            return True
        except OSError:
            return False

    def _handle_esp_log_line(self, text):
        payload = text.split("ESP LOG |", 1)[-1].strip()
        fields = payload.split("|", 5)
        if len(fields) != 6:
            return None
        try:
            event_id = int(fields[0])
            boot_id = int(fields[1])
            uptime_ms = int(fields[2])
        except ValueError:
            return None

        if event_id > self.esp_last_event_id:
            stored = self._append_connection_event(
                "ESP", fields[4], fields[5], event_id=event_id,
                boot_id=boot_id, uptime_ms=uptime_ms, level=fields[3],
            )
            if not stored:
                return None
            self.esp_last_event_id = event_id
        return event_id

    def _send_bridge_control(self, command):
        if (self.serial_port is None or not self.serial_port.is_open or
                self.link_mode_var.get() != "Wi-Fi"):
            return False
        payload = (command + "\r\n").encode("ascii", errors="ignore")
        try:
            self.serial_port.write(payload)
            self.tx_count += len(payload)
            return True
        except Exception:
            return False

    def _link_watchdog(self):
        now = time.monotonic()
        if (self.link_mode_var.get() == "Wi-Fi" and self.serial_port is not None and
                self.serial_port.is_open):
            if (now - self.last_bridge_ping_time) >= 3.0:
                self._send_bridge_control("@ESP PING")
                self.last_bridge_ping_time = now
            if self.reader_alive.is_set() and (now - self.last_bridge_rx_time) >= 12.0:
                self._append_connection_event(
                    "PANEL", "HEARTBEAT_TIMEOUT",
                    f"No bridge data for {now - self.last_bridge_rx_time:.1f}s",
                )
                self.reader_alive.clear()
                try:
                    self.serial_port.close()
                except Exception:
                    pass
                self.rx_queue.put("__ARGOS_LINK_LOST__")
        self.after(2000, self._link_watchdog)

    def _configure_style(self):
        self.style = ttk.Style(self)
        self.style.theme_use("clam")
        self.style.configure("TCombobox", fieldbackground="#0b1620", background="#152231",
                             foreground="#e8f1ff", arrowcolor="#e8f1ff")
        self.style.configure("TSeparator", background="#233448")

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=0)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        sidebar_wrap = tk.Frame(self, bg="#0a141f", width=300)
        sidebar_wrap.grid(row=0, column=0, sticky="ns")
        sidebar_wrap.grid_propagate(False)
        sidebar_wrap.grid_columnconfigure(0, weight=1)
        sidebar_wrap.grid_rowconfigure(0, weight=1)

        self.sidebar_canvas = tk.Canvas(sidebar_wrap, bg="#0a141f", highlightthickness=0, width=300)
        sidebar_scrollbar = tk.Scrollbar(sidebar_wrap, orient="vertical", command=self.sidebar_canvas.yview,
                                         bg="#0a141f", troughcolor="#050b12", activebackground="#203247")
        self.sidebar_canvas.configure(yscrollcommand=sidebar_scrollbar.set)
        self.sidebar_canvas.grid(row=0, column=0, sticky="nsew")
        sidebar_scrollbar.grid(row=0, column=1, sticky="ns")

        sidebar = tk.Frame(self.sidebar_canvas, bg="#0a141f", width=284)
        self.sidebar_window = self.sidebar_canvas.create_window((0, 0), window=sidebar, anchor="nw")
        sidebar.bind("<Configure>", self._on_sidebar_configure)
        self.sidebar_canvas.bind("<Configure>", self._on_sidebar_canvas_configure)
        self.sidebar_canvas.bind("<Enter>", lambda _event: self._bind_mousewheel(self.sidebar_canvas))
        self.sidebar_canvas.bind("<Leave>", self._unbind_mousewheel)

        scroll_wrap = tk.Frame(self, bg="#071018")
        scroll_wrap.grid(row=0, column=1, sticky="nsew")
        scroll_wrap.grid_columnconfigure(0, weight=1)
        scroll_wrap.grid_rowconfigure(0, weight=1)

        self.main_canvas = tk.Canvas(scroll_wrap, bg="#071018", highlightthickness=0)
        scrollbar = tk.Scrollbar(scroll_wrap, orient="vertical", command=self.main_canvas.yview,
                                 bg="#0a141f", troughcolor="#050b12", activebackground="#203247")
        self.main_canvas.configure(yscrollcommand=scrollbar.set)
        self.main_canvas.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

        main = tk.Frame(self.main_canvas, bg="#071018")
        self.main_window = self.main_canvas.create_window((0, 0), window=main, anchor="nw")
        main.grid_columnconfigure(0, weight=1)
        main.bind("<Configure>", self._on_main_configure)
        self.main_canvas.bind("<Configure>", self._on_canvas_configure)
        self.main_canvas.bind("<Enter>", lambda _event: self._bind_mousewheel(self.main_canvas))
        self.main_canvas.bind("<Leave>", self._unbind_mousewheel)

        self._build_sidebar(sidebar)
        self._build_header(main)
        self._build_metrics(main)
        self._build_motor_dashboard(main)
        self._build_imu_dashboard(main)
        self._build_tof_dashboard(main)
        self._build_console(main)

    def _on_main_configure(self, _event=None):
        if self.main_canvas is not None:
            self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))

    def _on_canvas_configure(self, event):
        if self.main_canvas is not None and self.main_window is not None:
            self.main_canvas.itemconfigure(self.main_window, width=event.width)

    def _on_sidebar_configure(self, _event=None):
        if self.sidebar_canvas is not None:
            self.sidebar_canvas.configure(scrollregion=self.sidebar_canvas.bbox("all"))

    def _on_sidebar_canvas_configure(self, event):
        if self.sidebar_canvas is not None and self.sidebar_window is not None:
            self.sidebar_canvas.itemconfigure(self.sidebar_window, width=event.width)

    def _bind_mousewheel(self, canvas):
        self.active_scroll_canvas = canvas
        canvas.bind_all("<MouseWheel>", self._on_mousewheel)

    def _unbind_mousewheel(self, _event=None):
        if self.active_scroll_canvas is not None:
            self.active_scroll_canvas.unbind_all("<MouseWheel>")
        self.active_scroll_canvas = None

    def _on_mousewheel(self, event):
        if self.active_scroll_canvas is not None:
            self.active_scroll_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def _build_sidebar(self, parent):
        title = tk.Label(parent, text="ARGOS", bg="#0a141f", fg="#f6fbff",
                         font=("Segoe UI", 24, "bold"))
        title.pack(anchor="w", padx=18, pady=(16, 0))

        subtitle = tk.Label(parent, text="Field mobility control", bg="#0a141f",
                            fg="#6f8aa4", font=("Segoe UI", 10))
        subtitle.pack(anchor="w", padx=20, pady=(0, 14))

        self._section_label(parent, "Robot Link")
        mode_frame = tk.Frame(parent, bg="#0a141f")
        mode_frame.pack(fill="x", padx=16, pady=(0, 8))
        tk.Label(mode_frame, text="Mode", bg="#0a141f", fg="#8da4bc").pack(side="left")
        mode_combo = ttk.Combobox(mode_frame, textvariable=self.link_mode_var, state="readonly",
                                  values=("Wi-Fi", "COM"), width=11)
        mode_combo.pack(side="right")
        mode_combo.bind("<<ComboboxSelected>>", self._on_link_mode_changed)

        wifi_frame = tk.Frame(parent, bg="#0a141f")
        wifi_frame.pack(fill="x", padx=16, pady=(0, 8))
        self.wifi_host_entry = tk.Entry(wifi_frame, textvariable=self.wifi_host_var, bg="#0b1620",
                                        fg="#e8f1ff", insertbackground="#e8f1ff", relief="flat")
        self.wifi_host_entry.pack(side="left", fill="x", expand=True)
        self.wifi_port_entry = tk.Entry(wifi_frame, textvariable=self.wifi_port_var, bg="#0b1620",
                                        fg="#e8f1ff", insertbackground="#e8f1ff", relief="flat", width=6)
        self.wifi_port_entry.pack(side="left", padx=(8, 0))

        ports_frame = tk.Frame(parent, bg="#0a141f")
        ports_frame.pack(fill="x", padx=16, pady=(0, 8))
        self.ports_frame = ports_frame
        self.port_combo = ttk.Combobox(ports_frame, textvariable=self.port_var, state="readonly")
        self.port_combo.pack(side="left", fill="x", expand=True)
        self._small_button(ports_frame, "REF", self.refresh_ports).pack(side="left", padx=(8, 0))

        baud_frame = tk.Frame(parent, bg="#0a141f")
        baud_frame.pack(fill="x", padx=16, pady=(0, 8))
        self.baud_frame = baud_frame
        tk.Label(baud_frame, text="Baud", bg="#0a141f", fg="#8da4bc").pack(side="left")
        tk.Entry(baud_frame, textvariable=self.baud_var, bg="#0b1620", fg="#e8f1ff",
                 insertbackground="#e8f1ff", relief="flat", width=10).pack(side="right")

        conn_frame = tk.Frame(parent, bg="#0a141f")
        conn_frame.pack(fill="x", padx=16, pady=(0, 12))
        self._action_button(conn_frame, "CONNECT", self.connect_serial, "#23c483").pack(side="left", expand=True, fill="x")
        self._action_button(conn_frame, "DISCONNECT", self.disconnect_serial, "#ff5c7a").pack(side="left", expand=True, fill="x", padx=(8, 0))

        self._section_label(parent, "Drive")
        drive = tk.Frame(parent, bg="#0a141f")
        drive.pack(fill="x", padx=16, pady=(0, 10))
        mode_row = tk.Frame(drive, bg="#0a141f")
        mode_row.pack(fill="x", pady=(0, 7))
        tk.Label(mode_row, text="Control", bg="#0a141f", fg="#8da4bc",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        mode_combo = ttk.Combobox(mode_row, textvariable=self.drive_mode_var,
                                  values=("PWM", "RPM"), state="readonly", width=7)
        mode_combo.pack(side="right")
        mode_combo.bind("<<ComboboxSelected>>", self._on_drive_mode_changed)
        self._drive_slider(drive, "Left", self.left_drive_var).pack(fill="x", pady=(0, 6))
        self._drive_slider(drive, "Right", self.right_drive_var).pack(fill="x", pady=(0, 6))
        drive_buttons = tk.Frame(drive, bg="#0a141f")
        drive_buttons.pack(fill="x")
        self._small_button(drive_buttons, "SEND", self.send_drive_command).pack(side="left", fill="x", expand=True)
        self._small_button(drive_buttons, "STOP", self.stop_drive).pack(side="left", fill="x", expand=True, padx=(8, 0))

        self._section_label(parent, "Live Sensors")
        sensor_live = tk.Frame(parent, bg="#0a141f")
        sensor_live.pack(fill="x", padx=16, pady=(0, 8))
        self._small_button(sensor_live, "MPU LIVE", self.toggle_mpu_live).pack(side="left", fill="x", expand=True)
        self._small_button(sensor_live, "TOF LIVE", self.toggle_tof_live).pack(side="left", fill="x", expand=True, padx=(8, 0))

        sensor_once = tk.Frame(parent, bg="#0a141f")
        sensor_once.pack(fill="x", padx=16, pady=(0, 10))
        self._small_button(sensor_once, "MPU READ", lambda: self.send_command("m")).pack(side="left", fill="x", expand=True)
        self._small_button(sensor_once, "TOF CHECK", lambda: self.send_command("t")).pack(side="left", fill="x", expand=True, padx=(8, 0))
        self._small_button(parent, "CALIBRATE IMU", self.calibrate_imu).pack(
            fill="x", padx=16, pady=(0, 10))

        self._section_label(parent, "Diagnostics")
        self.system_test_button = self._action_button(
            parent, "SYSTEM TEST", self.open_system_test_menu, "#43b5ff")
        self.system_test_button.pack(fill="x", padx=16, pady=(0, 8))
        self._action_button(parent, "PHASE 02 CALIBRATION", self.open_phase02_calibration, "#a978ff").pack(
            fill="x", padx=16, pady=(0, 8))
        self._action_button(parent, "STM32 WIRELESS UPDATE", self.start_wireless_stm32_update, "#ff9f43").pack(
            fill="x", padx=16, pady=(0, 8))
        diagnostics = tk.Frame(parent, bg="#0a141f")
        diagnostics.pack(fill="x", padx=16, pady=(0, 10))
        self._small_button(diagnostics, "PIN TEST", lambda: self.send_command("K")).pack(side="left", fill="x", expand=True)
        self._small_button(diagnostics, "RAMP TEST", lambda: self.send_command("Y")).pack(side="left", fill="x", expand=True, padx=(8, 0))
        self._small_button(parent, "LINK LOGBOOK", self._show_connection_logbook).pack(
            fill="x", padx=16, pady=(0, 14))
        self._on_link_mode_changed()

    def _on_link_mode_changed(self, _event=None):
        wifi = self.link_mode_var.get() == "Wi-Fi"
        state = "normal" if wifi else "disabled"
        self.wifi_host_entry.configure(state=state)
        self.wifi_port_entry.configure(state=state)
        combo_state = "disabled" if wifi else "readonly"
        self.port_combo.configure(state=combo_state)
        for child in self.baud_frame.winfo_children():
            if isinstance(child, tk.Entry):
                child.configure(state="disabled" if wifi else "normal")

    def _build_header(self, parent):
        header = tk.Frame(parent, bg="#071018")
        header.grid(row=0, column=0, sticky="ew", padx=18, pady=(14, 8))
        header.grid_columnconfigure(0, weight=1)

        tk.Label(header, text="Field Control", bg="#071018", fg="#f6fbff",
                 font=("Segoe UI", 24, "bold")).grid(row=0, column=0, sticky="w")
        tk.Label(header, text="ARGOS mobile platform command, telemetry and safety link",
                 bg="#071018", fg="#6f8aa4", font=("Segoe UI", 10)).grid(row=1, column=0, sticky="w")

        self.status_badge = tk.Label(header, textvariable=self.status_var, bg="#3a1923",
                                     fg="#ffb3c0", padx=14, pady=8, font=("Segoe UI", 10, "bold"))
        self.status_badge.grid(row=0, column=1, rowspan=2, sticky="e")

    def _build_metrics(self, parent):
        metrics = tk.Frame(parent, bg="#071018")
        metrics.grid(row=1, column=0, sticky="ew", padx=18, pady=(0, 8))
        for index in range(9):
            metrics.grid_columnconfigure(index, weight=1)

        self._metric_card(metrics, 0, "Bridge", self.bridge_health_var)
        self._metric_card(metrics, 1, "Wi-Fi", self.wifi_rssi_var)
        self._metric_card(metrics, 2, "ToF Left", self.tof_left_var)
        self._metric_card(metrics, 3, "ToF Right", self.tof_right_var)
        self._metric_card(metrics, 4, "Left Motor", self.left_motor_var)
        self._metric_card(metrics, 5, "Right Motor", self.right_motor_var)
        self._metric_card(metrics, 6, "MPU6050", self.mpu_var)
        self._metric_card(metrics, 7, "MPU Stream", self.stream_var)
        self._metric_card(metrics, 8, "ToF Stream", self.tof_stream_var)

    def _build_motor_dashboard(self, parent):
        motor = tk.Frame(parent, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        motor.grid(row=2, column=0, sticky="ew", padx=18, pady=(0, 8))
        motor.grid_columnconfigure(0, weight=0)
        motor.grid_columnconfigure(1, weight=1)

        top = tk.Frame(motor, bg="#0b1620")
        top.grid(row=0, column=0, columnspan=2, sticky="ew", padx=12, pady=(10, 4))
        top.grid_columnconfigure(0, weight=1)
        tk.Label(top, text="Mobility Control", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        tk.Label(top, text="BTS7960 PWM + encoder feedback", bg="#0b1620", fg="#8da4bc",
                 font=("Segoe UI", 9)).grid(row=0, column=1, sticky="e")

        controls = tk.Frame(motor, bg="#071018", width=360, height=250,
                            highlightthickness=1, highlightbackground="#17283a")
        controls.grid(row=1, column=0, sticky="nsew", padx=(12, 6), pady=(0, 12))
        controls.grid_propagate(False)
        for column in range(3):
            controls.grid_columnconfigure(column, weight=1, uniform="drive")
        for row in range(1, 4):
            controls.grid_rowconfigure(row, weight=1, uniform="drive")

        tk.Label(controls, text="LIVE DRIVE", bg="#071018", fg="#8da4bc",
                 font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(10, 4))
        self.drive_state_label = tk.Label(controls, textvariable=self.drive_command_var, bg="#071018",
                                          fg="#72f0c0", font=("Consolas", 10, "bold"))
        self.drive_state_label.grid(row=0, column=1, columnspan=2, sticky="e", padx=12, pady=(10, 4))

        self._drive_button(controls, "▲", "FORWARD", 0, 1, "#1f7a5a").grid(
            row=1, column=1, sticky="nsew", padx=5, pady=5)
        self._drive_button(controls, "◀", "TURN LEFT", -1, 0, "#28689b").grid(
            row=2, column=0, sticky="nsew", padx=5, pady=5)
        stop = tk.Button(controls, text="■", command=self.stop_drive, bg="#d9445f", fg="#ffffff",
                         activebackground="#f05b73", activeforeground="#ffffff", relief="flat",
                         font=("Segoe UI Symbol", 22, "bold"), cursor="hand2")
        stop.grid(row=2, column=1, sticky="nsew", padx=5, pady=5)
        self._drive_button(controls, "▶", "TURN RIGHT", 1, 0, "#28689b").grid(
            row=2, column=2, sticky="nsew", padx=5, pady=5)
        self._drive_button(controls, "▼", "REVERSE", 0, -1, "#9b6331").grid(
            row=3, column=1, sticky="nsew", padx=5, pady=5)

        speed_wrap = tk.Frame(controls, bg="#071018")
        speed_wrap.grid(row=4, column=0, columnspan=3, sticky="ew", padx=12, pady=(4, 10))
        speed_wrap.grid_columnconfigure(1, weight=1)
        tk.Label(speed_wrap, text="MODE", bg="#071018", fg="#8da4bc",
                 font=("Segoe UI", 8, "bold")).grid(row=0, column=0, sticky="w")
        main_mode_combo = ttk.Combobox(speed_wrap, textvariable=self.drive_mode_var,
                                       values=("PWM", "RPM"), state="readonly", width=7)
        main_mode_combo.grid(row=0, column=1, sticky="w", padx=10, pady=(0, 3))
        main_mode_combo.bind("<<ComboboxSelected>>", self._on_drive_mode_changed)
        tk.Label(speed_wrap, text="SPEED", bg="#071018", fg="#8da4bc",
                 font=("Segoe UI", 8, "bold")).grid(row=1, column=0, sticky="w")
        self.main_speed_scale = tk.Scale(
            speed_wrap, from_=50, to=100, orient="horizontal", variable=self.drive_speed_var,
            bg="#071018", fg="#e8f1ff", troughcolor="#18283a", activebackground="#23c483",
            highlightthickness=0, relief="flat", showvalue=False)
        self.main_speed_scale.grid(row=1, column=1, sticky="ew", padx=10)
        speed_value = tk.Frame(speed_wrap, bg="#071018")
        speed_value.grid(row=1, column=2, sticky="e")
        tk.Label(speed_value, textvariable=self.drive_speed_var, bg="#071018", fg="#f6fbff",
                 font=("Consolas", 10, "bold"), width=3).pack(side="left")
        tk.Label(speed_value, textvariable=self.drive_speed_unit_var, bg="#071018", fg="#8da4bc",
                 font=("Consolas", 9, "bold"), width=3).pack(side="left")

        telemetry = tk.Frame(motor, bg="#050b12")
        telemetry.grid(row=1, column=1, sticky="nsew", padx=(6, 12), pady=(0, 12))
        telemetry.grid_columnconfigure(0, weight=1)
        telemetry.grid_rowconfigure(1, weight=1)
        tk.Label(telemetry, text="MOTOR TELEMETRY", bg="#050b12", fg="#7690aa",
                 font=("Segoe UI", 9, "bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(10, 0))
        self.motor_canvas = tk.Canvas(telemetry, height=210, bg="#050b12", highlightthickness=0)
        self.motor_canvas.grid(row=1, column=0, sticky="nsew", padx=4, pady=(0, 4))
        self.motor_canvas.bind("<Configure>", lambda _event: self._draw_motor_field())
        self._draw_motor_field()

    def _drive_button(self, parent, symbol, label, horizontal, vertical, color):
        if vertical > 0:
            command = lambda: self._drive_preset(label, 1, 1)
        elif vertical < 0:
            command = lambda: self._drive_preset(label, -1, -1)
        elif horizontal < 0:
            command = lambda: self._drive_preset(label, -1, 1)
        else:
            command = lambda: self._drive_preset(label, 1, -1)
        return tk.Button(parent, text=symbol, command=command, bg=color, fg="#ffffff",
                         activebackground="#35a47d" if vertical else "#3b82b8",
                         activeforeground="#ffffff", relief="flat",
                         font=("Segoe UI Symbol", 22, "bold"), cursor="hand2")

    def _drive_preset(self, label, left_sign, right_sign):
        if self.lowband_test_active:
            return
        speed = self.drive_speed_var.get()
        self.left_drive_var.set(left_sign * speed)
        self.right_drive_var.set(right_sign * speed)
        if self.send_drive_command():
            unit = "RPM" if self.drive_mode_var.get() == "RPM" else "%"
            self.drive_command_var.set(f"{label}  {speed} {unit}")
            self.drive_state_label.configure(fg="#72f0c0")
        else:
            self.left_drive_var.set(0)
            self.right_drive_var.set(0)
            self.drive_command_var.set("LINK REQUIRED")
            self.drive_state_label.configure(fg="#ff7187")

    def _keyboard_input_allowed(self):
        focused = self.focus_get()
        return not isinstance(focused, (tk.Entry, tk.Text, tk.Scale, ttk.Combobox))

    def _on_drive_key_press(self, event):
        if not self._keyboard_input_allowed():
            return None
        key = event.keysym
        if self.lowband_test_active:
            if key == "space":
                self._abort_lowband_rpm_test("Emergency stop from keyboard.")
            return "break" if key in ("space", "Up", "Down", "Left", "Right") else None
        if key == "space":
            self.pressed_drive_keys.clear()
            self.keyboard_drive_active = False
            self.stop_drive()
            return "break"
        if key not in ("Up", "Down", "Left", "Right"):
            return None
        if key not in self.pressed_drive_keys:
            self.pressed_drive_keys.add(key)
            self._apply_keyboard_drive()
        return "break"

    def _on_drive_key_release(self, event):
        key = event.keysym
        if key not in ("Up", "Down", "Left", "Right"):
            return None
        self.pressed_drive_keys.discard(key)
        self._apply_keyboard_drive()
        return "break"

    def _apply_keyboard_drive(self):
        throttle = int("Up" in self.pressed_drive_keys) - int("Down" in self.pressed_drive_keys)
        steer = int("Right" in self.pressed_drive_keys) - int("Left" in self.pressed_drive_keys)
        speed = self.drive_speed_var.get()
        inner = max(0, round(speed * 0.38))

        if throttle > 0:
            left = inner if steer < 0 else speed
            right = inner if steer > 0 else speed
            label = "FORWARD"
        elif throttle < 0:
            left = -inner if steer < 0 else -speed
            right = -inner if steer > 0 else -speed
            label = "REVERSE"
        elif steer < 0:
            left, right, label = -speed, speed, "PIVOT LEFT"
        elif steer > 0:
            left, right, label = speed, -speed, "PIVOT RIGHT"
        else:
            left, right, label = 0, 0, "STOPPED"

        target = (left, right)
        if target == self.last_keyboard_drive:
            return
        self.last_keyboard_drive = target
        self.left_drive_var.set(left)
        self.right_drive_var.set(right)

        if target == (0, 0):
            if self.keyboard_drive_active:
                self.drive_active = False
                self.keyboard_drive_active = False
                self.drive_command_var.set("STOPPED")
                if self.serial_port is not None and self.serial_port.is_open:
                    self.send_command("x")
            return

        if not self.robot_ready:
            self.drive_command_var.set("LINK REQUIRED")
            self.drive_state_label.configure(fg="#ff7187")
            return

        if self.send_drive_command():
            self.keyboard_drive_active = True
            self.drive_command_var.set(f"{label}  L{left:+d} R{right:+d}")
            self.drive_state_label.configure(fg="#72f0c0")

    def _drive_focus_watchdog(self):
        try:
            panel_has_focus = self.focus_displayof() is not None
        except tk.TclError:
            return

        if self.keyboard_drive_active and not panel_has_focus:
            self.pressed_drive_keys.clear()
            self.last_keyboard_drive = (0, 0)
            self.stop_drive()
            self.drive_command_var.set("FOCUS STOP")
        self.after(100, self._drive_focus_watchdog)

    def _build_imu_dashboard(self, parent):
        dash = tk.Frame(parent, bg="#071018")
        dash.grid(row=3, column=0, sticky="ew", padx=18, pady=(0, 8))
        dash.grid_columnconfigure(0, weight=0)
        dash.grid_columnconfigure(1, weight=1)

        horizon_wrap = tk.Frame(dash, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        horizon_wrap.grid(row=0, column=0, sticky="nsw", padx=(0, 12))
        tk.Label(horizon_wrap, text="IMU Attitude", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).pack(anchor="w", padx=12, pady=(10, 2))
        self.tilt_label = tk.Label(horizon_wrap, textvariable=self.tilt_var, bg="#0b1620", fg="#8da4bc",
                                   font=("Segoe UI", 9))
        self.tilt_label.pack(anchor="w", padx=12)
        self.horizon_canvas = tk.Canvas(horizon_wrap, width=220, height=110, bg="#050b12",
                                        highlightthickness=0)
        self.horizon_canvas.pack(padx=12, pady=12)
        self.horizon_canvas.bind("<Configure>", lambda _event: self._redraw_imu())

        plot_wrap = tk.Frame(dash, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        plot_wrap.grid(row=0, column=1, sticky="nsew")
        plot_wrap.grid_columnconfigure(0, weight=1)
        plot_wrap.grid_rowconfigure(1, weight=1)

        plot_top = tk.Frame(plot_wrap, bg="#0b1620")
        plot_top.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 4))
        plot_top.grid_columnconfigure(0, weight=1)
        tk.Label(plot_top, text="Live IMU Trace", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        values = tk.Frame(plot_top, bg="#0b1620")
        values.grid(row=0, column=1, sticky="e")
        tk.Label(values, textvariable=self.accel_var, bg="#0b1620", fg="#72f0c0",
                 font=("Consolas", 9)).pack(anchor="e")
        tk.Label(values, textvariable=self.gyro_var, bg="#0b1620", fg="#8bc7ff",
                 font=("Consolas", 9)).pack(anchor="e")
        tk.Label(values, textvariable=self.temp_var, bg="#0b1620", fg="#ffc66d",
                 font=("Consolas", 9)).pack(anchor="e")

        self.trace_canvas = tk.Canvas(plot_wrap, height=110, bg="#050b12", highlightthickness=0)
        self.trace_canvas.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 12))
        self.trace_canvas.bind("<Configure>", lambda _event: self._redraw_imu())

    def _build_tof_dashboard(self, parent):
        tof = tk.Frame(parent, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        tof.grid(row=4, column=0, sticky="ew", padx=18, pady=(0, 8))
        tof.grid_columnconfigure(0, weight=1)

        top = tk.Frame(tof, bg="#0b1620")
        top.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 4))
        top.grid_columnconfigure(0, weight=1)
        tk.Label(top, text="Front ToF Field", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        values = tk.Frame(top, bg="#0b1620")
        values.grid(row=0, column=1, sticky="e")
        tk.Label(values, textvariable=self.tof_left_distance_var, bg="#0b1620", fg="#9cd6ff",
                 font=("Consolas", 10, "bold")).pack(side="left", padx=(0, 18))
        tk.Label(values, textvariable=self.tof_right_distance_var, bg="#0b1620", fg="#ffb38f",
                 font=("Consolas", 10, "bold")).pack(side="left")

        self.tof_canvas = tk.Canvas(tof, height=125, bg="#050b12", highlightthickness=0)
        self.tof_canvas.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 12))
        self.tof_canvas.bind("<Configure>", lambda _event: self._draw_tof_field())
        self._draw_tof_field()

    def _build_console(self, parent):
        console_wrap = tk.Frame(parent, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        console_wrap.grid(row=5, column=0, sticky="nsew", padx=18, pady=(0, 16))
        console_wrap.grid_columnconfigure(0, weight=1)
        console_wrap.grid_rowconfigure(1, weight=1)

        top = tk.Frame(console_wrap, bg="#0b1620")
        top.grid(row=0, column=0, sticky="ew", padx=14, pady=12)
        top.grid_columnconfigure(0, weight=1)
        tk.Label(top, text="Live Serial Feed", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 12, "bold")).grid(row=0, column=0, sticky="w")
        self._small_button(top, "CLEAR", self.clear_console).grid(row=0, column=1, sticky="e")

        console_body = tk.Frame(console_wrap, bg="#050b12")
        console_body.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 14))
        console_body.grid_columnconfigure(0, weight=1)
        console_body.grid_rowconfigure(0, weight=1)

        self.console = tk.Text(console_body, bg="#050b12", fg="#d8e7f7", insertbackground="#d8e7f7",
                               relief="flat", wrap="word", font=("Consolas", 10), padx=12, pady=12,
                               height=14)
        console_scrollbar = tk.Scrollbar(console_body, orient="vertical", command=self.console.yview,
                                         bg="#0a141f", troughcolor="#050b12", activebackground="#203247")
        self.console.configure(yscrollcommand=console_scrollbar.set)
        self.console.grid(row=0, column=0, sticky="nsew")
        console_scrollbar.grid(row=0, column=1, sticky="ns")
        self.console.bind("<MouseWheel>", self._on_console_mousewheel)
        self.console.bind("<KeyRelease>", self._refresh_console_autoscroll)
        self.console.bind("<ButtonRelease-1>", self._refresh_console_autoscroll)
        self.console.tag_configure("rx", foreground="#d8e7f7")
        self.console.tag_configure("tx", foreground="#7fe7ad")
        self.console.tag_configure("warn", foreground="#ffc66d")
        self.console.tag_configure("error", foreground="#ff7187")
        self.console.tag_configure("ok", foreground="#72f0c0")
        self._redraw_imu()

    def _section_label(self, parent, text):
        tk.Label(parent, text=text, bg="#0a141f", fg="#9fb5cc",
                 font=("Segoe UI", 9, "bold")).pack(anchor="w", padx=22, pady=(8, 8))

    def _metric_card(self, parent, column, label, variable):
        card = tk.Frame(parent, bg="#0b1620", highlightthickness=1, highlightbackground="#203247")
        card.grid(row=0, column=column, sticky="ew", padx=(0 if column == 0 else 8, 0))
        tk.Label(card, text=label, bg="#0b1620", fg="#7690aa",
                 font=("Segoe UI", 9)).pack(anchor="w", padx=14, pady=(10, 0))
        tk.Label(card, textvariable=variable, bg="#0b1620", fg="#f6fbff",
                 font=("Segoe UI", 14, "bold")).pack(anchor="w", padx=14, pady=(0, 12))

    def _action_button(self, parent, text, command, color):
        return tk.Button(parent, text=text, command=command, bg=color, fg="#061017",
                         activebackground=color, activeforeground="#061017", relief="flat",
                         font=("Segoe UI", 9, "bold"), padx=8, pady=7, cursor="hand2")

    def _small_button(self, parent, text, command):
        return tk.Button(parent, text=text, command=command, bg="#18283a", fg="#e8f1ff",
                         activebackground="#22384f", activeforeground="#ffffff", relief="flat",
                         font=("Segoe UI", 8, "bold"), padx=8, pady=6, cursor="hand2")

    def _drive_slider(self, parent, label, variable):
        wrap = tk.Frame(parent, bg="#0a141f")
        row = tk.Frame(wrap, bg="#0a141f")
        row.pack(fill="x")
        tk.Label(row, text=label, bg="#0a141f", fg="#8da4bc",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        tk.Label(row, textvariable=variable, bg="#0a141f", fg="#e8f1ff",
                 font=("Consolas", 9, "bold"), width=4).pack(side="right")
        scale = tk.Scale(wrap, from_=-100, to=100, orient="horizontal", variable=variable,
                         bg="#0a141f", fg="#8da4bc", troughcolor="#18283a",
                         activebackground="#23c483", highlightthickness=0, relief="flat",
                         length=235, showvalue=False)
        scale.pack(fill="x")
        self.drive_sliders.append(scale)
        return wrap

    def _on_drive_mode_changed(self, _event=None):
        self.stop_drive()
        rpm_mode = self.drive_mode_var.get() == "RPM"
        limit = 55 if rpm_mode else 100
        minimum = 15 if rpm_mode else 50
        default_speed = 35 if rpm_mode else 60
        for scale in self.drive_sliders:
            scale.configure(from_=-limit, to=limit)
        if self.main_speed_scale is not None:
            self.main_speed_scale.configure(from_=minimum, to=limit)
        self.drive_speed_var.set(default_speed)
        self.drive_speed_unit_var.set("RPM" if rpm_mode else "%")
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.active_drive_mode = self.drive_mode_var.get()
        self.drive_command_var.set(f"{self.active_drive_mode} MODE")

    def refresh_ports(self):
        if list_ports is None:
            self.port_combo["values"] = []
            self._log("pyserial yok. Once 'pip install pyserial' kurulmali.\n", "error")
            return

        ports = []
        for port in list_ports.comports():
            ports.append(f"{port.device} - {port.description}")

        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])
        if not ports:
            self._log("COM port bulunamadi. USB kabloyu ve ST-LINK VCP surucusunu kontrol et.\n", "warn")

    def connect_serial(self, quiet=False):
        if self.serial_port is not None and self.serial_port.is_open:
            return True

        if self.link_mode_var.get() == "Wi-Fi":
            requested_host = self.wifi_host_var.get().strip() or "AUTO"
            host = requested_host
            port = self.wifi_port_var.get()
            try:
                if requested_host.upper() == "AUTO":
                    discovered = discover_argos()
                    if discovered is not None:
                        host, port = discovered
                    else:
                        host, port = WIFI_HOST, int(self.wifi_port_var.get())
                else:
                    host, port = requested_host, int(self.wifi_port_var.get())
                self.serial_port = TcpLineTransport(host, port)
            except Exception as exc:
                self.serial_port = None
                if not quiet:
                    messagebox.showerror("Wi-Fi baglanti hatasi",
                                         f"Once bilgisayari ARGOS Wi-Fi agina bagla.\n\n{host}:{self.wifi_port_var.get()}\n{exc}")
                return False
            link_name = f"Wi-Fi {host}:{port}"
        else:
            return self._connect_com(quiet)

        self._start_reader(link_name)
        return True

    def _connect_com(self, quiet=False):
        if serial is None:
            if not quiet:
                messagebox.showerror("pyserial eksik", "Seri port icin pyserial gerekiyor: pip install pyserial")
            return False

        selected = self.port_var.get()
        if not selected:
            if not quiet:
                messagebox.showwarning("Port yok", "Once bir COM port sec.")
            return False

        port_name = selected.split(" - ", 1)[0]
        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port_name, baud, timeout=0.05, write_timeout=0.5)
        except Exception as exc:
            if not quiet:
                messagebox.showerror("Baglanti hatasi", str(exc))
            return False

        self._start_reader(f"COM {port_name} @ {baud}")
        return True

    def _start_reader(self, link_name):
        self.connection_label = link_name
        self.intentional_disconnect = False
        self.link_loss_handled = False
        self.last_bridge_rx_time = time.monotonic()
        self.last_bridge_ping_time = 0.0
        self.reader_alive.set()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.robot_ready = False
        self.auto_stream_started = False
        if link_name.startswith("Wi-Fi"):
            self.status_var.set("WIFI CONNECTED - STM WAITING")
            self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
        else:
            self.status_var.set(f"CONNECTED {link_name} - WAITING")
            self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
        self._append_connection_event("PANEL", "LINK_CONNECTED", link_name)
        self._log(f"[CONNECTED] {link_name}\n", "ok")
        if link_name.startswith("Wi-Fi"):
            self.after(250, lambda: self._send_bridge_control("@ESP LOGS"))

    def _auto_connect_wifi(self):
        if self.link_mode_var.get() == "Wi-Fi" and (self.serial_port is None or not self.serial_port.is_open):
            if not self.connect_serial(quiet=True):
                self.status_var.set("WAITING FOR ARGOS WI-FI")
                self.after(2500, self._auto_connect_wifi)

    def disconnect_serial(self):
        disconnected_link = self.connection_label
        self.intentional_disconnect = True
        if self.lowband_test_active:
            self._abort_lowband_rpm_test("Connection closed by operator.")
        self.reader_alive.clear()
        self.drive_active = False
        self.keyboard_drive_active = False
        self.pressed_drive_keys.clear()
        self.last_keyboard_drive = (0, 0)
        self.robot_ready = False
        self.auto_stream_started = False
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_command_var.set("STOPPED")
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.connection_label = "Disconnected"
        self.status_var.set("DISCONNECTED")
        self.status_badge.configure(bg="#3a1923", fg="#ffb3c0")
        self.bridge_health_var.set("OFFLINE")
        self.wifi_rssi_var.set("-")
        self._append_connection_event(
            "PANEL", "LINK_DISCONNECTED", "User/application disconnect",
            link=disconnected_link,
        )
        self._log("[DISCONNECTED]\n", "warn")

    def send_drive_command(self, allow_test=False, force_pwm=False):
        if self.lowband_test_active and not allow_test:
            self._log("[LOW-BAND] Manual drive command ignored while the test is active.\n", "warn")
            return False
        if not self.robot_ready:
            messagebox.showwarning("STM32 bekleniyor", "ESP bagli fakat STM32 UART yaniti henuz alinmadi.")
            return False
        mode = "PWM" if force_pwm else self.drive_mode_var.get()
        left = self.left_drive_var.get()
        right = self.right_drive_var.get()
        if mode == "RPM":
            left = max(-55, min(55, left))
            right = max(-55, min(55, right))
            command = f"V {left} {right}"
        else:
            command = f"P {left} {right}"
        sent = self.send_command(command)
        if sent:
            self.active_drive_mode = mode
        self.drive_active = bool(sent and (self.left_drive_var.get() != 0 or self.right_drive_var.get() != 0))
        return sent

    def stop_drive(self, abort_test=True):
        if self.lowband_test_active and abort_test:
            self._abort_lowband_rpm_test("Emergency/manual stop requested.")
            return
        self.drive_active = False
        self.keyboard_drive_active = False
        self.pressed_drive_keys.clear()
        self.last_keyboard_drive = (0, 0)
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_command_var.set("STOPPED")
        if hasattr(self, "drive_state_label"):
            self.drive_state_label.configure(fg="#72f0c0")
        self.send_command("x")

    def start_wireless_stm32_update(self):
        if self.firmware_update_active:
            messagebox.showinfo("Update active", "An STM32 update is already running.")
            return
        if self.link_mode_var.get() != "Wi-Fi" or self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning(
                "Wi-Fi required",
                "Connect the panel to the ARGOS ESP bridge before starting a wireless STM32 update.",
            )
            return
        default_image = Path(__file__).resolve().parent.parent / "Debug" / "ARGOS_OTA.bin"
        selected = filedialog.askopenfilename(
            title="Select ARGOS OTA application image",
            initialdir=str(default_image.parent),
            initialfile=default_image.name,
            filetypes=(("ARGOS OTA binary", "*.bin"), ("All files", "*.*")),
        )
        if not selected:
            return
        image_path = Path(selected)
        try:
            image = image_path.read_bytes()
        except OSError as exc:
            messagebox.showerror("Image read failed", str(exc))
            return
        try:
            validate_ota_image(image)
        except ValueError as exc:
            messagebox.showerror("Invalid image", str(exc))
            return

        image_crc = zlib.crc32(image) & 0xFFFFFFFF
        if not messagebox.askyesno(
            "Wireless STM32 update",
            "The robot will STOP and remain unavailable during programming.\n\n"
            f"File: {image_path.name}\nSize: {len(image)} bytes\nCRC32: {image_crc:08X}\n\n"
            "Continue?",
        ):
            return

        try:
            host = self.serial_port.socket.getpeername()[0]
        except (AttributeError, OSError):
            host = self.wifi_host_var.get().strip()
        if not host or host.upper() == "AUTO":
            messagebox.showerror("Bridge address missing", "The ESP bridge IP address could not be determined.")
            return

        self.stop_drive()
        self.firmware_update_active = True
        self.status_var.set("STM32 WIRELESS UPDATE")
        self.status_badge.configure(bg="#4a3010", fg="#ffd79a")
        self._log(
            f"[STM32 UPDATE] Uploading {image_path.name}, {len(image)} bytes, CRC {image_crc:08X}.\n",
            "warn",
        )
        worker = threading.Thread(
            target=self._wireless_update_worker,
            args=(host, image, image_crc, int(time.time())),
            daemon=True,
        )
        worker.start()

    def _wireless_update_worker(self, host, image, image_crc, build_id):
        connection = None
        try:
            boundary = f"ARGOS-{time.time_ns()}"
            while boundary.encode("ascii") in image:
                boundary += "X"
            body = (
                f"--{boundary}\r\n"
                'Content-Disposition: form-data; name="firmware"; filename="ARGOS_OTA.bin"\r\n'
                "Content-Type: application/octet-stream\r\n\r\n"
            ).encode("ascii") + image + f"\r\n--{boundary}--\r\n".encode("ascii")
            connection = http.client.HTTPConnection(host, WIFI_UPDATE_PORT, timeout=150)
            connection.request(
                "POST",
                "/stm32",
                body=body,
                headers={
                    "Content-Type": f"multipart/form-data; boundary={boundary}",
                    "X-ARGOS-Size": str(len(image)),
                    "X-ARGOS-CRC32": str(image_crc),
                    "X-ARGOS-Build": str(build_id),
                    "X-ARGOS-Confirm": "STOPPED",
                },
            )
            response = connection.getresponse()
            detail = response.read(4096).decode("utf-8", errors="replace").strip()
            ok = response.status == 200 and detail.startswith("UPDATE PASS")
            result = detail or f"HTTP {response.status} {response.reason}"
        except Exception as exc:
            ok = False
            result = str(exc)
        finally:
            if connection is not None:
                connection.close()
        self.after(0, lambda: self._finish_wireless_update(ok, result))

    def _finish_wireless_update(self, ok, result):
        self.firmware_update_active = False
        self._append_connection_event(
            "PANEL",
            "STM32_UPDATE_PASS" if ok else "STM32_UPDATE_FAIL",
            result,
        )
        if ok:
            self._log(f"[STM32 UPDATE PASS] {result}\n", "ok")
            self.status_var.set("UPDATE PASS - STM32 RESTARTING")
            self.status_badge.configure(bg="#123c2c", fg="#8effc5")
            messagebox.showinfo(
                "STM32 update complete",
                "The image was cached by ESP, written by the STM32 bootloader and CRC verified.\n\n"
                + result,
            )
        else:
            self._log(f"[STM32 UPDATE FAIL] {result}\n", "error")
            self.status_var.set("STM32 UPDATE FAILED")
            self.status_badge.configure(bg="#3a1923", fg="#ffb3c0")
            messagebox.showerror(
                "STM32 update failed",
                result
                + "\n\nIf the bridge says the bootloader is missing, install ARGOS_BOOTSTRAP.bin once through ST-LINK.",
            )

    def _motor_keepalive(self):
        if (self.drive_active and self.serial_port is not None and self.serial_port.is_open and
                (self.left_drive_var.get() != 0 or self.right_drive_var.get() != 0)):
            prefix = "V" if self.active_drive_mode == "RPM" else "P"
            payload = f"{prefix} {self.left_drive_var.get()} {self.right_drive_var.get()}\r\n".encode("ascii")
            try:
                self.serial_port.write(payload)
                self.tx_count += len(payload)
            except Exception:
                pass
        self.after(500, self._motor_keepalive)

    def toggle_mpu_live(self):
        self.send_command("M")

    def toggle_tof_live(self):
        self.send_command("D")

    def calibrate_imu(self):
        if not self.robot_ready:
            messagebox.showwarning("STM32 bekleniyor", "Kalibrasyon icin once STM32 baglantisi kurulmalidir.")
            return
        if not messagebox.askokcancel(
                "IMU kalibrasyonu",
                "Robotu tamamen hareketsiz ve duz bir zeminde tut. Yaklasik 3 saniye surer."):
            return
        self.stop_drive()
        self.send_command("S MPU 0")
        self.after(200, lambda: self.send_command("c"))
        self.after(4200, lambda: self.send_command("S MPU 1"))

    def open_phase02_calibration(self):
        if self.phase02_window is not None and self.phase02_window.winfo_exists():
            self.phase02_window.lift()
            self.phase02_window.focus_force()
            return

        self.stop_drive()
        self.encoder_calibration_zero = None
        self.encoder_calibration_measurements = {}

        popup = tk.Toplevel(self)
        self.phase02_window = popup
        popup.title("ARGOS Phase 02 Motion Calibration")
        popup.geometry("760x690")
        popup.minsize(680, 620)
        popup.configure(bg="#071018")
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(2, weight=1)
        popup.protocol("WM_DELETE_WINDOW", self._close_phase02_calibration)

        tk.Label(popup, text="Phase 02 - Encoder / Geometry Calibration",
                 bg="#071018", fg="#f6fbff", font=("Segoe UI", 17, "bold")).grid(
                     row=0, column=0, sticky="w", padx=18, pady=(16, 4))
        tk.Label(
            popup,
            text=("Motors stay stopped. Lift the wheels, mark one point on each wheel and rotate "
                  f"each wheel exactly {ENCODER_CALIBRATION_TURNS} turns in its FORWARD driving direction."),
            bg="#071018", fg="#8da4bc", justify="left", wraplength=700,
            font=("Segoe UI", 10),
        ).grid(row=1, column=0, sticky="ew", padx=18, pady=(0, 12))

        body = tk.Frame(popup, bg="#0a141f", highlightbackground="#203247", highlightthickness=1)
        body.grid(row=2, column=0, sticky="nsew", padx=18, pady=(0, 12))
        body.grid_columnconfigure(1, weight=1)

        self.phase02_vars = {
            "circumference": tk.StringVar(
                value=f"{float(self.motion_calibration.get('wheel_circumference_mm', DEFAULT_WHEEL_CIRCUMFERENCE_MM)):.1f}"),
            "track": tk.StringVar(
                value=f"{float(self.motion_calibration.get('track_width_mm', DEFAULT_TRACK_WIDTH_MM)):.1f}"),
            "zero": tk.StringVar(value="Not captured"),
            "left": tk.StringVar(value=self._phase02_saved_side_text("left")),
            "right": tk.StringVar(value=self._phase02_saved_side_text("right")),
            "result": tk.StringVar(value="Waiting for encoder zero."),
        }

        tk.Label(body, text="Measured geometry", bg="#0a141f", fg="#72f0c0",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, columnspan=3, sticky="w", padx=14, pady=(14, 8))
        self._phase02_entry_row(body, 1, "Wheel circumference", self.phase02_vars["circumference"], "mm")
        self._phase02_entry_row(body, 2, "Track width", self.phase02_vars["track"], "mm")

        tk.Label(body, text="Manual encoder measurement", bg="#0a141f", fg="#72f0c0",
                 font=("Segoe UI", 11, "bold")).grid(row=3, column=0, columnspan=3, sticky="w", padx=14, pady=(18, 8))

        zero_button = self._small_button(body, "1  ZERO ENCODERS", self._phase02_zero_encoders)
        zero_button.grid(row=4, column=0, sticky="ew", padx=14, pady=5)
        tk.Label(body, textvariable=self.phase02_vars["zero"], bg="#0a141f", fg="#d8e7f7",
                 anchor="w", font=("Consolas", 10)).grid(row=4, column=1, columnspan=2, sticky="ew", padx=10)

        left_button = self._small_button(
            body, f"2  CAPTURE LEFT ({ENCODER_CALIBRATION_TURNS} TURNS)",
            lambda: self._phase02_request_capture("left"))
        left_button.grid(row=5, column=0, sticky="ew", padx=14, pady=5)
        tk.Label(body, textvariable=self.phase02_vars["left"], bg="#0a141f", fg="#d8e7f7",
                 anchor="w", font=("Consolas", 10)).grid(row=5, column=1, columnspan=2, sticky="ew", padx=10)

        right_button = self._small_button(
            body, f"3  CAPTURE RIGHT ({ENCODER_CALIBRATION_TURNS} TURNS)",
            lambda: self._phase02_request_capture("right"))
        right_button.grid(row=6, column=0, sticky="ew", padx=14, pady=5)
        tk.Label(body, textvariable=self.phase02_vars["right"], bg="#0a141f", fg="#d8e7f7",
                 anchor="w", font=("Consolas", 10)).grid(row=6, column=1, columnspan=2, sticky="ew", padx=10)

        result_box = tk.Label(body, textvariable=self.phase02_vars["result"], bg="#050b12",
                              fg="#8bc7ff", justify="left", anchor="nw", wraplength=680,
                              font=("Consolas", 10), padx=12, pady=12)
        result_box.grid(row=7, column=0, columnspan=3, sticky="nsew", padx=14, pady=(14, 12))
        body.grid_rowconfigure(7, weight=1)

        actions = tk.Frame(popup, bg="#071018")
        actions.grid(row=3, column=0, sticky="ew", padx=18, pady=(0, 16))
        actions.grid_columnconfigure(0, weight=1)
        self._action_button(actions, "SAVE CALIBRATION", self._save_phase02_calibration, "#23c483").grid(
            row=0, column=1, padx=(8, 0))
        self._small_button(actions, "CLOSE", self._close_phase02_calibration).grid(
            row=0, column=2, padx=(8, 0))

    def _phase02_entry_row(self, parent, row, label, variable, suffix):
        tk.Label(parent, text=label, bg="#0a141f", fg="#8da4bc",
                 font=("Segoe UI", 10)).grid(row=row, column=0, sticky="w", padx=14, pady=5)
        tk.Entry(parent, textvariable=variable, bg="#071018", fg="#f6fbff",
                 insertbackground="#f6fbff", relief="flat", width=12,
                 font=("Consolas", 10)).grid(row=row, column=1, sticky="w", padx=10, pady=5)
        tk.Label(parent, text=suffix, bg="#0a141f", fg="#627891",
                 font=("Segoe UI", 9)).grid(row=row, column=2, sticky="w", padx=(0, 14))

    def _phase02_saved_side_text(self, side):
        counts = self.motion_calibration.get(f"{side}_counts_per_wheel_rev")
        sign = self.motion_calibration.get(f"{side}_forward_sign")
        if counts is None:
            return "Not measured"
        direction = "+" if int(sign or 1) > 0 else "-"
        return f"saved {float(counts):.1f} count/rev  forward sign {direction}"

    def _phase02_connection_ready(self):
        if self.serial_port is None or not self.serial_port.is_open or not self.robot_ready:
            messagebox.showwarning("STM32 waiting", "Connect ARGOS and wait for STM32 telemetry first.")
            return False
        return True

    def _phase02_zero_encoders(self):
        if not self._phase02_connection_ready():
            return
        self.stop_drive()
        self.send_command("Z")
        self.after(250, lambda: self.send_command("E"))
        self.after(550, self._phase02_store_zero)

    def _phase02_store_zero(self):
        if not self.phase02_vars:
            return
        self.encoder_calibration_zero = {
            "left": int(self.motor_state.get("left_enc", 0)),
            "right": int(self.motor_state.get("right_enc", 0)),
        }
        self.encoder_calibration_measurements = {}
        self.phase02_vars["zero"].set(
            f"L {self.encoder_calibration_zero['left']:+d} / R {self.encoder_calibration_zero['right']:+d}")
        self.phase02_vars["left"].set("Rotate LEFT wheel forward exactly 10 turns, then capture.")
        self.phase02_vars["right"].set("Rotate RIGHT wheel forward exactly 10 turns, then capture.")
        self.phase02_vars["result"].set(
            "Zero captured. Motors remain stopped. Rotate only the requested wheel by hand; "
            "the wheel mark must finish at its starting position.")

    def _phase02_request_capture(self, side):
        if not self._phase02_connection_ready():
            return
        if self.encoder_calibration_zero is None:
            messagebox.showwarning("Zero required", "Press ZERO ENCODERS before measuring wheel turns.")
            return
        self.send_command("E")
        self.after(350, lambda target=side: self._phase02_capture_side(target))

    def _phase02_capture_side(self, side):
        if not self.phase02_vars or self.encoder_calibration_zero is None:
            return
        current = int(self.motor_state.get(f"{side}_enc", 0))
        delta = current - int(self.encoder_calibration_zero[side])
        if abs(delta) < ENCODER_CALIBRATION_TURNS:
            messagebox.showwarning(
                "Encoder movement too small",
                f"{side.title()} encoder changed only {delta:+d} count. Check the wheel rotation or encoder link.",
            )
            return
        counts_per_rev = abs(delta) / float(ENCODER_CALIBRATION_TURNS)
        forward_sign = 1 if delta > 0 else -1
        self.encoder_calibration_measurements[side] = {
            "total_counts": delta,
            "counts_per_wheel_rev": counts_per_rev,
            "forward_sign": forward_sign,
        }
        sign_text = "+" if forward_sign > 0 else "-"
        self.phase02_vars[side].set(
            f"delta {delta:+d} / {counts_per_rev:.1f} count/rev / forward sign {sign_text}")
        self._phase02_update_result()

    def _phase02_update_result(self):
        left = self.encoder_calibration_measurements.get("left")
        right = self.encoder_calibration_measurements.get("right")
        if not left or not right:
            captured = "LEFT" if left else "RIGHT" if right else "none"
            self.phase02_vars["result"].set(
                f"Captured: {captured}. Measure the other wheel before saving.")
            return
        try:
            circumference = float(self.phase02_vars["circumference"].get().replace(",", "."))
            track = float(self.phase02_vars["track"].get().replace(",", "."))
        except ValueError:
            self.phase02_vars["result"].set("Geometry values must be numeric.")
            return
        left_mm_per_count = circumference / left["counts_per_wheel_rev"]
        right_mm_per_count = circumference / right["counts_per_wheel_rev"]
        quarter_turn_travel = math.pi * track / 4.0
        self.phase02_vars["result"].set(
            "Measurement ready:\n"
            f"LEFT  {left['counts_per_wheel_rev']:.1f} count/rev, {left_mm_per_count:.5f} mm/count\n"
            f"RIGHT {right['counts_per_wheel_rev']:.1f} count/rev, {right_mm_per_count:.5f} mm/count\n"
            f"1000 mm estimate: L {1000.0 / left_mm_per_count:.0f}, R {1000.0 / right_mm_per_count:.0f} counts\n"
            f"90 deg in-place wheel travel: {quarter_turn_travel:.1f} mm each side"
        )

    def _save_phase02_calibration(self):
        left = self.encoder_calibration_measurements.get("left")
        right = self.encoder_calibration_measurements.get("right")
        if not left or not right:
            messagebox.showwarning("Measurement incomplete", "Capture both LEFT and RIGHT wheel measurements first.")
            return
        try:
            circumference = float(self.phase02_vars["circumference"].get().replace(",", "."))
            track = float(self.phase02_vars["track"].get().replace(",", "."))
        except ValueError:
            messagebox.showerror("Invalid geometry", "Wheel circumference and track width must be numeric.")
            return
        if circumference <= 0.0 or track <= 0.0:
            messagebox.showerror("Invalid geometry", "Geometry values must be greater than zero.")
            return

        calibration = {
            "version": 1,
            "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "measurement_turns": ENCODER_CALIBRATION_TURNS,
            "wheel_circumference_mm": circumference,
            "track_width_mm": track,
            "left_counts_per_wheel_rev": left["counts_per_wheel_rev"],
            "right_counts_per_wheel_rev": right["counts_per_wheel_rev"],
            "left_forward_sign": left["forward_sign"],
            "right_forward_sign": right["forward_sign"],
        }
        try:
            self.motion_calibration_path.write_text(
                json.dumps(calibration, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        except OSError as exc:
            messagebox.showerror("Save failed", str(exc))
            return
        self.motion_calibration = calibration
        self._log(f"[PHASE 02] Motion calibration saved: {self.motion_calibration_path}\n", "ok")
        self._phase02_update_result()
        messagebox.showinfo(
            "Calibration saved",
            "Geometry and encoder scale saved. Live motor telemetry now uses calibrated wheel RPM and distance.",
        )

    def _close_phase02_calibration(self):
        if self.phase02_window is not None:
            try:
                self.phase02_window.destroy()
            except tk.TclError:
                pass
        self.phase02_window = None
        self.phase02_vars = {}

    def open_system_test_menu(self):
        if self.lowband_test_active:
            self._abort_lowband_rpm_test("Operator cancelled the test from SYSTEM TEST.")
            return
        popup = tk.Toplevel(self)
        popup.title("ARGOS System Test")
        popup.geometry("560x330")
        popup.resizable(False, False)
        popup.configure(bg="#071018")
        popup.transient(self)
        popup.grab_set()
        popup.grid_columnconfigure(0, weight=1)

        tk.Label(popup, text="Choose Test", bg="#071018", fg="#f6fbff",
                 font=("Segoe UI", 17, "bold")).grid(
                     row=0, column=0, sticky="w", padx=18, pady=(16, 4))
        tk.Label(
            popup,
            text="Both motor tests require every driven wheel to be raised and the chassis secured.",
            bg="#071018", fg="#8da4bc", justify="left", wraplength=520,
            font=("Segoe UI", 10),
        ).grid(row=1, column=0, sticky="ew", padx=18, pady=(0, 14))

        general = tk.Frame(popup, bg="#0a141f", highlightbackground="#203247", highlightthickness=1)
        general.grid(row=2, column=0, sticky="ew", padx=18, pady=(0, 10))
        general.grid_columnconfigure(0, weight=1)
        tk.Label(general, text="General health + ramp test", bg="#0a141f", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(10, 2))
        tk.Label(general, text="UART, I2C, IMU, ToF, motor safety, encoders and 50/60/70/80% ramp.",
                 bg="#0a141f", fg="#8da4bc", font=("Segoe UI", 9)).grid(
                     row=1, column=0, sticky="w", padx=12, pady=(0, 10))
        self._small_button(
            general, "RUN GENERAL",
            lambda: (popup.destroy(), self.start_system_test()),
        ).grid(row=0, column=1, rowspan=2, padx=12, pady=10)

        lowband = tk.Frame(popup, bg="#0a141f", highlightbackground="#203247", highlightthickness=1)
        lowband.grid(row=3, column=0, sticky="ew", padx=18, pady=(0, 10))
        lowband.grid_columnconfigure(0, weight=1)
        tk.Label(lowband, text="Phase 02 low-band RPM", bg="#0a141f", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w", padx=12, pady=(10, 2))
        tk.Label(lowband, text="Focused 50/52/54/56/58/60% characterization with calibrated wheel RPM.",
                 bg="#0a141f", fg="#8da4bc", font=("Segoe UI", 9)).grid(
                     row=1, column=0, sticky="w", padx=12, pady=(0, 10))
        self._small_button(
            lowband, "RUN PHASE 02",
            lambda: (popup.destroy(), self.start_lowband_rpm_test()),
        ).grid(row=0, column=1, rowspan=2, padx=12, pady=10)

        self._small_button(popup, "CLOSE", popup.destroy).grid(
            row=4, column=0, sticky="e", padx=18, pady=(0, 14))

    def start_lowband_rpm_test(self):
        if self.lowband_test_active:
            self._abort_lowband_rpm_test("Operator cancelled the test.")
            return
        if self.serial_port is None or not self.serial_port.is_open or not self.robot_ready:
            messagebox.showwarning("ARGOS offline", "Connect Wi-Fi/STM32 before starting the motor test.")
            return
        if not self._motion_calibration_ready():
            messagebox.showwarning(
                "Calibration missing",
                "Complete and save the Phase 02 ten-turn encoder calibration first.",
            )
            return
        if not messagebox.askokcancel(
                "Low-band motor test",
                "This test runs each motor separately at 50, 52, 54, 56, 58 and 60 percent "
                "in both directions. Lift every driven wheel fully off the ground, secure the "
                "chassis and keep cables/hands clear. The test takes about two minutes."):
            return

        self.stop_drive()
        self.lowband_test_active = True
        self.lowband_test_started_at = datetime.now()
        self.lowband_test_rows = []
        self.lowband_test_index = 0
        self.lowband_current_step = None
        self.lowband_start_snapshot = None
        self.lowband_test_steps = []
        for side in ("left", "right"):
            for direction_name, direction_sign in (("FWD", 1), ("REV", -1)):
                for pwm in LOWBAND_PWM_LEVELS:
                    self.lowband_test_steps.append({
                        "side": side,
                        "direction": direction_name,
                        "direction_sign": direction_sign,
                        "pwm": pwm,
                    })

        if self.system_test_button is not None:
            self.system_test_button.configure(text="CANCEL LOW-BAND TEST", bg="#ff5c7a")
        self.clear_console()
        self._log(
            "[LOW-BAND] Test started. Motors run one at a time; STM32 timeout remains active.\n",
            "warn",
        )
        self.send_command("S MPU 0")
        self.send_command("S TOF 0")
        self.after(350, self._run_next_lowband_step)

    def _run_next_lowband_step(self):
        if not self.lowband_test_active:
            return
        if self.lowband_test_index >= len(self.lowband_test_steps):
            self.stop_drive(abort_test=False)
            self.after(700, self._complete_lowband_rpm_test)
            return

        step = self.lowband_test_steps[self.lowband_test_index]
        self.lowband_current_step = step
        command = step["pwm"] * step["direction_sign"]
        left_command = command if step["side"] == "left" else 0
        right_command = command if step["side"] == "right" else 0
        self.left_drive_var.set(left_command)
        self.right_drive_var.set(right_command)
        self.drive_command_var.set(
            f"LOW-BAND {step['side'].upper()} {step['direction']} {step['pwm']}%")
        if not self.send_drive_command(allow_test=True, force_pwm=True):
            self._abort_lowband_rpm_test("Motor command could not be sent.")
            return
        self._log(
            f"[LOW-BAND] {self.lowband_test_index + 1:02d}/{len(self.lowband_test_steps)} "
            f"{step['side'].upper()} {step['direction']} {step['pwm']}% settling...\n",
            "tx",
        )
        self.after(LOWBAND_SETTLE_MS, self._lowband_capture_start)

    def _request_lowband_motor_snapshot(self, callback, baseline=None, attempt=0):
        if not self.lowband_test_active:
            return
        if baseline is None:
            baseline = self.motor_sample_seq
            if not self.send_command("E"):
                self._abort_lowband_rpm_test("Motor telemetry request failed.")
                return
        if self.motor_sample_seq > baseline:
            callback({
                "left": int(self.motor_state.get("left_enc", 0)),
                "right": int(self.motor_state.get("right_enc", 0)),
                "time": time.monotonic(),
            })
            return
        if attempt >= 20:
            self._abort_lowband_rpm_test("Timed out while waiting for motor telemetry.")
            return
        if attempt in (7, 14):
            self.send_command("E")
        self.after(
            100,
            lambda cb=callback, base=baseline, next_attempt=attempt + 1:
                self._request_lowband_motor_snapshot(cb, base, next_attempt),
        )

    def _lowband_capture_start(self):
        if not self.lowband_test_active:
            return
        self._request_lowband_motor_snapshot(self._lowband_store_start)

    def _lowband_store_start(self, snapshot):
        if not self.lowband_test_active:
            return
        self.lowband_start_snapshot = snapshot
        self.after(LOWBAND_MEASURE_MS, self._lowband_capture_end)

    def _lowband_capture_end(self):
        if not self.lowband_test_active:
            return
        self._request_lowband_motor_snapshot(self._lowband_store_end)

    def _lowband_store_end(self, snapshot):
        if not self.lowband_test_active or self.lowband_start_snapshot is None:
            return
        step = self.lowband_current_step
        start = self.lowband_start_snapshot
        elapsed_s = max(snapshot["time"] - start["time"], 0.001)
        left_delta = snapshot["left"] - start["left"]
        right_delta = snapshot["right"] - start["right"]
        active_delta = left_delta if step["side"] == "left" else right_delta
        counts_per_rev = float(self.motion_calibration[f"{step['side']}_counts_per_wheel_rev"])
        forward_sign = int(self.motion_calibration[f"{step['side']}_forward_sign"])
        wheel_rpm = (active_delta * forward_sign * 60.0) / (counts_per_rev * elapsed_s)
        row = {
            "side": step["side"].upper(),
            "direction": step["direction"],
            "pwm": step["pwm"],
            "elapsed_s": elapsed_s,
            "left_delta": left_delta,
            "right_delta": right_delta,
            "active_delta": active_delta,
            "wheel_rpm": wheel_rpm,
        }
        self.lowband_test_rows.append(row)
        self._log(
            f"[LOW-BAND] {row['side']} {row['direction']} {row['pwm']}% -> "
            f"{row['wheel_rpm']:+.2f} wheel RPM ({row['active_delta']:+d} counts)\n",
            "ok",
        )
        self.drive_active = False
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.send_command("x")
        self.lowband_test_index += 1
        self.lowband_start_snapshot = None
        self.after(LOWBAND_STOP_MS, self._run_next_lowband_step)

    def _complete_lowband_rpm_test(self):
        if not self.lowband_test_active:
            return
        self.lowband_test_active = False
        self.drive_active = False
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_command_var.set("STOPPED")
        report = self._build_lowband_report("COMPLETED")
        try:
            self.lowband_report_path.write_text(report, encoding="utf-8")
            self._log(f"[LOW-BAND] Report updated: {self.lowband_report_path}\n", "ok")
        except OSError as exc:
            self._log(f"[LOW-BAND] Report write failed: {exc}\n", "error")
        self._reset_lowband_button()
        self.send_command("S MPU 1")
        self.after(150, lambda: self.send_command("S TOF 1"))
        self._show_system_report(report, title="ARGOS Low-Band RPM Report")

    def _abort_lowband_rpm_test(self, reason):
        if not self.lowband_test_active:
            return
        self.lowband_test_active = False
        self.drive_active = False
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_command_var.set("TEST ABORTED")
        if self.serial_port is not None and self.serial_port.is_open:
            self.send_command("x")
        report = self._build_lowband_report(f"ABORTED - {reason}")
        try:
            self.lowband_report_path.write_text(report, encoding="utf-8")
        except OSError:
            pass
        self._reset_lowband_button()
        self._log(f"[LOW-BAND] Aborted: {reason}\n", "error")

    def _reset_lowband_button(self):
        if self.system_test_button is not None:
            self.system_test_button.configure(text="SYSTEM TEST", bg="#43b5ff")

    def _build_lowband_report(self, status):
        finished_at = datetime.now()
        lines = [
            "ARGOS PHASE 02 LOW-BAND RPM CHARACTERIZATION",
            f"Started : {self.lowband_test_started_at.strftime('%Y-%m-%d %H:%M:%S') if self.lowband_test_started_at else '-'}",
            f"Finished: {finished_at.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Status  : {status}",
            f"Link    : {self.connection_label}",
            f"Safety  : SAFE:{self.motor_state.get('safe', '-')}",
            "",
            "CALIBRATION",
            f"- Wheel circumference: {float(self.motion_calibration['wheel_circumference_mm']):.1f} mm",
            f"- Track width: {float(self.motion_calibration['track_width_mm']):.1f} mm",
            f"- Left: {float(self.motion_calibration['left_counts_per_wheel_rev']):.1f} count/rev",
            f"- Right: {float(self.motion_calibration['right_counts_per_wheel_rev']):.1f} count/rev",
            f"- Settle/measurement: {LOWBAND_SETTLE_MS}/{LOWBAND_MEASURE_MS} ms",
            "",
            "RESULTS",
            "- SIDE   DIR  PWM   WHEEL_RPM   ACTIVE_DELTA   ELAPSED",
        ]
        for row in self.lowband_test_rows:
            lines.append(
                f"- {row['side']:<6} {row['direction']:<3}  {row['pwm']:>3}%  "
                f"{row['wheel_rpm']:>+9.2f}  {row['active_delta']:>+13d}  {row['elapsed_s']:.3f}s"
            )
        lines.extend(["", "TRANSITION SLOPES (RPM per +2% PWM)"])
        for side in ("LEFT", "RIGHT"):
            for direction in ("FWD", "REV"):
                rows = [row for row in self.lowband_test_rows
                        if row["side"] == side and row["direction"] == direction]
                rows.sort(key=lambda row: row["pwm"])
                if len(rows) < 2:
                    continue
                slopes = []
                for previous, current in zip(rows, rows[1:]):
                    delta_rpm = abs(current["wheel_rpm"]) - abs(previous["wheel_rpm"])
                    slopes.append(f"{previous['pwm']}-{current['pwm']}:{delta_rpm:+.2f}")
                lines.append(f"- {side} {direction}: " + " | ".join(slopes))
        lines.extend([
            "",
            "NOTES",
            "- Positive RPM means robot-forward wheel direction; reverse samples are negative.",
            "- Test is no-load with wheels raised.",
            "- Panel keepalive is required; link loss leaves STM32's 1200 ms timeout as the final stop layer.",
        ])
        return "\n".join(lines) + "\n"

    def start_system_test(self):
        if self.lowband_test_active:
            messagebox.showwarning("Motor test active", "Cancel or finish the low-band RPM test first.")
            return
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning("Bagli degil", "Once ARGOS Wi-Fi veya COM baglantisini kur.")
            return
        if not self.robot_ready:
            messagebox.showwarning("STM32 bekleniyor", "Sistem testi icin once STM32 UART baglantisi kurulmalidir.")
            return
        if not messagebox.askokcancel(
                "Motor testi guvenligi",
                "Test iki motoru da ileri ve geri calistiracak. Tekerleri yerden kaldir veya robotun onunde ve arkasinda acik alan birak."):
            return

        self.system_test_active = True
        self.system_test_finishing = False
        self.system_test_started_at = datetime.now()
        self.system_test_lines = []
        self.clear_console()
        self._record_system_line("ARGOS SYSTEM TEST")
        self._record_system_line(f"Started: {self.system_test_started_at.strftime('%Y-%m-%d %H:%M:%S')}")
        self._record_system_line(f"Link: {self.connection_label}")
        self._log("[SYSTEM TEST] Basladi. Motor ramp + encoder tablosu rapora eklenecek.\n", "warn")

        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_active = False
        self.system_test_start_counts = {
            "left": self.motor_state.get("left_enc", 0),
            "right": self.motor_state.get("right_enc", 0),
        }
        steps = [
            (0, "S MPU 0"),
            (100, "S TOF 0"),
            (300, "x"),
            (550, "Z"),
            (950, "s"),
            (1850, "m"),
            (2450, "T"),
            (3450, "t"),
            (4650, "g"),
            (5150, "V"),
            (5800, "S SYNC 1"),
            (6200, "Y"),
        ]
        for delay_ms, command in steps:
            self.after(delay_ms, lambda cmd=command: self._send_system_test_command(cmd))
        self.system_test_timeout_job = self.after(60000, self._finish_system_test)

    def _send_system_test_command(self, command):
        if self.system_test_active:
            self.send_command(command)

    def _finish_system_test(self):
        if not self.system_test_active:
            return

        self.system_test_active = False
        self.system_test_finishing = False
        if self.system_test_timeout_job is not None:
            try:
                self.after_cancel(self.system_test_timeout_job)
            except tk.TclError:
                pass
            self.system_test_timeout_job = None
        finished_at = datetime.now()
        left_delta = self.motor_state.get("left_enc", 0) - self.system_test_start_counts.get("left", 0)
        right_delta = self.motor_state.get("right_enc", 0) - self.system_test_start_counts.get("right", 0)
        report = self._build_system_report(finished_at, left_delta, right_delta)
        self._write_system_report(report)
        self._show_system_report(report)
        if self.serial_port is not None and self.serial_port.is_open:
            self.send_command("S MPU 1")
            self.after(150, lambda: self.send_command("S TOF 1"))

    def _record_system_line(self, text):
        self.system_test_lines.append(text.rstrip())

    def _build_system_report(self, finished_at, left_delta, right_delta):
        ramp_pattern = re.compile(
            r"MOTOR RAMP\s*\|\s*(LEFT|RIGHT)_(FWD|REV)\s+([+-]?\d+)\s+\d+\s+\d+\s+"
            r"([+-]?\d+)\s+([+-]?\d+)\s+([+-]?\d+)\s+([+-]?\d+)",
            re.IGNORECASE,
        )
        balance_pattern = re.compile(
            r"MOTOR BALANCE\s*\|\s*(FWD|REV)\s+([+-]?\d+)\s+"
            r"L_DENC:\s*([+-]?\d+)\s+R_DENC:\s*([+-]?\d+)\s+"
            r"L_CPS:\s*(\d+)\s+R_CPS:\s*(\d+)\s+ERRx100:\s*([+-]?\d+)",
            re.IGNORECASE,
        )
        ramp_samples = []
        balance_samples = []
        for line in self.system_test_lines:
            ramp_match = ramp_pattern.search(line)
            if ramp_match:
                side, direction, command, l_denc, r_denc, l_rpm, r_rpm = ramp_match.groups()
                ramp_samples.append({
                    "side": side.upper(),
                    "direction": direction.upper(),
                    "command": int(command),
                    "left_delta": int(l_denc),
                    "right_delta": int(r_denc),
                    "left_rpm_x100": int(l_rpm),
                    "right_rpm_x100": int(r_rpm),
                    "wheel_rpm": (
                        self._calibrated_wheel_rpm("left", int(l_rpm))
                        if side.upper() == "LEFT"
                        else self._calibrated_wheel_rpm("right", int(r_rpm))
                    ),
                })
            balance_match = balance_pattern.search(line)
            if balance_match:
                direction, command, l_denc, r_denc, l_cps, r_cps, error_x100 = balance_match.groups()
                balance_samples.append({
                    "direction": direction.upper(),
                    "command": int(command),
                    "left_delta": int(l_denc),
                    "right_delta": int(r_denc),
                    "left_cps": int(l_cps),
                    "right_cps": int(r_cps),
                    "error_x100": int(error_x100),
                })

        checks = []
        checks.append(("UART", "PASS", "Serial TX/RX aktif; komut yanitlari alindi."))
        checks.append(("I2C", "PASS" if "DEVICE FOUND" in self.last_i2c_var.get() or "cihaz" in self.last_i2c_var.get().lower() else "CHECK",
                       self.last_i2c_var.get()))
        checks.append(("IMU", "PASS" if self.mpu_var.get() == "OK" else "CHECK", self.mpu_var.get()))
        checks.append(("ToF Left", "PASS" if "mm" in self.tof_left_var.get() or "OK" in self.tof_left_var.get() else "CHECK",
                       self.tof_left_var.get()))
        checks.append(("ToF Right", "PASS" if "mm" in self.tof_right_var.get() or "OK" in self.tof_right_var.get() else "CHECK",
                       self.tof_right_var.get()))
        checks.append(("Motor Safety", "PASS" if self.motor_state.get("safe") in ("STOP", "TIMEOUT_STOP") else "CHECK",
                       f"SAFE:{self.motor_state.get('safe', '-')}"))
        left_active = [abs(row["left_delta"]) for row in ramp_samples if row["side"] == "LEFT"]
        right_active = [abs(row["right_delta"]) for row in ramp_samples if row["side"] == "RIGHT"]
        if left_active and right_active:
            enc_ok = min(left_active) > 100 and min(right_active) > 100
            encoder_detail = (f"ramp samples L/R {len(left_active)}/{len(right_active)}, "
                              f"minimum active delta {min(left_active)}/{min(right_active)}")
        else:
            enc_ok = abs(left_delta) > 5 and abs(right_delta) > 5
            encoder_detail = f"legacy delta L {left_delta:+d}, R {right_delta:+d}"
        checks.append(("Encoders", "PASS" if enc_ok else "CHECK", encoder_detail))

        balance_eval = [row for row in balance_samples if abs(row["command"]) >= 60]
        if balance_eval:
            avg_error = sum(abs(row["error_x100"]) for row in balance_eval) / (100.0 * len(balance_eval))
            max_error = max(abs(row["error_x100"]) for row in balance_eval) / 100.0
            balance_status = "PASS" if max_error <= 8.0 else "CHECK"
            balance_detail = (f"60-80% average mismatch {avg_error:.1f}%, maximum {max_error:.1f}%; "
                              "+ means left faster")
        else:
            balance_status = "CHECK"
            balance_detail = "paired-wheel samples missing; rerun with updated STM32 firmware"
        checks.append(("Motor Balance", balance_status, balance_detail))

        lines = [
            "ARGOS SYSTEM TEST REPORT",
            f"Started : {self.system_test_started_at.strftime('%Y-%m-%d %H:%M:%S') if self.system_test_started_at else '-'}",
            f"Finished: {finished_at.strftime('%Y-%m-%d %H:%M:%S')}",
            f"Link    : {self.connection_label}",
            "",
            "SUMMARY",
        ]
        for name, status, detail in checks:
            lines.append(f"- {name}: {status} ({detail})")

        lines.extend([
            "",
            "LATEST TELEMETRY",
            f"- MPU: {self.mpu_var.get()} | {self.accel_var.get()} | {self.gyro_var.get()} | {self.tilt_var.get()}",
            f"- ToF: {self.tof_left_distance_var.get()} | {self.tof_right_distance_var.get()}",
            f"- Motors: L {self.left_motor_var.get()} enc {self.motor_state.get('left_enc', 0)} | "
            f"R {self.right_motor_var.get()} enc {self.motor_state.get('right_enc', 0)} | SAFE {self.motor_state.get('safe', '-')}",
        ])
        if ramp_samples:
            lines.extend(["", "PWM TO WHEEL RPM CHARACTERIZATION"])
            if self._motion_calibration_ready():
                lines.append(
                    f"- Calibration: L {float(self.motion_calibration['left_counts_per_wheel_rev']):.1f}, "
                    f"R {float(self.motion_calibration['right_counts_per_wheel_rev']):.1f} count/wheel revolution"
                )
                lines.append("- Values are no-load wheel RPM; positive is robot-forward direction.")
                lines.append("- SIDE   DIR  PWM    WHEEL_RPM   ACTIVE_DELTA")
                for row in ramp_samples:
                    active_delta = row["left_delta"] if row["side"] == "LEFT" else row["right_delta"]
                    lines.append(
                        f"- {row['side']:<6} {row['direction']:<3}  {abs(row['command']):>3}%  "
                        f"{float(row['wheel_rpm']):>+9.2f}  {active_delta:>+12d}"
                    )
            else:
                lines.append("- CHECK: wheel encoder calibration is missing; only raw firmware data is available.")
        if balance_samples:
            lines.extend(["", "MOTOR BALANCE TABLE", "- Raw count/s is retained for direct left/right comparison."])
            for row in balance_samples:
                lines.append(
                    f"- {row['direction']} {row['command']:+d}%: "
                    f"L {row['left_cps']} count/s, R {row['right_cps']} count/s, "
                    f"mismatch {row['error_x100'] / 100.0:+.1f}%"
                )
        lines.extend(["", "SERIAL LOG"])
        lines.extend(self.system_test_lines)
        return "\n".join(lines) + "\n"

    def _write_system_report(self, report):
        self.system_test_report_path.write_text(report, encoding="utf-8")
        self._log(f"[SYSTEM TEST] Rapor guncellendi: {self.system_test_report_path}\n", "ok")

    def _show_system_report(self, report, title="ARGOS System Test Report"):
        popup = tk.Toplevel(self)
        popup.title(title)
        popup.geometry("760x620")
        popup.configure(bg="#071018")
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(1, weight=1)

        tk.Label(popup, text=title, bg="#071018", fg="#f6fbff",
                 font=("Segoe UI", 16, "bold")).grid(row=0, column=0, sticky="w", padx=14, pady=(12, 8))

        body = tk.Frame(popup, bg="#050b12")
        body.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 12))
        body.grid_columnconfigure(0, weight=1)
        body.grid_rowconfigure(0, weight=1)
        text = tk.Text(body, bg="#050b12", fg="#d8e7f7", insertbackground="#d8e7f7",
                       relief="flat", wrap="word", font=("Consolas", 10), padx=12, pady=12)
        scroll = tk.Scrollbar(body, orient="vertical", command=text.yview)
        text.configure(yscrollcommand=scroll.set)
        text.grid(row=0, column=0, sticky="nsew")
        scroll.grid(row=0, column=1, sticky="ns")
        text.insert("1.0", report)
        text.configure(state="disabled")

        actions = tk.Frame(popup, bg="#071018")
        actions.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 14))
        actions.grid_columnconfigure(0, weight=1)
        self._small_button(actions, "SAVE AS", lambda: self._save_report_as(report)).grid(row=0, column=1, padx=(8, 0))
        self._small_button(actions, "CLOSE", popup.destroy).grid(row=0, column=2, padx=(8, 0))

    def _save_report_as(self, report):
        path = filedialog.asksaveasfilename(
            title="Raporu farkli kaydet",
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            initialfile=f"argos_system_test_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt",
        )
        if path:
            Path(path).write_text(report, encoding="utf-8")
            self._log(f"[SYSTEM TEST] Rapor farkli kaydedildi: {path}\n", "ok")

    def _read_connection_logbook(self):
        records = []
        try:
            with self.connection_logbook_path.open("r", encoding="utf-8") as handle:
                for row in handle:
                    try:
                        records.append(json.loads(row))
                    except (ValueError, TypeError):
                        continue
        except OSError:
            return "No connection events recorded yet.\n"

        records = records[-500:]
        if not records:
            return "No connection events recorded yet.\n"

        problem_codes = {
            "BROWNOUT", "PANIC", "INT_WDT", "TASK_WDT", "WDT",
            "WIFI_DISCONNECT", "WIFI_DISCONNECT_SUMMARY", "WIFI_TIMEOUT",
            "HEARTBEAT_TIMEOUT", "LINK_LOST",
            "TCP_DISCONNECTED", "PANEL_LINE",
        }
        reset_faults = ("BROWNOUT", "PANIC", "INT_WDT", "TASK_WDT", "WDT")
        problem_count = sum(
            1 for record in records
            if (record.get("code") in problem_codes or
                (record.get("code") == "BOOT" and record.get("detail") in reset_faults))
        )
        lines = [
            "ARGOS CONNECTION LOGBOOK",
            f"Events shown: {len(records)} | flagged: {problem_count}",
            "",
        ]
        for record in records:
            timestamp = record.get("received_at", "-")
            source = record.get("source", "-")
            code = record.get("code", "-")
            detail = record.get("detail", "")
            suffix = ""
            if source == "ESP":
                suffix = (f" | boot {record.get('boot_id', '-')}"
                          f" | up {record.get('uptime_ms', '-')} ms")
            lines.append(f"{timestamp} | {source:<5} | {code:<20} | {detail}{suffix}")
        return "\n".join(lines) + "\n"

    def _show_connection_logbook(self):
        popup = tk.Toplevel(self)
        popup.title("ARGOS Connection Logbook")
        popup.geometry("920x620")
        popup.configure(bg="#071018")
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(1, weight=1)

        tk.Label(popup, text="Connection Logbook", bg="#071018", fg="#f6fbff",
                 font=("Segoe UI", 16, "bold")).grid(
                     row=0, column=0, sticky="w", padx=14, pady=(12, 8))

        body = tk.Frame(popup, bg="#050b12")
        body.grid(row=1, column=0, sticky="nsew", padx=14, pady=(0, 12))
        body.grid_columnconfigure(0, weight=1)
        body.grid_rowconfigure(0, weight=1)
        text = tk.Text(body, bg="#050b12", fg="#d8e7f7", insertbackground="#d8e7f7",
                       relief="flat", wrap="none", font=("Consolas", 9), padx=12, pady=12)
        vertical = tk.Scrollbar(body, orient="vertical", command=text.yview)
        horizontal = tk.Scrollbar(body, orient="horizontal", command=text.xview)
        text.configure(yscrollcommand=vertical.set, xscrollcommand=horizontal.set)
        text.grid(row=0, column=0, sticky="nsew")
        vertical.grid(row=0, column=1, sticky="ns")
        horizontal.grid(row=1, column=0, sticky="ew")

        def refresh():
            text.configure(state="normal")
            text.delete("1.0", "end")
            text.insert("1.0", self._read_connection_logbook())
            text.configure(state="disabled")
            text.see("end")

        actions = tk.Frame(popup, bg="#071018")
        actions.grid(row=2, column=0, sticky="ew", padx=14, pady=(0, 14))
        actions.grid_columnconfigure(0, weight=1)
        self._small_button(actions, "REFRESH", refresh).grid(row=0, column=1, padx=(8, 0))
        self._small_button(actions, "CLOSE", popup.destroy).grid(row=0, column=2, padx=(8, 0))
        refresh()

    def send_command(self, command):
        if self.firmware_update_active:
            self._log(f"[STM32 UPDATE] Command blocked during update: {command}\n", "warn")
            return False
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning("Bagli degil", "Once ARGOS Wi-Fi veya COM baglantisini kur.")
            return False

        payload = (command + "\r\n").encode("ascii", errors="ignore")
        try:
            self.serial_port.write(payload)
            self.tx_count += len(payload)
            if self.system_test_active:
                self._record_system_line(f">> {command}")
            if command == "M":
                self.stream_var.set("TOGGLED")
            elif command == "D":
                self.tof_stream_var.set("TOGGLED")
            self._log(f">> {command}\n", "tx")
            return True
        except Exception as exc:
            self._log(f"TX error: {exc}\n", "error")
            self.disconnect_serial()
            return False

    def clear_console(self):
        self.console.delete("1.0", "end")
        self.console_autoscroll = True

    def _reader_loop(self):
        while self.reader_alive.is_set() and self.serial_port is not None:
            try:
                data = self.serial_port.readline()
                if data:
                    self.rx_queue.put(data.decode("utf-8", errors="replace"))
            except Exception as exc:
                if not self.intentional_disconnect:
                    self.rx_queue.put(f"\nRX error: {exc}\n")
                    self.rx_queue.put("__ARGOS_LINK_LOST__")
                break
            time.sleep(READ_POLL_SECONDS)

    def _drain_rx_queue(self):
        highest_esp_ack = None
        while True:
            try:
                text = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if text == "__ARGOS_LINK_LOST__":
                if self.link_loss_handled:
                    continue
                self.link_loss_handled = True
                self.reader_alive.clear()
                if self.serial_port is not None:
                    try:
                        self.serial_port.close()
                    except Exception:
                        pass
                self.serial_port = None
                self.drive_active = False
                self.keyboard_drive_active = False
                self.pressed_drive_keys.clear()
                self.last_keyboard_drive = (0, 0)
                self.robot_ready = False
                self.auto_stream_started = False
                self.left_drive_var.set(0)
                self.right_drive_var.set(0)
                self.drive_command_var.set("LINK LOST")
                self.status_var.set("LINK LOST - RETRYING")
                self.status_badge.configure(bg="#3a1923", fg="#ffb3c0")
                self.bridge_health_var.set("LOST")
                self.wifi_rssi_var.set("-")
                self._append_connection_event("PANEL", "LINK_LOST", "Reader or heartbeat failure")
                if self.lowband_test_active:
                    self._abort_lowband_rpm_test("Wi-Fi/STM32 link lost; STM32 timeout is stopping the motors.")
                if self.link_mode_var.get() == "Wi-Fi":
                    self.after(1200, self._auto_connect_wifi)
                continue
            self.last_bridge_rx_time = time.monotonic()
            upper = text.upper()
            if "ESP LOG |" in text:
                event_id = self._handle_esp_log_line(text)
                if event_id is not None:
                    highest_esp_ack = max(highest_esp_ack or 0, event_id)
            if upper.startswith("ESP PONG |") or upper.startswith("ESP STATUS |"):
                self._update_bridge_metrics(text)
            if "ARGOS BRIDGE" in upper and "STM:WAITING" in upper:
                self.status_var.set("WIFI CONNECTED - STM UART WAITING")
                self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
            elif "ARGOS BRIDGE" in upper and "STM:ONLINE" in upper:
                self._mark_robot_online()
            elif any(marker in upper for marker in (
                    "ARGOS LINK | STM:ONLINE", "MOTOR |", "MPU OK",
                    "TOF RANGE", "ARGOS TEST PLATFORM")):
                self._mark_robot_online()
            self.rx_count += len(text)
            self._update_metrics(text)
            if (self.system_test_active and not self.system_test_finishing and
                    "motor ramp test bitti" in text.lower()):
                self.system_test_finishing = True
                self.after(150, lambda: self._send_system_test_command("E"))
                self.after(1800, self._finish_system_test)
            if self.system_test_active:
                for line in text.splitlines():
                    if line.strip():
                        self._record_system_line(line)
            self._log(text, self._tag_for_line(text))

        if highest_esp_ack is not None:
            self._send_bridge_control(f"@ESP ACK {highest_esp_ack}")
        self.after(40, self._drain_rx_queue)

    def _update_bridge_metrics(self, text):
        rssi_match = re.search(r"rssi:(-?\d+)", text, re.IGNORECASE)
        heap_match = re.search(r"heap:(\d+)", text, re.IGNORECASE)
        if rssi_match:
            rssi = int(rssi_match.group(1))
            self.wifi_rssi_var.set(f"{rssi} dBm" if rssi != 0 else "AP mode")
        if heap_match:
            heap_kb = int(heap_match.group(1)) // 1024
            self.bridge_health_var.set(f"OK / {heap_kb} KB")

    def _mark_robot_online(self):
        if self.robot_ready:
            return
        self.robot_ready = True
        if self.drive_command_var.get() in ("LINK LOST", "LINK REQUIRED"):
            self.drive_command_var.set("STOPPED")
        self.status_var.set("ARGOS ONLINE - WIFI + STM32")
        self.status_badge.configure(bg="#123525", fg="#98ffc2")
        self._log("[ROBOT ONLINE] ESP ve STM32 UART hatti hazir.\n", "ok")
        if not self.auto_stream_started:
            self.auto_stream_started = True
            self.after(100, lambda: self.send_command("S MPU 1"))
            self.after(250, lambda: self.send_command("S TOF 1"))
        if self.pressed_drive_keys:
            self.last_keyboard_drive = (0, 0)
            self.after(300, self._apply_keyboard_drive)

    def _update_metrics(self, text):
        lower = text.lower()
        if self.system_test_active and "encoder sayaclari sifirlandi" in lower:
            self.system_test_start_counts = {"left": 0, "right": 0}
        if "toplam" in lower and "cihaz" in lower:
            self.last_i2c_var.set(text.strip())
        elif "bulundu: 0x" in lower:
            self.last_i2c_var.set("DEVICE FOUND")

        if "mpu ok" in lower:
            self.mpu_var.set("OK")
            if "int:" in lower:
                self.int_var.set(text.strip().split("INT:", 1)[-1])
            self._parse_imu_sample(text)
        elif "uyumlu okuma deneniyor" in lower:
            self.mpu_var.set("TRYING")
        elif "mpu6050 yok" in lower or "who_am_i taninmiyor" in lower:
            self.mpu_var.set("CHECK")

        if "tof assign" in lower:
            if "left 0x30:ok" in lower:
                self.tof_left_var.set("0x30 OK")
            elif "left 0x30:yok" in lower:
                self.tof_left_var.set("NOT FOUND")
            if "right 0x31:ok" in lower:
                self.tof_right_var.set("0x31 OK")
            elif "right 0x31:yok" in lower:
                self.tof_right_var.set("NOT FOUND")

        tof_match = re.search(r"TOF\s+(LEFT|RIGHT).*STATUS:(OK|YOK).*INT:(\d)", text, re.IGNORECASE)
        if tof_match:
            side, status, int_state = tof_match.groups()
            label = f"{status} INT:{int_state}"
            range_match = re.search(r"RANGE:(OK|YOK)\s+(\d+)\s+mm", text, re.IGNORECASE)
            if range_match and range_match.group(1).upper() == "OK":
                label = f"{range_match.group(2)} mm"
            if side.upper() == "LEFT":
                self.tof_left_var.set(label)
                if range_match and range_match.group(1).upper() == "OK":
                    self.tof_left_distance_var.set(f"LEFT {range_match.group(2)} mm")
            else:
                self.tof_right_var.set(label)
                if range_match and range_match.group(1).upper() == "OK":
                    self.tof_right_distance_var.set(f"RIGHT {range_match.group(2)} mm")

        if "surekli akis: acik" in lower:
            if "tof" in lower:
                self.tof_stream_var.set("ON")
            else:
                self.stream_var.set("ON")
        elif "surekli akis: kapali" in lower:
            if "tof" in lower:
                self.tof_stream_var.set("OFF")
            else:
                self.stream_var.set("OFF")

        if "tof range" in lower:
            self._parse_tof_sample(text)

        if "motor |" in lower:
            self._parse_motor_sample(text)

        if "gyro offset ok" in lower:
            self.gyro_var.set("gyro calibrated")

    def _parse_motor_sample(self, text):
        match = re.search(
            r"MOTOR\s*\|\s*LEFT:(-?\d+)\s+RPWM:(\d+)\s+LPWM:(\d+)\s+ENC:(-?\d+)"
            r"(?:\s+DENC:(-?\d+)\s+RPMx100:(-?\d+))?\s+AB:(\d+)\s*\|\s*"
            r"RIGHT:(-?\d+)\s+RPWM:(\d+)\s+LPWM:(\d+)\s+ENC:(-?\d+)"
            r"(?:\s+DENC:(-?\d+)\s+RPMx100:(-?\d+))?\s+AB:(\d+)"
            r"(?:\s*\|\s*SAFE:([A-Z_]+))?",
            text,
            re.IGNORECASE,
        )
        if not match:
            return

        groups = match.groups()
        left_rpm = int(groups[5] or 0) / 100.0
        right_rpm = int(groups[12] or 0) / 100.0
        left_distance_mm = 0.0
        right_distance_mm = 0.0
        calibrated_units = False
        left_counts_per_rev = self.motion_calibration.get("left_counts_per_wheel_rev")
        right_counts_per_rev = self.motion_calibration.get("right_counts_per_wheel_rev")
        left_forward_sign = self.motion_calibration.get("left_forward_sign")
        right_forward_sign = self.motion_calibration.get("right_forward_sign")
        circumference_mm = self.motion_calibration.get("wheel_circumference_mm")
        if (left_counts_per_rev and right_counts_per_rev and left_forward_sign and
                right_forward_sign and circumference_mm):
            left_counts_per_rev = float(left_counts_per_rev)
            right_counts_per_rev = float(right_counts_per_rev)
            circumference_mm = float(circumference_mm)
            left_rpm = self._calibrated_wheel_rpm("left", int(groups[5] or 0))
            right_rpm = self._calibrated_wheel_rpm("right", int(groups[12] or 0))
            left_distance_mm = (int(groups[3]) * int(left_forward_sign) * circumference_mm /
                                left_counts_per_rev)
            right_distance_mm = (int(groups[10]) * int(right_forward_sign) * circumference_mm /
                                 right_counts_per_rev)
            calibrated_units = True
        sync_match = re.search(
            r"SYNC:([A-Z_]+)\s+ERRx100:([+-]?\d+)\s+TRIM:([+-]?\d+)\s+APPLIED:([+-]?\d+)/([+-]?\d+)",
            text,
            re.IGNORECASE,
        )
        if sync_match:
            sync_state, sync_error, sync_trim, left_applied, right_applied = sync_match.groups()
        else:
            sync_state = self.motor_state.get("sync", "IDLE")
            sync_error = int(self.motor_state.get("sync_error", 0.0) * 100.0)
            sync_trim = self.motor_state.get("sync_trim", 0)
            left_applied = int(groups[0])
            right_applied = int(groups[7])
        control_match = re.search(
            r"CTRL:(PWM|PI)\s+TARGET_RPMx100:([+-]?\d+)/([+-]?\d+)\s+"
            r"RAMP_RPMx100:([+-]?\d+)/([+-]?\d+)\s+"
            r"MEAS_RPMx100:([+-]?\d+)/([+-]?\d+)\s+"
            r"I_x100:([+-]?\d+)/([+-]?\d+)",
            text,
            re.IGNORECASE,
        )
        if control_match:
            (control_mode, left_target, right_target, left_ramped, right_ramped,
             left_measured, right_measured, left_pi_i, right_pi_i) = control_match.groups()
            if control_mode.upper() == "PI":
                left_rpm = int(left_measured) / 100.0
                right_rpm = int(right_measured) / 100.0
                calibrated_units = True
        else:
            control_mode = self.motor_state.get("control", "PWM")
            left_target = int(self.motor_state.get("left_target_rpm", 0.0) * 100.0)
            right_target = int(self.motor_state.get("right_target_rpm", 0.0) * 100.0)
            left_ramped = int(self.motor_state.get("left_ramped_rpm", 0.0) * 100.0)
            right_ramped = int(self.motor_state.get("right_ramped_rpm", 0.0) * 100.0)
            left_pi_i = int(self.motor_state.get("left_pi_i", 0.0) * 100.0)
            right_pi_i = int(self.motor_state.get("right_pi_i", 0.0) * 100.0)
        self.motor_state.update({
            "left_pct": int(groups[0]),
            "left_pwm_r": int(groups[1]),
            "left_pwm_l": int(groups[2]),
            "left_enc": int(groups[3]),
            "left_delta": int(groups[4] or 0),
            "left_rpm": left_rpm,
            "left_ab": int(groups[6]),
            "right_pct": int(groups[7]),
            "right_pwm_r": int(groups[8]),
            "right_pwm_l": int(groups[9]),
            "right_enc": int(groups[10]),
            "right_delta": int(groups[11] or 0),
            "right_rpm": right_rpm,
            "left_distance_mm": left_distance_mm,
            "right_distance_mm": right_distance_mm,
            "right_ab": int(groups[13]),
            "safe": groups[14] or self.motor_state.get("safe", "STOP"),
            "sync": sync_state.upper(),
            "sync_error": int(sync_error) / 100.0,
            "sync_trim": int(sync_trim),
            "left_applied": int(left_applied),
            "right_applied": int(right_applied),
            "control": control_mode.upper(),
            "left_target_rpm": int(left_target) / 100.0,
            "right_target_rpm": int(right_target) / 100.0,
            "left_ramped_rpm": int(left_ramped) / 100.0,
            "right_ramped_rpm": int(right_ramped) / 100.0,
            "left_pi_i": int(left_pi_i) / 100.0,
            "right_pi_i": int(right_pi_i) / 100.0,
        })
        self.motor_sample_seq += 1
        rpm_suffix = "wheel rpm" if calibrated_units else "rpm est."
        if control_mode.upper() == "PI":
            self.left_motor_var.set(
                f"T {int(left_target) / 100.0:+.0f} / M {left_rpm:+.1f} rpm / {int(groups[0]):+d}%")
            self.right_motor_var.set(
                f"T {int(right_target) / 100.0:+.0f} / M {right_rpm:+.1f} rpm / {int(groups[7]):+d}%")
        else:
            self.left_motor_var.set(f"{int(groups[0])}% / {left_rpm:+.1f} {rpm_suffix}")
            self.right_motor_var.set(f"{int(groups[7])}% / {right_rpm:+.1f} {rpm_suffix}")
        self._draw_motor_field()

    def _parse_tof_sample(self, text):
        match = re.search(
            r"TOF RANGE\s*\|\s*LEFT:(OK|YOK)\s+(\d+)\s+mm\s*\|\s*"
            r"RIGHT:(OK|YOK)\s+(\d+)\s+mm\s*\|\s*"
            r"YAW L:(-?\d+) R:(-?\d+)\s*\|\s*UP:(-?\d+)\s*\|\s*BASE:(\d+)",
            text,
            re.IGNORECASE,
        )
        if not match:
            return

        left_status, left_mm, right_status, right_mm, left_yaw, right_yaw, up_deg, baseline = match.groups()
        sample = {
            "left_ok": left_status.upper() == "OK",
            "right_ok": right_status.upper() == "OK",
            "left_mm": int(left_mm),
            "right_mm": int(right_mm),
            "left_yaw": int(left_yaw),
            "right_yaw": int(right_yaw),
            "up_deg": int(up_deg),
            "baseline": int(baseline),
        }
        self.tof_samples.append(sample)

        self.tof_left_distance_var.set(
            f"LEFT {sample['left_mm']} mm" if sample["left_ok"] else "LEFT no range"
        )
        self.tof_right_distance_var.set(
            f"RIGHT {sample['right_mm']} mm" if sample["right_ok"] else "RIGHT no range"
        )
        if sample["left_ok"]:
            self.tof_left_var.set(f"{sample['left_mm']} mm")
        elif self.tof_left_var.get() != "NOT FOUND":
            self.tof_left_var.set("NO RANGE")
        if sample["right_ok"]:
            self.tof_right_var.set(f"{sample['right_mm']} mm")
        elif self.tof_right_var.get() != "NOT FOUND":
            self.tof_right_var.set("NO RANGE")
        self._draw_tof_field()

    def _parse_imu_sample(self, text):
        sample = None
        scaled = re.search(
            r"ACC mg X:\s*(-?\d+) Y:\s*(-?\d+) Z:\s*(-?\d+).*?"
            r"GYR cdps X:\s*(-?\d+) Y:\s*(-?\d+) Z:\s*(-?\d+).*?"
            r"TEMP cC:\s*(-?\d+)",
            text,
        )
        if scaled:
            ax, ay, az, gx, gy, gz, temp = [int(value) for value in scaled.groups()]
            sample = {
                "ax_g": ax / 1000.0,
                "ay_g": ay / 1000.0,
                "az_g": az / 1000.0,
                "gx_dps": gx / 100.0,
                "gy_dps": gy / 100.0,
                "gz_dps": gz / 100.0,
                "temp_c": temp / 100.0,
            }
        else:
            raw = re.search(
                r"ACC raw X:\s*(-?\d+) Y:\s*(-?\d+) Z:\s*(-?\d+).*?"
                r"GYR raw X:\s*(-?\d+) Y:\s*(-?\d+) Z:\s*(-?\d+).*?"
                r"TEMP raw:\s*(-?\d+)",
                text,
            )
            if raw:
                ax, ay, az, gx, gy, gz, temp = [int(value) for value in raw.groups()]
                sample = {
                    "ax_g": ax / 16384.0,
                    "ay_g": ay / 16384.0,
                    "az_g": az / 16384.0,
                    "gx_dps": gx / 131.0,
                    "gy_dps": gy / 131.0,
                    "gz_dps": gz / 131.0,
                    "temp_c": (temp / 340.0) + 36.53,
                }

        if sample is None:
            return

        sample["roll"] = math.degrees(math.atan2(sample["ay_g"], sample["az_g"]))
        sample["pitch"] = math.degrees(
            math.atan2(-sample["ax_g"], math.sqrt(sample["ay_g"] ** 2 + sample["az_g"] ** 2))
        )
        self.imu_samples.append(sample)

        self.accel_var.set(
            f"ACC {sample['ax_g']:+.2f} / {sample['ay_g']:+.2f} / {sample['az_g']:+.2f} g"
        )
        self.gyro_var.set(
            f"GYR {sample['gx_dps']:+.1f} / {sample['gy_dps']:+.1f} / {sample['gz_dps']:+.1f} deg/s"
        )
        self.temp_var.set(f"TEMP {sample['temp_c']:.1f} C")
        self.tilt_var.set(f"roll {sample['roll']:+.1f} / pitch {sample['pitch']:+.1f}")
        self._redraw_imu()

    def _redraw_imu(self):
        self._draw_horizon()
        self._draw_trace()

    def _draw_motor_field(self):
        canvas = getattr(self, "motor_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 420)
        height = max(canvas.winfo_height(), 105)
        left = 48
        right = width - 48
        mid_x = width / 2
        top = 22
        row_gap = 42

        canvas.create_line(mid_x, 12, mid_x, height - 12, fill="#17283a")
        self._draw_motor_bar(canvas, "LEFT", self.motor_state["left_pct"],
                             self.motor_state["left_enc"], self.motor_state.get("left_ab", 0),
                             self.motor_state.get("left_delta", 0), self.motor_state.get("left_rpm", 0.0),
                             self.motor_state.get("left_distance_mm", 0.0),
                             self.motor_state.get("left_target_rpm", 0.0),
                             left, top, mid_x - 24, "#72f0c0")
        self._draw_motor_bar(canvas, "RIGHT", self.motor_state["right_pct"],
                             self.motor_state["right_enc"], self.motor_state.get("right_ab", 0),
                             self.motor_state.get("right_delta", 0), self.motor_state.get("right_rpm", 0.0),
                             self.motor_state.get("right_distance_mm", 0.0),
                             self.motor_state.get("right_target_rpm", 0.0),
                             mid_x + 24, top, right, "#8bc7ff")
        if self.motor_state.get("control") == "PI":
            sync_text = (f"CTRL PI  ramp {self.motor_state.get('left_ramped_rpm', 0.0):+.0f}/"
                         f"{self.motor_state.get('right_ramped_rpm', 0.0):+.0f} rpm  "
                         f"I {self.motor_state.get('left_pi_i', 0.0):+.1f}/"
                         f"{self.motor_state.get('right_pi_i', 0.0):+.1f}%")
        else:
            sync_text = (f"CTRL PWM  SYNC {self.motor_state.get('sync', '-')}  "
                         f"err {self.motor_state.get('sync_error', 0.0):+.1f}%  "
                         f"trim {self.motor_state.get('sync_trim', 0):+d}%  "
                         f"applied {self.motor_state.get('left_applied', 0):+d}/"
                         f"{self.motor_state.get('right_applied', 0):+d}")
        canvas.create_text(width / 2, top + row_gap + 19,
                           text=f"SAFE {self.motor_state.get('safe', '-')}    {sync_text}",
                           fill="#627891", font=("Segoe UI", 8))

    def _draw_motor_bar(self, canvas, name, percent, encoder, ab_state, delta, rpm, distance_mm,
                        target_rpm,
                        x1, y, x2, color):
        center = (x1 + x2) / 2
        bar_y = y + 22
        half = (x2 - x1) / 2
        fill = color if percent >= 0 else "#ff8a65"
        end = center + (max(min(percent, 100), -100) / 100.0) * half

        canvas.create_text(x1, y, text=name, anchor="w", fill="#e8f1ff",
                           font=("Segoe UI", 9, "bold"))
        rpm_text = (f"T{target_rpm:+.0f} M{rpm:+.1f} rpm" if self.motor_state.get("control") == "PI"
                    else f"{rpm:+.1f} wheel rpm")
        canvas.create_text(x2, y, text=f"{percent:+d}%  {rpm_text}  d{delta:+d}",
                           anchor="e", fill="#8da4bc", font=("Consolas", 9))
        distance_text = f"  {distance_mm / 10.0:+.1f} cm" if self._motion_calibration_ready() else ""
        canvas.create_text(x2, y + 15, text=f"enc {encoder}  AB {ab_state}{distance_text}",
                           anchor="e", fill="#627891", font=("Consolas", 8))
        canvas.create_rectangle(x1, bar_y - 6, x2, bar_y + 6, fill="#0f1c29", outline="#203247")
        canvas.create_line(center, bar_y - 11, center, bar_y + 11, fill="#314052")
        if percent != 0:
            canvas.create_rectangle(min(center, end), bar_y - 5, max(center, end), bar_y + 5,
                                    fill=fill, outline="")

    def _draw_tof_field(self):
        canvas = getattr(self, "tof_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 420)
        height = max(canvas.winfo_height(), 120)
        cx = width / 2
        base_y = height - 22
        scale = min((height - 44) / 1200.0, (width - 80) / 1200.0)
        latest = self.tof_samples[-1] if self.tof_samples else None
        baseline = latest["baseline"] if latest else 70
        up_deg = latest["up_deg"] if latest else 5

        canvas.create_line(36, base_y, width - 36, base_y, fill="#17283a")
        for mm in (300, 600, 900):
            y = base_y - mm * scale
            canvas.create_line(40, y, width - 40, y, fill="#0f1c29")
            canvas.create_text(44, y - 7, text=f"{mm}mm", anchor="w", fill="#627891",
                               font=("Segoe UI", 8))

        left_sensor = (cx - (baseline / 2) * scale, base_y)
        right_sensor = (cx + (baseline / 2) * scale, base_y)
        canvas.create_rectangle(cx - 55, base_y - 12, cx + 55, base_y + 10,
                                fill="#0f1c29", outline="#203247")
        canvas.create_text(cx, base_y, text="ARGOS FRONT", fill="#8da4bc",
                           font=("Segoe UI", 8, "bold"))

        if latest is None:
            canvas.create_text(cx, height / 2, text="TOF STREAM bekleniyor", fill="#627891",
                               font=("Segoe UI", 10, "bold"))
            self._draw_tof_beam(canvas, left_sensor, -15, 650, False, "#9cd6ff", scale, up_deg)
            self._draw_tof_beam(canvas, right_sensor, 15, 650, False, "#ffb38f", scale, up_deg)
            return

        self._draw_tof_beam(canvas, left_sensor, latest["left_yaw"], latest["left_mm"],
                            latest["left_ok"], "#9cd6ff", scale, up_deg)
        self._draw_tof_beam(canvas, right_sensor, latest["right_yaw"], latest["right_mm"],
                            latest["right_ok"], "#ffb38f", scale, up_deg)

    def _draw_tof_beam(self, canvas, origin, yaw_deg, distance_mm, ok, color, scale, up_deg):
        ox, oy = origin
        shown_mm = distance_mm if ok else 700
        horizontal_mm = shown_mm * math.cos(math.radians(up_deg))
        yaw = math.radians(yaw_deg)
        ex = ox + math.sin(yaw) * horizontal_mm * scale
        ey = oy - math.cos(yaw) * horizontal_mm * scale
        beam_color = color if ok else "#314052"
        width = 4 if ok else 2

        canvas.create_line(ox, oy, ex, ey, fill=beam_color, width=width)
        canvas.create_oval(ox - 5, oy - 5, ox + 5, oy + 5, fill=color, outline="")
        if ok:
            canvas.create_oval(ex - 8, ey - 8, ex + 8, ey + 8, fill=color, outline="")
            label = f"{distance_mm} mm"
        else:
            label = "no range"
        canvas.create_text(ex, ey - 16, text=label, fill=beam_color,
                           font=("Segoe UI", 9, "bold"))
        canvas.create_text(ox, oy + 16, text=f"{yaw_deg:+d} deg", fill="#627891",
                           font=("Segoe UI", 8))

    def _draw_horizon(self):
        canvas = getattr(self, "horizon_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 220)
        height = max(canvas.winfo_height(), 105)
        cx = width / 2
        cy = height / 2
        radius = min(width, height) * 0.38

        canvas.create_oval(cx - radius, cy - radius, cx + radius, cy + radius,
                           outline="#203247", width=2)
        canvas.create_line(cx - radius, cy, cx + radius, cy, fill="#203247")
        canvas.create_line(cx, cy - radius, cx, cy + radius, fill="#203247")

        if not self.imu_samples:
            canvas.create_text(cx, cy, text="waiting for IMU", fill="#627891",
                               font=("Segoe UI", 10, "bold"))
            return

        sample = self.imu_samples[-1]
        roll = max(min(sample["roll"], 45.0), -45.0)
        pitch = max(min(sample["pitch"], 45.0), -45.0)
        dot_x = cx + (roll / 45.0) * radius
        dot_y = cy + (pitch / 45.0) * radius

        canvas.create_oval(dot_x - 9, dot_y - 9, dot_x + 9, dot_y + 9,
                           fill="#23c483", outline="")
        canvas.create_text(cx, height - 18, text="roll / pitch", fill="#8da4bc",
                           font=("Segoe UI", 9))

    def _draw_trace(self):
        canvas = getattr(self, "trace_canvas", None)
        if canvas is None:
            return
        canvas.delete("all")
        width = max(canvas.winfo_width(), 320)
        height = max(canvas.winfo_height(), 105)
        left = 42
        right = width - 12
        top = 12
        bottom = height - 26
        mid = (top + bottom) / 2

        canvas.create_rectangle(left, top, right, bottom, outline="#203247")
        canvas.create_line(left, mid, right, mid, fill="#17283a")
        canvas.create_text(left, height - 10, text="-2g", fill="#627891", font=("Segoe UI", 8))
        canvas.create_text(right - 18, height - 10, text="+2g", fill="#627891", font=("Segoe UI", 8))

        if len(self.imu_samples) < 2:
            canvas.create_text(width / 2, mid, text="stream IMU to draw traces", fill="#627891",
                               font=("Segoe UI", 10, "bold"))
            return

        samples = list(self.imu_samples)
        series = [
            ("ax_g", "#72f0c0", "AX"),
            ("ay_g", "#8bc7ff", "AY"),
            ("az_g", "#ffc66d", "AZ"),
        ]
        for key, color, label in series:
            points = []
            for index, sample in enumerate(samples):
                x = left + (index / (len(samples) - 1)) * (right - left)
                value = max(min(sample[key], 2.0), -2.0)
                y = mid - (value / 2.0) * ((bottom - top) / 2)
                points.extend((x, y))
            canvas.create_line(*points, fill=color, width=2, smooth=True)

        for offset, (_key, color, label) in enumerate(series):
            canvas.create_text(left + 18 + offset * 42, top + 12, text=label,
                               fill=color, font=("Segoe UI", 8, "bold"))

    def _tag_for_line(self, text):
        lower = text.lower()
        if "ok" in lower or "bulundu" in lower or "hazir" in lower:
            return "ok"
        if "hata" in lower or "error" in lower or "yok" in lower:
            return "error"
        if "kontrol" in lower or "reset" in lower or "deneniyor" in lower:
            return "warn"
        return "rx"

    def _log(self, text, tag):
        _, bottom = self.console.yview()
        self.console_autoscroll = bottom >= 0.995
        self.console.insert("end", text, tag)
        if self.console_autoscroll:
            self.console.see("end")

    def _on_console_mousewheel(self, _event=None):
        self.after(1, self._refresh_console_autoscroll)

    def _refresh_console_autoscroll(self, _event=None):
        if getattr(self, "console", None) is not None:
            _, bottom = self.console.yview()
            self.console_autoscroll = bottom >= 0.995

    def on_close(self):
        self.disconnect_serial()
        self.destroy()


if __name__ == "__main__":
    app = ArgosControlPanel()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
