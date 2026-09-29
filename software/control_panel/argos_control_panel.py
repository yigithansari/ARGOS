import math
import queue
import re
import socket
import threading
import time
import tkinter as tk
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
        for broadcast in ("255.255.255.255", "192.168.137.255"):
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
        self.left_motor_var = tk.StringVar(value="0% / 0.0 rpm")
        self.right_motor_var = tk.StringVar(value="0% / 0.0 rpm")
        self.int_var = tk.StringVar(value="-")
        self.stream_var = tk.StringVar(value="OFF")
        self.tof_stream_var = tk.StringVar(value="OFF")
        self.accel_var = tk.StringVar(value="0.00 g / 0.00 g / 0.00 g")
        self.gyro_var = tk.StringVar(value="0.00 / 0.00 / 0.00 deg/s")
        self.temp_var = tk.StringVar(value="-")
        self.tilt_var = tk.StringVar(value="roll 0.0 / pitch 0.0")
        self.tof_left_distance_var = tk.StringVar(value="LEFT -")
        self.tof_right_distance_var = tk.StringVar(value="RIGHT -")
        self.left_drive_var = tk.IntVar(value=0)
        self.right_drive_var = tk.IntVar(value=0)
        self.drive_speed_var = tk.IntVar(value=60)
        self.drive_command_var = tk.StringVar(value="STOPPED")
        self.drive_active = False
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
            "safe": "STOP",
        }
        self.custom_command_var = tk.StringVar()

        self._configure_style()
        self._build_ui()
        self.refresh_ports()
        self.after(40, self._drain_rx_queue)
        self.after(900, self._auto_connect_wifi)
        self.after(500, self._motor_keepalive)

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

        subtitle = tk.Label(parent, text="Robot test ve kontrol paneli", bg="#0a141f",
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
        self._drive_slider(drive, "Left", self.left_drive_var).pack(fill="x", pady=(0, 6))
        self._drive_slider(drive, "Right", self.right_drive_var).pack(fill="x", pady=(0, 6))
        drive_buttons = tk.Frame(drive, bg="#0a141f")
        drive_buttons.pack(fill="x")
        self._small_button(drive_buttons, "SEND", self.send_drive_command).pack(side="left", fill="x", expand=True)
        self._small_button(drive_buttons, "STOP", self.stop_drive).pack(side="left", fill="x", expand=True, padx=(8, 0))
        self._small_button(drive_buttons, "PIN TEST", lambda: self.send_command("K")).pack(side="left", fill="x", expand=True, padx=(8, 0))
        self._small_button(drive, "MOTOR RAMP TEST", lambda: self.send_command("Y")).pack(fill="x", pady=(8, 0))

        self._section_label(parent, "Live Sensors")
        sensor_live = tk.Frame(parent, bg="#0a141f")
        sensor_live.pack(fill="x", padx=16, pady=(0, 8))
        self._small_button(sensor_live, "MPU LIVE", self.toggle_mpu_live).pack(side="left", fill="x", expand=True)
        self._small_button(sensor_live, "TOF LIVE", self.toggle_tof_live).pack(side="left", fill="x", expand=True, padx=(8, 0))

        sensor_once = tk.Frame(parent, bg="#0a141f")
        sensor_once.pack(fill="x", padx=16, pady=(0, 10))
        self._small_button(sensor_once, "MPU READ", lambda: self.send_command("m")).pack(side="left", fill="x", expand=True)
        self._small_button(sensor_once, "TOF CHECK", lambda: self.send_command("t")).pack(side="left", fill="x", expand=True, padx=(8, 0))

        self._section_label(parent, "Validation")
        self._action_button(parent, "SYSTEM TEST", self.start_system_test, "#43b5ff").pack(fill="x", padx=16, pady=(0, 10))

        hint = tk.Label(parent, text="MPU/ToF canli akisi ac-kapat.\nSYSTEM TEST son rapor dosyasini gunceller.",
                        bg="#0a141f", fg="#627891", justify="left", font=("Segoe UI", 9))
        hint.pack(anchor="w", padx=18, pady=(10, 0))
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

        tk.Label(header, text="Mission Console", bg="#071018", fg="#f6fbff",
                 font=("Segoe UI", 24, "bold")).grid(row=0, column=0, sticky="w")
        tk.Label(header, text="STM32F446RE sensor, ToF ve motor test altyapisi",
                 bg="#071018", fg="#6f8aa4", font=("Segoe UI", 10)).grid(row=1, column=0, sticky="w")

        self.status_badge = tk.Label(header, textvariable=self.status_var, bg="#3a1923",
                                     fg="#ffb3c0", padx=14, pady=8, font=("Segoe UI", 10, "bold"))
        self.status_badge.grid(row=0, column=1, rowspan=2, sticky="e")

    def _build_metrics(self, parent):
        metrics = tk.Frame(parent, bg="#071018")
        metrics.grid(row=1, column=0, sticky="ew", padx=18, pady=(0, 8))
        for index in range(9):
            metrics.grid_columnconfigure(index, weight=1)

        self._metric_card(metrics, 0, "I2C Devices", self.last_i2c_var)
        self._metric_card(metrics, 1, "MPU6050", self.mpu_var)
        self._metric_card(metrics, 2, "ToF Left", self.tof_left_var)
        self._metric_card(metrics, 3, "ToF Right", self.tof_right_var)
        self._metric_card(metrics, 4, "Left Motor", self.left_motor_var)
        self._metric_card(metrics, 5, "Right Motor", self.right_motor_var)
        self._metric_card(metrics, 6, "MPU INT", self.int_var)
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
        tk.Label(top, text="Drive System", bg="#0b1620", fg="#e8f1ff",
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

        tk.Label(controls, text="MANUAL DRIVE", bg="#071018", fg="#8da4bc",
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
        tk.Label(speed_wrap, text="SPEED", bg="#071018", fg="#8da4bc",
                 font=("Segoe UI", 8, "bold")).grid(row=0, column=0, sticky="w")
        tk.Scale(speed_wrap, from_=50, to=100, orient="horizontal", variable=self.drive_speed_var,
                 bg="#071018", fg="#e8f1ff", troughcolor="#18283a", activebackground="#23c483",
                 highlightthickness=0, relief="flat", showvalue=False).grid(row=0, column=1, sticky="ew", padx=10)
        tk.Label(speed_wrap, textvariable=self.drive_speed_var, bg="#071018", fg="#f6fbff",
                 font=("Consolas", 10, "bold"), width=4).grid(row=0, column=2, sticky="e")

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
        speed = self.drive_speed_var.get()
        self.left_drive_var.set(left_sign * speed)
        self.right_drive_var.set(right_sign * speed)
        if self.send_drive_command():
            self.drive_command_var.set(f"{label}  {speed}%")
            self.drive_state_label.configure(fg="#72f0c0")
        else:
            self.left_drive_var.set(0)
            self.right_drive_var.set(0)
            self.drive_command_var.set("LINK REQUIRED")
            self.drive_state_label.configure(fg="#ff7187")

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

    def _wide_command(self, parent, label, command, color):
        button = tk.Button(parent, text=label, command=lambda: self.send_command(command),
                           bg="#0f1c29", fg="#e8f1ff", activebackground="#17283a",
                           activeforeground="#ffffff", relief="flat", anchor="w",
                           font=("Segoe UI", 9, "bold"), padx=12, pady=7, cursor="hand2")
        button.pack(fill="x", padx=16, pady=2)
        button.configure(highlightthickness=2, highlightbackground=color)

    def _drive_slider(self, parent, label, variable):
        wrap = tk.Frame(parent, bg="#0a141f")
        row = tk.Frame(wrap, bg="#0a141f")
        row.pack(fill="x")
        tk.Label(row, text=label, bg="#0a141f", fg="#8da4bc",
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        tk.Label(row, textvariable=variable, bg="#0a141f", fg="#e8f1ff",
                 font=("Consolas", 9, "bold"), width=4).pack(side="right")
        tk.Scale(wrap, from_=-100, to=100, orient="horizontal", variable=variable,
                 bg="#0a141f", fg="#8da4bc", troughcolor="#18283a",
                 activebackground="#23c483", highlightthickness=0, relief="flat",
                 length=235, showvalue=False).pack(fill="x")
        return wrap

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

        self.reader_alive.set()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.connection_label = link_name
        self.robot_ready = False
        self.auto_stream_started = False
        if link_name.startswith("Wi-Fi"):
            self.status_var.set("WIFI CONNECTED - STM WAITING")
            self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
        else:
            self.status_var.set(f"CONNECTED {link_name} - WAITING")
            self.status_badge.configure(bg="#3a3012", fg="#ffe39a")
        self._log(f"[CONNECTED] {link_name}\n", "ok")

    def _auto_connect_wifi(self):
        if self.link_mode_var.get() == "Wi-Fi" and (self.serial_port is None or not self.serial_port.is_open):
            if not self.connect_serial(quiet=True):
                self.status_var.set("WAITING FOR ARGOS WI-FI")
                self.after(2500, self._auto_connect_wifi)

    def disconnect_serial(self):
        self.reader_alive.clear()
        self.drive_active = False
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
        self._log("[DISCONNECTED]\n", "warn")

    def send_custom_command(self):
        command = self.custom_command_var.get().strip()
        if command:
            self.send_command(command)
            self.custom_command_var.set("")

    def send_drive_command(self):
        if not self.robot_ready:
            messagebox.showwarning("STM32 bekleniyor", "ESP bagli fakat STM32 UART yaniti henuz alinmadi.")
            return False
        sent = self.send_command(f"P {self.left_drive_var.get()} {self.right_drive_var.get()}")
        self.drive_active = bool(sent and (self.left_drive_var.get() != 0 or self.right_drive_var.get() != 0))
        return sent

    def stop_drive(self):
        self.drive_active = False
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.drive_command_var.set("STOPPED")
        if hasattr(self, "drive_state_label"):
            self.drive_state_label.configure(fg="#72f0c0")
        self.send_command("x")

    def _motor_keepalive(self):
        if (self.drive_active and self.serial_port is not None and self.serial_port.is_open and
                (self.left_drive_var.get() != 0 or self.right_drive_var.get() != 0)):
            payload = f"P {self.left_drive_var.get()} {self.right_drive_var.get()}\r\n".encode("ascii")
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

    def start_system_test(self):
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning("Bagli degil", "Once ARGOS Wi-Fi veya COM baglantisini kur.")
            return
        if not self.robot_ready:
            messagebox.showwarning("STM32 bekleniyor", "Sistem testi icin once STM32 UART baglantisi kurulmalidir.")
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
            (6000, "Y"),
        ]
        for delay_ms, command in steps:
            self.after(delay_ms, lambda cmd=command: self._send_system_test_command(cmd))
        self.system_test_timeout_job = self.after(45000, self._finish_system_test)

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
        enc_ok = abs(left_delta) > 5 and abs(right_delta) > 5
        checks.append(("Encoders", "PASS" if enc_ok else "CHECK",
                       f"L delta {left_delta:+d}, R delta {right_delta:+d}"))

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
            "",
            "SERIAL LOG",
        ])
        lines.extend(self.system_test_lines)
        return "\n".join(lines) + "\n"

    def _write_system_report(self, report):
        self.system_test_report_path.write_text(report, encoding="utf-8")
        self._log(f"[SYSTEM TEST] Rapor guncellendi: {self.system_test_report_path}\n", "ok")

    def _show_system_report(self, report):
        popup = tk.Toplevel(self)
        popup.title("ARGOS System Test Report")
        popup.geometry("760x620")
        popup.configure(bg="#071018")
        popup.grid_columnconfigure(0, weight=1)
        popup.grid_rowconfigure(1, weight=1)

        tk.Label(popup, text="System Test Report", bg="#071018", fg="#f6fbff",
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

    def send_command(self, command):
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
                self.rx_queue.put(f"\nRX error: {exc}\n")
                self.rx_queue.put("__ARGOS_LINK_LOST__")
                break
            time.sleep(READ_POLL_SECONDS)

    def _drain_rx_queue(self):
        while True:
            try:
                text = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            if text == "__ARGOS_LINK_LOST__":
                self.reader_alive.clear()
                if self.serial_port is not None:
                    try:
                        self.serial_port.close()
                    except Exception:
                        pass
                self.serial_port = None
                self.drive_active = False
                self.robot_ready = False
                self.auto_stream_started = False
                self.left_drive_var.set(0)
                self.right_drive_var.set(0)
                self.drive_command_var.set("LINK LOST")
                self.status_var.set("LINK LOST - RETRYING")
                self.status_badge.configure(bg="#3a1923", fg="#ffb3c0")
                if self.link_mode_var.get() == "Wi-Fi":
                    self.after(1200, self._auto_connect_wifi)
                continue
            upper = text.upper()
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

        self.after(40, self._drain_rx_queue)

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

    def _update_metrics(self, text):
        lower = text.lower()
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
            "right_ab": int(groups[13]),
            "safe": groups[14] or self.motor_state.get("safe", "STOP"),
        })
        self.left_motor_var.set(f"{int(groups[0])}% / {left_rpm:+.1f} rpm")
        self.right_motor_var.set(f"{int(groups[7])}% / {right_rpm:+.1f} rpm")
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
                             left, top, mid_x - 24, "#72f0c0")
        self._draw_motor_bar(canvas, "RIGHT", self.motor_state["right_pct"],
                             self.motor_state["right_enc"], self.motor_state.get("right_ab", 0),
                             self.motor_state.get("right_delta", 0), self.motor_state.get("right_rpm", 0.0),
                             mid_x + 24, top, right, "#8bc7ff")
        canvas.create_text(width / 2, top + row_gap + 23,
                           text=f"PC6/PC8 left PWM    PA0/PA1 right PWM    SAFE {self.motor_state.get('safe', '-')}",
                           fill="#627891", font=("Segoe UI", 8))

    def _draw_motor_bar(self, canvas, name, percent, encoder, ab_state, delta, rpm, x1, y, x2, color):
        center = (x1 + x2) / 2
        bar_y = y + 22
        half = (x2 - x1) / 2
        fill = color if percent >= 0 else "#ff8a65"
        end = center + (max(min(percent, 100), -100) / 100.0) * half

        canvas.create_text(x1, y, text=name, anchor="w", fill="#e8f1ff",
                           font=("Segoe UI", 9, "bold"))
        canvas.create_text(x2, y, text=f"{percent:+d}%  {rpm:+.1f} rpm  d{delta:+d}",
                           anchor="e", fill="#8da4bc", font=("Consolas", 9))
        canvas.create_text(x2, y + 15, text=f"enc {encoder}  AB {ab_state}",
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
