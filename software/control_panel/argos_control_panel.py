import math
import queue
import re
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None


BAUD_RATE = 115200
READ_POLL_SECONDS = 0.03


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

        self.port_var = tk.StringVar()
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
        self.encoder_test = None
        self.custom_command_var = tk.StringVar()

        self._configure_style()
        self._build_ui()
        self.refresh_ports()
        self.after(40, self._drain_rx_queue)

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

        self._section_label(parent, "Serial Link")
        ports_frame = tk.Frame(parent, bg="#0a141f")
        ports_frame.pack(fill="x", padx=16, pady=(0, 8))
        self.port_combo = ttk.Combobox(ports_frame, textvariable=self.port_var, state="readonly")
        self.port_combo.pack(side="left", fill="x", expand=True)
        self._small_button(ports_frame, "REF", self.refresh_ports).pack(side="left", padx=(8, 0))

        baud_frame = tk.Frame(parent, bg="#0a141f")
        baud_frame.pack(fill="x", padx=16, pady=(0, 8))
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
        self._small_button(drive_buttons, "ENC TEST", self.start_encoder_test).pack(side="left", fill="x", expand=True, padx=(8, 0))

        self._section_label(parent, "Quick Commands")
        commands = [
            ("SCAN I2C", "s", "#43b5ff"),
            ("MPU READ", "m", "#26d18d"),
            ("MPU STREAM", "M", "#f1b84b"),
            ("CALIBRATE GYRO", "c", "#ffd166"),
            ("TOF ASSIGN", "T", "#d0a2ff"),
            ("TOF CHECK", "t", "#a98bff"),
            ("TOF STREAM", "D", "#c7ff68"),
            ("LEFT TOF", "L", "#9cd6ff"),
            ("RIGHT TOF", "R", "#ffb38f"),
            ("FWD 50", "f", "#23c483"),
            ("BACK 50", "b", "#ff8a65"),
            ("TURN LEFT", "<", "#8bc7ff"),
            ("TURN RIGHT", ">", "#8bc7ff"),
            ("MOTOR STATUS", "E", "#a6b4c4"),
            ("ENC RAW", "Q", "#c7ff68"),
            ("ENC PULL", "V", "#d0a2ff"),
            ("RESET ENC", "Z", "#ffd166"),
            ("GPIO TEST", "g", "#54d4d4"),
            ("RUN ALL", "a", "#ff7a4f"),
            ("MENU", "h", "#70859f"),
        ]
        for label, command, color in commands:
            self._wide_command(parent, label, command, color)

        custom = tk.Frame(parent, bg="#0a141f")
        custom.pack(fill="x", padx=16, pady=(10, 0))
        tk.Entry(custom, textvariable=self.custom_command_var, bg="#0b1620", fg="#e8f1ff",
                 insertbackground="#e8f1ff", relief="flat").pack(side="left", fill="x", expand=True, ipady=8)
        self._small_button(custom, "SEND", self.send_custom_command).pack(side="left", padx=(8, 0))

        hint = tk.Label(parent, text="Kart resetlenince menu otomatik gelmeli.\nKomutlar tek harf olarak gider.",
                        bg="#0a141f", fg="#627891", justify="left", font=("Segoe UI", 9))
        hint.pack(anchor="w", padx=18, pady=(10, 0))

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
        motor.grid_columnconfigure(0, weight=1)

        top = tk.Frame(motor, bg="#0b1620")
        top.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 4))
        top.grid_columnconfigure(0, weight=1)
        tk.Label(top, text="Drive System", bg="#0b1620", fg="#e8f1ff",
                 font=("Segoe UI", 11, "bold")).grid(row=0, column=0, sticky="w")
        tk.Label(top, text="BTS7960 PWM + encoder feedback", bg="#0b1620", fg="#8da4bc",
                 font=("Segoe UI", 9)).grid(row=0, column=1, sticky="e")

        self.motor_canvas = tk.Canvas(motor, height=115, bg="#050b12", highlightthickness=0)
        self.motor_canvas.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 12))
        self.motor_canvas.bind("<Configure>", lambda _event: self._draw_motor_field())
        self._draw_motor_field()

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

    def connect_serial(self):
        if serial is None:
            messagebox.showerror("pyserial eksik", "Seri port icin pyserial gerekiyor: pip install pyserial")
            return

        selected = self.port_var.get()
        if not selected:
            messagebox.showwarning("Port yok", "Once bir COM port sec.")
            return

        port_name = selected.split(" - ", 1)[0]
        try:
            baud = int(self.baud_var.get())
            self.serial_port = serial.Serial(port_name, baud, timeout=0.05, write_timeout=0.5)
        except Exception as exc:
            messagebox.showerror("Baglanti hatasi", str(exc))
            return

        self.reader_alive.set()
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()
        self.status_var.set(f"CONNECTED {port_name}")
        self.status_badge.configure(bg="#123525", fg="#98ffc2")
        self._log(f"[CONNECTED] {port_name} @ {baud}\n", "ok")
        self._log("Kart reset tusuna bas; menu geldikten sonra komut gonderebilirsin.\n", "warn")

    def disconnect_serial(self):
        self.reader_alive.clear()
        if self.serial_port is not None:
            try:
                self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None
        self.status_var.set("DISCONNECTED")
        self.status_badge.configure(bg="#3a1923", fg="#ffb3c0")
        self._log("[DISCONNECTED]\n", "warn")

    def send_custom_command(self):
        command = self.custom_command_var.get().strip()
        if command:
            self.send_command(command)
            self.custom_command_var.set("")

    def send_drive_command(self):
        self.send_command(f"P {self.left_drive_var.get()} {self.right_drive_var.get()}")

    def stop_drive(self):
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.send_command("x")

    def start_encoder_test(self):
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning("Bagli degil", "Once COM porta baglan.")
            return

        self.encoder_test = {
            "start_left": self.motor_state.get("left_enc", 0),
            "start_right": self.motor_state.get("right_enc", 0),
            "forward_left": 0,
            "forward_right": 0,
            "reverse_left": 0,
            "reverse_right": 0,
        }
        self._log("\n[ENC TEST] Basladi: stop, reset, ileri, geri, stop.\n", "warn")
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self.send_command("x")
        self.send_command("Z")
        self.after(450, self._encoder_test_forward)

    def _encoder_test_forward(self):
        if self.encoder_test is None:
            return
        self.encoder_test["start_left"] = self.motor_state.get("left_enc", 0)
        self.encoder_test["start_right"] = self.motor_state.get("right_enc", 0)
        self.left_drive_var.set(50)
        self.right_drive_var.set(50)
        self.send_command("P 50 50")
        self.after(850, self._encoder_test_capture_forward)

    def _encoder_test_capture_forward(self):
        if self.encoder_test is None:
            return
        self.send_command("x")
        self.send_command("E")
        self.after(250, self._encoder_test_reverse)

    def _encoder_test_reverse(self):
        if self.encoder_test is None:
            return
        self.encoder_test["forward_left"] = self.motor_state.get("left_enc", 0)
        self.encoder_test["forward_right"] = self.motor_state.get("right_enc", 0)
        self.left_drive_var.set(-50)
        self.right_drive_var.set(-50)
        self.send_command("P -50 -50")
        self.after(850, self._encoder_test_capture_reverse)

    def _encoder_test_capture_reverse(self):
        if self.encoder_test is None:
            return
        self.send_command("x")
        self.send_command("E")
        self.after(300, self._finish_encoder_test)

    def _finish_encoder_test(self):
        if self.encoder_test is None:
            return

        self.encoder_test["reverse_left"] = self.motor_state.get("left_enc", 0)
        self.encoder_test["reverse_right"] = self.motor_state.get("right_enc", 0)
        result = self._evaluate_encoder_test(self.encoder_test)
        self.left_drive_var.set(0)
        self.right_drive_var.set(0)
        self._log(result + "\n", "ok" if "PASS" in result else "warn")
        self.encoder_test = None

    def _evaluate_encoder_test(self, data):
        left_forward = data["forward_left"] - data["start_left"]
        right_forward = data["forward_right"] - data["start_right"]
        left_reverse = data["reverse_left"] - data["forward_left"]
        right_reverse = data["reverse_right"] - data["forward_right"]

        left_ok = abs(left_forward) > 5 and abs(left_reverse) > 5 and (left_forward * left_reverse) < 0
        right_ok = abs(right_forward) > 5 and abs(right_reverse) > 5 and (right_forward * right_reverse) < 0
        left_dir_ok = left_forward > 0 and left_reverse < 0
        right_dir_ok = right_forward > 0 and right_reverse < 0

        if left_ok and right_ok and left_dir_ok and right_dir_ok:
            verdict = "PASS"
        elif left_ok and right_ok:
            verdict = "DIR WARN"
        else:
            verdict = "FAIL"

        return (
            f"[ENC TEST] {verdict} | "
            f"L ileri {left_forward:+d}, geri {left_reverse:+d} | "
            f"R ileri {right_forward:+d}, geri {right_reverse:+d}"
        )

    def send_command(self, command):
        if self.serial_port is None or not self.serial_port.is_open:
            messagebox.showwarning("Bagli degil", "Once COM porta baglan.")
            return

        payload = (command + "\r\n").encode("ascii", errors="ignore")
        try:
            self.serial_port.write(payload)
            self.tx_count += len(payload)
            if command == "M":
                self.stream_var.set("TOGGLED")
            elif command == "D":
                self.tof_stream_var.set("TOGGLED")
            self._log(f">> {command}\n", "tx")
        except Exception as exc:
            self._log(f"TX error: {exc}\n", "error")
            self.disconnect_serial()

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
                break
            time.sleep(READ_POLL_SECONDS)

    def _drain_rx_queue(self):
        while True:
            try:
                text = self.rx_queue.get_nowait()
            except queue.Empty:
                break
            self.rx_count += len(text)
            self._update_metrics(text)
            self._log(text, self._tag_for_line(text))

        self.after(40, self._drain_rx_queue)

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
                           text=f"PC6/PC8 left PWM    PB14/PB15 right PWM    SAFE {self.motor_state.get('safe', '-')}",
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
