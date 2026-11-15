"""
========================================================
  PATIENT ECG MONITOR  v2.1  —  ESP32 + AD8232
  Dual connection: WiFi WebSocket  OR  Serial COM Port

  v2.1 FIXES / ADDITIONS:
    • SMS fixed: only sends ?phone= (ESP32 composes vitals)
    • Start BP Measurement button added (after Send SMS)
    • Room Temperature (DHT11/22) added to Vitals + Analyse
    • Analyse box expanded to 6 rows (HR/SpO2/BodyT/RoomT/BP/Hum)

  Serial JSON format from ESP32:
    {"v":2048,"lo":0,"bpm":72}   (legacy serial mode)
  WebSocket JSON from ESP32:
    {"ecg":[v1,v2,...]}   /   vitals JSON

  Install (auto-attempted on first run):
    pip install pyserial websocket-client numpy matplotlib requests pillow
========================================================
"""

# ── Auto-install missing packages ─────────────────────
import subprocess, sys

_REQUIRED = {
    'serial'    : 'pyserial',
    'websocket' : 'websocket-client',
    'numpy'     : 'numpy',
    'matplotlib': 'matplotlib',
    'requests'  : 'requests',
    'PIL'       : 'Pillow',
}

def _ensure(mod, pkg):
    try:
        __import__(mod)
        return True
    except ImportError:
        print(f"[Auto-install] Installing {pkg}…")
        r = subprocess.run(
            [sys.executable, '-m', 'pip', 'install', pkg, '--quiet'],
            capture_output=True)
        if r.returncode == 0:
            print(f"[Auto-install] {pkg} installed ✅")
            return True
        else:
            print(f"[Auto-install] {pkg} FAILED ❌\n{r.stderr.decode()}")
            return False

_AVAIL = {mod: _ensure(mod, pkg) for mod, pkg in _REQUIRED.items()}

# ── Standard imports ──────────────────────────────────
import threading, time, json, csv, math, os
from datetime import datetime
from collections import deque

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk, messagebox, filedialog

if _AVAIL.get('serial'):
    import serial
    import serial.tools.list_ports

if _AVAIL.get('websocket'):
    import websocket as _websocket

if _AVAIL.get('requests'):
    import requests

if _AVAIL.get('PIL'):
    from PIL import ImageGrab

# ═══════════════════ CONFIGURATION ═══════════════════
ECG_WINDOW      = 1000      # visible ECG samples
BPM_HISTORY     = 60
ADC_MAX         = 4095
UPDATE_MS       = 30        # ~33 fps
BAUD_RATE       = 115200
SERIAL_TIMEOUT  = 0.05
CSV_DURATION_S  = 60        # 1-minute CSV
ECG_SAMPLE_HZ   = 200

# ═══════════════════ COLORS ══════════════════════════
C = {
    'bg'        : '#07102a',
    'panel'     : '#061020',
    'border'    : '#0a2a4a',
    'grid_maj'  : '#003c1e',
    'grid_min'  : '#001a0d',
    'trace'     : '#ff3366',
    'trace2'    : '#ff6688',
    'bpm_line'  : '#e74c3c',
    'bpm_fill'  : '#3a0e0e',
    'text_main' : '#c8e6ff',
    'text_dim'  : '#3a6080',
    'ok'        : '#2ecc71',
    'warn'      : '#e74c3c',
    'amber'     : '#f39c12',
    'accent'    : '#00ff88',
    'risk_high' : '#ff2244',
    'risk_mod'  : '#f39c12',
    'risk_low'  : '#2ecc71',
    'analyse_bg': '#060f1e',
}

# ═══════════════════ AGE RANGES ══════════════════════
# Returns (label, normal_hr_lo, normal_hr_hi, normal_spo2_lo,
#          normal_sys_hi, normal_dia_hi, normal_temp_hi)
def _age_profile(age_str):
    try:
        age = int(str(age_str).strip())
    except (ValueError, TypeError):
        age = -1
    if 0 <= age <= 1:
        return ("Infant",       100, 160, 95, 100, 70, 38.0)
    elif 2 <= age <= 11:
        return ("Child",        70,  120, 95, 110, 70, 37.5)
    elif 12 <= age <= 17:
        return ("Adolescent",   60,  100, 96, 120, 80, 37.5)
    elif 18 <= age <= 59:
        return ("Adult",        60,  100, 96, 120, 80, 37.2)
    elif age >= 60:
        return ("Elderly",      55,  100, 95, 130, 85, 37.2)
    else:
        return ("Adult",        60,  100, 96, 120, 80, 37.2)

# ═══════════════════ ANALYSE ═════════════════════════
def analyse_vitals(vitals, age_str):
    """
    Returns list of (param, value_str, status, note)
    status: 'ok' | 'warn' | 'high' | 'low'
    Also returns overall risk: 'RISK' | 'MODERATE' | 'LOW'
    """
    label, hr_lo, hr_hi, spo2_lo, sys_hi, dia_hi, temp_hi = _age_profile(age_str)

    ecg_bpm = vitals.get('ecg_bpm', 0) or vitals.get('bpm', 0)
    spo2    = vitals.get('spo2',  0)
    bodyT   = vitals.get('bodyT', 0.0)
    roomT   = vitals.get('roomT', 0.0)
    sys_bp  = vitals.get('sys',   0)
    dia_bp  = vitals.get('dia',   0)
    roomH   = vitals.get('roomH', 0.0)

    rows  = []
    flags = []   # 'high'/'low' count drives risk

    # Heart Rate
    if ecg_bpm > 0:
        if ecg_bpm < hr_lo:
            rows.append(("HR",    f"{ecg_bpm} BPM", "low",  f"< {hr_lo} (low)"))
            flags.append("low")
        elif ecg_bpm > hr_hi:
            rows.append(("HR",    f"{ecg_bpm} BPM", "high", f"> {hr_hi} (high)"))
            flags.append("high")
        else:
            rows.append(("HR",    f"{ecg_bpm} BPM", "ok",   "Normal"))
    else:
        rows.append(("HR", "--", "dim", "No data"))

    # SpO2
    if spo2 > 0:
        if spo2 < spo2_lo:
            rows.append(("SpO2",  f"{spo2}%",        "low",  f"< {spo2_lo}% (low)"))
            flags.append("low")
        elif spo2 > 100:
            rows.append(("SpO2",  f"{spo2}%",        "high", "Reading error"))
        else:
            rows.append(("SpO2",  f"{spo2}%",        "ok",   "Normal"))
    else:
        rows.append(("SpO2", "--", "dim", "No data"))

    # Temperature
    if bodyT > 1.0:
        if bodyT > temp_hi + 0.8:
            rows.append(("Temp",  f"{bodyT:.1f}°C",  "high", "Fever"))
            flags.append("high")
        elif bodyT > temp_hi:
            rows.append(("Temp",  f"{bodyT:.1f}°C",  "warn", "Slightly high"))
            flags.append("warn")
        elif bodyT < 35.5:
            rows.append(("Temp",  f"{bodyT:.1f}°C",  "low",  "Hypothermia"))
            flags.append("low")
        else:
            rows.append(("Temp",  f"{bodyT:.1f}°C",  "ok",   "Normal"))
    else:
        rows.append(("Temp", "--", "dim", "No data"))

    # Room Temperature
    if roomT > 1.0:
        if roomT > 32.0:
            rows.append(("RoomT", f"{roomT:.1f}°C", "high", "Too hot"))
        elif roomT < 18.0:
            rows.append(("RoomT", f"{roomT:.1f}°C", "low",  "Too cold"))
        else:
            rows.append(("RoomT", f"{roomT:.1f}°C", "ok",   "Normal"))
    else:
        rows.append(("RoomT", "--", "dim", "No data"))

    # Blood Pressure
    if sys_bp > 0 and dia_bp > 0:
        bp_str = f"{sys_bp}/{dia_bp}"
        if sys_bp > sys_hi or dia_bp > dia_hi:
            rows.append(("BP",    bp_str,             "high", "Hypertension"))
            flags.append("high")
        elif sys_bp < 90 or dia_bp < 60:
            rows.append(("BP",    bp_str,             "low",  "Hypotension"))
            flags.append("low")
        else:
            rows.append(("BP",    bp_str,             "ok",   "Normal"))
    else:
        rows.append(("BP", "--/--", "dim", "No data"))

    # Humidity
    if roomH > 0:
        if roomH > 80:
            rows.append(("Hum",   f"{roomH:.0f}%",   "high", "Very humid"))
        elif roomH < 30:
            rows.append(("Hum",   f"{roomH:.0f}%",   "low",  "Very dry"))
        else:
            rows.append(("Hum",   f"{roomH:.0f}%",   "ok",   "Normal"))
    else:
        rows.append(("Hum", "--", "dim", "No data"))

    # Risk level
    high_cnt = sum(1 for f in flags if f in ('high', 'low'))
    warn_cnt = sum(1 for f in flags if f == 'warn')
    if high_cnt >= 2:
        risk = "RISK"
    elif high_cnt == 1 or warn_cnt >= 2:
        risk = "MODERATE"
    else:
        risk = "LOW"

    return label, rows, risk

def build_sms_text(patient_name, patient_id, age_str, vitals):
    age_label, rows, risk = analyse_vitals(vitals, age_str)
    lines = [
        f"ECG Patient Report",
        f"Name: {patient_name}  ID: {patient_id}",
        f"Age: {age_str} ({age_label})",
        f"Date: {datetime.now().strftime('%d/%m/%Y %H:%M')}",
        f"Risk Level: {risk}",
        "---",
    ]
    for param, val, status, note in rows:
        flag = ""
        if status == "high":  flag = " [HIGH]"
        elif status == "low": flag = " [LOW]"
        elif status == "warn":flag = " [WARN]"
        lines.append(f"{param}: {val}{flag}  {note}")
    return "\n".join(lines)


# ══════════════════════════════════════════════════════
class PatientECGMonitor:
    def __init__(self, root):
        self.root = root
        self.root.title("Patient ECG Monitor v2.1 — AD8232 / ESP32")
        self.root.geometry("1400x860")
        self.root.configure(bg=C['bg'])
        self.root.resizable(True, True)

        # ── State ─────────────────────────────────────
        self.lock         = threading.Lock()
        self.ecg_buf      = deque(maxlen=ECG_WINDOW)
        self.all_ecg_ts   = []   # list of (timestamp_float, adc_value)
        self.bpm_buf      = deque(maxlen=BPM_HISTORY)
        self.bpm_time     = deque(maxlen=BPM_HISTORY)
        self.lead_off     = False
        self.connected    = False
        self.running      = False
        self.sample_total = 0
        self.record_start = None

        # Serial
        self.serial_conn  = None
        self.read_thread  = None

        # WebSocket
        self.ws_conn      = None
        self.ws_thread    = None

        self.vitals = {
            'bpm': 0, 'spo2': 0, 'bodyT': 0.0,
            'roomT': 0.0, 'roomH': 0.0,
            'sys': 0, 'dia': 0, 'bp_pulse': 0,
            'ecg_bpm': 0, 'ecg_leads': True,
            'rssi': 0, 'ip': '', 'uptime': 0,
        }

        # Patient
        self.patient_name = tk.StringVar(value="Patient Name")
        self.patient_id   = tk.StringVar(value="ID-0001")
        self.patient_age  = tk.StringVar(value="25")

        # Connection mode
        self.conn_mode = tk.StringVar(value="wifi")   # 'wifi' or 'serial'
        self.esp_ip_var = tk.StringVar(value="192.168.1.100")
        self.port_var   = tk.StringVar()
        self.phone_var  = tk.StringVar(value="+8801XXXXXXXXX")

        self._build_ui()
        self._build_plots()
        self._refresh_ports()

    # ══════════════════════════════════════════════════
    #  UI
    # ══════════════════════════════════════════════════
    def _build_ui(self):
        hdr = tk.Frame(self.root, bg=C['panel'], height=56)
        hdr.pack(fill=tk.X)
        hdr.pack_propagate(False)

        tk.Label(hdr, text="❤  ECG PATIENT MONITOR  v2.1",
                 fg=C['accent'], bg=C['panel'],
                 font=('Courier New', 14, 'bold')).pack(side=tk.LEFT, padx=18)

        self.clock_lbl = tk.Label(hdr, text="", fg=C['text_dim'],
                                  bg=C['panel'], font=('Courier New', 11))
        self.clock_lbl.pack(side=tk.RIGHT, padx=18)
        self._tick_clock()

        self.rec_lbl = tk.Label(hdr, text="● REC  00:00", fg=C['warn'],
                                bg=C['panel'], font=('Courier New', 11, 'bold'))
        self.rec_lbl.pack(side=tk.RIGHT, padx=12)
        self.rec_lbl.pack_forget()

        body = tk.Frame(self.root, bg=C['bg'])
        body.pack(fill=tk.BOTH, expand=True)

        sidebar = tk.Frame(body, bg=C['panel'], width=252)
        sidebar.pack(side=tk.LEFT, fill=tk.Y, padx=(6, 4), pady=6)
        sidebar.pack_propagate(False)
        self._build_sidebar(sidebar)

        right = tk.Frame(body, bg=C['bg'])
        right.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 6), pady=6)

        self.ecg_frame = tk.Frame(right, bg=C['bg'])
        self.ecg_frame.pack(fill=tk.BOTH, expand=True)

        self._build_statusbar(right)

    # ─── SIDEBAR ──────────────────────────────────────
    def _build_sidebar(self, parent):
        # Scrollable canvas inside sidebar
        canvas = tk.Canvas(parent, bg=C['panel'], highlightthickness=0)
        sb = ttk.Scrollbar(parent, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=sb.set)
        sb.pack(side=tk.RIGHT, fill=tk.Y)
        canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        frame = tk.Frame(canvas, bg=C['panel'])
        win_id = canvas.create_window((0, 0), window=frame, anchor='nw')

        def _on_cfg(e):
            canvas.configure(scrollregion=canvas.bbox("all"))
            canvas.itemconfig(win_id, width=canvas.winfo_width())
        frame.bind("<Configure>", _on_cfg)
        canvas.bind("<Configure>", lambda e: canvas.itemconfig(win_id, width=e.width))

        def _on_scroll(e):
            canvas.yview_scroll(int(-1 * (e.delta / 120)), "units")
        canvas.bind_all("<MouseWheel>", _on_scroll)

        p = frame   # shorthand

        def section(text):
            tk.Label(p, text=text, fg=C['text_dim'], bg=C['panel'],
                     font=('Courier New', 9)).pack(anchor='w', padx=10, pady=(12, 2))

        def lentry(label, var):
            tk.Label(p, text=label, fg=C['text_dim'], bg=C['panel'],
                     font=('Courier New', 9)).pack(anchor='w', padx=10)
            tk.Entry(p, textvariable=var, bg='#0d1e3a',
                     fg=C['text_main'], font=('Courier New', 10),
                     insertbackground='white', relief='flat', bd=4
                     ).pack(fill=tk.X, padx=10, pady=2)

        # ── Patient ───────────────────────────────────
        section("── PATIENT ──")
        lentry("Name", self.patient_name)
        lentry("ID",   self.patient_id)
        lentry("Age",  self.patient_age)

        self.age_status_lbl = tk.Label(p, text="Adult  |  Age: 25",
                                       fg=C['accent'], bg=C['panel'],
                                       font=('Courier New', 9, 'bold'))
        self.age_status_lbl.pack(anchor='w', padx=14, pady=(2, 0))
        self.patient_age.trace_add('write', self._update_age_label)
        self._update_age_label()

        # ── Vitals ────────────────────────────────────
        section("── VITALS ──")
        self.bpm_big = tk.Label(p, text="--", fg=C['warn'],
                                bg=C['panel'], font=('Courier New', 38, 'bold'))
        self.bpm_big.pack()
        tk.Label(p, text="ECG BPM", fg=C['text_dim'], bg=C['panel'],
                 font=('Courier New', 10)).pack()

        self.vitals_lbl = tk.Label(
            p,
            text="SpO2: --\nBodyT:--°C\nRoomT:--°C\nHum:  --\nBP:   --/--",
            fg=C['text_main'], bg=C['panel'],
            font=('Courier New', 9), justify=tk.LEFT)
        self.vitals_lbl.pack(anchor='w', padx=14, pady=(6, 2))

        self.lead_lbl = tk.Label(p, text="⬤  LEAD OK",
                                 fg=C['ok'], bg=C['panel'],
                                 font=('Courier New', 11, 'bold'))
        self.lead_lbl.pack(pady=2)

        # ── ANALYSE BOX ───────────────────────────────
        section("── ANALYSE ──")

        self.risk_lbl = tk.Label(p, text="RISK LEVEL: --",
                                 fg=C['amber'], bg=C['panel'],
                                 font=('Courier New', 11, 'bold'))
        self.risk_lbl.pack(anchor='w', padx=10, pady=(0, 4))

        self.analyse_frame = tk.Frame(p, bg=C['analyse_bg'],
                                      relief='flat', bd=0)
        self.analyse_frame.pack(fill=tk.X, padx=8, pady=2)

        # 6 rows: HR, SpO2, BodyTemp, RoomTemp, BP, Hum
        self._analyse_rows = []
        for _ in range(6):
            row = tk.Frame(self.analyse_frame, bg=C['analyse_bg'])
            row.pack(fill=tk.X, padx=4, pady=1)
            param_lbl = tk.Label(row, text="--", width=6,
                                 fg=C['text_dim'], bg=C['analyse_bg'],
                                 font=('Courier New', 8), anchor='w')
            param_lbl.pack(side=tk.LEFT)
            val_lbl = tk.Label(row, text="--", width=9,
                               fg=C['text_main'], bg=C['analyse_bg'],
                               font=('Courier New', 8, 'bold'), anchor='w')
            val_lbl.pack(side=tk.LEFT)
            note_lbl = tk.Label(row, text="--",
                                fg=C['text_dim'], bg=C['analyse_bg'],
                                font=('Courier New', 8), anchor='w')
            note_lbl.pack(side=tk.LEFT, fill=tk.X, expand=True)
            self._analyse_rows.append((param_lbl, val_lbl, note_lbl))

        # ── Connection mode ───────────────────────────
        section("── CONNECTION ──")

        mode_frame = tk.Frame(p, bg=C['panel'])
        mode_frame.pack(fill=tk.X, padx=10, pady=(2, 4))
        for txt, val in (("📡 WiFi", "wifi"), ("🔌 COM Port", "serial")):
            tk.Radiobutton(mode_frame, text=txt, variable=self.conn_mode,
                           value=val, command=self._on_mode_change,
                           bg=C['panel'], fg=C['text_main'], selectcolor='#0d1e3a',
                           activebackground=C['panel'],
                           font=('Courier New', 9)
                           ).pack(side=tk.LEFT, padx=6)

        # WiFi frame
        self.wifi_frame = tk.Frame(p, bg=C['panel'])
        self.wifi_frame.pack(fill=tk.X, padx=10)
        tk.Label(self.wifi_frame, text="ESP32 IP:", fg=C['text_dim'],
                 bg=C['panel'], font=('Courier New', 9)).pack(anchor='w')
        tk.Entry(self.wifi_frame, textvariable=self.esp_ip_var, bg='#0d1e3a',
                 fg=C['accent'], font=('Courier New', 10),
                 insertbackground='white', relief='flat', bd=4
                 ).pack(fill=tk.X, pady=2)

        # Serial frame
        self.serial_frame = tk.Frame(p, bg=C['panel'])
        tk.Label(self.serial_frame, text="COM Port:", fg=C['text_dim'],
                 bg=C['panel'], font=('Courier New', 9)).pack(anchor='w')
        self.port_cb = ttk.Combobox(self.serial_frame, textvariable=self.port_var,
                                    state='readonly', font=('Courier New', 9))
        self.port_cb.pack(fill=tk.X, pady=2)
        tk.Button(self.serial_frame, text="⟳ Refresh",
                  command=self._refresh_ports,
                  bg='#0d1e3a', fg=C['text_main'], relief='flat',
                  font=('Courier New', 8), pady=3, cursor='hand2'
                  ).pack(fill=tk.X)

        self._on_mode_change()   # show correct frame

        self.conn_btn = tk.Button(p, text="▶  Connect",
                                  command=self._toggle_connection,
                                  bg='#0e3a1e', fg=C['ok'], relief='flat',
                                  font=('Courier New', 10, 'bold'), pady=6,
                                  cursor='hand2')
        self.conn_btn.pack(fill=tk.X, padx=10, pady=6)

        # ── SMS ───────────────────────────────────────
        section("── SEND SMS ──")
        tk.Label(p, text="Phone number:", fg=C['text_dim'], bg=C['panel'],
                 font=('Courier New', 9)).pack(anchor='w', padx=10)
        tk.Entry(p, textvariable=self.phone_var, bg='#0d1e3a',
                 fg=C['accent'], font=('Courier New', 10),
                 insertbackground='white', relief='flat', bd=4
                 ).pack(fill=tk.X, padx=10, pady=2)

        self.sms_btn = tk.Button(
            p, text="📨  Send SMS",
            command=self._send_sms,
            bg='#0e3a1e', fg=C['accent'], relief='flat',
            font=('Courier New', 10, 'bold'), pady=6, cursor='hand2')
        self.sms_btn.pack(fill=tk.X, padx=10, pady=2)

        self.bp_btn = tk.Button(
            p, text="🩸  Start BP Measurement",
            command=self._start_bp,
            bg='#1a0a2e', fg='#c084fc', relief='flat',
            font=('Courier New', 10, 'bold'), pady=6, cursor='hand2')
        self.bp_btn.pack(fill=tk.X, padx=10, pady=2)

        self.sms_status = tk.Label(p, text="", fg=C['amber'],
                                   bg=C['panel'], font=('Courier New', 8),
                                   wraplength=220, justify=tk.LEFT)
        self.sms_status.pack(fill=tk.X, padx=10)

        # ── Export ────────────────────────────────────
        section("── EXPORT ──")
        btn_s = dict(bg='#0d1e3a', fg=C['text_main'], relief='flat',
                     font=('Courier New', 9), pady=5, cursor='hand2')
        tk.Button(p, text="💾  Save CSV (last 1 min)",
                  command=self._save_csv, **btn_s
                  ).pack(fill=tk.X, padx=10, pady=2)
        tk.Button(p, text="🖼  Save Screenshot JPG",
                  command=lambda: self._save_img('jpg'), **btn_s
                  ).pack(fill=tk.X, padx=10, pady=2)
        tk.Button(p, text="📷  Save Screenshot PNG",
                  command=lambda: self._save_img('png'), **btn_s
                  ).pack(fill=tk.X, padx=10, pady=2)

        tk.Label(p, text="", bg=C['panel']).pack(pady=6)  # bottom spacer

    def _on_mode_change(self):
        mode = self.conn_mode.get()
        if mode == 'wifi':
            self.serial_frame.pack_forget()
            self.wifi_frame.pack(fill=tk.X, padx=10)
        else:
            self.wifi_frame.pack_forget()
            self.serial_frame.pack(fill=tk.X, padx=10)

    def _update_age_label(self, *_):
        label, *_ = _age_profile(self.patient_age.get())
        self.age_status_lbl.config(
            text=f"{label}  |  Age: {self.patient_age.get()}")

    def _build_statusbar(self, parent):
        bar = tk.Frame(parent, bg=C['panel'], height=28)
        bar.pack(fill=tk.X, pady=(4, 0))
        bar.pack_propagate(False)

        self.status_lbl = tk.Label(bar, text="Disconnected",
                                   fg=C['warn'], bg=C['panel'],
                                   font=('Courier New', 9))
        self.status_lbl.pack(side=tk.LEFT, padx=12)

        self.sample_lbl = tk.Label(bar, text="Samples: 0",
                                   fg=C['text_dim'], bg=C['panel'],
                                   font=('Courier New', 9))
        self.sample_lbl.pack(side=tk.LEFT, padx=20)

        self.last_val_lbl = tk.Label(bar, text="ADC: ---",
                                     fg=C['text_dim'], bg=C['panel'],
                                     font=('Courier New', 9))
        self.last_val_lbl.pack(side=tk.LEFT, padx=20)

        tk.Label(bar, text="200 Hz | 12-bit ADC | AD8232",
                 fg=C['text_dim'], bg=C['panel'],
                 font=('Courier New', 9)).pack(side=tk.RIGHT, padx=12)

    # ══════════════════════════════════════════════════
    #  MATPLOTLIB PLOTS
    # ══════════════════════════════════════════════════
    def _build_plots(self):
        self.fig = plt.Figure(figsize=(10, 6), dpi=96, facecolor=C['bg'])
        gs = gridspec.GridSpec(2, 1, figure=self.fig,
                               height_ratios=[3.2, 1], hspace=0.35,
                               left=0.06, right=0.98, top=0.93, bottom=0.08)

        self.ax_ecg = self.fig.add_subplot(gs[0])
        self._style_ecg_axes(self.ax_ecg)
        self.ecg_line, = self.ax_ecg.plot(
            [], [], color=C['trace'], linewidth=1.6,
            solid_capstyle='round', solid_joinstyle='round')
        self.sweep_line = self.ax_ecg.axvline(
            x=0, color=C['trace2'], linewidth=0.8, alpha=0.4)

        self.ax_bpm = self.fig.add_subplot(gs[1])
        self._style_bpm_axes(self.ax_bpm)
        self.bpm_fill = self.ax_bpm.fill_between(
            [], [], color=C['bpm_fill'], alpha=0.5)
        self.bpm_line_plot, = self.ax_bpm.plot(
            [], [], color=C['bpm_line'], linewidth=1.5)
        self.ax_bpm.axhspan(60, 100, color='#0e3a1e', alpha=0.25)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.ecg_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        self.anim = FuncAnimation(self.fig, self._animate,
                                  interval=UPDATE_MS, blit=False,
                                  cache_frame_data=False)

    def _style_ecg_axes(self, ax):
        ax.set_facecolor(C['bg'])
        ax.set_xlim(0, ECG_WINDOW)
        ax.set_ylim(-100, ADC_MAX + 100)
        ax.set_title("Real-time ECG  —  AD8232 / ESP32",
                     color=C['text_main'], fontsize=11,
                     fontfamily='Courier New', loc='left', pad=6)
        ax.set_ylabel("ADC Value", color=C['text_dim'],
                      fontsize=8, fontfamily='Courier New')
        ax.tick_params(colors=C['text_dim'], labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor(C['border'])
        ax.grid(True, which='major', color=C['grid_maj'], linewidth=0.6)
        ax.grid(True, which='minor', color=C['grid_min'], linewidth=0.3)
        ax.set_xticks(np.arange(0, ECG_WINDOW + 1, 100))
        ax.set_yticks(np.arange(0, ADC_MAX + 1, 500))
        ax.minorticks_on()
        ax.set_xticks(np.arange(0, ECG_WINDOW + 1, 20), minor=True)
        ax.set_yticks(np.arange(0, ADC_MAX + 1, 100), minor=True)
        ax.axhline(y=2048, color=C['text_dim'], linewidth=0.4,
                   linestyle='--', alpha=0.4)

    def _style_bpm_axes(self, ax):
        ax.set_facecolor(C['bg'])
        ax.set_title("Heart Rate (ECG BPM)", color=C['text_main'],
                     fontsize=9, fontfamily='Courier New', loc='left', pad=4)
        ax.set_ylim(20, 200)
        ax.set_ylabel("BPM", color=C['text_dim'],
                      fontsize=8, fontfamily='Courier New')
        ax.tick_params(colors=C['text_dim'], labelsize=7)
        for sp in ax.spines.values():
            sp.set_edgecolor(C['border'])
        ax.grid(True, color=C['grid_maj'], linewidth=0.4)
        ax.axhline(y=40,  color=C['warn'], linewidth=0.7, linestyle=':', alpha=0.6)
        ax.axhline(y=150, color=C['warn'], linewidth=0.7, linestyle=':', alpha=0.6)

    # ══════════════════════════════════════════════════
    #  ANIMATION
    # ══════════════════════════════════════════════════
    def _animate(self, frame):
        with self.lock:
            ecg    = list(self.ecg_buf)
            bpm_y  = list(self.bpm_buf)
            bpm_t  = list(self.bpm_time)
            lead   = self.lead_off
            vitals = dict(self.vitals)

        # ECG trace
        if len(ecg) > 1:
            x = np.arange(len(ecg))
            self.ecg_line.set_data(x, ecg)
            self.ax_ecg.set_xlim(0, ECG_WINDOW)
            self.sweep_line.set_xdata([len(ecg), len(ecg)])
            self.ecg_line.set_alpha(0.2 if lead else 1.0)

        for txt in self.ax_ecg.texts:
            txt.remove()
        if lead:
            self.ax_ecg.text(
                ECG_WINDOW / 2, 2048,
                "⚠  LEAD OFF — CHECK ELECTRODES",
                color=C['warn'], fontsize=13, fontfamily='Courier New',
                fontweight='bold', ha='center', va='center',
                bbox=dict(boxstyle='round,pad=0.4', facecolor='#3a0e0e',
                          alpha=0.85, edgecolor=C['warn']))

        if len(bpm_y) > 1:
            self.bpm_line_plot.set_data(bpm_t, bpm_y)
            self.ax_bpm.set_xlim(min(bpm_t), max(bpm_t) + 1)
            self.bpm_fill.remove()
            self.bpm_fill = self.ax_bpm.fill_between(
                bpm_t, bpm_y, color=C['bpm_fill'], alpha=0.5)

        # Sidebar: BPM
        ecg_bpm = vitals.get('ecg_bpm', 0) or vitals.get('bpm', 0)
        if ecg_bpm and ecg_bpm > 0:
            self.bpm_big.config(text=str(ecg_bpm))
            color = C['ok'] if 40 <= ecg_bpm <= 150 else C['warn']
            self.bpm_big.config(fg=color)

        self.lead_lbl.config(
            text="⚠  LEAD OFF" if lead else "⬤  LEAD OK",
            fg=C['warn'] if lead else C['ok'])

        # Vitals text
        spo2   = vitals.get('spo2', 0)
        bodyT  = vitals.get('bodyT', 0.0)
        roomT  = vitals.get('roomT', 0.0)
        roomH  = vitals.get('roomH', 0.0)
        sys_bp = vitals.get('sys', 0)
        dia_bp = vitals.get('dia', 0)
        bp_str = f"{sys_bp}/{dia_bp}" if sys_bp > 0 else "--/--"
        self.vitals_lbl.config(
            text=(f"SpO2: {spo2 if spo2 else '--'}%\n"
                  f"BodyT:{bodyT:.1f}°C\n"
                  f"RoomT:{roomT:.1f}°C\n"
                  f"Hum:  {roomH:.0f}%\n"
                  f"BP:   {bp_str} mmHg"))

        # Age label
        self._update_age_label()

        # Analyse box update
        self._update_analyse(vitals)

        # Status bar
        with self.lock:
            n    = self.sample_total
            last = ecg[-1] if ecg else 0
        self.sample_lbl.config(text=f"Samples: {n:,}")
        self.last_val_lbl.config(text=f"ADC: {last}")

        if self.record_start:
            elapsed = int(time.time() - self.record_start)
            m, s = divmod(elapsed, 60)
            self.rec_lbl.config(text=f"● REC  {m:02d}:{s:02d}")

        return self.ecg_line, self.bpm_line_plot

    def _update_analyse(self, vitals):
        age_str = self.patient_age.get()
        label, rows, risk = analyse_vitals(vitals, age_str)

        # Risk label
        risk_colors = {'RISK': C['risk_high'], 'MODERATE': C['risk_mod'],
                       'LOW': C['risk_low']}
        self.risk_lbl.config(
            text=f"RISK LEVEL: {risk}",
            fg=risk_colors.get(risk, C['amber']))

        # Status → color
        status_col = {
            'ok':   C['ok'],
            'high': C['risk_high'],
            'low':  '#38bdf8',
            'warn': C['amber'],
            'dim':  C['text_dim'],
        }
        for i, (param, val, status, note) in enumerate(rows):
            if i >= len(self._analyse_rows):
                break
            p_lbl, v_lbl, n_lbl = self._analyse_rows[i]
            p_lbl.config(text=param, fg=C['text_dim'])
            v_lbl.config(text=val,   fg=status_col.get(status, C['text_main']))
            flag = ""
            if status == 'high': flag = " ▲HIGH"
            elif status == 'low': flag = " ▼LOW"
            elif status == 'warn': flag = " ⚠WARN"
            n_lbl.config(text=flag if flag else note,
                         fg=status_col.get(status, C['text_dim']))

    # ══════════════════════════════════════════════════
    #  CONNECTION
    # ══════════════════════════════════════════════════
    def _refresh_ports(self):
        if not _AVAIL.get('serial'):
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_cb['values'] = ports
        if ports:
            self.port_cb.current(0)

    def _toggle_connection(self):
        if self.connected:
            self._disconnect()
        else:
            self._connect()

    def _connect(self):
        mode = self.conn_mode.get()
        if mode == 'wifi':
            self._connect_wifi()
        else:
            self._connect_serial()

    def _disconnect(self):
        self.running   = False
        self.connected = False
        # Serial
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        self.serial_conn = None
        # WebSocket
        if self.ws_conn:
            try:
                self.ws_conn.close()
            except Exception:
                pass
            self.ws_conn = None

        self.conn_btn.config(text="▶  Connect", bg='#0e3a1e', fg=C['ok'])
        self.status_lbl.config(text="Disconnected", fg=C['warn'])
        self.lead_lbl.config(text="⬤  LEAD OK", fg=C['ok'])
        self.rec_lbl.pack_forget()
        self.record_start = None
        with self.lock:
            self.ecg_buf.clear()

    # ── WiFi / WebSocket ──────────────────────────────
    def _connect_wifi(self):
        if not _AVAIL.get('websocket'):
            messagebox.showerror("Missing library",
                                 "websocket-client not available.\n"
                                 "Restart the script — it will auto-install.")
            return
        ip  = self.esp_ip_var.get().strip()
        url = f"ws://{ip}:81"
        self.running = True
        self.ws_thread = threading.Thread(
            target=self._ws_loop, args=(url,), daemon=True)
        self.ws_thread.start()
        self.conn_btn.config(text="■  Disconnect", bg='#3a0e0e', fg=C['warn'])
        self.status_lbl.config(text=f"Connecting → {url}…", fg=C['amber'])

    def _ws_loop(self, url):
        def on_open(wsapp):
            self.ws_conn      = wsapp
            self.connected    = True
            self.record_start = time.time()
            self.root.after(0, lambda: (
                self.status_lbl.config(text=f"WiFi  →  {url}", fg=C['ok']),
                self.rec_lbl.pack(side=tk.RIGHT, padx=12),
            ))

        def on_message(wsapp, message):
            self._parse_ws(message)

        def on_close(wsapp, code, msg):
            self.connected = False
            if self.running:
                self.root.after(0, lambda: self.status_lbl.config(
                    text="Reconnecting…", fg=C['amber']))
                time.sleep(3)
                if self.running:
                    self._ws_loop(url)

        def on_error(wsapp, err):
            self.root.after(0, lambda: self.status_lbl.config(
                text=f"WS error: {err}", fg=C['warn']))

        app = _websocket.WebSocketApp(url,
                                      on_open=on_open,
                                      on_message=on_message,
                                      on_close=on_close,
                                      on_error=on_error)
        app.run_forever(ping_interval=20, ping_timeout=10)

    def _parse_ws(self, raw):
        try:
            d = json.loads(raw)
        except Exception:
            return

        if 'ecg' in d:
            samples = d['ecg']
            if not isinstance(samples, list):
                return
            now = time.time()
            with self.lock:
                for v in samples:
                    try:
                        iv = int(v)
                        if 0 <= iv <= ADC_MAX:
                            self.ecg_buf.append(iv)
                            self.all_ecg_ts.append((now, iv))
                            self.sample_total += 1
                    except (ValueError, TypeError):
                        pass
            return

        with self.lock:
            for key in ('bpm', 'spo2', 'bodyT', 'roomT', 'roomH',
                        'sys', 'dia', 'bp_pulse', 'ecg_bpm',
                        'ecg_leads', 'rssi', 'uptime', 'ip'):
                if key in d:
                    self.vitals[key] = d[key]
            self.lead_off = bool(d.get('ecg_leads', False))
            ecg_bpm = d.get('ecg_bpm', 0)
            if ecg_bpm and ecg_bpm > 0:
                t = self.sample_total / float(ECG_SAMPLE_HZ)
                self.bpm_buf.append(ecg_bpm)
                self.bpm_time.append(t)

    # ── Serial / COM ──────────────────────────────────
    def _connect_serial(self):
        if not _AVAIL.get('serial'):
            messagebox.showerror("Missing library",
                                 "pyserial not available.\n"
                                 "Restart the script — it will auto-install.")
            return
        port = self.port_var.get()
        if not port:
            messagebox.showerror("Error", "Select a COM port.")
            return
        try:
            self.serial_conn = serial.Serial(
                port, BAUD_RATE, timeout=SERIAL_TIMEOUT)
            time.sleep(0.15)
            self.serial_conn.reset_input_buffer()
            self.connected    = True
            self.running      = True
            self.record_start = time.time()
            self.read_thread  = threading.Thread(
                target=self._serial_read_loop, daemon=True)
            self.read_thread.start()
            self.conn_btn.config(text="■  Disconnect", bg='#3a0e0e', fg=C['warn'])
            self.status_lbl.config(text=f"Serial  →  {port}", fg=C['ok'])
            self.rec_lbl.pack(side=tk.RIGHT, padx=12)
        except Exception as e:
            messagebox.showerror("Connection Error", f"Cannot open {port}:\n{e}")

    def _serial_read_loop(self):
        while self.running:
            try:
                if not (self.serial_conn and self.serial_conn.is_open):
                    break
                if self.serial_conn.in_waiting:
                    raw  = self.serial_conn.readline()
                    line = raw.decode('utf-8', errors='ignore').strip()
                    if line:
                        self._parse_serial(line)
                else:
                    time.sleep(0.002)
            except serial.SerialException:
                break
            except Exception:
                pass

    def _parse_serial(self, line):
        """Handles both {"v":N,"lo":0,"bpm":N} and {"ecg":[...]} formats."""
        try:
            d = json.loads(line)
            if 'status' in d:
                return
            # Array ECG format
            if 'ecg' in d:
                self._parse_ws(line)
                return
            # Single-sample format
            val = int(d.get('v', 0))
            lo  = bool(d.get('lo', 0))
            bpm = int(d.get('bpm', 0))
            if not lo and not (0 <= val <= ADC_MAX):
                return
            now = time.time()
            with self.lock:
                self.ecg_buf.append(val)
                self.all_ecg_ts.append((now, val))
                self.lead_off     = lo
                self.sample_total += 1
                if bpm > 0:
                    self.vitals['ecg_bpm'] = bpm
                    t = self.sample_total / float(ECG_SAMPLE_HZ)
                    self.bpm_buf.append(bpm)
                    self.bpm_time.append(t)
        except (json.JSONDecodeError, KeyError, ValueError):
            pass

    # ══════════════════════════════════════════════════
    #  SMS
    # ══════════════════════════════════════════════════
    def _send_sms(self):
        if not _AVAIL.get('requests'):
            messagebox.showerror("Missing library",
                                 "requests not available.\n"
                                 "Restart the script — it will auto-install.")
            return

        phone = self.phone_var.get().strip()
        if not phone or len(phone) < 7:
            messagebox.showerror("Error", "Enter a valid phone number.")
            return
        ip = self.esp_ip_var.get().strip()
        if not ip:
            messagebox.showerror("Error", "Enter ESP32 IP address.")
            return

        self.sms_btn.config(state=tk.DISABLED, text="⏳ Sending…")
        self.sms_status.config(text="Sending SMS…", fg=C['amber'])

        def do_send():
            try:
                import urllib.parse
                # ESP32 /sendSMS only accepts ?phone= parameter.
                # It composes its own vitals summary via buildVitalsSummary().
                url = f"http://{ip}/sendSMS?phone={urllib.parse.quote(phone)}"
                r   = requests.get(url, timeout=14)
                txt = r.text.strip()
                if r.ok and txt == "OK":
                    msg = f"✅ SMS sent to {phone}"
                    col = C['ok']
                elif r.status_code == 503:
                    msg = "❌ SIM800L not ready on ESP32"
                    col = C['warn']
                elif r.status_code == 400:
                    msg = f"❌ ESP32: {txt}"
                    col = C['warn']
                else:
                    msg = f"❌ ESP32 replied: {txt[:60]}"
                    col = C['warn']
            except requests.exceptions.ConnectionError:
                msg = f"❌ Cannot reach {ip} — check WiFi/IP"
                col = C['warn']
            except requests.exceptions.Timeout:
                msg = "❌ Request timed out"
                col = C['warn']
            except Exception as e:
                msg = f"❌ Error: {str(e)[:50]}"
                col = C['warn']
            self.root.after(0, lambda: (
                self.sms_status.config(text=msg, fg=col),
                self.sms_btn.config(state=tk.NORMAL, text="📨  Send SMS"),
                self.root.after(9000, lambda: self.sms_status.config(text="")),
            ))

        threading.Thread(target=do_send, daemon=True).start()

    # ══════════════════════════════════════════════════
    #  START BP MEASUREMENT
    # ══════════════════════════════════════════════════
    def _start_bp(self):
        """
        Triggers BP measurement on the ESP32.
        WiFi mode: HTTP GET /startBP  +  WebSocket 'startBP'
        Serial mode: sends 'startBP\n' over COM
        """
        mode = self.conn_mode.get()

        if mode == 'wifi':
            ip = self.esp_ip_var.get().strip()
            if not ip:
                messagebox.showerror("Error", "Enter ESP32 IP address.")
                return
            self.bp_btn.config(state=tk.DISABLED, text="⏳ Starting…")
            self.sms_status.config(text="BP measurement started…", fg=C['amber'])

            def do_bp():
                try:
                    # Send via HTTP
                    if _AVAIL.get('requests'):
                        requests.get(f"http://{ip}/startBP", timeout=5)
                    # Also send via WebSocket if connected
                    if self.ws_conn:
                        try:
                            self.ws_conn.send("startBP")
                        except Exception:
                            pass
                    msg = "🩸 BP measurement started"
                    col = C['ok']
                except Exception as e:
                    msg = f"❌ BP start failed: {str(e)[:40]}"
                    col = C['warn']
                self.root.after(0, lambda: (
                    self.bp_btn.config(state=tk.NORMAL,
                                       text="🩸  Start BP Measurement"),
                    self.sms_status.config(text=msg, fg=col),
                    self.root.after(5000, lambda: self.sms_status.config(text="")),
                ))

            threading.Thread(target=do_bp, daemon=True).start()

        elif mode == 'serial':
            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(b'startBP\n')
                    self.sms_status.config(
                        text="🩸 BP command sent via serial", fg=C['ok'])
                    self.root.after(5000, lambda: self.sms_status.config(text=""))
                except Exception as e:
                    self.sms_status.config(
                        text=f"❌ Serial BP error: {e}", fg=C['warn'])
            else:
                messagebox.showwarning("Not Connected",
                                       "Connect via COM port first.")

    # ══════════════════════════════════════════════════
    #  EXPORT
    # ══════════════════════════════════════════════════
    def _save_csv(self):
        """Save only the last 1 minute (CSV_DURATION_S seconds) of ECG."""
        with self.lock:
            data = list(self.all_ecg_ts)

        if not data:
            messagebox.showinfo("No Data", "No ECG data recorded yet.")
            return

        cutoff = time.time() - CSV_DURATION_S
        recent = [(ts, v) for ts, v in data if ts >= cutoff]
        if not recent:
            recent = data[-ECG_SAMPLE_HZ * CSV_DURATION_S:]

        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        name   = self.patient_name.get().replace(" ", "_")
        path   = filedialog.asksaveasfilename(
            defaultextension=".csv",
            initialfile=f"ECG_{name}_{ts_str}_1min.csv",
            filetypes=[("CSV files", "*.csv")])
        if not path:
            return

        with self.lock:
            vitals = dict(self.vitals)
        age_label, rows, risk = analyse_vitals(vitals, self.patient_age.get())

        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(["Patient",      self.patient_name.get()])
            w.writerow(["ID",           self.patient_id.get()])
            w.writerow(["Age",          self.patient_age.get()])
            w.writerow(["Age Group",    age_label])
            w.writerow(["Risk Level",   risk])
            w.writerow(["Date",         datetime.now().isoformat()])
            w.writerow(["CSV Duration", f"Last {CSV_DURATION_S}s"])
            w.writerow([])
            w.writerow(["--- VITALS ---"])
            w.writerow(["ECG BPM", vitals.get('ecg_bpm', '--')])
            w.writerow(["SpO2",    vitals.get('spo2',    '--')])
            w.writerow(["BodyTemp",vitals.get('bodyT',   '--')])
            w.writerow(["RoomTemp",vitals.get('roomT',   '--')])
            w.writerow(["Humidity",vitals.get('roomH',   '--')])
            w.writerow(["BP",      f"{vitals.get('sys','--')}/{vitals.get('dia','--')}"])
            w.writerow([])
            w.writerow(["--- ANALYSIS ---"])
            for param, val, status, note in rows:
                flag = "HIGH" if status=="high" else ("LOW" if status=="low" else
                        "WARN" if status=="warn" else "OK")
                w.writerow([param, val, flag, note])
            w.writerow([])
            w.writerow(["--- ECG SAMPLES (last 1 min) ---"])
            w.writerow(["sample_index", "timestamp", "adc_value"])
            t0 = recent[0][0] if recent else 0
            for i, (ts, v) in enumerate(recent):
                w.writerow([i, f"{ts - t0:.4f}", v])

        messagebox.showinfo("Saved",
                            f"Last {CSV_DURATION_S}s ECG saved ({len(recent):,} samples):\n{path}")

    def _save_img(self, fmt='jpg'):
        """Save screenshot. fmt = 'jpg' or 'png'."""
        ts_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        name   = self.patient_name.get().replace(" ", "_")
        ext    = fmt.lower()
        ftype  = "JPEG image" if ext == 'jpg' else "PNG image"
        path   = filedialog.asksaveasfilename(
            defaultextension=f".{ext}",
            initialfile=f"ECG_{name}_{ts_str}.{ext}",
            filetypes=[(ftype, f"*.{ext}")])
        if not path:
            return

        # Try Pillow full-window screenshot first
        if _AVAIL.get('PIL'):
            try:
                self.root.update_idletasks()
                x  = self.root.winfo_rootx()
                y  = self.root.winfo_rooty()
                x2 = x + self.root.winfo_width()
                y2 = y + self.root.winfo_height()
                img = ImageGrab.grab(bbox=(x, y, x2, y2))
                if ext == 'jpg':
                    img = img.convert('RGB')
                    img.save(path, 'JPEG', quality=95)
                else:
                    img.save(path, 'PNG')
                messagebox.showinfo("Saved", f"Screenshot saved:\n{path}")
                return
            except Exception:
                pass

        # Fallback: matplotlib figure only
        save_fmt = 'jpeg' if ext == 'jpg' else 'png'
        self.fig.savefig(path, dpi=150, facecolor=C['bg'],
                         bbox_inches='tight', format=save_fmt)
        messagebox.showinfo("Saved", f"Screenshot (chart only) saved:\n{path}")

    # ══════════════════════════════════════════════════
    #  CLOCK
    # ══════════════════════════════════════════════════
    def _tick_clock(self):
        self.clock_lbl.config(
            text=datetime.now().strftime("%H:%M:%S  %d/%m/%Y"))
        self.root.after(1000, self._tick_clock)

    # ══════════════════════════════════════════════════
    #  CLOSE
    # ══════════════════════════════════════════════════
    def on_close(self):
        self.running   = False
        self.connected = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        if self.ws_conn:
            try:
                self.ws_conn.close()
            except Exception:
                pass
        plt.close('all')
        self.root.destroy()


# ══════════════════════════════════════════════════════
if __name__ == "__main__":
    root = tk.Tk()
    app  = PatientECGMonitor(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
