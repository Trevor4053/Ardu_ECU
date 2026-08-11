"""
Ardu_ECU Rev11 - serial engine GUI + CSV logger.

Connects to the ECU over a USB serial port (the same interface the firmware's
SendSerialTelemetry / ProcessSerialCommands use), reads the 1 Hz JSON telemetry
line, sends commands, and logs every reading to CSV + JSONL.

Controls (all buttons/sliders send commands to the ECU serial interface):
  START            -> mode signal 100, throttle 0  (engine enters purge)
  STOP             -> mode signal 0                (engine enters cooldown)
  ABORT            -> immediate AbortAll()
  RESET            -> clear error, back to Auto Start
  RC               -> release serial override, back to real RC/switch control
  PING             -> liveness check (replies CMD:PING OK)
  MODE <0-6>       -> force a trial mode (5 = starter only, 6 = fuel pump only)
  Throttle slider  -> THROTTLE 0-100

Run:  python engine_gui.py
Self-test (no serial port needed):  python engine_gui.py --selftest
"""

import csv
import json
import math
import os
import queue
import re
import sys
import tempfile
import threading
import time
import tkinter as tk
from tkinter import ttk, scrolledtext

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None

# --------------------------------------------------------------------------
# config
# --------------------------------------------------------------------------
BAUD = 115200  # matches Serial.begin(115200) in ECU_Rev11.ino
HEARTBEAT_MS = 2000  # PING interval while connected; keeps the firmware
                     # serial-override watchdog (5000 ms) from tripping
CSV_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "engine_log.csv")
JSONL_LOG = os.path.join(os.path.dirname(os.path.abspath(__file__)), "engine_log.jsonl")

MODES = {
    0: "Auto Start",
    1: "Starting",
    2: "Idling",
    3: "Operating",
    4: "Cooldown",
    5: "Trial Starter",
    6: "Trial Fuel",
}
ERRORS = {
    0: "No ignition",
    1: "Temp exceeded",
    2: "RPM exceeded",
    3: "RC signal lost",
    4: "Flameout",
    5: "RPM sensor failure",
    6: "No fuel / no accel",
    7: "Unable to reach idle RPM",
}

MAX_RPM = 110000
MAX_TEMP = 800
OUTPUT_MAX = 1000

CSV_HEADER = [
    "wallclock", "t_ms", "mode", "mode_name", "stage",
    "throttle", "modesig", "rpm", "temp", "volt", "run_min",
    "fuel", "starter", "glow", "gas", "fuelv", "err", "err_names", "loop_ms",
]

THEMES = {
    "dark": {
        "bg": "#1e1e1e", "fg": "#e8e8e8", "panel": "#2d2d2d",
        "panel_hi": "#3a3a3a", "panel_lo": "#222222", "border": "#444444",
        "entry": "#2d2d2d", "trough": "#111111", "status": "#7cf57c",
        "log_bg": "#111111", "log_fg": "#d0d0d0", "fg_dim": "#666666",
    },
    "light": {
        "bg": "#f0f0f0", "fg": "#1a1a1a", "panel": "#e6e6e6",
        "panel_hi": "#ffffff", "panel_lo": "#cccccc", "border": "#a0a0a0",
        "entry": "#ffffff", "trough": "#d0d0d0", "status": "#0a5",
        "log_bg": "#ffffff", "log_fg": "#000000", "fg_dim": "#888888",
    },
}

DIAL_THEMES = {
    "dark": {"face": "#000000", "rim": "#7a7a7a", "band": "#2a2a2a",
             "tick": "#dddddd", "num": "#dddddd", "value": "#ffffff",
             "hub": "#555555", "hub_rim": "#aaaaaa"},
    "light": {"face": "#fafafa", "rim": "#7a7a7a", "band": "#d5d5d5",
              "tick": "#222222", "num": "#222222", "value": "#111111",
              "hub": "#999999", "hub_rim": "#666666"},
}


# --------------------------------------------------------------------------
# serial parsing
# --------------------------------------------------------------------------
def parse_telemetry(line):
    line = line.strip()
    if not (line.startswith("{") and line.endswith("}")):
        return None
    try:
        d = json.loads(line)
    except (ValueError, TypeError):
        return None
    if not all(k in d for k in ("t", "mode", "rpm")):
        return None
    return d


def err_names(err):
    if not err:
        return "None"
    names = [ERRORS[i] for i in sorted(ERRORS) if err & (1 << i)]
    return ",".join(names) if names else "Unknown(0x%x)" % err


# --------------------------------------------------------------------------
# serial port link
# --------------------------------------------------------------------------
class SerialLink:
    def __init__(self, port, baud, events, status_cb):
        self.port_name = port
        self.baud = baud
        self.events = events
        self.status_cb = status_cb
        self.port = None
        self._lock = threading.Lock()
        self._stop_flag = threading.Event()
        self._reader_thread = None

    @property
    def alive(self):
        return self.port is not None and self.port.is_open

    def start(self):
        self._stop_flag.clear()
        self.port = serial.Serial(self.port_name, self.baud, timeout=0.1)
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()
        self.status_cb("connected to %s @ %d" % (self.port_name, self.baud))

    def send(self, cmd):
        with self._lock:
            if not self.alive:
                return False
            try:
                self.port.write((cmd + "\n").encode("utf-8"))
                return True
            except (serial.SerialException, OSError):
                return False

    def close(self):
        self._stop_flag.set()
        with self._lock:
            if self.port is not None:
                try:
                    self.port.close()
                except serial.SerialException:
                    pass
                self.port = None
        self.status_cb("disconnected")

    def _reader_loop(self):
        buf = b""
        while not self._stop_flag.is_set():
            if not self.alive:
                if not self._stop_flag.is_set():
                    self.events.put(("disconnect", "serial port closed"))
                break
            try:
                data = self.port.read(256)
            except serial.SerialException:
                self.events.put(("disconnect", "serial read error"))
                break
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.decode("utf-8", errors="replace"))

    def _handle_line(self, line):
        tele = parse_telemetry(line)
        if tele is not None:
            tele["_wallclock"] = time.strftime("%Y-%m-%d %H:%M:%S")
            self.events.put(("telemetry", tele))
            return
        s = line.strip()
        if s.startswith("CMD:"):
            self.events.put(("ack", s))
            return
        if s and not s.startswith(("E (", "W (", "I (")):
            self.events.put(("notice", s))


# --------------------------------------------------------------------------
# CSV + JSONL logger
# --------------------------------------------------------------------------
class Logger:
    def __init__(self, csv_path, jsonl_path):
        self.csv_path = csv_path
        self.jsonl_path = jsonl_path
        self._csv = None
        self._csvw = None
        self._json = None
        self.count = 0

    def start(self):
        new = not os.path.exists(self.csv_path) or os.path.getsize(self.csv_path) == 0
        self._csv = open(self.csv_path, "a", newline="", encoding="utf-8")
        self._csvw = csv.writer(self._csv)
        if new:
            self._csvw.writerow(CSV_HEADER)
        self._json = open(self.jsonl_path, "a", encoding="utf-8")

    def log(self, tele):
        if self._csv is None:
            return
        mode = tele.get("mode", 0)
        row = [
            tele.get("_wallclock", ""),
            tele.get("t", ""),
            mode,
            MODES.get(mode, "?"),
            tele.get("stage", ""),
            tele.get("thr", ""),
            tele.get("modesig", ""),
            tele.get("rpm", ""),
            tele.get("temp", ""),
            tele.get("volt", ""),
            tele.get("run", ""),
            tele.get("fuel", ""),
            tele.get("starter", ""),
            tele.get("glow", ""),
            tele.get("gas", ""),
            tele.get("fuelv", ""),
            tele.get("err", ""),
            err_names(tele.get("err", 0)),
            tele.get("loop", ""),
        ]
        try:
            self._csvw.writerow(row)
            self._csv.flush()
            self._json.write(json.dumps(tele) + "\n")
            self._json.flush()
            self.count += 1
        except (OSError, ValueError):
            pass

    def stop(self):
        for f in (self._csv, self._json):
            if f is not None:
                try:
                    f.close()
                except OSError:
                    pass
        self._csv = self._json = None


# --------------------------------------------------------------------------
# aviation-style dial gauge (canvas)
# --------------------------------------------------------------------------
class DialGauge(ttk.Frame):
    SWEEP_START = 225   # lower-left (min)
    SWEEP_EXTENT = 270  # clockwise over the top to lower-right (max)

    def __init__(self, parent, label, vmin, vmax,
                 yellow_from, yellow_to, red_from,
                 major_step, minor_step, label_scale=1,
                 needle_color="#ffe066", size=190, unit=""):
        super().__init__(parent)
        self.vmin = float(vmin)
        self.vmax = float(vmax)
        self.yellow_from = yellow_from
        self.yellow_to = yellow_to
        self.red_from = red_from
        self.major_step = float(major_step)
        self.minor_step = float(minor_step)
        self.label_scale = float(label_scale)
        self.unit = unit
        self.size = size
        self.cx = size / 2.0
        self.cy = size / 2.0
        self._needle_color = needle_color
        self._last = None
        self._dark = True

        self.canvas = tk.Canvas(self, width=size, height=size,
                                highlightthickness=0, bd=0)
        self.canvas.pack()
        ttk.Label(self, text=label, font=("Arial", 9, "bold")).pack(pady=(4, 0))

        self.apply_theme(True)

    def _angle(self, v):
        f = (float(v) - self.vmin) / (self.vmax - self.vmin)
        return self.SWEEP_START - f * self.SWEEP_EXTENT

    def _point(self, r, angle):
        rad = math.radians(angle)
        return (self.cx + r * math.cos(rad), self.cy - r * math.sin(rad))

    def _band(self, v_from, v_to, color, r):
        a0 = self._angle(v_from)
        a1 = self._angle(v_to)
        self.canvas.create_arc(
            self.cx - r, self.cy - r, self.cx + r, self.cy + r,
            start=a0, extent=a1 - a0, style=tk.ARC, width=7, outline=color)

    def _tick_values(self):
        vals = []
        v = 0.0
        while v <= self.vmax + 1e-9:
            vals.append(v)
            v += self.minor_step
        if vals and abs(vals[-1] - self.vmax) > 1e-6:
            vals.append(self.vmax)
        return vals

    def _draw_face(self, t):
        c = self.canvas
        cx, cy, R = self.cx, self.cy, self.size / 2.0 - 6
        c.create_oval(cx - R, cy - R, cx + R, cy + R,
                      outline=t["rim"], width=4, fill=t["face"])
        band_r = R - 16
        self._band(self.vmin, self.vmax, t["band"], band_r)
        if self.yellow_from is not None and self.yellow_to is not None:
            self._band(self.yellow_from, self.yellow_to, "#e6c200", band_r)
        if self.red_from is not None:
            self._band(self.red_from, self.vmax, "#e01", band_r)

        ticks_r = band_r - 8
        for v in self._tick_values():
            major = (abs((v / self.major_step) - round(v / self.major_step)) < 1e-6
                     or abs(v - self.vmax) < 1e-6)
            a = self._angle(v)
            r_in = ticks_r - (11 if major else 5)
            c.create_line(*self._point(r_in, a), *self._point(ticks_r, a),
                          fill=t["tick"], width=2 if major else 1)
            if major:
                c.create_text(*self._point(r_in - 12, a),
                              text=str(int(round(v / self.label_scale))),
                              fill=t["num"], font=("Arial", 8))

        c.create_oval(cx - 6, cy - 6, cx + 6, cy + 6,
                      fill=t["hub"], outline=t["hub_rim"], width=2)

    def apply_theme(self, dark):
        self._dark = bool(dark)
        t = DIAL_THEMES["dark" if dark else "light"]
        c = self.canvas
        c.delete("all")
        c.configure(bg=t["face"])
        self._draw_face(t)
        self._needle = c.create_line(0, 0, 0, 0,
                                     fill=self._needle_color, width=3)
        self._needle_tail = c.create_line(0, 0, 0, 0,
                                          fill=self._needle_color, width=3)
        self._value_item = c.create_text(
            self.cx, self.cy + 0.22 * self.size, text="--",
            fill=t["value"], font=("Consolas", 11, "bold"))
        self.set_value(self._last)

    def set_value(self, v):
        if v is None:
            v = self.vmin
        v = max(self.vmin, min(self.vmax, float(v)))
        self._last = v
        a = self._angle(v)
        R = self.size / 2.0 - 6
        x, y = self._point(0.80 * R, a)
        tx, ty = self._point(-0.15 * R, a)
        self.canvas.coords(self._needle, self.cx, self.cy, x, y)
        self.canvas.coords(self._needle_tail, self.cx, self.cy, tx, ty)
        text = ("%.0f %s" % (v, self.unit)) if self.unit else "%.0f" % v
        self.canvas.itemconfigure(self._value_item, text=text)


# --------------------------------------------------------------------------
# solenoid valve icon (canvas)
# --------------------------------------------------------------------------
class ValveIcon(ttk.Frame):
    W = 78
    H = 46

    def __init__(self, parent, label):
        super().__init__(parent)
        self.open = False
        self.canvas = tk.Canvas(self, width=self.W, height=self.H,
                                highlightthickness=0, bd=0)
        self.canvas.pack()
        ttk.Label(self, text=label, font=("Arial", 8, "bold")).pack(pady=(2, 0))

        cx, cy = self.W / 2.0, self.H / 2.0 - 2
        self.canvas.create_line(2, cy, cx - 13, cy, fill="#888", width=4)
        self.canvas.create_line(cx + 13, cy, self.W - 2, cy, fill="#888", width=4)
        self._body = self.canvas.create_oval(cx - 11, cy - 11, cx + 11, cy + 11,
                                             outline="#999", width=2)
        self._disc = self.canvas.create_line(0, 0, 0, 0, fill="#e33", width=5)
        self.set_state(False)

    def set_state(self, open_):
        open_ = bool(open_)
        if open_ == self.open:
            return
        self.open = open_
        cx, cy = self.W / 2.0, self.H / 2.0 - 2
        if open_:
            self.canvas.coords(self._disc, cx - 8, cy, cx + 8, cy)
            self.canvas.itemconfigure(self._disc, fill="#3c3")
            self.canvas.itemconfigure(self._body, outline="#3c3")
        else:
            self.canvas.coords(self._disc, cx, cy - 8, cx, cy + 8)
            self.canvas.itemconfigure(self._disc, fill="#e33")
            self.canvas.itemconfigure(self._body, outline="#e33")

    def apply_theme(self, dark):
        t = THEMES["dark" if dark else "light"]
        self.canvas.configure(bg=t["bg"])


# --------------------------------------------------------------------------
# GUI
# --------------------------------------------------------------------------
class App:
    def __init__(self, root):
        self.root = root
        root.title("Ardu_ECU Rev11 - Serial Engine GUI")
        root.geometry("1060x640")
        root.minsize(900, 560)

        self.events = queue.Queue()
        self.link = None
        self.logger = Logger(CSV_LOG, JSONL_LOG)
        self.last_tele = {}
        self.gauge_bars = {}
        self.sliders = {}
        self.dark = True

        self._build_ui()
        self._apply_theme(self.dark)
        self.root.bind_all("<KeyPress-w>", lambda e: self._key_throttle(1))
        self.root.bind_all("<KeyPress-W>", lambda e: self._key_throttle(1))
        self.root.bind_all("<KeyPress-s>", lambda e: self._key_throttle(-1))
        self.root.bind_all("<KeyPress-S>", lambda e: self._key_throttle(-1))
        self.root.bind_all("<KeyPress-x>", lambda e: self._key_cut())
        self.root.bind_all("<KeyPress-X>", lambda e: self._key_cut())
        self.root.after(200, self._drain_events)
        self.root.after(HEARTBEAT_MS, self._heartbeat)

    # ---- UI construction -------------------------------------------------
    def _build_ui(self):
        self._build_topbar()
        self._build_body()
        self._build_logpanel()

    def _build_topbar(self):
        bar = ttk.Frame(self.root, padding=(8, 6))
        bar.pack(fill=tk.X)

        self.theme_btn = ttk.Button(bar, text="Light mode", command=self._toggle_theme)
        self.theme_btn.pack(side=tk.LEFT)

        self.status_var = tk.StringVar(value="not connected")
        self.status_lbl = tk.Label(bar, textvariable=self.status_var, fg="#0a5")
        self.status_lbl.pack(side=tk.LEFT, padx=(10, 0))

        self.conn_btn = ttk.Button(bar, text="Connect", command=self._toggle_connect)
        self.conn_btn.pack(side=tk.RIGHT, padx=2)

        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(bar, textvariable=self.port_var, width=14)
        self.port_box.pack(side=tk.RIGHT, padx=2)
        ttk.Button(bar, text="Refresh", command=self.refresh_ports).pack(side=tk.RIGHT, padx=2)
        ttk.Label(bar, text="Port:").pack(side=tk.RIGHT, padx=(8, 2))
        self.refresh_ports()

    def _build_body(self):
        body = ttk.Frame(self.root, padding=(8, 4))
        body.pack(fill=tk.BOTH, expand=True)

        # gauges (left)
        g = ttk.LabelFrame(body, text="Gauges", padding=8)
        g.pack(side=tk.LEFT, fill=tk.BOTH, padx=(0, 6))

        dials = ttk.Frame(g)
        dials.pack(fill=tk.X)
        self.rpm_gauge = DialGauge(
            dials, "RPM", 0, MAX_RPM + 10000,
            MAX_RPM - 10000, MAX_RPM, MAX_RPM,
            20000, 5000, label_scale=1000, needle_color="#ff5522")
        self.rpm_gauge.pack(side=tk.LEFT, padx=(0, 8))
        self.egt_gauge = DialGauge(
            dials, "Exhaust Temp C", 0, 1100,
            630, 750, 750,
            200, 50, unit="C", needle_color="#ffe066")
        self.egt_gauge.pack(side=tk.LEFT)

        self._gauge_row(g, "Throttle %", 0, 100)
        self._gauge_row(g, "Voltage V", 0, 20)

        # state (middle)
        s = ttk.LabelFrame(body, text="Engine State", padding=8)
        s.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6)
        self.state_vars = {}
        rows = [
            ("Mode", ""), ("Stage", ""), ("Error", ""),
            ("Fuel Pump", ""), ("Starter", ""), ("Glow Plug", ""),
            ("Gas Valve", ""), ("Fuel Valve", ""), ("Loop Time", ""),
            ("Run Time", ""),
        ]
        for i, (name, _) in enumerate(rows):
            ttk.Label(s, text=name + ":").grid(row=i, column=0, sticky=tk.E, pady=1)
            v = tk.StringVar(value="--")
            ttk.Label(s, textvariable=v).grid(row=i, column=1, sticky=tk.W, padx=(8, 0))
            self.state_vars[name] = v

        valve_bar = ttk.Frame(s)
        valve_bar.grid(row=10, column=0, columnspan=2, sticky=tk.W, pady=(14, 0))
        self.gas_valve = ValveIcon(valve_bar, "GAS SOLENOID")
        self.gas_valve.pack(side=tk.LEFT, padx=(0, 14))
        self.fuel_valve = ValveIcon(valve_bar, "FUEL SOLENOID")
        self.fuel_valve.pack(side=tk.LEFT)

        # controls (right)
        c = ttk.LabelFrame(body, text="Controls", padding=8)
        c.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(6, 0))

        ttk.Button(c, text="START", command=lambda: self.cmd("START"),
                   width=14).grid(row=0, column=0, pady=3)
        ttk.Button(c, text="STOP", command=lambda: self.cmd("STOP"),
                   width=14).grid(row=0, column=1, pady=3)
        ttk.Button(c, text="ABORT", command=lambda: self.cmd("ABORT"),
                   width=14).grid(row=1, column=0, pady=3)
        ttk.Button(c, text="RESET", command=lambda: self.cmd("RESET"),
                   width=14).grid(row=1, column=1, pady=3)
        ttk.Button(c, text="RC (release override)", command=lambda: self.cmd("RC"),
                   width=20).grid(row=2, column=0, columnspan=2, pady=3, sticky=tk.EW)
        ttk.Button(c, text="PING", command=lambda: self.cmd("PING"),
                   width=14).grid(row=3, column=0, columnspan=2, pady=3, sticky=tk.EW)

        ttk.Label(c, text="MODE").grid(row=4, column=0, sticky=tk.W, pady=(10, 0))
        self.mode_var = tk.StringVar()
        self.mode_box = ttk.Combobox(c, textvariable=self.mode_var, state="readonly",
                                     width=16, values=list(MODES.values()))
        self.mode_box.current(0)
        self.mode_box.grid(row=4, column=1, sticky=tk.W, pady=(10, 0))
        self.mode_box.bind("<<ComboboxSelected>>", self._on_mode)

        self._slider(c, "Throttle %", "THROTTLE", 0, 100, 5, 0, row=5)

        tk.Label(c, text="W / S: throttle up / down    X: cut to 0",
                 fg="#666").grid(row=6, column=0, columnspan=2, sticky=tk.W, pady=(8, 0))

        tk.Label(c, text="Outputs are 0-1000 units.", fg="#666").grid(
            row=7, column=0, columnspan=2, sticky=tk.W, pady=(12, 0))

    def _gauge_row(self, parent, label, lo, hi):
        row = ttk.Frame(parent)
        row.pack(fill=tk.X, pady=2)
        ttk.Label(row, text=label).pack(side=tk.LEFT)
        bar = ttk.Progressbar(row, length=170, maximum=hi - lo)
        bar.pack(side=tk.LEFT, padx=8)
        val = tk.StringVar(value="--")
        ttk.Label(row, textvariable=val, width=8, anchor=tk.E).pack(side=tk.LEFT)
        self.gauge_bars[label] = (bar, lo, val)

    def _slider(self, parent, label, cmd, lo, hi, step, default, row):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky=tk.W, pady=(10, 0))
        var = tk.DoubleVar(value=default)
        ttk.Scale(parent, from_=lo, to=hi, variable=var, length=150,
                  command=lambda v, c=cmd, x=var: self._on_slider(c, x)).grid(
            row=row, column=1, sticky=tk.W, pady=(10, 0))
        val = tk.StringVar(value=str(default))
        ttk.Label(parent, textvariable=val, width=8, anchor=tk.E).grid(
            row=row, column=2, pady=(10, 0))
        var._val_lbl = val
        self.sliders[cmd] = var

    def _build_logpanel(self):
        f = ttk.LabelFrame(self.root, text="Event / Command Log", padding=4)
        f.pack(fill=tk.BOTH, padx=8, pady=(0, 8))

        header = ttk.Frame(f)
        header.pack(fill=tk.X, pady=(0, 2))
        ttk.Label(header, text="Logs:").pack(side=tk.LEFT)
        ttk.Label(header, text=CSV_LOG, foreground="#3465a4",
                  font=("Consolas", 8)).pack(side=tk.LEFT, padx=(6, 0))
        ttk.Label(header, text=JSONL_LOG, foreground="#3465a4",
                  font=("Consolas", 8)).pack(side=tk.LEFT, padx=(8, 0))
        ttk.Button(header, text="Open CSV", width=9,
                   command=lambda: self._open_log(CSV_LOG)).pack(side=tk.RIGHT)
        ttk.Button(header, text="Open JSONL", width=9,
                   command=lambda: self._open_log(JSONL_LOG)).pack(side=tk.RIGHT, padx=2)
        ttk.Button(header, text="Open folder", width=10,
                   command=self._open_log_folder).pack(side=tk.RIGHT, padx=2)

        self.log_text = scrolledtext.ScrolledText(f, height=9, state=tk.DISABLED,
                                                  font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def _open_log(self, path):
        if not os.path.exists(path):
            self._log("! log file not created yet: %s" % path)
            return
        try:
            os.startfile(path)
        except OSError as e:
            self._log("! could not open %s: %s" % (path, e))

    def _open_log_folder(self):
        folder = os.path.dirname(CSV_LOG)
        try:
            os.startfile(folder)
        except OSError as e:
            self._log("! could not open folder: %s" % e)

    # ---- state -----------------------------------------------------------
    def _log(self, msg):
        self.log_text.configure(state=tk.NORMAL)
        self.log_text.insert(tk.END, msg + "\n")
        self.log_text.see(tk.END)
        self.log_text.configure(state=tk.DISABLED)

    def _set_status(self, msg):
        self.status_var.set(msg)

    # ---- theme -----------------------------------------------------------
    def _toggle_theme(self):
        self._apply_theme(not self.dark)

    def _apply_theme(self, dark):
        self.dark = bool(dark)
        t = THEMES["dark" if dark else "light"]
        self.root.configure(bg=t["bg"])
        style = ttk.Style(self.root)
        style.theme_use("clam")
        style.configure(".", background=t["bg"], foreground=t["fg"])
        style.configure("TFrame", background=t["bg"])
        style.configure("TLabel", background=t["bg"], foreground=t["fg"])
        style.configure("TButton", background=t["panel"], foreground=t["fg"])
        style.map("TButton",
                  background=[("active", t["panel_hi"]), ("pressed", t["panel_lo"])],
                  foreground=[("disabled", t["fg_dim"])])
        style.configure("TLabelframe", background=t["bg"], bordercolor=t["border"])
        style.configure("TLabelframe.Label", background=t["bg"], foreground=t["fg"])
        style.configure("TCombobox", fieldbackground=t["entry"],
                        background=t["panel"], foreground=t["fg"],
                        arrowcolor=t["fg"])
        style.map("TCombobox", fieldbackground=[("readonly", t["entry"])])
        style.configure("TScale", background=t["bg"], troughcolor=t["trough"])
        self.status_lbl.configure(fg=t["status"])
        self.log_text.configure(bg=t["log_bg"], fg=t["log_fg"],
                                insertbackground=t["fg"])
        self.theme_btn.configure(text="Light mode" if dark else "Dark mode")
        self.rpm_gauge.apply_theme(dark)
        self.egt_gauge.apply_theme(dark)
        self.gas_valve.apply_theme(dark)
        self.fuel_valve.apply_theme(dark)

    # ---- port selection --------------------------------------------------
    def refresh_ports(self):
        if serial is None:
            self._log("! pyserial not installed (pip install pyserial)")
            return
        ports = []
        try:
            for p in list_ports.comports():
                ports.append(p.device)
        except Exception as e:
            self._log("! could not list ports: %s" % e)
        current = self.port_var.get()
        self.port_box["values"] = ports
        if not current and ports:
            self.port_var.set(ports[0])

    # ---- serial lifecycle ------------------------------------------------
    def _toggle_connect(self):
        if self.link is not None and self.link.alive:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        if self.link is not None and self.link.alive:
            self._log("already connected")
            return
        port = self.port_var.get().strip()
        if not port:
            self._log("! no serial port selected")
            return
        if self.logger._csv is None:
            self.logger.start()
            self._log("logging to: %s" % self.logger.csv_path)
        self.link = SerialLink(port, BAUD, self.events, self._set_status)
        try:
            self.link.start()
        except serial.SerialException as e:
            self._log("! connect failed on %s: %s" % (port, e))
            self.link = None
            self._set_status("connect failed")
            self.conn_btn.configure(text="Connect")
            return
        self.conn_btn.configure(text="Disconnect")
        self._log("connected to %s @ %d" % (port, BAUD))

    def disconnect(self):
        if self.link is not None:
            self.link.close()
        self.conn_btn.configure(text="Connect")
        self._set_status("disconnected")

    def on_close(self):
        self.disconnect()
        self.logger.stop()
        self.root.destroy()

    # ---- commands --------------------------------------------------------
    def cmd(self, text):
        if self.link is None or not self.link.alive:
            self._log("! not connected - command ignored: %s" % text)
            return
        if self.link.send(text):
            self._log("> %s" % text)
        else:
            self._log("! send failed: %s" % text)

    def _on_slider(self, cmd, var):
        try:
            val = int(round(var.get()))
        except Exception:
            return
        if hasattr(var, "_last_sent") and var._last_sent == val:
            return
        var._last_sent = val
        if hasattr(var, "_val_lbl"):
            var._val_lbl.set(str(val))
        self.cmd("%s %d" % (cmd, val))

    def _key_throttle(self, delta):
        w = self.root.focus_get()
        if isinstance(w, (tk.Entry, ttk.Combobox)):
            return
        var = self.sliders["THROTTLE"]
        var.set(max(0, min(100, int(round(var.get())) + delta)))
        self._on_slider("THROTTLE", var)

    def _key_cut(self):
        w = self.root.focus_get()
        if isinstance(w, (tk.Entry, ttk.Combobox)):
            return
        var = self.sliders["THROTTLE"]
        var.set(0)
        self._on_slider("THROTTLE", var)

    def _on_mode(self, _event=None):
        idx = self.mode_box.current()
        if idx >= 0:
            self.cmd("MODE %d" % idx)

    # ---- event drain -----------------------------------------------------
    def _drain_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "telemetry":
                    self._on_telemetry(payload)
                elif kind == "ack":
                    # heartbeat PING replies are expected spam; don't log them
                    if not payload.startswith("CMD:PING"):
                        self._log("< %s" % payload)
                elif kind == "notice":
                    self._log(payload)
                elif kind == "disconnect":
                    self._log("! %s" % payload)
                    self.conn_btn.configure(text="Connect")
                    self._set_status("disconnected")
        except queue.Empty:
            pass
        self.root.after(200, self._drain_events)

    def _heartbeat(self):
        # While connected, keep sending PING so the firmware's serial-override
        # watchdog (5000 ms) never expires even when nothing else is changed.
        if self.link is not None and self.link.alive:
            self.link.send("PING")
        self.root.after(HEARTBEAT_MS, self._heartbeat)

    def _on_telemetry(self, d):
        self.last_tele = d
        self.logger.log(d)

        mode = d.get("mode", 0)
        self.state_vars["Mode"].set(MODES.get(mode, "?%d" % mode))
        self.state_vars["Stage"].set(d.get("stage", "?"))
        err = d.get("err", 0)
        self.state_vars["Error"].set("%d (%s)" % (err, err_names(err)))
        self.state_vars["Fuel Pump"].set(d.get("fuel", 0))
        self.state_vars["Starter"].set(d.get("starter", 0))
        self.state_vars["Glow Plug"].set(d.get("glow", 0))
        self.state_vars["Gas Valve"].set("ON" if d.get("gas") else "OFF")
        self.state_vars["Fuel Valve"].set("ON" if d.get("fuelv") else "OFF")
        self.gas_valve.set_state(d.get("gas", 0))
        self.fuel_valve.set_state(d.get("fuelv", 0))
        self.state_vars["Loop Time"].set("%d ms" % d.get("loop", 0))
        self.state_vars["Run Time"].set("%d min" % d.get("run", 0))

        self.rpm_gauge.set_value(d.get("rpm", 0))
        self.egt_gauge.set_value(d.get("temp", 0))

        for label, key in (("Throttle %", "thr"), ("Voltage V", "volt")):
            bar, lo, val = self.gauge_bars[label]
            v = d.get(key, 0)
            bar["value"] = max(0, min(bar["maximum"], float(v)))
            val.set(("%.1f" % v) if isinstance(v, float) else str(v))

        uptime = d.get("t", 0) / 1000.0
        self._set_status(
            "connected %ds  mode=%s  rpm=%s  temp=%s  err=%s  run=%s min  rows=%d" % (
                int(uptime), MODES.get(mode, mode), d.get("rpm"), d.get("temp"),
                err, d.get("run", 0), self.logger.count))


def main():
    if "--selftest" in sys.argv:
        selftest()
        return
    root = tk.Tk()
    app = App(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()


# --------------------------------------------------------------------------
# offline self-test (no serial port needed)
# --------------------------------------------------------------------------
# Command grammar the GUI may send, matching ECU_Rev11.ino ProcessSerialCommands:
#   START | STOP | THROTTLE <0-100> | MODE <0-6> | SETRPM <0-100000> |
#   SETTEMP <0-1000> | ABORT | RESET | RC | PING | HELP   (case-insensitive)
COMMAND_VERBS = ("START", "STOP", "THROTTLE", "MODE", "SETRPM", "SETTEMP",
                 "ABORT", "RESET", "RC", "PING", "HELP")
NOARG_VERBS = {"START", "STOP", "ABORT", "RESET", "RC", "PING", "HELP"}


def command_valid(text):
    parts = text.strip().split()
    if not parts:
        return False
    verb = parts[0].upper()
    if verb not in COMMAND_VERBS:
        return False
    if verb in NOARG_VERBS:
        return len(parts) == 1
    return len(parts) == 2 and parts[1].isdigit() and _command_in_range(verb, int(parts[1]))


def _command_in_range(verb, n):
    if verb == "THROTTLE":
        return 0 <= n <= 100
    if verb == "MODE":
        return 0 <= n <= 6
    if verb == "SETRPM":
        return 0 <= n <= 100000
    if verb == "SETTEMP":
        return 0 <= n <= 1000
    return False


def selftest():
    failures = []

    def check(name, cond, detail=""):
        print(("PASS" if cond else "FAIL"), name, ("- " + detail) if detail else "")
        if not cond:
            failures.append(name)

    good = ('{"t":1000,"mode":1,"stage":"ramp","thr":50,"modesig":100,'
            '"rpm":9000,"temp":500,"volt":12.3,"run":87,"fuel":1,"starter":1,'
            '"glow":1,"gas":1,"fuelv":1,"err":0,"loop":12}')
    d = parse_telemetry(good)
    check("parse valid telemetry", d is not None and d["mode"] == 1
          and d["rpm"] == 9000 and d["stage"] == "ramp")
    check("parse run time", d is not None and d.get("run") == 87)
    check("parse rejects non-json", parse_telemetry("not json {") is None)
    check("parse rejects missing keys", parse_telemetry('{"t":1,"mode":1}') is None)
    check("parse rejects empty", parse_telemetry("") is None)
    check("parse rejects prefix garbage", parse_telemetry("[ESP32] {bogus") is None)

    check("err 0 -> None", err_names(0) == "None")
    check("err bits combine", err_names(1 | 4) == "No ignition,RPM exceeded")
    check("unknown err mask", err_names(1 << 8).startswith("Unknown"))

    with tempfile.TemporaryDirectory() as td:
        lg = Logger(os.path.join(td, "t.csv"), os.path.join(td, "t.jsonl"))
        lg.start()
        lg.log(dict(d, _wallclock="12:00:00"))
        lg.log(dict(d, t=2000, mode=3, err=1))
        lg.stop()
        with open(os.path.join(td, "t.csv"), encoding="utf-8") as f:
            rows = list(csv.reader(f))
        with open(os.path.join(td, "t.jsonl"), encoding="utf-8") as f:
            lines = [ln for ln in f.read().splitlines() if ln.strip()]
        check("csv header + 2 rows", len(rows) == 3 and rows[0] == CSV_HEADER)
        check("csv row content", rows[1][2] == "1" and rows[2][1] == "2000")
        check("csv run_min", rows[1][10] == "87")
        check("jsonl 2 lines", len(lines) == 2 and json.loads(lines[1])["mode"] == 3)
        check("logger count", lg.count == 2)

    gui_cmds = ["START", "STOP", "ABORT", "RESET", "RC", "PING"]
    gui_cmds += ["MODE %d" % i for i in range(7)]
    gui_cmds += ["THROTTLE %d" % v for v in (0, 1, 50, 100)]
    check("all GUI command strings valid", all(command_valid(c) for c in gui_cmds),
          "%d commands" % len(gui_cmds))

    bad = ["", " ", "THROTTLE", "THROTTLE abc", "THROTTLE -5", "THROTTLE 5.5",
           "START 1", "MODEX 9", "MODE 7", "MODE -1", "SETRPM 999999999",
           "PING extra", "GAMEOVER"]
    check("grammar rejects bad commands", all(not command_valid(b) for b in bad))

    print()
    if failures:
        print("SELFTEST: %d FAILED - %s" % (len(failures), ", ".join(failures)))
        sys.exit(1)
    print("SELFTEST: all checks passed")


if __name__ == "__main__":
    main()
