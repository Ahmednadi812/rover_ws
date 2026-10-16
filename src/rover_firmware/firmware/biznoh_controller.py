#!/usr/bin/env python3
"""
BIZNOH Robot Controller
=======================
Serial GUI for Final_BIZNOH.ino (Arduino Mega 2560, 115200 baud, newline-terminated).

    pip install pyserial
    python biznoh_controller.py

Tabs
  Manual : W/A/S/D + STOP, timed motor moves, closed-loop (PID) settings,
           gripper / magnet / siren / sensors, raw command line.
  Auto   : AUTOM session, live status message (large, subtle) with parsed fields,
           velocity / servo / gripper frame sender  ->  R(P/N)vL(P/N)vG(P/N)(1/0)S(deg)

  Gripper: live gripper-cycle state (D -> A -> U -> B -> D), timers, event history,
           and an error / warning list (board ERR lines, gripper faults, link problems)

The log at the bottom shows every command sent (TX) and every line received (RX).
"""

import queue
import re
import sys
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import filedialog, font as tkfont, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("pyserial is required:  pip install pyserial")

# ----------------------------------------------------------------------------
# Settings
# ----------------------------------------------------------------------------
BAUD_DEFAULT = 115200
SLIDER_MAX = 3000            # counts/s range of the auto-mode velocity sliders
LOG_MAX_LINES = 4000
STATUS_RE = re.compile(
    r"^R([PN])(\d+)L([PN])(\d+)VR([PN])(\d+)VL([PN])(\d+)MD([01])G([DABUF])PROX([01]{5})$"
)
GRIP_NAMES = {"D": "DOWN", "A": "MOVING UP", "U": "UP", "B": "MOVING DOWN", "F": "FAULT"}
GRIP_DESC = {
    "D": "Lift at bottom · magnet follows the metal detector · waiting for a GRIP request",
    "A": "Lift moving up (cycle, or the start-up self-test)",
    "U": "Top reached · lift and magnet off, letting the metal drop",
    "B": "Lift lowering (end of cycle, recovery, or self-test)",
    "F": "Lift fault · GRIP disabled until the Arduino is reset",
}
GRIP_LIMITS = {"A": 10, "U": 5, "B": 10}      # seconds: lift timeout / drop wait (from the sketch)
GRIP_ARM_S = 10                               # a GRIP request waits this long for its conditions
STAT_GRIP_RE = re.compile(r"^GRIP st=([DABUF]) US=(\d+)\.(\d+) cm")
# text printed by the board in MANUAL mode (not in auto mode): (substring, state, phase, level)
MSG_RULES = (
    ("sanity failed", "F", "Self-test failed", "fault"),
    ("timeout going down", "F", "Lowering timeout", "fault"),
    ("sanity check, going up", "A", "Self-test · going up", None),
    ("sanity check, going down", "B", "Self-test · going down", None),
    ("sanity ok", "D", "", None),
    ("recovered", "D", "", None),
    ("done, ready", "D", "", None),
    ("request expired", None, None, "warn"),
    ("ignored", None, None, "warn"),
    ("armed", None, None, None),
    ("moving up", "A", "Lifting", None),
    ("top, releasing", "U", "Releasing", None),
    ("lowering", "B", "Lowering", None),
    ("recovering", "B", "Recovery · lowering", "warn"),
)

# Palette
BG, PANEL, FIELD, BTN = "#15171c", "#1d2027", "#12141a", "#2a2f3a"
FG, MUTED, DIM = "#d7dce3", "#7d8896", "#4b5563"
ACCENT, GOOD, WARN, BAD = "#4f9cf9", "#3fb950", "#e3b341", "#ff7b72"
TX_COL, RX_COL = "#6cb6ff", "#7ee2a8"


def pick_mono():
    fams = set(tkfont.families())
    for f in ("JetBrains Mono", "Cascadia Mono", "Consolas", "Menlo", "DejaVu Sans Mono", "Courier New"):
        if f in fams:
            return f
    return "TkFixedFont"


# ----------------------------------------------------------------------------
# Serial link (reader thread -> queue)
# ----------------------------------------------------------------------------
class Link:
    def __init__(self, rxq: queue.Queue):
        self.rxq = rxq
        self.ser = None
        self._stop = threading.Event()
        self._thread = None
        self._wlock = threading.Lock()

    @property
    def connected(self):
        return self.ser is not None and self.ser.is_open

    def open(self, port, baud):
        self.close()
        self.ser = serial.Serial(port, baud, timeout=0.05, write_timeout=1)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._reader, args=(self.ser, self._stop), daemon=True)
        self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=0.5)
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def write(self, line: str):
        with self._wlock:
            self.ser.write((line + "\n").encode("ascii", "replace"))

    def reset_board(self):
        """Pulse DTR: resets an Arduino Mega (same thing that happens when the port is opened)."""
        with self._wlock:
            self.ser.dtr = False
            time.sleep(0.1)
            self.ser.dtr = True

    def _reader(self, ser, stop):
        buf = b""
        while not stop.is_set():
            try:
                data = ser.read(ser.in_waiting or 1)
            except Exception as exc:                      # unplugged etc.
                self.rxq.put(("closed", str(exc)))
                return
            if not data:
                continue
            buf += data
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("ascii", "replace").strip("\r ")
                if line:
                    self.rxq.put(("rx", line))


# ----------------------------------------------------------------------------
# Application
# ----------------------------------------------------------------------------
class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("BIZNOH Controller")
        self.geometry("1180x880")
        self.minsize(980, 700)
        self.configure(bg=BG)

        self.mono = pick_mono()
        self.rxq = queue.Queue()
        self.link = Link(self.rxq)

        # state
        self.auto_on = False
        self.last_frame_t = 0.0
        self.frame_times = deque(maxlen=60)
        self.last_status_log = 0.0
        self._last_key = {}
        self._suppress = False
        self._frame_pending = False
        self._md_job = None
        self.history, self.hist_idx = [], 0
        # gripper view state
        self.g_state, self.g_since, self.g_phase, self.g_src = None, time.monotonic(), "", ""
        self.g_armed_until, self.g_fault_reason = 0.0, ""
        self.g_counts = {"done": 0, "recov": 0, "fault": 0}
        self.err_count, self.err_last = 0, ""

        self._style()
        self._build_topbar()
        self._build_main()
        self._refresh_ports(select=True)

        self.bind("<KeyPress>", self._on_key)
        self.protocol("WM_DELETE_WINDOW", self._on_close)
        self.after(20, self._poll)
        self.after(250, self._tick)
        self.log("info", "Ready. Pick a port and press Connect.  (Opening the port resets the Mega.)")

    # ------------------------------------------------------------------ style
    def _style(self):
        s = ttk.Style(self)
        s.theme_use("clam")
        s.configure(".", background=BG, foreground=FG, fieldbackground=FIELD, bordercolor=BTN,
                    lightcolor=BG, darkcolor=BG, troughcolor=FIELD, focuscolor=BG, font=("TkDefaultFont", 10))
        s.configure("TFrame", background=BG)
        s.configure("Panel.TFrame", background=PANEL)
        s.configure("TLabel", background=BG, foreground=FG)
        s.configure("Muted.TLabel", foreground=MUTED)
        s.configure("TLabelframe", background=BG, bordercolor=BTN, relief="solid")
        s.configure("TLabelframe.Label", background=BG, foreground=MUTED, font=("TkDefaultFont", 9, "bold"))
        s.configure("TButton", background=BTN, foreground=FG, borderwidth=0, padding=(10, 5))
        s.map("TButton", background=[("active", "#3a4150"), ("pressed", ACCENT)])
        s.configure("Accent.TButton", background=ACCENT, foreground="#0b1220")
        s.map("Accent.TButton", background=[("active", "#78b4fb"), ("pressed", "#2f7fe0")])
        s.configure("Danger.TButton", background="#b62324", foreground="white", font=("TkDefaultFont", 10, "bold"))
        s.map("Danger.TButton", background=[("active", "#da3633"), ("pressed", "#8e1519")])
        s.configure("Pad.TButton", font=("TkDefaultFont", 13, "bold"), padding=(0, 10))
        s.configure("TEntry", fieldbackground=FIELD, foreground=FG, insertcolor=FG, padding=4)
        s.configure("TCombobox", fieldbackground=FIELD, background=BTN, foreground=FG, arrowcolor=FG, padding=3)
        s.map("TCombobox", fieldbackground=[("readonly", FIELD)], foreground=[("readonly", FG)])
        self.option_add("*TCombobox*Listbox.background", FIELD)
        self.option_add("*TCombobox*Listbox.foreground", FG)
        self.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
        s.configure("TCheckbutton", background=BG, foreground=FG)
        s.map("TCheckbutton", background=[("active", BG)])
        s.configure("TRadiobutton", background=BG, foreground=FG)
        s.map("TRadiobutton", background=[("active", BG)])
        s.configure("TNotebook", background=BG, borderwidth=0)
        s.configure("TNotebook.Tab", background=PANEL, foreground=MUTED, padding=(22, 8), borderwidth=0)
        s.map("TNotebook.Tab", background=[("selected", BTN)], foreground=[("selected", FG)])
        s.configure("Horizontal.TScale", background=BG)
        s.configure("TPanedwindow", background=BG)
        s.configure("Treeview", background=FIELD, fieldbackground=FIELD, foreground=FG, borderwidth=0,
                    rowheight=22, font=(self.mono, 9))
        s.map("Treeview", background=[("selected", "#24344d")], foreground=[("selected", FG)])
        s.configure("Treeview.Heading", background=BTN, foreground=MUTED, relief="flat", font=("TkDefaultFont", 9, "bold"))
        s.map("Treeview.Heading", background=[("active", "#3a4150")])
        s.configure("Horizontal.TProgressbar", background=ACCENT, troughcolor=FIELD, borderwidth=0, thickness=8)

    # ---------------------------------------------------------------- top bar
    def _build_topbar(self):
        bar = ttk.Frame(self, padding=(12, 10))
        bar.pack(fill="x")
        ttk.Label(bar, text="BIZNOH", font=("TkDefaultFont", 14, "bold")).pack(side="left", padx=(0, 16))

        self.led = tk.Canvas(bar, width=14, height=14, bg=BG, highlightthickness=0)
        self.led.pack(side="left")
        self._led_dot = self.led.create_oval(2, 2, 12, 12, fill=DIM, outline="")

        self.port_var = tk.StringVar()
        self.port_cb = ttk.Combobox(bar, textvariable=self.port_var, width=18)
        self.port_cb.pack(side="left", padx=(8, 4))
        ttk.Button(bar, text="⟳", width=3, command=self._refresh_ports).pack(side="left")
        self.baud_var = tk.StringVar(value=str(BAUD_DEFAULT))
        ttk.Combobox(bar, textvariable=self.baud_var, width=8,
                     values=("9600", "57600", "115200", "250000")).pack(side="left", padx=6)
        self.conn_btn = ttk.Button(bar, text="Connect", style="Accent.TButton", command=self._toggle_connect)
        self.conn_btn.pack(side="left", padx=4)

        ttk.Button(bar, text="■  E-STOP", style="Danger.TButton", command=self._estop).pack(side="right")

    def _refresh_ports(self, select=False):
        ports = list(list_ports.comports())
        names = [p.device for p in ports]
        self.port_cb["values"] = names
        if select and names:
            pref = [p.device for p in ports if any(k in (p.description + p.device).lower()
                                                   for k in ("arduino", "acm", "usbmodem", "wch", "usbserial"))]
            self.port_var.set((pref or names)[0])

    def _toggle_connect(self):
        if self.link.connected:
            self._disconnect()
            return
        port = self.port_var.get().strip()
        if not port:
            self.log("err", "No port selected")
            return
        try:
            self.link.open(port, int(self.baud_var.get()))
        except Exception as exc:
            self.log("err", f"Cannot open {port}: {exc}")
            return
        self._set_led(True)
        self._grip_reset_view()
        self.conn_btn.configure(text="Disconnect")
        self.log("info", f"Connected to {port} @ {self.baud_var.get()}  (board reboots, gripper self-test runs)")

    def _disconnect(self, reason=None):
        if self.link.connected:
            try:
                self.link.write("AUTOM OFF" if self.auto_on else "STOP")
            except Exception:
                pass
        self.link.close()
        self.auto_on = False
        self._set_led(False)
        self.conn_btn.configure(text="Connect")
        self._update_auto_chip()
        self.log("info", reason or "Disconnected")
        if reason:
            self._add_error("Link", reason, "err")

    def _set_led(self, on):
        self.led.itemconfigure(self._led_dot, fill=GOOD if on else DIM)

    # ------------------------------------------------------------ main layout
    def _build_main(self):
        pw = ttk.PanedWindow(self, orient="vertical")
        pw.pack(fill="both", expand=True, padx=10, pady=(0, 10))

        self.nb = ttk.Notebook(pw)
        self.tab_manual = ttk.Frame(self.nb, padding=10)
        self.tab_auto = ttk.Frame(self.nb, padding=10)
        self.nb.add(self.tab_manual, text="Manual")
        self.tab_grip = ttk.Frame(self.nb, padding=10)
        self.nb.add(self.tab_auto, text="Auto")
        self.nb.add(self.tab_grip, text="Gripper")
        self.nb.bind("<<NotebookTabChanged>>", self._on_tab)
        pw.add(self.nb, weight=3)

        self._build_manual(self.tab_manual)
        self._build_auto(self.tab_auto)
        self._build_gripper(self.tab_grip)

        logf = ttk.Frame(pw)
        pw.add(logf, weight=2)
        self._build_log(logf)

    def _on_tab(self, _=None):
        if self.nb.index("current") == 0 and self.auto_on:
            self.log("info", "Leaving Auto tab -> AUTOM OFF")
            self._stop_auto()

    # ================================================================== MANUAL
    def _build_manual(self, p):
        p.columnconfigure(0, weight=1, uniform="m")
        p.columnconfigure(1, weight=1, uniform="m")
        left = ttk.Frame(p)
        right = ttk.Frame(p)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        right.grid(row=0, column=1, sticky="nsew", padx=(6, 0))

        # --- Drive pad
        f = ttk.LabelFrame(left, text="DRIVE  (open loop, 2 s per press)", padding=10)
        f.pack(fill="x", pady=(0, 8))
        pad = ttk.Frame(f)
        pad.pack()
        for (r, c, txt, cmd) in ((0, 1, "W ▲", "W"), (1, 0, "A ◀", "A"), (1, 1, "STOP", "STOP"),
                                 (1, 2, "D ▶", "D"), (2, 1, "S ▼", "S")):
            ttk.Button(pad, text=txt, width=7, style="Pad.TButton",
                       command=lambda c=cmd: self.send(c)).grid(row=r, column=c, padx=3, pady=3)
        sp = ttk.Frame(f)
        sp.pack(fill="x", pady=(8, 0))
        ttk.Label(sp, text="Speed").pack(side="left")
        self.speed_lbl = ttk.Label(sp, text="50 %", width=6, anchor="e")
        self.speed_lbl.pack(side="right")
        self.speed = ttk.Scale(sp, from_=0, to=100, value=50,
                               command=lambda v: self.speed_lbl.configure(text=f"{int(float(v))} %"))
        self.speed.pack(side="left", fill="x", expand=True, padx=8)
        self.speed.bind("<ButtonRelease-1>", lambda e: self.send(f"SPEED {int(self.speed.get())}"))
        self.kb_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(f, text="Keyboard: W A S D, Space = STOP", variable=self.kb_var).pack(anchor="w", pady=(6, 0))

        # --- Timed motor
        f = ttk.LabelFrame(left, text="TIMED MOTOR MOVE  (20 % duty)", padding=10)
        f.pack(fill="x", pady=(0, 8))
        self.mv_motor = tk.StringVar(value="A")
        ttk.Label(f, text="Motor").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.mv_motor, values=("A", "B"), width=4, state="readonly").grid(row=0, column=1, padx=6)
        ttk.Label(f, text="Seconds").grid(row=0, column=2, padx=(10, 0))
        self.mv_sec = tk.StringVar(value="1.5")
        ttk.Entry(f, textvariable=self.mv_sec, width=7).grid(row=0, column=3, padx=6)
        ttk.Button(f, text="MOVE", command=lambda: self.send(f"MOVE {self.mv_motor.get()} {self.mv_sec.get()}")).grid(row=0, column=4, padx=4)
        ttk.Label(f, text="A = right,  B = left", style="Muted.TLabel").grid(row=1, column=0, columnspan=5, sticky="w", pady=(6, 0))

        # --- Gripper & I/O
        f = ttk.LabelFrame(left, text="GRIPPER & I/O", padding=10)
        f.pack(fill="x", pady=(0, 8))
        for i, (t, c) in enumerate((("GRIP", "GRIP"), ("MGNT toggle", "MGNT"), ("SIREN toggle", "SIREN"),
                                    ("PROX", "PROX"), ("MD", "MD"))):
            ttk.Button(f, text=t, command=lambda c=c: self.send(c)).grid(row=i // 3, column=i % 3, padx=3, pady=3, sticky="ew")
            f.columnconfigure(i % 3, weight=1)

        # --- Diagnostics
        f = ttk.LabelFrame(left, text="DIAGNOSTICS", padding=10)
        f.pack(fill="x")
        for i, c in enumerate(("STAT", "HELP", "ENCDBG", "PPR A", "PPR B")):
            ttk.Button(f, text=c, command=lambda c=c: self.send(c)).grid(row=0, column=i, padx=3, sticky="ew")
            f.columnconfigure(i, weight=1)
        r2 = ttk.Frame(f)
        r2.grid(row=1, column=0, columnspan=5, sticky="w", pady=(8, 0))
        ttk.Label(r2, text="INV").pack(side="left")
        self.inv_var = tk.StringVar(value="A")
        ttk.Combobox(r2, textvariable=self.inv_var, values=("A", "B", "LIFT", "ENCA", "ENCB"), width=6, state="readonly").pack(side="left", padx=6)
        ttk.Button(r2, text="Toggle", command=lambda: self.send(f"INV {self.inv_var.get()}")).pack(side="left")
        ttk.Label(r2, text="   TEL ms").pack(side="left")
        self.tel_var = tk.StringVar(value="100")
        ttk.Entry(r2, textvariable=self.tel_var, width=6).pack(side="left", padx=6)
        ttk.Button(r2, text="Set", command=lambda: self.send(f"TEL {self.tel_var.get()}")).pack(side="left")
        ttk.Button(r2, text="Off", command=lambda: self.send("TEL 0")).pack(side="left", padx=4)

        # --- Closed loop
        f = ttk.LabelFrame(right, text="CLOSED LOOP", padding=10)
        f.pack(fill="x", pady=(0, 8))
        r = ttk.Frame(f)
        r.pack(fill="x")
        ttk.Label(r, text="Mode").pack(side="left")
        self.mode_var = tk.StringVar(value="VEL")
        for m, t in (("POS", "Position (counts)"), ("VEL", "Velocity (counts/s)")):
            ttk.Radiobutton(r, text=t, value=m, variable=self.mode_var,
                            command=lambda: self.send(f"MODE {self.mode_var.get()}")).pack(side="left", padx=8)
        r = ttk.Frame(f)
        r.pack(fill="x", pady=6)
        ttk.Label(r, text="Loop").pack(side="left")
        ttk.Button(r, text="ON", style="Accent.TButton", command=lambda: self.send("LOOP ON")).pack(side="left", padx=6)
        ttk.Button(r, text="OFF", command=lambda: self.send("LOOP OFF")).pack(side="left")
        ttk.Button(r, text="ZERO", command=lambda: self.send("ZERO")).pack(side="right")
        ttk.Button(r, text="RESET", command=lambda: self.send("RESET")).pack(side="right", padx=6)

        g = ttk.Frame(f)
        g.pack(fill="x")
        self.tgt_a = self._param(g, 0, "Target A (right)", "0", "TGT A {}")
        self.tgt_b = self._param(g, 1, "Target B (left)", "0", "TGT B {}")
        self.tgt_all = self._param(g, 2, "Target ALL", "0", "TGT ALL {}")
        self.pid_ms = self._param(g, 3, "PID period (ms)", "10", "PID {}")
        self.tol = self._param(g, 4, "Tolerance", "5", "TOL {}")

        f = ttk.LabelFrame(right, text="PID GAINS", padding=10)
        f.pack(fill="x", pady=(0, 8))
        self.gains = {}
        for i, (k, d) in enumerate((("P", "1.0"), ("I", "0.0"), ("D", "0.0"))):
            ttk.Label(f, text=k).grid(row=0, column=i * 2, padx=(0, 4))
            v = tk.StringVar(value=d)
            e = ttk.Entry(f, textvariable=v, width=8)
            e.grid(row=0, column=i * 2 + 1, padx=(0, 10))
            e.bind("<Return>", lambda _e: self._apply_gains())
            self.gains[k] = v
        ttk.Button(f, text="Apply", style="Accent.TButton", command=self._apply_gains).grid(row=0, column=6, padx=4)

        f = ttk.LabelFrame(right, text="PWM LIMITS  (% duty)", padding=10)
        f.pack(fill="x", pady=(0, 8))
        self.pwm_target = tk.StringVar(value="both")
        ttk.Label(f, text="Motor").grid(row=0, column=0, sticky="w")
        ttk.Combobox(f, textvariable=self.pwm_target, values=("both", "A", "B"), width=6, state="readonly").grid(row=0, column=1, padx=6)
        self.pwm_max, self.pwm_min = tk.StringVar(value="100"), tk.StringVar(value="0")
        for row, (name, var, cmd) in enumerate((("PWMMAX", self.pwm_max, "PWMMAX"), ("PWMMIN", self.pwm_min, "PWMMIN")), start=1):
            ttk.Label(f, text=name).grid(row=row, column=0, sticky="w", pady=3)
            ttk.Entry(f, textvariable=var, width=8).grid(row=row, column=1, padx=6)
            ttk.Button(f, text="Set", width=6, command=lambda v=var, c=cmd: self._send_pwm(c, v.get())).grid(row=row, column=2)

        # --- Raw command
        f = ttk.LabelFrame(right, text="Tip", padding=10)
        f.pack(fill="x")
        ttk.Label(f, style="Muted.TLabel", justify="left", wraplength=420,
                  text="Any command can also be typed into the command line below the log "
                       "(↑/↓ for history). W/A/S/D/MOVE switch the PID loop off; use LOOP ON to re-enable it.").pack(anchor="w")

    def _param(self, parent, row, label, default, fmt, width=9):
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", padx=(0, 8), pady=2)
        var = tk.StringVar(value=default)
        e = ttk.Entry(parent, textvariable=var, width=width)
        e.grid(row=row, column=1, pady=2)

        def go(_=None):
            v = var.get().strip()
            if v:
                self.send(fmt.format(v))
        ttk.Button(parent, text="Set", width=6, command=go).grid(row=row, column=2, padx=6)
        e.bind("<Return>", go)
        return var

    def _apply_gains(self):
        for k, v in self.gains.items():
            if v.get().strip():
                self.send(f"{k} {v.get().strip()}")

    def _send_pwm(self, cmd, val):
        t = self.pwm_target.get()
        self.send(f"{cmd} {val.strip()}" + ("" if t == "both" else f" {t}"))

    # =================================================================== AUTO
    def _build_auto(self, p):
        # ---- Large, subtle status display
        disp = tk.Frame(p, bg=PANEL)
        disp.pack(fill="x")
        top = tk.Frame(disp, bg=PANEL)
        top.pack(fill="x", padx=16, pady=(10, 0))
        self.auto_chip = tk.Label(top, text="AUTO OFF", bg=PANEL, fg=DIM, font=("TkDefaultFont", 10, "bold"))
        self.auto_chip.pack(side="left")
        tk.Label(top, text="STATUS MESSAGE", bg=PANEL, fg=DIM, font=("TkDefaultFont", 9)).pack(side="right")

        self.raw_lbl = tk.Label(disp, text="—", bg=PANEL, fg="#8a94a3", font=(self.mono, 24), anchor="w")
        self.raw_lbl.pack(fill="x", padx=16, pady=(2, 8))

        cells = tk.Frame(disp, bg=PANEL)
        cells.pack(fill="x", padx=16, pady=(0, 12))
        self.cell = {}
        for i, (key, cap) in enumerate((("er", "ENC R"), ("el", "ENC L"), ("vr", "VEL R  cps"), ("vl", "VEL L  cps"),
                                        ("md", "METAL"), ("g", "GRIPPER"))):
            c = tk.Frame(cells, bg=PANEL)
            c.grid(row=0, column=i, sticky="w", padx=(0, 26))
            tk.Label(c, text=cap, bg=PANEL, fg=DIM, font=("TkDefaultFont", 8)).pack(anchor="w")
            lbl = tk.Label(c, text="—", bg=PANEL, fg="#a3adbb", font=(self.mono, 22))
            lbl.pack(anchor="w")
            self.cell[key] = lbl
        c = tk.Frame(cells, bg=PANEL)
        c.grid(row=0, column=6, sticky="w")
        tk.Label(c, text="PROX  D23 · D25 · D27 · D29 · D31", bg=PANEL, fg=DIM, font=("TkDefaultFont", 8)).pack(anchor="w")
        self.prox = tk.Canvas(c, width=150, height=34, bg=PANEL, highlightthickness=0)
        self.prox.pack(anchor="w", pady=(4, 0))
        self.prox_dots = [self.prox.create_oval(4 + i * 29, 5, 28 + i * 29, 29, fill=BTN, outline="") for i in range(5)]

        # ---- Controls
        ctl = ttk.Frame(p)
        ctl.pack(fill="both", expand=True, pady=(10, 0))
        for i in range(3):
            ctl.columnconfigure(i, weight=1, uniform="a")

        # Session
        f = ttk.LabelFrame(ctl, text="SESSION", padding=10)
        f.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        r = ttk.Frame(f)
        r.pack(fill="x")
        ttk.Label(r, text="Period (ms)").pack(side="left")
        self.autom_ms = tk.StringVar(value="10")
        ttk.Entry(r, textvariable=self.autom_ms, width=7).pack(side="left", padx=6)
        ttk.Label(f, text="= PID sampling time and status-message period",
                  style="Muted.TLabel").pack(anchor="w", pady=(4, 8))
        r = ttk.Frame(f)
        r.pack(fill="x")
        ttk.Button(r, text="Start auto", style="Accent.TButton", command=self._start_auto).pack(side="left")
        ttk.Button(r, text="Stop auto", command=self._stop_auto).pack(side="left", padx=6)
        r = ttk.Frame(f)
        r.pack(fill="x", pady=(14, 0))
        self.keep_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(r, text="Resend frame every", variable=self.keep_var).pack(side="left")
        self.keep_ms = tk.StringVar(value="200")
        ttk.Entry(r, textvariable=self.keep_ms, width=5).pack(side="left", padx=4)
        ttk.Label(r, text="ms").pack(side="left")
        self.frame_lbl = tk.Label(f, text="", bg=BG, fg=MUTED, font=(self.mono, 9), anchor="w")
        self.frame_lbl.pack(fill="x", pady=(14, 0))

        # Drive
        f = ttk.LabelFrame(ctl, text="DRIVE  (counts/s, + = forward)", padding=10)
        f.grid(row=0, column=1, sticky="nsew", padx=6)
        top = ttk.Frame(f)
        top.pack(fill="x")
        ttk.Label(top, text="Speed").pack(side="left")
        self.auto_speed = tk.StringVar(value="1000")
        ttk.Entry(top, textvariable=self.auto_speed, width=7).pack(side="left", padx=6)
        self.hold_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(top, text="Hold to drive", variable=self.hold_var).pack(side="left", padx=8)
        pad = ttk.Frame(f)
        pad.pack(pady=6)
        for (r_, c_, txt, sr, sl) in ((0, 1, "▲", 1, 1), (1, 0, "◀", 1, -1), (1, 1, "■", 0, 0),
                                      (1, 2, "▶", -1, 1), (2, 1, "▼", -1, -1)):
            b = ttk.Button(pad, text=txt, width=4, style="Pad.TButton")
            b.grid(row=r_, column=c_, padx=2, pady=2)
            b.bind("<ButtonPress-1>", lambda e, a=sr, l=sl: self._pad_press(a, l))
            b.bind("<ButtonRelease-1>", lambda e: self._pad_release())
        self.vr = tk.IntVar(value=0)
        self.vl = tk.IntVar(value=0)
        self.vr_lbl = self._slider(f, "R", self.vr, -SLIDER_MAX, SLIDER_MAX)
        self.vl_lbl = self._slider(f, "L", self.vl, -SLIDER_MAX, SLIDER_MAX)

        # Tool
        f = ttk.LabelFrame(ctl, text="SERVO & GRIPPER", padding=10)
        f.grid(row=0, column=2, sticky="nsew", padx=(6, 0))
        self.servo = tk.IntVar(value=90)
        self.servo_lbl = self._slider(f, "Servo °", self.servo, 0, 180, unit="°")
        r = ttk.Frame(f)
        r.pack(fill="x", pady=(2, 10))
        for a in (0, 45, 90, 135, 180):
            ttk.Button(r, text=str(a), width=4, command=lambda a=a: self._set_servo(a)).pack(side="left", padx=2)
        ttk.Button(f, text="Trigger gripper cycle  (G ↑)", style="Accent.TButton",
                   command=self._trigger_grip).pack(fill="x")
        ttk.Label(f, style="Muted.TLabel", wraplength=300, justify="left",
                  text="G is sent as a rising edge: G=1, then G=0 150 ms later.").pack(anchor="w", pady=(6, 0))

    def _slider(self, parent, name, var, lo, hi, unit=""):
        r = ttk.Frame(parent)
        r.pack(fill="x", pady=2)
        ttk.Label(r, text=name, width=6).pack(side="left")
        lbl = ttk.Label(r, text=f"{var.get()}{unit}", width=7, anchor="e")
        lbl.pack(side="right")

        def cb(v):
            var.set(int(round(float(v))))
            lbl.configure(text=f"{var.get()}{unit}")
            if not self._suppress:
                self._schedule_frame()
        s = ttk.Scale(r, from_=lo, to=hi, variable=var, command=cb)
        s.pack(side="left", fill="x", expand=True, padx=6)
        s._lbl, s._unit, s._var = lbl, unit, var
        if not hasattr(self, "_sliders"):
            self._sliders = []
        self._sliders.append(s)
        return lbl

    def _refresh_slider_labels(self):
        for s in self._sliders:
            s._lbl.configure(text=f"{s._var.get()}{s._unit}")

    # --- auto session
    def _start_auto(self):
        try:
            ms = int(self.autom_ms.get())
        except ValueError:
            self.log("err", "Period must be an integer (1..1000 ms)")
            return
        self.vr.set(0)
        self.vl.set(0)
        self._refresh_slider_labels()
        if self.send(f"AUTOM {ms}"):
            self.auto_on = True
            self._update_auto_chip()
            self.after(150, self._send_frame)         # push servo / zero targets once
            self._keepalive()

    def _stop_auto(self):
        self.auto_on = False
        self.vr.set(0)
        self.vl.set(0)
        self._refresh_slider_labels()
        self.send("AUTOM OFF")
        self._update_auto_chip()

    def _pad_press(self, sr, sl):
        try:
            v = abs(int(self.auto_speed.get()))
        except ValueError:
            self.log("err", "Drive speed must be an integer")
            return
        self._set_targets(sr * v, sl * v)

    def _pad_release(self):
        if self.hold_var.get():
            self._set_targets(0, 0)

    def _set_targets(self, r, l):
        self._suppress = True
        self.vr.set(r)
        self.vl.set(l)
        self._refresh_slider_labels()
        self._suppress = False
        self._send_frame()

    def _set_servo(self, a):
        self._suppress = True
        self.servo.set(a)
        self._refresh_slider_labels()
        self._suppress = False
        self._send_frame()

    def _trigger_grip(self):
        if self._send_frame(g=1):
            self.after(150, self._send_frame)

    # --- frame sending
    def _frame_str(self, g=0):
        def sv(ch, v):
            return f"{ch}{'N' if v < 0 else 'P'}{abs(int(v))}"
        return f"{sv('R', self.vr.get())}{sv('L', self.vl.get())}GP{g}S({int(self.servo.get())})"

    def _schedule_frame(self):                      # coalesce slider drags
        if not self._frame_pending:
            self._frame_pending = True
            self.after(40, self._flush_frame)

    def _flush_frame(self):
        self._frame_pending = False
        self._send_frame()

    def _send_frame(self, g=0):
        if not self.auto_on:
            self.log("info", "Start auto first (AUTOM) - frames are only accepted in auto mode")
            return False
        frame = self._frame_str(g)
        self.frame_lbl.configure(text=f"last frame  {frame}")
        return self.send(frame)

    def _keepalive(self):
        if self.auto_on and self.keep_var.get() and self.link.connected:
            self.send(self._frame_str(0), quiet=True)
        if self.auto_on:
            try:
                ms = max(50, int(self.keep_ms.get()))
            except ValueError:
                ms = 200
            self.after(ms, self._keepalive)

    # --- status display
    def _update_auto_chip(self):
        if not self.auto_on:
            self.auto_chip.configure(text="AUTO OFF", fg=DIM)
            return
        age = time.monotonic() - self.last_frame_t
        if age > 1.0:
            self.auto_chip.configure(text="AUTO ON · waiting for data…", fg=WARN)
        else:
            t = [x for x in self.frame_times if time.monotonic() - x < 2.0]
            hz = (len(t) - 1) / (t[-1] - t[0]) if len(t) > 2 and t[-1] > t[0] else 0
            self.auto_chip.configure(text=f"AUTO ON · {hz:.0f} Hz", fg=GOOD)

    def _update_status(self, m, raw):
        now = time.monotonic()
        self.last_frame_t = now
        self.frame_times.append(now)
        if not self.auto_on:                               # frames only exist in auto mode
            self.auto_on = True
        sg = lambda s, n: f"{'-' if s == 'N' else '+'}{int(n)}"
        self.raw_lbl.configure(text=raw, fg="#8a94a3")
        self.cell["er"].configure(text=sg(m[1], m[2]))
        self.cell["el"].configure(text=sg(m[3], m[4]))
        self.cell["vr"].configure(text=sg(m[5], m[6]))
        self.cell["vl"].configure(text=sg(m[7], m[8]))
        g = m[10]
        self.cell["g"].configure(text=f"{g} · {GRIP_NAMES[g]}", fg=BAD if g == "F" else "#a3adbb")
        self._grip_set(g, "frame")
        if m[9] == "1":
            self.cell["md"].configure(text="EDGE", fg=WARN)
            if self._md_job:
                self.after_cancel(self._md_job)
            self._md_job = self.after(700, lambda: self.cell["md"].configure(text="—", fg="#a3adbb"))
        elif self._md_job is None:
            self.cell["md"].configure(text="—", fg="#a3adbb")
        for dot, bit in zip(self.prox_dots, m[11]):
            self.prox.itemconfigure(dot, fill=GOOD if bit == "1" else BTN)
        self._update_auto_chip()

    # ================================================================= GRIPPER
    def _build_gripper(self, p):
        # error banner
        self.g_banner = tk.Label(p, text="", bg=PANEL, fg=GOOD, anchor="w", justify="left",
                                 font=("TkDefaultFont", 11, "bold"), padx=14, pady=8)
        self.g_banner.pack(fill="x")
        self.g_banner.bind("<Configure>", lambda e: self.g_banner.configure(wraplength=max(200, e.width - 30)))

        # state card
        card = tk.Frame(p, bg=PANEL)
        card.pack(fill="x", pady=(8, 0))
        left = tk.Frame(card, bg=PANEL)
        left.pack(side="left", padx=16, pady=10)
        tk.Label(left, text="GRIPPER CYCLE STATE", bg=PANEL, fg=DIM, font=("TkDefaultFont", 8)).grid(row=0, column=0, columnspan=2, sticky="w")
        self.g_big = tk.Label(left, text="—", bg=PANEL, fg="#a3adbb", font=(self.mono, 44, "bold"), width=2)
        self.g_big.grid(row=1, column=0, rowspan=3)
        self.g_name = tk.Label(left, text="UNKNOWN", bg=PANEL, fg="#8a94a3", font=(self.mono, 22), anchor="w")
        self.g_name.grid(row=1, column=1, sticky="w", padx=(10, 0))
        self.g_desc = tk.Label(left, text="", bg=PANEL, fg=MUTED, anchor="w", justify="left", wraplength=480)
        self.g_desc.grid(row=2, column=1, sticky="w", padx=(10, 0))
        self.g_phase_lbl = tk.Label(left, text="", bg=PANEL, fg=WARN, anchor="w", font=(self.mono, 10))
        self.g_phase_lbl.grid(row=3, column=1, sticky="w", padx=(10, 0))

        right = tk.Frame(card, bg=PANEL)
        right.pack(side="right", padx=16, pady=10)
        self.g_timer = tk.Label(right, text="in state  —", bg=PANEL, fg="#a3adbb", font=(self.mono, 14), anchor="e")
        self.g_timer.pack(anchor="e")
        self.g_bar = ttk.Progressbar(right, length=230, maximum=100)
        self.g_bar.pack(anchor="e", pady=4)
        self.g_armed = tk.Label(right, text=" ", bg=PANEL, fg=WARN, font=(self.mono, 10), anchor="e")
        self.g_armed.pack(anchor="e")
        self.g_counts_lbl = tk.Label(right, text="", bg=PANEL, fg=MUTED, font=(self.mono, 9), anchor="e")
        self.g_counts_lbl.pack(anchor="e", pady=(4, 0))
        self.g_src_lbl = tk.Label(right, text="", bg=PANEL, fg=DIM, font=(self.mono, 8), anchor="e")
        self.g_src_lbl.pack(anchor="e")

        # cycle diagram
        self.flow = tk.Canvas(p, height=100, bg=BG, highlightthickness=0)
        self.flow.pack(fill="x", pady=(8, 0))
        self.flow.bind("<Configure>", lambda e: self._draw_flow())

        # buttons
        bt = ttk.Frame(p)
        bt.pack(fill="x", pady=(6, 0))
        ttk.Button(bt, text="GRIP  (arm cycle)", style="Accent.TButton", command=self._grip_arm_cmd).pack(side="left")
        ttk.Button(bt, text="Query state (STAT)", command=lambda: self.send("STAT")).pack(side="left", padx=6)
        ttk.Button(bt, text="MGNT toggle", command=lambda: self.send("MGNT")).pack(side="left")
        ttk.Button(bt, text="Reset board (DTR)", command=self._reset_board).pack(side="left", padx=6)
        self.g_dist_lbl = ttk.Label(bt, text="distance (last STAT)  —", style="Muted.TLabel")
        self.g_dist_lbl.pack(side="right")

        # history + errors
        low = ttk.Frame(p)
        low.pack(fill="both", expand=True, pady=(8, 0))
        low.columnconfigure(0, weight=3, uniform="g")
        low.columnconfigure(1, weight=2, uniform="g")
        low.rowconfigure(0, weight=1)
        f1 = ttk.LabelFrame(low, text="CYCLE EVENTS", padding=6)
        f1.grid(row=0, column=0, sticky="nsew", padx=(0, 6))
        self.ev_tv = self._tree(f1, (("t", "Time", 95), ("s", "State", 50), ("m", "Event", 360)))
        f2 = ttk.LabelFrame(low, text="ERRORS & WARNINGS", padding=6)
        f2.grid(row=0, column=1, sticky="nsew", padx=(6, 0))
        ttk.Button(f2, text="Clear", command=self._clear_errors).pack(side="bottom", anchor="e", pady=(4, 0))
        self.err_tv = self._tree(f2, (("t", "Time", 95), ("src", "Source", 60), ("m", "Message", 260)))

        self._grip_refresh()

    def _tree(self, parent, cols):
        fr = ttk.Frame(parent)
        fr.pack(fill="both", expand=True)
        tv = ttk.Treeview(fr, columns=[c[0] for c in cols], show="headings", height=5, selectmode="browse")
        for cid, head, w in cols:
            tv.heading(cid, text=head, anchor="w")
            tv.column(cid, width=w, minwidth=40, stretch=(cid == "m"), anchor="w")
        sb = ttk.Scrollbar(fr, command=tv.yview)
        tv.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        tv.pack(side="left", fill="both", expand=True)
        for tag, col in (("fault", BAD), ("err", BAD), ("warn", WARN), ("state", TX_COL), ("info", MUTED)):
            tv.tag_configure(tag, foreground=col)
        return tv

    @staticmethod
    def _stamp():
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    @staticmethod
    def _trim(tv, n=300):
        for k in tv.get_children()[n:]:
            tv.delete(k)

    # --- diagram
    def _draw_flow(self):
        c = self.flow
        c.delete("all")
        w = max(c.winfo_width(), 640)
        steps = (("D", "DOWN", "ready"), ("A", "MOVING UP", "lift"), ("U", "UP", "release"), ("B", "MOVING DOWN", "lower"))
        fw, y0, y1 = 120, 8, 62
        usable = w - fw - 50
        bw = min(160, usable / 4 - 30)
        gap = (usable - 4 * bw) / 3
        centers = []
        for i, (k, name, sub) in enumerate(steps):
            x0 = 10 + i * (bw + gap)
            x1 = x0 + bw
            on = self.g_state == k
            c.create_rectangle(x0, y0, x1, y1, fill="#24344d" if on else PANEL, outline=ACCENT if on else BTN, width=2)
            c.create_text((x0 + x1) / 2, y0 + 20, text=f"{k} · {name}", fill=FG if on else MUTED, font=(self.mono, 10, "bold"))
            c.create_text((x0 + x1) / 2, y0 + 40, text=sub, fill=MUTED if on else DIM, font=(self.mono, 9))
            centers.append((x0 + x1) / 2)
            if i < 3:
                c.create_line(x1 + 4, (y0 + y1) / 2, x1 + gap - 4, (y0 + y1) / 2, fill=DIM, arrow=tk.LAST, width=2)
        yy = y1 + 16
        c.create_line(centers[3], y1, centers[3], yy, centers[0], yy, centers[0], y1, fill=DIM, dash=(3, 3), arrow=tk.LAST)
        c.create_text((centers[0] + centers[3]) / 2, yy + 11, text="cycle complete · back to D", fill=DIM, font=(self.mono, 8))
        on = self.g_state == "F"
        x0 = w - fw - 10
        c.create_rectangle(x0, y0, x0 + fw, y1, fill="#3a1d1f" if on else PANEL, outline=BAD if on else BTN, width=2)
        c.create_text(x0 + fw / 2, y0 + 20, text="F · FAULT", fill=BAD if on else MUTED, font=(self.mono, 10, "bold"))
        c.create_text(x0 + fw / 2, y0 + 40, text="reset board", fill=BAD if on else DIM, font=(self.mono, 9))

    # --- state handling
    def _grip_set(self, new, src, phase=None, log=True):
        """Apply a (possibly new) gripper state from a status frame, STAT or a board message."""
        old = self.g_state
        if new == old:
            self.g_src = src
            return
        if new == "D" and old in ("A", "U", "B"):             # cycle ended: classify using the OLD phase
            if self.g_phase.startswith("Recovery"):
                self.g_counts["recov"] += 1
            elif not self.g_phase.startswith("Self-test"):
                self.g_counts["done"] += 1
        if new == "D":
            self.g_phase = ""
        elif phase is not None:
            self.g_phase = phase
        if new == "A":
            self.g_armed_until = 0.0
        if new == "F":
            self.g_counts["fault"] += 1
            reason = self.g_fault_reason or "lift fault (reason is only printed in manual mode)"
            self._add_error("Gripper", f"FAULT: {reason}", "fault")
        else:
            self.g_fault_reason = ""
        self.g_state, self.g_since, self.g_src = new, time.monotonic(), src
        if log:
            self._grip_event(f"{old or '—'} → {new}   ({src})", "fault" if new == "F" else "state")
        self._grip_refresh()

    def _grip_message(self, line):
        """Interpret a 'GRIP...' text line printed by the board (manual mode)."""
        body = line[6:] if line.startswith("GRIP: ") else line
        low = line.lower()
        for key, state, phase, level in MSG_RULES:
            if key in low:
                break
        else:
            self._grip_event(body, "info")
            return
        if level == "fault":
            self.g_fault_reason = body
        if state:
            self._grip_set(state, "message", phase, log=False)
        elif phase is not None:
            self.g_phase = phase
        if key == "armed" and self.g_state in (None, "D"):
            self.g_armed_until = time.monotonic() + GRIP_ARM_S
        if key in ("request expired", "ignored"):
            self.g_armed_until = 0.0
        if level == "warn":
            self._add_error("Gripper", body, "warn")
        self._grip_event(body, level or "info")
        self._grip_refresh()

    def _g_arm(self):
        if self.g_state in (None, "D"):
            self.g_armed_until = time.monotonic() + GRIP_ARM_S

    def _grip_arm_cmd(self):
        if self.auto_on:
            self._trigger_grip()
        else:
            self.send("GRIP")

    def _grip_reset_view(self):
        self.g_state, self.g_phase, self.g_src = None, "", ""
        self.g_armed_until, self.g_fault_reason = 0.0, ""
        self.g_since = time.monotonic()
        self.g_dist_lbl.configure(text="distance (last STAT)  —")
        self._grip_refresh()

    def _reset_board(self):
        if not self.link.connected:
            self.log("err", "Not connected - cannot reset the board")
            return
        try:
            self.link.reset_board()
        except Exception as exc:
            self.log("err", f"Board reset failed: {exc}")
            return
        self.auto_on = False
        self._set_targets_silent()
        self._update_auto_chip()
        self._grip_reset_view()
        self.log("info", "Board reset via DTR - the gripper self-test restarts (boot messages follow)")
        self._grip_event("board reset (DTR)", "warn")

    def _set_targets_silent(self):
        self._suppress = True
        self.vr.set(0)
        self.vl.set(0)
        self._refresh_slider_labels()
        self._suppress = False

    # --- lists
    def _grip_event(self, text, tag="info"):
        self.ev_tv.insert("", 0, values=(self._stamp(), self.g_state or "—", text), tags=(tag,))
        self._trim(self.ev_tv)

    def _add_error(self, source, msg, level="err"):
        self.err_tv.insert("", 0, values=(self._stamp(), source, msg), tags=(level,))
        self._trim(self.err_tv)
        self.err_count += 1
        self.err_last = msg
        self._refresh_banner()

    def _clear_errors(self):
        for k in self.err_tv.get_children():
            self.err_tv.delete(k)
        self.err_count, self.err_last = 0, ""
        self._refresh_banner()

    # --- display refresh
    def _refresh_banner(self):
        fault = self.g_state == "F"
        if fault:
            r = self.g_fault_reason or "lift fault reported in the status frame"
            self.g_banner.configure(text=f"⛔  FAULT · {r} · GRIP stays disabled until the Arduino is reset",
                                    bg="#3a1d1f", fg=BAD)
        elif self.err_count:
            self.g_banner.configure(text=f"⚠  {self.err_count} error/warning(s) logged · last: {self.err_last}",
                                    bg="#3a321a", fg=WARN)
        else:
            self.g_banner.configure(text="✓  No errors", bg=PANEL, fg=GOOD)
        badge = " ⛔" if fault else (f"  ⚠ {self.err_count}" if self.err_count else "")
        self.nb.tab(self.tab_grip, text="Gripper" + badge)

    def _grip_refresh(self):
        s = self.g_state
        self.g_big.configure(text=s or "—", fg=BAD if s == "F" else (ACCENT if s in ("A", "U", "B") else "#a3adbb"))
        self.g_name.configure(text=GRIP_NAMES.get(s, "UNKNOWN"), fg=BAD if s == "F" else "#8a94a3")
        self.g_desc.configure(text=GRIP_DESC.get(s, "No state yet · connect (boot messages), start auto, or press Query state"))
        self.g_phase_lbl.configure(text=self.g_phase or " ")
        c = self.g_counts
        self.g_counts_lbl.configure(text=f"completed {c['done']} · recovered {c['recov']} · faults {c['fault']}")
        self.g_src_lbl.configure(text=f"source: {self.g_src}" if self.g_src else "")
        self._draw_flow()
        self._refresh_banner()

    def _grip_tick(self):
        now = time.monotonic()
        s = self.g_state
        if s:
            el = now - self.g_since
            lim = GRIP_LIMITS.get(s)
            self.g_timer.configure(text=f"in state {el:5.1f} s" + (f" / {lim} s" if lim else ""),
                                   fg=BAD if (lim and el > lim) else "#a3adbb")
            self.g_bar["value"] = min(100, el / lim * 100) if lim else 0
        else:
            self.g_timer.configure(text="in state  —", fg="#a3adbb")
            self.g_bar["value"] = 0
        left = self.g_armed_until - now
        if self.g_armed_until and left > 0:
            self.g_armed.configure(text=f"ARMED · waiting for metal edge + distance change · {left:4.1f} s")
        else:
            self.g_armed.configure(text=" ")
            if self.g_armed_until:                      # window ran out
                self.g_armed_until = 0.0
                if self.auto_on:                        # in manual mode the board reports it itself
                    self._grip_event(f"GRIP request window ({GRIP_ARM_S} s) ended without a cycle start", "warn")

    # ==================================================================== LOG
    def _build_log(self, p):
        bar = ttk.Frame(p)
        bar.pack(fill="x", pady=(8, 4))
        ttk.Label(bar, text="LOG", font=("TkDefaultFont", 9, "bold"), style="Muted.TLabel").pack(side="left", padx=(2, 12))
        self.show_tx, self.show_rx = tk.BooleanVar(value=True), tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="TX", variable=self.show_tx, command=self._apply_filters).pack(side="left")
        ttk.Checkbutton(bar, text="RX", variable=self.show_rx, command=self._apply_filters).pack(side="left", padx=(4, 12))
        ttk.Label(bar, text="Auto status frames").pack(side="left")
        self.status_mode = tk.StringVar(value="5 / s")
        ttk.Combobox(bar, textvariable=self.status_mode, values=("All", "5 / s", "1 / s", "Off"),
                     width=6, state="readonly").pack(side="left", padx=6)
        self.autoscroll = tk.BooleanVar(value=True)
        ttk.Checkbutton(bar, text="Autoscroll", variable=self.autoscroll).pack(side="left", padx=8)
        ttk.Button(bar, text="Save…", command=self._save_log).pack(side="right")
        ttk.Button(bar, text="Clear", command=lambda: self._clear_log()).pack(side="right", padx=6)

        body = ttk.Frame(p)
        body.pack(fill="both", expand=True)
        self.txt = tk.Text(body, bg=FIELD, fg=FG, insertbackground=FG, relief="flat", wrap="none", height=8,
                           font=(self.mono, 10), state="disabled", padx=8, pady=6, highlightthickness=0)
        sb = ttk.Scrollbar(body, command=self.txt.yview)
        self.txt.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.txt.pack(side="left", fill="both", expand=True)
        self.txt.tag_configure("tx", foreground=TX_COL)
        self.txt.tag_configure("rx", foreground=RX_COL)
        self.txt.tag_configure("err", foreground=BAD)
        self.txt.tag_configure("info", foreground=WARN)
        self.txt.tag_configure("ts", foreground=DIM)         # created last -> wins for the timestamp

        cmd = ttk.Frame(p)
        cmd.pack(fill="x", pady=(6, 0))
        ttk.Label(cmd, text="Command", style="Muted.TLabel").pack(side="left", padx=(2, 8))
        self.cmd_var = tk.StringVar()
        e = ttk.Entry(cmd, textvariable=self.cmd_var, font=(self.mono, 10))
        e.pack(side="left", fill="x", expand=True)
        e.bind("<Return>", self._send_cmd_line)
        e.bind("<Up>", lambda _e: self._hist(-1))
        e.bind("<Down>", lambda _e: self._hist(1))
        ttk.Button(cmd, text="Send", style="Accent.TButton", command=self._send_cmd_line).pack(side="left", padx=(6, 0))

    def log(self, kind, text, record=True):
        if kind == "err" and record:
            self._add_error("GUI", text, "err")
        ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        arrow = {"tx": "TX ▶", "rx": "RX ◀", "err": "RX ◀", "info": " •  "}[kind]
        self.txt.configure(state="normal")
        self.txt.insert("end", f"{ts}  ", (kind, "ts"))
        self.txt.insert("end", f"{arrow}  {text}\n", (kind,))
        lines = int(self.txt.index("end-1c").split(".")[0])
        if lines > LOG_MAX_LINES:
            self.txt.delete("1.0", f"{lines - LOG_MAX_LINES + 1}.0")
        self.txt.configure(state="disabled")
        if self.autoscroll.get():
            self.txt.see("end")

    def _apply_filters(self):
        self.txt.tag_configure("tx", elide=not self.show_tx.get())
        self.txt.tag_configure("rx", elide=not self.show_rx.get())
        self.txt.tag_configure("err", elide=not self.show_rx.get())

    def _clear_log(self):
        self.txt.configure(state="normal")
        self.txt.delete("1.0", "end")
        self.txt.configure(state="disabled")

    def _save_log(self):
        path = filedialog.asksaveasfilename(defaultextension=".txt", initialfile=f"biznoh_{datetime.now():%Y%m%d_%H%M%S}.txt")
        if path:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(self.txt.get("1.0", "end"))
            self.log("info", f"Log saved to {path}")

    def _send_cmd_line(self, _=None):
        c = self.cmd_var.get().strip()
        if not c:
            return
        if self.send(c):
            self.history.append(c)
            self.hist_idx = len(self.history)
            self.cmd_var.set("")

    def _hist(self, d):
        if not self.history:
            return
        self.hist_idx = max(0, min(len(self.history), self.hist_idx + d))
        self.cmd_var.set(self.history[self.hist_idx] if self.hist_idx < len(self.history) else "")

    # ============================================================ I/O plumbing
    def send(self, cmd, quiet=False):
        if not self.link.connected:
            self.log("err", f"Not connected - not sent: {cmd}")
            return False
        try:
            self.link.write(cmd)
        except Exception as exc:
            self.log("err", f"Write failed: {exc}")
            self._disconnect("Connection lost")
            return False
        if not quiet:
            self.log("tx", cmd)
        u = cmd.strip().upper()
        if u == "GRIP" or "GP1S(" in u:                 # GRIP command or auto frame with G=1
            self._g_arm()
        return True

    def _poll(self):
        try:
            for _ in range(300):
                kind, payload = self.rxq.get_nowait()
                if kind == "closed":
                    self._disconnect(f"Connection lost: {payload}")
                else:
                    self._handle_rx(payload)
        except queue.Empty:
            pass
        self.after(20, self._poll)

    def _handle_rx(self, line):
        m = STATUS_RE.match(line)
        if m:
            self._update_status(m, line)
            mode = self.status_mode.get()
            gap = {"All": 0.0, "5 / s": 0.2, "1 / s": 1.0}.get(mode)
            now = time.monotonic()
            if gap is not None and now - self.last_status_log >= gap:
                self.last_status_log = now
                self.log("rx", line)
            return
        if line.startswith("AUTOM = "):
            self.auto_on = not line.endswith("OFF")
            self._update_auto_chip()
        ms = STAT_GRIP_RE.match(line)
        if ms:
            self._grip_set(ms.group(1), "STAT")
            self.g_dist_lbl.configure(text=f"distance (last STAT)  {ms.group(2)}.{ms.group(3)} cm")
        elif line.startswith("GRIP"):
            self._grip_message(line)
        if line.startswith("ERR"):
            self.log("err", line, record=False)
            self._add_error("Board", line, "err")
        else:
            self.log("rx", line)

    def _tick(self):
        self._grip_tick()
        if self.auto_on:
            self._update_auto_chip()
            if time.monotonic() - self.last_frame_t > 1.0:
                self.raw_lbl.configure(fg=DIM)
        self.after(250, self._tick)

    # ----------------------------------------------------------------- misc
    def _estop(self):
        if self.auto_on:
            self._set_targets(0, 0)
        else:
            self.send("STOP")

    def _on_key(self, e):
        if not self.kb_var.get() or self.nb.index("current") != 0:
            return
        if isinstance(e.widget, (tk.Entry, ttk.Entry, ttk.Combobox, tk.Text)):
            return
        k = e.keysym.lower()
        if k in ("w", "a", "s", "d"):
            now = time.monotonic()
            if now - self._last_key.get(k, 0) > 0.3:       # tame key auto-repeat
                self._last_key[k] = now
                self.send(k.upper())
        elif k == "space" and not isinstance(e.widget, ttk.Button):
            self.send("STOP")

    def _on_close(self):
        self._disconnect()
        self.destroy()


if __name__ == "__main__":
    App().mainloop()
