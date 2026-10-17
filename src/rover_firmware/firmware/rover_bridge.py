#!/usr/bin/env python3
"""
rover_bridge.py  -  middleman between the ROS 2 rover_interface (C++ hardware plugin)
and the Arduino Mega running safsaf222.ino, with the Arduino kept in MANUAL mode.

                 ROS frames                          manual "C<cmd>" lines
 rover_interface ----------> [virtual port] bridge ---------------------------> Arduino
 (rover_firmware)  rp..,lp..,gp..,sp..,\\n   (pty)   (this script)   /dev/ttyACM0   (safsaf222.ino)

Why a translator is needed
--------------------------
The ROS plugin writes   rp12.34,lp-05.00,gp00.00,sp00.00,\\n   at the control rate.
If those bytes reached the Arduino directly, the firmware would switch itself to AUTO
mode on the first token (rosToken() -> startAuto()). To stay in manual mode this script
never forwards ROS frames; it parses them and sends the firmware's own typed commands
(every command starts with 'C' and ends with a newline).

Drive modes (--drive)
---------------------
pid  (default)  Closed-loop manual mode. Uses  MODE VEL / LOOP ON / TGT A|B <counts/s>.
                Right/left rad/s from ROS are converted to counts/s (2400 PPR / 2*pi), so
                differential steering (arcs) works. The PID gains are set to the same
                values the firmware uses in auto mode (AUTO_KP/KI/KD/TOL/PWMMIN/PWMMAX).
keys            Open-loop manual mode. Uses  W / A / S / D  (+ SPEED xx). Only straight
                or pivot motion, no arcs. Fallback if the encoders/PID are not usable.

Other mappings
--------------
g token  : rising edge (0 -> non-zero)  ->  GRIP
s token  : IGNORED. The firmware has no manual command for the camera servo; the servo
           can only be driven through an automatic-mode ROS frame.
Watchdog : the firmware watchdog only runs in auto mode, so the bridge has its own: if no
           ROS frame arrives for --watchdog seconds the wheels are stopped.
"""

import argparse
import errno
import glob
import logging
import math
import os
import pty
import re
import select
import signal
import sys
import threading
import time
import tty

import serial  # pip install pyserial   /   sudo apt install python3-serial

# Must match safsaf222.ino: ENC_COUNTS_PER_REV = 2400
COUNTS_PER_RAD = 2400.0 / (2.0 * math.pi)
MAX_TARGET_CPS = 100000.0  # same clamp as AUTO_MAX_TARGET_CPS

# Same values as the AUTO_* constants of the firmware
PID_PERIOD_MS = 10
AUTO_KP, AUTO_KI, AUTO_KD = 0.02, 0.10, 0.0
AUTO_TOL_CPS = 50.0
AUTO_PWM_MIN, AUTO_PWM_MAX = 20, 100

TOKEN_RE = re.compile(r"^([rlgs])([pn]?)(\d*\.?\d+)$")  # e.g. rp12.34  ln5.00  gp00.00
ENC_RE = re.compile(r"ENC([AB])\s*=\s*(-?\d+)")

log = logging.getLogger("rover_bridge")


# ----------------------------------------------------------------------------------------
def find_arduino():
    for pattern in ("/dev/serial/by-id/*", "/dev/ttyACM*", "/dev/ttyUSB*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    return None


def parse_frame(line):
    """'rp12.34,ln5.00,gp00.00,sp00.00,' -> ({'r': 12.34, 'l': -5.0, ...}, n_bad_tokens)"""
    values, bad = {}, 0
    for tok in line.strip().split(","):
        tok = tok.strip()
        if not tok:
            continue
        m = TOKEN_RE.match(tok)
        if not m:
            bad += 1
            continue
        val = float(m.group(3))
        values[m.group(1)] = -val if m.group(2) == "n" else val
    return values, bad


# ----------------------------------------------------------------------------------------
class Bridge:
    def __init__(self, args):
        self.args = args
        self.running = True
        self.tx_lock = threading.Lock()
        self.last_tx = 0.0

        # ROS-side state
        self.rads = {"r": 0.0, "l": 0.0}
        self.last_frame = 0.0
        self.wd_tripped = False
        self.grip_prev = False
        self.servo_warned = False
        self.ros_buf = b""

        # pid drive state (A = right, B = left)
        self.want = {"A": 0.0, "B": 0.0}
        self.sent = {"A": 0.0, "B": 0.0}
        self.sent_t = {"A": 0.0, "B": 0.0}

        # keys drive state
        self.key_cmd = None
        self.key_t = 0.0
        self.key_pct = None

        # encoder replies from the Arduino (for --feedback)
        self.enc = {}

        self.link_path = None
        self.master = self.slave = None
        self.ser = None

    # ---------------------------------------------------------------- serial to Arduino
    def open_arduino(self):
        port = self.args.arduino or find_arduino()
        if not port:
            sys.exit("No Arduino serial port found. Use --arduino /dev/ttyXXX")
        log.info("Opening Arduino on %s @ %d", port, self.args.baud)
        self.ser = serial.Serial(port, self.args.baud, timeout=0.1, write_timeout=1.0)
        # Opening the port toggles DTR and resets the Mega: wait for the bootloader.
        log.info("Waiting %.1f s for the Arduino to boot...", self.args.boot_wait)
        time.sleep(self.args.boot_wait)
        self.ser.reset_input_buffer()

    def send(self, cmd):
        """Send one manual command, e.g. send('TGT A 1200.0') -> 'CTGT A 1200.0\\n'"""
        with self.tx_lock:
            gap = 0.01 - (time.monotonic() - self.last_tx)
            if gap > 0:
                time.sleep(gap)
            try:
                self.ser.write(("C" + cmd + "\n").encode("ascii"))
            except (serial.SerialException, OSError) as exc:
                log.error("Arduino write failed: %s", exc)
                self.running = False
                return
            self.last_tx = time.monotonic()
        log.debug("-> Arduino: C%s", cmd)

    def arduino_reader(self):
        while self.running:
            try:
                raw = self.ser.readline()
            except (serial.SerialException, OSError) as exc:
                log.error("Arduino read failed: %s", exc)
                self.running = False
                return
            if not raw:
                continue
            line = raw.decode("ascii", "replace").strip()
            if not line:
                continue
            m = ENC_RE.search(line)
            if m:
                self.enc[m.group(1)] = (int(m.group(2)), time.monotonic())
            elif line.startswith("ERR"):
                log.warning("Arduino: %s", line)
            else:
                log.debug("Arduino: %s", line)

    def init_arduino(self):
        self.ser.write(b"\n")  # clear any half-received token in the firmware parser
        time.sleep(0.05)
        self.send("STOP")
        self.send("TEL 0")  # no telemetry lines
        if self.args.drive == "pid":
            self.send("PID %d" % PID_PERIOD_MS)
            self.send("MODE VEL")  # also halts the drive and zeroes the targets
            self.send("P %g" % AUTO_KP)
            self.send("I %g" % AUTO_KI)
            self.send("D %g" % AUTO_KD)
            self.send("TOL %g" % AUTO_TOL_CPS)
            self.send("PWMMAX %d" % AUTO_PWM_MAX)  # max first, then min (firmware checks min<=max)
            self.send("PWMMIN %d" % AUTO_PWM_MIN)
            self.send("LOOP ON")
            log.info("Manual closed-loop velocity mode ready (MODE VEL, LOOP ON)")
        else:
            log.info("Manual open-loop W/A/S/D mode ready")

    # ------------------------------------------------------------------ virtual port
    def open_ros_port(self):
        self.master, self.slave = pty.openpty()
        tty.setraw(self.slave)  # raw, no echo: ROS bytes must not be echoed back
        os.set_blocking(self.master, False)
        os.set_blocking(self.slave, False)
        slave_name = os.ttyname(self.slave)
        path = self.args.ros_port
        if os.path.islink(path) or os.path.exists(path):
            os.remove(path)
        os.symlink(slave_name, path)
        self.link_path = path
        # The bridge keeps its own slave fd open so the master never gets EIO while the
        # ROS plugin closes/re-opens the port (deactivate / activate).
        log.info("Virtual port for ROS: %s -> %s", path, slave_name)

    def to_ros(self, text):
        try:
            os.write(self.master, text.encode("ascii"))
        except (BlockingIOError, OSError):
            pass  # nobody is reading, drop it

    # ------------------------------------------------------------------ ROS -> Arduino
    def handle_frame(self, line):
        values, bad = parse_frame(line)
        if bad:
            log.debug("Ignored %d malformed token(s) in %r", bad, line)
        if not values:
            return
        log.debug("<- ROS: %s", values)

        self.last_frame = time.monotonic()
        self.wd_tripped = False

        for key in ("r", "l"):
            if key in values:
                self.rads[key] = values[key]
        if "r" in values or "l" in values:
            self.drive(self.rads["r"], self.rads["l"])

        if "g" in values:
            g_on = abs(values["g"]) > 0.001
            if g_on and not self.grip_prev:
                log.info("Gripper request -> GRIP")
                self.send("GRIP")
            self.grip_prev = g_on

        if "s" in values and not self.servo_warned:
            self.servo_warned = True
            log.info("Camera servo token ignored (no manual-mode servo command in the firmware)")

    def drive(self, r_rads, l_rads):
        if self.args.drive == "pid":
            clamp = lambda v: max(-MAX_TARGET_CPS, min(MAX_TARGET_CPS, v * COUNTS_PER_RAD))
            self.want["A"] = clamp(r_rads)  # A = right
            self.want["B"] = clamp(l_rads)  # B = left
            self.apply_pid()
        else:
            self.apply_keys(r_rads, l_rads)

    def apply_pid(self):
        """Send TGT only when needed. Every TGT resets the PID integrator in the firmware,
        so resending the same/similar value at 50-100 Hz would wreck the I term."""
        now = time.monotonic()
        deadband = self.args.deadband * COUNTS_PER_RAD
        for m in ("A", "B"):
            want, sent = self.want[m], self.sent[m]
            if want == sent:
                continue
            to_zero = want == 0.0
            flip = want * sent < 0.0
            changed = abs(want - sent) >= deadband
            due = (now - self.sent_t[m]) >= self.args.min_interval
            if to_zero or flip or (changed and due):
                self.send("TGT %s %.1f" % (m, want))
                self.sent[m] = want
                self.sent_t[m] = now

    def apply_keys(self, r_rads, l_rads):
        eps = self.args.deadband
        v = (r_rads + l_rads) / 2.0   # linear part
        w = (r_rads - l_rads) / 2.0   # angular part, > 0 = turn left
        now = time.monotonic()
        if abs(v) < eps and abs(w) < eps:
            if self.key_cmd is not None:
                self.send("STOP")
                self.key_cmd = None
            return
        cmd = ("W" if v > 0 else "S") if abs(v) >= abs(w) else ("A" if w > 0 else "D")
        pct = int(round(min(1.0, max(abs(r_rads), abs(l_rads)) / self.args.max_rads) * 100))
        pct = max(pct, AUTO_PWM_MIN)
        if self.key_pct is None or abs(pct - self.key_pct) >= 5:
            self.send("SPEED %d" % pct)
            self.key_pct = pct
        # each W/A/S/D runs 2 s in the firmware; refresh it before it expires
        if cmd != self.key_cmd or now - self.key_t >= 0.5:
            self.send(cmd)
            self.key_cmd, self.key_t = cmd, now

    def stop_wheels(self):
        self.rads = {"r": 0.0, "l": 0.0}
        if self.args.drive == "pid":
            self.want = {"A": 0.0, "B": 0.0}
            self.apply_pid()
        else:
            self.send("STOP")
            self.key_cmd = None

    def moving(self):
        if self.args.drive == "pid":
            return any(v != 0.0 for v in self.sent.values())
        return self.key_cmd is not None

    # ------------------------------------------------------------------- feedback
    def feedback_loop(self):
        """Optional: poll the encoders (PPR A / PPR B) and give ROS a status line in the
        format it parses:  rp12.34,ln5.00,"""
        prev = {}
        while self.running:
            self.send("PPR A")
            self.send("PPR B")
            time.sleep(0.1)
            vel = {}
            for m in ("A", "B"):
                cur = self.enc.get(m)
                if cur and m in prev and cur[1] > prev[m][1]:
                    vel[m] = (cur[0] - prev[m][0]) / (cur[1] - prev[m][1]) / COUNTS_PER_RAD
                if cur:
                    prev[m] = cur
            if len(vel) == 2:
                fmt = lambda c, v: "%s%s%.2f," % (c, "n" if v < 0 else "p", abs(v))
                self.to_ros(fmt("r", vel["A"]) + fmt("l", vel["B"]) + "\r\n")

    # ------------------------------------------------------------------- main loop
    def run(self):
        self.open_arduino()
        self.open_ros_port()
        threading.Thread(target=self.arduino_reader, daemon=True).start()
        self.init_arduino()
        if self.args.feedback:
            threading.Thread(target=self.feedback_loop, daemon=True).start()
        log.info("Bridge running. Point the ROS hardware parameter 'port' at %s", self.args.ros_port)

        while self.running:
            ready, _, _ = select.select([self.master], [], [], 0.02)
            if ready:
                try:
                    data = os.read(self.master, 4096)
                except BlockingIOError:
                    data = b""
                except OSError as exc:
                    if exc.errno != errno.EIO:
                        raise
                    data = b""
                if data:
                    self.ros_buf += data
                    while b"\n" in self.ros_buf:
                        line, self.ros_buf = self.ros_buf.split(b"\n", 1)
                        self.handle_frame(line.decode("ascii", "replace"))
                    if len(self.ros_buf) > 4096:
                        self.ros_buf = b""

            now = time.monotonic()
            if self.args.drive == "pid":
                self.apply_pid()  # flush a rate-limited value
            if (self.last_frame and not self.wd_tripped and self.moving()
                    and now - self.last_frame > self.args.watchdog):
                log.warning("No ROS frame for %.2f s -> stopping the wheels", self.args.watchdog)
                self.stop_wheels()
                self.wd_tripped = True

    def shutdown(self):
        self.running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.write(b"\nCSTOP\n")
                self.ser.flush()
                time.sleep(0.1)
                self.ser.close()
        except Exception:
            pass
        if self.link_path and os.path.islink(self.link_path):
            try:
                os.remove(self.link_path)
            except OSError:
                pass


# ----------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arduino", help="Arduino serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200, help="must match SERIAL_BAUD in the sketch")
    ap.add_argument("--ros-port", default="/tmp/ttyROS", help="virtual port path the ROS plugin opens")
    ap.add_argument("--drive", choices=("pid", "keys"), default="pid")
    ap.add_argument("--deadband", type=float, default=0.05, help="rad/s change needed to send a new target")
    ap.add_argument("--min-interval", type=float, default=0.05, help="min seconds between TGT updates per wheel")
    ap.add_argument("--max-rads", type=float, default=10.0, help="keys mode: rad/s that maps to 100%% duty")
    ap.add_argument("--watchdog", type=float, default=0.5, help="stop wheels after this many s without a ROS frame")
    ap.add_argument("--boot-wait", type=float, default=2.5, help="seconds to wait after opening the port (Arduino resets)")
    ap.add_argument("--feedback", action="store_true", help="poll encoders and send rp/lp status lines to ROS")
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO,
                        format="%(asctime)s %(levelname)s %(message)s", datefmt="%H:%M:%S")

    bridge = Bridge(args)

    def _stop(*_):
        bridge.running = False
    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        bridge.run()
    finally:
        bridge.shutdown()
        log.info("Bridge stopped, wheels commanded to stop")


if __name__ == "__main__":
    main()
