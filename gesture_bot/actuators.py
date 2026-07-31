"""
Actuation layer (the hardware-abstraction layer).

Every backend consumes the SAME command -- (linear_x, angular_z), i.e. a
geometry_msgs/Twist -- so they are fully interchangeable. In the ROS2 version
each of these becomes a node subscribing to /cmd_vel; here they are plain objects.

Backends:
    SimRobot     2D differential-drive simulation with a live top-down render.
    ComputerHID  maps the command to keyboard/media actions (volume, play/pause).
    SerialServo  diff-drive kinematics -> servo microseconds over a serial link
                 (firmware prototyped in Wokwi; real port opened in Phase 2).
"""

import math
import time
from abc import ABC, abstractmethod

import cv2
import numpy as np


def diff_drive(linear_x, angular_z, wheel_base):
    """Unicycle (v, w) -> (left, right) wheel linear speeds."""
    left = linear_x - angular_z * wheel_base / 2.0
    right = linear_x + angular_z * wheel_base / 2.0
    return left, right


class Actuator(ABC):
    name = "actuator"

    @abstractmethod
    def apply(self, linear_x, angular_z, dt):
        """Act on one motion command. dt = seconds since previous call."""

    def render(self):
        """Optional visualization frame (BGR ndarray) or None."""
        return None

    def telemetry(self):
        """Short status strings for the HUD."""
        return []

    def close(self):
        pass


# --------------------------------------------------------------------------- #
class SimRobot(Actuator):
    """Top-down 2D differential-drive robot integrated with a unicycle model."""
    name = "sim"

    WALL_MARGIN = 0.1        # m from the arena edge; see the clamp in apply()

    def __init__(self, world_m=6.0, px_per_m=90, wheel_base=0.3):
        self.world_m = world_m
        self.px_per_m = px_per_m
        self.wheel_base = wheel_base
        self.size = int(world_m * px_per_m)
        self.reset()

    def reset(self):
        self.x = self.world_m / 2.0
        self.y = self.world_m / 2.0
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        self.trail = []

    def apply(self, linear_x, angular_z, dt):
        self.v, self.w = linear_x, angular_z
        # unicycle integration
        self.theta = (self.theta + angular_z * dt) % (2 * math.pi)
        self.x += linear_x * math.cos(self.theta) * dt
        self.y += linear_x * math.sin(self.theta) * dt
        # Keep inside the arena. This is a clamp, not a bounce: the robot stops
        # at the wall and keeps its heading, so a command driving into the wall
        # stays pinned there until the command changes.
        lo, hi = self.WALL_MARGIN, self.world_m - self.WALL_MARGIN
        self.x = min(max(self.x, lo), hi)
        self.y = min(max(self.y, lo), hi)
        self.trail.append((self.x, self.y))
        if len(self.trail) > 400:
            self.trail.pop(0)

    def _to_px(self, x, y):
        # y up in world -> y down in image
        return int(x * self.px_per_m), int(self.size - y * self.px_per_m)

    def render(self):
        img = np.full((self.size, self.size, 3), 24, np.uint8)
        step = self.px_per_m
        for g in range(0, self.size + 1, step):          # 1m grid
            cv2.line(img, (g, 0), (g, self.size), (45, 45, 45), 1)
            cv2.line(img, (0, g), (self.size, g), (45, 45, 45), 1)
        for i in range(1, len(self.trail)):              # path trail
            cv2.line(img, self._to_px(*self.trail[i - 1]),
                     self._to_px(*self.trail[i]), (60, 120, 60), 2)
        # robot body + heading
        cx, cy = self._to_px(self.x, self.y)
        r = int(0.18 * self.px_per_m)
        cv2.circle(img, (cx, cy), r, (0, 200, 255), -1)
        hx = int(cx + 1.6 * r * math.cos(-self.theta))
        hy = int(cy + 1.6 * r * math.sin(-self.theta))
        cv2.line(img, (cx, cy), (hx, hy), (255, 255, 255), 3, cv2.LINE_AA)
        return img

    def telemetry(self):
        return [f"sim x={self.x:4.2f} y={self.y:4.2f} th={math.degrees(self.theta):4.0f}",
                f"v={self.v:+4.2f} w={self.w:+4.2f}"]


# --------------------------------------------------------------------------- #
class ComputerHID(Actuator):
    """Maps the motion command to OS keyboard/media actions.

    dry_run (default) only reports the intended action so it can't mess with your
    machine during testing. Pass live=True to actually send keystrokes.
    """
    name = "hid"

    def __init__(self, live=False, cooldown=0.6):
        self.live = live
        self.cooldown = cooldown
        self._last_fire = 0.0
        self._last_action = "-"
        self._gui = None
        if live:
            try:
                import pyautogui
                pyautogui.FAILSAFE = False
                self._gui = pyautogui
            except Exception as e:
                print(f"(hid) pyautogui unavailable, falling back to dry-run: {e}")
                self.live = False

    def apply(self, linear_x, angular_z, dt):
        now = time.monotonic()
        action = None
        if linear_x > 0.6:
            action = ("volumeup", "VOLUME +")
        elif linear_x < -0.3:
            action = ("volumedown", "VOLUME -")
        elif angular_z > 0.6:
            action = ("right", "SEEK >")
        elif angular_z < -0.6:
            action = ("left", "< SEEK")
        if action and now - self._last_fire >= self.cooldown:
            key, label = action
            self._last_action = ("[LIVE] " if self.live else "[dry] ") + label
            if self.live and self._gui:
                try:
                    self._gui.press(key)
                except Exception as e:
                    self._last_action = f"hid error: {e}"
            self._last_fire = now

    def telemetry(self):
        return [f"hid: {self._last_action}"]


# --------------------------------------------------------------------------- #
class SerialServo(Actuator):
    """Diff-drive command -> servo microseconds over serial.

    Phase 1: dry_run formats and records the command (no port needed). Phase 2:
    set a port to actually open it and write. Firmware lives in firmware/.
    """
    name = "serial"

    def __init__(self, port=None, baud=115200, wheel_base=0.3, v_max=1.0,
                 dry_run=True, transport=None, expect_ack=False):
        self.wheel_base = wheel_base
        self.v_max = v_max
        self._last_cmd = "-"
        self._transport = transport          # any object with .write()/.close()
        if transport is not None:            # injected (e.g. for tests)
            self.dry_run = False
        elif port and not dry_run:           # open a real serial port
            import serial  # pyserial
            self._transport = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)                  # let the board reset
            self.dry_run = False
        else:                                # no port -> just format, don't send
            self.dry_run = True

        # --- Phase 4 instrumentation (opt-in) ---------------------------------
        # Requires a transport that can be read from, so a dry run or the
        # write-only fake in the framing tests silently stays uninstrumented.
        self.expect_ack = bool(expect_ack) and hasattr(self._transport, "readline")
        self.latencies_ms = []
        self.last_latency_ms = None
        self.last_ack = None                 # (seq, left_us, right_us, device_ms)
        self.acks_received = 0
        self.acks_missed = 0
        self.seq_gaps = 0                    # commands the device never acked
        self.watchdog_events = 0
        self._expected_seq = 0
        if self.expect_ack:
            self._send("T1\n")               # firmware default is silent

    @staticmethod
    def find_arduino_port():
        """Best-effort autodetect of an Arduino USB serial device, or None."""
        from serial.tools import list_ports
        for p in list_ports.comports():
            dev = p.device or ""
            if any(k in dev for k in ("usbmodem", "usbserial", "ttyACM", "ttyUSB")):
                return dev
        return None

    def _to_us(self, wheel_speed):
        """Map wheel linear speed [-v_max, v_max] -> servo us [1000, 2000]."""
        frac = max(-1.0, min(1.0, wheel_speed / self.v_max))
        return int(1500 + frac * 500)

    def _send(self, line):
        """Raw write. Control lines go here so they never become _last_cmd."""
        if self._transport is not None:
            self._transport.write(line.encode())

    def _write(self, line):
        self._last_cmd = line.strip()
        self._send(line)

    def _read_ack(self, timeout_s=0.25):
        """Read one 'A,...' acknowledgement, or None if the device stayed quiet.

        Watchdog notices are counted and skipped rather than returned -- they
        are unsolicited, so treating one as this command's ack would attribute
        the wrong latency to it.

        Polls rather than sleeping: a real port's own read timeout does the
        waiting, and adding a sleep here would quantise the latency figure this
        exists to measure.
        """
        deadline = time.perf_counter() + timeout_s
        while time.perf_counter() < deadline:
            raw = self._transport.readline()
            if not raw:
                continue
            text = raw.decode(errors="replace").strip()
            if text.startswith("W,"):
                self.watchdog_events += 1
                continue
            if not text.startswith("A,"):
                continue
            parts = text.split(",")
            if len(parts) != 5:
                continue
            try:
                seq, left_us, right_us, device_ms = (int(p) for p in parts[1:])
            except ValueError:
                continue
            self.acks_received += 1
            self._expected_seq += 1
            if seq != self._expected_seq:
                # The device counts every command it accepted, so a jump means
                # commands went unacknowledged between here and there.
                self.seq_gaps += seq - self._expected_seq
                self._expected_seq = seq
            self.last_ack = (seq, left_us, right_us, device_ms)
            return self.last_ack
        return None

    def _send_us(self, left_us, right_us):
        """Write one pulse-width pair and, if instrumented, time its ack."""
        line = f"{left_us},{right_us}\n"
        if not self.expect_ack:
            self._write(line)
            return None

        t0 = time.perf_counter()
        self._write(line)
        ack = self._read_ack()
        if ack is None:
            self.acks_missed += 1
            self.last_latency_ms = None
        else:
            self.last_latency_ms = (time.perf_counter() - t0) * 1000.0
            self.latencies_ms.append(self.last_latency_ms)
        return ack

    def write_us(self, left_us, right_us):
        """Send raw pulse widths, bypassing the (v, w) mapping.

        Phase 4's deadband sweep walks pulse width directly; going through
        apply() would quantise it through _to_us and v_max, which is the very
        mapping the sweep exists to find the error in. Returns the ack or None.
        """
        return self._send_us(int(left_us), int(right_us))

    def apply(self, linear_x, angular_z, dt):
        left, right = diff_drive(linear_x, angular_z, self.wheel_base)
        self._send_us(self._to_us(left), self._to_us(right))

    def telemetry(self):
        tag = "dry" if self.dry_run else "LIVE"
        line = f"serial[{tag}]: {self._last_cmd} us"
        if self.expect_ack and self.last_latency_ms is not None:
            return [line, f"rtt {self.last_latency_ms:5.1f} ms  "
                          f"acks {self.acks_received} miss {self.acks_missed}"]
        return [line]

    def latency_stats(self):
        """Round-trip latency summary in ms, or None if nothing was measured."""
        if not self.latencies_ms:
            return None
        ordered = sorted(self.latencies_ms)
        last = len(ordered) - 1

        def pct(p):
            return ordered[min(last, int(round(p / 100.0 * last)))]

        return {
            "n": len(ordered),
            "min": ordered[0],
            "mean": sum(ordered) / len(ordered),
            "p50": pct(50),
            "p95": pct(95),
            "p99": pct(99),
            "max": ordered[-1],
        }

    def close(self):
        self._write("1500,1500\n")           # failsafe: stop before disconnecting
        if self.expect_ack:
            self._send("T0\n")               # leave the board quiet for the next host
        if self._transport is not None:
            self._transport.close()


def make_actuator(kind, **kwargs):
    kinds = {"sim": SimRobot, "hid": ComputerHID, "serial": SerialServo}
    if kind not in kinds:
        raise ValueError(f"unknown actuator '{kind}', choose from {list(kinds)}")
    return kinds[kind](**kwargs)
