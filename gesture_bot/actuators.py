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
        # keep inside the arena (bounce off walls)
        self.x = min(max(self.x, 0.1), self.world_m - 0.1)
        self.y = min(max(self.y, 0.1), self.world_m - 0.1)
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
                 dry_run=True):
        self.wheel_base = wheel_base
        self.v_max = v_max
        self.dry_run = dry_run or port is None
        self._last_cmd = "-"
        self._ser = None
        if not self.dry_run:
            import serial  # pyserial
            self._ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(2.0)  # let the board reset

    def _to_us(self, wheel_speed):
        """Map wheel linear speed [-v_max, v_max] -> servo us [1000, 2000]."""
        frac = max(-1.0, min(1.0, wheel_speed / self.v_max))
        return int(1500 + frac * 500)

    def apply(self, linear_x, angular_z, dt):
        left, right = diff_drive(linear_x, angular_z, self.wheel_base)
        lu, ru = self._to_us(left), self._to_us(right)
        line = f"{lu},{ru}\n"
        self._last_cmd = line.strip()
        if not self.dry_run and self._ser:
            self._ser.write(line.encode())

    def telemetry(self):
        tag = "dry" if self.dry_run else "LIVE"
        return [f"serial[{tag}]: {self._last_cmd} us"]

    def close(self):
        if self._ser:
            self._ser.close()


def make_actuator(kind, **kwargs):
    kinds = {"sim": SimRobot, "hid": ComputerHID, "serial": SerialServo}
    if kind not in kinds:
        raise ValueError(f"unknown actuator '{kind}', choose from {list(kinds)}")
    return kinds[kind](**kwargs)
