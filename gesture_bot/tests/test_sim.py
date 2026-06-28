"""Unit tests for the diff-drive kinematics + sim integrator (no GUI needed)."""

import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from actuators import SimRobot, SerialServo, diff_drive  # noqa: E402


def test_diff_drive_straight_is_equal_wheels():
    l, r = diff_drive(1.0, 0.0, wheel_base=0.3)
    assert abs(l - r) < 1e-9 and l > 0


def test_diff_drive_turn_splits_wheels():
    l, r = diff_drive(0.0, 1.0, wheel_base=0.3)   # spin left in place
    assert l < 0 < r


def test_sim_forward_moves_along_heading():
    bot = SimRobot()
    x0, y0 = bot.x, bot.y
    for _ in range(10):
        bot.apply(1.0, 0.0, dt=0.1)   # 1 m/s for 1s total
    # heading starts at 0 (+x), so x should grow, y unchanged
    assert bot.x > x0 + 0.5
    assert abs(bot.y - y0) < 1e-6


def test_sim_rotation_changes_heading():
    bot = SimRobot()
    th0 = bot.theta
    for _ in range(10):
        bot.apply(0.0, 1.0, dt=0.1)   # ~1 rad total
    assert abs(bot.theta - th0) > 0.5


def test_sim_stays_in_arena():
    bot = SimRobot(world_m=4.0)
    for _ in range(1000):
        bot.apply(5.0, 0.0, dt=0.1)   # drive hard into a wall
    assert 0.0 <= bot.x <= bot.world_m
    assert 0.0 <= bot.y <= bot.world_m


def test_serial_protocol_format_and_range():
    s = SerialServo(dry_run=True, v_max=1.0)
    s.apply(0.0, 0.0, dt=0.1)          # stop -> both 1500us
    assert s._last_cmd == "1500,1500"
    s.apply(1.0, 0.0, dt=0.1)          # full forward -> both 2000us
    assert s._last_cmd == "2000,2000"
    s.apply(-1.0, 0.0, dt=0.1)         # full reverse -> both 1000us
    assert s._last_cmd == "1000,1000"


def test_serial_us_clamped():
    s = SerialServo(dry_run=True, v_max=1.0)
    s.apply(10.0, 0.0, dt=0.1)         # way over v_max -> clamped to 2000
    lu, ru = s._last_cmd.split(",")
    assert int(lu) == 2000 and int(ru) == 2000


if __name__ == "__main__":
    import traceback
    fns = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    passed = 0
    for f in fns:
        try:
            f(); print(f"PASS {f.__name__}"); passed += 1
        except Exception:
            print(f"FAIL {f.__name__}"); traceback.print_exc()
    print(f"\n{passed}/{len(fns)} sim tests passed")
    sys.exit(0 if passed == len(fns) else 1)
