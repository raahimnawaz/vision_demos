"""Tests for the live serial write path, using an injected fake transport
(no Arduino required). Verifies the exact bytes the firmware will receive."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from actuators import SerialServo  # noqa: E402


class FakeSerial:
    """Stand-in for pyserial: records everything written."""
    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, b):
        self.writes.append(b)
        return len(b)

    def close(self):
        self.closed = True


def test_injected_transport_is_live_not_dry():
    fs = FakeSerial()
    s = SerialServo(transport=fs, v_max=1.0)
    assert s.dry_run is False


def test_forward_writes_max_microseconds():
    fs = FakeSerial()
    s = SerialServo(transport=fs, v_max=1.0)
    s.apply(1.0, 0.0, dt=0.1)
    assert fs.writes[-1] == b"2000,2000\n"


def test_turn_splits_wheels_over_serial():
    fs = FakeSerial()
    s = SerialServo(transport=fs, v_max=1.0, wheel_base=0.3)
    s.apply(0.0, 1.0, dt=0.1)              # spin left in place
    left_us, right_us = fs.writes[-1].decode().strip().split(",")
    assert int(left_us) < 1500 < int(right_us)


def test_close_sends_failsafe_stop_then_closes():
    fs = FakeSerial()
    s = SerialServo(transport=fs, v_max=1.0)
    s.apply(1.0, 0.0, dt=0.1)
    s.close()
    assert fs.writes[-1] == b"1500,1500\n"  # last thing sent is a stop
    assert fs.closed is True


def test_every_line_is_well_formed():
    fs = FakeSerial()
    s = SerialServo(transport=fs, v_max=1.0)
    for v, w in [(0.2, 0.0), (-0.4, 0.3), (1.0, -1.0), (0.0, 0.0)]:
        s.apply(v, w, dt=0.1)
    for b in fs.writes:
        text = b.decode().strip()
        lu, ru = text.split(",")
        assert 1000 <= int(lu) <= 2000 and 1000 <= int(ru) <= 2000


if __name__ == "__main__":
    import traceback
    fns = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    passed = 0
    for f in fns:
        try:
            f(); print(f"PASS {f.__name__}"); passed += 1
        except Exception:
            print(f"FAIL {f.__name__}"); traceback.print_exc()
    print(f"\n{passed}/{len(fns)} serial-live tests passed")
    sys.exit(0 if passed == len(fns) else 1)
