"""Phase 4 instrumentation tests: the ack protocol and the measurement path.

Phase 4's whole point is producing trustworthy numbers about real hardware, so
the measuring instrument has to be checked first -- against fake_device, which
models the firmware exactly. If the harness cannot recover a latency it was
told to inject, it cannot be believed about a servo.

No board, no camera, no rclpy.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from actuators import SerialServo  # noqa: E402
from fake_device import (MAX_US, MIN_US, STOP_US, FakeArduino,  # noqa: E402
                         clamp_us, nominal_wheel_speed)


class FakeClock:
    """Injectable monotonic clock, so latency tests are not wall-clock races."""

    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


# --------------------------------------------------------------- the model
def test_device_clamps_like_the_firmware():
    assert clamp_us(500) == MIN_US
    assert clamp_us(9999) == MAX_US
    assert clamp_us(1650) == 1650


def test_device_is_silent_until_telemetry_is_enabled():
    """The firmware default is silent; a host that never reads must not stall it."""
    dev = FakeArduino()
    dev.write(b"1700,1700\n")
    assert dev.readline() == b""
    dev.write(b"T1\n")
    dev.write(b"1700,1700\n")
    assert dev.readline().startswith(b"A,")


def test_ack_reports_post_clamp_values():
    """The ack must say what was APPLIED, not what was asked for."""
    dev = FakeArduino()
    dev.write(b"T1\n")
    dev.write(b"3000,10\n")                      # both out of range
    _, _, left, right, _ = dev.readline().decode().strip().split(",")
    assert (int(left), int(right)) == (MAX_US, MIN_US)


def test_garbage_is_rejected_not_driven_as_full_reverse():
    """A corrupted line must be ignored, not parsed as zero.

    String::toInt() returns 0 for non-numeric text and 0 clamps to MIN_US, so
    before parseUs() a line mangled in transit was applied as full reverse on
    both wheels. That is the worst possible failure mode for a serial glitch.
    """
    dev = FakeArduino()
    dev.write(b"T1\n")
    dev.write(b"1800,1800\n")
    assert dev.readline().startswith(b"A,")

    for junk in (b"ab,cd\n", b"18x0,1800\n", b"1800,\n", b",1800\n", b"1800\n"):
        dev.write(junk)
        assert dev.readline() == b"", f"{junk!r} should produce no ack"
    assert (dev.left_us, dev.right_us) == (1800, 1800), \
        "garbage must not move the wheels"
    assert dev.rx_count == 1, "malformed lines must not be counted as commands"


def test_malformed_input_does_not_refresh_the_watchdog():
    """A host emitting only garbage must still get stopped."""
    clock = FakeClock()
    dev = FakeArduino(clock=clock, watchdog_ms=100)
    dev.write(b"T1\n")
    dev.write(b"1800,1800\n")
    dev.readline()

    clock.advance(0.05)
    dev.write(b"garbage\n")          # must not count as a keep-alive
    clock.advance(0.08)              # 130 ms total since the last real command
    dev.write(b"more,garbage\n")
    assert dev.readline().startswith(b"W,")
    assert dev.left_us == STOP_US


def test_watchdog_fires_once_on_the_edge():
    clock = FakeClock()
    dev = FakeArduino(clock=clock, watchdog_ms=500)
    dev.write(b"T1\n")
    dev.write(b"1800,1800\n")
    assert dev.readline().startswith(b"A,")

    clock.advance(0.6)                           # past the watchdog
    assert dev.readline().startswith(b"W,")
    assert dev.left_us == STOP_US and dev.right_us == STOP_US
    clock.advance(1.0)
    assert dev.readline() == b"", "watchdog must report the edge, not the state"


def test_nominal_wheel_speed_has_a_deadband_and_saturates():
    assert nominal_wheel_speed(STOP_US, deadband_us=40) == 0.0
    assert nominal_wheel_speed(STOP_US + 30, deadband_us=40) == 0.0
    assert nominal_wheel_speed(STOP_US + 100, deadband_us=40) > 0.0
    assert nominal_wheel_speed(MAX_US, deadband_us=40) == 1.0
    assert nominal_wheel_speed(MIN_US, deadband_us=40) == -1.0


# ------------------------------------------------------- the instrumentation
def test_instrumentation_is_off_by_default():
    """A write-only transport must stay uninstrumented rather than crash."""
    class WriteOnly:
        def write(self, b): return len(b)
        def close(self): pass

    servo = SerialServo(transport=WriteOnly(), v_max=1.0, expect_ack=True)
    assert servo.expect_ack is False, "no readline() -> no acks"
    servo.apply(1.0, 0.0, dt=0.1)                # must not raise


def test_enabling_acks_sends_the_telemetry_opt_in():
    dev = FakeArduino()
    SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    assert dev.written[0] == "T1", "the board is silent until asked"


def test_latency_is_recovered_from_the_transport():
    """The instrument must measure what it was told to inject."""
    dev = FakeArduino(latency_s=0.004)
    servo = SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    for _ in range(20):
        servo.apply(0.5, 0.0, dt=0.03)

    stats = servo.latency_stats()
    assert stats["n"] == 20
    assert servo.acks_missed == 0
    assert 3.5 <= stats["p50"] <= 6.0, f"recovered {stats['p50']:.2f} ms for 4 ms"


def test_dropped_commands_are_detected_not_silently_averaged():
    """A gap in the device's sequence must surface, not vanish into the mean.

    Note the two counters disagree by one, and should: drops land on commands
    4, 8 and 12, but a seq gap is only visible in the *next* ack. Command 12 is
    the last one, so nothing follows to expose it. A trailing drop is therefore
    detectable as a missed ack and not as a gap -- which is why the harness
    reports both numbers rather than picking one.
    """
    dev = FakeArduino(drop_every=4)
    servo = SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    for _ in range(12):
        servo.apply(0.5, 0.0, dt=0.01)

    assert servo.acks_missed == 3, f"3 of 12 dropped, saw {servo.acks_missed}"
    assert servo.seq_gaps == 2, "gaps 4 and 8 are exposed by the acks after them"


def test_watchdog_notice_is_not_mistaken_for_an_ack():
    """An unsolicited W line must not be timed as if it answered a command."""
    clock = FakeClock()
    dev = FakeArduino(clock=clock, watchdog_ms=100)
    servo = SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    servo.apply(0.5, 0.0, dt=0.01)
    assert servo.acks_received == 1

    clock.advance(0.5)                           # trip the watchdog
    servo.apply(0.5, 0.0, dt=0.01)
    assert servo.watchdog_events == 1
    assert servo.acks_received == 2, "the ack after the notice still counts"


def test_write_us_bypasses_the_speed_mapping():
    """The deadband sweep walks pulse width directly; _to_us must not quantise it."""
    dev = FakeArduino()
    servo = SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    ack = servo.write_us(1523, 1477)
    assert ack is not None
    assert (ack[1], ack[2]) == (1523, 1477)


def test_close_stops_the_wheels_before_disconnecting():
    dev = FakeArduino()
    servo = SerialServo(transport=dev, v_max=1.0, expect_ack=True)
    servo.apply(1.0, 0.0, dt=0.1)
    servo.close()
    assert dev.left_us == STOP_US and dev.right_us == STOP_US
    assert dev.telemetry is False, "the board should be left quiet"
    assert dev.closed is True


if __name__ == "__main__":
    import traceback
    fns = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    passed = 0
    for f in fns:
        try:
            f(); print(f"PASS {f.__name__}"); passed += 1
        except Exception:
            print(f"FAIL {f.__name__}"); traceback.print_exc()
    print(f"\n{passed}/{len(fns)} phase 4 tests passed")
    sys.exit(0 if passed == len(fns) else 1)
