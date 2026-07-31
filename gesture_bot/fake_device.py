"""A software model of firmware/gesture_bot.ino, shaped like a serial transport.

Two jobs.

1. It lets the Phase 4 measurement harness and its tests run with no board
   attached -- same discipline as the rest of the repo, where the decision layer
   and the serial framing are tested without a camera or an Arduino.

2. It is the *prediction*. Phase 4's deliverable is where a real board diverges
   from what the host believes it commanded, and you cannot state a divergence
   without writing down the expectation first. This is that expectation, in
   executable form: ideal timing, exact clamping, and a servo response that is
   linear outside a nominal deadband. Every way the real hardware fails to match
   this is a finding.

Implements write()/readline()/close(), so it drops into ``SerialServo`` exactly
where pyserial goes.
"""

import time


STOP_US = 1500
MIN_US = 1000
MAX_US = 2000
WATCHDOG_MS = 500


def clamp_us(value: int) -> int:
    """The device-side clamp, mirroring clampUs() in the sketch."""
    return max(MIN_US, min(MAX_US, int(value)))


def nominal_wheel_speed(us, deadband_us=0, v_max=1.0):
    """Predicted wheel speed for a pulse width, as a fraction of v_max.

    The model a continuous-rotation servo is *supposed* to follow: nothing
    inside the deadband, then linear out to the rails. Real servos deviate on
    both counts -- the deadband is rarely symmetric and the response is rarely
    linear near the ends -- which is exactly what the deadband sweep measures.
    """
    offset = us - STOP_US
    if abs(offset) <= deadband_us:
        return 0.0
    # Re-zero at the edge of the deadband so the response stays continuous.
    span = (MAX_US - STOP_US) - deadband_us
    if span <= 0:
        return 0.0
    signed = offset - deadband_us if offset > 0 else offset + deadband_us
    return max(-1.0, min(1.0, signed / span)) * v_max


class FakeArduino:
    """Serial-transport-shaped model of the firmware.

    Parameters mirror the things a real board gets wrong, so a test can ask for
    each one deliberately:

        latency_s     round-trip delay before an ack becomes readable
        deadband_us   width of the no-motion band around 1500 us
        drop_every    drop every Nth command, to exercise seq-gap detection
        clock         injectable time source, so tests are deterministic
    """

    def __init__(self, latency_s=0.0, deadband_us=0, drop_every=0, clock=None,
                 watchdog_ms=WATCHDOG_MS):
        self.latency_s = latency_s
        self.deadband_us = deadband_us
        self.drop_every = drop_every
        self.watchdog_ms = watchdog_ms
        self._clock = clock or time.monotonic

        self.telemetry = False
        self.left_us = STOP_US
        self.right_us = STOP_US
        self.rx_count = 0            # commands ACCEPTED, matching the sketch
        self.written = []            # every line received, for assertions
        self.closed = False

        self._t0 = self._clock()
        self._last_command_ms = None
        self._stopped = True
        self._out = []               # (ready_at, line) pending ack queue

    # ------------------------------------------------------------ device time
    def _millis(self):
        return int((self._clock() - self._t0) * 1000)

    def _emit(self, line):
        self._out.append((self._clock() + self.latency_s, line))

    # -------------------------------------------------------------- transport
    def write(self, data):
        # Tick first: the sketch's loop() runs continuously, so a watchdog stop
        # during the gap BEFORE this command arrived has already happened. Only
        # ticking inside readline() would let a late command retroactively
        # prevent a stop that a real board would already have performed.
        self._tick_watchdog()
        text = data.decode() if isinstance(data, (bytes, bytearray)) else data
        for raw in text.splitlines():
            self._handle(raw.strip())
        return len(data)

    def _handle(self, line):
        if not line:
            return
        self.written.append(line)

        if line.startswith("T"):
            self.telemetry = len(line) > 1 and line[1] == "1"
            return

        left, sep, right = line.partition(",")
        # Digits only, matching parseUs() in the sketch. Fields are not trimmed
        # individually there either, so "1500, 1500" is malformed in both.
        if not sep or not left.isdigit() or not right.isdigit():
            return              # ignored entirely: no motion, no ack, no watchdog reset

        self.rx_count += 1
        self.left_us = clamp_us(int(left))
        self.right_us = clamp_us(int(right))
        self._last_command_ms = self._millis()
        self._stopped = False

        # drop_every models a LOST ACK, not a lost command: the device applied
        # it and counted it, so the host sees the seq jump on the next ack.
        dropped = self.drop_every and self.rx_count % self.drop_every == 0
        if self.telemetry and not dropped:
            self._emit("A,%d,%d,%d,%d\n" % (self.rx_count, self.left_us,
                                            self.right_us, self._last_command_ms))

    def _tick_watchdog(self):
        """Stop the wheels if nothing has arrived for watchdog_ms."""
        if self._stopped or self._last_command_ms is None:
            return
        if self._millis() - self._last_command_ms > self.watchdog_ms:
            self.left_us = self.right_us = STOP_US
            self._stopped = True
            if self.telemetry:
                self._emit("W,%d\n" % self._millis())

    def readline(self):
        self._tick_watchdog()
        now = self._clock()
        if self._out and self._out[0][0] <= now:
            return self._out.pop(0)[1].encode()
        return b""

    def close(self):
        self.closed = True

    # ------------------------------------------------------------- inspection
    def wheel_speeds(self, v_max=1.0):
        """Predicted (left, right) wheel speeds for the currently applied us."""
        return (nominal_wheel_speed(self.left_us, self.deadband_us, v_max),
                nominal_wheel_speed(self.right_us, self.deadband_us, v_max))
