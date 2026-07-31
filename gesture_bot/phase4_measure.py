"""Phase 4: measure where the real actuator diverges from actuators.py.

Everything through Phase 3 runs against a simulated base, a dry-run port, or
Wokwi. The serial framing is asserted byte-for-byte against an injected fake,
which proves the protocol and proves nothing about a motor. This is the tool
that closes that gap, and the numbers it prints -- not the video -- are the
deliverable.

Three measurements, each answering a question the sim cannot:

  latency   What does a command actually cost, host write -> device ack? The
            decision layer reasons in FRAMES (stable_frames, lost_frames). This
            is the conversion to milliseconds, which is the unit that starts to
            matter once a motor is attached.

  deadband  Where does the servo actually start moving? actuators.py maps wheel
            speed linearly onto [1000, 2000] us with no deadband at all, so
            every command near stop is a command the robot ignores. The width
            and the asymmetry are both findings.

  step      Given a commanded (v, w), what did the device actually apply? Host
            clamping, device clamping and transport loss all live here, and the
            acked us is ground truth for what the board did.

Run against a real board:

    python phase4_measure.py latency  --port /dev/ttyACM0 --n 300
    python phase4_measure.py deadband --port /dev/ttyACM0
    python phase4_measure.py step     --port /dev/ttyACM0 --csv step.csv

Or against the software model in fake_device.py, which is how the harness is
tested and how you check your invocation before touching hardware:

    python phase4_measure.py latency --simulate --n 200

The firmware must be the Phase 4 sketch: telemetry is opt-in and SerialServo
enables it with "T1" on connect. An older board will simply never ack, and
every measurement here will report zero acks rather than silently inventing
numbers.
"""

import argparse
import csv
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from actuators import SerialServo, diff_drive  # noqa: E402
from fake_device import MAX_US, MIN_US, STOP_US, FakeArduino  # noqa: E402


# --------------------------------------------------------------------------- #
# connection
# --------------------------------------------------------------------------- #
def connect(args):
    """A SerialServo with telemetry on, backed by a board or by the model."""
    if args.simulate:
        device = FakeArduino(latency_s=args.sim_latency_ms / 1000.0,
                             deadband_us=args.sim_deadband_us)
        servo = SerialServo(transport=device, v_max=args.v_max, expect_ack=True)
        return servo, device

    port = args.port or SerialServo.find_arduino_port()
    if not port:
        sys.exit("No board found. Plug one in, pass --port, or use --simulate.\n"
                 "On WSL the port must be attached with usbipd first -- see "
                 "firmware/README.md.")
    print(f"port: {port}")
    servo = SerialServo(port=port, dry_run=False, v_max=args.v_max,
                        expect_ack=True)
    return servo, None


def require_acks(servo, what):
    if servo.acks_received == 0:
        sys.exit(
            f"\nThe board never acknowledged a command, so {what} cannot be "
            f"measured.\n"
            f"  - Is it running the Phase 4 sketch? Telemetry is opt-in ('T1').\n"
            f"  - {servo.acks_missed} commands went unanswered."
        )


# --------------------------------------------------------------------------- #
# latency
# --------------------------------------------------------------------------- #
def measure_latency(args):
    """Round-trip cost of one command, host write -> device ack."""
    servo, _ = connect(args)
    period = 1.0 / args.rate
    print(f"sending {args.n} commands at {args.rate:.0f} Hz ...")

    try:
        for i in range(args.n):
            start = time.perf_counter()
            # Alternate either side of stop so the board is doing real work
            # rather than repeatedly re-applying an identical pulse width.
            v = args.v_max * (0.5 if i % 2 else -0.5)
            servo.apply(v, 0.0, period)
            slack = period - (time.perf_counter() - start)
            if slack > 0:
                time.sleep(slack)
    finally:
        servo.close()

    require_acks(servo, "latency")
    stats = servo.latency_stats()
    print("\nround-trip latency, host write -> device ack (ms)")
    print(f"  n={stats['n']}  missed={servo.acks_missed}  "
          f"seq gaps={servo.seq_gaps}  watchdog={servo.watchdog_events}")
    for key in ("min", "mean", "p50", "p95", "p99", "max"):
        print(f"  {key:>4}: {stats[key]:7.2f}")

    frames = stats["p95"] / 1000.0 * args.rate
    print(f"\nAt {args.rate:.0f} Hz, p95 latency is {frames:.2f} frame(s).")
    print("decision.py counts in frames; that is the conversion.")
    _write_csv(args.csv, ["i", "latency_ms"],
               list(enumerate(servo.latencies_ms)))
    return stats


# --------------------------------------------------------------------------- #
# deadband
# --------------------------------------------------------------------------- #
def measure_deadband(args):
    """Walk outward from stop until the wheel actually turns.

    actuators.py assumes a linear map with no deadband. Whatever this prints is
    the size of that error, in microseconds, per direction.
    """
    servo, device = connect(args)
    rows, results = [], {}

    try:
        for direction, label in ((+1, "forward"), (-1, "reverse")):
            print(f"\n--- {label} ---")
            onset = None
            us = STOP_US
            while MIN_US <= us <= MAX_US:
                ack = servo.write_us(us, us)     # both wheels, raw pulse width
                rows.append([label, us, ack[1] if ack else None])
                if _observe_motion(args, device, us, label):
                    onset = us
                    break
                us += direction * args.step_us
            results[label] = onset
            print(f"{label}: onset at {onset} us"
                  if onset else f"{label}: no motion up to the rail")
            servo.write_us(STOP_US, STOP_US)
            time.sleep(args.dwell)
    finally:
        servo.close()

    print(f"\ndeadband, relative to {STOP_US} us stop "
          f"(+/- {args.step_us} us, the sweep resolution)")
    for label, onset in results.items():
        if onset is None:
            print(f"  {label:>7}: none observed")
        else:
            print(f"  {label:>7}: {abs(onset - STOP_US):4d} us  (onset {onset})")
    fwd, rev = results.get("forward"), results.get("reverse")
    if fwd and rev:
        asym = abs(fwd - STOP_US) - abs(rev - STOP_US)
        if asym == 0:
            print(f"  asymmetry: none resolved at {args.step_us} us steps")
        else:
            print(f"  asymmetry: {abs(asym)} us more to start "
                  f"{'forward' if asym > 0 else 'reverse'}")
        print("\nactuators.py._to_us maps speed linearly onto [1000, 2000] with "
              "no deadband,\nso every command inside this band is a command the "
              "robot ignores.")
    _write_csv(args.csv, ["direction", "commanded_us", "acked_us"], rows)
    return results


def _observe_motion(args, device, us, label):
    """Did the wheel move? Model in simulate mode, operator otherwise."""
    if device is not None:
        time.sleep(args.dwell)
        left, _ = device.wheel_speeds()
        return abs(left) > 1e-9
    reply = input(f"  {us} us -- moving? [y/N/q] ").strip().lower()
    if reply == "q":
        sys.exit("aborted by operator")
    return reply == "y"


# --------------------------------------------------------------------------- #
# step response
# --------------------------------------------------------------------------- #
def measure_step(args):
    """Commanded (v, w) vs the pulse widths the device says it applied."""
    servo, _ = connect(args)
    steps = [(0.0, 0.0), (0.5, 0.0), (1.0, 0.0), (0.0, 0.0),
             (-0.4, 0.0), (0.0, 1.2), (0.0, -1.2), (0.5, 0.8), (0.0, 0.0)]
    rows = []

    try:
        for v, w in steps:
            for _ in range(args.hold):
                left, right = diff_drive(v, w, servo.wheel_base)
                want_l, want_r = servo._to_us(left), servo._to_us(right)
                servo.apply(v, w, 1.0 / args.rate)
                got_l, got_r = (servo.last_ack[1], servo.last_ack[2]) \
                    if servo.last_ack else (None, None)
                rows.append([v, w, want_l, want_r, got_l, got_r,
                             servo.last_latency_ms])
                time.sleep(1.0 / args.rate)
    finally:
        servo.close()

    require_acks(servo, "the step response")
    mismatches = [r for r in rows
                  if r[4] is not None and (r[2] != r[4] or r[3] != r[5])]
    print(f"\nsteps: {len(rows)}  acked: {servo.acks_received}  "
          f"missed: {servo.acks_missed}  seq gaps: {servo.seq_gaps}")
    print(f"commanded us != applied us on {len(mismatches)} step(s)")
    for row in mismatches[:10]:
        print(f"  v={row[0]:+.2f} w={row[1]:+.2f}  "
              f"wanted ({row[2]},{row[3]})  applied ({row[4]},{row[5]})")
    if mismatches:
        print("\nEvery mismatch is host-side clamping the device disagreed with, "
              "or a dropped command.")
    _write_csv(args.csv,
               ["v", "w", "want_left_us", "want_right_us",
                "applied_left_us", "applied_right_us", "latency_ms"], rows)
    return rows


# --------------------------------------------------------------------------- #
def _write_csv(path, header, rows):
    if not path:
        return
    with open(path, "w", newline="") as fh:
        writer = csv.writer(fh)
        writer.writerow(header)
        writer.writerows(rows)
    print(f"\nwrote {path} ({len(rows)} rows)")


def build_parser():
    ap = argparse.ArgumentParser(
        description="Phase 4: measure the real actuator against actuators.py.",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    def common(p):
        p.add_argument("--port", default=None, help="e.g. /dev/ttyACM0, COM3")
        p.add_argument("--simulate", action="store_true",
                       help="run against fake_device.FakeArduino, no board")
        p.add_argument("--sim-latency-ms", type=float, default=3.0)
        p.add_argument("--sim-deadband-us", type=int, default=40)
        p.add_argument("--v-max", type=float, default=1.0)
        p.add_argument("--csv", default=None, help="write raw samples here")
        p.add_argument("--rate", type=float, default=30.0, help="Hz")
        return p

    lat = common(sub.add_parser("latency", help="round-trip command latency"))
    lat.add_argument("--n", type=int, default=300)

    dead = common(sub.add_parser("deadband", help="where the servo starts moving"))
    dead.add_argument("--step-us", type=int, default=5)
    dead.add_argument("--dwell", type=float, default=0.4)

    step = common(sub.add_parser("step", help="commanded vs applied pulse width"))
    step.add_argument("--hold", type=int, default=10)
    return ap


def main(argv=None):
    args = build_parser().parse_args(argv)
    {"latency": measure_latency,
     "deadband": measure_deadband,
     "step": measure_step}[args.mode](args)


if __name__ == "__main__":
    main()
