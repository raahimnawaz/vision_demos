"""Coordinate-contract tests: raw camera frames vs display frames.

The bug these exist for: `run_local.py` mirrored the frame before perception,
the ROS2 `gesture_node` did not, and `decision.py` steers on `(0.5 - hand_x)`.
Both entry points imported byte-for-byte the same controller and drove opposite
ways -- so "the same module is imported unchanged" was true and still not
enough. Nothing caught it because every decision test passes `hand_x` as a
literal, and nothing crossed perception into decision.

Runs in CI: `to_display_frame()` and `prepare()` are frame arithmetic, so no
camera, no mediapipe and no rclpy are involved.
"""

import os
import sys
from types import SimpleNamespace

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from decision import ControllerConfig, GestureController      # noqa: E402
from perception import GestureSource, to_display_frame        # noqa: E402


def raw_camera_frame(width=64, height=16, band=8):
    """A frame with a bright marker at camera-left, in RAW camera coordinates.

    Stands in for a hand: all these tests need is something asymmetric whose
    centroid can be measured the way MediaPipe measures a landmark centroid.
    """
    frame = np.zeros((height, width, 3), np.uint8)
    frame[:, :band] = 255
    return frame


def centroid_x(frame):
    """Normalized x of the marker, mimicking GestureObservation.hand_x."""
    cols = np.nonzero(frame.max(axis=(0, 2)))[0]
    assert cols.size, "marker vanished from the frame"
    return float(cols.mean()) / (frame.shape[1] - 1)


def steer(hand_x):
    """(v, w) from a controller that has committed to a driving mode."""
    ctrl = GestureController(ControllerConfig(stable_frames=2))
    out = (0.0, 0.0)
    for _ in range(3):
        out = ctrl.update("Open_Palm", 0.9, True, hand_x)
    assert ctrl.mode == "FORWARD", "setup failed: controller never committed"
    return out


def prepare_with(mirror, frame):
    """Run a frame through GestureSource.prepare() without building MediaPipe.

    Calls the real method against a stub `self`, so this exercises the code both
    entry points call rather than a copy of it.
    """
    return GestureSource.prepare(SimpleNamespace(mirror=mirror), frame)


# --------------------------------------------------------------------------- #
# the contract
# --------------------------------------------------------------------------- #
def test_to_display_frame_mirrors_horizontally():
    raw = raw_camera_frame()
    disp = to_display_frame(raw)
    assert centroid_x(raw) < 0.5, "setup: marker should start at camera-left"
    assert centroid_x(disp) > 0.5, "mirroring must move it to display-right"
    assert np.array_equal(disp, raw[:, ::-1])


def test_mirror_false_is_identity():
    raw = raw_camera_frame()
    assert np.array_equal(to_display_frame(raw, mirror=False), raw)


def test_prepare_delegates_to_the_contract():
    raw = raw_camera_frame()
    for mirror in (True, False):
        assert np.array_equal(prepare_with(mirror, raw),
                              to_display_frame(raw, mirror))


# --------------------------------------------------------------------------- #
# the regression
# --------------------------------------------------------------------------- #
def test_raw_coordinates_invert_steering():
    """The original bug, pinned as an assertion.

    The two conventions are not an offset apart, they are sign-inverted: feeding
    raw camera coordinates to the controller steers the opposite way. That is
    why exactly one of them may reach decision.py, and why the choice cannot be
    left to each call site.
    """
    raw = raw_camera_frame()                       # marker at camera-left

    _, w_display = steer(centroid_x(to_display_frame(raw, mirror=True)))
    _, w_raw = steer(centroid_x(to_display_frame(raw, mirror=False)))

    assert w_display != 0.0, "setup: steering term should be non-zero"
    assert np.isclose(w_display, -w_raw), (
        f"display={w_display:+.3f} raw={w_raw:+.3f} -- the two conventions must "
        "be mirror images of each other; if they are not, the contract drifted"
    )
    # And pin the absolute sign, so a later 'fix' cannot quietly flip it back:
    # camera-left is display-RIGHT, and a hand right of centre steers right.
    assert w_display < 0.0


def test_both_entry_points_agree_on_the_same_frame():
    """run_local.py and gesture_node must produce the same command.

    Both now route through GestureSource.prepare() with mirror defaulting True,
    which is what makes this hold. Note the limit of this test: it pins that the
    shared path agrees with itself and that the default is the mirrored one. It
    cannot see a future entry point that skips prepare() entirely -- that is
    what test_raw_coordinates_invert_steering documents the cost of.
    """
    raw = raw_camera_frame()

    local_frame = prepare_with(True, raw)          # run_local.py
    ros_frame = prepare_with(True, raw)            # gesture_node

    assert np.array_equal(local_frame, ros_frame)
    assert steer(centroid_x(local_frame)) == steer(centroid_x(ros_frame))


def test_mirror_is_the_default_everywhere():
    """A default of False would reintroduce the bug silently."""
    assert to_display_frame(raw_camera_frame()).tolist() != \
        raw_camera_frame().tolist(), "to_display_frame must mirror by default"

    import inspect
    for fn, arg in ((to_display_frame, "mirror"), (GestureSource.__init__, "mirror")):
        default = inspect.signature(fn).parameters[arg].default
        assert default is True, f"{fn.__qualname__}({arg}=) defaults to {default}"


if __name__ == "__main__":
    import traceback
    fns = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    passed = 0
    for f in fns:
        try:
            f(); print(f"PASS {f.__name__}"); passed += 1
        except Exception:
            print(f"FAIL {f.__name__}"); traceback.print_exc()
    print(f"\n{passed}/{len(fns)} framing tests passed")
    sys.exit(0 if passed == len(fns) else 1)
