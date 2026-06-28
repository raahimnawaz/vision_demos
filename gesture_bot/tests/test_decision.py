"""Unit tests for the gesture decision state machine (no camera/model needed)."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from decision import GestureController, ControllerConfig  # noqa: E402


def feed(ctrl, gesture, n, score=0.9, hand=True, hand_x=0.5):
    out = (0.0, 0.0)
    for _ in range(n):
        out = ctrl.update(gesture, score, hand, hand_x)
    return out


def test_debounce_requires_stable_frames():
    ctrl = GestureController(ControllerConfig(stable_frames=4))
    # 3 frames is not enough to commit -> still STOP
    feed(ctrl, "Open_Palm", 3)
    assert ctrl.mode == "STOP"
    # the 4th frame commits FORWARD
    v, w = feed(ctrl, "Open_Palm", 1)
    assert ctrl.mode == "FORWARD"
    assert v > 0


def test_low_confidence_ignored():
    ctrl = GestureController(ControllerConfig(stable_frames=2, confidence_min=0.55))
    feed(ctrl, "Open_Palm", 10, score=0.3)   # below threshold
    assert ctrl.mode == "STOP"


def test_each_gesture_maps_to_expected_sign():
    cfg = ControllerConfig(stable_frames=2)
    cases = {
        "Open_Palm":   lambda v, w: v > 0,
        "Thumb_Up":    lambda v, w: v > 0,
        "Thumb_Down":  lambda v, w: v < 0,
        "Closed_Fist": lambda v, w: v == 0 and w == 0,
        "Victory":     lambda v, w: w > 0 and v == 0,
        "Pointing_Up": lambda v, w: w < 0 and v == 0,
    }
    for g, check in cases.items():
        ctrl = GestureController(cfg)
        v, w = feed(ctrl, g, 3, hand_x=0.5)
        assert check(v, w), f"{g} -> v={v}, w={w}"


def test_thumb_up_is_faster_than_open_palm():
    cfg = ControllerConfig(stable_frames=2)
    c1 = GestureController(cfg); v_slow, _ = feed(c1, "Open_Palm", 3)
    c2 = GestureController(cfg); v_fast, _ = feed(c2, "Thumb_Up", 3)
    assert v_fast > v_slow


def test_proportional_steering_direction():
    ctrl = GestureController(ControllerConfig(stable_frames=2))
    feed(ctrl, "Open_Palm", 3, hand_x=0.5)
    _, w_left = ctrl.update("Open_Palm", 0.9, True, hand_x=0.2)   # hand left
    _, w_right = ctrl.update("Open_Palm", 0.9, True, hand_x=0.8)  # hand right
    assert w_left > 0 and w_right < 0


def test_hand_lost_triggers_stop():
    ctrl = GestureController(ControllerConfig(stable_frames=2, lost_frames=3))
    feed(ctrl, "Open_Palm", 3)
    assert ctrl.mode == "FORWARD"
    for _ in range(3):
        v, w = ctrl.update(None, 0.0, hand_present=False)
    assert ctrl.mode == "STOP" and v == 0 and w == 0


if __name__ == "__main__":
    import traceback
    fns = [f for name, f in sorted(globals().items()) if name.startswith("test_")]
    passed = 0
    for f in fns:
        try:
            f(); print(f"PASS {f.__name__}"); passed += 1
        except Exception:
            print(f"FAIL {f.__name__}"); traceback.print_exc()
    print(f"\n{passed}/{len(fns)} decision tests passed")
    sys.exit(0 if passed == len(fns) else 1)
