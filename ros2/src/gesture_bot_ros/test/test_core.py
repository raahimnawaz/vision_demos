"""Tests for the ROS-independent half of the wrapper.

These run without rclpy on purpose. The nodes are deliberately thin, so the
thing actually worth testing is the seam between them and the framework-free
modules -- specifically that the tunable-parameter list still matches
ControllerConfig. A field renamed in decision.py would otherwise silently stop
being exposed as a ROS parameter, with no error anywhere.

    GESTURE_BOT_SRC=/path/to/vision_demos pytest test/ -v
"""

import dataclasses
import os
import sys

import pytest

# Import the package by path so the test runs from a plain checkout, before
# anything has been installed into a ROS workspace.
_PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _PKG_DIR)

# Default GESTURE_BOT_SRC to this checkout: test/ -> gesture_bot_ros/ ->
# src/ -> ros2/ -> repo root.
_REPO = os.path.abspath(os.path.join(_PKG_DIR, "..", "..", ".."))
os.environ.setdefault("GESTURE_BOT_SRC", _REPO)

from gesture_bot_ros import _core


def test_repo_modules_are_importable():
    """_core must find gesture_bot/ and re-export the controller unchanged."""
    assert _core.ControllerConfig is not None
    assert _core.GestureController is not None
    assert "Open_Palm" in _core.GESTURE_MODES


def test_every_tunable_is_a_real_config_field():
    """The ROS parameter list must not drift from the dataclass."""
    fields = {f.name for f in dataclasses.fields(_core.ControllerConfig)}
    unknown = set(_core.TUNABLE) - fields
    assert not unknown, f"TUNABLE names not present on ControllerConfig: {unknown}"


def test_every_config_field_is_tunable():
    """And nothing should be silently un-exposed."""
    fields = {f.name for f in dataclasses.fields(_core.ControllerConfig)}
    missing = fields - set(_core.TUNABLE)
    assert not missing, f"ControllerConfig fields not exposed as parameters: {missing}"


def test_integer_params_are_a_subset_of_tunable():
    assert set(_core.INTEGER_PARAMS) <= set(_core.TUNABLE)


def test_integer_params_really_are_ints():
    """declare_parameter() infers the ROS type from the default, so an int
    field declared from a float default would be rejected at runtime."""
    defaults = _core.ControllerConfig()
    for name in _core.INTEGER_PARAMS:
        assert isinstance(getattr(defaults, name), int), f"{name} is not an int"


def test_non_integer_tunables_are_floats():
    defaults = _core.ControllerConfig()
    for name in set(_core.TUNABLE) - set(_core.INTEGER_PARAMS):
        assert isinstance(getattr(defaults, name), float), f"{name} is not a float"


def test_controller_still_debounces_through_the_reexport():
    """Smoke-test the reused logic via _core, so a bad path shim is caught here."""
    ctrl = _core.GestureController(_core.ControllerConfig(stable_frames=4))
    for _ in range(3):
        ctrl.update("Open_Palm", 0.9, True, 0.5)
    assert ctrl.mode == "STOP", "committed before stable_frames"
    v, _w = ctrl.update("Open_Palm", 0.9, True, 0.5)
    assert ctrl.mode == "FORWARD"
    assert v > 0


def test_controller_dead_man_stops_on_lost_hand():
    ctrl = _core.GestureController(_core.ControllerConfig(stable_frames=2, lost_frames=3))
    for _ in range(2):
        ctrl.update("Open_Palm", 0.9, True, 0.5)
    assert ctrl.mode == "FORWARD"
    for _ in range(3):
        v, w = ctrl.update(None, 0.0, False)
    assert ctrl.mode == "STOP"
    assert (v, w) == (0.0, 0.0)


@pytest.mark.parametrize("label", ["", None, "Not_A_Gesture"])
def test_unrecognized_labels_do_not_commit(label):
    """gesture_node publishes "" for 'hand visible, nothing recognized'."""
    ctrl = _core.GestureController(_core.ControllerConfig(stable_frames=2))
    for _ in range(10):
        ctrl.update(label, 0.99, True, 0.5)
    assert ctrl.mode == "STOP"


# ------------------------------------------------------------------ parameters
@pytest.mark.parametrize("name,value", [
    ("stable_frames", 1),
    ("lost_frames", 6),
    ("confidence_min", 0.0),
    ("confidence_min", 1.0),
    ("publish_rate", 30.0),
    ("gesture_timeout", 0.5),
    ("steer_gain", 0.0),
    ("v_forward", -1.0),          # reversing the default gear is legitimate
])
def test_acceptable_parameters_are_accepted(name, value):
    assert _core.reject_parameter(name, value) is None


@pytest.mark.parametrize("name,value", [
    ("stable_frames", 0),         # 0 would commit on any single frame
    ("lost_frames", -1),          # and disable the dead-man entirely
    ("confidence_min", 1.5),
    ("confidence_min", -0.1),
    ("publish_rate", 0.0),        # 1/rate -> ZeroDivisionError on the timer
    ("gesture_timeout", 0.0),     # every frame instantly stale
    ("steer_gain", -1.0),         # silently inverts steering
])
def test_unsafe_parameters_are_rejected(name, value):
    assert _core.reject_parameter(name, value), f"{name}={value} should be rejected"


def test_unknown_parameters_are_left_alone():
    """Validation must not reject names it has no opinion about."""
    assert _core.reject_parameter("some_future_param", "anything") is None


def test_batch_rejects_if_any_member_is_bad():
    """The desync guard: one bad value must sink the whole batch.

    rclpy applies nothing when the callback rejects, so decision_node validates
    everything up front. If this returned None for a batch containing a bad
    value, the node would apply the good ones and diverge from `ros2 param get`.
    """
    good = ("v_forward", 0.8)
    bad = ("stable_frames", 0)
    assert _core.reject_batch([good]) is None
    assert _core.reject_batch([good, bad]), "a later bad value must still reject"
    assert _core.reject_batch([bad, good]), "an earlier bad value must reject"


def test_batch_reports_the_first_reason():
    reason = _core.reject_batch([("stable_frames", 0), ("confidence_min", 9.0)])
    assert "stable_frames" in reason
