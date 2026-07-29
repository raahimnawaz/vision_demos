"""Locate and import the framework-free gesture_bot modules.

The ROS2 layer deliberately contains no control logic. ``decision.py``,
``actuators.py`` and ``perception.py`` are imported *unchanged* from the main
repo -- which is the whole reason ``decision.py`` was written with no cv2 or
mediapipe import. The nodes here only handle transport, parameters and
failsafes.

Because those modules live in the repo rather than in this ament package, the
source tree has to be found at runtime. Resolution order:

1. ``$GESTURE_BOT_SRC`` -- path to the repo root (the directory *containing*
   ``gesture_bot/``). Set this in your launch file or shell.
2. ``~/vision_demos`` -- the default checkout location used by the repo README.

Relative-to-``__file__`` resolution is deliberately not attempted: after
``colcon build`` this module lives under ``install/`` and no longer sits near
the repo.
"""

import os
import sys

_CANDIDATE_ENV = "GESTURE_BOT_SRC"
_DEFAULT = os.path.expanduser("~/vision_demos")


def _repo_root() -> str:
    tried = []
    for root in (os.environ.get(_CANDIDATE_ENV), _DEFAULT):
        if not root:
            continue
        root = os.path.abspath(os.path.expanduser(root))
        tried.append(root)
        if os.path.isfile(os.path.join(root, "gesture_bot", "decision.py")):
            return root
    raise ImportError(
        "Could not find the gesture_bot source tree. Set "
        f"{_CANDIDATE_ENV}=/path/to/vision_demos (the directory containing "
        f"gesture_bot/). Tried: {tried or '(nothing)'}"
    )


def ensure_on_path() -> str:
    """Put the repo's ``gesture_bot/`` package on sys.path. Returns the repo root."""
    root = _repo_root()
    for entry in (root, os.path.join(root, "gesture_bot")):
        if entry not in sys.path:
            sys.path.insert(0, entry)
    return root


ensure_on_path()

# Imported after the path is set up. `decision` is dependency-free; `actuators`
# needs cv2 + numpy; `perception` additionally needs mediapipe and the
# downloaded .task bundle, so it is imported lazily by gesture_node only.
from decision import GESTURE_MODES, ControllerConfig, GestureController

# ControllerConfig fields that decision_node exposes as live-tunable ROS
# parameters. Kept here rather than in the node so it can be checked against
# the dataclass without importing rclpy -- see test/test_core.py. A field
# renamed in decision.py without updating this tuple would otherwise silently
# stop being tunable.
TUNABLE = (
    "v_forward",
    "v_fast",
    "v_reverse",
    "w_rotate",
    "steer_gain",
    "confidence_min",
    "stable_frames",
    "lost_frames",
)

# Counts, not physical quantities: declared as ROS integer parameters.
INTEGER_PARAMS = ("stable_frames", "lost_frames")

__all__ = [
    "GESTURE_MODES",
    "INTEGER_PARAMS",
    "TUNABLE",
    "ControllerConfig",
    "GestureController",
    "ensure_on_path",
]
