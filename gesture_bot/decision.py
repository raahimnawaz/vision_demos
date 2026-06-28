"""
Decision layer: maps a stream of gesture observations to a robot motion command
(linear.x, angular.z) -- the same pair that becomes geometry_msgs/Twist on
/cmd_vel in the ROS2 version.

Pure Python, no cv2/mediapipe, so it is trivially unit-testable and reused as-is
by the ROS2 decision_node in Phase 3.
"""

from dataclasses import dataclass


# gesture -> high-level motion mode
GESTURE_MODES = {
    "Open_Palm":   "FORWARD",
    "Thumb_Up":    "FORWARD_FAST",
    "Thumb_Down":  "REVERSE",
    "Closed_Fist": "STOP",
    "Victory":     "ROTATE_LEFT",
    "Pointing_Up": "ROTATE_RIGHT",
}


@dataclass
class ControllerConfig:
    v_forward: float = 0.5       # m/s
    v_fast: float = 1.0          # m/s
    v_reverse: float = 0.4       # m/s
    w_rotate: float = 1.2        # rad/s (in-place turns)
    steer_gain: float = 2.0      # rad/s per unit of hand offset from center
    confidence_min: float = 0.55 # ignore low-confidence gestures
    stable_frames: int = 4       # frames a gesture must persist before committing
    lost_frames: int = 6         # frames without a hand before auto-STOP


class GestureController:
    """Debounced gesture state machine with proportional hand steering."""

    DRIVING = {"FORWARD", "FORWARD_FAST", "REVERSE"}

    def __init__(self, config: ControllerConfig = None):
        self.cfg = config or ControllerConfig()
        self.mode = "STOP"
        self._candidate = None
        self._candidate_count = 0
        self._lost_count = 0

    def reset(self):
        self.mode = "STOP"
        self._candidate = None
        self._candidate_count = 0
        self._lost_count = 0

    def update(self, gesture, score, hand_present, hand_x=0.5):
        """Advance the state machine one frame; return (linear_x, angular_z)."""
        # 1) hand lost -> count toward an automatic stop (hysteresis)
        if not hand_present:
            self._lost_count += 1
            self._candidate = None
            self._candidate_count = 0
            if self._lost_count >= self.cfg.lost_frames:
                self.mode = "STOP"
            return self._command(hand_x)
        self._lost_count = 0

        # 2) debounce: a recognized, confident gesture must persist N frames
        valid = (gesture in GESTURE_MODES and score >= self.cfg.confidence_min)
        if valid:
            if gesture == self._candidate:
                self._candidate_count += 1
            else:
                self._candidate = gesture
                self._candidate_count = 1
            if self._candidate_count >= self.cfg.stable_frames:
                self.mode = GESTURE_MODES[gesture]
        else:
            self._candidate = None
            self._candidate_count = 0

        return self._command(hand_x)

    def _command(self, hand_x):
        cfg = self.cfg
        if self.mode == "FORWARD":
            v = cfg.v_forward
        elif self.mode == "FORWARD_FAST":
            v = cfg.v_fast
        elif self.mode == "REVERSE":
            v = -cfg.v_reverse
        else:
            v = 0.0

        if self.mode == "ROTATE_LEFT":
            w = cfg.w_rotate
        elif self.mode == "ROTATE_RIGHT":
            w = -cfg.w_rotate
        elif self.mode in self.DRIVING:
            # proportional steering: hand left of center (x<0.5) turns left (+w)
            w = cfg.steer_gain * (0.5 - hand_x)
        else:
            w = 0.0
        return v, w
