"""
Perception layer for the gesture-controlled robot.

Wraps MediaPipe's Gesture Recognizer (canned gestures + hand landmarks) and
reuses the YOLO object detector from the existing vision_demo. Framework-free:
this same module is imported unchanged by the ROS2 perception node in Phase 3.
"""

import os
import sys
import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2

# reuse helpers + ObjectDetector from the object_detection demo
_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "object_detection"))
from vision_demo import MODELS, ObjectDetector, put_text  # noqa: E402


# The 7 canned labels the default gesture_recognizer.task emits (plus "None").
KNOWN_GESTURES = (
    "Open_Palm", "Closed_Fist", "Pointing_Up",
    "Thumb_Up", "Thumb_Down", "Victory", "ILoveYou",
)


@dataclass
class GestureObservation:
    """One frame of hand perception, in normalized + pixel coords."""
    gesture: Optional[str] = None        # e.g. "Open_Palm", or None if no hand
    score: float = 0.0                   # gesture confidence 0..1
    hand_present: bool = False
    hand_x: float = 0.5                  # hand centroid, normalized [0,1] (display coords)
    hand_y: float = 0.5
    handedness: Optional[str] = None     # "Left" / "Right"
    landmarks_px: List[Tuple[int, int]] = field(default_factory=list)


class GestureSource:
    """Realtime hand-gesture perception in MediaPipe VIDEO mode."""

    def __init__(self, num_hands: int = 1, model_path: Optional[str] = None):
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions
        self.mp = mp
        self._connections = vision.HandLandmarksConnections.HAND_CONNECTIONS
        path = model_path or os.path.join(MODELS, "gesture_recognizer.task")
        if not os.path.exists(path):
            raise FileNotFoundError(
                f"gesture model not found at {path}. Download gesture_recognizer.task "
                f"into {MODELS}/ (see README).")
        opts = vision.GestureRecognizerOptions(
            base_options=BaseOptions(model_asset_path=path),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=num_hands,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.recognizer = vision.GestureRecognizer.create_from_options(opts)
        self._start = time.monotonic()

    def _ts_ms(self) -> int:
        return int((time.monotonic() - self._start) * 1000)

    def process(self, frame_bgr) -> GestureObservation:
        h, w = frame_bgr.shape[:2]
        rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
        mp_img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        res = self.recognizer.recognize_for_video(mp_img, self._ts_ms())

        if not res.hand_landmarks:
            return GestureObservation()

        lms = res.hand_landmarks[0]
        pts = [(int(lm.x * w), int(lm.y * h)) for lm in lms]
        cx = sum(lm.x for lm in lms) / len(lms)
        cy = sum(lm.y for lm in lms) / len(lms)

        gesture, score = None, 0.0
        if res.gestures and res.gestures[0]:
            top = res.gestures[0][0]
            if top.category_name in KNOWN_GESTURES:
                gesture, score = top.category_name, float(top.score)

        handed = res.handedness[0][0].category_name if res.handedness else None
        return GestureObservation(gesture, score, True, cx, cy, handed, pts)

    def draw(self, frame, obs: GestureObservation):
        """Overlay the hand skeleton + gesture label."""
        if not obs.hand_present:
            return frame
        pts = obs.landmarks_px
        for c in self._connections:
            cv2.line(frame, pts[c.start], pts[c.end], (0, 255, 0), 2, cv2.LINE_AA)
        for p in pts:
            cv2.circle(frame, p, 4, (0, 140, 255), -1, cv2.LINE_AA)
        if obs.gesture:
            put_text(frame, f"{obs.gesture} {obs.score:.2f}",
                     (pts[0][0] - 10, pts[0][1] + 28), 0.7, (0, 255, 255), 2)
        return frame

    def close(self):
        self.recognizer.close()


class ObjectSource:
    """Thin wrapper around the existing YOLO detector for the object overlay."""

    def __init__(self, conf: float = 0.4, imgsz: int = 512):
        from types import SimpleNamespace
        self.det = ObjectDetector(SimpleNamespace(conf=conf, imgsz=imgsz))

    def process_and_draw(self, frame):
        frame, info = self.det.process(frame)
        return frame, info
