"""
Realtime webcam vision demos on Apple Silicon (M5, MPS / Metal accelerated).

Modes:
    objects   YOLO11 object detection -- bounding boxes + labels, 80 COCO classes
              (person, cat, cup, laptop, ...). Runs on the Apple GPU via MPS.
    hands     MediaPipe hand landmarks -- 21 points per hand (up to 2 hands),
              connected into a hand skeleton, with left/right labels.
    face      MediaPipe face mesh -- 468 landmark points + contours/tesselation.

Usage:
    source ~/vision_demos_env/bin/activate
    python vision_demo.py objects
    python vision_demo.py hands
    python vision_demo.py face

    # options:
    python vision_demo.py objects --conf 0.5      # detection confidence
    python vision_demo.py objects --cam 1         # pick a different camera index

Controls (in the video window):
    q  -- quit
    s  -- save a screenshot to ~/vision_demos/shots/
    m  -- (face mode) toggle the dense tesselation mesh on/off
"""

import sys
import os
import time
import argparse
from collections import deque

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, "models")
SHOTS = os.path.join(HERE, "shots")


# --------------------------------------------------------------------------- #
# small drawing helpers
# --------------------------------------------------------------------------- #
def label_color(idx):
    """Deterministic bright BGR color from an integer id."""
    rng = np.random.RandomState(idx * 9973 + 1)
    c = rng.randint(60, 256, size=3)
    return int(c[0]), int(c[1]), int(c[2])


def put_text(img, text, org, scale=0.6, color=(255, 255, 255), thick=1):
    """Text with a dark outline so it's readable on any background."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thick, cv2.LINE_AA)


def draw_hud(frame, lines):
    y = 24
    for ln in lines:
        put_text(frame, ln, (10, y), 0.6, (0, 255, 180), 1)
        y += 24


# --------------------------------------------------------------------------- #
# mode: object detection (YOLO)
# --------------------------------------------------------------------------- #
class ObjectDetector:
    name = "objects"

    def __init__(self, args):
        from ultralytics import YOLO
        weights = os.path.join(HERE, "yolo11n.pt")
        self.model = YOLO(weights if os.path.exists(weights) else "yolo11n.pt")
        self.names = self.model.names
        self.conf = args.conf
        self.imgsz = getattr(args, "imgsz", None) or 640
        # warm up on the GPU so the first real frame isn't slow
        self.model.predict(np.zeros((480, 640, 3), np.uint8),
                           device="mps", imgsz=self.imgsz, verbose=False)

    def process(self, frame):
        res = self.model.predict(frame, device="mps", conf=self.conf,
                                 imgsz=self.imgsz, verbose=False)[0]
        n = 0
        for box in res.boxes:
            n += 1
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            color = label_color(cls)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            tag = f"{self.names[cls]} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(frame, tag, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return frame, [f"objects: {n}"]

    def close(self):
        pass


# --------------------------------------------------------------------------- #
# mediapipe base (hands + face share VIDEO-mode plumbing)
# --------------------------------------------------------------------------- #
class _MPBase:
    def __init__(self):
        self._start = time.monotonic()

    def _ts_ms(self):
        # must be monotonically increasing for VIDEO running mode
        return int((time.monotonic() - self._start) * 1000)

    @staticmethod
    def _to_px(landmark, w, h):
        return int(landmark.x * w), int(landmark.y * h)


# --------------------------------------------------------------------------- #
# mode: hand tracking
# --------------------------------------------------------------------------- #
class HandTracker(_MPBase):
    name = "hands"

    def __init__(self, args):
        super().__init__()
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions
        self.mp = mp
        self.connections = vision.HandLandmarksConnections.HAND_CONNECTIONS
        opts = vision.HandLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=os.path.join(MODELS, "hand_landmarker.task")),
            running_mode=vision.RunningMode.VIDEO,
            num_hands=2,
            min_hand_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )
        self.landmarker = vision.HandLandmarker.create_from_options(opts)

    def process(self, frame):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        res = self.landmarker.detect_for_video(mp_img, self._ts_ms())

        for hand, handedness in zip(res.hand_landmarks, res.handedness):
            pts = [self._to_px(lm, w, h) for lm in hand]
            for c in self.connections:
                cv2.line(frame, pts[c.start], pts[c.end], (0, 255, 0), 2,
                         cv2.LINE_AA)
            for p in pts:
                cv2.circle(frame, p, 4, (0, 140, 255), -1, cv2.LINE_AA)
            side = handedness[0].category_name
            put_text(frame, side, (pts[0][0] - 10, pts[0][1] + 24), 0.7,
                     (0, 255, 255), 2)
        return frame, [f"hands: {len(res.hand_landmarks)}"]

    def close(self):
        self.landmarker.close()


# --------------------------------------------------------------------------- #
# mode: face mesh
# --------------------------------------------------------------------------- #
class FaceMesh(_MPBase):
    name = "face"

    def __init__(self, args):
        super().__init__()
        import mediapipe as mp
        from mediapipe.tasks.python import vision, BaseOptions
        self.mp = mp
        fc = vision.FaceLandmarksConnections
        self.tesselation = fc.FACE_LANDMARKS_TESSELATION
        self.contours = (
            fc.FACE_LANDMARKS_FACE_OVAL + fc.FACE_LANDMARKS_LIPS +
            fc.FACE_LANDMARKS_LEFT_EYE + fc.FACE_LANDMARKS_RIGHT_EYE +
            fc.FACE_LANDMARKS_LEFT_EYEBROW + fc.FACE_LANDMARKS_RIGHT_EYEBROW +
            fc.FACE_LANDMARKS_NOSE
        )
        self.show_mesh = True
        opts = vision.FaceLandmarkerOptions(
            base_options=BaseOptions(
                model_asset_path=os.path.join(MODELS, "face_landmarker.task")),
            running_mode=vision.RunningMode.VIDEO,
            num_faces=1,
        )
        self.landmarker = vision.FaceLandmarker.create_from_options(opts)

    def toggle(self):
        self.show_mesh = not self.show_mesh

    def process(self, frame):
        h, w = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        mp_img = self.mp.Image(image_format=self.mp.ImageFormat.SRGB, data=rgb)
        res = self.landmarker.detect_for_video(mp_img, self._ts_ms())

        for face in res.face_landmarks:
            pts = [self._to_px(lm, w, h) for lm in face]
            if self.show_mesh:
                for c in self.tesselation:
                    cv2.line(frame, pts[c.start], pts[c.end], (80, 80, 80), 1,
                             cv2.LINE_AA)
            for c in self.contours:
                cv2.line(frame, pts[c.start], pts[c.end], (0, 255, 0), 1,
                         cv2.LINE_AA)
            for p in pts:
                cv2.circle(frame, p, 1, (0, 200, 255), -1)
        mesh = "on" if self.show_mesh else "off"
        return frame, [f"faces: {len(res.face_landmarks)}", f"mesh(m): {mesh}"]

    def close(self):
        self.landmarker.close()


# --------------------------------------------------------------------------- #
# mode: all-in-one (objects + hands + face layered in one frame)
# --------------------------------------------------------------------------- #
class CombinedDemo:
    name = "all"

    def __init__(self, args):
        # smaller YOLO input keeps the combined pipeline near realtime
        if getattr(args, "imgsz", None) is None:
            args.imgsz = 512
        self.objects = ObjectDetector(args)
        self.hands = HandTracker(args)
        self.face = FaceMesh(args)
        self.face.show_mesh = False  # contours+dots only; less clutter when layered

    def process(self, frame):
        frame, oi = self.objects.process(frame)   # boxes first (bottom layer)
        frame, hi = self.hands.process(frame)     # hand skeletons
        frame, fi = self.face.process(frame)      # face mesh on top
        return frame, oi + hi + fi

    def toggle(self):
        self.face.toggle()

    def close(self):
        self.objects.close()
        self.hands.close()
        self.face.close()


MODES = {m.name: m for m in
         (ObjectDetector, HandTracker, FaceMesh, CombinedDemo)}


# --------------------------------------------------------------------------- #
# main loop
# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="Realtime webcam vision demos")
    ap.add_argument("mode", choices=list(MODES),
                    help="objects | hands | face | all")
    ap.add_argument("--cam", type=int, default=0, help="camera index (default 0)")
    ap.add_argument("--conf", type=float, default=0.35,
                    help="detection confidence for objects mode")
    ap.add_argument("--fps", type=float, default=30.0,
                    help="max frames per second (default 30)")
    ap.add_argument("--imgsz", type=int, default=None,
                    help="YOLO input size; lower = faster (default 640, 512 in 'all')")
    args = ap.parse_args()

    print(f"Loading '{args.mode}' demo...")
    demo = MODES[args.mode](args)
    print("Ready.")

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print("ERROR: could not open webcam. Check camera permissions for "
              "your terminal/IDE in System Settings > Privacy > Camera.")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    os.makedirs(SHOTS, exist_ok=True)
    win = f"vision_demo [{args.mode}]  (q quit, s screenshot)"
    fps_hist = deque(maxlen=30)
    target_dt = 1.0 / max(1.0, args.fps)
    prev = time.monotonic()

    while True:
        loop_start = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame.")
            break
        frame = cv2.flip(frame, 1)  # mirror, like a selfie cam

        frame, info = demo.process(frame)

        # true end-to-end fps (camera read + inference + draw), measured tick-to-tick
        now = time.monotonic()
        fps_hist.append(1.0 / max(1e-6, now - prev))
        prev = now
        fps = sum(fps_hist) / len(fps_hist)

        draw_hud(frame, [f"{args.mode}  |  {fps:4.1f} fps (cap {args.fps:.0f})"] + info)
        cv2.imshow(win, frame)

        # cap the frame rate: sleep out the remainder of the time budget.
        # waitKey doubles as the sleep AND the key reader, so no CPU spin.
        elapsed = time.monotonic() - loop_start
        wait_ms = max(1, int((target_dt - elapsed) * 1000))
        key = cv2.waitKey(wait_ms) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("s"):
            path = os.path.join(SHOTS, f"{args.mode}_{int(time.time())}.png")
            cv2.imwrite(path, frame)
            print(f"saved {path}")
        elif key == ord("m") and hasattr(demo, "toggle"):
            demo.toggle()

    cap.release()
    cv2.destroyAllWindows()
    demo.close()


if __name__ == "__main__":
    main()
