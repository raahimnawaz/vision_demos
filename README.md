# vision_demos

Realtime computer-vision projects running locally on Apple Silicon (GPU-accelerated
via MPS / Metal). Two parts:

1. **`vision_demo.py`** — realtime webcam demos: YOLO11 object detection, MediaPipe
   hand tracking, face mesh, and an all-in-one overlay. ~30 fps on an M-series Mac.
2. **`gesture_bot/`** — a "close the loop" robotics project: webcam **gestures →
   decision → actuation**, with pluggable backends (2D sim, Arduino serial, computer
   HID) behind one `Twist`-style command, designed to be wrapped as a ROS2 graph.
   See [`gesture_bot/README.md`](gesture_bot/README.md).

## Setup
```bash
python3.12 -m venv ~/vision_demos_env        # MediaPipe/Torch need Python 3.11-3.13
source ~/vision_demos_env/bin/activate
pip install opencv-python mediapipe ultralytics pyserial pyautogui
./download_models.sh                          # fetch MediaPipe .task bundles
```

## Run
```bash
# realtime demos
python vision_demo.py objects        # or: hands | face | all

# gesture-controlled robot (Phase 1)
cd gesture_bot && python run_local.py --actuator sim
```

## Notes
- Model weights (`*.pt`) and MediaPipe bundles (`models/*.task`) are not version-
  controlled — they auto-download (YOLO) or are fetched by `download_models.sh`.
- YOLO uses Apple **MPS**; MediaPipe uses **Metal** GL. On Linux with an NVIDIA GPU,
  YOLO uses CUDA automatically.
