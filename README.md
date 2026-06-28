# vision_demos

Local, GPU-accelerated computer-vision projects on Apple Silicon (MPS / Metal;
CUDA on Linux). Three self-contained projects, increasing in ambition:

| folder | what it is |
|---|---|
| [`object_detection/`](object_detection/) | Realtime webcam demos — YOLO11 object detection, MediaPipe hand tracking, face mesh, and an all-in-one overlay (~30 fps). |
| [`locateanything/`](locateanything/) | Open-vocabulary localization with NVIDIA's **LocateAnything-3B** vision-language model via MLX — describe an object in words, get boxes. |
| [`gesture_bot/`](gesture_bot/) | **The headline project.** A "close the loop" robot: webcam **gestures → decision → actuation**, with pluggable backends (2D sim, Arduino serial, computer HID) behind one `Twist`-style command. Built to become a ROS2 graph. |

## Setup
```bash
python3.12 -m venv ~/vision_demos_env        # MediaPipe/Torch need Python 3.11-3.13
source ~/vision_demos_env/bin/activate
pip install opencv-python mediapipe ultralytics pyserial pyautogui
./download_models.sh                          # fetch MediaPipe .task bundles
```
(`locateanything/` uses a **separate** MLX venv — see its README.)

## Run
```bash
# realtime detection demos
python object_detection/vision_demo.py objects     # or: hands | face | all

# gesture-controlled robot
cd gesture_bot && python run_local.py --actuator sim   # or: hid | serial
```

## Layout
```
object_detection/   YOLO + MediaPipe webcam demos
locateanything/     NVIDIA LocateAnything-3B (MLX) — open-vocabulary localization
gesture_bot/        gesture → decision → actuation (sim / Arduino / HID), ROS2-ready
  └─ firmware/      Arduino sketch + Wokwi project for the serial backend
models/             MediaPipe .task bundles (gitignored; via download_models.sh)
testimg/            small test fixtures
```

## Notes
- Model weights (`*.pt`) and MediaPipe bundles (`models/*.task`) are not version-
  controlled — they auto-download (YOLO) or are fetched by `download_models.sh`.
- The same `ObjectDetector` powers both `object_detection/` and `gesture_bot/`.
- On Linux with an NVIDIA GPU, YOLO uses CUDA automatically.
