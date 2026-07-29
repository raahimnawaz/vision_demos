# vision_demos

[![CI](https://github.com/raahimnawaz/vision_demos/actions/workflows/ci.yml/badge.svg)](https://github.com/raahimnawaz/vision_demos/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://www.python.org/)

Local, GPU-accelerated computer vision on Apple Silicon (MPS / Metal; CUDA on
Linux), building up to a closed perception → decision → actuation loop.

Three self-contained projects, increasing in ambition:

| folder | what it is |
|---|---|
| [`object_detection/`](object_detection/) | Realtime webcam demos — YOLO11 object detection, MediaPipe hand tracking, face mesh, and an all-in-one overlay (~30 fps). |
| [`locateanything/`](locateanything/) | Open-vocabulary localization with NVIDIA's **LocateAnything-3B** vision-language model via MLX — describe an object in words, get boxes. |
| [`gesture_bot/`](gesture_bot/) | **The headline project.** Webcam **gestures → decision → actuation**, with pluggable backends (2D sim, Arduino serial, computer HID) behind one `Twist`-style command. |
| [`ros2/`](ros2/) | The same loop as a **ROS2 node graph** — `/image_raw` → `/gesture` → `/cmd_vel`, reusing the modules below unchanged. |

## gesture_bot — closing the loop

The other two folders stop at perception: pixels in, boxes out. `gesture_bot`
carries a perception result all the way to something that moves.

```
webcam ─[MediaPipe]→ gesture ─[debounced state machine]→ (v, ω) ─[actuator]→ sim | Arduino | HID
```

The interesting part is the middle. A raw per-frame gesture classifier is far
too twitchy to drive anything — a single misread frame would slam a robot into
reverse. So the decision layer in [`gesture_bot/decision.py`](gesture_bot/decision.py)
is a debounced state machine with three guards:

- **Confidence gate** — observations below `confidence_min` (0.55) are discarded outright.
- **Stability requirement** — a gesture must persist `stable_frames` (4) consecutive frames before it commits to a mode change.
- **Dead-man timeout** — `lost_frames` (6) frames without a hand in view falls back to `STOP`, so walking out of frame stops the robot rather than latching the last command.

Output is `(linear.x, angular.z)` — deliberately the same pair that becomes
`geometry_msgs/Twist` on `/cmd_vel`. `decision.py` imports nothing but
`dataclasses`, which is why it is fully unit-tested without a camera — and why
the [ROS2 graph in `ros2/`](ros2/) imports it unchanged rather than
reimplementing it.

| Gesture | Mode |
|---|---|
| Open palm | `FORWARD` |
| Thumb up | `FORWARD_FAST` |
| Thumb down | `REVERSE` |
| Closed fist | `STOP` |
| Victory | `ROTATE_LEFT` |
| Pointing up | `ROTATE_RIGHT` |

Hand position left/right of center adds a proportional steering term on top of
the committed mode, so the arm's horizontal offset trims the heading while the
gesture sets the gear.

### Actuator backends

All three implement the same interface, so the perception and decision layers
never know which one is attached:

| Backend | What it does |
|---|---|
| `sim` | 2D differential-drive integrator with an OpenCV top-down view. |
| `serial` | Frames `(v, ω)` into differential wheel commands over pyserial to an Arduino ([firmware + Wokwi project](gesture_bot/firmware/)). |
| `hid` | Maps modes to keystrokes via pyautogui — drive anything that takes a keyboard. |

Both the HID and serial backends default to **dry-run** — they print what they
would send and transmit nothing. `--live-hid` arms the keystrokes; in serial
mode, `SerialServo.find_arduino_port()` auto-detects a board and goes live if
one is present, falling back to dry-run when none is found. The serial framing
is tested against an injected fake transport that asserts the exact bytes the
firmware receives, and the sketch runs in [Wokwi](gesture_bot/firmware/wokwi/)
without physical hardware.

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

## Tests

```bash
pip install pytest numpy opencv-python-headless
pytest gesture_bot/tests -v
```

The decision state machine, the differential-drive kinematics, and the serial
byte framing are all covered without a camera, GPU, or Arduino — that is the
point of keeping `decision.py` dependency-free.

## Layout

```
object_detection/   YOLO + MediaPipe webcam demos
locateanything/     NVIDIA LocateAnything-3B (MLX) — open-vocabulary localization
gesture_bot/        gesture → decision → actuation (sim / Arduino / HID)
  ├─ perception.py    MediaPipe gesture recognition
  ├─ decision.py      debounced state machine → (v, ω)   [dependency-free]
  ├─ actuators.py     sim / serial / HID backends
  ├─ firmware/        Arduino sketch + Wokwi project for the serial backend
  └─ tests/           decision, kinematics, and serial-framing tests
models/             MediaPipe .task bundles (gitignored; via download_models.sh)
testimg/            small test fixtures
```

## Notes

- Model weights (`*.pt`) and MediaPipe bundles (`models/*.task`) are not version-controlled — they auto-download (YOLO) or are fetched by `download_models.sh`.
- The same `ObjectDetector` powers both `object_detection/` and `gesture_bot/`.
- On Linux with an NVIDIA GPU, YOLO uses CUDA automatically.

## License

MIT — see [LICENSE](LICENSE).
