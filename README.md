# vision_demos

[![CI](https://github.com/raahimnawaz/vision_demos/actions/workflows/ci.yml/badge.svg)](https://github.com/raahimnawaz/vision_demos/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.11+](https://img.shields.io/badge/python-3.11%2B-blue.svg)](https://www.python.org/)

A closed **perception → decision → actuation** loop: webcam gestures drive a
robot base, as a plain Python loop *and* as a ROS2 node graph — with the same
control logic imported unchanged by both, and a real Arduino at the end of it.

| folder | what it is |
|---|---|
| [`gesture_bot/`](gesture_bot/) | **The project.** Webcam **gestures → decision → actuation**, with pluggable backends (2D sim, Arduino serial, computer HID) behind one `Twist`-style command. |
| [`ros2/`](ros2/) | The same loop as a **ROS2 node graph** — `/image_raw` → `/gesture` → `/cmd_vel`. The nodes add no control logic; they import the modules below unchanged. |
| [`object_detection/`](object_detection/) | The **perception library** the rest of the repo is built on — `ObjectDetector` (YOLO11) plus `HandTracker` and `FaceMesh` (MediaPipe), imported by both `detector_node` and `gesture_bot/perception.py`. Also runs standalone as a webcam demo. |
| [`locateanything/`](locateanything/) | Open-vocabulary localization — describe an object in words, get boxes, no fixed class list. **OWLv2** in PyTorch on CUDA/MPS/CPU, exposing the same `Detection` interface as the YOLO detector. Not yet wired into the ROS2 graph (Phase 5). |

## gesture_bot — closing the loop

`object_detection/` and `locateanything/` stop at perception: pixels in, boxes
out. `gesture_bot` carries a perception result all the way to something that
moves.

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

## Roadmap

| Phase | What | State |
|------|------|-------|
| 1 | Perception library — YOLO11, MediaPipe hands, face mesh | ✅ done |
| 2 | Arduino serial actuation — firmware, byte framing, Wokwi | ✅ done |
| 3 | ROS2 node graph — `/image_raw` → `/gesture` → `/cmd_vel` | ✅ done |
| 4 | Drive real hardware, recorded | ⬜ |
| 5 | Open-vocabulary detector node | 🚧 model swapped; node pending |
| 6 | Latency budget, gesture → motion | ⬜ |
| 7 | rosbag regression test in CI | ⬜ |

**Phase 4 — drive real hardware, recorded.** Everything through Phase 3 runs
against a simulated base, a dry-run serial port, or Wokwi. The framing is
asserted byte-for-byte against an injected fake transport, which proves the
protocol and proves nothing about a motor. This phase is a recorded run of a
gesture turning a physical servo — and, more usefully, where the real actuator
diverges from `actuators.py`'s integrator: serial round-trip latency, servo
deadband, and the gap between commanded `(v, ω)` and what the hardware actually
did. **That discrepancy is the deliverable; the video is only evidence it
happened.**

**Phase 5 — open-vocabulary detector node.** Half done.

**Done — the model swap.** `locateanything/` ran **LocateAnything-3B through
MLX**, which pinned it to Apple Silicon. The blocker was never MLX but memory:
3B parameters need ~6 GB in fp16 for weights alone, before activations, against
a 6 GB card — the old code carried an OOM retry ladder down to 320 px to cope,
even on 16 GB of unified memory. It now runs **OWLv2** (~150M) in PyTorch on
CUDA/MPS/CPU, in 1.6 GB, exposing the same `Detection` fields and
`detect`/`draw`/`process` methods as `ObjectDetector`.

That swap already produced the comparison this phase existed to make, and it is
starker than expected:

| | latency | fps | vocabulary |
|---|---:|---:|---|
| YOLO11n | ~33 ms \* | ~30 \* | 80 fixed classes |
| OWLv2 (fp16) | **490 ms** | **2.0** | anything you can name |

\* The OWLv2 row is measured on a GTX 980 Ti (`sm_52`), batch 1, three queries.
The YOLO row is this repo's existing ~30 fps claim for the webcam demo, *not* a
back-to-back measurement on the same machine — worth re-running both together
when the node below lands.

**Roughly an order of magnitude more latency to stop being limited to a fixed
vocabulary.** And it does
not improve by lowering camera resolution — OWLv2 resizes every input to
960×960 internally, so the patch count is constant. fp16 beats fp32 by 1.35× on
this card despite Maxwell having no fast half path, because at batch 1 it is
bandwidth-bound.

**Not done — the node.** Wrap it as `locate_node` publishing the same
`vision_msgs/Detection2DArray` on `/detections` that `detector_node` already
does, so the two detectors become a launch-time swap with nothing downstream
changed. The 2 fps figure decides its shape: `locate_node` belongs on an
auxiliary topic, **not** in the gesture control path, which runs per frame.

**Phase 6 — latency budget.** Per-node processing time, and end-to-end latency
from hand movement to motor response. How much does enabling `detector_node`
cost the gesture path, and where does the time actually go? The decision layer
currently reasons in frames (`stable_frames`, `lost_frames`); this converts
those into milliseconds, which is the unit that starts to matter once hardware
is in the loop.

**Phase 7 — rosbag regression in CI.** [`ros2/README.md`](ros2/) already argues
for recording `/image_raw`, because that is how you debug a perception bug more
than once. This finishes the thought: commit a short bag, replay it in CI, and
assert the decision layer emits the expected command sequence. Perception
regression testing with no camera, no GPU, and no board — the same property that
keeps `decision.py` dependency-free, extended to the whole graph.

Order is deliberate. **Phase 4 is worth more than 5, 6 and 7 combined**, because
it is the only one that produces something a simulator cannot fake.

## What runs where

Developed across an Apple Silicon Mac and a Linux/WSL box with an NVIDIA GPU.
Almost everything runs on both:

| component | Apple Silicon | Linux / WSL + CUDA |
|---|---|---|
| YOLO11 (`ObjectDetector`) | ✅ MPS | ✅ CUDA, or CPU |
| MediaPipe (hands, face mesh) | ✅ | ✅ |
| `gesture_bot` — sim / HID / Arduino serial | ✅ | ✅ |
| ROS2 node graph ([`ros2/`](ros2/)) | ❌ no supported macOS path | ✅ Jazzy |
| OWLv2 open-vocab ([`locateanything/`](locateanything/)) | ✅ MPS | ✅ CUDA, or CPU |

**ROS2 is now the only hard split**, and it decides where the work happens: four
of the open roadmap phases below run through the node graph, so Linux is the
primary machine. `locateanything/` used to be the opposite split — it ran
LocateAnything-3B through MLX and was Apple-only — until it moved to OWLv2 in
PyTorch. That was a memory decision more than a portability one: 3B parameters
need ~6 GB in fp16 for weights alone against a 6 GB card, while OWLv2 runs in
1.6 GB. See [`locateanything/`](locateanything/) for the measured numbers.

`default_device()` in [`object_detection/vision_demo.py`](object_detection/vision_demo.py)
resolves MPS → CUDA → CPU at runtime, so nothing needs configuring by hand.
Note that a Maxwell-era card (GTX 9xx, `sm_52`) needs a PyTorch build whose
`torch.cuda.get_arch_list()` includes `sm_50` — binary compatibility covers
`sm_52` from there, but newer builds have been dropping the architecture.

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
object_detection/   perception library — ObjectDetector / HandTracker / FaceMesh
                    (imported by gesture_bot and by ros2/detector_node)
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
