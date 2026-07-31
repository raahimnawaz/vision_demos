# object_detection

The **perception library** the rest of the repo is built on: `ObjectDetector`
(YOLO11) plus `HandTracker` and `FaceMesh` (MediaPipe), imported by both
`gesture_bot/perception.py` and the ROS2 `detector_node`. It also runs
standalone as a webcam demo — one script, four modes.

`default_device()` resolves **MPS → CUDA → CPU** at runtime, so it runs on a
Linux/NVIDIA box or an Apple Silicon Mac with nothing to configure.

| mode | what it does | model |
|---|---|---|
| `objects` | bounding boxes + labels, 80 COCO classes | YOLO11n (Ultralytics) |
| `hands` | 21-point hand skeletons, up to 2 hands | MediaPipe Hand Landmarker |
| `face` | 468-point face mesh | MediaPipe Face Landmarker |
| `all` | objects + hands + face layered in one window | all of the above |

## Run
```bash
source ~/vision_demos_env/bin/activate
./../download_models.sh           # if you haven't already (from repo root)
python vision_demo.py objects     # or: hands | face | all
python vision_demo.py objects --conf 0.5 --cam 1
```
Keys: `q` quit · `s` screenshot · `m` toggle the dense face mesh (face/all modes).

## Notes
- YOLO weights (`yolo11n.pt`) auto-download on first run; MediaPipe `.task`
  bundles come from `download_models.sh`. Both live at the repo root and are
  gitignored.
- `ObjectDetector` and the drawing helpers here are reused by
  [`../gesture_bot/`](../gesture_bot/) — the same object detector feeds the
  gesture-controlled robot.
