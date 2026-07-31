# locateanything

Open-vocabulary object **localization** on a live webcam. You type a description
("keys", "a red mug") and the model returns bounding boxes around matching
objects — no fixed class list, unlike the YOLO detector next door.

Runs [**OWLv2**](https://huggingface.co/google/owlv2-base-patch16-ensemble)
(~150M params) in PyTorch, on **CUDA, MPS, or CPU**. No separate environment —
it uses the same `~/vision_demos_env` as the rest of the repo.

## Setup

```bash
source ~/vision_demos_env/bin/activate
pip install transformers torch opencv-python
```

The model auto-downloads on first run (~600 MB).

## Run

```bash
python webcam_locate.py "keys"
python webcam_locate.py "cup, laptop, phone"
python webcam_locate.py "a red mug" --conf 0.15 --device cuda
```

Controls: `space` pause/resume · `q` quit. Boxes are returned in pixel
coordinates, already un-letterboxed to the original frame.

`--conf` defaults to `0.1`. OWLv2 scores run considerably lower than YOLO's, so
do not read `0.15` here as low confidence the way you would for a YOLO box.

## Speed — measured, not assumed

GTX 980 Ti (`sm_52`), batch 1, three queries:

| dtype | ms/frame | fps | peak VRAM |
|---|---:|---:|---:|
| fp32 | 664 | 1.5 | 1.95 GB |
| **fp16** (default on CUDA) | **490** | **2.0** | **1.61 GB** |

fp16 is the CUDA default: 1.35× faster *and* lighter, despite Maxwell having no
fast half-precision path — this is bandwidth-bound at batch 1, not compute-bound.
MPS and CPU stay fp32.

**This is ~2 fps, not realtime.** OWLv2 resizes every input to 960×960
internally, so ~3,600 patches go through the encoder regardless of camera
resolution — dropping the capture size does not help. For comparison,
[`../object_detection/`](../object_detection/) reports ~30 fps for YOLO11n on
its 80 fixed classes (that figure is the existing demo's claim, not a
back-to-back run against the numbers above).

That is the entire trade in one line: **roughly an order of magnitude more
latency to stop being limited to a fixed vocabulary.** Use YOLO when the class
is in its list; use this when it is not.

## Why OWLv2 and not LocateAnything-3B

This ran NVIDIA's **LocateAnything-3B** through MLX, which pinned it to Apple
Silicon. The blocker on moving it was never MLX — it was memory. 3B parameters
need roughly 6 GB in fp16 for weights alone, before activations, and the target
machine has a 6 GB card. The old code carried a downscale ladder retrying at
512/448/384/320 px purely to dodge OOM, which shows how tight it was even on
16 GB of unified memory.

OWLv2 fits in 1.6 GB, runs anywhere PyTorch does, and needs no OOM ladder. What
is given up is phrase grounding on long descriptive sentences — OWLv2 matches
short noun phrases, not paragraphs. Swap `MODEL_ID` (or `--model`) for an
`IDEA-Research/grounding-dino-*` checkpoint if longer phrases start to matter.

## Interface

`OpenVocabDetector` deliberately mirrors `ObjectDetector` in
[`../object_detection/vision_demo.py`](../object_detection/vision_demo.py) —
same `Detection` fields, same `detect` / `draw` / `process` / `close` methods.
That is what makes the two detectors a launch-time swap rather than two parallel
implementations, and it is what lets the planned `locate_node` publish
`vision_msgs/Detection2DArray` by reusing `detector_node`'s body almost
unchanged.

At ~2 fps, that node belongs on an auxiliary topic — not in the gesture control
path, which runs per frame.
