"""Open-vocabulary object localization -- describe an object in words, get boxes.

Runs OWLv2 (google/owlv2-base-patch16-ensemble, ~150M params) on CUDA, MPS, or
CPU. Continuous rather than capture-on-keypress -- but see the speed note below
before expecting a realtime feed.

    python webcam_locate.py "cat"
    python webcam_locate.py "person, cup, laptop"
    python webcam_locate.py "a red mug" --conf 0.15 --device cuda

Controls:
    q  -- quit
    space -- pause / resume

Why OWLv2 and not LocateAnything-3B
-----------------------------------
This module previously ran nvidia/LocateAnything-3B through MLX, which pinned it
to Apple Silicon. The blocker on moving it was never MLX -- it was memory. 3B
parameters need roughly 6 GB in fp16 for weights alone, before activations, and
the target machine has a 6 GB card. The old code carried a DOWNSCALE_LADDER that
retried at 512/448/384/320 px purely to dodge OOM, which tells you how tight it
was even on 16 GB of unified memory.

OWLv2 does the same job -- text query in, boxes out -- at ~150M parameters. It
fits with room to spare and needs no OOM ladder. What is given up is phrase
grounding on long descriptive sentences: OWLv2 matches short noun phrases
("a red mug"), not paragraphs. For this project's purpose -- an open-vocabulary
alternative to YOLO's fixed 80 classes -- that is the right trade. Swap
MODEL_ID for an IDEA-Research/grounding-dino-* checkpoint if longer phrases
start to matter.

Speed, measured rather than assumed
-----------------------------------
On a GTX 980 Ti (sm_52), batch 1, three queries:

    fp32   664 ms/frame   1.5 fps   1.95 GB peak
    fp16   490 ms/frame   2.0 fps   1.61 GB peak

So fp16 is the default on CUDA -- 1.35x faster and lighter, despite Maxwell
having no fast half-precision path, because this is bandwidth-bound rather than
compute-bound at batch 1. MPS and CPU stay fp32.

Two consequences worth stating plainly. **This is ~2 fps, not realtime**: OWLv2
resizes every input to 960x960 internally, so ~3,600 patches go through the
encoder regardless of camera resolution, and lowering the capture size does not
help. And **YOLO11n runs ~30 fps on the same machine** for its 80 fixed classes.
That is the whole trade in one line: roughly 15x the latency to stop being
limited to a fixed vocabulary. Use YOLO when the class is in its list; use this
when it is not. It also means a ROS2 `locate_node` belongs on an auxiliary
topic, not in the gesture control path, which runs per frame.

Interface note
--------------
``OpenVocabDetector`` deliberately mirrors ``ObjectDetector`` in
``object_detection/vision_demo.py``: same ``Detection`` fields, same
``detect`` / ``draw`` / ``process`` / ``close`` methods. That is what lets the
planned ``locate_node`` publish ``vision_msgs/Detection2DArray`` by reusing
``detector_node``'s body almost unchanged, and lets the two detectors be a
launch-time swap rather than two parallel implementations.
"""

import argparse
import os
import sys
import time
from typing import List, NamedTuple

import cv2
import numpy as np

_REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(_REPO, "object_detection"))

MODEL_ID = os.environ.get("LOCATE_MODEL_ID", "google/owlv2-base-patch16-ensemble")


class Detection(NamedTuple):
    """One box in pixel coordinates.

    Field-for-field identical to object_detection.vision_demo.Detection so both
    detectors are interchangeable downstream. ``cls`` is the index of the query
    that matched, so it indexes into ``OpenVocabDetector.queries`` the same way
    YOLO's ``cls`` indexes into its class names.
    """
    x1: int
    y1: int
    x2: int
    y2: int
    cls: int
    label: str
    conf: float


def default_device():
    """Best available torch device: CUDA, then MPS, then CPU.

    Imported from the perception library when available so there is one
    definition of this, with a local fallback for standalone use.
    """
    try:
        from vision_demo import default_device as _dd
        return _dd()
    except Exception:
        try:
            import torch
        except ImportError:
            return "cpu"
        if torch.cuda.is_available():
            return "cuda"
        if getattr(torch.backends, "mps", None) is not None and torch.backends.mps.is_available():
            return "mps"
        return "cpu"


def label_color(idx):
    try:
        from vision_demo import label_color as _lc
        return _lc(idx)
    except Exception:
        rng = np.random.default_rng(idx * 9973)
        return tuple(int(c) for c in rng.integers(64, 255, 3))


class OpenVocabDetector:
    name = "locate"

    def __init__(self, queries, conf=0.1, device=None, model_id=MODEL_ID):
        import torch
        from transformers import Owlv2ForObjectDetection, Owlv2Processor

        self.torch = torch
        self.queries = [q.strip() for q in queries if q.strip()]
        if not self.queries:
            raise ValueError("at least one non-empty query is required")
        self.conf = conf
        self.device = device or default_device()

        self.processor = Owlv2Processor.from_pretrained(model_id)

        # transformers renamed this between 4.x and 5.x:
        #   4.x  post_process_object_detection(outputs, threshold, target_sizes)
        #   5.x  post_process_grounded_object_detection(same three, + text_labels)
        # Both are present on their respective versions and neither is present on
        # the other, so resolve once here rather than per frame. This is not
        # hypothetical tidiness: this repo is developed across machines that
        # currently run 4.48 and 5.14.
        self._post_process = getattr(
            self.processor, "post_process_grounded_object_detection", None
        ) or getattr(self.processor, "post_process_object_detection", None)
        if self._post_process is None:
            raise RuntimeError(
                "Owlv2Processor exposes neither post_process_grounded_object_detection "
                "nor post_process_object_detection; transformers version "
                "is not supported"
            )

        self.model = Owlv2ForObjectDetection.from_pretrained(model_id)

        # transformers renamed this between 4.x and 5.x -- 4.48 has
        # post_process_object_detection, 5.14 has only
        # post_process_grounded_object_detection. Both take the same
        # (outputs, threshold, target_sizes) and return the same
        # scores/labels/boxes, so bind whichever exists rather than pinning a
        # transformers version. Resolved once here, not per frame.
        self._post_process = getattr(
            self.processor, "post_process_grounded_object_detection", None
        ) or getattr(self.processor, "post_process_object_detection", None)
        if self._post_process is None:
            raise RuntimeError(
                "Owlv2Processor exposes neither post_process_grounded_object_detection "
                "nor post_process_object_detection; unsupported transformers version"
            )
        # fp16 on CUDA roughly halves both weights and activations; MPS and CPU
        # stay fp32, where half precision is either unsupported or slower.
        self.dtype = torch.float16 if self.device == "cuda" else torch.float32
        self.model = self.model.to(self.device, dtype=self.dtype).eval()

        # Text features depend only on the queries, so encode them once here
        # rather than per frame -- that is most of the per-call cost avoided.
        self._text_inputs = self.processor(
            text=[self.queries], return_tensors="pt"
        ).to(self.device)

        # Warm up so the first real frame is not paying kernel autotune + alloc.
        self.detect(np.zeros((480, 640, 3), np.uint8))

    def detect(self, frame) -> List[Detection]:
        """Run the model and return structured boxes -- no drawing, no side effects.

        Split out from process() for the same reason ObjectDetector does it: so a
        ROS2 node can publish vision_msgs/Detection2DArray without re-parsing an
        annotated image.
        """
        torch = self.torch
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        inputs = self.processor(images=rgb, return_tensors="pt").to(self.device)
        if self.dtype == torch.float16:
            inputs["pixel_values"] = inputs["pixel_values"].half()
        inputs.update(self._text_inputs)

        with torch.no_grad():
            outputs = self.model(**inputs)

        # target_sizes is (height, width) of the ORIGINAL frame; OWLv2 letterboxes
        # internally to a square, and post-processing undoes that for us.
        h, w = frame.shape[:2]
        results = self._post_process(
            outputs=outputs,
            threshold=self.conf,
            target_sizes=torch.tensor([[h, w]], device=self.device),
        )[0]

        out = []
        for score, label_idx, box in zip(
            results["scores"], results["labels"], results["boxes"]
        ):
            x1, y1, x2, y2 = (int(v) for v in box.tolist())
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(w - 1, x2), min(h - 1, y2)
            if x2 <= x1 or y2 <= y1:
                continue
            cls = int(label_idx)
            out.append(
                Detection(x1, y1, x2, y2, cls, self.queries[cls], float(score))
            )
        return out

    def draw(self, frame, detections):
        """Annotate a frame in place with boxes from detect()."""
        for d in detections:
            color = label_color(d.cls)
            cv2.rectangle(frame, (d.x1, d.y1), (d.x2, d.y2), color, 2)
            tag = f"{d.label} {d.conf:.2f}"
            (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (d.x1, d.y1 - th - 6), (d.x1 + tw + 4, d.y1), color, -1)
            cv2.putText(frame, tag, (d.x1 + 2, d.y1 - 4), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 0, 0), 1, cv2.LINE_AA)
        return frame

    def process(self, frame):
        detections = self.detect(frame)
        frame = self.draw(frame, detections)
        return frame, [f"locate: {len(detections)}"]

    def close(self):
        pass


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("queries", help='comma-separated, e.g. "person, cup, laptop"')
    ap.add_argument("--conf", type=float, default=0.1,
                    help="score threshold (OWLv2 scores run lower than YOLO's; "
                         "0.1 is a sane floor, raise it if boxes are noisy)")
    ap.add_argument("--device", default=None, help="cuda | mps | cpu (default: auto)")
    ap.add_argument("--model", default=MODEL_ID)
    ap.add_argument("--camera", type=int, default=0)
    args = ap.parse_args()

    queries = args.queries.split(",")
    print(f"Looking for: {queries}")
    print(f"Loading {args.model} ...")
    det = OpenVocabDetector(queries, conf=args.conf, device=args.device,
                            model_id=args.model)
    print(f"Model loaded on {det.device} ({det.dtype}).")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print("ERROR: could not open webcam.")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("\nControls: 'q' quit, space pause/resume.\n")
    paused = False
    fps, last = 0.0, time.time()

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from webcam.")
            break

        if not paused:
            frame, notes = det.process(frame)
            now = time.time()
            fps = 0.9 * fps + 0.1 / max(now - last, 1e-6)
            last = now
            cv2.putText(frame, f"{fps:4.1f} fps  {det.device}  {notes[0]}",
                        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
                        cv2.LINE_AA)

        cv2.imshow("open-vocabulary locate (q quit, space pause)", frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord(" "):
            paused = not paused
            last = time.time()

    cap.release()
    cv2.destroyAllWindows()
    det.close()


if __name__ == "__main__":
    main()
