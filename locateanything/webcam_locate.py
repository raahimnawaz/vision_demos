"""
Live webcam object localization using nvidia/LocateAnything-3B (MLX, 8-bit),
running locally on Apple Silicon.

Usage:
    python webcam_locate.py "cat"
    python webcam_locate.py "person, cup, laptop"

Controls:
    q  -- quit
    c  -- capture current frame and run inference
"""

import sys
import os
import time
import tempfile
import cv2
import numpy as np
from PIL import Image, ImageDraw
import re

import mlx.core as mx
from mlx_vlm import load, apply_chat_template
from mlx_vlm.generate import dispatch as mlx_generate
from mlx_vlm.utils import load_config

MODEL_PATH = "mlx-community/LocateAnything-3B-8bit"

# --- Memory tuning (16GB unified memory machine) ---------------------------
# Short side the captured frame is downscaled to before inference. Vision-token
# count (and thus peak activation memory) scales with image area, so this is the
# single biggest lever against the Metal OOM. Each entry is tried in order if the
# previous one runs out of memory.
DOWNSCALE_LADDER = [
    int(os.environ.get("LOCATE_SHORT_SIDE", "512")),
    448,
    384,
    320,
]


def configure_mlx_memory():
    """Give MLX the full recommended GPU working set and stop it hoarding cache."""
    try:
        info = mx.device_info()
        working_set = int(info.get("max_recommended_working_set_size", 0))
        if working_set:
            # Let MLX wire up to the Apple-recommended max (~11.8GB here) instead
            # of a conservative default, so the ~8.8GB model actually fits.
            mx.set_wired_limit(working_set)
    except Exception as e:
        print(f"(note) could not set wired limit: {e}")
    # Don't let MLX retain a large buffer cache that competes with the OS / camera.
    mx.set_cache_limit(0)


def parse_mixed_results(text):
    results = []
    ref_box_pattern = r"(<ref>.*?</ref>)|(<box>.*?</box>)"
    current_label = None

    for m in re.finditer(ref_box_pattern, text, flags=re.IGNORECASE | re.DOTALL):
        token = m.group(0)
        if token.lower().startswith("<ref>"):
            current_label = re.sub(r"</?ref>", "", token, flags=re.IGNORECASE).strip()
        else:
            content = re.sub(r"</?box>", "", token, flags=re.IGNORECASE)
            nums = re.findall(r"<\s*([0-9]+(?:\.[0-9]+)?)\s*>", content)
            coords = [float(n) for n in nums]
            if len(coords) == 4:
                results.append({"coords": coords, "label": current_label or "object"})
    return results


def draw_detections(frame_bgr, detections):
    h, w = frame_bgr.shape[:2]
    pil_img = Image.fromarray(cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB))
    draw = ImageDraw.Draw(pil_img)

    for det in detections:
        x1, y1, x2, y2 = det["coords"]
        x1, x2 = x1 * w / 1000, x2 * w / 1000
        y1, y2 = y1 * h / 1000, y2 * h / 1000
        draw.rectangle([x1, y1, x2, y2], outline=(0, 255, 0), width=3)
        draw.text((x1 + 4, max(0, y1 - 20)), det["label"], fill=(0, 255, 0))

    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def downscale(pil_image, short_side):
    """Resize so the shorter side == short_side, preserving aspect ratio.
    Dims are rounded to multiples of 28 (the Qwen2.5-VL patch grid)."""
    w, h = pil_image.size
    if min(w, h) <= short_side:
        scaled = pil_image
    else:
        scale = short_side / float(min(w, h))
        new_w = max(28, int(round(w * scale)))
        new_h = max(28, int(round(h * scale)))
        new_w -= new_w % 28
        new_h -= new_h % 28
        scaled = pil_image.resize((new_w, new_h), Image.LANCZOS)
    return scaled


def _is_oom(err):
    msg = str(err).lower()
    return "insufficient memory" in msg or "out of memory" in msg or "oom" in msg


def run_inference(model, processor, config, captured_frame_bgr, category_prompt):
    pil_image = Image.fromarray(cv2.cvtColor(captured_frame_bgr, cv2.COLOR_BGR2RGB))

    raw_prompt = f"Locate all the instances that matches the following description: {category_prompt}."
    formatted_prompt = apply_chat_template(
        processor, config, raw_prompt, num_images=1
    )

    last_err = None
    for short_side in DOWNSCALE_LADDER:
        scaled = downscale(pil_image, short_side)

        with tempfile.NamedTemporaryFile(suffix=".jpg", delete=False) as tmp:
            tmp_path = tmp.name
        scaled.save(tmp_path, quality=90)

        mx.reset_peak_memory()
        try:
            result = mlx_generate.generate(
                model, processor, formatted_prompt, image=tmp_path, verbose=False,
            )
            peak_gb = mx.get_peak_memory() / 1e9
            print(f"  [ok @ {scaled.size[0]}x{scaled.size[1]}, peak {peak_gb:.2f}GB]")
            return result.text if hasattr(result, "text") else str(result)
        except Exception as e:
            last_err = e
            if _is_oom(e):
                print(f"  [OOM @ short side {short_side}px -> clearing cache, "
                      f"retrying smaller]")
                mx.clear_cache()
                continue
            raise
        finally:
            os.remove(tmp_path)
            mx.clear_cache()

    raise RuntimeError(
        f"Inference still OOM'd at the smallest size ({DOWNSCALE_LADDER[-1]}px). "
        f"Free up memory (quit other apps) and retry. Last error: {last_err}"
    )

def main():
    if len(sys.argv) < 2:
        print('Usage: python webcam_locate.py "object description"')
        print('Example: python webcam_locate.py "cat, laptop, cup"')
        sys.exit(1)

    category_prompt = sys.argv[1]
    print(f"Looking for: {category_prompt}")

    configure_mlx_memory()

    print("Loading model...")
    model, processor = load(MODEL_PATH)
    config = load_config(MODEL_PATH)
    mx.clear_cache()
    print("Model loaded.")

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("ERROR: could not open webcam. Check camera permissions for Terminal/your IDE.")
        sys.exit(1)

    # Keep camera buffers modest; the inference image is downscaled anyway.
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("\nControls: press 'c' to capture + run inference, 'q' to quit.\n")

    last_annotated_frame = None

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame from webcam.")
            break

        display_frame = last_annotated_frame if last_annotated_frame is not None else frame
        cv2.imshow("LocateAnything (press c to capture, q to quit)", display_frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("c"):
            print("Captured frame, running inference...")
            start = time.time()

            try:
                output_text = run_inference(
                    model, processor, config, frame, category_prompt
                )
            except Exception as e:
                print(f"Inference failed: {e}")
                mx.clear_cache()
                continue

            elapsed = time.time() - start
            print(f"Inference took {elapsed:.2f}s")
            print(f"Raw output: {output_text}")

            detections = parse_mixed_results(output_text)
            print(f"Found {len(detections)} detection(s): "
                  f"{[d['label'] for d in detections]}")

            last_annotated_frame = draw_detections(frame, detections)

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()