# locateanything

Open-vocabulary object **localization** on a live webcam using NVIDIA's
[**LocateAnything-3B**](https://huggingface.co/nvidia/LocateAnything-3B) vision-
language model, running locally on Apple Silicon via **MLX**. You type a
description ("keys", "the red mug") and the model returns bounding boxes around
matching objects — no fixed class list, unlike the YOLO demo next door.

> ⚠️ **Separate environment.** This demo uses MLX + `mlx-vlm`, which conflict with
> the MediaPipe/Torch stack the other demos use. It runs in its **own** venv
> (`~/locateanything_env`), *not* `~/vision_demos_env`.

## Setup
```bash
python3 -m venv ~/locateanything_env
source ~/locateanything_env/bin/activate
pip install "git+https://github.com/beshkenadze/mlx-vlm@feat/locateanything-3b"
```
The model (`mlx-community/LocateAnything-3B-8bit`) auto-downloads on first run.

## Run
```bash
source ~/locateanything_env/bin/activate
python webcam_locate.py "keys"
python webcam_locate.py "cup, laptop, phone"
```
Controls: `c` capture the current frame and run localization · `q` quit.
Output coordinates are normalized 0–1000 and drawn as boxes on the frame.

## Notes
- This is a heavy **VLM** (~8.8 GB peak unquantized; runs 8-bit here), so inference
  is per-capture (press `c`), not the continuous ~30 fps of the YOLO demo.
- On a 16 GB Mac it can hit a Metal out-of-memory error; the script mitigates this
  with adaptive input downscaling, MLX cache management, and an OOM retry ladder —
  see the comments in `webcam_locate.py`.
- **Why it's here:** a second, contrasting perception approach — a general VLM for
  open-vocabulary grounding vs. the fast fixed-class YOLO detector in
  [`../object_detection/`](../object_detection/).
