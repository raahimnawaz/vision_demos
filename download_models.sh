#!/usr/bin/env bash
#
# Download the MediaPipe Tasks model bundles used by the vision demos.
# (YOLO weights, e.g. yolo11n.pt, auto-download via ultralytics on first run.)
#
# Usage:  ./download_models.sh
#
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p models && cd models

base="https://storage.googleapis.com/mediapipe-models"

fetch() {  # name url
  if [ -f "$1" ]; then
    echo "  ok   $1 (already present)"
  else
    echo "  get  $1"
    curl -fSL -o "$1" "$2"
  fi
}

fetch hand_landmarker.task    "$base/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task"
fetch face_landmarker.task    "$base/face_landmarker/face_landmarker/float16/latest/face_landmarker.task"
fetch gesture_recognizer.task "$base/gesture_recognizer/gesture_recognizer/float16/latest/gesture_recognizer.task"

echo "All models ready in $(pwd)"
