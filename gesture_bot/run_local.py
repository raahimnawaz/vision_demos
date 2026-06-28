"""
Phase 1 runner: the full perception -> decision -> actuation loop, framework-free.

    source ~/vision_demos_env/bin/activate
    cd ~/vision_demos/gesture_bot
    python run_local.py --actuator sim          # gesture-driven 2D robot (default)
    python run_local.py --actuator hid          # control your computer (dry-run)
    python run_local.py --actuator hid --live-hid
    python run_local.py --actuator serial       # dry-run serial (no board needed)
    python run_local.py --actuator serial --serial-port /dev/cu.usbmodemXXXX

Gestures:  Open_Palm=forward  Thumb_Up=fast  Thumb_Down=reverse  Closed_Fist=stop
           Victory=rotate left  Pointing_Up=rotate right
           (while driving) lean your hand left/right to steer

Keys:  q quit   s screenshot   space reset
"""

import os
import sys
import time
import argparse

import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from vision_demo import draw_hud, SHOTS  # noqa: E402

from perception import GestureSource, ObjectSource          # noqa: E402
from decision import GestureController                       # noqa: E402
from actuators import make_actuator                          # noqa: E402


def main():
    ap = argparse.ArgumentParser(description="Gesture-controlled robot (Phase 1)")
    ap.add_argument("--actuator", choices=["sim", "hid", "serial"], default="sim")
    ap.add_argument("--cam", type=int, default=0)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--objects", action="store_true",
                    help="also overlay YOLO object detection")
    ap.add_argument("--live-hid", action="store_true",
                    help="actually send keystrokes in hid mode")
    ap.add_argument("--serial-port", default=None,
                    help="open a real serial port in serial mode")
    args = ap.parse_args()

    print(f"Loading gesture-control loop (actuator={args.actuator})...")
    gestures = GestureSource(num_hands=1)
    objects = ObjectSource() if args.objects else None
    controller = GestureController()
    if args.actuator == "hid":
        actuator = make_actuator("hid", live=args.live_hid)
    elif args.actuator == "serial":
        actuator = make_actuator("serial", port=args.serial_port)
    else:
        actuator = make_actuator("sim")
    print("Ready.")

    cap = cv2.VideoCapture(args.cam)
    if not cap.isOpened():
        print("ERROR: could not open webcam. Grant camera permission to your "
              "terminal in System Settings > Privacy & Security > Camera.")
        sys.exit(1)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    os.makedirs(SHOTS, exist_ok=True)
    cam_win = f"gesture_bot camera [{args.actuator}]"
    sim_win = "gesture_bot actuator"
    target_dt = 1.0 / max(1.0, args.fps)
    prev = time.monotonic()
    fps = 0.0

    while True:
        loop_start = time.monotonic()
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame.")
            break
        frame = cv2.flip(frame, 1)  # mirror

        if objects:
            frame, _ = objects.process_and_draw(frame)

        obs = gestures.process(frame)
        gestures.draw(frame, obs)

        now = time.monotonic()
        dt = now - prev
        v, w = controller.update(obs.gesture, obs.score, obs.hand_present, obs.hand_x)
        actuator.apply(v, w, dt)

        fps = 0.9 * fps + 0.1 * (1.0 / max(1e-6, dt)) if fps else 1.0 / max(1e-6, dt)
        prev = now

        hud = [f"{fps:4.1f} fps  |  mode: {controller.mode}",
               f"cmd  v={v:+4.2f}  w={w:+4.2f}"] + actuator.telemetry()
        draw_hud(frame, hud)
        cv2.imshow(cam_win, frame)

        sim_img = actuator.render()
        if sim_img is not None:
            cv2.imshow(sim_win, sim_img)

        elapsed = time.monotonic() - loop_start
        key = cv2.waitKey(max(1, int((target_dt - elapsed) * 1000))) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("s"):
            stamp = int(time.time())
            cv2.imwrite(os.path.join(SHOTS, f"gesturebot_cam_{stamp}.png"), frame)
            if sim_img is not None:
                cv2.imwrite(os.path.join(SHOTS, f"gesturebot_sim_{stamp}.png"), sim_img)
            print(f"saved screenshot(s) @ {stamp}")
        elif key == ord(" "):
            controller.reset()
            if hasattr(actuator, "reset"):
                actuator.reset()

    cap.release()
    cv2.destroyAllWindows()
    gestures.close()
    actuator.close()


if __name__ == "__main__":
    main()
