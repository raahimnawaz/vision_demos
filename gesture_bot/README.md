# gesture_bot — gesture-controlled robot (perception → decision → actuation)

A closed-loop robotics project: a webcam reads your **hand gestures**, a debounced
**decision** layer turns them into a motion command, and a **pluggable actuator**
drives a robot. Every actuator consumes the same command — `(linear_x, angular_z)`,
i.e. a `geometry_msgs/Twist` — so the simulation, a real Arduino, or your computer
are fully interchangeable (a hardware-abstraction layer). These same modules are
imported unchanged by the **ROS2** node graph in [`../ros2/`](../ros2/).

```
 webcam ─▶ perception ─▶ decision ─▶  /cmd_vel (v, ω)  ─▶ ┬─ SimRobot   (2D sim)
          gestures+YOLO   state machine                    ├─ SerialServo(Arduino)
                          + hysteresis                      └─ ComputerHID(keyboard)
```

## Status
- **Phase 1 (done):** framework-free core loop — perception, decision, sim/HID/serial
  backends, unit + integration tests. Runs on macOS (MPS/Metal) or Linux.
- **Phase 2 (done):** Arduino firmware (`firmware/`, prototyped in Wokwi) + live serial
  backend with port auto-detect, diff-drive→servo-µs protocol, and a watchdog failsafe.
- **Phase 3 (done):** the same modules wrapped as ROS2 nodes — `/image_raw` →
  `/gesture` → `/cmd_vel`, plus `/detections`. See [`../ros2/`](../ros2/).
- **Phase 4 (next):** drive a physical board, recorded. See the roadmap in the
  [repo README](../README.md) — the deliverable is where the real actuator
  diverges from `actuators.py`'s integrator, not the video.

## Run (Phase 1)
```bash
source ~/vision_demos_env/bin/activate
cd ~/vision_demos/gesture_bot

python run_local.py --actuator sim       # gesture-driven 2D robot (default)
python run_local.py --actuator hid       # control your computer (dry-run; --live-hid to arm)
python run_local.py --actuator serial    # dry-run serial protocol (no board needed)
python run_local.py --actuator sim --objects   # also overlay YOLO object detection
```

## Gestures
| gesture | action | gesture | action |
|---|---|---|---|
| Open_Palm | drive forward | Closed_Fist | stop |
| Thumb_Up | forward (fast) | Victory | rotate left |
| Thumb_Down | reverse | Pointing_Up | rotate right |

While driving, **lean your hand left/right** to steer (proportional control).
Keys: `q` quit · `s` screenshot · `space` reset.

## Design notes (the robotics bits)
- **Universal command interface** (`/cmd_vel`-style Twist) → actuators are swappable.
- **Debounced state machine**: a gesture must hold ~4 frames above a confidence
  threshold before the mode switches, and a lost hand auto-stops — prevents jitter
  driving the robot erratically (hysteresis).
- **Differential-drive kinematics**: `(v, ω) → (left, right)` wheel speeds → servo
  microseconds for the serial/Arduino backend.
- **Unicycle integrator** for the sim, with a top-down render + path trail.
- **A display-coordinate contract** (`perception.to_display_frame`): `hand_x` is
  measured on the mirrored frame the user sees, because the steering term is
  `steer_gain * (0.5 - hand_x)` — feed it raw camera coordinates and the sign
  inverts. Both this loop and the ROS2 `gesture_node` go through the same
  `GestureSource.prepare()` so they cannot drift apart again.

## Modules
| file | role | reused by ROS2? |
|---|---|---|
| `perception.py` | gesture recognizer + YOLO wrapper + coordinate contract | yes (`gesture_node`) |
| `decision.py` | debounced gesture→Twist state machine | yes (`decision_node`) |
| `actuators.py` | Actuator HAL: Sim / Serial / HID | yes (`base_driver_node`) |
| `run_local.py` | wires the loop (no ROS2) | replaced by the launch file |
| `firmware/` | Arduino sketch + Wokwi project for the serial backend | — (runs on the MCU) |

## Tests

```bash
pytest tests -v                  # 25 tests, no camera / GPU / Arduino needed
```

| file | covers |
|---|---|
| `test_decision.py` | debounce, gesture→mode mapping, steering, auto-stop |
| `test_sim.py` | diff-drive kinematics, unicycle integrator, arena clamp, serial framing |
| `test_serial_live.py` | the live write path against an injected fake transport |
| `test_framing.py` | the display-coordinate contract and its steering sign |

Each file also runs standalone (`python tests/test_decision.py`) with a plain
pass/fail summary, for when pytest isn't installed.
