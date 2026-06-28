# gesture_bot — gesture-controlled robot (perception → decision → actuation)

A closed-loop robotics project: a webcam reads your **hand gestures**, a debounced
**decision** layer turns them into a motion command, and a **pluggable actuator**
drives a robot. Every actuator consumes the same command — `(linear_x, angular_z)`,
i.e. a `geometry_msgs/Twist` — so the simulation, a real Arduino, or your computer
are fully interchangeable (a hardware-abstraction layer). Built to be wrapped as a
**ROS2** node graph (Phase 3).

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
- **Phase 3 (next):** wrap the same modules as ROS2 nodes (`/gesture`, `/cmd_vel`, `/detections`).

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

## Modules
| file | role | reused by ROS2? |
|---|---|---|
| `perception.py` | gesture recognizer + YOLO wrapper | yes (perception_node) |
| `decision.py` | debounced gesture→Twist state machine | yes (decision_node) |
| `actuators.py` | Actuator HAL: Sim / Serial / HID | yes (actuator_node) |
| `run_local.py` | wires the loop (Phase 1, no ROS2) | replaced by launch file |
| `firmware/` | Arduino sketch + Wokwi project for the serial backend | — (runs on the MCU) |

## Tests
```bash
python tests/test_decision.py    # 6 tests: debounce, mapping, steering, auto-stop
python tests/test_sim.py         # 7 tests: kinematics, integrator, serial protocol
```
