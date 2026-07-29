# gesture_bot on ROS2 (Phase 3)

The same perception → decision → actuation loop as [`../gesture_bot/`](../gesture_bot/),
running as a ROS2 node graph. **The nodes add no control logic.** `decision.py`,
`actuators.py` and `perception.py` are imported *unchanged* from the repo — which
is why `decision.py` was written with no cv2 or mediapipe import in the first
place. The wrappers handle transport, parameters, and failsafes only.

```
v4l2_camera ──/image_raw──┬──► gesture_node  ──/gesture────► decision_node ──/cmd_vel──► base_driver_node
       (sensor_msgs/Image)│      (MediaPipe)   (Gesture)      (GestureController)  (Twist)   │ sim | Arduino | HID
                          └──► detector_node ──/detections──►  (optional consumers)          └──► ~/render
                                  (YOLO11)   (vision_msgs/Detection2DArray)              (sensor_msgs/Image)
```

## Interfaces

| topic | type | published by |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | `v4l2_camera` (external) |
| `/gesture` | `gesture_bot_msgs/Gesture` | `gesture_node` |
| `/detections` | `vision_msgs/Detection2DArray` | `detector_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | `decision_node` |
| `/base_driver_node/render` | `sensor_msgs/Image` | `base_driver_node` |

Only **one** custom message exists — `Gesture`, because ROS has no standard type
for canned hand gestures. Detections use `vision_msgs` and motion uses
`geometry_msgs/Twist`, both standard, so anything else in the ecosystem can
consume this graph.

## Four decisions worth explaining

**The camera is a separate node.** The plain Python loop owns the capture device,
which means only it can see the frames. Publishing `/image_raw` instead makes the
whole stack recordable:

```bash
ros2 bag record /image_raw /gesture /cmd_vel
ros2 launch gesture_bot_ros gesture_bot.launch.py camera:=false
ros2 bag play rosbag2_2026_07_29-06_40_00
```

That replays a session deterministically with no webcam and no hand waving —
which is how you debug a perception bug more than once.

**Gesture and detection are separate nodes.** MediaPipe and YOLO are separate
models at separate rates; in one node they block each other. Separate nodes run
independently and the detector simply isn't launched when it isn't wanted
(`detector:=true` to enable).

**`ControllerConfig` fields are ROS parameters.** Retune the controller while it
drives, no restart:

```bash
ros2 param set /decision_node v_forward 0.8
ros2 param set /decision_node stable_frames 6
```

Values that would make the loop unsafe are rejected by a parameter callback
(`stable_frames < 1`, `confidence_min` outside `[0, 1]`).

**Silence means stop, at both hops.** `lost_frames` only counts frames that
actually arrive — if `gesture_node` dies, `update()` is never called again and
the last command would latch forever. So `decision_node` treats a stale
`/gesture` topic as a stop, and `base_driver_node` independently stops on a stale
`/cmd_vel`. Both timeouts default to 0.5 s. This mirrors the watchdog already in
the Arduino firmware.

## Build

Requires **ROS2 Jazzy** on Ubuntu 24.04. There is no supported macOS path for
ROS2, so this half of the project does not run on the Mac the rest of it targets.

```bash
sudo apt install ros-jazzy-vision-msgs ros-jazzy-cv-bridge ros-jazzy-v4l2-camera

cd ros2
colcon build
source install/setup.bash
```

The nodes locate the repo's framework-free modules at runtime via
`GESTURE_BOT_SRC` (default `~/vision_demos`) — set it if your checkout is
elsewhere. The launch file passes it through.

## Run

```bash
# sim, with the camera
ros2 launch gesture_bot_ros gesture_bot.launch.py

# watch the sim robot
ros2 run rqt_image_view rqt_image_view /base_driver_node/render

# drive a real board
ros2 launch gesture_bot_ros gesture_bot.launch.py actuator:=serial serial_port:=/dev/ttyACM0

# with object detection too
ros2 launch gesture_bot_ros gesture_bot.launch.py detector:=true
```

## Tests

The node wrappers are thin by design, so the thing worth testing is the seam
between them and the reused modules — in particular that the tunable-parameter
list has not drifted from `ControllerConfig`. A field renamed in `decision.py`
would otherwise silently stop being exposed as a ROS parameter, with no error
anywhere.

These run **without ROS installed**:

```bash
cd ros2/src/gesture_bot_ros
GESTURE_BOT_SRC=$(git rev-parse --show-toplevel) pytest test/ -v
```

## Status

Written and lint-clean; the ROS-independent tests pass in CI. **The packages have
not yet been built with `colcon` or run on a live graph** — that needs a Jazzy
machine. Expect to fix small things on first build.
