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
                          │       (YOLO11)   (vision_msgs/Detection2DArray)              (sensor_msgs/Image)
                          └──► locate_node   ──/detections──►  (same topic, same type;
                                  (OWLv2)                       run one or the other)
```

## Interfaces

| topic | type | published by |
|---|---|---|
| `/image_raw` | `sensor_msgs/Image` | `v4l2_camera` (external) |
| `/gesture` | `gesture_bot_msgs/Gesture` | `gesture_node` |
| `/detections` | `vision_msgs/Detection2DArray` | `detector_node` *or* `locate_node` |
| `/cmd_vel` | `geometry_msgs/Twist` | `decision_node` |
| `/base_driver_node/render` | `sensor_msgs/Image` | `base_driver_node` |

Only **one** custom message exists — `Gesture`, because ROS has no standard type
for canned hand gestures. Detections use `vision_msgs` and motion uses
`geometry_msgs/Twist`, both standard, so anything else in the ecosystem can
consume this graph.

### Two detectors, one interface

`detector_node` (YOLO11, 80 fixed classes) and `locate_node` (OWLv2,
open-vocabulary) publish the **same message on the same topic**, so swapping
them changes nothing downstream:

```bash
ros2 launch gesture_bot_ros gesture_bot.launch.py detector:=true
ros2 launch gesture_bot_ros gesture_bot.launch.py locate:=true \
    queries:="a red mug, keys, remote control"
ros2 param set /locate_node queries "['scissors','notebook']"   # live, no restart
```

That is only possible because `OpenVocabDetector` was written to mirror
`ObjectDetector` field-for-field — same `Detection`, same
`detect`/`draw`/`process`. Run **one or the other**, never both: they would
each write `/detections`.

They are not interchangeable on rate, though. OWLv2 runs ~2 fps against
YOLO11n's ~30, and it does not improve with a smaller camera image because
OWLv2 letterboxes every input to 960×960 internally. So `locate_node`
subscribes with **depth-1 best-effort QoS** — while a frame is in flight the
newer ones are dropped rather than queued, because a slow consumer that
backlogs a sensor stream ends up publishing detections for a scene that is
seconds stale. Nothing in the control path depends on `/detections`
(`decision_node` reads `/gesture`), so the slow detector cannot stall actuation.

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

Built and run against ROS2 Jazzy. Verified end to end:

- `colcon build` succeeds; `rosidl` generates `gesture_bot_msgs/Gesture`.
- `decision_node` advertises `/cmd_vel` as `geometry_msgs/msg/Twist`.
- A continuous `Open_Palm` stream drives `linear.x` to **0.5** — the debounce
  commits `FORWARD` exactly as the non-ROS loop does.
- Killing the publisher drops `/cmd_vel` to **0.0** and it stays there. The
  dead-man engages and does not flap.

`gesture_node` and `detector_node` were not run: they need mediapipe and
ultralytics, which a `ros-base` install does not carry.

### One real bug this shook out

Both nodes originally measured timeouts with `self.get_clock().now()` — the
node's default **system** clock. Under WSL2 that clock stepped hard: a node alive
5,779 s logged timestamps spanning 25,378 s, and staleness readings came out as
`25196.79s` instead of milliseconds.

That is a genuine defect, not just a WSL artifact. Elapsed-time measurements
belong on a monotonic clock — a system clock steps on NTP correction and on VM
suspend/resume too. The dangerous direction is *backward*: `now - last_msg` goes
negative and the failsafe silently never fires. Both nodes now use
`Clock(clock_type=ClockType.STEADY_TIME)` for durations, keeping system time only
for message header stamps, where it belongs. After the fix, the same environment
reports a single plausible `4.89s`.

### If you run this under WSL2

Two environment fixes were needed, neither of them code problems:

1. **FastDDS shared memory fails** — `RTPS_TRANSPORT_SHM Error: Failed init_port
   fastrtps_port7001`, because `/dev/shm` does not behave as it does on native
   Linux. Publishers appear in `ros2 topic list` but no data flows. Switch RMW:
   ```bash
   sudo apt install ros-jazzy-rmw-cyclonedds-cpp
   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
   ```
2. **Delivery is bursty** — even at `-r 30`, messages arrive roughly every 5 s
   rather than 33 ms, so the dead-man fires between bursts. This is WSL2
   transport/timer behaviour, not the wrapper; it does not reproduce on native
   Linux. Raise `gesture_timeout` if you want a quieter log while testing there.
