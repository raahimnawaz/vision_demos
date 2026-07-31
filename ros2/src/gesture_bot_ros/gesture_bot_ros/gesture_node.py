"""gesture_node -- /image_raw -> /gesture.

Wraps ``perception.GestureSource`` (MediaPipe Gesture Recognizer). Subscribing
to an image topic rather than owning the camera is what makes the stack
recordable: ``ros2 bag record /image_raw`` captures a session that can be
replayed into this node with no webcam and no hand waving.

Image subscriptions use ``SensorDataQoS`` (best-effort, small depth). Reliable
QoS on a video stream will stall the pipeline the moment a subscriber cannot
keep up.

``/image_raw`` arrives in raw camera coordinates, but ``Gesture.hand_x`` is
defined in *display* coordinates, so every frame goes through
``GestureSource.prepare()`` first. This node used to skip that step while
``run_local.py`` did it inline, which inverted ``decision.py``'s steering term
in the ROS2 graph only -- the same controller, imported unchanged, driving the
opposite way. ``mirror:=false`` restores the old behaviour for a camera that is
already mirrored upstream.
"""

import rclpy
from cv_bridge import CvBridge
from gesture_bot_msgs.msg import Gesture
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from . import _core

_core.ensure_on_path()


class GestureNode(Node):
    def __init__(self):
        super().__init__("gesture_node")

        self.declare_parameter("num_hands", 1)
        self.declare_parameter("model_path", "")   # "" -> perception's default
        self.declare_parameter("publish_empty", True)
        self.declare_parameter("mirror", True)     # raw camera -> display coords

        # Imported here, not at module scope: mediapipe and the downloaded
        # .task bundle are only needed by this node, and importing it eagerly
        # would make the whole package unimportable on a machine without them.
        from perception import GestureSource

        model_path = self.get_parameter("model_path").value or None
        mirror = bool(self.get_parameter("mirror").value)
        self.source = GestureSource(
            num_hands=int(self.get_parameter("num_hands").value),
            model_path=model_path,
            mirror=mirror,
        )

        self.bridge = CvBridge()
        self._pub = self.create_publisher(Gesture, "gesture", 10)
        self._sub = self.create_subscription(
            Image, "image_raw", self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f"gesture_node up: /image_raw -> /gesture (mirror={mirror})"
        )

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        # Raw camera coords -> display coords, before anything reads hand_x.
        frame = self.source.prepare(frame)

        obs = self.source.process(frame)
        if not obs.hand_present and not self.get_parameter("publish_empty").value:
            return

        out = Gesture()
        # Carry the camera's stamp/frame through so downstream latency is
        # measurable against the original capture, not against this node.
        out.header = msg.header
        out.gesture = obs.gesture or ""
        out.score = float(obs.score)
        out.hand_present = bool(obs.hand_present)
        out.hand_x = float(obs.hand_x)
        out.hand_y = float(obs.hand_y)
        out.handedness = obs.handedness or ""
        self._pub.publish(out)

    def destroy_node(self):
        try:
            self.source.close()
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GestureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
