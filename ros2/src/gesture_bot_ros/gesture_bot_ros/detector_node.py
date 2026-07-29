"""detector_node -- /image_raw -> /detections.

Publishes ``vision_msgs/Detection2DArray``, the ROS standard for 2D object
detection, rather than a custom message. That is what makes the output usable
by anything else in the ecosystem instead of only by this project.

Split from ``gesture_node`` on purpose: YOLO and MediaPipe are separate models
at separate rates, and in one node they would block each other. Separate nodes
also mean the detector can simply not be launched when it isn't wanted, which
is what the ``--objects`` flag does in the plain Python loop.

NOTE: targets ``vision_msgs`` 4.x (Humble/Jazzy), where ``BoundingBox2D.center``
is a ``vision_msgs/Pose2D`` with a nested ``position``. On older releases
``center`` was a ``geometry_msgs/Pose2D`` with flat ``x``/``y`` fields.
"""

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from . import _core

_core.ensure_on_path()


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector_node")

        self.declare_parameter("conf", 0.4)
        self.declare_parameter("imgsz", 512)
        self.declare_parameter("device", "")       # "" -> auto (cuda/mps/cpu)
        self.declare_parameter("publish_annotated", False)

        import os
        import sys
        from types import SimpleNamespace

        # ObjectDetector lives in object_detection/, which perception.py also
        # adds to sys.path; do it explicitly here so this node does not depend
        # on perception (and therefore on mediapipe) being imported first.
        sys.path.insert(0, os.path.join(_core.ensure_on_path(), "object_detection"))
        from vision_demo import ObjectDetector

        self.detector = ObjectDetector(
            SimpleNamespace(
                conf=float(self.get_parameter("conf").value),
                imgsz=int(self.get_parameter("imgsz").value),
                device=self.get_parameter("device").value or None,
            )
        )
        self.get_logger().info(f"YOLO running on device='{self.detector.device}'")

        self.bridge = CvBridge()
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._annotated_pub = None
        if bool(self.get_parameter("publish_annotated").value):
            self._annotated_pub = self.create_publisher(Image, "~/annotated", 1)

        self._sub = self.create_subscription(
            Image, "image_raw", self._on_image, qos_profile_sensor_data
        )
        self.get_logger().info("detector_node up: /image_raw -> /detections")

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        detections = self.detector.detect(frame)

        out = Detection2DArray()
        out.header = msg.header
        for d in detections:
            det = Detection2D()
            det.header = msg.header
            det.id = d.label

            bbox = BoundingBox2D()
            bbox.center.position.x = float((d.x1 + d.x2) / 2.0)
            bbox.center.position.y = float((d.y1 + d.y2) / 2.0)
            bbox.center.theta = 0.0
            bbox.size_x = float(abs(d.x2 - d.x1))
            bbox.size_y = float(abs(d.y2 - d.y1))
            det.bbox = bbox

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = d.label
            hyp.hypothesis.score = float(d.conf)
            det.results.append(hyp)

            out.detections.append(det)
        self._pub.publish(out)

        if self._annotated_pub is not None:
            annotated = self.detector.draw(frame, detections)
            img = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            img.header = msg.header
            self._annotated_pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
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
