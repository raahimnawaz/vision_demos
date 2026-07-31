"""locate_node -- /image_raw -> /detections, open-vocabulary.

The same contract as ``detector_node``: ``vision_msgs/Detection2DArray`` on the
same topic. The difference is the vocabulary. ``detector_node`` runs YOLO11 over
80 fixed classes; this runs OWLv2 over whatever you put in the ``queries``
parameter. Because the message, the topic and the frame semantics are identical,
swapping which one you launch changes nothing downstream -- that is the point,
and it is why ``OpenVocabDetector`` was written to mirror ``ObjectDetector``'s
interface field-for-field rather than inventing its own.

Rate, and why this node is not a drop-in for the gesture path
------------------------------------------------------------
OWLv2 runs ~2 fps on a GTX 980 Ti (490 ms/frame, fp16) against YOLO11n's ~30.
That is not a tuning problem: OWLv2 letterboxes every input to 960x960, so the
encoder sees ~3,600 patches no matter what the camera is set to, and lowering
the capture resolution buys nothing.

Two consequences are baked in below. The subscription uses depth-1 best-effort
QoS, so while a frame is being processed the newer ones are dropped rather than
queued -- a slow consumer on a sensor stream must never build a backlog, or it
ends up publishing detections for a frame that is seconds stale. And the node is
kept off the gesture control path by construction: ``decision_node`` consumes
``/gesture``, never ``/detections``, so nothing that actuates the robot is
waiting on this.

Live re-querying
----------------
``queries`` is a ROS parameter and can be changed while the node runs. Text
features depend only on the query strings, so they are encoded once and reused
per frame; a parameter change re-encodes them and takes effect on the next
frame. That is most of the per-call cost avoided, and it means you can ask for
something new without a restart -- the same live-tuning property
``decision_node`` has for its gains.
"""

import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from . import _core

_core.ensure_on_path()


class LocateNode(Node):
    def __init__(self):
        super().__init__("locate_node")

        self.declare_parameter("queries", ["person", "cup", "laptop"])
        self.declare_parameter("conf", 0.1)
        self.declare_parameter("device", "")       # "" -> auto (cuda/mps/cpu)
        self.declare_parameter("publish_annotated", False)

        import os
        import sys

        # OpenVocabDetector lives in locateanything/, outside the ROS package,
        # and is imported unchanged -- same arrangement as detector_node pulling
        # ObjectDetector out of object_detection/. Explicit path insert so this
        # node does not depend on anything else having been imported first.
        sys.path.insert(0, os.path.join(_core.ensure_on_path(), "locateanything"))
        from webcam_locate import OpenVocabDetector

        self._OpenVocabDetector = OpenVocabDetector
        queries = list(self.get_parameter("queries").value)
        self.detector = OpenVocabDetector(
            queries,
            conf=float(self.get_parameter("conf").value),
            device=self.get_parameter("device").value or None,
        )
        self.get_logger().info(
            f"OWLv2 on device='{self.detector.device}' dtype={self.detector.dtype} "
            f"queries={self.detector.queries}"
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.bridge = CvBridge()
        self._pub = self.create_publisher(Detection2DArray, "detections", 10)
        self._annotated_pub = None
        if bool(self.get_parameter("publish_annotated").value):
            self._annotated_pub = self.create_publisher(Image, "~/annotated", 1)

        # Depth 1, best effort: drop stale frames rather than queue them. At
        # ~2 fps against a 30 fps publisher this is the difference between
        # "detections for the current frame" and "detections for whatever was in
        # front of the camera several seconds ago".
        qos = QoSProfile(depth=1)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self._sub = self.create_subscription(Image, "image_raw", self._on_image, qos)
        self.get_logger().info("locate_node up: /image_raw -> /detections")

    def _on_set_parameters(self, params):
        """Re-encode text features when the queries change; reject bad input.

        Returning successful=False leaves the previous queries in place, so a
        typo cannot leave the node running with no vocabulary at all.
        """
        for p in params:
            if p.name == "queries":
                queries = [q.strip() for q in list(p.value) if q and q.strip()]
                if not queries:
                    return SetParametersResult(
                        successful=False,
                        reason="queries must contain at least one non-empty string",
                    )
                try:
                    self.detector.queries = queries
                    self.detector._text_inputs = self.detector.processor(
                        text=[queries], return_tensors="pt"
                    ).to(self.detector.device)
                except Exception as exc:  # noqa: BLE001
                    return SetParametersResult(
                        successful=False, reason=f"re-encoding queries failed: {exc}"
                    )
                self.get_logger().info(f"queries -> {queries}")
            elif p.name == "conf":
                self.detector.conf = float(p.value)
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warn(f"cv_bridge conversion failed: {exc}")
            return

        try:
            detections = self.detector.detect(frame)
        except Exception as exc:  # noqa: BLE001 - nor must one bad inference
            self.get_logger().warn(f"inference failed: {exc}")
            return

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
    node = LocateNode()
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
