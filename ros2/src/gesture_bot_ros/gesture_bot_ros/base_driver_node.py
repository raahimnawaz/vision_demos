"""base_driver_node -- /cmd_vel -> actuator.

Wraps ``actuators.make_actuator``, so the sim, the Arduino and the HID backend
stay interchangeable exactly as they are in the plain Python loop. In ROS terms
this is the base driver: it is the node you would replace with a real robot's
driver (or with ``diff_drive_controller`` from ros2_control) while everything
upstream stays untouched.

Two things matter here beyond calling ``apply()``:

* **``/cmd_vel`` timeout.** Distributed systems fail by going silent. If commands
  stop arriving, a robot that keeps executing the last one drives into a wall,
  so a stale topic latches a zero command.
* **``render()`` is published, not shown.** Calling ``cv2.imshow`` inside a node
  is bad practice -- it needs a GUI thread and can't be recorded. The sim's
  top-down view goes out as ``~/render`` instead, which ``rqt_image_view``
  displays and ``ros2 bag`` captures.
"""

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Image

from . import _core

_core.ensure_on_path()


def _actuator_kwargs(node: Node, kind: str) -> dict:
    """Backend-specific constructor arguments, read from ROS parameters."""
    if kind == "sim":
        return {
            "world_m": float(node.get_parameter("sim_world_m").value),
            "px_per_m": int(node.get_parameter("sim_px_per_m").value),
            "wheel_base": float(node.get_parameter("wheel_base").value),
        }
    if kind == "hid":
        return {"live": bool(node.get_parameter("hid_live").value)}
    if kind == "serial":
        port = node.get_parameter("serial_port").value or None
        return {
            "port": port,
            "baud": int(node.get_parameter("serial_baud").value),
            "wheel_base": float(node.get_parameter("wheel_base").value),
            "dry_run": port is None,
        }
    raise ValueError(f"unknown actuator '{kind}'")


class BaseDriverNode(Node):
    def __init__(self):
        super().__init__("base_driver_node")

        self.declare_parameter("actuator", "sim")          # sim | hid | serial
        self.declare_parameter("rate", 30.0)               # Hz, actuation tick
        self.declare_parameter("cmd_vel_timeout", 0.5)     # s before input is stale
        self.declare_parameter("wheel_base", 0.3)
        self.declare_parameter("publish_render", True)
        self.declare_parameter("sim_world_m", 6.0)
        self.declare_parameter("sim_px_per_m", 90)
        self.declare_parameter("hid_live", False)
        self.declare_parameter("serial_port", "")
        self.declare_parameter("serial_baud", 115200)

        kind = str(self.get_parameter("actuator").value)

        from actuators import make_actuator

        self.actuator = make_actuator(kind, **_actuator_kwargs(self, kind))

        self._cmd = (0.0, 0.0)
        self._last_msg = None
        self._stale_latched = False
        self._last_tick = self._now()

        self._sub = self.create_subscription(Twist, "cmd_vel", self._on_cmd, 10)

        self.bridge = CvBridge()
        self._render_pub = None
        if bool(self.get_parameter("publish_render").value):
            self._render_pub = self.create_publisher(Image, "~/render", 1)

        rate = float(self.get_parameter("rate").value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"base_driver_node up: actuator='{kind}' at {rate:.0f} Hz, "
            f"cmd_vel timeout {float(self.get_parameter('cmd_vel_timeout').value):.2f}s"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg: Twist):
        self._last_msg = self._now()
        if self._stale_latched:
            self.get_logger().info("/cmd_vel recovered")
            self._stale_latched = False
        self._cmd = (msg.linear.x, msg.angular.z)

    def _on_timer(self):
        now = self._now()
        dt = max(now - self._last_tick, 1e-6)
        self._last_tick = now

        timeout = float(self.get_parameter("cmd_vel_timeout").value)
        if self._last_msg is None or now - self._last_msg > timeout:
            if self._last_msg is not None and not self._stale_latched:
                self.get_logger().warn(
                    f"/cmd_vel stale for {now - self._last_msg:.2f}s "
                    f"(> {timeout:.2f}s) -- stopping"
                )
                self._stale_latched = True
            self._cmd = (0.0, 0.0)

        v, w = self._cmd
        self.actuator.apply(v, w, dt)

        if self._render_pub is not None:
            frame = self.actuator.render()
            if frame is not None:
                img = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img.header.stamp = self.get_clock().now().to_msg()
                img.header.frame_id = "base_link"
                self._render_pub.publish(img)

    def destroy_node(self):
        # SerialServo.close() writes a 1500,1500 neutral frame -- the actuator
        # must be stopped before the port drops, not after.
        try:
            self.actuator.close()
        except Exception as exc:  # noqa: BLE001 - never block shutdown
            self.get_logger().warn(f"actuator close failed: {exc}")
        finally:
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseDriverNode()
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
