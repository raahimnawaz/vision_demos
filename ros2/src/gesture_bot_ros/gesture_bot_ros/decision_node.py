"""decision_node -- /gesture -> /cmd_vel.

Wraps ``decision.GestureController`` without reimplementing any of it. The node
adds exactly three things the plain Python loop did not need:

* **ROS parameters.** Every ``ControllerConfig`` field is a live-tunable
  parameter, so the robot can be retuned with ``ros2 param set`` while it drives.
* **Fixed-rate publishing.** ``/cmd_vel`` is republished on a timer even when no
  new gesture arrives, so downstream consumers can implement their own timeout.
* **A dead-man on the input topic.** ``lost_frames`` only counts frames that
  actually arrive. If ``gesture_node`` dies, ``update()`` is never called again
  and the last command would otherwise latch forever. A stale ``/gesture`` topic
  therefore forces a stop.
"""

import rclpy
from geometry_msgs.msg import Twist
from gesture_bot_msgs.msg import Gesture
from rcl_interfaces.msg import SetParametersResult
from rclpy.clock import Clock, ClockType
from rclpy.node import Node

from ._core import INTEGER_PARAMS, TUNABLE, ControllerConfig, GestureController


class DecisionNode(Node):
    def __init__(self):
        super().__init__("decision_node")

        defaults = ControllerConfig()
        for name in TUNABLE:
            self.declare_parameter(name, getattr(defaults, name))

        self.declare_parameter("publish_rate", 30.0)      # Hz, /cmd_vel republish
        self.declare_parameter("gesture_timeout", 0.5)    # s before input is stale

        self.controller = GestureController(self._config_from_params())
        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Staleness is an ELAPSED-time measurement, so it must come off a
        # monotonic clock. The node's default clock is RCL_SYSTEM_TIME, which
        # steps: NTP corrections, VM suspend/resume, and WSL2 in particular
        # (observed jumping ~25,000 s inside a node that had been alive 5,779 s).
        # A forward step fires this dead-man spuriously; a BACKWARD step makes
        # `now - last_msg` negative and silently suppresses it, which is the
        # failure that would matter on a real robot. Message header stamps stay
        # on system time -- those are for cross-machine correlation, not duration.
        self._steady = Clock(clock_type=ClockType.STEADY_TIME)

        self._cmd = (0.0, 0.0)
        self._last_msg = None          # steady-clock seconds of last /gesture
        self._stale_latched = False

        self._pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._sub = self.create_subscription(Gesture, "gesture", self._on_gesture, 10)

        rate = float(self.get_parameter("publish_rate").value)
        self._timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f"decision_node up: publishing /cmd_vel at {rate:.0f} Hz, "
            f"gesture timeout {float(self.get_parameter('gesture_timeout').value):.2f}s"
        )

    # ---------------------------------------------------------------- params
    def _config_from_params(self) -> ControllerConfig:
        values = {name: self.get_parameter(name).value for name in TUNABLE}
        for name in INTEGER_PARAMS:      # counts, not physical quantities
            values[name] = int(values[name])
        return ControllerConfig(**values)

    def _on_set_parameters(self, params):
        """Apply live retuning. Rejects values that would make the loop unsafe."""
        for p in params:
            if p.name in INTEGER_PARAMS and int(p.value) < 1:
                return SetParametersResult(
                    successful=False, reason=f"{p.name} must be >= 1"
                )
            if p.name == "confidence_min" and not 0.0 <= float(p.value) <= 1.0:
                return SetParametersResult(
                    successful=False, reason="confidence_min must be in [0, 1]"
                )
            if p.name in TUNABLE:
                setattr(self.controller.cfg, p.name, p.value)
        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------- loop
    def _now(self) -> float:
        """Monotonic seconds. Never the system clock -- see __init__."""
        return self._steady.now().nanoseconds * 1e-9

    def _on_gesture(self, msg: Gesture):
        self._last_msg = self._now()
        if self._stale_latched:
            self.get_logger().info("/gesture recovered")
            self._stale_latched = False
        # An empty label means "hand visible, no recognized gesture" -- the
        # controller already treats an unknown label as not-valid, so pass it
        # straight through rather than special-casing here.
        self._cmd = self.controller.update(
            msg.gesture, msg.score, msg.hand_present, msg.hand_x
        )

    def _on_timer(self):
        timeout = float(self.get_parameter("gesture_timeout").value)
        now = self._now()

        if self._last_msg is None:
            # Nothing has ever arrived; hold the robot still rather than
            # assuming the pipeline is merely slow to start.
            self._cmd = (0.0, 0.0)
        elif now - self._last_msg > timeout:
            if not self._stale_latched:
                self.get_logger().warn(
                    f"/gesture stale for {now - self._last_msg:.2f}s "
                    f"(> {timeout:.2f}s) -- stopping"
                )
                self._stale_latched = True
            self.controller.reset()
            self._cmd = (0.0, 0.0)

        v, w = self._cmd
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(w)
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DecisionNode()
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
