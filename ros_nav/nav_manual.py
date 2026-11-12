"""Manual driving through the bridge: a person's commands, checked against the scan.

The daemon reads the game controller and sends `{"op": "manual", "on": true,
"linear": m/s, "angular": rad/s}` twenty times a second. Each one is checked here
against the newest `/scan` (`manual_gate.py`) and what survives goes out on
`/cmd_vel`, so it reaches the wheels through `base_node`'s measured drive curves
and its half-second timeout like everything else that drives this rover.

**Manual mode is a lease.** It lasts MANUAL_LEASE_S past the daemon's last word,
so a daemon that dies or loses this connection does not leave the rover in it.
While it lasts, `/manual_mode` (latched) says so, which is what pauses the
mapper (`localizer_node.py`), the change watch (`change_node.py`) and the world
model's scan log (`world_state/visit_model/scan_log.py`); and Nav2 goals are
refused, so nothing but the person drives.
"""

import time

from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from manual_gate import clearance, gate

#: How long manual mode outlives the daemon's last word.
MANUAL_LEASE_S = 2.0


def _round(value):
    return None if value is None else round(value, 3)


class NavManual:
    """Mixed into NavBridge."""

    def manual_setup(self):
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.manual_pub = self.create_publisher(Bool, "manual_mode", latched)
        self.manual_on = False
        self.manual_until = 0.0
        self.manual_who = ""
        self.manual_moving = False
        # Said once at start-up: a bridge restarted under a manual drive must
        # not leave the mapper believing it is still paused for one.
        self.manual_pub.publish(Bool(data=False))
        self.create_timer(0.25, self.manual_lapse, callback_group=self.group)

    def manual(self, on, linear, angular, who=""):
        """One command from the controller, or the end of manual mode."""
        now = time.monotonic()
        with self._lock:
            changed = on != self.manual_on
            self.manual_on = on
            self.manual_until = now + MANUAL_LEASE_S if on else 0.0
            self.manual_who = who if on else ""
            scan, scan_at = self.scan_msg, self.scan_at
        if changed:
            self.manual_pub.publish(Bool(data=on))
            self.get_logger().info("manual driving %s%s" % (
                "began" if on else "ended", " (%s)" % who if who else ""))
        front = rear = age = None
        if scan is not None and scan_at is not None:
            front, rear = clearance(scan.ranges, scan.angle_min, scan.angle_increment,
                                    scan.range_min, scan.range_max)
            age = now - scan_at
        if not on:
            linear = angular = 0.0
        linear, angular, blocked = gate(float(linear), float(angular), front, rear, age)
        self.manual_send(linear, angular)
        return {"manual": on, "front_m": _round(front), "rear_m": _round(rear),
                "scan_age_s": _round(age), "blocked": blocked,
                "linear": round(linear, 3), "angular": round(angular, 3)}

    def manual_send(self, linear, angular):
        if linear or angular:
            twist = Twist()
            twist.linear.x, twist.angular.z = linear, angular
            self.cmd_pub.publish(twist)
            self.manual_moving = True
        elif self.manual_moving:
            # Three times, as `halt` does: cmd_vel is best-effort by the wheels.
            for _ in range(3):
                self.cmd_pub.publish(Twist())
            self.manual_moving = False

    def manual_lapse(self):
        with self._lock:
            lapsed = self.manual_on and time.monotonic() > self.manual_until
            if lapsed:
                self.manual_on = False
                self.manual_who = ""
        if lapsed:
            self.manual_pub.publish(Bool(data=False))
            self.manual_send(0.0, 0.0)
            self.get_logger().warn("manual driving lapsed: the daemon stopped "
                                   "renewing it for %.0f s" % MANUAL_LEASE_S)
