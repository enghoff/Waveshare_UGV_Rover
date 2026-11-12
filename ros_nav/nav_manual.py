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

**A drive that leaves the map holds it until the rover is back** (`off_map.py`).
`/manual_mode` then stays true after the lease has gone -- the controller asleep
on a balcony is not the rover back in the flat -- and the map is not
trustworthy (`nav_map.map_trustworthy`), so nothing is saved, the world store
records nothing and autonomous runs are refused. It ends when the localizer has
matched well for a while and the drift check's wide search agrees the rover is
where it believes; a refit from the console meanwhile searches that wide window.
Moves the console asks for are not refused: a person may need them to bring the
rover back.
"""

import json
import math
import threading
import time

from geometry_msgs.msg import Twist
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool, String

from manual_gate import clearance, gate
from nav_map import DRIFT_DEG, DRIFT_M, DRIFT_WINDOW_DEG, DRIFT_WINDOW_M
from off_map import LEAVE_STEPS, OffMap

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
        self.off_map = OffMap()
        # What `/manual_mode` last said: the lease, or the map held after it.
        self.manual_held = False
        # Said once at start-up: a bridge restarted under a manual drive must
        # not leave the mapper believing it is still paused for one.
        self.manual_pub.publish(Bool(data=False))
        self.create_subscription(String, "localizer/step", self.on_localizer_step, 50,
                                 callback_group=self.group)
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
            self.get_logger().info("manual driving %s%s" % (
                "began" if on else "ended", " (%s)" % who if who else ""))
            self.manual_hold()
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
                "linear": round(linear, 3), "angular": round(angular, 3),
                "off_map": self.off_map_status()}

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
            self.manual_send(0.0, 0.0)
            self.get_logger().warn("manual driving lapsed: the daemon stopped "
                                   "renewing it for %.0f s" % MANUAL_LEASE_S)
            self.manual_hold()

    def manual_hold(self):
        """Say on `/manual_mode` whether the map is held: while a person drives,
        and after, for as long as the rover is off its map."""
        with self._lock:
            held = self.manual_on or self.off_map.off
            if held == self.manual_held:
                return
            self.manual_held = held
        self.manual_pub.publish(Bool(data=held))

    # --- off the map and back ---------------------------------------------------

    def on_localizer_step(self, msg):
        """One of the localizer's steps, for the off-map latch (off_map.py)."""
        try:
            step = json.loads(msg.data)
            x, y, heading = step["map_base"]
            pose = (float(x), float(y), math.degrees(float(heading)))
            searched, accepted = bool(step["searched"]), bool(step["accepted"])
            score = float(step["score"])
        except (ValueError, KeyError, TypeError):
            return
        with self._lock:
            said = self.off_map.step(self.manual_on, searched, accepted, score, pose,
                                     time.monotonic(), time.time())
            left_at = self.off_map.left_at
        if said == "left":
            self.get_logger().warn(
                "off the map: the localizer could place none of the last %d scans; "
                "the map and the world store are held until the rover is back on "
                "it (last on it at %s)" % (
                    LEAVE_STEPS,
                    "?" if left_at is None else "%.2f, %.2f" % left_at[:2]))
            self.manual_hold()
        elif said == "check":
            threading.Thread(target=self.off_map_confirm, daemon=True,
                             name="off-map-check").start()

    def off_map_confirm(self):
        """The drift check's wide search, asked whether the rover is back where
        it believes it is. Ends the hold only if it is."""
        back, why = False, "the search did not run"
        try:
            with self.map_lock:
                answer, fit, _where = self.map_measure(window_m=DRIFT_WINDOW_M,
                                                       window_deg=DRIFT_WINDOW_DEG)
            if fit is None:
                why = answer.get("why", "nothing to search")
            elif not fit.ok:
                why = "the scan fits nowhere well enough: %s" % fit.why
            elif (abs(answer["moved_m"]) >= DRIFT_M
                    or abs(answer["turned_deg"]) >= DRIFT_DEG):
                why = ("the scan fits %.0f cm and %.0f degrees from the tracked pose"
                       % (100 * answer["moved_m"], answer["turned_deg"]))
            else:
                back = True
                why = "%.0f%% of the scan on the map's walls" % (100 * fit.score)
        except Exception as error:                  # a thread: never past here
            why = "%s: %s" % (type(error).__name__, error)
        with self._lock:
            self.off_map.checked(back)
        if back:
            self.get_logger().info("back on the map (%s); the map and the world "
                                   "store are given back" % why)
            # And the lidar's own look at the next tick rather than in five
            # minutes, as after a refit (nav_map.map_fit_now).
            self._map_drift_at = None
        else:
            self.get_logger().info("still off the map: %s" % why)
        self.manual_hold()

    def off_map_status(self):
        with self._lock:
            return self.off_map.as_dict()
