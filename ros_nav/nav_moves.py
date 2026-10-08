#!/usr/bin/env python3
"""Asking Nav2 to move, and deciding what its answer means.

One shape runs through all of it: send a goal, wait for it with a time allowance
built from what the route actually costs, and turn whatever comes back into a
sentence and a reason code the daemon can hand to a person. The allowance is the
part worth reading -- a goal that is refused instantly and a goal that is still
being driven look identical from outside until it expires.

A mixin rather than a module of functions because every one of these needs the
node: its clock, its action clients and its idea of where the rover is. It is
mixed into `NavBridge` in nav_bridge.py, which is the only thing that
instantiates it.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, PoseStamped
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose, Spin
from nav2_msgs.srv import GetCostmap
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters

# Beside this file and with no ROS in them, for the reasons nav_bridge.py gives:
# the phrases, the geometry and what a route costs are each one function shared
# with the checks, not a copy of one.
import goal_fit
import route_cost
import autonomy_guard
import frontier
from nav_codes import START_OCCUPIED, phrase_for, reason_for
from nav_limits import (
    COSTMAP_TIMEOUT_S, DEFAULT_SPEED_MS, DEFAULT_TURN_DPS, PROGRESS_S,
    REVERSE_LIMIT_M, ROUTE_TURN_DPS, TIME_ALLOWANCE_FLOOR_S,
    TIME_ALLOWANCE_MIN_ROUTE_S, TIME_ALLOWANCE_SLACK, UNWEDGE_MOVED_M, duration,
    wrap,
)

#: **A goal nearer than this that must arrive facing a given way is driven as
#: a turn, a straight line and a turn**, because the lattice planner will not
#: turn on the spot when a loop is cheaper, and close to a goal the controller
#: will not drive the loop. Only a goal with a heading: a console click carries
#: none and has not been seen to swing, so it is left to Nav2 as it always was;
#: an autonomous look carries the bearing to its thing. Measured on the
#: rover on 2026-10-07 from where M3 session 7 rocked for 72 s: a goal 0.5 m
#: away facing -70 deg planned as a 3.3 to 4.0 m loop from every start heading
#: between 130 and 173 deg, and the real controller, given that loop, preferred
#: turning on the spot to driving it by 45 points to 67 (dwb_bench.py), since
#: the loop sets off away from the goal. Turned to face the goal first, the same
#: goal planned as 0.48 m straight. 1.5 m covers every short goal that has
#: planned a route round since the sessions began (0.22 to 1.2 m).
NEAR_GOAL_M = 1.5

#: How nearly the rover must face a near goal before driving to it. Measured
#: the same day: facing within 15 to 30 deg of the goal it planned straight, and
#: at 45 deg off it looped again. 20 leaves room for a turn that lands short.
FACE_WITHIN_DEG = 20.0

#: Turns allowed to face a near goal: one, and one more to correct it, because
#: a turn is counted by the gyro and the gyro has been measured 7-9% out on a
#: large turn (2026-10-01, 2026-10-07).
FACE_TURNS = 2

#: The heading asked for is turned to after arriving only when it is further
#: off than the goal checker's own tolerance (yaw_goal_tolerance, 0.26 rad):
#: anything closer Nav2 would have called arrived anyway. Measured the same
#: day: facing the goal, a final heading 11.5 deg off planned straight and one
#: 40 deg off looped, so the heading is never handed to the planner here.
FINAL_TURN_DEG = 15.0

#: Closer than this the rover is already there as far as the goal checker is
#: concerned (xy_goal_tolerance), so a near goal is only its heading.
THERE_M = 0.22

#: **A near goal whose straight line is blocked is waited for, not driven
#: round.** Facing it, the planner is asked for the route; longer than the
#: straight line by more than this, something is in the way -- on 2026-10-07 the
#: owner, standing 0.5 m in front -- and the route round it is the kind of loop
#: the controller swings on the spot over (3.6 m for a goal 0.7 m away; the
#: owner called the swinging erratic and unnecessary). The same slack the
#: identity trial's path check allows.
DETOUR_SLACK_M = 0.5

#: How long a blocked near goal waits, still, for the way to clear, and how
#: often it asks the planner again. A person stepping aside takes a few
#: seconds; a chair does not move. After this the rover goes round in straight
#: legs (`round_by_legs`), or hands the goal back as blocked if it cannot.
BLOCKED_WAIT_S = 3.0      # the owner's choice, 2026-10-07 (was 10, then 6)
BLOCKED_ASK_S = 1.0

#: Going round something in the way of a near goal: at most this many straight
#: legs, and a route at most this much longer than the straight line. Past
#: either, the goal is handed back as blocked.
ROUND_LEGS = 4
ROUND_EXTRA_M = 3.0

#: **A near goal looks for what is in its way on the live scan, not the map.**
#: Until 2026-10-08 the planner's costmap was slam_toolbox's map alone, which a
#: person standing in front of a still rover is never drawn into, and it has
#: the live scan within 3 m now (config/nav2.yaml, `live_layer`) only when that
#: layer is switched on. In M3 session 13 (2026-10-07) the planner drew a 1.0 m
#: goal straight through the owner, the controller -- which sees them --
#: refused every move forward, and the rover swung on the spot for 25 s until
#: the stall watch ended it. A near goal is still driven this way rather than
#: by the planner, live layer or not. So the
#: way is read on the local costmap: blocked when the body would sweep a cell
#: the scan hit, bar the first `LIVE_SKIP_M`. A way round keeps the centre
#: `ROUND_CLEAR_M` from them -- the footprint's furthest corner, 0.244 m, and
#: 6 cm to spare -- or its half-width within `ROUND_RELAX_M` of either end.
LIVE_SKIP_M = 0.10
ROUND_CLEAR_M = 0.30
ROUND_TIGHT_M = 0.14
ROUND_RELAX_M = 0.30

#: How often a near goal's drive looks at the live scan for something stepping
#: into its way once it has set off, and how many times one goal may be
#: stopped for that before it is handed back.
LIVE_LOOK_S = 1.0
NEAR_TRIES = 3

#: How every near-goal sentence about something in the way begins, which is
#: how the caller tells it from any other refusal.
IN_THE_WAY = "something is in the way"

#: **A longer goal watches the route ahead on the live scan too.** In M4
#: session 4 (2026-10-08) the owner stood in the way of a 1.8 m goal, further
#: than `NEAR_GOAL_M`, so it was one Nav2 goal and nothing looked at the live
#: scan: the planner sent the same path through them every second for 27 s and
#: the controller turned on the spot until they moved. So every second the next
#: `ROUTE_LOOK_M` of the route Nav2 is following is checked as a near goal's
#: straight line is; if the scan has something on it, the goal is stopped and
#: the rover goes to a point `PAST_IT_M` further along the route as a near goal
#: -- which waits for the way, then goes round in straight legs -- and then on
#: to the goal, at most `NEAR_TRIES` times.
ROUTE_LOOK_M = 1.0
PAST_IT_M = 1.4

#: A cell of slam_toolbox's map at this or above is a wall: the planner's
#: static layer's own `lethal_cost_threshold`, left at Nav2's default.
MAP_OCCUPIED = 100

#: **Someone where there is no way round them is waited for, not spun at.**
#: With the planner's live layer, a person standing in a doorway leaves the
#: planner no route at all, and Nav2 answers that with its recoveries. On
#: 2026-10-08 the owner stood in a corridor's 1.0 m mouth 2 m from the rover:
#: every plan failed after about 2.5 s, each failure cleared the costmaps, and
#: from 11 s it spun 225 degrees, waited, reversed 0.35 m and spun again, until
#: they stepped aside. The route watch, looking 1 m ahead, never came near them.
#: So when Nav2 starts a recovery and the planner has sent no route for this
#: goal in `PLAN_STALE_S` -- the planner failing rather than the controller,
#: which replans every second while it struggles -- the goal is stopped there,
#: and the rover holds still asking the planner itself for `BLOCKED_WAIT_S`, then
#: drives on or hands the goal back as blocked (`wait_for_a_route`).
PLAN_STALE_S = 2.0
NO_WAY = "there is no way past"
#: nav2_msgs ComputePathToPose NO_VALID_PATH (nav_codes.py): the one refusal
#: that waiting can mend.
NO_VALID_PATH = 208

#: How far past `goal_fit.REACH_M` a spot's refusal looks for something only the
#: scan has: the body's furthest corner, 0.244 m, rounded up.
FIT_BODY_M = 0.25


def route_ahead(route, here, metres):
    """The route from the point on it nearest `here` on for `metres`, as a list
    of (x, y) starting at `here`; [] when there is no route."""
    if not route or here is None:
        return []
    nearest = min(range(len(route)),
                  key=lambda i: math.hypot(route[i][0] - here[0], route[i][1] - here[1]))
    out = [(float(here[0]), float(here[1]))]
    left = float(metres)
    for point in route[nearest + 1:] if nearest + 1 < len(route) else route[nearest:]:
        last = out[-1]
        step = math.hypot(point[0] - last[0], point[1] - last[1])
        if step <= 1e-9:
            continue
        if step >= left:
            share = left / step
            out.append((last[0] + (point[0] - last[0]) * share,
                        last[1] + (point[1] - last[1]) * share))
            return out
        out.append((float(point[0]), float(point[1])))
        left -= step
    return out


def to_odom(correction, point):
    """A point in the map frame, in the odom frame. `correction` is `map ->
    odom` as `(x, y, yaw)`, as `correction()` reads it."""
    dx, dy = point[0] - correction[0], point[1] - correction[1]
    cos_yaw, sin_yaw = math.cos(correction[2]), math.sin(correction[2])
    return (cos_yaw * dx + sin_yaw * dy, -sin_yaw * dx + cos_yaw * dy)


def to_map(correction, point):
    """A point in the odom frame, in the map frame: `to_odom` undone."""
    cos_yaw, sin_yaw = math.cos(correction[2]), math.sin(correction[2])
    return (correction[0] + cos_yaw * point[0] - sin_yaw * point[1],
            correction[1] + sin_yaw * point[0] + cos_yaw * point[1])


class NavMoves:
    """The half of `NavBridge` that asks Nav2 to move the rover."""

    def wait(self, future, limit_s):
        """Wait for a future without spinning: the executor is already doing that.

        `spin_until_future_complete` is the usual answer and is wrong here. This
        runs on a connection thread, not on the executor's, and calling spin from
        two threads at once is how rclpy deadlocks. The executor services the
        future; this only has to notice.
        """
        deadline = time.monotonic() + limit_s
        while not future.done():
            if time.monotonic() > deadline:
                return False
            time.sleep(0.02)
        return True

    def run_goal(self, kind, goal_msg, limit_s, say, measure, motion="driving",
                 budget=None, give_up=None, guard=None):
        """Send one Nav2 goal and narrate it until it ends.

        `say` publishes a progress line and `measure` turns the action's own
        feedback into the numbers the daemon reports, because each action counts
        something different -- degrees for a spin, metres for a drive, metres
        remaining for a navigation.

        `budget`, where there is one, is asked on every pass how many seconds the
        move now deserves, and the deadline moves out to match. `drive` and
        `turn_in_place` do not need it -- what they were asked for is what they
        will do -- but a navigation does: the route is not known when the goal is
        sent, and it is the route rather than the goal that has to be driven.

        `motion` is the word for what the rover is doing once the goal is
        accepted, and it is a parameter because the consoles read it: both turn
        the phase into a sentence, and a spin narrating itself as "driving +45
        deg" is a rover describing something it is not doing.

        `give_up` is asked, every pass, whether this goal is worth continuing,
        and a sentence back from it cancels the goal and becomes the reason. It
        is the opposite of `budget`, which can only ever push the deadline out.
        `goto` always passes one: `explore` its own, everyone else
        `frontier.Stall`, which leaves Nav2's recoveries alone and ends only a
        goal that has gone nowhere with nothing being attempted. See
        `frontier.Stall` for what it watches and why Nav2 cannot see it.
        """
        client = self.actions[kind]
        if not client.wait_for_server(timeout_sec=2.0):
            return {"reason": "refused", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 is not running, so the rover will not drive "
                              "itself. Only the mapping half of the stack is up."}
        guard_pose = self.pose() if guard is not None else None
        with self._lock:
            blocked = autonomy_guard.refusal(guard, self.stop_seq, pose=guard_pose)
            if blocked:
                return {"reason": "stopped", "travelled_m": 0.0,
                        "turned_deg": 0.0, "detail": blocked}
            if self.estop:
                return {"reason": "blocked", "travelled_m": 0.0,
                        "turned_deg": 0.0,
                        "detail": "the stop is latched; clear it first"}
            self.cancelled = False

        # Dead reckoning, not the map frame: see dead_reckoned() and
        # finish() for the 19 degrees that cost.
        started = self.dead_reckoned()
        feedback = {}
        recoveries = [0]

        def on_feedback(message):
            fields = measure(message.feedback)
            # Kept outside `feedback`, which is overwritten each time: the count
            # only matters once the move has failed, and by then Nav2's last
            # feedback may have reset it.
            recoveries[0] = max(recoveries[0], int(fields.get("recoveries") or 0))
            feedback.update(fields)

        say("planning", "the goal is with Nav2")
        with self._lock:
            if guard is not None and guard.get("stop_seq") != self.stop_seq:
                return {"reason": "stopped", "travelled_m": 0.0,
                        "turned_deg": 0.0, "detail": "a stop invalidated this goal"}
            send = client.send_goal_async(goal_msg, feedback_callback=on_feedback)
        if not self.wait(send, 10.0):
            return {"reason": "failed", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 did not answer the goal in ten seconds"}
        handle = send.result()
        if handle is None or not handle.accepted:
            return {"reason": "refused", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 would not accept the goal, which usually means "
                              "the rover is standing inside something the costmap "
                              "believes in"}

        with self._lock:
            self.active_goal = handle
            self.driving = True
            # A stop can land while Nav2 is accepting the goal, before there is
            # a handle for halt() to cancel. Cancel that handle as soon as it exists.
            stopped_during_send = (guard is not None and
                                  guard.get("stop_seq") != self.stop_seq)
        if stopped_during_send:
            handle.cancel_goal_async()
        abandoned = None
        try:
            result_future = handle.get_result_async()
            began = time.monotonic()
            deadline = began + limit_s
            said_at = 0.0
            while not result_future.done():
                now = time.monotonic()
                if give_up is not None:
                    abandoned = give_up(now, dict(feedback))
                    if abandoned:
                        handle.cancel_goal_async()
                        self.wait(result_future, 5.0)
                        break
                if budget is not None:
                    # Re-asked every pass rather than once at the start, because
                    # the route does not exist yet when the goal is sent and it
                    # changes at every replan. Only ever pushed outwards: a
                    # replan that happens to come back shorter must not pull the
                    # deadline back past where the rover has already got to.
                    deadline = max(deadline, began + budget())
                if now > deadline:
                    handle.cancel_goal_async()
                    self.wait(result_future, 5.0)
                    break
                if now - said_at > PROGRESS_S:
                    said_at = now
                    say(motion, "", **dict(feedback))
                time.sleep(0.05)
            outcome = self.finish(result_future, started, feedback)
            # A goal this file gave up on is not one that ran out of time, and
            # `finish` cannot tell them apart -- it sees a cancelled goal either
            # way, and would report the time allowance running out on a move that
            # had most of it left. Said in its own words instead.
            if abandoned:
                outcome["reason"] = "blocked"
                outcome["detail"] = abandoned
            # What it tried before giving up. A bare "blocked" sends somebody to
            # look at the rover; "blocked after 10 recoveries, and the planner
            # could not find a route" sends them to look at the map, which is
            # where the answer is.
            if recoveries[0] and outcome.get("reason") != "arrived":
                outcome["detail"] = (
                    "%s -- Nav2 gave up after %d recovery attempt%s"
                    % (outcome.get("detail") or "no route",
                       recoveries[0], "" if recoveries[0] == 1 else "s"))
        finally:
            with self._lock:
                self.active_goal = None
                self.driving = False
                self.remaining_m = None
        return outcome

    def finish(self, result_future, started, feedback):
        """What the move did, measured against dead reckoning.

        `started` is an odom-frame pose -- see `dead_reckoned` for why it must not
        be a map-frame one. Two refinements on top of the plain difference, and
        both matter:

        A wrapped heading difference cannot tell 200 degrees from -160, so where
        the behaviour has been counting rotation of its own -- `Spin` does -- its
        accumulating figure wins whenever it is the larger of the two. And a
        straight drive is credited with the distance `DriveOnHeading` measured
        rather than the straight line between its ends, because a rover that
        wandered a little covers more ground than the chord between where it
        started and where it stopped.
        """
        travelled = turned = 0.0
        ended = self.dead_reckoned()
        if started is not None and ended is not None:
            travelled = math.hypot(ended[0] - started[0], ended[1] - started[1])
            turned = math.degrees(wrap(ended[2] - started[2]))
        if "turned_deg" in feedback and abs(feedback["turned_deg"]) > abs(turned):
            turned = feedback["turned_deg"]
        if "travelled_m" in feedback and feedback["travelled_m"] > travelled:
            travelled = feedback["travelled_m"]

        with self._lock:
            cancelled = self.cancelled
        if not result_future.done():
            return {"reason": "timed out", "travelled_m": travelled,
                    "turned_deg": turned,
                    "detail": "Nav2 was still working when the time allowance ran "
                              "out, and the goal was cancelled"}
        wrapped = result_future.result()
        status = getattr(wrapped, "status", None)
        result = getattr(wrapped, "result", None)
        code = getattr(result, "error_code", 0) or 0
        message = (getattr(result, "error_msg", "") or "").strip()

        if status == GoalStatus.STATUS_SUCCEEDED and not code:
            return {"reason": "arrived", "travelled_m": travelled,
                    "turned_deg": turned}
        if status == GoalStatus.STATUS_CANCELED:
            return {"reason": "stopped" if cancelled else "timed out",
                    "travelled_m": travelled, "turned_deg": turned,
                    "detail": "a stop was asked for" if cancelled
                              else "the time allowance ran out"}
        # Code 0 is NONE, and NONE only means "arrived" beside a SUCCEEDED
        # status, which the branch above has already taken. Down here the goal
        # was aborted, and an abort that carries no code is one `bt_navigator`
        # ended without filling in a reason -- which it does when a server under
        # it stops answering. Reading the table for 0 here turned exactly that
        # into "arrived", so a rover that gave up 0.7 m into a 1.5 m drive
        # reported success, twice, while somebody was trying to work out why it
        # was not driving properly.
        if not code:
            return {"reason": "failed", "travelled_m": travelled,
                    "turned_deg": turned,
                    "detail": message or ("Nav2 abandoned the goal without saying "
                                          "why, which usually means a server under "
                                          "it stopped answering in time")}
        return {"reason": reason_for(code), "code": int(code),
                "travelled_m": travelled, "turned_deg": turned,
                "detail": (phrase_for(code, message)
                           or "Nav2 gave up without saying why (code %s)" % code)}

    def drive(self, distance_m, speed_ms, say, guard=None):
        """Straight ahead or straight back, and stop rather than hit anything.

        `DriveOnHeading` and `BackUp` are the same behaviour in two directions,
        and neither steers: they drive the heading they were given and abort with
        COLLISION_AHEAD when the costmap says the footprint would hit something.
        That is a narrower promise than the old `drive` made -- it used to weave
        around obstacles -- and the honest place to want weaving is `drive_to`,
        which has a planner behind it.
        """
        speed = abs(speed_ms or DEFAULT_SPEED_MS)
        reach = abs(distance_m)
        if distance_m < -REVERSE_LIMIT_M:
            return self.reverse_by_turning(reach, speed, say)
        limit = max(TIME_ALLOWANCE_FLOOR_S,
                    TIME_ALLOWANCE_SLACK * reach / max(speed, 0.05))
        if distance_m >= 0:
            goal = DriveOnHeading.Goal()
            kind = "forward"
        else:
            goal = BackUp.Goal()
            kind = "back"
        goal.target = Point(x=reach, y=0.0, z=0.0)
        goal.speed = float(speed)
        goal.time_allowance = duration(limit)
        return self.run_goal(
            kind, goal, limit + 5.0, say,
            lambda fb: {"travelled_m": round(abs(fb.distance_traveled), 3)},
            guard=guard)

    def reverse_by_turning(self, reach, speed, say):
        """A long way backwards, driven forwards, because the lidar faces one way.

        The rover sees with a lidar bolted on looking ahead of it, so anything
        behind it is unmapped and unwatched, and `BackUp` will drive into it at
        full speed reporting nothing wrong -- its collision check reads the same
        costmap, and the costmap behind the rover is whatever was there when it
        last faced that way. A short reverse is fine on those terms because the
        rover was looking at that ground moments ago; REVERSE_LIMIT_M is where
        that stops being true.

        So this turns round and drives forwards, which covers the same ground
        with the sensor pointed at it. The rover ends up facing the other way,
        which is the honest cost of the manoeuvre and is why the reply says so.
        """
        about = self.turn(180.0, say)
        if about.get("reason") != "arrived":
            about["detail"] = (
                "%s -- the rover was turning round first, because %0.1f m is "
                "further than it will reverse blind"
                % (about.get("detail") or "the turn did not finish", reach))
            return about
        onward = self.drive(reach, speed, say)
        onward["turned_deg"] = round(
            (about.get("turned_deg") or 0.0) + (onward.get("turned_deg") or 0.0),
            1)
        onward["detail"] = (
            "%s -- " % onward["detail"] if onward.get("detail") else "") + (
            "the rover turned round and drove forwards rather than reversing "
            "%0.1f m blind, so it is now facing the other way" % reach)
        return onward

    def turn(self, angle_deg, say, guard=None):
        """On the spot, by `Spin`, which is collision-checked like everything else.

        Not refused when the rover is boxed in, unlike a navigation goal: rotating
        is how something that has got too close to a wall gets away from it, and
        Nav2's spin only aborts if the rotation itself would sweep through an
        obstacle.
        """
        limit = max(TIME_ALLOWANCE_FLOOR_S,
                    TIME_ALLOWANCE_SLACK * abs(angle_deg) / DEFAULT_TURN_DPS)
        goal = Spin.Goal()
        goal.target_yaw = float(math.radians(angle_deg))
        goal.time_allowance = duration(limit)
        return self.run_goal(
            "spin", goal, limit + 5.0, say,
            lambda fb: {"turned_deg": round(
                math.copysign(math.degrees(abs(fb.angular_distance_traveled)),
                              angle_deg), 1)},
            motion="turning", guard=guard)

    def footprint(self):
        """The body outline the costmap node is configured with, asked for once.

        A parameter query rather than a constant in this file, because the
        footprint is a measurement of the rover -- `lidar_slam/slam2d.c` has the
        same rectangle -- and somebody re-measuring it in config/nav2.yaml should
        not have to know that a second copy exists here. Cached after the first
        answer: costmap footprints do not change while a node is running.
        """
        if self.body is not None:
            return self.body
        if not self.footprint_client.wait_for_service(timeout_sec=1.0):
            return None
        request = GetParameters.Request()
        request.names = ["footprint", "robot_radius"]
        future = self.footprint_client.call_async(request)
        if not self.wait(future, COSTMAP_TIMEOUT_S):
            return None
        answer = future.result()
        if answer is None or len(answer.values) < 2:
            return None
        self.inscribed_m = goal_fit.inscribed_radius(
            answer.values[0].string_value, answer.values[1].double_value)
        self.body = goal_fit.polygon_from(answer.values[0].string_value,
                                          answer.values[1].double_value)
        return self.body

    def walking_body(self):
        """What a walk over the occupancy grid needs in order to agree with Nav2.

        Sent with the map (`nav_bridge.grid`) for the autonomy executive, which
        decides whether a place can be reached by walking the grid and has no
        planner to ask. Walked as a point, that walk went through gaps the
        planner refuses: every frontier an autonomous run chose before 2026-10-05
        was in a pocket reached through a 30-40 cm gap, and every one failed --
        twice by driving off on a 38 m way round. Two numbers close it, both the
        bridge's own: the clearance the planner keeps from walls, and how far
        `fit_goal` will move a goal onto floor where the body fits.

        Empty when the costmap node has not said what the body is. The ready
        check keeps a map request from waiting on a costmap that is not up, and
        an empty answer leaves the walk as it was rather than guessing a body.
        """
        if self.body is None and not self.footprint_client.service_is_ready():
            return {}
        if self.footprint() is None or self.inscribed_m is None:
            return {}
        return {"inscribed_radius_m": round(self.inscribed_m, 4),
                "goal_fit_reach_m": goal_fit.REACH_M}

    def costmap(self):
        """The global costmap as the planner currently holds it, or None.

        `GetCostmap` rather than the published topic on purpose: the topic sends
        one full grid and then deltas, so a subscriber that joined late or missed
        an update holds something subtly wrong, and subtly wrong is the failure
        this whole check exists to catch.
        """
        if not self.costmap_client.wait_for_service(timeout_sec=1.0):
            return None
        future = self.costmap_client.call_async(GetCostmap.Request())
        if not self.wait(future, COSTMAP_TIMEOUT_S):
            return None
        answer = future.result()
        if answer is None:
            return None
        grid = answer.map
        return goal_fit.CostGrid(grid.metadata.size_x, grid.metadata.size_y,
                                 grid.metadata.resolution,
                                 grid.metadata.origin.position.x,
                                 grid.metadata.origin.position.y,
                                 bytes(bytearray(grid.data)))

    def mapped_walls(self):
        """The SLAM map's walls as a costmap, or None before there is a map:
        lethal where the map is occupied and nothing anywhere else, which is
        what the planner's static layer makes of the same grid. Not the
        planner's costmap itself, which has the live layer's marks on it too.

        Converted once per map, not per question: the map changes every few
        seconds while the rover is looking at once a second.
        """
        with self._lock:
            msg = getattr(self, "map_msg", None)
            kept = getattr(self, "_walls", None)
        if msg is None:
            return None
        if kept is not None and kept[0] is msg:
            return kept[1]
        info = msg.info
        grid = goal_fit.CostGrid(
            info.width, info.height, info.resolution, info.origin.position.x,
            info.origin.position.y,
            bytes(goal_fit.LETHAL if v >= MAP_OCCUPIED else 0 for v in msg.data))
        with self._lock:
            self._walls = (msg, grid)
        return grid

    def live_costmap(self):
        """The local costmap -- the live scan round the rover, in the odom
        frame -- and the `map -> odom` correction to read it by, or None.

        The one costmap a person standing in the way is on; see `LIVE_SKIP_M`.
        """
        client = getattr(self, "live_client", None)
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            return None
        correction = self.correction()
        if correction is None:
            return None
        future = client.call_async(GetCostmap.Request())
        if not self.wait(future, COSTMAP_TIMEOUT_S):
            return None
        answer = future.result()
        if answer is None:
            return None
        grid = answer.map
        return (goal_fit.CostGrid(grid.metadata.size_x, grid.metadata.size_y,
                                  grid.metadata.resolution,
                                  grid.metadata.origin.position.x,
                                  grid.metadata.origin.position.y,
                                  bytes(bytearray(grid.data))), correction)

    def seen_in_the_way(self, goal, live=None):
        """A sentence when the live scan has something on the straight line
        from the rover to `goal`, or "". Also "" when there is no live costmap,
        pose or body to ask, which leaves a near goal to the planner as before.
        """
        live = live or self.live_costmap()
        here = self.pose()
        body = self.footprint() if live is not None else None
        if live is None or here is None or body is None:
            return ""
        grid, correction = live
        if goal_fit.line_fits(grid, body, to_odom(correction, here),
                              to_odom(correction, goal), worst=goal_fit.LETHAL,
                              skip_m=LIVE_SKIP_M):
            return ""
        return ("%s: the scan has something on the straight line to a goal "
                "%.1f m away" % (IN_THE_WAY, math.hypot(goal[0] - here[0],
                                                       goal[1] - here[1])))

    def near_watch(self, goal, give_up=None):
        """The give-up for a near goal's straight drive. Something stepping
        into the way, looked for on the live scan every `LIVE_LOOK_S`, ends it;
        so does the caller's own give-up, or the stall watch every goal has.
        """
        if give_up is None:
            stall = frontier.Stall()

            def give_up(now, feedback):
                return stall.update(now, self.pose(),
                                    int(feedback.get("recoveries") or 0))

        looked = [None]

        def watch(now, feedback):
            if looked[0] is None or now - looked[0] >= LIVE_LOOK_S:
                looked[0] = now
                seen = self.seen_in_the_way(goal)
                if seen:
                    return seen
            return give_up(now, feedback)

        return watch

    def live_layer(self, enabled=None):
        """The planner's live obstacle layer's switch, set first when `enabled`
        is given (config/nav2.yaml, `live_layer`). The run-time way to take the
        layer back out: switched off it clears its marks on the next update."""
        name = "live_layer.enabled"
        if enabled is not None:
            client = getattr(self, "layer_set_client", None)
            if client is None or not client.wait_for_service(timeout_sec=1.0):
                return {"ok": False, "error": "the planner's costmap is not answering"}
            value = ParameterValue()
            value.type = ParameterType.PARAMETER_BOOL
            value.bool_value = bool(enabled)
            request = SetParameters.Request()
            request.parameters = [Parameter(name=name, value=value)]
            future = client.call_async(request)
            if not self.wait(future, COSTMAP_TIMEOUT_S) or future.result() is None:
                return {"ok": False, "error": "the planner's costmap did not answer"}
            result = future.result().results[0]
            if not result.successful:
                return {"ok": False, "error": result.reason or "the costmap refused it"}
        if not self.footprint_client.wait_for_service(timeout_sec=1.0):
            return {"ok": False, "error": "the planner's costmap is not answering"}
        request = GetParameters.Request()
        request.names = [name]
        future = self.footprint_client.call_async(request)
        if (not self.wait(future, COSTMAP_TIMEOUT_S) or future.result() is None
                or not future.result().values):
            return {"ok": False, "error": "the planner's costmap did not answer"}
        value = future.result().values[0]
        if value.type != ParameterType.PARAMETER_BOOL:
            return {"ok": True, "enabled": None,
                    "note": "the planner's costmap has no live layer"}
        return {"ok": True, "enabled": bool(value.bool_value)}

    def no_way_watch(self, give_up):
        """`give_up`, and a `NO_WAY` sentence when Nav2 starts a recovery while
        the planner has sent no route for this goal in `PLAN_STALE_S`.

        Asks nothing of the planner itself: a request while Nav2's goal runs
        would take the planner server from the behaviour tree's own and fail
        that, which reads to the tree as one more failure to recover from.
        """
        began = [None]
        seen = [0]

        def watch(now, feedback):
            if began[0] is None:
                began[0] = now
            count = int(feedback.get("recoveries") or 0)
            if count > seen[0]:
                seen[0] = count
                with self._lock:
                    at = getattr(self, "plan_at", None)
                if at is None or at < began[0] or now - at > PLAN_STALE_S:
                    return ("%s: Nav2 began recovering with no route from the "
                            "planner" % NO_WAY)
            return give_up(now, feedback) if give_up else ""

        return watch

    def wait_for_a_route(self, where, yaw_deg, placed, say, give_up, guard,
                         unwedge, stopped, tries):
        """The planner found no route and the goal was stopped before Nav2's
        recoveries turned the rover on the spot (`no_way_watch`). Hold still and
        ask the planner every `BLOCKED_ASK_S` -- nothing else is asking it now --
        for `BLOCKED_WAIT_S`: with a route, on to the goal; still none, the goal
        is handed back as blocked. A refusal of any other kind, about where the
        rover stands say, goes back to Nav2 as one goal with neither watch, to be
        dealt with as it always was. Nothing moves in here.
        """
        seq = self.stop_seq
        gx, gy, yaw = placed
        began = time.monotonic()
        waited = False
        while True:
            route, code = self.route_to(gx, gy, yaw)
            if route is not None or code != NO_VALID_PATH:
                break
            refused = ("a stop was asked for while waiting for a way past"
                       if self.stop_seq != seq else
                       autonomy_guard.refusal(guard, self.stop_seq, pose=self.pose()))
            if refused:
                stopped["detail"] = refused
                return stopped
            spent = time.monotonic() - began
            if spent >= BLOCKED_WAIT_S:
                stopped["detail"] = ("%s: something is in the way and the planner "
                                     "has no way round it, and it did not clear "
                                     "in %.0f s" % (NO_WAY, spent))
                return stopped
            if not waited:
                say("waiting", "something is in the way and there is no way "
                               "round it; waiting for it to move")
                waited = True
            time.sleep(BLOCKED_ASK_S)
        rest = self.goto(where, yaw_deg, say, give_up=give_up, guard=guard,
                         unwedge=unwedge,
                         tries=tries + 1 if route is not None else NEAR_TRIES)
        for key in ("travelled_m", "turned_deg"):
            rest[key] = round(float(stopped.get(key) or 0.0)
                              + float(rest.get(key) or 0.0), 3)
        if rest.get("reason") == "arrived" and waited:
            said = "waited for something in the way with no way round it"
            rest["detail"] = ("%s -- %s" % (said, rest["detail"])
                              if rest.get("detail") else said)
        return rest

    def route_points(self):
        """The route Nav2 is following, as (x, y) in the map frame, or []."""
        with self._lock:
            plan = self.plan
        try:
            return [(float(one.pose.position.x), float(one.pose.position.y))
                    for one in plan.poses]
        except AttributeError:
            return []

    def seen_on_the_route(self, live=None):
        """A sentence when the live scan has something on the next
        `ROUTE_LOOK_M` of the route, or "" -- also when there is no route, live
        costmap, pose or body to ask, which leaves the goal to Nav2."""
        here = self.pose()
        ahead = route_ahead(self.route_points(), here, ROUTE_LOOK_M) if here else []
        if len(ahead) < 2:
            return ""
        live = live or self.live_costmap()
        body = self.footprint() if live is not None else None
        if live is None or body is None:
            return ""
        grid, correction = live
        mapped = [None]
        for i, (a, b) in enumerate(zip(ahead, ahead[1:])):
            skip = LIVE_SKIP_M if i == 0 else 0.0
            if goal_fit.line_fits(grid, body, to_odom(correction, a),
                                  to_odom(correction, b), worst=goal_fit.LETHAL,
                                  skip_m=skip):
                continue
            # **Only what the map does not have.** A route through a doorway
            # can pass close enough to its frame that the body's sweep touches
            # the scan's hits on it, and that is a wall the planner meant to
            # pass, not something stepping into the way. So the same stretch is
            # asked of the map's walls, once, and a stretch that is blocked
            # there too is left to Nav2.
            #
            # The map, not the planner's costmap: since its live layer
            # (2026-10-08) that has a person in the way on it as well, and a
            # person in a doorway -- no way round, so Nav2 keeps the last route,
            # through them -- would be taken for a wall and left to Nav2's
            # recoveries, a quarter turn on the spot in front of them.
            if mapped[0] is None:
                mapped[0] = self.mapped_walls() or False
            if mapped[0] and not goal_fit.line_fits(mapped[0], body, a, b,
                                                    worst=goal_fit.LETHAL,
                                                    skip_m=skip):
                continue
            return ("%s: the scan has something on the route within %.1f m"
                    % (IN_THE_WAY, ROUTE_LOOK_M))
        return ""

    def route_watch(self, give_up):
        """`give_up` with the route ahead looked at on the live scan every
        `LIVE_LOOK_S`, which ends the goal with an `IN_THE_WAY` sentence."""
        looked = [None]

        def watch(now, feedback):
            if looked[0] is None or now - looked[0] >= LIVE_LOOK_S:
                looked[0] = now
                seen = self.seen_on_the_route()
                if seen:
                    return seen
            return give_up(now, feedback) if give_up else ""

        return watch

    def past_it(self, where, yaw_deg, say, give_up, guard, unwedge, stopped, tries):
        """A longer goal stopped for something on its route: on to a point
        `PAST_IT_M` along the route as a near goal, which waits for the way
        and goes round, then on to the goal. Handed back as blocked when there
        is no way round, or after `NEAR_TRIES` stops."""
        here = self.pose()
        route = self.route_points()
        if tries + 1 >= NEAR_TRIES or here is None:
            return stopped
        ahead = route_ahead(route or [tuple(here[:2]), tuple(where)], here, PAST_IT_M)
        point = ahead[-1] if len(ahead) >= 2 else tuple(where)
        went = self.near(point, None, "", say, give_up=None, guard=guard,
                         unwedge=unwedge)
        moved = float(stopped.get("travelled_m") or 0.0) + float(
            went.get("travelled_m") or 0.0)
        swung = float(stopped.get("turned_deg") or 0.0) + float(
            went.get("turned_deg") or 0.0)
        if went.get("reason") != "arrived":
            went.update(travelled_m=round(moved, 3), turned_deg=round(swung, 1))
            return went
        if math.hypot(point[0] - where[0], point[1] - where[1]) <= THERE_M and (
                yaw_deg is None):
            went.update(travelled_m=round(moved, 3), turned_deg=round(swung, 1))
            return went
        rest = self.goto(where, yaw_deg, say, give_up=give_up, guard=guard,
                         unwedge=unwedge, tries=tries + 1)
        rest["travelled_m"] = round(moved + float(rest.get("travelled_m") or 0.0), 3)
        rest["turned_deg"] = round(swung + float(rest.get("turned_deg") or 0.0), 1)
        if rest.get("reason") == "arrived":
            said = went.get("detail") or "waited for something in the way"
            rest["detail"] = ("%s -- %s" % (said, rest["detail"])
                              if rest.get("detail") else said)
        return rest

    def scan_has_near(self, grid, x, y, radius):
        """Is anything lethal on the planner's costmap within `radius` of
        (x, y) that the map's walls do not have, give or take a cell? False
        when there is no map to ask, which keeps the old wording."""
        walls = self.mapped_walls()
        if walls is None:
            return False
        reach = int(math.ceil(radius / grid.resolution))
        col, row = grid.cell_of(x, y)
        for r in range(max(0, row - reach), min(grid.height, row + reach + 1)):
            for c in range(max(0, col - reach), min(grid.width, col + reach + 1)):
                if grid.data[r * grid.width + c] != goal_fit.LETHAL:
                    continue
                cx = grid.origin_x + (c + 0.5) * grid.resolution
                cy = grid.origin_y + (r + 0.5) * grid.resolution
                if math.hypot(cx - x, cy - y) > radius:
                    continue
                wc, wr = walls.cell_of(cx, cy)
                if not any(0 <= wc + dc < walls.width and 0 <= wr + dr < walls.height
                           and walls.data[(wr + dr) * walls.width + wc + dc]
                           == goal_fit.LETHAL
                           for dc in (-1, 0, 1) for dr in (-1, 0, 1)):
                    return True
        return False

    def fit_goal(self, gx, gy, yaw):
        """Move a goal to the nearest place the rover's body will actually go.

        Returns the pose to send and a sentence about it, or None for the goal
        and a sentence saying why when there is nowhere near it that fits.

        Nav2 will not do this for itself, and the two halves of it disagree in a
        way that reads as a broken rover: NavFn plans for a point, so a cell five
        centimetres from a wall is a fine destination and it returns a clean
        straight path to it, while DWB checks the real rectangle and will not end
        a rollout there. What that looked like on the rover was twenty-five
        seconds of small heading corrections and then a timeout, with nothing
        anywhere saying the goal had been inside a wall the whole time. See
        goal_fit.py.

        A failure to ask -- the costmap service missing, the parameters not
        answering -- sends the goal unchanged. This is a check that improves a
        goal, not one the rover depends on to move, and a stack half way through
        starting up should not mean a refusal to drive.
        """
        body = self.footprint()
        grid = self.costmap() if body else None
        if grid is None:
            return (gx, gy, yaw), None
        placed = goal_fit.fit(grid, body, gx, gy, yaw)
        if placed is None:
            # The planner's costmap has what the scan sees as well as the map
            # (`live_layer`), so a refusal may be about a person standing there
            # or something moved, and blaming a wall for that sends whoever
            # asked to look at the map.
            if self.scan_has_near(grid, gx, gy, goal_fit.REACH_M + FIT_BODY_M):
                return None, ("there is nowhere within half a metre of that spot "
                              "where the rover's body fits right now -- the scan "
                              "sees something there that the map does not have, "
                              "a person or something moved")
            return None, ("there is nowhere within half a metre of that spot "
                          "where the rover's body fits -- it is inside a wall "
                          "or under something")
        if placed["moved_m"] < grid.resolution / 2.0:
            return (gx, gy, yaw), None
        return ((placed["x"], placed["y"], placed["yaw"]),
                "the spot asked for is too close to something for the rover to "
                "stand in, so the goal was moved %d cm to the nearest one it "
                "fits" % round(placed["moved_m"] * 100))

    def goto(self, where, yaw_deg, say, give_up=None, guard=None, unwedge=True,
             near=True, tries=0):
        """Somewhere on the map, with a planner and a costmap between.

        `where` is already in map coordinates -- the daemon converts an offset into
        one, because it is the daemon that knows the pose the map picture was drawn
        at. See `drive_to` in rover_nav.py for why a model is never shown map
        coordinates.

        With no `yaw_deg` the goal faces along the way it travelled, which is what
        the old planner left the rover doing and what makes a series of goals read
        as a journey rather than a set of arrivals in random directions.

        **A refusal about where the rover is standing is dealt with here, once.**
        START_OCCUPIED means the rover's own cell is inside the costmap's
        inscribed band, and Nav2 will refuse every destination on the map until
        it is somewhere else -- so handing that back asks the caller for a move
        it may not have. An autonomous run does not: on 2026-10-05 a look left
        the rover 0.2 m from a wall and the run's next three drives were refused
        that way until the run ended, while one turn by hand freed it. So the
        drive backs off the way exploring does (`back_off`, the short shuffle
        `goal_fit` names), under the same guard as the drive, and asks once more.
        A second refusal is handed back rather than shuffled on.

        **A goal nearer than `NEAR_GOAL_M` with a heading is a turn, a
        straight line and a turn** (`near`), and `near=False` is how that sends
        its straight line. One without a heading is one goal, as it always was.

        **No goal turns on the spot for ever.** One with no `give_up` of its own
        gets `frontier.Stall`: 25 s without getting 0.5 m further on, with Nav2
        attempting nothing, ends it. Nav2's progress checker cannot see that,
        because it counts a 20-degree swing as progress, and on 2026-10-07 a
        drive swung between 143 and 167 degrees for 72 s until somebody stopped
        it.
        """
        start = self.pose()
        if start is None:
            return {"reason": "lost", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "nothing is publishing the rover's position, so "
                              "there is no frame to drive in"}
        gx, gy = where
        if yaw_deg is None:
            yaw = math.atan2(gy - start[1], gx - start[0])
        else:
            yaw = math.radians(yaw_deg)

        placed, note = self.fit_goal(gx, gy, yaw)
        if placed is None:
            return {"reason": "blocked", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": note}
        gx, gy, yaw = placed
        blocked = autonomy_guard.refusal(guard, self.stop_seq, pose=start,
                                         goal=(gx, gy))
        if blocked:
            return {"reason": "blocked", "travelled_m": 0.0,
                    "turned_deg": 0.0, "detail": blocked}

        straight = math.hypot(gx - start[0], gy - start[1])
        if near and yaw_deg is not None and straight < NEAR_GOAL_M:
            return self.near(
                (gx, gy), None if yaw_deg is None else yaw, note, say,
                give_up=give_up, guard=guard, unwedge=unwedge)

        caller_give_up = give_up
        if give_up is None:
            watch = frontier.Stall()

            def give_up(now, feedback):
                return watch.update(now, self.pose(),
                                    int(feedback.get("recoveries") or 0))

        if near:
            # Not for a near goal's own straight line (`near=False`), which has
            # `near_watch` already.
            give_up = self.route_watch(give_up)
            if tries < NEAR_TRIES:
                give_up = self.no_way_watch(give_up)

        previous_give_up = give_up
        if guard is not None:
            def give_up(now, feedback):
                with self._lock:
                    plan, seq = self.plan, self.stop_seq
                return (autonomy_guard.refusal(guard, seq, pose=self.pose(),
                                               goal=(gx, gy), path=plan)
                        or (previous_give_up(now, feedback)
                            if previous_give_up else ""))

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.args.map_frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(gx)
        goal.pose.pose.position.y = float(gy)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        # Generous, and a backstop rather than a schedule: Nav2 may legitimately
        # spend a while backing out of a corner and trying again, and a limit tight
        # enough to be a schedule would cancel exactly the recoveries that were
        # about to work. It starts from the straight line only because that is all
        # there is to go on before the planner has answered; `budget` below
        # replaces it with the route as soon as there is one. See
        # ROUTE_SAMPLE_M for the 3 m goal that used to time out on 8.8 m of route.
        limit = max(TIME_ALLOWANCE_MIN_ROUTE_S,
                    TIME_ALLOWANCE_SLACK * straight / DEFAULT_SPEED_MS)
        # The longest route seen while this move has been running, kept rather
        # than recomputed from the current plan alone: the plan shortens as the
        # rover eats into it, and an allowance that shortened with it would
        # tighten exactly as the rover ran out of time.
        longest = [0.0, 0.0]

        def budget():
            with self._lock:
                plan = self.plan
            metres, turning = route_cost.from_path(plan)
            if metres > longest[0]:
                longest[0], longest[1] = metres, turning
            if longest[0] <= 0.0:
                return limit
            return route_cost.seconds_for(
                longest[0], longest[1], DEFAULT_SPEED_MS, ROUTE_TURN_DPS,
                slack=TIME_ALLOWANCE_SLACK, floor=limit)

        def measure(fb):
            with self._lock:
                self.remaining_m = round(float(fb.distance_remaining), 2)
                plan = self.plan
            # How many poses the planner produced, so the console can say what
            # route was accepted rather than only how far is left. Nav2's feedback
            # does not carry it; the plan it publishes does.
            return {"remaining_m": self.remaining_m,
                    "waypoints": len(plan.poses) if plan is not None else 0,
                    "route_m": round(longest[0], 2) or None,
                    "recoveries": int(fb.number_of_recoveries)}

        outcome = self.run_goal("goto", goal, limit, say, measure, budget=budget,
                                give_up=give_up, **({"guard": guard} if guard is not None else {}))
        if (near and outcome.get("reason") == "blocked"
                and str(outcome.get("detail") or "").startswith(IN_THE_WAY)):
            return self.past_it(where, yaw_deg, say, caller_give_up, guard,
                                unwedge, outcome, tries)
        if (near and outcome.get("reason") == "blocked"
                and str(outcome.get("detail") or "").startswith(NO_WAY)):
            return self.wait_for_a_route(where, yaw_deg, (gx, gy, yaw), say,
                                         caller_give_up, guard, unwedge, outcome,
                                         tries)
        if (unwedge and outcome.get("code") == START_OCCUPIED
                and float(outcome.get("travelled_m") or 0.0) < UNWEDGE_MOVED_M):
            return self.unwedged(outcome, where, yaw_deg, say, previous_give_up,
                                 guard)
        # **How far the route was, said out loud.** A move that ran out of time on
        # a route three times the length of the straight line is a different event
        # from one that ran out of time going nowhere, and the console could not
        # tell them apart: both said "timed out".
        if longest[0] > straight * 1.3 and outcome.get("reason") != "arrived":
            outcome["detail"] = (
                "%s -- the route round was %.1f m for a goal %.1f m away"
                % (outcome.get("detail") or "no route", longest[0], straight))
        # Said whatever happened, including on arrival: a rover that stopped 20 cm
        # from where somebody pointed has done the right thing, and the console
        # saying so is the difference between that and a rover that missed.
        if note:
            outcome["detail"] = ("%s -- %s" % (outcome["detail"], note)
                                 if outcome.get("detail") else note)
        return outcome

    def near(self, goal, yaw, note, say, give_up=None, guard=None,
             unwedge=True):
        """A goal close by: face it, drive straight to it, then turn to `yaw`.

        `goal` has already been fitted and checked against the guard; `yaw` is
        in radians, or None when the caller asked for no heading. See
        `NEAR_GOAL_M` for what this replaces. A turn the rover cannot make where
        it stands -- 23% of the places its body fits are too tight to turn all
        the way round in (trap_sim.py) -- falls back to the one goal Nav2 would
        have been given, so nothing that drove before is refused now. A stop is
        not fallen back from.
        """
        moved, swung = [0.0], [0.0]

        def tally(outcome):
            moved[0] += float(outcome.get("travelled_m") or 0.0)
            swung[0] += float(outcome.get("turned_deg") or 0.0)
            outcome["travelled_m"] = round(moved[0], 3)
            outcome["turned_deg"] = round(swung[0], 1)
            if note:
                outcome["detail"] = ("%s -- %s" % (outcome["detail"], note)
                                     if outcome.get("detail") else note)
            return outcome

        here = self.pose()
        if math.hypot(goal[0] - here[0], goal[1] - here[1]) <= THERE_M:
            off = 0.0 if yaw is None else math.degrees(wrap(yaw - here[2]))
            if abs(off) <= FINAL_TURN_DEG:
                return tally({"reason": "arrived", "travelled_m": 0.0,
                              "turned_deg": 0.0, "detail": ""})
            return tally(self.turn(off, say, guard=guard))

        for _ in range(FACE_TURNS):
            here = self.pose()
            off = wrap(math.atan2(goal[1] - here[1], goal[0] - here[0]) - here[2])
            if abs(math.degrees(off)) <= FACE_WITHIN_DEG:
                break
            turned = self.turn(math.degrees(off), say, guard=guard)
            if turned.get("reason") == "stopped":
                return tally(turned)
            swung[0] += float(turned.get("turned_deg") or 0.0)
            if turned.get("reason") != "arrived":
                # Cannot pivot here: the single goal, as it always was.
                return tally(self.goto(
                    goal, None if yaw is None else math.degrees(yaw), say,
                    give_up=give_up, guard=guard, unwedge=unwedge, near=False))

        watch = self.near_watch(goal, give_up)
        for attempt in range(NEAR_TRIES):
            blocked = self.wait_for_the_way(goal, say, guard)
            if blocked and not blocked.startswith(IN_THE_WAY):
                return tally({"reason": "blocked", "travelled_m": 0.0,
                              "turned_deg": 0.0, "detail": blocked})
            if blocked:
                went = self.round_by_legs(goal, say, guard, blocked)
                if went.get("reason") != "arrived" or yaw is None:
                    return tally(went)
                moved[0] += float(went.get("travelled_m") or 0.0)
                swung[0] += float(went.get("turned_deg") or 0.0)
                here = self.pose()
                off = math.degrees(wrap(yaw - here[2]))
                if abs(off) <= FINAL_TURN_DEG:
                    return tally(dict(went, travelled_m=0.0, turned_deg=0.0))
                turned = self.turn(off, say, guard=guard)
                if turned.get("reason") == "arrived":
                    turned["detail"] = went.get("detail") or ""
                return tally(turned)

            # Straight there, arriving the way it is travelling, so the planner
            # has no heading to loop round for -- and stopping, to wait or go
            # round, if something steps into the way once it has set off.
            drove = self.goto(goal, None, say, give_up=watch, guard=guard,
                              unwedge=unwedge, near=False)
            if (drove.get("reason") == "blocked" and attempt + 1 < NEAR_TRIES
                    and (drove.get("detail") or "").startswith(IN_THE_WAY)):
                moved[0] += float(drove.get("travelled_m") or 0.0)
                swung[0] += float(drove.get("turned_deg") or 0.0)
                continue
            break
        if drove.get("reason") != "arrived" or yaw is None:
            return tally(drove)
        moved[0] += float(drove.get("travelled_m") or 0.0)
        swung[0] += float(drove.get("turned_deg") or 0.0)
        here = self.pose()
        off = math.degrees(wrap(yaw - here[2]))
        if abs(off) <= FINAL_TURN_DEG:
            return tally(dict(drove, travelled_m=0.0, turned_deg=0.0))
        turned = self.turn(off, say, guard=guard)
        if turned.get("reason") == "arrived" and drove.get("detail"):
            turned["detail"] = drove["detail"]
        return tally(turned)

    def wait_for_the_way(self, goal, say, guard=None):
        """Hold still while the straight way to a near goal is blocked.

        Returns "" once the live scan has nothing on the straight line
        (`seen_in_the_way`) and the planner's route is within `DETOUR_SLACK_M`
        of it -- or when the planner does not answer, which leaves the goal to
        Nav2 as before -- and a sentence when the way stayed blocked for
        `BLOCKED_WAIT_S` or a stop came while waiting. Nothing moves in here.
        """
        seq = self.stop_seq
        waited = 0.0
        while True:
            here = self.pose()
            if here is None:
                return ""
            straight = math.hypot(goal[0] - here[0], goal[1] - here[1])
            seen = self.seen_in_the_way(goal)
            route = None
            if not seen:
                route, _ = self.route_to(
                    goal[0], goal[1],
                    math.atan2(goal[1] - here[1], goal[0] - here[0]))
                if route is None or route[0] <= straight + DETOUR_SLACK_M:
                    return ""
            if self.stop_seq != seq:
                return "a stop was asked for while the way was blocked"
            stopped = autonomy_guard.refusal(guard, self.stop_seq, pose=here)
            if stopped:
                return stopped
            if waited >= BLOCKED_WAIT_S:
                if seen:
                    return "%s, and it did not clear in %.0f s" % (seen, waited)
                return ("%s: the only route was %.1f m for a goal %.1f m away, "
                        "and it did not clear in %.0f s"
                        % (IN_THE_WAY, route[0], straight, waited))
            if waited == 0.0:
                say("waiting", "something is in the way; waiting for it to move")
            time.sleep(BLOCKED_ASK_S)
            waited += BLOCKED_ASK_S

    def round_by_legs(self, goal, say, guard, why):
        """Go round what is in the way, as straight legs with turns between.

        Round something the live scan sees, the way is found on the live
        costmap (`goal_fit.legs_round`), since the planner's map does not have
        it. Otherwise the route is the planner's last answer from
        `wait_for_the_way`, cut by `goal_fit.straight_legs` into the fewest
        lines the body fits down on its costmap. Each leg is a turn to face its
        end and a straight drive that stops at anything in its way (`drive`),
        checked against the guard first. Where there is no such set of legs,
        the goal is handed back as blocked with `why`.
        """
        here = self.pose()
        straight = math.hypot(goal[0] - here[0], goal[1] - here[1])
        live = self.live_costmap()
        if live is not None and self.seen_in_the_way(goal, live):
            grid, correction = live
            found = goal_fit.legs_round(
                grid, to_odom(correction, here), to_odom(correction, goal),
                ROUND_CLEAR_M, ROUND_TIGHT_M, ROUND_RELAX_M, ROUND_LEGS)
            legs = [to_map(correction, end) for end in found] if found else None
            ends = [tuple(here[:2])] + list(legs or [])
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(ends, ends[1:]))
            if legs and length > straight + ROUND_EXTRA_M:
                legs = None
        else:
            route = getattr(self, "last_route", None)
            body = self.footprint()
            grid = self.costmap() if body else None
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(route or [], (route or [])[1:]))
            legs = (goal_fit.straight_legs(grid, body, here[:2], route, ROUND_LEGS)
                    if route and grid is not None
                    and length <= straight + ROUND_EXTRA_M else None)
        if not legs:
            return {"reason": "blocked", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": why + ", and there is no way round it in %d straight "
                                    "legs" % ROUND_LEGS}
        say("choosing", "going round what is in the way, in %d straight leg%s"
                        % (len(legs), "" if len(legs) == 1 else "s"))
        moved, swung = 0.0, 0.0
        for end in legs:
            here = self.pose()
            refused = autonomy_guard.refusal(guard, self.stop_seq, pose=here[:2],
                                             goal=end)
            if refused:
                return {"reason": "blocked", "travelled_m": round(moved, 3),
                        "turned_deg": round(swung, 1), "detail": refused}
            off = math.degrees(wrap(math.atan2(end[1] - here[1], end[0] - here[0])
                                    - here[2]))
            if abs(off) > 1.0:
                turned = self.turn(off, say, guard=guard)
                swung += float(turned.get("turned_deg") or 0.0)
                if turned.get("reason") != "arrived":
                    turned.update(travelled_m=round(moved, 3), turned_deg=round(swung, 1))
                    return turned
            here = self.pose()
            leg = self.drive(math.hypot(end[0] - here[0], end[1] - here[1]),
                             DEFAULT_SPEED_MS, say, guard=guard)
            moved += float(leg.get("travelled_m") or 0.0)
            if leg.get("reason") != "arrived":
                leg.update(travelled_m=round(moved, 3), turned_deg=round(swung, 1))
                return leg
        return {"reason": "arrived", "travelled_m": round(moved, 3),
                "turned_deg": round(swung, 1),
                "detail": "went round something in the way, in %d straight leg%s"
                          % (len(legs), "" if len(legs) == 1 else "s")}

    def unwedged(self, refused, where, yaw_deg, say, give_up, guard):
        """Back off from where the planner will not plan, then ask again once.

        The back-off's metres and degrees are added to whatever the second
        attempt reports, because they are real driving and belong to this goal:
        an autonomous run attributes every movement to the goal that made it.
        """
        escape = self.back_off(say, guard=guard)
        moved = float(escape.get("travelled_m") or 0.0)
        swung = float(escape.get("turned_deg") or 0.0)
        if escape.get("reason") != "arrived":
            refused["travelled_m"] = round(
                float(refused.get("travelled_m") or 0.0) + moved, 3)
            refused["turned_deg"] = round(
                float(refused.get("turned_deg") or 0.0) + swung, 1)
            refused["detail"] = "%s -- and backing off did not help: %s" % (
                refused.get("detail") or "the planner would not plan from here",
                escape.get("detail") or escape.get("reason"))
            return refused
        again = self.goto(where, yaw_deg, say, give_up=give_up, guard=guard,
                          unwedge=False)
        again["travelled_m"] = round(
            float(again.get("travelled_m") or 0.0) + moved, 3)
        again["turned_deg"] = round(
            float(again.get("turned_deg") or 0.0) + swung, 1)
        again["detail"] = "%s, then %s" % (
            escape.get("detail") or "backed off",
            again.get("detail") or again.get("reason"))
        return again
