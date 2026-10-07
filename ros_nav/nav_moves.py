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
from rcl_interfaces.srv import GetParameters

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
#: seconds; a chair does not move. After this the goal is handed back as
#: blocked rather than driven round.
BLOCKED_WAIT_S = 10.0
BLOCKED_ASK_S = 2.0


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
             near=True):
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

        if give_up is None:
            watch = frontier.Stall()

            def give_up(now, feedback):
                return watch.update(now, self.pose(),
                                    int(feedback.get("recoveries") or 0))

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

        blocked = self.wait_for_the_way(goal, say, guard)
        if blocked:
            return tally({"reason": "blocked", "travelled_m": 0.0,
                          "turned_deg": 0.0, "detail": blocked})

        # Straight there, arriving the way it is travelling, so the planner has
        # no heading to loop round for.
        drove = self.goto(goal, None, say, give_up=give_up, guard=guard,
                          unwedge=unwedge, near=False)
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

        Returns "" once the planner's route is within `DETOUR_SLACK_M` of the
        straight line -- or when the planner does not answer, which leaves the
        goal to Nav2 as before -- and a sentence when the way stayed blocked for
        `BLOCKED_WAIT_S` or a stop came while waiting. Nothing moves in here.
        """
        seq = self.stop_seq
        waited = 0.0
        while True:
            here = self.pose()
            if here is None:
                return ""
            straight = math.hypot(goal[0] - here[0], goal[1] - here[1])
            route, _ = self.route_to(goal[0], goal[1],
                                     math.atan2(goal[1] - here[1], goal[0] - here[0]))
            if route is None or route[0] <= straight + DETOUR_SLACK_M:
                return ""
            if self.stop_seq != seq:
                return "a stop was asked for while the way was blocked"
            stopped = autonomy_guard.refusal(guard, self.stop_seq, pose=here)
            if stopped:
                return stopped
            if waited >= BLOCKED_WAIT_S:
                return ("something is in the way: the only route was %.1f m for a "
                        "goal %.1f m away, and it did not clear in %.0f s"
                        % (route[0], straight, waited))
            if waited == 0.0:
                say("waiting", "something is in the way; waiting for it to move")
            time.sleep(BLOCKED_ASK_S)
            waited += BLOCKED_ASK_S

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
