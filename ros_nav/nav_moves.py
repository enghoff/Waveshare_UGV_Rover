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
from nav_codes import phrase_for, reason_for
from nav_limits import (
    COSTMAP_TIMEOUT_S, DEFAULT_SPEED_MS, DEFAULT_TURN_DPS, PROGRESS_S,
    REVERSE_LIMIT_M, ROUTE_TURN_DPS, TIME_ALLOWANCE_FLOOR_S,
    TIME_ALLOWANCE_MIN_ROUTE_S, TIME_ALLOWANCE_SLACK, duration, wrap,
)


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
                 budget=None, give_up=None):
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
        is the opposite of `budget`, which can only ever push the deadline out,
        and only `explore` passes one: a caller who asked for one particular
        place is owed every recovery Nav2 has before being told no, and a caller
        with sixteen other frontiers to try is not. See `frontier.Stall` for what
        it watches and why Nav2 cannot see it.
        """
        client = self.actions[kind]
        if not client.wait_for_server(timeout_sec=2.0):
            return {"reason": "refused", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "Nav2 is not running, so the rover will not drive "
                              "itself. Only the mapping half of the stack is up."}
        with self._lock:
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
        return {"reason": reason_for(code), "travelled_m": travelled,
                "turned_deg": turned,
                "detail": (phrase_for(code, message)
                           or "Nav2 gave up without saying why (code %s)" % code)}

    def drive(self, distance_m, speed_ms, say):
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
            lambda fb: {"travelled_m": round(abs(fb.distance_traveled), 3)})

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

    def turn(self, angle_deg, say):
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
            motion="turning")

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
        self.body = goal_fit.polygon_from(answer.values[0].string_value,
                                          answer.values[1].double_value)
        return self.body

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

    def goto(self, where, yaw_deg, say, give_up=None):
        """Somewhere on the map, with a planner and a costmap between.

        `where` is already in map coordinates -- the daemon converts an offset into
        one, because it is the daemon that knows the pose the map picture was drawn
        at. See `drive_to` in rover_nav.py for why a model is never shown map
        coordinates.

        With no `yaw_deg` the goal faces along the way it travelled, which is what
        the old planner left the rover doing and what makes a series of goals read
        as a journey rather than a set of arrivals in random directions.
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
        straight = math.hypot(gx - start[0], gy - start[1])
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
                                give_up=give_up)
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
