#!/usr/bin/env python3
"""Exploring: choosing where to go next, and getting unstuck.

`explore` is the only thing in this stack that decides for itself where the rover
should be. It asks `frontier` which gap in the map is worth driving to, plans to
it, drives, and does it again until the budget runs out or there is nothing left
worth reaching. `back_off` and `shuffle_by_turning` are what happens when it
cannot: a rover that has run out of places it can plan from is not a finished
house, and the difference matters to whoever reads the reason afterwards.

A mixin, like the moves it drives through, and for the same reason -- it needs
the node's clock, its costmap and its pose. Mixed into `NavBridge` beside it.
"""

import math
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

# Beside this file and with no ROS in them: which gap in the map is worth driving
# to is grid arithmetic, and the checks argue with the same `frontier` against a
# real map saved off the rover rather than against a second copy of it.
import frontier
import goal_fit
import nav_codes
import route_cost
from nav_limits import (
    ESCAPE_SPEED_MS, ESCAPE_TURN_DEG, EXPLORE_BUDGET_S, EXPLORE_DETOUR_NOTE,
    EXPLORE_SHUFFLES, EXPLORE_TRIES, PLAN_TIMEOUT_S,
    TIME_ALLOWANCE_MIN_ROUTE_S, wrap,
)


class NavExplore:
    """The half of `NavBridge` that decides where the rover should go."""

    def route_to(self, gx, gy, yaw):
        """Is there actually a route from here to there? The planner's answer.

        Returns `(route, code)`. `route` is `(metres, degrees)` for the drive it
        would take, or None when there is none; `code` is the planner's own
        `error_code`, or None when it never answered. Nothing moves:
        `ComputePathToPose` is the planner server answering the same question
        `NavigateToPose` would start by asking, and asking it directly costs
        about a second.

        **The code is not decoration, and the caller must read it.** A refusal
        naming the *start* is a statement about the rover, and it will be the
        same refusal for every destination on the map -- see `ABOUT_THE_ROVER` in
        `nav_codes.py` for the run where reading four of those as verdicts on
        four frontiers had the rover announce that the house was fully explored
        with 73% of the map still unknown.

        **Why this is worth a second before every frontier.** `frontier.py` ranks
        frontiers by walking the occupancy grid cell to cell, which is fast, gives
        the real distance round the furniture rather than through it, and is
        wrong in one direction: it walks as a point. The planner plans with the
        rover's inflated body, so a frontier the walk reached through the 5 cm gap
        between a table leg and the wall is one the rover cannot get to at all.
        Without this check that frontier is the best-ranked candidate every round
        for as long as it takes the goal to fail, and each of those failures costs
        the full recovery ladder -- three progress-checker windows, two costmap
        clears and a spin, which is the better part of a minute of the rover
        shuffling in a doorway it does not fit through.

        A planner that is not answering at all returns None too, which reads here
        as "no route" and is the safe way round: the alternative is sending goals
        into a stack that cannot plan them.
        """
        if not self.plan_client.wait_for_server(timeout_sec=2.0):
            return None, None
        goal = ComputePathToPose.Goal()
        goal.goal = PoseStamped()
        goal.goal.header.frame_id = self.args.map_frame
        goal.goal.header.stamp = self.get_clock().now().to_msg()
        goal.goal.pose.position.x = float(gx)
        goal.goal.pose.position.y = float(gy)
        goal.goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.goal.pose.orientation.w = math.cos(yaw / 2.0)
        # Named rather than left empty. `planner_plugins` has one entry today and
        # an empty id resolves to it, but it resolves by *accident* -- the first
        # of however many are loaded -- and the day a second planner is added for
        # an experiment this would quietly start checking routes with a planner
        # the rover is not going to drive with.
        goal.planner_id = "GridBased"
        # From where the rover actually is, which is the only start that means
        # anything here: the question is whether it can get there from here.
        goal.use_start = False

        send = self.plan_client.send_goal_async(goal)
        if not self.wait(send, PLAN_TIMEOUT_S):
            return None, None
        handle = send.result()
        if handle is None or not handle.accepted:
            return None, None
        result_future = handle.get_result_async()
        if not self.wait(result_future, PLAN_TIMEOUT_S):
            handle.cancel_goal_async()
            return None, None
        wrapped = result_future.result()
        answer = getattr(wrapped, "result", None)
        # Read off the aborted result as well as the successful one, which is the
        # whole point of asking: `planner_server` fills `error_code` in and then
        # aborts the handle, so the reason is only ever on a result nobody would
        # look at if they were checking the status first.
        code = getattr(answer, "error_code", None)
        if getattr(wrapped, "status", None) != GoalStatus.STATUS_SUCCEEDED:
            return None, code
        path = getattr(answer, "path", None)
        # Two poses, not one. The planner answers a goal it is already standing on
        # with a single-pose path, which is a true answer to a question worth
        # nothing -- and `route_cost` prices it at zero, which would make it the
        # cheapest frontier on the map for ever.
        if path is None or len(path.poses) < 2:
            return None, code
        return route_cost.from_path(path), code

    def back_off(self, say):
        """Shuffle the rover to somewhere the planner is willing to plan from.

        Returns an outcome in the same shape every other move here does, so the
        run that called it can add the metres and degrees to its own account
        rather than losing them: a back-off is real driving and belongs in the
        "0.1 m driven" the caller reports.

        Nothing here is about a destination. This runs when the planner has
        refused a route because of where the rover is *standing*, which it will
        go on doing for every destination on the map until the rover is
        somewhere else.

        **Where to go is not guessed.** `goal_fit.fit` is already the answer to
        "the nearest place this body fits", it reads the same global costmap the
        planner refused with, and it is the check every goal goes through
        anyway -- so the spot it names is one the planner has in effect already
        agreed to. On the rover on 2026-09-01 the pose in every refusal was
        0.156 m from a mapped wall against a 0.200 m footprint, `fit` named a
        place 27 cm away, and three of the four frontiers that had just been
        called unreachable planned from there in 0.01 s.

        **It turns and drives forwards rather than reversing**, for
        `reverse_by_turning`'s reason: the lidar looks one way, and ground the
        rover is about to occupy is worth having a sensor pointed at. The turn
        costs a few seconds of a ten-minute budget, and turning on the spot is
        the one move this rover's own behaviour plugins guarantee while it is
        touching something.
        """
        where = self.pose()
        if where is None:
            return {"reason": "lost", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "nothing is publishing the rover's position, so "
                              "there is no telling which way is out"}
        body = self.footprint()
        grid = self.costmap() if body else None
        if grid is None:
            # No costmap to ask, so no opinion about which way is out. A turn is
            # the blind version of the same move and is what Nav2's own recovery
            # would try, so it is worth an attempt before giving up.
            return self.shuffle_by_turning(
                say, "the costmap did not answer, so this is a turn on the spot "
                     "and a hope")
        placed = goal_fit.fit(grid, body, where[0], where[1], where[2])
        if placed is None:
            return {"reason": "blocked", "travelled_m": 0.0, "turned_deg": 0.0,
                    "detail": "the rover is up against something and there is "
                              "nowhere within half a metre of it where its body "
                              "fits, so it cannot get itself out of this"}
        away = math.hypot(placed["x"] - where[0], placed["y"] - where[1])
        if away < grid.resolution:
            # The body fits where it is standing, so the planner refused over
            # something this cannot see. Turning re-registers the scan match and
            # moves the rover a few centimetres whether it means to or not, which
            # is what freed it the one time this was watched happening.
            return self.shuffle_by_turning(
                say, "the costmap says the rover fits where it is, so this is a "
                     "turn to shake the disagreement loose")

        bearing = math.atan2(placed["y"] - where[1], placed["x"] - where[0])
        say("choosing", "backing off %d cm to somewhere it can plan from"
                        % round(away * 100))
        about = self.turn(math.degrees(wrap(bearing - where[2])), say)
        if about.get("reason") != "arrived":
            about["detail"] = (
                "it could not even turn towards the one spot nearby where its "
                "body fits -- %s" % (about.get("detail") or "the turn did not "
                                                            "finish"))
            return about
        onward = self.drive(away, ESCAPE_SPEED_MS, say)
        onward["turned_deg"] = round(
            (about.get("turned_deg") or 0.0) + (onward.get("turned_deg") or 0.0),
            1)
        if onward.get("reason") != "arrived":
            onward["detail"] = (
                "it turned towards the nearest spot its body fits and then could "
                "not get there -- %s" % (onward.get("detail")
                                         or "the drive did not finish"))
            return onward
        onward["detail"] = ("backed off %d cm to somewhere the planner will plan "
                            "from" % round(away * 100))
        return onward

    def shuffle_by_turning(self, say, why):
        """A quarter turn, for when there is nothing better to go on."""
        say("choosing", why)
        about = self.turn(ESCAPE_TURN_DEG, say)
        if about.get("reason") == "arrived":
            about["detail"] = ("turned on the spot to see whether that frees the "
                               "planner -- %s" % why)
        return about

    def explore(self, say, budget_s=EXPLORE_BUDGET_S, min_frontier_m=None):
        """Drive to the edge of the map until there is no edge left to drive to.

        One long move rather than a mode: it holds the move mutex for its whole
        length like `drive` and `goto` do, so a `drive_to` arriving in the middle
        of it is refused as busy rather than fighting it for the same action
        server. Stopping is never blocked -- `halt` cancels the goal in flight and
        this notices and ends -- which is the same arrangement every other move
        here has and the reason it is safe to give the rover ten minutes of its
        own work.

        The loop is four steps and they are all cheap except the last:

            look at the map        frontier.py, about 20 ms on a room
            check the best one     the planner, about a second
            drive to it           Nav2, a minute if it is in the next room
            write it off          so the next round chooses something else

        **A refusal is read for what it is about before anything is written
        off.** The planner declines a route for two quite different reasons and
        they need opposite responses: the destination is walled off, so try
        another one -- or the rover's own cell is inside the inscribed band, in
        which case it will decline every destination on the map and the thing to
        deal with is the rover. `back_off` is that, and `ABOUT_THE_ROVER` in
        `nav_codes.py` is how the two are told apart. Conflating them is what
        had this loop announce a fully explored house from four planner calls
        with 73% of the map unknown; the account is in that constant.

        **Every frontier is written off once it has been driven to, whether or
        not the rover got there.** For a failure that is obvious. For an arrival
        it is the rule that makes the loop terminate: the rover stops within
        22 cm of the goal by the controller's tolerance, and if the lidar did not
        happen to see past the corner from there the frontier is still on the map
        and still the nearest one, so without this the rover drives the same
        30 cm again for as long as the budget lasts. What it costs is the
        occasional pocket left unexplored behind a corner the rover stood next to
        -- which a second `explore` picks up, because the blacklist lives as long
        as one call and no longer.
        """
        if min_frontier_m is None:
            min_frontier_m = frontier.MIN_FRONTIER_M
        began = time.monotonic()
        deadline = began + max(0.0, float(budget_s))

        # Which frontier, and which ones are finished with, is `frontier.py`'s
        # and not this file's -- so that `explore_sim.py` drives the same policy
        # against a room the rover has already been round, rather than a second
        # copy of it that agrees today.
        explorer = frontier.Explorer(min_frontier_m=min_frontier_m)
        goals = arrived = refused = shuffles = 0
        travelled = turned = 0.0

        def totals(reason, detail):
            """The outcome, with the whole run's account written into the sentence.

            The counts go in `detail` rather than into fields of their own
            because `Outcome` in `lidar_slam/nav_types.py` is four fields and
            both consoles and the voice model read it by name. Widening it for
            this one op would mean teaching every one of them a shape only this
            op ever produces; a sentence is something they all already render,
            and it is what a person actually wants to be told.
            """
            said = [detail]
            if goals:
                said.append("%d frontier%s tried, %d reached, %.1f m driven"
                            % (goals, "" if goals == 1 else "s", arrived,
                               travelled))
            left = explorer.summary.get("frontiers", 0)
            if left:
                said.append("%d more still on the map" % left)
            share = frontier.unknown_share(explorer.summary)
            if share is not None:
                said.append("%.0f%% of the map is still unknown" % (100 * share))
            return {"reason": reason, "travelled_m": travelled,
                    "turned_deg": turned,
                    "detail": " -- ".join(part for part in said if part),
                    # Kept on the wire as well as in the sentence. Nothing reads
                    # them today; a console that wants to draw the run rather
                    # than read about it will not have to change this file.
                    "goals": goals, "arrived": arrived, "frontiers_left": left,
                    "unroutable": refused, "shuffles": shuffles,
                    "unknown_share": None if share is None else round(share, 3)}

        with self._lock:
            # Cleared here for `run_goal`'s reason, and it is load-bearing rather
            # than tidy: `halt` sets this and only the start of a move clears it,
            # so an explore begun after somebody stopped a drive would read the
            # previous move's stop as its own and end before it had chosen
            # anything.
            self.cancelled = False
            self.exploring = True
            # Every stop from here on is this run's, however many goals it takes.
            # `cancelled` alone cannot say that: `run_goal` clears it as each goal
            # starts, so a stop landing while this was choosing where to go next
            # -- a good few seconds, between the map and the planner -- was wiped
            # by the next goal and the rover set off again with the STOP button
            # already pressed. A counter cannot be cleared by accident.
            stops = self.stop_seq
        try:
            while True:
                with self._lock:
                    if self.cancelled or self.stop_seq != stops:
                        return totals("stopped", "a stop was asked for")
                    if self.estop:
                        return totals("blocked", "the stop is latched")
                left = deadline - time.monotonic()
                # Checked before choosing rather than before driving, and the
                # floor is a whole goal's worth of clock. Starting a goal with
                # twenty seconds left buys a cancelled move and a rover parked in
                # a doorway; stopping with twenty seconds unused buys a rover
                # parked where it chose to be.
                if left < TIME_ALLOWANCE_MIN_ROUTE_S:
                    return totals("timed out",
                                  "the time allowed for exploring ran out")

                with self._lock:
                    grid_msg = self.map_msg
                where = self.pose()
                if grid_msg is None or where is None:
                    return totals("lost",
                                  "there is no map or no position to explore "
                                  "from, so nothing knows where the edges are")

                grid = frontier.Grid(
                    grid_msg.info.width, grid_msg.info.height,
                    grid_msg.info.resolution,
                    grid_msg.info.origin.position.x,
                    grid_msg.info.origin.position.y,
                    grid_msg.data)
                found = explorer.choose(grid, (where[0], where[1]))
                if not found:
                    # The one honest way to say the map is finished, and it is
                    # reached by having looked at every frontier on it rather
                    # than by having a sample refused. `refused` separates the
                    # two endings a person cares about: a house that has been
                    # mapped, and a house whose remaining edges the planner would
                    # not route to -- which reads very differently beside the
                    # "still unknown" figure this sentence ends with.
                    if refused and not arrived:
                        return totals(
                            "blocked",
                            "the rover could not get a route to any of the %d "
                            "place%s left on the map"
                            % (refused, "" if refused == 1 else "s"))
                    return totals(
                        "arrived",
                        "there is nothing left on the map worth driving to"
                        + (", after %d place%s tried"
                           % (explorer.tried(),
                              "" if explorer.tried() == 1 else "s")
                           if explorer.tried() else ""))

                # --- which of them the planner will actually take
                chosen = route = None
                wedged = None
                for candidate in found[:EXPLORE_TRIES]:
                    say("choosing", "%.1f m away, %.1f m of new edge, "
                                    "%d frontier%s on the map"
                        % (candidate["distance_m"], candidate["size_m"],
                           explorer.summary["frontiers"],
                           "" if explorer.summary["frontiers"] == 1 else "s"),
                        frontiers_left=explorer.summary["frontiers"],
                        goals=goals)
                    placed, note = self.fit_goal(
                        candidate["x"], candidate["y"], candidate["yaw"])
                    if placed is None:
                        # A real frontier with nowhere beside it the body fits.
                        # Counted with the planner's refusals rather than
                        # separately, because both mean the same thing to the
                        # person reading the outcome: that edge of the map is
                        # still there and the rover cannot get to it.
                        refused += 1
                        explorer.wrote_off(candidate["x"], candidate["y"])
                        continue
                    priced, code = self.route_to(*placed)
                    if code in nav_codes.ABOUT_THE_ROVER:
                        # Not this frontier's fault and not the next one's
                        # either: the planner has refused the *start*, so it will
                        # refuse every destination on the map until the rover is
                        # somewhere else. Stop pricing frontiers and go and deal
                        # with the rover.
                        wedged = code
                        break
                    if priced is None:
                        # The walk got there and the planner will not, which is
                        # the case this check exists for. Written off without
                        # driving a centimetre.
                        refused += 1
                        explorer.wrote_off(candidate["x"], candidate["y"])
                        continue
                    chosen, route = (candidate, placed, note), priced
                    break

                if wedged is not None:
                    if shuffles >= EXPLORE_SHUFFLES:
                        return totals("blocked",
                                      "the rover is somewhere the planner will "
                                      "not plan from and %d attempts to shuffle "
                                      "it clear did not help"
                                      % EXPLORE_SHUFFLES)
                    if wedged != nav_codes.START_OCCUPIED:
                        return totals("lost",
                                      "the rover is off the edge of the costmap, "
                                      "so nothing can plan a route from where it "
                                      "is standing")
                    shuffles += 1
                    escape = self.back_off(say)
                    travelled += float(escape.get("travelled_m") or 0.0)
                    turned += abs(float(escape.get("turned_deg") or 0.0))
                    say("choosing", escape.get("detail") or "",
                        frontiers_left=explorer.summary["frontiers"])
                    if escape.get("reason") != "arrived":
                        return totals("blocked", escape.get("detail")
                                      or "the rover could not shuffle clear")
                    continue

                if chosen is None:
                    # Every candidate this round was refused a route or had
                    # nowhere the body fits. They are all written off now, so the
                    # next round looks at what is left rather than concluding
                    # anything about it from this sample.
                    continue

                candidate, placed, note = chosen
                metres, _degrees = route
                if metres > candidate["distance_m"] * EXPLORE_DETOUR_NOTE:
                    say("choosing",
                        "the way round is %.1f m for a frontier %.1f m off"
                        % (metres, candidate["distance_m"]))
                if note:
                    say("choosing", note)

                # --- drive to it
                #
                # Asked again here rather than only at the top of the loop, and
                # this is the check that matters: everything between the two is
                # the map, the body check and the planner, which is seconds of
                # wall clock with the rover standing still and somebody entirely
                # entitled to press stop during it.
                with self._lock:
                    if self.cancelled or self.stop_seq != stops or self.estop:
                        return totals("stopped", "a stop was asked for before "
                                                 "the rover set off")
                goals += 1
                explorer.committed(candidate["x"], candidate["y"])

                def narrate(phase, why="", _n=goals,
                            _left=explorer.summary["frontiers"], **fields):
                    say(phase, why, frontier_goal=_n, frontiers_left=_left,
                        **fields)

                # The one thing an exploring goal has that a commanded one does
                # not: somewhere else to be. See `frontier.Stall`.
                watch = frontier.Stall()

                def going_nowhere(now, feedback):
                    return watch.update(now, self.pose(),
                                        int(feedback.get("recoveries") or 0))

                outcome = self.goto(
                    (placed[0], placed[1]), math.degrees(placed[2]), narrate,
                    give_up=going_nowhere)
                travelled += float(outcome.get("travelled_m") or 0.0)
                # Summed as magnitudes, unlike every other move here. A single
                # move's turn has a direction worth keeping; a run of nine goals
                # does not, and signed addition reports a rover that turned left
                # ninety degrees and then right ninety as having turned nowhere.
                turned += abs(float(outcome.get("turned_deg") or 0.0))
                reason = outcome.get("reason")
                if reason == "arrived":
                    arrived += 1
                elif outcome.get("detail", "").startswith("it has not got"):
                    # Abandoned by the watcher above rather than by Nav2. Worth
                    # saying out loud on the way past: it is the one outcome here
                    # that means the controller, not the room.
                    say("choosing", "gave that one up -- %s"
                                    % outcome["detail"], frontiers_left=
                                    explorer.summary["frontiers"])
                elif reason == "stopped":
                    return totals("stopped", "a stop was asked for")
                elif reason in ("refused", "lost"):
                    # Not this frontier's fault: the stack is not in a state to
                    # drive anywhere, so trying the next one is trying the same
                    # thing again more slowly.
                    return totals(reason, outcome.get("detail") or
                                  "the stack would not take the goal")
        finally:
            with self._lock:
                self.exploring = False
