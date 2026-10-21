#!/usr/bin/env python3
"""Keeping the rover pinned to its map, scan by scan.

Its own module with no ROS in it, for `refit.py`'s reason: the localizer node
runs it on the rover and the selftest and the replay benches run it on a
workstation, and a copy in either would be a copy that drifts.

## Why this exists

The rover's position on the map used to be slam_toolbox's, and slam_toolbox in
mapping mode matches each new scan against the rover's own recent scans. An
error that gets into those -- a few degrees left over from a turn on the spot,
or odometry counting wheel travel the rover never made -- is in the reference
too, so it is carried along instead of taken out, and only an occasional loop
closure snaps it back. On 2026-10-21 that left the rover 4.5 degrees out after
one 127-degree turn and 20-26 cm sideways after 8 m, and 0.8 m out after a wheel
slip it never recovered from; the mapper's distance penalty, which stopped
moved furniture dragging the pose 1.33 m on 2026-10-12, is also what kept it
from pulling the pose back. Matched against the stored map instead -- a
reference that does not move with the rover -- the same recordings held to a
few centimetres. docs/plans/continuous-localization.md has the measurements.

## What it does

`Tracker.update` takes one scan and the odometry pose it was taken at, predicts
where the rover is from the last answer plus odometry's step since, and searches
a small window around that prediction -- `WINDOW_M` and `WINDOW_DEG` -- for the
pose that puts the most of the scan on the map's walls, with `refit.py`'s
field and scorer. The best pose, nudged towards the prediction by a small prior
so that a corridor which fits equally well along its length is not slid down it,
becomes the answer.

**It is a tracker, never a fitter.** The window is the most the answer can move
in one step, so it cannot carry the rover into another room on its own; finding
a rover that has been carried is still a person's act
([R-NAV-3](../docs/requirements/navigation.md#r-nav-3)), and `set_pose` is how a
person's refit or a restore hands it a pose. What it cannot explain -- a scan that
fits nowhere in the window -- it says, and keeps odometry's answer.

**Standing still it does not wander.** A parked rover's odometry does not move,
so the only thing a search can respond to is the room: a person walking past.
Parked, a correction is taken only when it is clearly better than staying put and
the same correction is found `PARKED_VOTES` scans running.
"""

import math

import numpy as np

import refit

#: How far one step may move the answer from odometry's prediction. A step is
#: one processed scan, a fifth of a second at the node's rate; at the rover's
#: 0.45 m/s top speed a 100% wheel slip is 9 cm a step, and the turn left over
#: from a fast spin has measured 8-9 degrees at its worst.
WINDOW_M = 0.25
WINDOW_DEG = 8.0

#: The coarse pass and the fine pass around its winner. Five centimetres is the
#: map's own cell and a degree is about 5 cm at the far wall of a room.
COARSE_STEP_M = 0.05
COARSE_STEP_DEG = 1.0
FINE_SPAN_M = 0.05
FINE_SPAN_DEG = 1.0
FINE_STEP_M = 0.0125
FINE_STEP_DEG = 0.25

#: How far a scan point may be from a wall and still mostly count as on it. Half
#: refit's 10 cm: that search starts up to a metre out and needs a wide basin,
#: this one starts a few centimetres out and wants a sharp peak.
SMEAR_M = 0.06

#: Scan points used per step, spread evenly over the scan. A revolution is 360
#: and the answer barely changes with half of them.
MAX_POINTS = 240

#: The share of the scan that must lie on the map's walls for the answer to be
#: believed at all. Under it the step keeps odometry's prediction and says why.
#: See docs/plans/continuous-localization.md for how it was chosen.
MIN_SCORE = 0.55

#: The pull towards odometry's prediction, as score given up at the edge of the
#: window. Small enough that a real correction always wins, large enough to break
#: the near-ties a corridor or a long wall produces.
PRIOR = 0.02

#: Parked: how much better than staying put a correction must score, and on how
#: many scans running it must be found (within PARKED_AGREE_M and _DEG).
PARKED_GAIN = 0.05
PARKED_VOTES = 3
PARKED_AGREE_M = 0.03
PARKED_AGREE_DEG = 1.0

#: Odometry moving less than this between two steps counts as standing still.
STILL_M = 0.005
STILL_DEG = 0.3

#: Faster than this, a scan swept while turning is smeared past matching (see
#: nav_map.MEASURE_MAX_TURN_DPS); the step keeps odometry's prediction.
MAX_TURN_DPS = 60.0

#: **The most one step may correct, and that is the guard against moved
#: furniture.** A wheel slip at the rover's top speed is 9 cm a step and the gyro
#: is a degree or two out on a fast turn, so a real error arrives a few
#: centimetres at a time. A better fit half a metre away arrives at once, and
#: replayed on the M5 shakedown of 2026-10-12 one such step -- 43 cm, 89% against
#: 64%, among the living room's moved armchairs -- took the answer off the map for
#: good. A bigger correction is taken this much at a time, so a real one is
#: finished within a second and a one-scan coincidence costs this and no more.
#: Scans skipped while turning add `SKIPPED_STEP_*` each, since odometry ran
#: unchecked through them.
MAX_STEP_M = 0.10
MAX_STEP_DEG = 3.0
SKIPPED_STEP_M = 0.05
SKIPPED_STEP_DEG = 2.0

#: **Far returns count for more than near ones.** What moves -- armchairs, bean
#: bags, people -- is mostly near the rover, and what does not -- walls -- is
#: mostly further away. The heading check found the same on 2026-10-12: scored
#: on its returns beyond 1.5 m the pose fitted within 10 cm among staged
#: furniture where the whole scan did not (nav_map.MEASURE_FAR_M).
NEAR_M = 1.5
NEAR_WEIGHT = 0.25


def compose(a, b):
    """`b`, given in `a`'s frame, in `a`'s parent frame. Poses are (x, y, deg)."""
    t = math.radians(a[2])
    c, s = math.cos(t), math.sin(t)
    return (a[0] + c * b[0] - s * b[1], a[1] + s * b[0] + c * b[1],
            (a[2] + b[2] + 180.0) % 360.0 - 180.0)


def between(a, b):
    """`b` in `a`'s frame: the step from `a` to `b`."""
    t = math.radians(a[2])
    c, s = math.cos(t), math.sin(t)
    dx, dy = b[0] - a[0], b[1] - a[1]
    return (c * dx + s * dy, -s * dx + c * dy, (b[2] - a[2] + 180.0) % 360.0 - 180.0)


def _weighted(walls, known, grid, near, far, guess, offsets_x, offsets_y, headings):
    """`refit._scores` over near and far returns separately, combined with near
    returns weighted `NEAR_WEIGHT`. Returns (score, count of far plus near)."""
    shape = (len(headings), len(offsets_y), len(offsets_x))
    total = np.zeros(shape, np.float64)
    weight = np.zeros(shape, np.float64)
    count = np.zeros(shape, np.int64)
    for pts, w in ((far, 1.0), (near, NEAR_WEIGHT)):
        if len(pts) == 0:
            continue
        score, seen = refit._scores(walls, known, grid, pts, guess, offsets_x, offsets_y, headings)
        total += w * score * seen
        weight += w * seen
        count += seen
    return np.where(weight > 0, total / np.maximum(weight, 1e-9), 0.0), count


class Step(object):
    """What one scan did to the answer, and why."""

    def __init__(self, pose, accepted, score, predicted_score, moved_m, turned_deg, why):
        self.pose = pose
        self.accepted = accepted
        self.score = score
        self.predicted_score = predicted_score
        #: How far the answer moved from odometry's prediction on this step.
        self.moved_m = moved_m
        self.turned_deg = turned_deg
        self.why = why

    def as_dict(self):
        return {"x_m": round(self.pose[0], 3), "y_m": round(self.pose[1], 3),
                "heading_deg": round(self.pose[2], 2), "accepted": self.accepted,
                "score": round(self.score, 3), "predicted_score": round(self.predicted_score, 3),
                "moved_m": round(self.moved_m, 3), "turned_deg": round(self.turned_deg, 2),
                "why": self.why}


class Tracker(object):
    """The rover's place on one map, carried from scan to scan."""

    def __init__(self):
        self.grid = None
        self.walls = None
        self.known = None
        self.pose = None
        self.odom = None
        self.odom_t = None
        self.votes = []
        self.skipped = 0

    def set_map(self, grid):
        """Score against this map from now on. The field is rebuilt here and
        nowhere else, so a map republished every two seconds costs one rebuild
        each time and nothing per scan."""
        walls, known = refit.field(grid, SMEAR_M)
        self.grid, self.walls, self.known = grid, walls.reshape(-1), known.reshape(-1)

    def set_pose(self, pose, odom, t=None):
        """Put the rover at `pose` on the map, where odometry reads `odom` --
        a restore, or a refit somebody asked for."""
        self.pose = tuple(float(v) for v in pose)
        self.odom = tuple(float(v) for v in odom)
        self.odom_t = t
        self.votes = []

    def predict(self, odom):
        """Where odometry says the rover is now, from the last answer."""
        return compose(self.pose, between(self.odom, odom))

    def _keep(self, predicted, odom, t, score, predicted_score, why, skipped=False):
        self.pose, self.odom, self.odom_t = predicted, tuple(odom), t
        self.skipped = self.skipped + 1 if skipped else 0
        return Step(predicted, False, score, predicted_score, 0.0, 0.0, why)

    def update(self, points, odom, t=None):
        """One scan: `points` as (n, 2) in the rover's frame, `odom` the odometry
        pose (x, y, deg) at the scan's own time, `t` that time in seconds."""
        if self.pose is None or self.grid is None:
            return None
        odom = tuple(float(v) for v in odom)
        predicted = self.predict(odom)
        step = between(self.odom, odom)
        still = math.hypot(step[0], step[1]) < STILL_M and abs(step[2]) < STILL_DEG
        if t is not None and self.odom_t is not None and t > self.odom_t:
            rate = abs(step[2]) / (t - self.odom_t)
            if rate > MAX_TURN_DPS:
                return self._keep(predicted, odom, t, 0.0, 0.0,
                                  "turning at %.0f deg/s, too fast to match" % rate, skipped=True)
        points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
        if len(points) > MAX_POINTS:
            points = points[np.linspace(0, len(points) - 1, MAX_POINTS).astype(int)]
        if len(points) < refit.MIN_POINTS:
            return self._keep(predicted, odom, t, 0.0, 0.0,
                              "only %d returns, too few to match" % len(points), skipped=True)
        ranges = np.hypot(points[:, 0], points[:, 1])
        near, far = points[ranges < NEAR_M], points[ranges >= NEAR_M]

        offsets = np.arange(-WINDOW_M, WINDOW_M + 1e-9, COARSE_STEP_M)
        turns = np.arange(-WINDOW_DEG, WINDOW_DEG + 1e-9, COARSE_STEP_DEG)
        coarse, seen = _weighted(self.walls, self.known, self.grid, near, far, predicted,
                                 offsets, offsets, predicted[2] + turns)
        coarse = np.where(seen >= refit.MIN_POINTS, coarse, 0.0)
        prior = PRIOR * ((offsets[None, None, :] ** 2 + offsets[None, :, None] ** 2) / WINDOW_M ** 2
                         + (turns[:, None, None] / WINDOW_DEG) ** 2)
        h, r, c = np.unravel_index(int(np.argmax(coarse - prior)), coarse.shape)
        guess = (predicted[0] + offsets[c], predicted[1] + offsets[r], predicted[2] + turns[h])

        fine_offsets = np.arange(-FINE_SPAN_M, FINE_SPAN_M + 1e-9, FINE_STEP_M)
        fine_turns = np.arange(-FINE_SPAN_DEG, FINE_SPAN_DEG + 1e-9, FINE_STEP_DEG)
        fine, fine_seen = _weighted(self.walls, self.known, self.grid, near, far, guess,
                                    fine_offsets, fine_offsets, guess[2] + fine_turns)
        fine = np.where(fine_seen >= refit.MIN_POINTS, fine, 0.0)
        dx = guess[0] - predicted[0] + fine_offsets[None, None, :]
        dy = guess[1] - predicted[1] + fine_offsets[None, :, None]
        dh = guess[2] - predicted[2] + fine_turns[:, None, None]
        fine_prior = PRIOR * ((dx ** 2 + dy ** 2) / WINDOW_M ** 2 + (dh / WINDOW_DEG) ** 2)
        h, r, c = np.unravel_index(int(np.argmax(fine - fine_prior)), fine.shape)
        best = (guess[0] + fine_offsets[c], guess[1] + fine_offsets[r],
                (guess[2] + fine_turns[h] + 180.0) % 360.0 - 180.0)
        score = float(fine[h, r, c])
        predicted_score = float(coarse[len(turns) // 2, len(offsets) // 2, len(offsets) // 2])
        moved = math.hypot(best[0] - predicted[0], best[1] - predicted[1])
        turned = (best[2] - predicted[2] + 180.0) % 360.0 - 180.0

        if score < MIN_SCORE:
            return self._keep(predicted, odom, t, score, predicted_score,
                              "the scan fits the map at %.0f%% at best here, under the "
                              "%.0f%% this needs; odometry kept" % (100 * score, 100 * MIN_SCORE))
        if still:
            if score - predicted_score < PARKED_GAIN:
                self.votes = []
                return self._keep(predicted, odom, t, score, predicted_score,
                                  "standing still and the map agrees")
            self.votes.append(best)
            self.votes = self.votes[-PARKED_VOTES:]
            agree = all(math.hypot(v[0] - best[0], v[1] - best[1]) <= PARKED_AGREE_M
                        and abs((v[2] - best[2] + 180.0) % 360.0 - 180.0) <= PARKED_AGREE_DEG
                        for v in self.votes)
            if len(self.votes) < PARKED_VOTES or not agree:
                return self._keep(predicted, odom, t, score, predicted_score,
                                  "standing still; a correction of %.0f cm seen %d time(s), "
                                  "not yet taken" % (100 * moved, len(self.votes)))
        self.votes = []
        limit_m = MAX_STEP_M + SKIPPED_STEP_M * self.skipped
        limit_deg = MAX_STEP_DEG + SKIPPED_STEP_DEG * self.skipped
        capped = ""
        if moved > limit_m or abs(turned) > limit_deg:
            f = min(1.0, limit_m / moved if moved > 0 else 1.0,
                    limit_deg / abs(turned) if turned else 1.0)
            best = (predicted[0] + f * (best[0] - predicted[0]),
                    predicted[1] + f * (best[1] - predicted[1]),
                    (predicted[2] + f * turned + 180.0) % 360.0 - 180.0)
            capped = "; taken %.0f%% of the way this step" % (100 * f)
        self.skipped = 0
        self.pose, self.odom, self.odom_t = best, odom, t
        return Step(best, True, score, predicted_score, moved, turned,
                    "matched: %.0f%% on the map's walls against %.0f%% at odometry's "
                    "guess, %.0f cm and %.1f deg from it%s"
                    % (100 * score, 100 * predicted_score, 100 * moved, turned, capped))
