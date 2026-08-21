#!/usr/bin/env python3
"""What a journey actually did, kept so it can be argued about on a desk.

A route that comes out convoluted on the rover cannot be diagnosed from the map
image afterwards, and it cannot be reproduced in simulation either, because every
simulation is a guess about the part that is going wrong. There are at least four
candidates and they call for opposite fixes:

  * the planner drew a bad route through a fine map
  * the map was ragged, so the only route through it was bad
  * the route was fine and the follower wove along it
  * the rover drove straight and the *pose* moved, so the trail is a drawing of
    the estimate rather than of the rover

The last one is not exotic on this machine -- most of the recent work on `slam2d`
is about poses that are confidently wrong -- and nothing downstream can tell it
apart from the others. So this records the inputs and the decisions rather than
the conclusions, and `python3 journey.py <file>` reads them back.

**Recording is switched on by making the directory**, and off by removing it:

    ssh rpi 'mkdir -p ~/ugv/journeys'      # record the next few drive_to calls
    ssh rpi 'rm -rf ~/ugv/journeys'        # stop

That is the whole control surface. It needs no flag, no restart and no new tool,
which matters because the daemon's arguments live in a crontab entry and
relaunching it by hand is how the rover silently loses them.

Nothing here may throw into the control loop. A diagnostic that can stop the
rover is worse than no diagnostic, so every entry point swallows its own errors
and sets `broken` instead -- see `_guard`.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time

DEFAULT_DIR = os.path.expanduser("~/ugv/journeys")
KEEP = 5                   # journeys on disk before the oldest is dropped
MAX_TICKS = 4000           # 400 s at 10 Hz; a journey is capped at 75
MAX_PLANS = 40             # one per replan, and MAX_REPLANS is 8

#: One row per revolution. Names are the file format -- the replay reads them
#: from the file rather than assuming this order, so adding a column here does
#: not invalidate journeys already recorded.
TICK_FIELDS = (
    "t",                   # seconds since the journey began
    "x", "y", "th",        # the pose the follower was handed, radians
    "score",               # how well the scan fit, 0..1
    "ambiguity",           # was there an equally good answer elsewhere
    "edge",                # did the winner sit against the rim of the window
    "rejected",            # was the match thrown away
    "map_ok",              # was this a pose worth writing the map from
    "measured_speed",      # what the matcher says the rover is doing
    "measured_turn",
    "want_speed",          # what the follower asked for
    "want_turn",
    "chosen_deg",          # follow-the-gap's answer, relative to the nose
    "clearance",           # and the room it found there
    "progress",            # metres along the route
    "cross",               # metres off it
)


def _guard(method):
    """A recorder that fails must go quiet, not take the move down with it."""
    def wrapper(self, *args, **kwargs):
        if self.broken:
            return None
        try:
            return method(self, *args, **kwargs)
        except Exception as exc:                          # noqa: BLE001
            self.broken = f"{type(exc).__name__}: {exc}"
            return None
    wrapper.__name__ = method.__name__
    wrapper.__doc__ = method.__doc__
    return wrapper


class Recorder:
    """One journey's inputs and decisions, in memory until it ends."""

    def __init__(self, directory=DEFAULT_DIR, keep=KEEP):
        self.directory = directory
        self.keep = keep
        self.broken = None
        self.started_at = None
        self.meta = {}
        self.plans = []        # dicts, each with an index into self.grids
        self.grids = []        # the occupancy grid each plan was handed
        self.ticks = []        # rows in TICK_FIELDS order
        self.events = []       # (t, kind, detail)

    @classmethod
    def if_armed(cls, directory=DEFAULT_DIR, keep=KEEP):
        """A Recorder if the directory is there, else None. Making the directory
        is how recording is switched on -- see the module docstring."""
        try:
            if os.path.isdir(directory):
                return cls(directory, keep)
        except OSError:
            pass
        return None

    # --- recording ------------------------------------------------------------

    @_guard
    def begin(self, kind, asked, pose):
        self.started_at = time.monotonic()
        self.meta = {"kind": kind, "asked": asked,
                     "start_pose": [float(v) for v in pose],
                     "wall_clock": time.strftime("%Y-%m-%dT%H:%M:%S")}

    @_guard
    def plan(self, grid, pose, target, inflate_m, preferred_m, comfort_m,
             path, why, seconds):
        """One call to the planner: what it was handed, and what it said.

        The grid is kept by reference. The caller has already copied it out from
        under the SLAM lock to plan on, and it does not touch it again, so this
        costs the memory and not the copy.
        """
        if len(self.plans) >= MAX_PLANS:
            return
        self.plans.append({
            "t": self._now(),
            "grid": len(self.grids),
            "pose": [float(v) for v in pose],
            "target": [float(v) for v in target],
            "inflate_m": inflate_m,
            "preferred_m": preferred_m,
            "comfort_m": comfort_m,
            "path": None if path is None else [[float(x), float(y)]
                                               for x, y in path],
            "why": why,
            "seconds": round(seconds, 3),
        })
        self.grids.append(grid)

    @_guard
    def tick(self, pose, health, measured, wanted, chosen, clearance,
             progress, cross):
        if len(self.ticks) >= MAX_TICKS:
            return
        self.ticks.append([
            self._now(),
            pose[0], pose[1], pose[2],
            health.get("score", 0.0),
            health.get("ambiguity", 0.0),
            1.0 if health.get("edge") else 0.0,
            1.0 if health.get("rejected") else 0.0,
            1.0 if health.get("map_ok") else 0.0,
            measured[0], measured[1],
            wanted[0], wanted[1],
            chosen if chosen is not None else float("nan"),
            clearance if clearance is not None else float("nan"),
            progress, cross,
        ])

    @_guard
    def event(self, kind, detail=""):
        self.events.append([self._now(), kind, str(detail)[:200]])

    @_guard
    def end(self, reason, detail=""):
        """Close the journey and write it, on a thread so the caller's tool call
        does not wait for an SD card."""
        self.event("end", f"{reason}: {detail}".strip(": "))
        self.meta["reason"] = reason
        self.meta["detail"] = detail
        thread = threading.Thread(target=self._write, daemon=True,
                                  name="journey-write")
        thread.start()
        return thread

    def _now(self):
        return 0.0 if self.started_at is None else time.monotonic() - self.started_at

    # --- the file -------------------------------------------------------------

    def _write(self):
        try:
            import numpy as np

            os.makedirs(self.directory, exist_ok=True)
            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = os.path.join(self.directory, f"journey-{stamp}.npz")
            grids = (np.stack([np.asarray(g) for g in self.grids])
                     if self.grids else np.zeros((0, 0, 0), dtype=np.int8))
            np.savez_compressed(
                path,
                grids=grids,
                ticks=np.asarray(self.ticks, dtype=np.float32).reshape(
                    (len(self.ticks), len(TICK_FIELDS))),
                header=np.frombuffer(json.dumps({
                    "meta": self.meta, "plans": self.plans,
                    "events": self.events, "tick_fields": list(TICK_FIELDS),
                }).encode(), dtype=np.uint8),
            )
            self._prune()
        except Exception as exc:                          # noqa: BLE001
            self.broken = f"write failed: {type(exc).__name__}: {exc}"

    def _prune(self):
        names = sorted(n for n in os.listdir(self.directory)
                       if n.startswith("journey-") and n.endswith(".npz"))
        for name in names[:-self.keep] if self.keep else []:
            try:
                os.remove(os.path.join(self.directory, name))
            except OSError:
                pass


# --- reading one back ----------------------------------------------------------

def load(path):
    """A recorded journey as plain Python, plus the grids as an array."""
    import numpy as np

    with np.load(path, allow_pickle=False) as data:
        header = json.loads(bytes(data["header"]).decode())
        return {
            "meta": header["meta"],
            "plans": header["plans"],
            "events": header["events"],
            "fields": header["tick_fields"],
            "ticks": data["ticks"],
            "grids": data["grids"],
        }


def column(journey, name):
    return journey["ticks"][:, journey["fields"].index(name)]


def pose_jumps(journey, tolerance_m=0.04):
    """Revolutions where the pose moved further than the wheels could have.

    This is the question the map image cannot answer. Between two revolutions the
    rover can only have travelled about `measured_speed * dt`, and the follower
    only asked for `want_speed * dt`. A pose that has stepped much further than
    the larger of those did not come from driving -- it came from the matcher
    changing its mind, and a trail drawn from it will show a kink the rover never
    made.

    Returns one row per suspect revolution: (t, jump_m, explainable_m, score,
    ambiguity). `tolerance_m` is the matcher's own resolution, which is a couple
    of centimetres, plus room for a revolution's honest noise.
    """
    t = column(journey, "t")
    x, y = column(journey, "x"), column(journey, "y")
    want = column(journey, "want_speed")
    measured = column(journey, "measured_speed")
    score = column(journey, "score")
    ambiguity = column(journey, "ambiguity")
    out = []
    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        if dt <= 0.0 or dt > 1.0:
            continue
        jump = math.hypot(float(x[i] - x[i - 1]), float(y[i] - y[i - 1]))
        explainable = max(abs(float(want[i - 1])),
                          abs(float(measured[i - 1]))) * dt + tolerance_m
        if jump > explainable:
            out.append((float(t[i]), jump, explainable,
                        float(score[i]), float(ambiguity[i])))
    return out


#: Events after which the heading is *supposed* to move without the wheels having
#: turned: a burst runs with matching suspended and ends by telling the matcher
#: where it is. A revolution spanning one of these is not evidence of anything.
RESEED_EVENTS = ("burst", "map held", "map resumed")


def heading_jumps(journey, tolerance_deg=6.0):
    """The same question for heading, which is the one that bends a trail.

    Revolutions that span a dead-reckoned burst are skipped, and skipping them is
    the whole difficulty of asking this question here. A burst suspends matching
    and re-seeds the heading afterwards, so the pose legitimately steps by most of
    a burst -- 53 degrees was observed across a 60 degree one, and counting that
    as a jump makes every healthy turn look broken. What is left after the skip is
    the heading moving when nothing asked it to, which is the real thing.
    """
    t = column(journey, "t")
    th = column(journey, "th")
    want = column(journey, "want_turn")
    measured = column(journey, "measured_turn")
    reseeds = [at for at, kind, _detail in journey["events"]
               if any(kind.startswith(e) for e in RESEED_EVENTS)]
    out = []
    for i in range(1, len(t)):
        dt = float(t[i] - t[i - 1])
        if dt <= 0.0 or dt > 1.0:
            continue
        if any(t[i - 1] <= at <= t[i] for at in reseeds):
            continue
        step = abs(math.degrees(
            (float(th[i]) - float(th[i - 1]) + math.pi) % (2 * math.pi) - math.pi))
        explainable = max(abs(float(want[i - 1])),
                          abs(float(measured[i - 1]))) * dt + tolerance_deg
        if step > explainable:
            out.append((float(t[i]), step, explainable))
    return out


def bursts(journey):
    """Each dead-reckoned burst: what it asked for and what the matcher saw.

    The cumulative figure is what the burst loop closes on, so the delivered
    angle is the difference between consecutive ones. A rate that no longer
    describes this floor shows up as the same sign of error on every burst; a
    matcher still settling shows up as errors that change sign.
    """
    out, previous = [], 0.0
    for at, kind, detail in journey["events"]:
        if kind != "burst":
            continue
        try:
            asked = float(detail.split("asked ")[1].split(" deg")[0])
            far = float(detail.split("matcher says ")[1].split(" of")[0])
            pwm = int(detail.split("PWM ")[1].split(",")[0])
        except (IndexError, ValueError):
            continue
        delivered = far - previous
        # Positive means it went further than asked, in the direction it was
        # asked -- so a left and a right turn that both fall short agree.
        along = 1.0 if asked >= 0 else -1.0
        out.append({"t": at, "pwm": pwm, "asked": asked,
                    "delivered": delivered, "cumulative": far,
                    "shortfall": (delivered - asked) * along})
        previous = far
    return out


def turning(points):
    """Degrees of heading change along a polyline."""
    total = 0.0
    for (ax, ay), (bx, by), (cx, cy) in zip(points, points[1:], points[2:]):
        h0 = math.atan2(by - ay, bx - ax)
        h1 = math.atan2(cy - by, cx - bx)
        total += abs(math.degrees((h1 - h0 + math.pi) % (2 * math.pi) - math.pi))
    return total


def replan_planner(journey, plan_fn):
    """Run a planner again on every map this journey was really handed.

    This is the point of keeping the grids. A change to `planner.py` can be tried
    against the rooms that actually produced a bad route, rather than against
    rooms invented to look like them.
    """
    out = []
    for entry in journey["plans"]:
        grid = journey["grids"][entry["grid"]]
        path, why = plan_fn(grid, entry["pose"], entry["target"],
                            entry["inflate_m"], entry["preferred_m"],
                            entry["comfort_m"])
        was = entry["path"]
        out.append({
            "t": entry["t"],
            "before": None if was is None else {
                "waypoints": len(was), "turn": round(turning(was), 1)},
            "after": None if path is None else {
                "waypoints": len(path), "turn": round(turning(path), 1)},
            "why_before": entry["why"],
            "why_after": why,
        })
    return out


def report(journey):
    """The timeline, and the one number that says whether to look at the pose."""
    lines = []
    meta = journey["meta"]
    lines.append(f"{meta.get('kind')} {meta.get('asked')} "
                 f"at {meta.get('wall_clock')}")
    lines.append(f"ended: {meta.get('reason')} -- {meta.get('detail')}")
    lines.append("")

    ticks = journey["ticks"]
    if len(ticks):
        t = column(journey, "t")
        lines.append(f"{len(ticks)} revolutions over {t[-1]:.1f} s")
        ok = column(journey, "map_ok")
        rej = column(journey, "rejected")
        lines.append(f"  match: mean score {column(journey, 'score').mean():.2f}, "
                     f"{int((1 - ok).sum())} revolutions not fit to map from, "
                     f"{int(rej.sum())} rejected outright")

    lines.append("")
    lines.append("routes:")
    for i, entry in enumerate(journey["plans"]):
        if entry["path"] is None:
            lines.append(f"  {entry['t']:6.1f}s  refused: {entry['why']}  "
                         f"({entry['seconds']:.1f} s to decide)")
        else:
            lines.append(f"  {entry['t']:6.1f}s  {len(entry['path'])} waypoints, "
                         f"{turning(entry['path']):.0f} deg of turning, "
                         f"{entry['seconds']:.1f} s to plan")

    lines.append("")
    lines.append("what happened:")
    for at, kind, detail in journey["events"]:
        lines.append(f"  {at:6.1f}s  {kind}: {detail}" if detail
                     else f"  {at:6.1f}s  {kind}")

    burst_rows = bursts(journey)
    if burst_rows:
        lines.append("")
        lines.append("dead-reckoned bursts (delivered is what the matcher saw):")
        for b in burst_rows:
            lines.append(f"  {b['t']:6.1f}s  PWM {b['pwm']:3d}  asked {b['asked']:+6.1f}"
                         f"  delivered {b['delivered']:+6.1f}  "
                         f"({b['shortfall']:+.1f} short)")
        # Signed *along the direction asked*, or a left turn that under-delivers
        # and a right turn that under-delivers look like opposite faults.
        signs = {1 if b["shortfall"] > 1.0 else -1 if b["shortfall"] < -1.0 else 0
                 for b in burst_rows}
        if signs <= {-1, 0}:
            worst = min(b["shortfall"] / abs(b["asked"]) for b in burst_rows
                        if abs(b["asked"]) > 5.0)
            lines.append(f"  every burst fell short in the direction it was asked "
                         f"for, by up to {abs(worst) * 100:.0f}% -- which is what a "
                         f"stale TURN_RATES looks like, so recalibrate")
        elif signs <= {1, 0}:
            lines.append("  every burst overshot in the direction it was asked for, "
                         "which is a stale TURN_RATES the other way -- recalibrate")
        else:
            lines.append("  the errors change sign, so this is not a rate that has "
                         "gone stale; suspect the heading still settling")

    jumps = pose_jumps(journey)
    turns = heading_jumps(journey)
    lines.append("")
    lines.append("did the rover move, or did the estimate?")
    if not len(ticks):
        lines.append("  no revolutions recorded")
    else:
        worst = max((j[1] for j in jumps), default=0.0)
        lines.append(f"  {len(jumps)} of {len(ticks)} revolutions moved the pose "
                     f"further than the wheels could have, worst {worst * 100:.0f} cm")
        worst_t = max((j[1] for j in turns), default=0.0)
        lines.append(f"  {len(turns)} turned it further than commanded outside a "
                     f"re-seed, worst {worst_t:.0f} deg")
        if jumps:
            lines.append("  the biggest, with the match that produced them:")
            for at, jump, allowed, score, amb in sorted(
                    jumps, key=lambda r: -r[1])[:5]:
                lines.append(f"    {at:6.1f}s  {jump * 100:5.1f} cm where "
                             f"{allowed * 100:4.1f} was possible, "
                             f"score {score:.2f}, ambiguity {amb:.2f}")
    return "\n".join(lines)


def _selftest():
    import tempfile

    import numpy as np

    with tempfile.TemporaryDirectory() as tmp:
        armed = os.path.join(tmp, "journeys")
        assert Recorder.if_armed(armed) is None, (
            "recording started without the directory being made")
        os.makedirs(armed)
        rec = Recorder.if_armed(armed)
        assert rec is not None, "making the directory did not arm the recorder"

        rec.begin("drive_to", {"ahead_m": 2.0, "left_m": 0.0}, (0.0, 0.0, 0.0))
        grid = np.full((40, 40), -10, dtype=np.int8)
        rec.plan(grid, (0.0, 0.0, 0.0), (2.0, 0.0), 0.25, 0.45, 0.55,
                 [(0.0, 0.0), (1.0, 0.4), (2.0, 0.0)], None, 0.8)

        health = {"score": 0.9, "ambiguity": 0.1, "edge": False,
                  "rejected": False, "map_ok": True}
        # A rover driving honestly: 2 cm a revolution at 0.2 m/s.
        for k in range(10):
            rec.ticks.append([k * 0.1, k * 0.02, 0.0, 0.0, 0.9, 0.1, 0.0, 0.0, 1.0,
                              0.2, 0.0, 0.2, 0.0, 0.0, 1.5, k * 0.02, 0.01])
        # ...and then the matcher changes its mind by 30 cm without the wheels
        # having turned, which is the thing this file exists to catch.
        rec.ticks.append([1.0, 0.48, 0.0, 0.0, 0.88, 0.7, 0.0, 0.0, 1.0,
                          0.2, 0.0, 0.2, 0.0, 0.0, 1.5, 0.48, 0.01])
        rec.event("replan", "drifted 0.61 m off the route")
        rec.end("arrived", "").join(timeout=10.0)
        assert rec.broken is None, rec.broken

        files = sorted(os.listdir(armed))
        assert len(files) == 1, files
        back = load(os.path.join(armed, files[0]))
        assert back["meta"]["reason"] == "arrived", back["meta"]
        assert back["grids"].shape == (1, 40, 40), back["grids"].shape
        assert len(back["ticks"]) == 11, len(back["ticks"])
        assert back["plans"][0]["path"][1] == [1.0, 0.4], back["plans"][0]

        jumps = pose_jumps(back)
        assert len(jumps) == 1, f"expected the one impossible step, got {jumps}"
        assert 0.25 < jumps[0][1] < 0.32, jumps
        # Two legs at +-21.8 degrees off the straight line, so 43.6 of turning.
        assert abs(turning(back["plans"][0]["path"]) - 43.6) < 0.1, (
            turning(back["plans"][0]["path"]))

        text = report(back)
        assert "did the rover move, or did the estimate?" in text
        assert "1 of 11 revolutions" in text, text

        # Replaying a planner over the recorded maps is the whole point of
        # keeping them.
        def straight_line(grid_, pose, target, inflate_m, preferred_m, comfort_m):
            return [tuple(pose[:2]), tuple(target)], None

        again = replan_planner(back, straight_line)
        assert again[0]["before"]["waypoints"] == 3, again
        assert again[0]["after"]["waypoints"] == 2, again

        # A recorder that hits trouble goes quiet rather than taking a move down.
        rec2 = Recorder(armed)
        rec2.begin("drive_to", {}, (0.0, 0.0, 0.0))
        rec2.tick(None, {}, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0, 0.0, 0.0)
        assert rec2.broken, "a bad tick did not disable the recorder"
        assert rec2.event("anything") is None, "a broken recorder kept recording"

        # Keeping only the last few, so a week of driving cannot fill the card.
        rec3 = Recorder(armed, keep=2)
        for k in range(4):
            open(os.path.join(armed, f"journey-2000010{k}-000000.npz"), "w").close()
        rec3._prune()
        left = sorted(n for n in os.listdir(armed) if n.startswith("journey-"))
        assert len(left) == 2, left

    print("journey: ok")
    return 0


def main(argv=None):
    import sys

    argv = sys.argv[1:] if argv is None else argv
    if not argv:
        return ("usage: journey.py <journey-*.npz> [--replan]\n"
                f"       recording is armed by making {DEFAULT_DIR}")
    path = argv[0]
    data = load(path)
    print(report(data))
    if "--replan" in argv:
        import planner

        def with_current_planner(grid, pose, target, inflate_m, preferred_m,
                                 comfort_m):
            return planner.plan(grid, 0.05, 20, (pose[0], pose[1]), target,
                                inflate_m=inflate_m, preferred_m=preferred_m,
                                comfort_m=comfort_m, start_yaw=pose[2])

        print()
        print("the same maps, planned again with the planner in this checkout:")
        for row in replan_planner(data, with_current_planner):
            before = row["before"] or row["why_before"]
            after = row["after"] or row["why_after"]
            print(f"  {row['t']:6.1f}s  {before}  ->  {after}")
    return 0


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        raise SystemExit(_selftest())
    raise SystemExit(main())
