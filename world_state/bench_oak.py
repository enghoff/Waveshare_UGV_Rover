#!/usr/bin/env python3
"""Where the OAK is bolted, measured against the camera the rover already trusts.

    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --pan -20 -10 0 10 20'
    ssh orin 'cd ~/ugv/world_state && python3 bench_oak.py --save /tmp/oak-align'

The rover has two cameras that see the room and they have never been in the same
frame. The gimbal camera's optics were swept and fitted on this rover and every
bearing the world state holds was drawn through them; the OAK has sat on the
front measuring millimetres with nothing reading it, and **there are no
extrinsics between it and anything** -- which is the one thing standing between a
range and the world state that wants one. This measures them.

**No target, no tape measure, and nothing to set up.** Both cameras look at
whatever room the rover is in; the perception sidecar finds regions in both
pictures; the regions are matched across the two by the same appearance vectors
the resolver uses, and each matched pair is then one thing seen down two known
lenses from two unknown-but-fixed places. The rotation that makes the two agree
is the mount's yaw and pitch, the spread left over is what the pair of them is
worth, and the ranges the OAK reports for those same regions turn the residual
parallax into the offset between the two lenses.

**What it prints goes into `oak.MOUNT` by hand, with the date.** It is not
written back automatically and should not be: a calibration that rewrites the
constant the whole component depends on, from whatever the rover happened to be
looking at, is a calibration nobody can review. Run it, read the spread, and if
the spread is small enough to believe, paste the numbers in and set
`oak.MEASURED`.

**It wants a room with things in it, at a spread of distances.** Pointed at a
blank wall a metre away it will find nothing to match and say so; the rotation
needs a handful of pairs anywhere in the frame, and the offset needs them at
genuinely different ranges, because what separates a lens two centimetres to the
left from one ten centimetres forward is how the disagreement changes with
distance. The report says which of the two it could and could not determine
rather than printing a number for both regardless.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
import os
import socket
import statistics
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
# The package is imported as a package -- `~/ugv` on the rover, the checkout root
# here -- and `face_tracking/` goes on beside it because `view` reads the gimbal
# camera's swept lens out of it. The same dance `bench_bearing.py` does.
for path in (ROOT, os.path.join(ROOT, "face_tracking")):
    if os.path.isdir(path) and path not in sys.path:
        sys.path.append(path)

from world_state import oak                                       # noqa: E402
from world_state import view                                      # noqa: E402
from world_state.depth_client import SidecarRanger                # noqa: E402
from world_state.perception_client import SidecarEyes             # noqa: E402

#: Where the daemon listens, which is what parks the gimbal and hands over its
#: camera's picture. The camera has one owner and this is it.
DAEMON = ("127.0.0.1", 8769)
DAEMON_TIMEOUT_S = 30.0

#: How alike two crops have to look before they are taken for one thing seen
#: down two lenses. **Higher than `resolve.DIFFERENT_THING`, and deliberately.**
#: There the question is whether two looks *could* be one object and a wrong
#: rejection merely delays a placement; here a wrong match is a bad pair in a
#: calibration that everything else will be built on, and there are usually more
#: honest pairs available than are needed. The two pictures are also of the same
#: room at the same instant from nearly the same place, which is the easiest case
#: appearance ever gets.
SAME_THING = 0.72
#: And how far clear of the runner-up the best match has to be. A region that
#: matches two things nearly equally well is a region whose identity the vectors
#: cannot settle, and one such pair can swing a fit built from six.
LEAD = 0.04
#: The fewest matched pairs worth fitting a rotation from. Three points determine
#: one; below about this the fit is describing the noise on two of them.
MIN_PAIRS = 5
#: And how much spread of range the pairs need before the offset between the two
#: lenses is determined at all, as a ratio of the furthest to the nearest.
#: Everything at one distance is a single parallax reading, which any combination
#: of offset and rotation can explain.
MIN_RANGE_RATIO = 1.8
#: How long to let the gimbal arrive before taking its picture, in seconds.
#: `usb_cameras/calibrate_fov.py` measured what happens with less -- a 25 degree
#: step that has not finished moving reads 20% short -- and this is a bench
#: script with nothing to hurry for.
SETTLE_S = 5.0


def call(name: str, arguments: dict | None = None) -> dict:
    """One control call on the daemon, or a dictionary saying why not."""
    request = json.dumps({"call": name, "arguments": arguments or {}})
    try:
        with socket.create_connection(DAEMON, DAEMON_TIMEOUT_S) as link:
            stream = link.makefile("rwb")
            stream.write(request.encode() + b"\n")
            stream.flush()
            return json.loads(stream.readline())
    except Exception as error:                                    # noqa: BLE001
        return {"ok": False, "error": f"{type(error).__name__}: {error}"}


def gimbal_frame(pan_deg: float) -> dict:
    """Park the gimbal and take one picture through it.

    The pan and tilt come back from the daemon rather than being assumed, because
    this rover's pan servo arrives about three degrees short at the ends of its
    travel with no feedback to correct it -- which is the largest term in any
    bearing it draws. What is read back is what it was *told*, so that error is
    still in here; it is the reason the report says which pans it used.
    """
    aimed = call("look_at", {"pan": pan_deg, "tilt": 0})
    if not aimed.get("ok"):
        return {"ok": False, "error": str(aimed.get("error", "the gimbal refused"))}
    time.sleep(SETTLE_S)
    got = call("camera_jpeg")
    if not got.get("ok") or not got.get("jpeg_base64"):
        return {"ok": False, "error": str(got.get("error", "no picture"))}
    return {"ok": True, "jpeg": base64.b64decode(got["jpeg_base64"]),
            "pan": float(aimed.get("pan", pan_deg)),
            "tilt": float(aimed.get("tilt", 0.0)),
            "size": (int(got.get("width") or 640), int(got.get("height") or 480))}


def unit(blob: bytes):
    """A stored vector as a list of floats scaled to unit length."""
    import numpy

    if not blob:
        return None
    values = numpy.frombuffer(blob, dtype=numpy.float32).astype(numpy.float64)
    length = float(numpy.linalg.norm(values))
    return None if length < 1e-9 else values / length


def pairs_between(here: list, there: list) -> list[tuple[int, int, float]]:
    """Which region of one picture is which region of the other.

    Mutual best match above `SAME_THING` and clear of the runner-up by `LEAD`:
    both directions have to agree, because a wide region that vaguely resembles
    everything will otherwise be somebody's best match in one direction only.
    """
    import numpy

    left = [unit(region.dino) for region in here]
    right = [unit(region.dino) for region in there]
    scores = numpy.full((len(left), len(right)), -1.0)
    for i, one in enumerate(left):
        if one is None:
            continue
        for j, other in enumerate(right):
            if other is None:
                continue
            scores[i, j] = float(one @ other)
    found = []
    for i in range(len(left)):
        row = scores[i]
        if row.max() < SAME_THING:
            continue
        j = int(row.argmax())
        best = float(row[j])
        rest = sorted(row, reverse=True)
        if len(rest) > 1 and best - rest[1] < LEAD:
            continue
        column = scores[:, j]
        if int(column.argmax()) != i:
            continue
        if len(column) > 1 and best - sorted(column, reverse=True)[1] < LEAD:
            continue
        found.append((i, j, best))
    return found


def centre_of(bbox) -> tuple[float, float] | None:
    try:
        left, top, right, bottom = (float(value) for value in bbox)
    except (TypeError, ValueError):
        return None
    return (left + right) / 2.0, (top + bottom) / 2.0


def rotation_between(from_chassis, to_oak):
    """The rotation taking chassis directions to the OAK's optical ones, by SVD.

    Wahba's problem, solved the way everybody solves it: the matrix that best
    lines up two sets of unit vectors is the one from the singular value
    decomposition of their outer product, with the determinant forced positive so
    the answer is a rotation and not a reflection. Written out because it is nine
    lines and the alternative is a dependency this rover does not carry.
    """
    import numpy

    a = numpy.array(from_chassis, dtype=numpy.float64)
    b = numpy.array(to_oak, dtype=numpy.float64)
    u, _s, vt = numpy.linalg.svd(b.T @ a)
    middle = numpy.eye(3)
    middle[2, 2] = numpy.sign(numpy.linalg.det(u @ vt))
    return u @ middle @ vt


def angles_of(matrix) -> tuple[float, float, float]:
    """A rotation as the mount's yaw, pitch and roll, in degrees.

    The convention is `oak._in_oak`'s, read off the row that is the camera's own
    optical axis: yaw positive to the right and pitch positive up, which is what
    the gimbal means by pan and tilt and therefore what an observation stores.
    Roll is the twist left over about that axis, and it is reported rather than
    used -- `oak.Mount` has no roll, on the assumption that a bracket bolted to a
    flat plate does not have one, and this is the number that says whether that
    assumption survives.
    """
    pitch = math.asin(max(-1.0, min(1.0, matrix[2][2])))
    yaw = math.atan2(matrix[2][1], matrix[2][0])
    roll = math.atan2(matrix[0][2], -matrix[1][2])
    return -math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


def offset_between(matrix, chassis, optical, ranges):
    """Where the OAK's lens sits relative to the gimbal camera's, in metres.

    **What is being read is parallax, and parallax is the only thing in these two
    pictures that knows about distance.** With the rotation settled, a thing three
    metres away should appear in exactly the same direction down both lenses; it
    does not, and how much it does not by is the offset divided by the range. So
    a fit over pairs at one distance is hopeless and a fit over pairs at two is
    arithmetic.

    Linear, because the residual is a cross product: the point the OAK measured
    is `offset + q` in the gimbal camera's frame, the gimbal camera says it lies
    along `g`, and `g x (offset + q) = 0` is three equations linear in the
    offset. Stacked over every pair and solved in the least-squares sense, with
    the condition number reported so a caller can see when the pairs did not
    span enough to determine it.
    """
    import numpy

    rows = []
    wanted = []
    for direction, optic, range_m in zip(chassis, optical, ranges):
        if range_m is None:
            continue
        q = numpy.array(matrix, dtype=numpy.float64).T @ (
            numpy.array(optic, dtype=numpy.float64) * float(range_m))
        g = numpy.array(direction, dtype=numpy.float64)
        skew = numpy.array([[0.0, -g[2], g[1]],
                            [g[2], 0.0, -g[0]],
                            [-g[1], g[0], 0.0]])
        rows.append(skew)
        wanted.append(-skew @ q)
    if len(rows) < 3:
        return None, None, 0
    a = numpy.vstack(rows)
    b = numpy.concatenate(wanted)
    solved, _residuals, _rank, singular = numpy.linalg.lstsq(a, b, rcond=None)
    smallest = float(singular[-1]) if len(singular) else 0.0
    condition = (float(singular[0]) / smallest) if smallest > 1e-12 else float("inf")
    return solved, condition, len(rows)


def residuals_of(matrix, chassis, optical) -> list[float]:
    """How far apart the two cameras still put each thing, in degrees."""
    import numpy

    rotation = numpy.array(matrix, dtype=numpy.float64)
    apart = []
    for direction, optic in zip(chassis, optical):
        predicted = rotation @ numpy.array(direction, dtype=numpy.float64)
        dot = float(numpy.clip(predicted @ numpy.array(optic), -1.0, 1.0))
        apart.append(math.degrees(math.acos(dot)))
    return apart


def collect(pans: list[float], eyes, ranger, save: str | None) -> dict:
    """Every matched pair the requested gimbal positions produced."""
    chassis: list[tuple[float, float, float]] = []
    optical: list[tuple[float, float, float]] = []
    ranges: list[float | None] = []
    scores: list[float] = []
    notes: list[str] = []

    lens = ranger.lens()
    if lens is None:
        return {"ok": False, "error": "the depth camera would not say what lens "
                                      "it has; is oak_depth running, and is this "
                                      "a build that serves /health colour?"}
    for pan in pans:
        frame = gimbal_frame(pan)
        if not frame.get("ok"):
            notes.append(f"pan {pan:+.0f}: {frame['error']}")
            continue
        theirs = ranger.frame()
        if not theirs.ok:
            notes.append(f"pan {pan:+.0f}: {theirs.error}")
            continue
        mine = eyes.look(frame["jpeg"])
        yours = eyes.look(theirs.jpeg)
        if not mine.ok or not yours.ok:
            notes.append(f"pan {pan:+.0f}: perception said "
                         f"{mine.error or yours.error}")
            continue
        if save:
            base = f"{save}-pan{int(round(pan)):+03d}"
            with open(base + "-gimbal.jpg", "wb") as handle:
                handle.write(frame["jpeg"])
            with open(base + "-oak.jpg", "wb") as handle:
                handle.write(theirs.jpeg)
        found = pairs_between(mine.regions, yours.regions)
        boxes = [list(yours.regions[j].bbox) for _i, j, _s in found]
        measured, error = ranger.ranges(boxes) if boxes else ([], "")
        if error:
            notes.append(f"pan {pan:+.0f}: no ranges ({error})")
        kept = 0
        for slot, (i, j, score) in enumerate(found):
            here = centre_of(mine.regions[i].bbox)
            there = centre_of(yours.regions[j].bbox)
            if here is None or there is None:
                continue
            direction = view.chassis_direction(here[0], here[1], frame["pan"],
                                               frame["tilt"], frame["size"])
            if direction is None:
                continue
            length = math.sqrt(sum(one * one for one in direction)) or 1.0
            chassis.append(tuple(one / length for one in direction))
            optical.append(oak.ray_at(there[0], there[1], lens))
            one = measured[slot] if slot < len(measured) else None
            ranges.append(None if one is None else one.range_m)
            scores.append(score)
            kept += 1
        notes.append(f"pan {pan:+.0f}: {len(mine.regions)} regions through the "
                     f"gimbal, {len(yours.regions)} through the OAK, "
                     f"{kept} matched")
    return {"ok": True, "chassis": chassis, "optical": optical,
            "ranges": ranges, "scores": scores, "notes": notes, "lens": lens}


def report(gathered: dict) -> int:
    """What was measured, and whether it is worth writing down."""
    chassis = gathered["chassis"]
    optical = gathered["optical"]
    for line in gathered["notes"]:
        print("  " + line)
    print()
    if len(chassis) < MIN_PAIRS:
        print(f"only {len(chassis)} matched pairs, and {MIN_PAIRS} is the fewest "
              f"worth fitting from.")
        print("Point the rover at a room with furniture in it, two to four metres "
              "off, and try again -- a blank wall has nothing to match.")
        return 1

    matrix = rotation_between(chassis, optical)
    yaw, pitch, roll = angles_of(matrix)
    apart = residuals_of(matrix, chassis, optical)
    print(f"{len(chassis)} matched pairs, appearance "
          f"{min(gathered['scores']):.2f} to {max(gathered['scores']):.2f}")
    print()
    print("the rotation between the two cameras")
    print(f"  yaw    {yaw:+7.2f} deg   positive to the right, as the gimbal means pan")
    print(f"  pitch  {pitch:+7.2f} deg   positive up, as the gimbal means tilt")
    print(f"  roll   {roll:+7.2f} deg   not carried by oak.Mount; large means it "
          f"should be")
    print(f"  what is left over: median {statistics.median(apart):.2f} deg, "
          f"worst {max(apart):.2f}")
    print(f"  (a bearing on this rover is believed to "
          f"{_bearing_sigma():.1f} deg, which is what that has to beat)")
    print()

    ranged = [one for one in gathered["ranges"] if one]
    if len(ranged) < 3:
        print("the offset between the two lenses: not measured -- fewer than "
              "three of the matched things had a range.")
        _leftover(yaw, pitch, roll)
        return 0
    spread = max(ranged) / max(min(ranged), 0.01)
    solved, condition, used = offset_between(matrix, chassis, optical,
                                             gathered["ranges"])
    print(f"the offset between the two lenses, from {used} ranged pairs "
          f"{min(ranged):.2f} to {max(ranged):.2f} m out")
    if solved is None:
        print("  not enough ranged pairs to solve for it")
    else:
        print(f"  forward {solved[0]:+.3f} m")
        print(f"  left    {solved[1]:+.3f} m")
        print(f"  up      {solved[2]:+.3f} m")
        print(f"  condition number {condition:.0f}, range spread "
              f"{spread:.1f}x")
        if spread < MIN_RANGE_RATIO or condition > 100:
            print("  **do not believe this**: everything matched was at much the "
                  "same distance, so the parallax cannot tell an offset from a "
                  "rotation. Either find a scene with near and far things in it, "
                  "or measure the offset with a ruler -- it is two brackets on "
                  "one chassis and a ruler is an honest instrument here.")
    print()
    _leftover(yaw, pitch, roll, solved)
    return 0


def _bearing_sigma() -> float:
    try:
        from world_state import locate

        return float(locate.BEARING_SIGMA_DEG)
    except Exception:                                             # noqa: BLE001
        return 1.5


def _leftover(yaw, pitch, roll, offset=None) -> None:
    """The block to paste into `oak.py`, so nobody retypes a number."""
    print("to adopt this, put it in oak.py and set MEASURED = True:")
    print()
    print("    MOUNT = Mount(")
    print(f"        yaw_deg={yaw:.2f},")
    print(f"        pitch_deg={pitch:.2f},")
    if offset is None:
        print("        forward_m=0.0,   # not measured -- see above")
        print("        left_m=0.0,")
        print("        up_m=0.0,")
    else:
        print(f"        forward_m={offset[0]:.3f},")
        print(f"        left_m={offset[1]:.3f},")
        print(f"        up_m={offset[2]:.3f},")
    print("    )")
    print("    MEASURED = True")
    print()
    if abs(roll) > 1.0:
        print(f"and note the roll of {roll:+.2f} deg, which oak.Mount does not "
              f"carry. Above a degree it is worth adding rather than ignoring.")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--pan", type=float, nargs="+", default=[0.0],
                        help="gimbal pan positions to take a pair at "
                             "(default: just 0). More positions means more "
                             "pairs and a fit that is not all from one corner "
                             "of the fisheye")
    parser.add_argument("--save", metavar="PREFIX",
                        help="write both pictures per position, so a person can "
                             "see what was matched")
    args = parser.parse_args()

    print("bench_oak -- where the OAK is, against the camera the rover trusts")
    print()
    eyes = SidecarEyes()
    ready, why = eyes.available()
    if not ready:
        print(f"perception is not there: {why}")
        return 1
    ranger = SidecarRanger()
    gathered = collect(args.pan, eyes, ranger, args.save)
    if not gathered.get("ok"):
        print(gathered["error"])
        return 1
    return report(gathered)


if __name__ == "__main__":
    raise SystemExit(main())
