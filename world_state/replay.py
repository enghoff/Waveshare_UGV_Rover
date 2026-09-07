#!/usr/bin/env python3
"""Replay a recorded world, so a change to identity can be judged before it flies.

    scp orin:'~/.ugv/world/world.db' /tmp/run.db
    scp orin:'~/.ugv/world/frames/*.jpg' /tmp/frames/
    python3 world_state/replay.py /tmp/run.db --frames /tmp/frames --detail

**Why this exists.** Every rule in `resolve.py` is a judgement about real rooms,
and the selftests exercise all of them against a fake that was written to agree
with the rule. A change that looks obviously right there can be worthless or
harmful on a recording of what the rover actually saw, and until this existed
there was no way to find that out short of another drive.

What it does is small and exact: it takes the observations out of a database the
rover wrote, feeds them back to a fresh store one inspection at a time in the
order they were recorded, and calls the live resolver after each one -- which is
what `Inspector` does. Nothing is re-perceived; the vectors, boxes, poses and
gimbal angles are the rover's own. **The check that makes it worth anything is
that replaying an unchanged build reproduces the entities the rover ended up
with, exactly.** It does: the drive of 2026-09-02 comes back as the same 23
entities with the same positions and the same observation counts.

Two scores come out, and neither asks the resolver to mark its own homework.

    mixed   how many of an entity's crops belong to something other than the
            biggest thing in it, by single-link clustering of the stored DINOv2
            vectors at 0.55 -- above what two regions of one frame score (median
            0.32) and below what one object scores across a real change of
            viewpoint (0.70). This is the number that answers "is this entity one
            thing".
    stray   how many of an entity's own bearings miss its own stated position by
            more than bearing error plus its own uncertainty allows. Pure
            geometry, and it answers "does this entity agree with its evidence".

`--frames` additionally rejects the regions with no picture in them the way
`perceive._blank` now does, so a recording taken before that filter existed can
be replayed as though it had been. `--no-untrusted-pose` drops the observations
taken at exactly the map origin, which is what the old `_world_pose` recorded
when the daemon had restarted and nobody had opened the map.

Runs at a desk. Nothing here touches the rover, and it needs numpy only for the
scoring.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import shutil
import sqlite3
import sys
import tempfile

if __package__ in (None, ""):                       # run as a script
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    __package__ = "world_state"

from . import resolve as resolver                   # noqa: E402
from . import view                                  # noqa: E402
from .store import WorldStore                       # noqa: E402

#: The columns an observation is replayed with. Everything the rover measured,
#: and deliberately not `entity_id`: what the resolver decided last time is the
#: thing being re-decided.
COLUMNS = (
    "inference_id observed_at source frame_id frame_path bbox_json "
    "observer_pan_deg observer_tilt_deg observer_pose_json map_session "
    "model_id raw_json bearing_deg span_deg origin_sigma_m bearing_sigma_deg "
    "elevation_deg elevation_span_deg "
    "range_m range_sigma_m camera "
    "region_source region_score "
    "dino_blob siglip_blob vectors_from dino_alone_blob mask_share"
).split()

#: What the depth camera measured, and the only columns this harness will drop on
#: purpose. `--no-ranges` blanks them, which is the whole of how one recording
#: taken with the OAK awake is replayed as though it had been taken without it.
#: That comparison is what M0 of `docs/plans/autonomous-curiosity.md` asks for,
#: and it is the only honest way to ask whether measured distance helps or harms
#: identity: two different drives differ by the room as well as by the camera.
#:
#: **They were missing from `COLUMNS` until 2026-09-07**, which is worth saying
#: plainly rather than quietly adding: every replay result quoted before that
#: date is a bearing-only result whatever the rover was doing when it recorded
#: the run. The drive of 2026-09-07 carries 583 ranges and replayed without one
#: of them, so "an unchanged build reproduces the entities the rover ended up
#: with" was a claim about a resolver running on less evidence than the rover's.
RANGE_COLUMNS = ("range_m", "range_sigma_m")

#: What two crops have to score to be called the same thing when the entities are
#: being marked. Between the two numbers this rover measured -- two regions of one
#: frame, which are different things by construction, score 0.32 in the median and
#: 0.69 at the 95th; one object across a real change of viewpoint scores 0.70.
JOIN = 0.55
#: A pose of exactly this is not a measurement. It is what the transform tree
#: answers when slam_toolbox has just started, and what the old `_world_pose`
#: recorded for it.
ORIGIN = '{"x_m": 0.0, "y_m": 0.0, "heading_deg": 0.0}'


# --- reading the recording ---------------------------------------------------

def inspections(path: str) -> list[list[dict]]:
    """The recorded observations, grouped by the look that took them, in order."""
    db = sqlite3.connect(path)
    db.row_factory = sqlite3.Row
    rows = [dict(row) for row in db.execute(
        "SELECT * FROM observations ORDER BY observed_at, id")]
    db.close()
    groups: dict = {}
    order: list = []
    for row in rows:
        key = row["inference_id"]
        if key not in groups:
            groups[key] = []
            order.append(key)
        groups[key].append(row)
    return [groups[key] for key in order]


def remeasure(groups: list[list[dict]], size: tuple[int, int] | None = None
              ) -> tuple[int, float]:
    """Work every recorded bearing out again through today's model, in place.

    **The gap this closes was written up before it was needed.** `resolve.ray_of`
    reads the bearing off the row rather than deriving it from the box --
    deliberately, because a bearing is a measurement taken at the moment of the
    look and a lens refitted afterwards must not silently rewrite the rover's
    history. The cost of that is that a recording replays with the bearings it
    was recorded with whatever `view` now says, so the harness that exists to
    judge a change to identity could not judge a change to *this*.

    So it is asked for explicitly and only here, at a desk, on a copy. The box,
    the pose and the gimbal angles are still the rover's own; what is recomputed
    is only the arithmetic between them. Answers how many rows changed and by how
    much in the median, because "nothing moved" is the result that says the
    recording was taken after the change rather than before it.
    """
    moved = []
    for group in groups:
        for row in group:
            pose = row.get("observer_pose_json")
            try:
                pose = json.loads(pose) if pose else None
            except (TypeError, ValueError):
                pose = None
            # Deliberately without the bearing the row already carries: `ray`
            # hands a stored one straight back, which is exactly the behaviour
            # this function exists to step around. `fov_deg` is the switch that
            # says a camera was known and no longer does any arithmetic.
            drawn = view.ray({"pose": pose,
                              "bbox": json.loads(row["bbox_json"])
                              if row.get("bbox_json") else None,
                              "observer_pan_deg": row.get("observer_pan_deg"),
                              "observer_tilt_deg": row.get("observer_tilt_deg")},
                             fov_deg=1.0, size=size)
            if drawn is None:
                continue
            was = row.get("bearing_deg")
            row["bearing_deg"] = drawn["bearing_deg"]
            row["span_deg"] = drawn["span_deg"]
            # The vertical half of the same ray, and **the only way a recording
            # taken before it was kept can be asked about it at all**: the
            # column did not exist, so every row carries null and the geometry
            # rightly abstains. Worked out here from the box, the tilt and the
            # lens the rover actually had, which are all still on the row.
            row["elevation_deg"] = drawn["elevation_deg"]
            row["elevation_span_deg"] = drawn["elevation_span_deg"]
            if was is not None:
                moved.append(abs(_wrap(float(was) - drawn["bearing_deg"])))
    moved.sort()
    return len(moved), (moved[len(moved) // 2] if moved else 0.0)


def blank_ids(path: str, frames_dir: str) -> set:
    """Which recorded regions have no picture in them, by `perceive._blank`.

    Read off the stored frames rather than from a column, because the recording
    predates the filter. Needs numpy and Pillow; without either it says nothing
    rather than guessing.
    """
    try:
        import numpy as np
        from PIL import Image
    except ImportError:
        print("  (--frames needs numpy and Pillow; not filtering blank regions)")
        return set()
    from .perceive import MAX_BLOWN, MIN_CONTRAST

    db = sqlite3.connect(path)
    db.row_factory = sqlite3.Row
    grey: dict = {}
    doomed = set()
    for row in db.execute("SELECT id, frame_id, bbox_json FROM observations"):
        name = row["frame_id"]
        if name not in grey:
            picture = os.path.join(frames_dir, f"{name}.jpg")
            if not os.path.exists(picture):
                continue
            grey[name] = np.asarray(Image.open(picture).convert("L"),
                                    dtype="float32")
        image = grey.get(name)
        if image is None:
            continue
        height, width = image.shape
        try:
            left, top, right, bottom = json.loads(row["bbox_json"])
        except (TypeError, ValueError):
            continue
        crop = image[int(top * height):max(int(bottom * height), int(top * height) + 2),
                     int(left * width):max(int(right * width), int(left * width) + 2)]
        if crop.size and (float(crop.std()) < MIN_CONTRAST
                          or float((crop > 250).mean()) > MAX_BLOWN):
            doomed.add(row["id"])
    db.close()
    return doomed


def reach_from(path: str):
    """A range bound out of a saved occupancy grid, or None.

    The grid is whatever the nav bridge answers `{"op": "map"}` with, saved as
    JSON -- one short script on the rover against loopback 8773, which the
    component README spells out.

    A copy of `rover_world._world_reach`, deliberately: the daemon's runs inside
    the process that owns the camera and answers from a live grid, this one
    answers from a file at a desk, and neither host would import a shared module
    the same way. Two short walks over an array is the cheaper thing to keep in
    step.
    """
    import base64
    import zlib
    try:
        import numpy as np
    except ImportError:
        print("  (--map needs numpy; bearings will be unbounded)")
        return None
    if not os.path.exists(path):
        print(f"  (no such map: {path}; bearings will be unbounded)")
        return None
    payload = json.load(open(path))
    width, height = int(payload["width"]), int(payload["height"])
    resolution = float(payload["resolution_m"])
    origin_x, origin_y = float(payload["origin_x_m"]), float(payload["origin_y_m"])
    cells = np.frombuffer(zlib.decompress(base64.b64decode(payload["data"])),
                          dtype=np.int8).reshape(height, width)
    step = resolution / 2.0

    def reach(x_m, y_m, bearing_deg):
        dx = math.cos(math.radians(bearing_deg)) * step
        dy = math.sin(math.radians(bearing_deg)) * step
        got = 0.0
        for count in range(1, int(12.0 / step) + 1):
            ix = math.floor((x_m + dx * count - origin_x) / resolution)
            iy = math.floor((y_m + dy * count - origin_y) / resolution)
            if not (0 <= ix < width and 0 <= iy < height):
                break
            if cells[iy, ix] >= 50:
                break
            got = step * count
        return got

    return reach


# --- the replay itself -------------------------------------------------------

def replay(path: str, skip: set | None = None, drop_untrusted: bool = False,
           verbose: bool = False, reach=None,
           groups: list[list[dict]] | None = None,
           drop_ranges: bool = False
           ) -> tuple[list[dict], list[dict]]:
    """Feed a recording back through the live resolver, from an empty world.

    `reach` is the same callable the daemon hands the resolver -- how far the
    rover could see from a place in a direction. `--map` builds one from a grid
    fetched off the rover; without it every bearing is unbounded, which is what
    the resolver did before it could ask.

    **The map sessions on the rows are followed, and that is the whole of how a
    map change gets replayed.** Every look the rover took carries the map it was
    taken under, and this used to read that off the first look and hold it for
    the rest of the run -- so a recording spanning two maps replayed as the first
    one with every later look silently dropped, because the pending pool is
    filtered by session. What happens when the map changes underneath a world
    full of things is not a rare case on this rover, and it was the one case the
    harness could not show. `--session` replays a single map where that is what
    is wanted.

    `drop_ranges` throws away what the depth camera measured, so that a single
    recording can be replayed both ways. Bearing-only is the harder problem and
    the one every entity this store held before the OAK was wired in was built
    under; carrying the ranges is what the rover itself does.
    """
    skip = skip or set()
    groups = inspections(path) if groups is None else groups
    directory = tempfile.mkdtemp(prefix="world-replay-")
    try:
        store = WorldStore(directory)
        session = None
        for group in groups:
            wanted = [row for row in group if row["id"] not in skip
                      and not (drop_untrusted
                               and row["observer_pose_json"] == ORIGIN)]
            if not wanted:
                continue
            if wanted[0]["map_session"] != session:
                session = wanted[0]["map_session"]
                store.db.execute(
                    "REPLACE INTO meta(key, value) VALUES('map_session', ?)",
                    (str(session),))
                store.db.commit()
            with store._lock, store.db:
                for row in wanted:
                    if drop_ranges:
                        row = dict(row)
                        for column in RANGE_COLUMNS:
                            row[column] = None
                    store.db.execute(
                        "INSERT INTO observations(entity_id, "
                        + ",".join(COLUMNS) + ") VALUES(NULL, "
                        + ",".join("?" * len(COLUMNS)) + ")",
                        # `get`, because a recording is a database written by
                        # an older build and may predate a column this one
                        # replays. Missing is null, which is what the store's own
                        # migration leaves on the rows it adds a column to.
                        tuple(row.get(name) for name in COLUMNS))
            outcome = resolver.resolve(store, reach=reach)
            if verbose:
                print(f"  look {group[0]['inference_id']}: "
                      f"+{len(wanted)} regions -> {outcome['matched']} matched, "
                      f"{outcome['created']} placed, "
                      f"{outcome['ambiguous']} ambiguous, "
                      f"{outcome['still_waiting']} still waiting")
        entities = [dict(row) for row in store.db.execute("SELECT * FROM entities")]
        observations = [dict(row) for row in store.db.execute(
            "SELECT id, entity_id, inference_id, bearing_deg, span_deg,"
            " elevation_deg, observer_pose_json, map_session, dino_blob"
            " FROM observations")]
        store.close()
        return entities, observations
    finally:
        shutil.rmtree(directory, ignore_errors=True)


# --- marking it --------------------------------------------------------------

def _unit(blob):
    import numpy as np
    vector = np.frombuffer(blob, dtype="<f4").astype("float64")
    length = float(np.linalg.norm(vector))
    return None if length < 1e-9 else vector / length


def _things(vectors, join: float = JOIN) -> int:
    """How many separate things these crops are, by single-link clustering.

    Single link rather than complete, deliberately: it joins one object seen from
    two sides through the views in between, so what it counts as a second thing
    really is one. It therefore *under*-reports mixing, which is the safe way for
    a number that is about to be used as evidence.
    """
    import numpy as np

    count = len(vectors)
    parent = list(range(count))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    near = np.array(vectors) @ np.array(vectors).T
    for i in range(count):
        for j in range(i + 1, count):
            if near[i, j] >= join:
                a, b = root(i), root(j)
                if a != b:
                    parent[a] = b
    return len({root(i) for i in range(count)})


def _wrap(degrees: float) -> float:
    return (degrees + 180.0) % 360.0 - 180.0


def score(entities, observations, sigma_deg: float = 1.5,
          detail: bool = False) -> dict:
    """What the replay came to, in the two numbers that are worth comparing."""
    from .locate import BEARING_SIGMA_DEG
    sigma_deg = BEARING_SIGMA_DEG if sigma_deg is None else sigma_deg

    held: dict = {}
    for one in observations:
        if one["entity_id"]:
            held.setdefault(one["entity_id"], []).append(one)
    mixed = stray = attached = 0
    rows = []
    for entity in entities:
        group = sorted(held.get(entity["id"], []), key=lambda one: one["id"])
        if not group:
            continue
        vectors = [v for v in (_unit(one["dino_blob"]) for one in group)
                   if v is not None]
        things = _things(vectors) if len(vectors) > 1 else len(vectors)
        odd = 0 if things <= 1 else len(vectors) - _biggest(vectors)
        try:
            placement = json.loads(entity["placement_json"] or "null")
        except (TypeError, ValueError):
            placement = None
        missed = 0
        for one in group:
            if not placement or one["bearing_deg"] is None:
                continue
            # A bearing is a direction *from a place*, and the place is a
            # position in the map of the day. A look taken before the map
            # changed cannot be asked whether it points at a position measured
            # after it -- the two are in different frames, so the miss it
            # reports is arithmetic on unrelated coordinates rather than
            # evidence about this entity. The resolver holds itself to the same
            # rule; see `resolve._replace_placement`.
            if one.get("map_session") != entity.get("placement_map_session"):
                continue
            try:
                pose = json.loads(one["observer_pose_json"])
            except (TypeError, ValueError):
                continue
            dx = placement["x_m"] - pose["x_m"]
            dy = placement["y_m"] - pose["y_m"]
            range_m = math.hypot(dx, dy)
            off = abs(_wrap(math.degrees(math.atan2(dy, dx)) - one["bearing_deg"]))
            allowed = (range_m * math.tan(math.radians(sigma_deg))
                       + placement.get("uncertainty_m", 0.0))
            if off >= 90.0 or range_m * math.tan(math.radians(min(off, 89.0))) > allowed:
                missed += 1
        mixed += odd
        stray += missed
        attached += len(group)
        rows.append((entity["id"], len(group), things, odd, missed))

    orphans = sum(1 for one in observations if not one["entity_id"])
    print(f"  {len(entities)} entities from {len(observations)} observations, "
          f"{attached} attached and {orphans} still waiting")
    print(f"  crops of something other than the entity's main thing: "
          f"{mixed} ({_share(mixed, attached)})")
    print(f"  bearings that miss the entity's own position:          "
          f"{stray} ({_share(stray, attached)})")
    survived = _across_maps(entities, held)
    if survived is not None:
        drivable, kept, last = survived
        print(f"  things standing in map {last}, which is the only map the rover "
              f"could be sent to one in: {drivable} of {len(entities)}")
        print(f"  things that kept what they were across a map change:   {kept}")
    if detail:
        print("    entity        looks  things  odd  stray")
        for row in sorted(rows, key=lambda one: -one[3]):
            print("    %-13s %4d %6d %5d %6d" % row)
    return {"entities": len(entities), "attached": attached, "mixed": mixed,
            "stray": stray, "orphans": orphans, "rows": rows}


def _across_maps(entities, held) -> tuple[int, int, int] | None:
    """What survived the map changes in this recording, or None if it has none.

    Two numbers, and the second is the one worth having. **A position is only
    ever a position in the map it was measured in**, so when the map is replaced
    the rover can be sent to a thing only if that thing has since been placed
    again in the new one -- which is the first number. The second is how many of
    those are the *same thing the rover already knew*, still carrying the looks
    it was recognised by, rather than a stranger that happens to be standing
    where a chair the rover has photographed forty times is standing.
    """
    sessions = {one.get("map_session") for group in held.values()
                for one in group if one.get("map_session") is not None}
    if len(sessions) < 2:
        return None
    last = max(sessions)
    drivable = kept = 0
    for entity in entities:
        if entity.get("placement_map_session") != last:
            continue
        drivable += 1
        if any(one.get("map_session") != last
               for one in held.get(entity["id"], [])):
            kept += 1
    return drivable, kept, last


def _biggest(vectors, join: float = JOIN) -> int:
    import numpy as np

    count = len(vectors)
    parent = list(range(count))

    def root(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    near = np.array(vectors) @ np.array(vectors).T
    for i in range(count):
        for j in range(i + 1, count):
            if near[i, j] >= join:
                a, b = root(i), root(j)
                if a != b:
                    parent[a] = b
    sizes: dict = {}
    for i in range(count):
        sizes[root(i)] = sizes.get(root(i), 0) + 1
    return max(sizes.values())


def _share(part: int, whole: int) -> str:
    return "n/a" if not whole else f"{100.0 * part / whole:.0f}%"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("database", help="a world.db copied off the rover")
    parser.add_argument("--frames", default="",
                        help="the matching frames directory; enables the "
                             "blank-region filter on an old recording")
    parser.add_argument("--map", default="",
                        help="a map.json fetched from the nav bridge; supplies "
                             "the range bound the occupancy grid gives")
    parser.add_argument("--recompute-bearings", action="store_true",
                        help="work every bearing out again through today's "
                             "view.ray instead of replaying the ones the rover "
                             "stored; the only way to judge a change to the "
                             "bearing model on a recording")
    parser.add_argument("--frame-size", default="640x480",
                        help="the capture mode the recording was taken in, "
                             "which chooses the lens")
    parser.add_argument("--no-untrusted-pose", action="store_true",
                        help="drop the observations taken at exactly the map "
                             "origin, which is what an unlocalised stack gave")
    parser.add_argument("--session", type=int, default=0,
                        help="replay only the looks taken under this map "
                             "session; the default follows the map changes the "
                             "recording itself went through")
    parser.add_argument("--no-ranges", action="store_true",
                        help="replay as though the depth camera had been off, "
                             "dropping every range the rover measured; the "
                             "bearing-only half of the M0 comparison")
    parser.add_argument("--detail", action="store_true",
                        help="one line per entity")
    parser.add_argument("--verbose", action="store_true",
                        help="one line per look, as it is replayed")
    args = parser.parse_args()

    if not os.path.exists(args.database):
        print(f"no such recording: {args.database}", file=sys.stderr)
        return 2
    skip = blank_ids(args.database, args.frames) if args.frames else set()
    if skip:
        print(f"  {len(skip)} regions with no picture in them, left out")
    reach = reach_from(args.map) if args.map else None
    groups = inspections(args.database)
    if args.session:
        groups = [group for group in groups
                  if group[0]["map_session"] == args.session]
        print(f"  map session {args.session} alone: {len(groups)} looks")
    if args.recompute_bearings:
        width, height = (int(part) for part in args.frame_size.lower().split("x"))
        count, median = remeasure(groups, (width, height))
        print(f"  {count} bearings worked out again, median move "
              f"{median:.2f} deg")
    carried = sum(1 for group in groups for row in group
                  if row.get("range_m") is not None)
    if args.no_ranges:
        print(f"  bearing-only: the {carried} ranges in this recording are "
              f"being dropped")
    else:
        print(f"  range-assisted: {carried} of "
              f"{sum(len(group) for group in groups)} regions carry a measured "
              f"distance")
    entities, observations = replay(args.database, skip=skip,
                                    drop_untrusted=args.no_untrusted_pose,
                                    verbose=args.verbose, reach=reach,
                                    groups=groups, drop_ranges=args.no_ranges)
    score(entities, observations, detail=args.detail)
    return 0


if __name__ == "__main__":
    sys.exit(main())
