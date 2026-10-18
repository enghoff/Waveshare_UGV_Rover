"""Version 4 of the world model kept up to date on the rover, beside the store.

    python3 job.py [--state DIR] [--every S] [--once]

Run by run_job.sh on the system Python, where version 4 was verified; the
lidar comes from scan_log.py, which listens inside navigation's environment
and writes it to `lidar/`. Every `--every` seconds it reads the regions the
store has added (read-only), lets the engine file every visit that has
closed, and writes:

- `things.json`: the model's records -- place, state, when seen, and the
  store's observation ids of every region filed to each -- for the daemon to
  read in place of the store's entities (`find_thing` first);
- `engine.pkl`: the engine itself, so a restart carries on where it stopped
  (and one that cannot be read is kept aside and the history filed afresh,
  under a new `generation`, which names the records);
- `decisions.jsonl`: what each visit decided, appended;
- `scans/`: the lidar and navigation's pose over each batch of visits it
  filed, in the drive recordings' format (bag_extract.py), so ordinary drives
  leave recordings behind as staged sessions do.

With no `engine.pkl` it first files the whole history, with the drive
recordings in `recordings/` (the desk's, copied over, read in name order as
the desk's run_versions.sh lists them), then carries on with the live lidar.

The store keeps filing as it always has; nothing here writes to it. A region
is read only once it is READ_DELAY_S old, so that what the store settles
about it just after the look is in.

The records are of one filling of the store on one map (`filling`). When the
store is cleared, or put on another map, the engine and everything of its map
are kept aside in `set-aside/` and the job files what the store holds then,
starting from nothing (`follow`). On a map other than the one version 4's
walls were drawn from, raised things are placed on the map's own grid
(`walls_for`).
"""
import argparse
import json
import math
import os
import sqlite3
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import engine  # noqa: E402
import data  # noqa: E402
import recorded  # noqa: E402
import walls  # noqa: E402

WORLD = os.path.expanduser(os.environ.get("UGV_WORLD_DIR", "~/.ugv/world"))
DB = os.path.join(WORLD, "world.db")
#: Where world_state/visit_things.py reads the records from.
STATE = os.path.join(WORLD, "visit_model")
READ_DELAY_S = 60.0
#: How much lidar the job holds: a visit filed later than this after it closed
#: is filed without the lidar's absence evidence, as the desk does for looks no
#: recording covers.
BUFFER_S = 1800.0
#: Earlier than this, a region was stamped by a clock not yet set: the Orin
#: reads 1970 for about 75 s after a cold boot, and the looking loop records
#: then. The history the job first files keeps them, as the desk's runs did.
#: Lidar stamped then is left out everywhere (`clean`).
CLOCK_SET_AFTER = 1.7e9
#: How the history is filed; a saved engine filed some other way is kept aside
#: and the history filed again. 1: each visit counts the looks for each record
#: (model.looked_for). 2: lidar stamped before the clock was set left out, which
#: had made one scan file "cover" 1970 to 2026 and taken from records the lidar
#: hits that let them be seen gone (2026-10-18: 74 gone, not about 110).
FILED_AS = 2
#: Lidar kept on disk around each batch of visits filed.
SCAN_MARGIN_S = 10.0
SCANS_KEPT_S = 14 * 86400.0
#: The map version 4's walls (engine.WALLS) were drawn from: the M5 sessions'
#: grids, all of map session 67. On any other map the walls are that map's
#: own grid, fetched again every WALLS_EVERY_S while the rover maps it.
WALLS_MAP_ID = "7da19bef3888"
WALLS_EVERY_S = 600.0
#: The rover's daemon, which answers for navigation's map.
DAEMON = ("127.0.0.1", 8769)
#: What goes aside with an engine when the store changes under it: its
#: records, what it decided, and the lidar, recordings and walls of its map.
KEPT_WITH_ENGINE = ("engine.pkl", "things.json", "decisions.jsonl", "scans", "recordings",
                    "walls.npz")


def clean(arrays):
    """A recording's arrays without what was stamped before the clock was set."""
    s = arrays["scan_meta"][:, 0] >= CLOCK_SET_AFTER
    o = arrays["odom_tf"][:, 0] >= CLOCK_SET_AFTER
    m = arrays["map_tf"][:, 0] >= CLOCK_SET_AFTER
    return {"ranges": arrays["ranges"][s], "scan_meta": arrays["scan_meta"][s],
            "odom_tf": arrays["odom_tf"][o], "map_tf": arrays["map_tf"][m]}


def lidar(directory):
    """The last BUFFER_S of lidar and pose that scan_log.py has written, as one
    recording's arrays, or None while there is none."""
    try:
        names = sorted(n for n in os.listdir(directory) if n.endswith(".npz") and not n.startswith("."))
    except OSError:
        return None
    now = time.time()
    parts = []
    for name in names:
        try:
            last = float(name[:-4].split("-")[1])
        except (IndexError, ValueError):
            continue
        if now - last > BUFFER_S:
            continue
        try:
            with np.load(os.path.join(directory, name)) as d:
                parts.append(clean({k: d[k] for k in ("ranges", "scan_meta", "odom_tf", "map_tf")}))
        except Exception:  # noqa: BLE001 - one bad file must not stop every batch
            # A file a power cut left empty raises EOFError, which no narrower
            # list here caught: on 2026-10-17 one stopped every batch for the
            # half hour it stayed inside BUFFER_S.
            continue
    if not parts:
        return None
    width = max(p["ranges"].shape[1] for p in parts)
    ranges = np.concatenate([np.pad(p["ranges"], ((0, 0), (0, width - p["ranges"].shape[1])),
                                    constant_values=np.nan) for p in parts])
    out = {"ranges": ranges, "scan_meta": np.concatenate([p["scan_meta"] for p in parts]),
           "odom_tf": np.concatenate([p["odom_tf"] for p in parts]),
           "map_tf": np.concatenate([p["map_tf"] for p in parts])}
    if len(out["scan_meta"]) < 10 or len(out["odom_tf"]) < 10 or len(out["map_tf"]) < 1:
        return None
    return out


def _slice(arrays, t0, t1):
    """The part of a recording's arrays between t0 and t1 (by stamp), with the
    last map correction before t0 kept so that its poses still compose."""
    s = (arrays["scan_meta"][:, 0] >= t0) & (arrays["scan_meta"][:, 0] <= t1)
    o = (arrays["odom_tf"][:, 0] >= t0) & (arrays["odom_tf"][:, 0] <= t1)
    m = arrays["map_tf"][:, 0] <= t1
    before = np.flatnonzero(arrays["map_tf"][:, 0] < t0)
    if len(before):
        m &= arrays["map_tf"][:, 0] >= arrays["map_tf"][before[-1], 0]
    return {"ranges": arrays["ranges"][s], "scan_meta": arrays["scan_meta"][s],
            "odom_tf": arrays["odom_tf"][o], "map_tf": arrays["map_tf"][m]}


def filling(db_path=None):
    """Which filling of the store, on which map: what records filed from it
    now are of. The store's generation changes when it is cleared, its map
    identity when navigation's map is replaced, and its session when that
    happens with rows kept (world_state/store.py, `follow_map`)."""
    con = sqlite3.connect("file:%s?mode=ro" % (db_path or DB), uri=True)
    try:
        meta = dict(con.execute("select key, value from meta where key in"
                                " ('generation', 'map_id', 'map_session')").fetchall())
    finally:
        con.close()
    session = str(meta.get("map_session") or "")
    return {"store_generation": meta.get("generation") or "", "map_id": meta.get("map_id") or "",
            "map_session": int(session) if session.isdigit() else 1}


def daemon(call, arguments=None, timeout=10.0):
    """One call to the rover's daemon, or None when it does not answer."""
    import socket
    try:
        with socket.create_connection(DAEMON, timeout) as s:
            f = s.makefile("rwb")
            f.write(json.dumps({"call": call, "arguments": arguments or {}}).encode() + b"\n")
            f.flush()
            return json.loads(f.readline())
    except (OSError, ValueError):
        return None


def grid_walls(map_id):
    """The walls of navigation's grid when its map is `map_id`, else None."""
    before = (daemon("nav_status") or {}).get("map_id")
    grid = daemon("nav_grid") or {}
    after = (daemon("nav_status") or {}).get("map_id")
    if not grid.get("ok") or not map_id or not before == after == map_id:
        return None
    return walls.from_grid(grid)


def walls_for(e, state, log):
    """The walls raised things are placed on: version 4's on the map they were
    drawn from, else the map's own grid, kept as `walls.npz` and fetched again
    when it is WALLS_EVERY_S old; none until navigation has sent one."""
    if e.filling["map_id"] == WALLS_MAP_ID:
        return walls.load(engine.WALLS)
    path = os.path.join(state, "walls.npz")
    try:
        age = time.time() - os.path.getmtime(path)
    except OSError:
        age = None
    if age is None or age > WALLS_EVERY_S:
        w = grid_walls(e.filling["map_id"])
        if w is not None:
            tmp = path + ".tmp.npz"
            np.savez_compressed(tmp, **w)
            os.replace(tmp, path)
            age = 0.0
        elif age is None:
            log("no grid of map %s from navigation: nothing raised is placed on a wall" % e.filling["map_id"])
    if age is None:
        return walls.none()
    return walls.load(path)


def things(e):
    """The records as the daemon reads them."""
    members = {}
    for rid, oid in e.resolved().items():
        members.setdefault(oid, []).append(int(rid))
    out = []
    for o in e.mem.objects:
        ids = sorted(members.get(o["id"], []))
        if not ids:
            continue
        out.append({"id": o["id"], "x_m": round(float(o["x"]), 3), "y_m": round(float(o["y"]), 3),
                    "spread_m": round(math.sqrt(float(o["var"])), 3),
                    "extent_m": round(float(o["extent"]), 3),
                    "height_m": None if not math.isfinite(float(o["z"])) else round(float(o["z"]), 3),
                    "state": o["state"], "seen_at": o["seen"], "since": o["since"],
                    "gone_at": o["gone_at"], "visits": len(o["visits"]),
                    "looked_for": o.get("looked_for", 0),
                    "path": o["path"], "observations": ids})
    return out


def write_json(path, payload):
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)


def once(e, state, use_lidar, log):
    now = time.time()
    arrays = lidar(os.path.join(state, "lidar")) if use_lidar else None
    recs = [recorded.Recording(arrays=arrays)] if arrays is not None else []
    a = engine.prepare(DB, recs, after_id=e.last_id, session=e.filling["map_session"],
                       wall_cells=walls_for(e, state, log))
    if filling(DB) != e.filling:
        # Cleared or moved onto another map while the batch was read, which may
        # then hold the new map's looks: `follow` starts afresh next pass.
        log("the store changed while a batch was read: nothing filed")
        return 0
    if len(a["id"]):
        # Only regions READ_DELAY_S old, and only an unbroken run of ids, so
        # that a region the store adds late is not skipped.
        young = a["t"] > now - READ_DELAY_S
        if young.any():
            first_young = int(a["id"][young].min())
            keep = a["id"] < first_young
            a = {k: v[keep] for k, v in a.items()}
        unset = a["t"] < CLOCK_SET_AFTER
        if unset.any():
            # Filed in time order, a look stamped in 1970 would be a visit 56
            # years before the rest, and would set filed_until to 1970: on
            # 2026-10-18 eleven did, after a cold boot. Passed over, not filed.
            log("%d regions stamped before the clock was set: passed over" % int(unset.sum()))
            e.last_id = max(e.last_id, int(a["id"][unset].max()))
            a = {k: v[~unset] for k, v in a.items()}
        e.add(a)
    decisions = []
    filed = e.step(now - READ_DELAY_S, recs, decisions)
    if decisions:
        with open(os.path.join(state, "decisions.jsonl"), "a") as f:
            for d in decisions:
                f.write(json.dumps(d) + "\n")
    if filed:
        if arrays is not None and decisions:
            ts = [d["t"] for d in decisions]
            part = _slice(arrays, min(ts) - SCAN_MARGIN_S - 150.0, e.taken_until + SCAN_MARGIN_S)
            if len(part["scan_meta"]):
                os.makedirs(os.path.join(state, "scans"), exist_ok=True)
                np.savez_compressed(os.path.join(state, "scans", "%d.npz" % int(part["scan_meta"][0, 0])),
                                    **part)
        e.save(os.path.join(state, "engine.pkl"))
    records = things(e)
    write_json(os.path.join(state, "things.json"), {
        "model": "visit_model version 4 (cabe003)", "written_at": time.time(),
        "generation": e.generation, "map_session": e.filling["map_session"],
        "map_id": e.filling["map_id"], "store_generation": e.filling["store_generation"],
        "filed_until": e.taken_until, "last_observation": e.last_id,
        "waiting": 0 if e.pending is None else int(len(e.pending["id"])),
        "lidar": arrays is not None, "things": records})
    if filed:
        log("filed %d regions; %d records, %d waiting; lidar %s" % (
            filed, len(records), 0 if e.pending is None else len(e.pending["id"]),
            "yes" if arrays is not None else "no"))
    return filed


def bootstrap(state, log):
    """An engine that has filed the whole history, with the drive recordings
    kept in `recordings/` and then the lidar this job kept in `scans/`."""
    import glob
    recs = [recorded.Recording(p) for p in sorted(glob.glob(os.path.join(state, "recordings", "*.npz")))]
    for p in sorted(glob.glob(os.path.join(state, "scans", "*.npz")),
                    key=lambda p: float(os.path.basename(p)[:-4])):
        with np.load(p) as d:
            arrays = clean({k: d[k] for k in ("ranges", "scan_meta", "odom_tf", "map_tf")})
        if len(arrays["scan_meta"]) >= 10 and len(arrays["odom_tf"]) >= 10 and len(arrays["map_tf"]):
            recs.append(recorded.Recording(arrays=arrays))
    log("filing the history with %d recordings" % len(recs))
    t0 = time.time()
    e = engine.Engine()
    e.generation = mint()
    e.filed_as = FILED_AS
    e.filling = filling(DB)
    a = engine.prepare(DB, recs, after_id=0, session=e.filling["map_session"],
                       wall_cells=walls_for(e, state, log))
    young = a["t"] > time.time() - READ_DELAY_S
    if young.any():
        keep = a["id"] < int(a["id"][young].min())
        a = {k: v[keep] for k, v in a.items()}
    e.add(a)
    decisions = []
    n = e.step(time.time() - READ_DELAY_S, recs, decisions)
    with open(os.path.join(state, "decisions.jsonl"), "a") as f:
        for d in decisions:
            f.write(json.dumps(d) + "\n")
    log("history filed: %d regions in %.0f s, %d records" % (n, time.time() - t0, len(e.mem.objects)))
    return e


def mint():
    """A name for one engine's records, new whenever the history is filed
    afresh: a record's identifier means that record only under it. Sixteen hex
    characters, as the store's generation and autonomy's references are
    (autonomy/refs.py)."""
    return os.urandom(8).hex()


def load(path, state, log):
    """The saved engine, or one filed afresh when there is none or it cannot
    be read; a bad one is kept aside, not overwritten."""
    if os.path.exists(path):
        try:
            e = engine.Engine.load(path)
        except Exception as exc:  # noqa: BLE001 - any unreadable save
            os.replace(path, path + ".bad")
            log("the saved engine could not be read (%r): kept as %s.bad" % (exc, os.path.basename(path)))
        else:
            filed_as = getattr(e, "filed_as", 0)
            if filed_as < FILED_AS:
                # Filed some earlier way (FILED_AS): kept aside, and the
                # history filed again.
                os.replace(path, "%s.filed-as-%d" % (path, filed_as))
                log("the saved engine was filed as %d, not %d: kept as %s.filed-as-%d,"
                    " filing the history again" % (filed_as, FILED_AS, os.path.basename(path), filed_as))
            else:
                if not getattr(e, "generation", None):
                    # Saved before engines were named.
                    e.generation = mint()
                    e.save(path)
                e.filling = getattr(e, "filling", None)
                if e.filling is None and filling(DB)["map_session"] == data.MAP_SESSION:
                    # Saved before the job wrote down what it files from: it
                    # filed map session 67 alone, of which the store has held
                    # one filling on one map since 2026-10-03. Of any other,
                    # `follow` keeps it aside.
                    e.filling = filling(DB)
                    e.save(path)
                return e
    e = bootstrap(state, log)
    e.save(path)
    return e


def follow(e, state, log):
    """The engine for the store as it is now: `e` while the store holds the
    filling, on the map, that `e` files from; else a new one filing what the
    store holds now, with `e` and everything of its map kept aside."""
    now = filling(DB)
    if e.filling == now:
        return e
    was = e.filling or {}
    moved = (now["map_id"], now["map_session"]) != (was.get("map_id"), was.get("map_session"))
    aside = os.path.join(state, "set-aside", "%s-%s-%s" % (
        time.strftime("%Y%m%d-%H%M%S"), "map" if moved else "cleared", e.generation))
    os.makedirs(aside, exist_ok=True)
    for name in KEPT_WITH_ENGINE:
        if os.path.exists(os.path.join(state, name)):
            os.replace(os.path.join(state, name), os.path.join(aside, name))
    log("%s: the records named %s are kept in %s, and what the store holds now is filed afresh" % (
        "the store is on map %s, session %d, not %s, session %s" % (
            now["map_id"], now["map_session"], was.get("map_id"), was.get("map_session"))
        if moved else "the world state was cleared", e.generation, aside))
    e = bootstrap(state, log)
    e.save(os.path.join(state, "engine.pkl"))
    return e


def prune(state):
    d = os.path.join(state, "scans")
    if not os.path.isdir(d):
        return
    for name in os.listdir(d):
        p = os.path.join(d, name)
        if time.time() - os.path.getmtime(p) > SCANS_KEPT_S:
            os.remove(p)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=STATE)
    ap.add_argument("--every", type=float, default=60.0)
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--no-lidar", action="store_true")
    args = ap.parse_args()
    os.makedirs(args.state, exist_ok=True)

    def log(msg):
        print("%s %s" % (time.strftime("%Y-%m-%d %H:%M:%S"), msg), flush=True)

    e = load(os.path.join(args.state, "engine.pkl"), args.state, log)
    log("started: %d regions filed so far, last observation %d; records named %s" % (
        len(e.filing), e.last_id, e.generation))
    while True:
        try:
            e = follow(e, args.state, log)
            once(e, args.state, not args.no_lidar, log)
            prune(args.state)
        except Exception as exc:  # noqa: BLE001 - a bad batch must not stop the job
            log("batch failed: %r" % (exc,))
            if args.once:
                raise
        if args.once:
            return
        time.sleep(args.every)


if __name__ == "__main__":
    main()
