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
about it just after the look is in. Regions of another map session than the
model's are not read (data.MAP_SESSION).
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
CLOCK_SET_AFTER = 1.7e9
#: Lidar kept on disk around each batch of visits filed.
SCAN_MARGIN_S = 10.0
SCANS_KEPT_S = 14 * 86400.0


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
                parts.append({k: d[k] for k in ("ranges", "scan_meta", "odom_tf", "map_tf")})
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


def map_session(db_path):
    con = sqlite3.connect("file:%s?mode=ro" % db_path, uri=True)
    try:
        row = con.execute("select value from meta where key = 'map_session'").fetchone()
    finally:
        con.close()
    return int(row[0]) if row and str(row[0]).isdigit() else None


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
    session = map_session(DB)
    if session != data.MAP_SESSION:
        log("the store is on map session %s, the model on %s: nothing read" % (session, data.MAP_SESSION))
        return 0
    arrays = lidar(os.path.join(state, "lidar")) if use_lidar else None
    recs = [recorded.Recording(arrays=arrays)] if arrays is not None else []
    a = engine.prepare(DB, recs, after_id=e.last_id)
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
        "generation": e.generation, "map_session": data.MAP_SESSION,
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
    paths = sorted(glob.glob(os.path.join(state, "recordings", "*.npz")))
    paths += sorted(glob.glob(os.path.join(state, "scans", "*.npz")),
                    key=lambda p: float(os.path.basename(p)[:-4]))
    recs = [recorded.Recording(p) for p in paths]
    log("filing the history with %d recordings" % len(recs))
    t0 = time.time()
    e = engine.Engine()
    e.generation = mint()
    a = engine.prepare(DB, recs, after_id=0)
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
            if not getattr(e, "generation", None):
                # Saved before engines were named.
                e.generation = mint()
                e.save(path)
            return e
    e = bootstrap(state, log)
    e.save(path)
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
