"""Score M4's attempts against the owner's tape: each chosen look and its re-look.

M4 (docs/plans/autonomous-curiosity.md, agreed 2026-10-09) asks whether a look from
the viewpoint the rover chose improves where it knows a thing to be more than a
re-look from where it stood when it chose. A trial run copies the world store
before each attempt (`world_snapshot`), takes the re-look, drives, and takes the
chosen look. Here each look is filed into its own copy of that snapshot with the
production code (`world_state.aimed.file_by_aim`), so both start from the same
knowledge whatever else the run did, and the target's placement is scored against
the tape before and after.

    python experiments/m4/score_attempts.py attempts --episodes episodes.db \\
        --store world.db --snapshots DIR --truth truth.json --out report.json
    python experiments/m4/score_attempts.py plan 0.17 0.02

`attempts` reads every episode of the episode store that took a snapshot. The
snapshot paths the episodes recorded are the rover's; `--snapshots` names the
directory they were copied to, which is where `<episode>.db` and its
`.map.json` are looked for. `--store` is a world store taken after the runs,
which holds the looks themselves. `--truth` is
`{"things": {name: {"x_m", "y_m", "records": [ids], "where": room}}}` in map
coordinates; an attempt whose target is in no thing's records is reported as not
scorable, never dropped. `plan` prints how many pairs the comparison needs, for a
chosen look and a re-look that each improve their thing at the given rates.

**The score.** The log-likelihood of the tape's position under the placement's
stated position and uncertainty, taking the stated figure as the one-sigma
radius -- the square root of the covariance's trace, as
`experiments/entity_association/placement_calibration.py` and the resolver mean
it -- and floored at the score of a 2 m claim 2 m off, so that one wild claim
cannot decide the mean. Coming closer scores higher, and so does a claim that
narrows and still holds the tape; one that narrows past the tape scores lower. A
look that files nothing changes nothing and gains exactly zero. Fixed here before
any acceptance attempt, as M4's criterion 4 asks.

**Whether the rover's own account agrees with the tape** (criterion 8, added
2026-10-09 by docs/decisions/every-gate-says-what-it-is-for.md, worded in the plan
at c0bf0d8). The figure is the chosen look's: the fall in its target's claimed
uncertainty, worked out as the executive measures it (`situation.claimed_m`), on
the same copy of the store as its tape score. That is the signal an ordinary run,
with no re-look, records for itself, and the one M10 would learn from. Its sign is
compared with the tape score's only over looks whose claim changed by
`OWN_NO_CHANGE_M` or more: a look that filed nothing agrees with a tape score of
zero, and counting those would flatter the rate, so they are reported apart. Over
the looks that changed something, better than chance is better than one half.
Re-looks are reported the same way, separately. The executive's own measurement of
the whole attempt, which in a trial holds the re-look's change and the chosen
look's together, is compared with the tape score of both looks filed together, as
a second figure.

**Both placements are worked out again here**, before the look and after it, with
the resolver's own `_replace_placement` and the map the snapshot kept, so that the
gain is the look's and not a difference between how the rover once worked a
placement out and how it would now.
"""
from __future__ import annotations

import argparse
from contextlib import closing
import json
import math
import os
import random
import sqlite3
import statistics
import sys
import tempfile
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "autonomy"))
from world_state import aimed, locate, replay, resolve  # noqa: E402
from world_state.store import WorldStore  # noqa: E402

#: The stated figure is the one-sigma radius, sqrt of the covariance's trace, so
#: each axis of a round claim has this share of it.
AXIS_OF_STATED = 1.0 / math.sqrt(2.0)
#: The least a claim may state before its score stops meaning anything: a
#: centimetre.
LEAST_STATED_M = 0.01
#: What the floor is the score of: a claim of this size, this far off.
FLOOR_STATED_M = 2.0
FLOOR_OFF_M = 2.0
#: A gain smaller than this either way is no change: a look that filed nothing
#: gains exactly zero, and anything else moves the score by far more.
NO_CHANGE = 1e-6
#: A fall in the rover's own stated uncertainty smaller than this, either way,
#: is no change: placements are written to the millimetre, and a few of them
#: wobble by that much when nothing new arrived.
OWN_NO_CHANGE_M = 0.005
#: The conventional 95% two-sided interval and 80% power, for `plan`.
Z_95, Z_80 = 1.96, 0.84
BOOTSTRAP = 2000


def _raw_score(off_m: float, stated_m: float) -> float:
    sigma = max(stated_m, LEAST_STATED_M) * AXIS_OF_STATED
    return -math.log(2.0 * math.pi * sigma * sigma) - off_m * off_m / (2.0 * sigma * sigma)


FLOOR = _raw_score(FLOOR_OFF_M, FLOOR_STATED_M)


def score(placement: dict[str, Any] | None, truth: tuple[float, float]) -> dict[str, Any]:
    """How well a placement says where the thing is. An unplaced thing scores the
    floor: it says nothing, which is the worst a claim is allowed to count."""
    if not placement or placement.get("x_m") is None:
        return {"placed": False, "score": FLOOR}
    stated = locate.stated_uncertainty(placement)
    if stated is None:
        return {"placed": False, "score": FLOOR}
    off = math.hypot(float(placement["x_m"]) - truth[0], float(placement["y_m"]) - truth[1])
    return {"placed": True, "off_m": round(off, 3), "stated_m": round(stated, 3),
            "inside": off <= stated, "overconfident": off > 2.0 * stated,
            "score": round(max(FLOOR, _raw_score(off, stated)), 4)}


# --- filing a look into a copy of a store ---------------------------------------

def open_copy(path: str, directory: str) -> WorldStore:
    """A working copy of the store at `path`, in `directory`. Read only at the
    source, through sqlite's own backup, which takes the write-ahead log with it."""
    store = WorldStore(directory)
    with closing(sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True)) as source:
        source.backup(store.db)
    store._create()
    return store


def bring_in(store: WorldStore, source_path: str, frames: list[str],
             as_of: float | None = None) -> int:
    """Put the regions of `frames` into the copy, unfiled, as they stood just
    after the looks were taken: copied from `source_path` where the copy does not
    have them, taken off whatever they were filed to where it does. With `as_of`,
    everything recorded at or after it is taken out first, so the copy stands as
    the store did then (for scoring recorded looks that had no snapshot)."""
    if as_of is not None:
        with store._lock:
            store.db.execute(
                "DELETE FROM aimed_looks WHERE filed_at >= ?", (as_of,))
            store.db.execute(
                "DELETE FROM observations WHERE observed_at >= ? AND (frame_id IS NULL"
                f" OR frame_id NOT IN ({','.join('?' * len(frames))}))",
                (as_of, *frames))
            store.db.commit()
    columns = [row[1] for row in store.db.execute("PRAGMA table_info(observations)")]
    with closing(sqlite3.connect(Path(source_path).resolve().as_uri() + "?mode=ro", uri=True)) as source:
        have = {row[1] for row in source.execute("PRAGMA table_info(observations)")}
        shared = [one for one in columns if one in have]
        rows = source.execute(
            f"SELECT {', '.join(shared)} FROM observations WHERE frame_id IN"
            f" ({','.join('?' * len(frames))})", frames).fetchall()
        frame_rows = source.execute(
            f"SELECT id, path, taken_at, bytes, width, height FROM frames WHERE id IN"
            f" ({','.join('?' * len(frames))})", frames).fetchall()
    with store._lock:
        for row in rows:
            body = dict(zip(shared, row))
            body["entity_id"] = None
            store.db.execute("DELETE FROM aimed_looks WHERE observation_id = ?", (body["id"],))
            store.db.execute(
                f"INSERT OR REPLACE INTO observations({', '.join(body)})"
                f" VALUES({', '.join('?' * len(body))})", list(body.values()))
        for row in frame_rows:
            store.db.execute("INSERT OR IGNORE INTO frames(id, path, taken_at, bytes, width,"
                             " height) VALUES(?,?,?,?,?,?)", row)
        store.db.commit()
    return len(rows)


def placement_of(store: WorldStore, target: str) -> dict[str, Any] | None:
    session = store.map_session()
    found = next((one for one in store.placed(map_session=session)
                  if one["id"] == target), None)
    return (found or {}).get("placement")


def file_looks(snapshot: str, source: str, target: str, frames: list[str],
               reach=None, as_of: float | None = None) -> dict[str, Any]:
    """The target's placement before and after filing `frames` by aim at it, in a
    copy of `snapshot`, and what each filing said."""
    with tempfile.TemporaryDirectory(prefix="m4-score-") as directory:
        store = open_copy(snapshot, directory)
        try:
            brought = bring_in(store, source, frames, as_of) if frames else 0
            session = store.map_session()
            resolve._replace_placement(store, target, session, reach)
            before = placement_of(store, target)
            filings = []
            for frame in frames:
                filings.append(aimed.file_by_aim(store, target, frame, reach=reach))
            after = placement_of(store, target)
        finally:
            store.close()
    return {"before": before, "after": after, "filings": filings, "regions": brought}


def sign(value: float | None, no_change: float) -> int | None:
    if value is None:
        return None
    return 0 if abs(value) < no_change else (1 if value > 0 else -1)


def agrees(own_m: float | None, tape_gain: float) -> bool | None:
    """Whether the rover's own account of a gain has the tape's sign. None when
    it has no account, or when its claim did not change: those are reported
    apart rather than counted as agreeing with a tape score of zero."""
    own = sign(own_m, OWN_NO_CHANGE_M)
    return None if not own else own == sign(tape_gain, NO_CHANGE)


def claimed(placement: dict[str, Any] | None) -> float | None:
    """What the rover claims for a placement, read as the executive reads it."""
    import situation as situation_mod
    return None if not placement else situation_mod.claimed_m({"placement": placement})


def gain_of(filed: dict[str, Any], truth: tuple[float, float]) -> dict[str, Any]:
    before, after = score(filed["before"], truth), score(filed["after"], truth)
    gain = round(after["score"] - before["score"], 4)
    if abs(gain) <= NO_CHANGE:
        gain = 0.0
    was, now = claimed(filed["before"]), claimed(filed["after"])
    own = None if was is None or now is None else round(was - now, 4)
    return {"before": before, "after": after, "gain": gain,
            "own_change_m": own, "own_agrees": agrees(own, gain),
            "filed": sum(1 for one in filed["filings"] if one.get("filed")),
            "ranged": any(one.get("ranged") for one in filed["filings"]),
            "why": [one.get("why") for one in filed["filings"]]}


# --- the episodes -----------------------------------------------------------------

def attempts_in(episodes_path: str) -> list[dict[str, Any]]:
    """Every episode that took a snapshot, with what it was aimed at and which
    looks were the re-look's and which the chosen viewpoint's."""
    db = sqlite3.connect(Path(episodes_path).resolve().as_uri() + "?mode=ro", uri=True)
    out = []
    for episode_id, ref, opened in db.execute(
            "SELECT id, ref, opened_at FROM episodes ORDER BY id"):
        events = [(kind, json.loads(body or "{}")) for kind, body in db.execute(
            "SELECT kind, body_json FROM events WHERE episode_id = ? ORDER BY seq",
            (episode_id,))]
        snap = next((body for kind, body in events if kind == "call"
                     and body.get("call") == "world_snapshot"), None)
        if snap is None:
            continue
        chose = next((body.get("chose") for kind, body in events if kind == "decision"), None)
        candidate = next((body.get("params") or {} for kind, body in events
                          if kind == "candidate" and body.get("goal") == chose), {})
        relook, chosen, target = [], [], None
        for kind, body in events:
            if kind != "call" or body.get("call") != "world_inspect":
                continue
            result = body.get("result") or {}
            frame = result.get("frame_id")
            if result.get("role") == "relook":
                if body.get("ok") and frame:
                    relook.append(frame)
            else:
                target = (body.get("params") or {}).get("target") or target
                if body.get("ok") and frame:
                    chosen.append(frame)
        closed = next((body.get("outcome") for kind, body in events if kind == "closed"), None)
        own = next((body.get("placement_improved_m") for kind, body in events
                    if kind == "measured" and body.get("what") == "the attempt"), None)
        out.append({"episode": ref, "opened_at": opened, "target": target,
                    "snapshot_ok": bool(snap.get("ok")),
                    "snapshot": (snap.get("result") or {}).get("path"),
                    "map": (snap.get("result") or {}).get("map_path"),
                    "relook_frames": relook, "chosen_frames": chosen,
                    "constraints": candidate.get("constraints") or {},
                    "predicted_gain_m": candidate.get("gain_value"),
                    "own_gain_m": own, "outcome": closed})
    db.close()
    return out


def score_attempts(attempts: list[dict[str, Any]], truth: dict[str, Any],
                   store_path: str, snapshots: str | None) -> list[dict[str, Any]]:
    owner = {record: name for name, thing in truth["things"].items()
             for record in thing.get("records", [])}
    scored = []
    for attempt in attempts:
        row = {k: attempt.get(k) for k in ("episode", "target", "outcome",
                                           "predicted_gain_m", "own_gain_m")}
        row["in_depth_view"] = attempt["constraints"].get("in_depth_view")
        thing = owner.get(attempt["target"] or "")
        snapshot = attempt.get("snapshot")
        if snapshots and snapshot:
            snapshot = os.path.join(snapshots, os.path.basename(snapshot))
        why = ("no snapshot was taken" if not attempt["snapshot_ok"] else
               "the snapshot is not here" if not snapshot or not os.path.exists(snapshot) else
               "its target is none of the taped things" if thing is None else
               "the chosen viewpoint's look was not taken" if not attempt["chosen_frames"]
               else "")
        if why:
            scored.append({**row, "scorable": False, "why": why})
            continue
        map_path = os.path.splitext(snapshot)[0] + ".map.json"
        reach = replay.reach_from(map_path) if os.path.exists(map_path) else None
        spot = (float(truth["things"][thing]["x_m"]), float(truth["things"][thing]["y_m"]))
        chosen = gain_of(file_looks(snapshot, store_path, attempt["target"],
                                    attempt["chosen_frames"], reach), spot)
        again = gain_of(file_looks(snapshot, store_path, attempt["target"],
                                   attempt["relook_frames"], reach), spot)
        both = gain_of(file_looks(snapshot, store_path, attempt["target"],
                                  attempt["relook_frames"] + attempt["chosen_frames"],
                                  reach), spot)
        scored.append({**row, "scorable": True, "thing": thing,
                       "where": truth["things"][thing].get("where"),
                       "reach": reach is not None, "chosen": chosen, "relook": again,
                       "both": both,
                       "difference": round(chosen["gain"] - again["gain"], 4),
                       "recorded_agrees": agrees(attempt.get("own_gain_m"),
                                                 both["gain"])})
    return scored


# --- what the attempts add up to --------------------------------------------------

def interval(rows: list[dict[str, Any]], value, seed: int = 7) -> dict[str, Any]:
    """The mean of `value` over `rows` and its 95% interval, resampling things
    rather than attempts: looks at one thing are not independent of each other
    (the plan's measurement rules)."""
    values = [value(one) for one in rows]
    if not values:
        return {"n": 0}
    by_thing: dict[str, list[float]] = {}
    for one, v in zip(rows, values):
        by_thing.setdefault(one.get("thing") or one["episode"], []).append(v)
    groups = list(by_thing.values())
    rng = random.Random(seed)
    means = []
    for _ in range(BOOTSTRAP):
        drawn = [v for _ in groups for v in rng.choice(groups)]
        means.append(sum(drawn) / len(drawn))
    means.sort()
    return {"n": len(values), "things": len(groups),
            "mean": round(statistics.fmean(values), 4),
            "low": round(means[int(0.025 * BOOTSTRAP)], 4),
            "high": round(means[int(0.975 * BOOTSTRAP) - 1], 4)}


def shares(rows: list[dict[str, Any]], side: str) -> dict[str, int]:
    gains = [one[side]["gain"] for one in rows]
    return {"improved": sum(1 for g in gains if g > 0), "unchanged": sum(1 for g in gains if g == 0),
            "worse": sum(1 for g in gains if g < 0)}


def summary(scored: list[dict[str, Any]]) -> dict[str, Any]:
    ok = [one for one in scored if one["scorable"]]
    out: dict[str, Any] = {
        "attempts": len(scored), "scorable": len(ok),
        "not_scorable": {why: sum(1 for one in scored if one.get("why") == why)
                         for why in sorted({one.get("why") for one in scored
                                            if not one["scorable"]})},
        "criterion_3_difference": interval(ok, lambda one: one["difference"]),
        "criterion_5_chosen_gain": interval(ok, lambda one: one["chosen"]["gain"]),
        "relook_gain": interval(ok, lambda one: one["relook"]["gain"]),
        "chosen": shares(ok, "chosen"), "relook": shares(ok, "relook"),
    }
    for side in ("chosen", "relook"):
        after = [one[side]["after"] for one in ok if one[side]["after"]["placed"]]
        out[f"{side}_honesty"] = {
            "placed": len(after), "inside": sum(1 for a in after if a["inside"]),
            "overconfident": sum(1 for a in after if a["overconfident"])}
    by: dict[str, dict[str, Any]] = {}
    for name, key in (("in_depth_view", lambda one: str(one.get("in_depth_view"))),
                      ("ranged", lambda one: str(one["chosen"]["ranged"])),
                      ("where", lambda one: str(one.get("where")))):
        groups: dict[str, list] = {}
        for one in ok:
            groups.setdefault(key(one), []).append(one)
        by[name] = {value: shares(rows, "chosen") for value, rows in sorted(groups.items())}
    out["chosen_by_condition"] = by
    out["pairs_needed"] = pairs_needed_from(ok)
    out["criterion_8"] = {
        "chosen": own_agreement(ok, lambda one: one["chosen"]["own_change_m"],
                                lambda one: one["chosen"]["gain"]),
        "relook": own_agreement(ok, lambda one: one["relook"]["own_change_m"],
                                lambda one: one["relook"]["gain"]),
        "attempt_as_recorded": own_agreement(ok, lambda one: one.get("own_gain_m"),
                                             lambda one: one["both"]["gain"])}
    return out


def own_agreement(ok: list[dict[str, Any]], own_of, tape_of) -> dict[str, Any]:
    """How often the rover's own account had the tape's sign, over the looks
    whose claim changed, with its interval; and what became of the rest."""
    changed = [one for one in ok if sign(own_of(one), OWN_NO_CHANGE_M)]
    rate = interval(changed, lambda one: 1.0 if agrees(own_of(one), tape_of(one))
                    else 0.0)
    unchanged = [one for one in ok if sign(own_of(one), OWN_NO_CHANGE_M) == 0]
    return {**rate, "above_half": (rate.get("low", 0.0) > 0.5) if changed else None,
            "claim_unchanged": len(unchanged),
            "claim_unchanged_but_tape_moved": sum(
                1 for one in unchanged if sign(tape_of(one), NO_CHANGE)),
            "no_account": sum(1 for one in ok if own_of(one) is None)}


def pairs_needed_from(ok: list[dict[str, Any]]) -> int | None:
    """How many pairs the comparison needs, from the differences seen so far,
    inflated by how much more the things-resampled interval spreads than an
    attempts-resampled one would."""
    diffs = [one["difference"] for one in ok]
    if len(diffs) < 3 or not any(diffs):
        return None
    mean, sd = statistics.fmean(diffs), statistics.stdev(diffs)
    if mean <= 0 or sd == 0:
        return None
    clustered = interval(ok, lambda one: one["difference"])
    iid_half = Z_95 * sd / math.sqrt(len(diffs))
    effect = max(1.0, ((clustered["high"] - clustered["low"]) / 2.0 / iid_half) ** 2)
    return math.ceil(((Z_95 + Z_80) * sd / mean) ** 2 * effect)


def pairs_needed(chosen_rate: float, relook_rate: float) -> int | None:
    """Pairs for a 95% two-sided interval on the difference to exclude zero with
    80% power, counting each look as improved or not and the two as independent
    -- the sign test on the pairs that differ. A planning figure: the scored gains
    carry more than a yes or no, and looks at one thing are not independent."""
    differ_chosen = chosen_rate * (1.0 - relook_rate)
    differ_relook = relook_rate * (1.0 - chosen_rate)
    differ = differ_chosen + differ_relook
    if differ <= 0 or differ_chosen <= differ_relook:
        return None
    p = differ_chosen / differ
    discordant = ((Z_95 * 0.5 + Z_80 * math.sqrt(p * (1.0 - p))) / (p - 0.5)) ** 2
    return math.ceil(discordant / differ)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = parser.add_subparsers(dest="what", required=True)
    one = sub.add_parser("attempts", help="score a trial's attempts against the tape")
    one.add_argument("--episodes", required=True)
    one.add_argument("--store", required=True)
    one.add_argument("--snapshots", default=None)
    one.add_argument("--truth", required=True)
    one.add_argument("--out", required=True)
    two = sub.add_parser("plan", help="pairs needed for given improvement rates")
    two.add_argument("chosen_rate", type=float)
    two.add_argument("relook_rates", type=float, nargs="+")
    args = parser.parse_args(argv)
    if args.what == "plan":
        for rate in args.relook_rates:
            need = pairs_needed(args.chosen_rate, rate)
            print(f"chosen {args.chosen_rate:.0%}, re-look {rate:.0%}: "
                  + (f"{need} pairs" if need else "cannot be told apart"))
        return 0
    assert not os.path.exists(args.out), "choose a new output file"
    truth = json.loads(Path(args.truth).read_text())
    scored = score_attempts(attempts_in(args.episodes), truth, args.store, args.snapshots)
    report = {"summary": summary(scored), "attempts": scored,
              "score": {"floor": round(FLOOR, 4), "axis_of_stated": AXIS_OF_STATED}}
    Path(args.out).write_text(json.dumps(report, indent=1) + "\n")
    print(json.dumps(report["summary"], indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
