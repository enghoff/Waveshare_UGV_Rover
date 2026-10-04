"""Replay a frozen commit, diagnose live merges, and try reader-only groups.

Only temporary stores change. Historical observations, their labels, and the
running rover are never written. --source must name a clean archived checkout.
"""
from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import math
import json
from pathlib import Path
import sys
import tempfile
import time

import numpy as np

from audit import Evidence, LABELS, RECORDING, ROOT


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def run(source, output, fold, mode, geometric_gate=True, excluded=(), same_person=False,
        trace_decisions=False):
    sys.path.insert(0, str(source.resolve()))
    from world_state import merging, replay, resolve
    from world_state.store import WorldStore

    evidence = Evidence(excluded=excluded, same_person=same_person)
    model = evidence.fit(fold)
    merging.APPEARANCE_WEIGHTS = tuple(model["coef"])
    merging.APPEARANCE_OFFSET = model["offset"]
    real_propose = merging.propose
    gate_refusals = [0]

    def gated_propose(store):
        """Filter geometry BEFORE ranking; don't let an invalid pair occupy a slot."""
        if not geometric_gate:
            return real_propose(store)
        real_geometry = merging.geometry

        def gated_geometry(a, b, density):
            d = np.array([a.placement["x_m"] - b.placement["x_m"],
                          a.placement["y_m"] - b.placement["y_m"]])
            distance2 = float(d @ np.linalg.solve(a.covariance() + b.covariance(), d))
            if distance2 > 13.815510557964274:  # chi-square, 2 dimensions, 99.9%
                gate_refusals[0] += 1
                return -float("inf")
            return real_geometry(a, b, density)
        merging.geometry = gated_geometry
        try:
            return real_propose(store)
        finally:
            merging.geometry = real_geometry

    reach = replay.reach_from(str(RECORDING / "map.json"))
    groups = [g for g in replay.inspections(str(evidence.database)) if g[0]["map_session"] == 67]
    checkpoints = {max(1, round(len(groups) * n / 5)) for n in range(1, 6)}
    idmap, events, trace, scores = {}, [], [], []
    targets = {i for i, t, kind in evidence.items if kind == "main" and
               evidence.object_of[t] in {"object:246", "object:249"}}
    last_merge = None
    reader_alias = {}
    began = time.monotonic()

    def owners(store):
        return {idmap[r["id"]]: r["entity_id"] for r in store.db.execute(
            "SELECT id,entity_id FROM observations")}

    def merge_pass(store, frame, purpose):
        alias = {}
        for _ in range(10):
            proposals = gated_propose(store)["proposals"]
            if not proposals:
                break
            done = merging.apply(store, [[p["keep"], p["gone"]] for p in proposals], reach=reach)
            if not done.get("ok"):
                raise RuntimeError(done)
            for joined in done["joined"]:
                keep, gone = joined["keep"], joined["gone"]
                alias[gone] = keep
                events.append({"frame": frame, "purpose": purpose, **joined})
        for gone in list(alias):
            keep = alias[gone]
            while keep in alias:
                keep = alias[keep]
            alias[gone] = keep
        return alias

    def group_copy(store, frame):
        # Re-solve a disposable copy. The live resolver receives neither these
        # merged placements nor their combined exemplars on its next look.
        before = owners(store)
        with tempfile.TemporaryDirectory(prefix="entity-group-") as directory:
            clone = WorldStore(directory)
            try:
                store.db.backup(clone.db)
                alias = merge_pass(clone, frame, "reader_group")
            finally:
                clone.close()
        assert owners(store) == before, "reader groups changed resolver membership"
        return alias

    real_by_look = resolve._by_look

    def traced_by_look(store, group, entities, session, taken_in, reach=None):
        tracked = [o for o in group if idmap.get(o["id"]) in targets]
        records = []
        for observation in tracked:
            ray = resolve.ray_of(observation, reach)
            if ray is None:
                continue
            frame_id = observation.get("inference_id")
            already = taken_in.get(frame_id, store.entities_in_frame(frame_id))
            candidates = []
            for entity in entities:
                point = entity.get("placement") or {}
                used = resolve._allowance_used(point, ray)
                if used is None and entity["id"] not in {"object:6", "object:9"}:
                    continue
                seen = resolve.appearance(store, entity["id"], observation.get("dino_blob") or b"")
                fell = resolve.collapsed(store, entity["id"], observation, seen)
                qualifies = ((seen is None or seen >= resolve.DIFFERENT_THING)
                             and (fell is None or fell < resolve.COLLAPSED_ALONE))
                same_frame = [idmap[r["id"]] for r in store.db.execute(
                    "SELECT id FROM observations WHERE inference_id=? AND entity_id=?",
                    (frame_id, entity["id"]))]
                candidates.append({"entity": entity["id"], "used": used, "appearance": seen,
                                   "masked_drop": fell, "appearance_pass": qualifies,
                                   "geometry_pass": used is not None,
                                   "height_pass": resolve.locate.stands_as_high(point, ray),
                                   "range_pass": resolve.locate.stands_at_range(point, ray),
                                   "visibility_pass": not resolve.locate.beyond_reach(ray, (point['x_m'],point['y_m'])),
                                   "distance_m": math.hypot(point['x_m']-ray['x_m'],point['y_m']-ray['y_m']),
                                   "placement": point,
                                   "tolerance_m": resolve.locate.match_tolerance(point, ray),
                                   "already_in_frame": entity["id"] in already,
                                   "same_frame_looks": same_frame})
            records.append({"observation": idmap[observation["id"]],
                            "frame": current_frame[0], "inference": frame_id,
                            "candidates": candidates})
        decisions = real_by_look(store, group, entities, session, taken_in, reach)
        decisions_by_id = {d.observation_id: d for d in decisions}
        for record, observation in zip(records, [o for o in tracked if resolve.ray_of(o, reach) is not None]):
            decision = decisions_by_id.get(observation["id"])
            record["decision"] = None if decision is None else {
                "outcome": decision.outcome, "entity": decision.entity_id, "why": decision.why}
            trace.append(record)
        return decisions

    current_frame = [0]
    if trace_decisions:
        resolve._by_look = traced_by_look
    database_hash = digest(evidence.database)
    try:
        with tempfile.TemporaryDirectory(prefix="entity-audit-replay-") as directory:
            store = WorldStore(directory)
            try:
                with store.db:
                    store.db.execute("REPLACE INTO meta(key,value) VALUES('map_session','67')")
                for n, group in enumerate(groups, 1):
                    current_frame[0] = n
                    with store.db:
                        for row in group:
                            cursor = store.db.execute(
                                "INSERT INTO observations(entity_id," + ",".join(replay.COLUMNS)
                                + ") VALUES(NULL," + ",".join("?" for _ in replay.COLUMNS) + ")",
                                tuple(row.get(key) for key in replay.COLUMNS))
                            idmap[cursor.lastrowid] = row["id"]
                    outcome = resolve.resolve(store, reach=reach)
                    now = max(row["observed_at"] for row in group)
                    if last_merge is None:
                        last_merge = now
                    due = now - last_merge >= 300
                    if due:
                        last_merge = now
                        if mode == "live":
                            merge_pass(store, n, "live")
                        elif mode == "groups":
                            reader_alias = group_copy(store, n)
                    if n in checkpoints:
                        base = owners(store)
                        grouped = {i: reader_alias.get(e, e) for i, e in base.items()}
                        fresh = group_copy(store, n) if mode == "groups" else {}
                        fresh_owners = {i: fresh.get(e, e) for i, e in base.items()}
                        scores.append({"frame": n, "observations": len(base),
                                       "resolver_owners": base,
                                       "reader_owners": grouped,
                                       "fresh_owners": fresh_owners,
                                       "resolver": evidence.score(base, fold, set(base)),
                                       "reader_groups": evidence.score(grouped, fold, set(base)),
                                       "fresh_groups": evidence.score(fresh_owners, fold, set(base))})
                    if n % 50 == 0 or n == len(groups):
                        print(f"fold {fold} {mode}: {n}/{len(groups)} frames, "
                              f"{outcome['still_waiting']} waiting, "
                              f"{time.monotonic()-began:.0f}s", flush=True)
                base = owners(store)
                end_alias = merge_pass(store, len(groups), "end") if mode != "none" else {}
                final = owners(store)
                entities = store.db.execute("SELECT COUNT(*) FROM entities").fetchone()[0]
            finally:
                store.close()
    finally:
        resolve._by_look = real_by_look
    assert digest(evidence.database) == database_hash, "source recording changed"
    result = {"mode": mode, "fold": fold, "geometric_gate": geometric_gate,
              "excluded_observations": evidence.excluded,
              "head_and_body_one_person": same_person,
              "trace_decisions": trace_decisions,
              "source": str(source.resolve()), "model": model,
              "database_sha256": database_hash, "labels_sha256": digest(LABELS),
              "seconds": time.monotonic()-began, "entities": entities,
              "gate_refusals": gate_refusals[0], "checkpoints": scores,
              "before_end": evidence.score(base, fold), "final": evidence.score(final, fold),
              "owners_before_end": base, "owners_final": final,
              "trace": trace, "merge_events": events}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print("RESULT", mode, fold, {k: v for k,v in result['final'].items() if k!='per_object'}, flush=True)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fold", type=int, choices=(0, 1), required=True)
    parser.add_argument("--mode", choices=("none", "live", "groups"), required=True)
    parser.add_argument("--ungated", action="store_true")
    parser.add_argument("--exclude-observation", action="append", type=int, default=[])
    parser.add_argument("--same-person", action="store_true")
    parser.add_argument("--trace", action="store_true")
    args = parser.parse_args()
    run(args.source, args.output, args.fold, args.mode, not args.ungated, args.exclude_observation,
        args.same_person, args.trace)
