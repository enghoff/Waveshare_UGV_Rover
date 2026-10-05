"""Revocable grouping previews over a snapshot; the resolver never reads them.

No live entity, exemplar, placement, observation, or merge journal is changed.
The existing merge implementation works only on a disposable SQLite backup.
These are review candidates, not persistent aliases or navigation destinations.
R-WS-13, R-WS-17 and R-WS-18 still require independently labelled acceptance.
"""
from __future__ import annotations

import hashlib
import json
import tempfile
import threading
import time

from . import merging
from .store import WorldStore

_preview_lock = threading.Lock()
MAX_ROUNDS = 10


def revision(store):
    """Identify the membership/placement evidence in a reader's snapshot."""
    with store._lock:
        evidence = {
            "generation": store.generation(), "map_session": store.map_session(),
            "observations": [tuple(row) for row in store.db.execute(
                "SELECT id,entity_id,observed_at,inference_id FROM observations ORDER BY id")],
            "entities": [tuple(row) for row in store.db.execute(
                "SELECT id,placement_json,placement_map_session,observation_count,"
                "last_seen_at FROM entities ORDER BY id")],
        }
    return hashlib.sha256(json.dumps(evidence, separators=(",", ":")).encode()).hexdigest()


def preview(store, reach=None):
    """Recompute from original records on every call, including withdrawn groups.

    Only the backup holds the source lock. A concurrent look can make the result
    stale; the caller receives that fact rather than a claim about the latest store.
    Concurrent previews are refused, keeping one bounded copy per daemon.
    """
    if not _preview_lock.acquire(blocking=False):
        return {"ok": False, "error": "a grouping preview is already running"}
    began = time.monotonic()
    try:
        with tempfile.TemporaryDirectory(prefix="ugv-reader-groups-") as directory:
            clone = WorldStore(directory)
            try:
                with store._lock:
                    if store.db.in_transaction:
                        return {"ok": False, "error": "world state has an unfinished write"}
                    store.db.backup(clone.db)
                # A historical capture may predate the merge journal. Upgrade
                # only the disposable copy; never migrate the source here.
                clone._create()
                snapshot = revision(clone)
                generation, session = clone.generation(), clone.map_session()
                members = {row["id"]: {row["id"]} for row in clone.db.execute(
                    "SELECT id FROM entities WHERE placement_map_session=?", (session,))}
                events = []
                converged = False
                for round_number in range(1, MAX_ROUNDS + 1):
                    proposals = merging.propose(clone)["proposals"]
                    if not proposals:
                        converged = True
                        break
                    outcome = merging.apply(clone, [[p["keep"], p["gone"]]
                                                   for p in proposals], reach=reach)
                    if not outcome.get("ok"):
                        return {"ok": False, "error": "snapshot grouping failed", "detail": outcome}
                    for joined in outcome["joined"]:
                        keep, gone = joined["keep"], joined["gone"]
                        members[keep].update(members.pop(gone))
                    events.append({"round": round_number, "proposals": proposals})
                if not converged:
                    converged = not merging.propose(clone)["proposals"]
                groups = [{"representative": key, "members": sorted(value)}
                          for key, value in sorted(members.items()) if len(value) > 1]
                observation_count = clone.db.execute("SELECT COUNT(*) FROM observations").fetchone()[0]
            finally:
                clone.close()
        stale = revision(store) != snapshot
        return {"ok": True, "preview_only": True, "accepted": False,
                "world_generation": generation, "map_session": session,
                "snapshot_revision": snapshot, "stale": stale, "converged": converged,
                "observations": observation_count, "groups": groups, "rounds": events,
                "geometry_limit2": merging.MAX_ELLIPSE_DISTANCE2,
                "seconds": round(time.monotonic() - began, 2)}
    finally:
        _preview_lock.release()
