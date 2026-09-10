"""Chronological, source-ID-preserving comparison of bounded entity revision.

Reads recordings only; each run owns a fresh temporary world. No perception is
rerun and no bearing is remeasured. Timing includes SQLite and index maintenance,
but excludes reading the recording and scoring. See --help for examples.
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import ExitStack
import hashlib
import json
import os
import platform
from pathlib import Path
import sqlite3
import sys
import tempfile
import time
from unittest.mock import patch

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
    __package__ = "world_state"

from . import locate, resolve
from .replay import COLUMNS
from .store import WorldStore


def read_recording(path):
    with sqlite3.connect(Path(path).resolve().as_uri() + "?mode=ro", uri=True) as db:
        db.row_factory = sqlite3.Row
        rows = [dict(r) for r in db.execute("SELECT * FROM observations ORDER BY observed_at,id")]
    groups = {}
    for row in rows:
        groups.setdefault(row["inference_id"], []).append(row)
    return list(groups.values())


def percentile(values, fraction):
    return sorted(values)[min(len(values)-1, int((len(values)-1)*fraction))] if values else 0


def run(path, mode="baseline", output=None, repeat=1, max_frames=None, max_seconds=None):
    groups = read_recording(path)
    if max_frames:
        groups = groups[:max_frames]
    if not groups or repeat < 1:
        raise ValueError("a non-empty recording and a positive repeat count are required")
    fingerprint = hashlib.sha256()
    for group in groups:
        for row in group:
            for column in ("id",*COLUMNS):
                value=row.get(column)
                data=value if isinstance(value,bytes) else json.dumps(value,sort_keys=True).encode()
                fingerprint.update(len(data).to_bytes(8,"little"))
                fingerprint.update(data)
    counters = Counter()
    times, trace = [], []
    def counted(name, fn):
        def call(*args, **kwargs):
            counters[name] += 1
            return fn(*args, **kwargs)
        return call
    with tempfile.TemporaryDirectory(prefix="world-incremental-") as directory, ExitStack() as stack:
        store = WorldStore(directory)
        stack.callback(store.close)
        for module, name in ((locate,"fix"), (locate,"agrees"),
                             (resolve,"_allowance_used"), (resolve,"similarity")):
            stack.enter_context(patch.object(module,name,counted(name,getattr(module,name))))
        engine = None
        cache_counts=None
        if mode == "cached":
            from .cached_geometry import cached_geometry
            cache_counts=stack.enter_context(cached_geometry())
        elif mode != "baseline":
            from .incremental import IncrementalResolver
            engine = IncrementalResolver(store, revision=mode in ("revision","negative"),
                                         negative=mode == "negative", frames=Path(path).parent/"frames")
        # Prefix repetition is explicitly a workload test, not fresh evidence.
        offset = max(row["id"] for group in groups for row in group) + 1
        frame_offset = max(int(group[0]["inference_id"] or 0) for group in groups) + 1
        timestamps=[row["observed_at"] for group in groups for row in group]
        time_offset=max(timestamps)-min(timestamps)+1
        resolve._solver()  # One-time SciPy import excluded equally from all modes.
        started = time.perf_counter()
        for epoch in range(repeat):
            for number, original in enumerate(groups):
                if max_seconds is not None and time.perf_counter()-started>=max_seconds:
                    break
                group = [dict(row, id=row["id"]+epoch*offset,
                              observed_at=row["observed_at"]+epoch*time_offset,
                              inference_id=(row["inference_id"] or 0)+epoch*frame_offset)
                         for row in original]
                tick = time.perf_counter()
                session = group[0]["map_session"]
                with store._lock, store.db:
                    store.db.execute("REPLACE INTO meta(key,value) VALUES('map_session',?)",(str(session),))
                    for row in group:
                        store.db.execute("INSERT INTO observations(id,entity_id,"+",".join(COLUMNS)+") VALUES(?,NULL,"+
                                         ",".join("?" for _ in COLUMNS)+")",
                                         (row["id"],*(row.get(c) for c in COLUMNS)))
                outcome = (resolve.resolve(store) if engine is None else engine.update(group))
                elapsed = time.perf_counter()-tick
                times.append(elapsed)
                trace.append({"frame":len(times),"ms":elapsed*1000,
                              "observations":sum(len(g) for g in groups[:number+1])+epoch*sum(map(len,groups)),
                              "work":dict(counters)})
                if len(times)%25 == 0:
                    print(f"{mode}: {len(times)}/{len(groups)*repeat} frames, {time.perf_counter()-started:.1f}s",flush=True)
        seconds = time.perf_counter()-started
        membership = {str(r["id"]):r["entity_id"] for r in store.db.execute("SELECT id,entity_id FROM observations")}
        entities = {}
        for row in store.db.execute("SELECT id,placement_json FROM entities"):
            entities[row["id"]] = {"placement":json.loads(row["placement_json"] or "null"),"looks":[]}
        for oid,eid in membership.items():
            if eid in entities:
                entities[eid]["looks"].append(int(oid))
        payload = {"recording":str(path),"mode":mode,"repeat":repeat,"frames":len(times),
                   "complete":len(times)==len(groups)*repeat,"intended_frames":len(groups)*repeat,
                   "observations":len(membership),"attached":sum(e is not None for e in membership.values()),
                   "entities":entities,"membership":membership,"seconds":seconds,
                   "p50_ms":percentile(times,.5)*1000,"p95_ms":percentile(times,.95)*1000,
                   "max_ms":max(times,default=0)*1000,"work":dict(counters),"trace":trace,
                   "engine":engine.stats() if engine else dict(cache_counts or {}),
                   "source_sha256":hashlib.sha256(Path(path).read_bytes()).hexdigest(),
                   "input_rows_sha256":fingerprint.hexdigest(),
                   "environment":{"platform":platform.platform(),"python":sys.version,"processor":os.environ.get("PROCESSOR_IDENTIFIER",platform.processor())},
                   "implementation_sha256":{name:hashlib.sha256((Path(__file__).parent/name).read_bytes()).hexdigest() for name in ("resolve.py","incremental.py","negative_evidence.py","cached_geometry.py","bench_incremental.py")},
                   "limitations":["No contemporaneous map snapshots: wall gate disabled equally in all modes.",
                                   "Stored detections only; empty or skipped frames are not replayed.",
                                   "Repeated input is a workload test, not independent accuracy evidence."]}
        if output:
            Path(output).parent.mkdir(parents=True,exist_ok=True)
            Path(output).write_text(json.dumps(payload,indent=2),encoding="utf-8")
        print(json.dumps({k:payload[k] for k in ("mode","frames","observations","attached","seconds","p50_ms","p95_ms","work")}),flush=True)
        return payload


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording")
    parser.add_argument("--mode",choices=("baseline","cached","shortlist","revision","negative"),default="baseline")
    parser.add_argument("--output",required=True)
    parser.add_argument("--repeat",type=int,default=1)
    parser.add_argument("--max-frames",type=int)
    parser.add_argument("--max-seconds",type=float)
    args=parser.parse_args()
    run(args.recording,args.mode,args.output,args.repeat,args.max_frames,args.max_seconds)


if __name__ == "__main__":
    main()
