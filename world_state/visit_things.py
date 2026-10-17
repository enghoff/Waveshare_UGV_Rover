"""The world model's records as the store's things, for `search.ThingIndex`.

Version 4 of the world model kept per visit (`visit_model/`) runs as a job of
its own on the rover and writes its records to `visit_model/things.json` in
the store's directory: where each thing is, whether it has been seen gone, and
which of the store's observations it holds. `VisitStore` answers the two
questions `ThingIndex` asks of a store -- what has changed, and what the
changed things' looks are -- from those records, with the looks' vectors read
from the store as they always are, so ranking a phrase against things works
unchanged and only what a thing is changes.

`fresh` says whether the records can be used: written recently, and in the
map session the store is in. When they cannot, a caller answers from the
store's own things, as it did before the model ran (docs/plans/
a-world-model-kept-per-visit.md, step 8).
"""
import json
import os
import time
from typing import Any

THINGS_FILE = os.path.join("visit_model", "things.json")
#: Older than this, the job has stopped, and the store's own things answer.
MAX_AGE_S = 900.0


class VisitStore:
    def __init__(self, store) -> None:
        self.store = store
        self.path = os.path.join(store.dir, THINGS_FILE)
        self._stamp = None
        self.payload: dict[str, Any] = {}
        self.records: dict[str, dict[str, Any]] = {}
        self.owner: dict[int, str] = {}
        self._looks: dict[int, tuple[str | None, int]] = {}
        self._taken: dict[int, float] = {}
        self._newest: dict[str, float] | None = None

    def load(self) -> bool:
        """Read the records again if the job has written them since; whether
        there are any."""
        # The job writes a new file and renames it over the old one, so the
        # inode changes with every write; a modification time alone can repeat
        # within one clock tick (the rover's selftest wrote three in one).
        try:
            st = os.stat(self.path)
            stamp = (st.st_ino, st.st_mtime_ns, st.st_size)
        except OSError:
            self._stamp, self.payload, self.records, self.owner = None, {}, {}, {}
            return False
        if stamp != self._stamp:
            try:
                with open(self.path) as f:
                    payload = json.load(f)
            except (OSError, ValueError):
                return bool(self.records)
            self._stamp, self.payload = stamp, payload
            self.records = {r["id"]: r for r in payload.get("things") or []}
            self.owner = {int(i): r["id"] for r in self.records.values() for i in r["observations"]}
            self._newest = None
        return bool(self.records)

    def fresh(self) -> str:
        """Why the records cannot answer now, or "" when they can."""
        if not self.load():
            return "the world model has written no records"
        age = time.time() - float(self.payload.get("written_at") or 0.0)
        if age > MAX_AGE_S:
            return "the world model's records are %d minutes old" % round(age / 60.0)
        # The records are in the coordinates of the map session the job reads
        # (visit_model/data.py's MAP_SESSION), which it writes beside them.
        if self.store.map_session() != self.payload.get("map_session"):
            return "the world model is on another map"
        return ""

    def generation(self) -> str:
        return "visit:%s" % self.store.generation()

    def _backends(self) -> dict[int, tuple[str, int]]:
        """(backend, id) of every held observation that has a searchable vector,
        read once each, with when it was taken: an observation's vector and time
        are never rewritten."""
        missing = [i for i in self.owner if i not in self._looks]
        for k in range(0, len(missing), 900):
            part = missing[k:k + 900]
            with self.store._lock:
                rows = self.store.db.execute(
                    "SELECT id, vectors_from, siglip_blob IS NOT NULL, observed_at FROM observations"
                    f" WHERE id IN ({','.join('?' * len(part))})", part).fetchall()
            found = {int(r[0]): ((r[1] or "") if r[2] else None, float(r[3] or 0.0)) for r in rows}
            for i in part:
                backend, taken = found.get(i, (None, 0.0))
                self._looks[i] = (backend, i)
                self._taken[i] = taken
        return {i: v for i, v in self._looks.items() if i in self.owner and v[0] is not None}

    def newest_look(self, record_id: str) -> float | None:
        """When the newest look a record holds was taken.

        **Not the record's own `seen_at`**: when the model folds one record into
        another it keeps the survivor's time (model.Memory.merge), so a painting
        whose looks run to this morning could say it was last seen a week ago
        (2026-10-17, on the rover)."""
        if self._newest is None:
            # Once per version of the records: asked for every record in a
            # list, this was the whole of a 13 s answer when each asking read
            # every look again (2026-10-17).
            self._backends()
            newest: dict[str, float] = {}
            for i, rid in self.owner.items():
                t = self._taken.get(i, 0.0)
                if t > newest.get(rid, 0.0):
                    newest[rid] = t
            self._newest = newest
        return self._newest.get(record_id)

    def thing_prints(self) -> dict[tuple[str, str], tuple[int, int, int]]:
        """As `Store.thing_prints`: per record and backend, how many looks with
        a vector, and the largest and the sum of their identifiers."""
        self.load()
        out: dict[tuple[str, str], list[int]] = {}
        for i, (backend, _i) in self._backends().items():
            key = (self.owner[i], backend)
            c = out.setdefault(key, [0, 0, 0])
            c[0] += 1
            c[1] = max(c[1], i)
            c[2] += i
        return {k: tuple(v) for k, v in out.items()}

    def thing_looks(self, record_ids: list[str] | None = None, page: int = 900):
        """As `Store.thing_looks`: (record, backend, vector, box) for every
        searchable look of these records, or of all of them."""
        wanted = set(self.records) if record_ids is None else set(record_ids)
        ids = sorted(i for i, r in self.owner.items() if r in wanted)
        for k in range(0, len(ids), page):
            part = ids[k:k + page]
            with self.store._lock:
                rows = self.store.db.execute(
                    "SELECT id, vectors_from, siglip_blob, bbox_json FROM observations"
                    " WHERE siglip_blob IS NOT NULL"
                    f" AND id IN ({','.join('?' * len(part))})", part).fetchall()
            for row in rows:
                try:
                    box = json.loads(row[3]) if row[3] else None
                except (ValueError, TypeError):
                    box = None
                yield self.owner[int(row[0])], row[1] or "", row[2], box

    def entity(self, record_id: str) -> dict[str, Any] | None:
        """A record in the shape `find_thing` reads a store entity: counts,
        times and a placement in this map session."""
        r = self.records.get(record_id)
        if r is None:
            return None
        first = r["path"][0][2] if r.get("path") else r.get("since")
        return {
            "id": record_id,
            "observation_count": len(r["observations"]),
            "created_at": first,
            "last_seen_at": self.newest_look(record_id) or r.get("seen_at"),
            "placement_map_session": self.payload.get("map_session"),
            "placement": {
                "x_m": r["x_m"], "y_m": r["y_m"],
                "uncertainty_m": r.get("spread_m"),
                # As the console draws a store placement's ring: the model keeps
                # one spread, so a circle.
                "error_major_m": r.get("spread_m"),
                "error_minor_m": r.get("spread_m"),
                "stated_uncertainty_m": r.get("spread_m"),
                "viewpoints": r.get("visits"),
                # The store keeps a half-width; the model a width.
                "extent_m": None if r.get("extent_m") is None else r["extent_m"] / 2.0,
            },
            "gone_at": r.get("gone_at") if r.get("state") == "gone" else None,
            "moved_from": [p[:2] for p in r.get("path", [])[:-1]][-1:] or None,
            "source": "world model",
        }

