"""Experimental bounded resolver, opt-in through bench_incremental only.

The archive is authoritative. Indexes retrieve a bounded working set; they do
not delete old measurements. Existing geometry and assignment remain in use.
Revisions detach unsupported memberships and refit their former owner before
offering the observations for discovery again. No daemon imports this module.
"""
from __future__ import annotations

from collections import Counter, deque
import json
import math
from pathlib import Path

import numpy as np

from . import locate, resolve

PENDING = 48
CANDIDATES = 24
BUCKET = 12
AUDIT_SIZE = 24
AUDITS_PER_FRAME = 2


def decoded(row):
    row = dict(row)
    row["pose"] = json.loads(row.get("observer_pose_json") or "null")
    row["bbox"] = json.loads(row.get("bbox_json") or "null")
    return row


class AppearanceIndex:
    """Four deterministic random-projection tables; bounded indexed SQL reads.

    Query cost does not scan all stored vectors. Each bucket is ordered by
    observation ID; overflow is measured, so approximate retrieval is explicit.
    """
    def __init__(self, db):
        self.db = db
        self.planes = {}
        db.execute("CREATE TABLE IF NOT EXISTS incremental_keys (backend TEXT, table_no INT, code INT, oid INT, PRIMARY KEY(backend,table_no,code,oid))")
        self.overflow = 0

    def keys(self, row):
        blob = row.get("dino_blob")
        if not blob:
            return []
        vector = np.frombuffer(blob,dtype="<f4")
        if len(vector) not in self.planes:
            self.planes[len(vector)] = np.random.default_rng(73).normal(size=(4,10,len(vector))).astype("float32")
        signs = np.einsum("thd,d->th",self.planes[len(vector)],vector)>0
        return [(str(row.get("vectors_from") or ""),i,int(sum(int(b)<<j for j,b in enumerate(bits))))
                for i,bits in enumerate(signs)]

    def add(self,row):
        self.db.executemany("INSERT OR IGNORE INTO incremental_keys VALUES(?,?,?,?)",
                            [(*key,row["id"]) for key in self.keys(row)])

    def query(self,row):
        ids = set()
        for key in self.keys(row):
            found = self.db.execute("SELECT oid FROM incremental_keys WHERE backend=? AND table_no=? AND code=? ORDER BY oid DESC LIMIT ?",
                                    (*key,BUCKET+1)).fetchall()
            self.overflow += len(found)>BUCKET
            ids.update(r[0] for r in found[:BUCKET])
        return ids


class WorkingStore:
    def __init__(self, engine):
        self.engine = engine

    def __getattr__(self,name):
        return getattr(self.engine.store,name)

    def unplaced(self,**kwargs):
        return self.engine.pending

    def placed(self,**kwargs):
        return self.engine.candidates

    def association_allowed(self,eid,oid):
        return (eid,oid) not in self.engine.rejected_pairs

    def attach(self,eid,ids,why=""):
        self.engine.changed.add(eid)
        return self.engine.store.attach(eid,ids,why)

    def place(self,eid,placement,session):
        self.engine.changed.add(eid)
        return self.engine.store.place(eid,placement,session)

    def add_exemplar(self,*args,**kwargs):
        return self.engine.store.add_exemplar(*args,**kwargs)


class IncrementalResolver:
    def __init__(self,store,*,revision=True,negative=False,frames=None):
        self.store = store
        self.proxy = WorkingStore(self)
        self.index = AppearanceIndex(store.db)
        self.revision = revision
        self.negative = negative
        self.frames = Path(frames) if frames else None
        self.entities = {}
        self.cells = {}
        self.entity_cells = {}
        self.pending = []
        self.candidates = []
        self.changed = set()
        self.queue = deque()
        self.queued = set()
        self.reopened = set()
        self.rejected_pairs = set()
        self.counts = Counter()
        self.events = []
        self.session = None
        self.frame_number = 0
        self.cooldown = {}
        self.views = None
        if negative:
            from .negative_evidence import DepthEvidence
            self.views = DepthEvidence(self.frames)
        for entity in store.placed():
            self._refresh(entity["id"])

    def _refresh(self,eid):
        for cell in self.entity_cells.pop(eid,()):
            self.cells[cell].pop(eid,None)
            if not self.cells[cell]:
                del self.cells[cell]
        row=self.store.db.execute("SELECT * FROM entities WHERE id=?",(eid,)).fetchone()
        if row is None or not row["placement_json"]:
            self.entities.pop(eid,None)
            return
        entity=dict(row)
        entity["placement"]=json.loads(row["placement_json"])
        self.entities[eid]=entity
        p=entity["placement"]
        # A bounded uncertainty envelope. Broader hypotheses still have the
        # independent appearance retrieval path and are counted for review.
        radius=min(2.0,max(.5,float(p.get("uncertainty_m") or 0)))
        cells=[]
        for x in range(math.floor(p["x_m"]-radius),math.floor(p["x_m"]+radius)+1):
            for y in range(math.floor(p["y_m"]-radius),math.floor(p["y_m"]+radius)+1):
                cell=(entity["placement_map_session"],x,y)
                self.cells.setdefault(cell,{})[eid]=None
                cells.append(cell)
        self.entity_cells[eid]=cells

    def _near(self,rows):
        found=set()
        for row in rows:
            ray=resolve.ray_of(row)
            if not ray:
                continue
            angle=math.radians(ray["bearing_deg"])
            # Follow the whole bearing, including a wrongly ranged observation.
            for distance in np.arange(0,locate.MAX_RANGE_M+.5,.75):
                x=math.floor(ray["x_m"]+distance*math.cos(angle))
                y=math.floor(ray["y_m"]+distance*math.sin(angle))
                for dx,dy in ((0,0),(1,0),(-1,0),(0,1),(0,-1)):
                    bucket=self.cells.get((self.session,x+dx,y+dy),())
                    # Avoid sorting/scanning unbounded dense buckets.
                    import itertools
                    found.update(itertools.islice(bucket,64))
                    self.counts["spatial_overflow"] += len(bucket)>64
        return found

    def _rows(self,ids):
        if not ids:
            return []
        ids=sorted(ids)
        return [decoded(r) for r in self.store.db.execute(
            "SELECT * FROM observations WHERE id IN ("+",".join("?" for _ in ids)+")",ids)]

    def _select(self,current):
        related=set()
        for row in current:
            related.update(self.index.query(row))
        archive=self._rows(related)
        # Newest pending is the fallback, related old evidence is retrieved
        # independently. A row detached during audit receives priority once.
        recent=self.store.unplaced(map_session=self.session,limit=PENDING)
        waiting={r["id"]:r for r in recent}
        waiting.update({r["id"]:r for r in archive if r["entity_id"] is None and r["map_session"]==self.session and r.get("bearing_deg") is not None})
        waiting.update({r["id"]:r for r in self._rows(self.reopened) if r["entity_id"] is None})
        now={r["id"] for r in current}
        order=sorted(waiting,key=lambda oid:(oid not in now,oid not in self.reopened,oid not in related,-oid))
        self.counts["pending_deferred"] += max(0,len(order)-PENDING)
        self.pending=[waiting[oid] for oid in sorted(order[:PENDING])]
        self.reopened.difference_update(order[:PENDING])
        self.counts["max_pending"]=max(self.counts["max_pending"],len(self.pending))

    def _candidates_for(self,current):
        # Each historical view needs its own shortlist. Sharing the latest
        # camera view's shortlist with older pending views causes fragmentation.
        related=set()
        for row in current:
            related.update(self.index.query(row))
        archive=self._rows(related)
        spatial=self._near(current)
        appearance={r["entity_id"] for r in archive if r["entity_id"] in self.entities and r["map_session"]==self.session}
        ids=spatial|appearance
        # Rank the bounded spatial result with current rays. Archive appearance
        # candidates can compete even when the old placement is implausible.
        rays=[r for r in (resolve.ray_of(o) for o in current) if r]
        def rank(eid):
            p=self.entities[eid]["placement"]
            values=[resolve._allowance_used(p,r) for r in rays]
            cost=min((v for v in values if v is not None),default=10.0)
            return (cost-(.1 if eid in appearance else 0),eid)
        ranked=sorted(ids,key=rank)
        self.counts["candidates_deferred"] += max(0,len(ranked)-CANDIDATES)
        candidates=[self.entities[eid] for eid in ranked[:CANDIDATES]]
        self.counts["max_candidates"]=max(self.counts["max_candidates"],len(candidates))
        return candidates

    def _enqueue(self,eid):
        if eid not in self.queued:
            self.queue.append(eid)
            self.queued.add(eid)

    def _detach(self,eid,ids,reason):
        if not ids:
            return
        with self.store.db:
            self.store.db.executemany("UPDATE observations SET entity_id=NULL,note=? WHERE id=? AND entity_id=?",
                                      [(reason,oid,eid) for oid in ids])
            self.store.db.execute("UPDATE entities SET observation_count=(SELECT count(*) FROM observations WHERE entity_id=?) WHERE id=?",(eid,eid))
            self.store.db.execute("UPDATE entities SET exemplars=NULL,exemplars_alone=NULL WHERE id=?",(eid,))
        self.reopened.update(ids)
        self.rejected_pairs.update((eid,oid) for oid in ids)
        # Rebuild exemplars from surviving support; no rejected crop survives in
        # the model. Source images/vectors and original observation IDs survive.
        kept=self._history(eid)
        for row in kept:
            if row.get("dino_blob"):
                self.store.add_exemplar(eid,row["dino_blob"],alone=row.get("dino_alone_blob") or b"")
        self.store.place(eid,None,self.session)
        resolve._replace_placement(self.store,eid,self.session)
        self._refresh(eid)
        self.counts["detached"]+=len(ids)
        self.events.append({"frame":self.frame_number,"entity":eid,"observations":ids,"reason":reason})

    def _history(self,eid):
        # Retain founding viewpoints as well as recent ones. Raw columns are
        # necessary: the public history API intentionally strips vector blobs.
        old=list(self.store.db.execute("SELECT * FROM observations WHERE entity_id=? AND map_session=? ORDER BY observed_at,id LIMIT 8",(eid,self.session)))
        new=list(self.store.db.execute("SELECT * FROM observations WHERE entity_id=? AND map_session=? ORDER BY observed_at DESC,id DESC LIMIT 16",(eid,self.session)))
        return list({r["id"]:decoded(r) for r in old+new}.values())

    def _audit(self,eid):
        rows=self._history(eid)
        rows=[r for r in rows if r.get("map_session")==self.session and r.get("dino_alone_blob")]
        if len(rows)<4:
            return
        self.counts["audits"]+=1
        rejected=[]
        for row in rows:
            # Leave-one-out comparison: the observation cannot validate itself.
            peers=[other for other in rows if other["id"]!=row["id"] and other.get("inference_id")!=row.get("inference_id")]
            if len(peers)<3:
                continue
            full=float(np.median([resolve.similarity(row["dino_blob"],p["dino_blob"]) for p in peers]))
            alone=float(np.median([resolve.similarity(row["dino_alone_blob"],p["dino_alone_blob"]) for p in peers]))
            if full-alone>=resolve.COLLAPSED_ALONE and alone<resolve.DIFFERENT_THING:
                rejected.append(row["id"])
        # A whole model falling apart is unresolved; do not delete its majority
        # based on an uncalibrated appearance threshold.
        if rejected and len(rejected)<=len(rows)//3:
            self._detach(eid,rejected,"bounded revision: appearance support disappears when foreground is masked")

    def update(self,group):
        self.frame_number+=1
        self.session=group[0]["map_session"]
        current=[decoded(r) for r in group]
        with self.store.db:
            for row in current:
                self.index.add(row)
        self._select(current)
        self.changed.clear()
        resolve._UNIT.clear()
        resolve._ELSEWHERE.clear()
        groups={}
        for row in self.pending:
            groups.setdefault(row.get("inference_id"),[]).append(row)
        decisions=[]
        leftover=[]
        taken={}
        for rows in groups.values():
            candidates=self._candidates_for(rows)
            settled=resolve._by_look(self.proxy,rows,candidates,self.session,taken)
            spoken={d.observation_id for d in settled}
            decisions.extend(settled)
            leftover.extend(r for r in rows if r["id"] not in spoken)
        self.candidates=self._candidates_for(leftover) if leftover else []
        decisions.extend(resolve.DISCOVERY(self.proxy,leftover,self.session,self.candidates,taken))
        resolve._UNIT.clear()
        resolve._ELSEWHERE.clear()
        outcome={"matched":sum(d.outcome==resolve.MATCH for d in decisions),
                 "created":sum(d.outcome==resolve.NEW for d in decisions),
                 "ambiguous":sum(d.outcome==resolve.AMBIGUOUS for d in decisions)}
        for eid in sorted(self.changed):
            self._refresh(eid)
            if self.revision and self.frame_number-self.cooldown.get(eid,-100)>=4:
                self._enqueue(eid)
        if self.revision:
            for _ in range(min(AUDITS_PER_FRAME,len(self.queue))):
                eid=self.queue.popleft()
                self.queued.discard(eid)
                self.cooldown[eid]=self.frame_number
                self._audit(eid)
        if self.views:
            # Query the fixed depth camera's view, including directions with no
            # detected region. A detector-only shortlist would miss empty space.
            depth_rays=[]
            for row in current[:1]:
                if row.get("pose") and row.get("bearing_deg") is not None:
                    for offset in (-30,0,30):
                        depth_rays.append(dict(row,bearing_deg=row["pose"]["heading_deg"]+offset))
            visible=sorted(self._near(depth_rays))
            self.counts["negative_candidates_deferred"]+=max(0,len(visible)-CANDIDATES)
            # Round-robin candidates in crowded views instead of starving high IDs.
            start=(self.frame_number*CANDIDATES)%max(1,len(visible))
            visible=(visible[start:]+visible[:start])[:CANDIDATES]
            for eid in visible:
                entity=self.entities.get(eid)
                if entity and self.views.observe(entity,current):
                    self.counts["negative_challenges"]+=1
                    self._enqueue(eid)
                    # Withdrawal challenges the position, not object existence.
                    self.store.place(eid,None,self.session)
                    self._refresh(eid)
                    self.events.append({"frame":self.frame_number,"entity":eid,"reason":"repeated clear depth contradicts placement"})
        self.counts["max_revision_queue"]=max(self.counts["max_revision_queue"],len(self.queue))
        return outcome

    def stats(self):
        return {**dict(self.counts),"appearance_bucket_overflow":self.index.overflow,
                "revision_queue_remaining":len(self.queue),"events":self.events,
                "negative_evidence":self.views.stats() if self.views else None}
