"""Offline regression tests for revision, evidence, indexing and benchmark scoring."""
import json
from contextlib import closing
from pathlib import Path
import sqlite3
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from world_state.incremental import AppearanceIndex, IncrementalResolver, decoded
from world_state.negative_evidence import DepthEvidence
from world_state.score_incremental import score
from world_state.cached_geometry import cached_geometry
from world_state import locate, resolve
from world_state.store import WorldStore


def observation(oid,vector,alone=None,x=0,y=0,bearing=45):
    return dict(id=oid,observed_at=float(oid),inference_id=oid,map_session=1,source="test",
                observer_pose_json=json.dumps(dict(x_m=x,y_m=y,heading_deg=0)),
                bearing_deg=bearing,bearing_sigma_deg=1.5,span_deg=5,
                bbox_json="[0.4,0.4,0.6,0.6]",elevation_deg=0,
                dino_blob=np.asarray(vector,dtype="<f4").tobytes(),
                dino_alone_blob=np.asarray(alone if alone is not None else vector,dtype="<f4").tobytes(),
                vectors_from="test",frame_id=f"frame-{oid}")


def insert(store,row):
    with store.db:
        store.db.execute("INSERT INTO observations("+",".join(row)+") VALUES("+",".join("?" for _ in row)+")",tuple(row.values()))


class IncrementalTests(unittest.TestCase):
    def test_each_view_retrieves_its_own_geometric_candidates(self):
        with tempfile.TemporaryDirectory() as directory, closing(WorldStore(directory)) as store:
            east,west=store.create_entity(),store.create_entity()
            store.place(east,dict(x_m=4,y_m=0,uncertainty_m=.1),1)
            store.place(west,dict(x_m=-4,y_m=0,uncertainty_m=.1),1)
            engine=IncrementalResolver(store)
            engine.session=1
            right=decoded(observation(1,[1,0],bearing=0))
            left=decoded(observation(2,[1,0],bearing=180))
            self.assertEqual([r["id"] for r in engine._candidates_for([right])],[east])
            self.assertEqual([r["id"] for r in engine._candidates_for([left])],[west])

    def test_old_observation_is_retrievable_without_scanning_recent_window(self):
        db=sqlite3.connect(":memory:")
        index=AppearanceIndex(db)
        old=observation(1,[1,0,0,0])
        index.add(old)
        for oid in range(2,600):
            index.add(observation(oid,[-1,0,0,0]))
        self.assertIn(1,index.query(observation(700,[1,0,0,0])))
        self.assertNotIn(1,index.query(dict(old,vectors_from="other-backend")))
        db.close()

    def test_revising_one_bad_attachment_preserves_measurement_and_clears_template(self):
        with tempfile.TemporaryDirectory() as directory, closing(WorldStore(directory)) as store:
            rows=[observation(i,[1,0],[1,0],x=float(i),bearing=90) for i in range(1,5)]
            rows.append(observation(5,[1,0],[0,1],x=5,bearing=90))
            for row in rows:insert(store,row)
            eid=store.create_entity()
            store.attach(eid,[1,2,3,4,5])
            store.add_exemplar(eid,rows[-1]["dino_blob"],alone=rows[-1]["dino_alone_blob"])
            engine=IncrementalResolver(store)
            engine.session=1
            engine._audit(eid)
            after=store.db.execute("SELECT * FROM observations WHERE id=5").fetchone()
            self.assertIsNone(after["entity_id"])
            self.assertEqual(after["dino_alone_blob"],rows[-1]["dino_alone_blob"])
            self.assertFalse(engine.proxy.association_allowed(eid,5))
            self.assertEqual(store.db.execute("SELECT observation_count FROM entities WHERE id=?",(eid,)).fetchone()[0],4)
            self.assertEqual(engine.counts["detached"],1)

    def test_accepted_attachment_can_be_vetoed_on_revisit(self):
        with tempfile.TemporaryDirectory() as directory, closing(WorldStore(directory)) as store:
            row=observation(9,[1,0],x=0,y=0,bearing=45)
            insert(store,row)
            eid=store.create_entity()
            p=dict(x_m=3,y_m=3,uncertainty_m=.1,extent_m=.2)
            store.place(eid,p,1)
            engine=IncrementalResolver(store)
            engine.candidates=store.placed(map_session=1)
            engine.rejected_pairs.add((eid,9))
            self.assertEqual(resolve._by_look(engine.proxy,[decoded(row)],engine.candidates,1,{}),[])
            self.assertIsNone(store.db.execute("SELECT entity_id FROM observations WHERE id=9").fetchone()[0])

    def test_negative_evidence_requires_distinct_viewpoints_and_resets_on_move(self):
        evidence=DepthEvidence(None)
        entity={"id":"thing","placement":{"x_m":3,"y_m":2,"height_m":.5}}
        with patch.object(evidence,"evidence",return_value="clear"):
            def at(x):return evidence.observe(entity,[{"pose":{"x_m":x,"y_m":0}}])
            self.assertFalse(at(0))
            self.assertFalse(at(0))
            self.assertFalse(at(.4))
            entity["placement"]["x_m"]=4
            self.assertFalse(at(.8))
            self.assertFalse(at(1.2))
            self.assertTrue(at(1.6))

    def test_unknown_depth_does_not_challenge_an_entity(self):
        evidence=DepthEvidence(None)
        self.assertEqual(evidence.evidence({"height_m":.5},{"frame_id":"missing","pose":{}}),"unknown")

    def test_depth_holes_and_occluders_abstain(self):
        from types import SimpleNamespace
        evidence=DepthEvidence(None)
        evidence.lens=SimpleNamespace(fx=100,fy=100,cx=100,cy=100,width=200,height=200)
        row={"frame_id":"test","pose":{"x_m":0,"y_m":0,"heading_deg":0},"bearing_deg":0}
        point={"x_m":2,"y_m":0,"height_m":0,"uncertainty_m":.05}
        far=np.full((200,200),5.)
        with patch.object(evidence,"_load",return_value=far),patch("world_state.oak._in_oak",return_value=(0,0,2)):
            self.assertEqual(evidence.evidence(point,row),"clear")
            far[100,100]=0
            self.assertEqual(evidence.evidence(point,row),"unknown")
            far[100,100]=1
            self.assertEqual(evidence.evidence(point,row),"unknown")

    def test_corrected_split_is_scored_as_separate_and_unassigned_is_not_a_success(self):
        labels={"things":{"one":{"verdict":"object","looks":[1,2]}}}
        cases={"cases":[{"name":"different","groups":[[1,2],[3]]}]}
        result={"membership":{"1":"a","2":"a","3":"b"}}
        self.assertEqual(score(result,labels,cases)["cases_separated"],1)
        result["membership"]["3"]=None
        self.assertEqual(score(result,labels,cases)["cases_unresolved"],1)
        result["membership"]["2"]="b"
        self.assertEqual(score(result,labels,cases)["clean_pairs_split"],1)

    def test_cache_copies_results_and_keys_measurement_changes(self):
        a={"x_m":0.,"y_m":0.,"bearing_deg":45.}
        b={"x_m":6.,"y_m":0.,"bearing_deg":135.}
        with cached_geometry(capacity=2) as counts:
            first=locate.fix(a,b)
            self.assertIsNotNone(first)
            first["x_m"]=100
            self.assertAlmostEqual(locate.fix(a,b)["x_m"],3)
            b["bearing_deg"]=90
            locate.fix(a,b)
            self.assertEqual(counts["fix_hits"],1)
            self.assertEqual(counts["fix_misses"],2)


if __name__=="__main__":
    unittest.main()
