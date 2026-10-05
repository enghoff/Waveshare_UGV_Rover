import tempfile
import unittest
import numpy as np
from experiments.entity_association.tentative_evidence import TentativeStore,witnesses
from world_state.store import WorldStore


def row(i,frame,v=None,backend='a'):
    blob=np.array(v or [1.,0.],dtype='<f4').tobytes()
    return {'id':i,'inference_id':frame,'vectors_from':backend,'dino_blob':blob,'dino_alone_blob':blob}


class TentativeContracts(unittest.TestCase):
    def test_same_frame_cannot_supply_two_witnesses(self):
        self.assertEqual(len(witnesses(row(1,1),[row(2,2),row(3,2),row(4,1)])),1)

    def test_missing_mask_and_other_backend_abstain(self):
        bad=row(2,2);bad['dino_alone_blob']=None
        self.assertEqual(witnesses(row(1,1),[bad,row(3,3,backend='b')]),[])

    def test_poor_views_cannot_confirm_each_other_without_confirmed_support(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=WorldStore(tmp)
            try:
                eid=store.create_entity();a=row(1,1)['dino_blob'];b=row(3,3,[.6,.8])['dino_blob']
                with store.db:
                    for i in range(1,5):
                        v=a if i<3 else b
                        store.db.execute("INSERT INTO observations(id,observed_at,source,inference_id,dino_blob,dino_alone_blob,vectors_from) VALUES(?,?,'test',?,?,?,?)",(i,i,i,v,v,'a'))
                proxy=TentativeStore(store);proxy.attach(eid,[1,2]);proxy.attach(eid,[3,4]);proxy.promote()
                self.assertEqual(proxy.confirmed,{1,2})
            finally:store.close()

    def test_tentative_does_not_evict_confirmed_history_or_update_exemplars(self):
        with tempfile.TemporaryDirectory() as tmp:
            store=WorldStore(tmp)
            try:
                eid=store.create_entity();v=row(1,1)['dino_blob']
                with store.db:
                    for i in range(1,5):store.db.execute("INSERT INTO observations(id,observed_at,source,inference_id,dino_blob,dino_alone_blob,vectors_from) VALUES(?,?,'test',?,?,?,?)",(i,i,i,v,v,'a'))
                proxy=TentativeStore(store);proxy.attach(eid,[1,2]);proxy.add_exemplar(eid,v,alone=v)
                proxy.attach(eid,[3,4]);before=store.exemplars(eid)
                proxy.add_exemplar(eid,v,alone=v)
                self.assertEqual(store.exemplars(eid),before)
                self.assertEqual([r['id'] for r in proxy.observations(eid,limit=1)],[2])
                proxy.promote()
                self.assertTrue({3,4}<=proxy.confirmed)
                self.assertEqual(proxy.support,{3:[1,2],4:[1,2]})
            finally:store.close()


if __name__=='__main__':unittest.main()
