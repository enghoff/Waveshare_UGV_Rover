import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np

from experiments.entity_association.probe_clipped_appearance import observation_fingerprint
from experiments.entity_association.semantic_features import SemanticFeatures, reader_channel
from world_state import merging


def observation(i):
    blob=np.asarray([1.,0.],dtype='<f4').tobytes()
    return {'id':i,'frame_id':str(i),'bbox_json':'[0,0,1,1]','outline_blob':b'mask',
            'vectors_from':'test','dino_blob':blob,'dino_alone_blob':blob,'siglip_blob':blob,
            'inference_id':i}


def thing(i,looks):
    return merging._Thing({'id':str(i),'created_at':i,'placement_json':'{}'},looks)


class SemanticContracts(unittest.TestCase):
    def test_source_provenance_is_checked_and_missing_is_explicit(self):
        with tempfile.TemporaryDirectory() as tmp:
            directory=Path(tmp);row=observation(1)
            np.savez(directory/'vectors.npz',ids=[1],clipped=[[1.,0.]])
            (directory/'summary.json').write_text(json.dumps({'kind':'semantic','observations':[
                {'id':1,'input_sha256':observation_fingerprint(row)}]}))
            bank=SemanticFeatures(directory)
            self.assertEqual(bank.validate([row,observation(2)]),[2])
            with self.assertRaises(ValueError):bank.validate([{**row,'bbox_json':'[0,0,.5,1]'}])

    def test_reader_channel_adds_only_its_mean_and_restores_after_failure(self):
        bank=SimpleNamespace(vectors={1:np.array([1.,0.]),2:np.array([0.,1.]),3:np.array([1.,0.])})
        original_thing,original_appearance=merging._Thing,merging.appearance
        baseline=merging.appearance(thing(1,[observation(1),observation(2)]),thing(2,[observation(3)]))
        with reader_channel(bank,0.):
            self.assertEqual(merging.appearance(thing(1,[observation(1),observation(2)]),thing(2,[observation(3)])),baseline)
        with self.assertRaisesRegex(RuntimeError,'stop'):
            with reader_channel(bank,4.):
                self.assertAlmostEqual(merging.appearance(thing(1,[observation(1),observation(2)]),thing(2,[observation(3)])),baseline+2.)
                self.assertIsNone(merging.appearance(thing(1,[observation(1)]),thing(2,[observation(4)])))
                raise RuntimeError('stop')
        self.assertIs(merging._Thing,original_thing)
        self.assertIs(merging.appearance,original_appearance)


if __name__=='__main__':unittest.main()
