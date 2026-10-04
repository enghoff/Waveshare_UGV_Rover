"""Missing or inconsistent evidence must never become a forced refusal."""
import struct
import unittest

from experiments.entity_association.replay_attachment import AppearanceEvidence, experimental_gate
from world_state import resolve


def vector(a,b):
    return struct.pack('<2f',a,b)


def row(i, plain=None, masked=None, semantic=None, backend='tensorrt'):
    return {'id':i, 'dino_blob':plain or vector(1,0),
            'dino_alone_blob':masked, 'siglip_blob':semantic, 'vectors_from':backend}


class Store:
    def __init__(self, vectors):
        self.vectors = vectors

    def exemplars(self, entity, width=0, alone=False):
        return self.vectors

    def add_exemplar(self, *args, **kwargs):
        return 0


class EvidenceTests(unittest.TestCase):
    def test_existing_score_distinguishes_matching_from_orthogonal(self):
        matching = row(1, masked=vector(1,0), semantic=vector(1,0))
        opposite = row(2, plain=vector(0,1), masked=vector(0,1), semantic=vector(0,1))
        evidence = AppearanceEvidence([matching])
        store = Store([matching['dino_blob']])
        self.assertGreater(evidence.score(store,'a',matching),0)
        self.assertLess(evidence.score(store,'a',opposite),0)

    def test_backend_mismatch_and_absent_mask_abstain(self):
        kept = row(1, masked=vector(1,0), semantic=vector(1,0))
        evidence = AppearanceEvidence([kept]);store=Store([kept['dino_blob']])
        self.assertIsNone(evidence.score(store,'a',row(2,masked=vector(1,0),semantic=vector(1,0),backend='other')))
        self.assertIsNone(evidence.score(store,'a',row(3,semantic=vector(1,0))))

    def test_identical_plain_vector_with_conflicting_evidence_abstains(self):
        kept = row(1, masked=vector(1,0), semantic=vector(1,0))
        conflict = row(2, masked=vector(0,1), semantic=vector(0,1))
        evidence = AppearanceEvidence([kept,conflict])
        self.assertIsNone(evidence.score(Store([kept['dino_blob']]),'a',kept))

    def test_experiment_patch_is_restored_on_failure(self):
        original=resolve.collapsed
        store=Store([vector(1,0)]);add=store.add_exemplar
        with self.assertRaisesRegex(RuntimeError,'stop'):
            with experimental_gate(store,'evidence',AppearanceEvidence([])):
                self.assertIsNot(resolve.collapsed,original)
                raise RuntimeError('stop')
        self.assertIs(resolve.collapsed,original)
        self.assertEqual(store.add_exemplar,add)


if __name__=='__main__':
    unittest.main()
