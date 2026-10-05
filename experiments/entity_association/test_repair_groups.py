import unittest
from types import SimpleNamespace

import numpy as np

from experiments.entity_association.repair_groups import cluster, tiles, verify_split_only


def look(i,picture,backend='a'):
    v=np.array([1.,0.])
    return SimpleNamespace(id=i,inference=picture,entity='old',v=v,a=v,g=v,ray=None,
                           row={'vectors_from':backend})


class RepairContracts(unittest.TestCase):
    def test_split_contract_allows_pieces_and_release(self):
        self.assertEqual(verify_split_only({1:'a',2:'a',3:'b',4:None},
                                          {1:'p',2:'q',3:None,4:None}),{'p':'a','q':'a'})

    def test_split_contract_rejects_merges_pending_assignment_and_missing_rows(self):
        before={1:'a',2:'b',3:None}
        for after in ({1:'p',2:'p',3:None},{1:'p',2:'q',3:'r'},{1:'p',2:'q'}):
            with self.assertRaises(ValueError):verify_split_only(before,after)

    def test_cannot_link_survives_a_transitive_join(self):
        pool=[look(1,1),look(2,2),look(3,1)]
        clusters,released,_=cluster(pool,{}, {'coef':[1.,1.,1.],'offset':0},veto=False)
        self.assertEqual(sorted([p.id for g in clusters for p in g]+[p.id for p in released]),[1,2,3])
        for group in clusters:self.assertFalse({1,3}<={p.id for p in group})

    def test_different_backends_never_join(self):
        clusters,released,_=cluster([look(1,1,'a'),look(2,2,'b')],{},
                                   {'coef':[1.,1.,1.],'offset':0},veto=False)
        self.assertFalse(clusters);self.assertEqual(len(released),2)

    def test_no_geometry_fit_does_not_approve_a_union(self):
        clusters,released,stats=cluster([look(1,1),look(2,2)],{},
                                       {'coef':[1.,1.,1.],'offset':0})
        self.assertFalse(clusters);self.assertEqual(len(released),2)
        self.assertGreater(stats['no_fit_veto'],0)

    def test_tiles_are_disjoint_and_obey_bound_when_records_fit(self):
        placed={name:{'x_m':x,'y_m':0.,'uncertainty_m':.1} for name,x in [('a',0),('b',.1),('c',.2)]}
        counts={'a':300,'b':300,'c':10}
        result=tiles(placed,counts)
        self.assertEqual(sorted(k for group in result for k in group),['a','b','c'])
        self.assertTrue(all(sum(counts[k] for k in group)<=512 for group in result))
        self.assertEqual(result,tiles(dict(reversed(list(placed.items()))),counts))


if __name__=='__main__':unittest.main()
