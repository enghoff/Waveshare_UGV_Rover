import json
from pathlib import Path
import tempfile
import unittest

from experiments.entity_association.release_position import auc, judged_by_labels, score


class ReleasePositionTest(unittest.TestCase):
    def test_gate_counts_kept_wrong_and_released_right(self):
        measured={1:{'statistic':5.0},2:{'statistic':1.0},3:{'statistic':0.5},4:{'statistic':None},5:{'statistic':9.0}}
        judged={1:'right',2:'right',3:'wrong',4:'wrong',5:'unclear'}
        got=score(measured,judged)
        self.assertEqual((got['right'],got['wrong']),(2,1))
        self.assertEqual(got['unmeasured'],{'wrong':1})
        self.assertEqual((got['right_still_released'],got['wrong_kept']),(1,1))
        self.assertFalse(got['passes'])

    def test_auc_ranks_right_above_wrong(self):
        self.assertEqual(auc([3,4],[1,2]),1.0)
        self.assertEqual(auc([1],[1]),.5)

    def test_labels_judge_against_the_largest_kept_cluster(self):
        labels={'things':{'object:1':{'main_looks':[10,11,12,20],'odd':[21],'unsure':[]},
                          'object:2':{'main_looks':[22],'odd':[],'unsure':[]}}}
        proposal={'released':[20,21,22,23,61656],
                  'before_owners':{str(i):'object:9' for i in (10,11,12,20,21,22,23,61656)},
                  'tiles':[{'clusters':[[10,11,12]]}]}
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'labels.json';path.write_text(json.dumps(labels))
            got=judged_by_labels(path,proposal)
        self.assertEqual(got,{20:'wrong',21:'right',22:'right'})


if __name__=='__main__':
    unittest.main()
