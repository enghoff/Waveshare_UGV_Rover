from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest

import numpy as np

from experiments.entity_association.release_category import KINDS, measure


def blob(kind,mix=0.0):
    v=np.zeros(len(KINDS),'<f4');v[kind]=1.0
    if mix:v[(kind+1)%len(KINDS)]=mix
    return v.tobytes()


class ReleaseCategoryTest(unittest.TestCase):
    def test_kind_match_scores_zero_and_mismatch_scores_the_margin(self):
        with tempfile.TemporaryDirectory() as tmp:
            db=Path(tmp)/'w.db'
            with closing(sqlite3.connect(db)) as con:
                con.execute('CREATE TABLE observations (id INTEGER, siglip_blob BLOB)')
                con.executemany('INSERT INTO observations VALUES (?,?)',
                                [(1,blob(3)),(2,blob(3)),(3,blob(4)),(10,blob(3,.5)),(11,blob(0)),(12,None)])
                con.commit()
            proposal={'database_sha256':hashlib.sha256(db.read_bytes()).hexdigest(),'released':[10,11,12],
                      'before_owners':{str(i):'object:1' for i in (1,2,3,10,11,12)},
                      'tiles':[{'clusters':[[1,2,3]]}]}
            path=Path(tmp)/'p.json';path.write_text(json.dumps(proposal))
            _,got=measure(path,db,np.eye(len(KINDS)))
        self.assertEqual(got[10]['record_kind'],KINDS[3])
        self.assertEqual(got[10]['statistic'],0.0)
        self.assertGreater(got[11]['statistic'],0.9)
        self.assertIsNone(got[12]['kind'])
        self.assertNotIn('statistic',got[12])


if __name__=='__main__':
    unittest.main()
