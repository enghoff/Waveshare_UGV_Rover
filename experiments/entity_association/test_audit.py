"""Tests of the evaluation contract, rather than the rover's association rules."""
import importlib.util
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
import numpy as np

spec = importlib.util.spec_from_file_location("audit", Path(__file__).with_name("audit.py"))
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class AccountingTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        root = Path(self.directory.name)
        labels = {"object:246": {"main_looks": [1, 2], "odd": [3]},
                  "object:351": {"main_looks": [4, 5], "odd": []},
                  "other": {"main_looks": [6, 7], "odd": []}}
        (root / "labels.json").write_text(json.dumps({"things": labels}))
        con = sqlite3.connect(root / "world.db")
        con.execute("CREATE TABLE observations(id INTEGER,inference_id INTEGER,"
                    "map_session INTEGER,observed_at REAL)")
        con.executemany("INSERT INTO observations VALUES(?,?,67,?)",
                        [(i, 1 if i in (1, 4) else i, i + 1e9) for i in range(1, 8)])
        con.commit()
        con.close()
        self.e = audit.Evidence(root / "world.db", root / "labels.json")

    def test_duplicate_halves_are_held_out_together(self):
        self.assertEqual(self.e.fold_of[self.e.object_of["object:246"]],
                         self.e.fold_of[self.e.object_of["object:351"]])

    def test_feature_ablation_zeros_omitted_channels(self):
        for i,row in self.e.rows.items():
            row['vectors_from']='test'
            for key in ('dino_blob','dino_alone_blob','siglip_blob'):
                row[key]=np.asarray([1.,float(i)/10],dtype='<f4').tobytes()
        model=self.e.fit(None,columns=(1,))
        self.assertEqual(model['coef'][0],0.)
        self.assertEqual(model['coef'][2],0.)
        self.assertNotEqual(model['coef'][1],0.)
        self.assertEqual(model['held_out_objects'],[])
        self.assertEqual(self.e.fit(None),self.e.fit(None,columns=(0,1,2)))
        extra={i:np.array([0.,1.]) for i in self.e.rows}
        control=self.e.fit(None,columns=(0,1,2),additional=extra)
        np.testing.assert_allclose(control['coef'][:3],self.e.fit(None)['coef'])
        self.assertEqual(control['coef'][3],0.)
        self.assertEqual(control['training_pairs'],self.e.fit(None)['training_pairs'])

    def test_invalid_or_repeated_feature_columns_are_refused(self):
        for columns in ((),(3,),(1,1)):
            with self.assertRaises(ValueError):self.e.fit(None,columns=columns)

    def test_pairs_partition_exactly_once_across_folds(self):
        self.assertEqual(len(self.e.pairs), len({(p[0], p[1]) for p in self.e.pairs}))
        owners = {i: "same" for i in range(1, 8)}
        full, a, b = [self.e.score(owners, fold) for fold in (None, 0, 1)]
        for key in ("same_together", "same_apart", "different_together", "different_apart"):
            self.assertEqual(full[key], a[key] + b[key])

    def test_same_picture_does_not_override_known_identity(self):
        pair = next(p for p in self.e.pairs if p[:2] == (1, 4))
        self.assertTrue(pair[2])

    def test_unassigned_looks_are_not_a_shared_identity(self):
        result = self.e.score({i: None for i in range(1, 8)})
        self.assertEqual(result["same_together"], 0)
        self.assertEqual(result["different_together"], 0)
        self.assertEqual(result["main_waiting"], 6)

    def test_odd_identity_is_unknown_except_against_original_object(self):
        touching = [p for p in self.e.pairs if 3 in p[:2]]
        self.assertEqual({i for p in touching for i in p[:2]} - {3}, {1, 2})
        self.assertTrue(all(not p[2] for p in touching))

    def test_duplicate_split_and_missing_are_separate(self):
        result = self.e.score({1: "a", 2: "a", 3: None, 4: "b", 5: "b", 6: None, 7: None})
        self.assertEqual(result["objects_split"], 1)
        self.assertEqual(result["main_waiting"], 2)

    def test_exclusion_does_not_relabel_or_overwrite_original_file(self):
        root = Path(self.directory.name)
        before = (root / "labels.json").read_bytes()
        reviewed = audit.Evidence(root / "world.db", root / "labels.json", excluded=[4])
        self.assertFalse(any(4 in p[:2] for p in reviewed.pairs))
        self.assertEqual(reviewed.excluded, [4])
        self.assertEqual((root / "labels.json").read_bytes(), before)

    def test_part_whole_equivalence_preserves_other_folds(self):
        root = Path(self.directory.name)
        labels = json.loads((root / "labels.json").read_text())
        labels["things"]["object:332"] = {"main_looks": [1, 2], "odd": []}
        labels["things"]["object:385"] = {"main_looks": [4, 5], "odd": []}
        del labels["things"]["object:246"]
        del labels["things"]["object:351"]
        (root / "labels.json").write_text(json.dumps(labels))
        separate = audit.Evidence(root / "world.db", root / "labels.json")
        together = audit.Evidence(root / "world.db", root / "labels.json", same_person=True)
        self.assertEqual(together.object_of["object:332"],together.object_of["object:385"])
        self.assertEqual(together.fold_of["other"],separate.fold_of["other"])
        self.assertEqual(together.fold_of["object:332"],separate.fold_of["object:332"])
        self.assertTrue(next(p for p in together.pairs if p[:2]==(1,4))[2])


if __name__ == "__main__":
    unittest.main()
