"""An independent review pack must include pending regions without revealing assignments."""
import csv
import json
from pathlib import Path
import sqlite3
import tempfile
import unittest
from contextlib import closing

from PIL import Image
from prepare_review import prepare


class ReviewPackTest(unittest.TestCase):
    def test_cutoff_missing_frames_and_blinding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            database = root / 'source.db'
            with closing(sqlite3.connect(database)) as db:
                db.execute('CREATE TABLE observations(id INTEGER,frame_id TEXT,bbox_json TEXT,'
                           'bearing_deg REAL,entity_id TEXT)')
                db.executemany('INSERT INTO observations VALUES(?,?,?,?,?)', [
                    (1, 'old', '[0,0,1,1]', 0, 'object:SECRET'),
                    (2, 'new', '[0.1,0.1,0.4,0.4]', 0, 'object:SECRET'),
                    (3, 'new', '[0.5,0.5,0.9,0.9]', None, None),
                    (4, 'missing', '[0,0,1,1]', 0, 'object:SECRET')])
                db.commit()
            frames = root / 'frames'
            frames.mkdir()
            Image.new('RGB', (200,200), 'gray').save(frames/'new.jpg')
            output = root / 'review'
            prepare(output, database=database, frames_dir=frames, after_observation=1)
            with (output/'labels.csv').open(newline='') as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual([int(row['observation_id']) for row in rows], [2,3])
            self.assertTrue(all(not row['physical_object'] and not row['verdict'] for row in rows))
            manifest = json.loads((output/'manifest.json').read_text())
            self.assertEqual(manifest['excluded_missing_frames'], 1)
            self.assertEqual(manifest['regions_without_bearing'], 1)
            self.assertFalse(manifest['independent_acceptance'])
            self.assertFalse(manifest['development_pilot'])
            for name in ('review.html','labels.csv','manifest.json'):
                self.assertNotIn('object:SECRET', (output/name).read_text())


if __name__ == '__main__':
    unittest.main()
