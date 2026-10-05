"""Check the fixed minority-depth rule on stored outlines and taped stationary looks."""
from __future__ import annotations
import argparse
import ast
import importlib.util
import json
import math
from pathlib import Path
import sqlite3
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline
from experiments.entity_association.audit_depth_abstention import digest, load_depth, stats, trace


def run(directory, output):
    directory = directory.resolve()
    if output.exists():
        raise ValueError('choose a new output file')
    spec = importlib.util.spec_from_file_location('recorded_tape_frame', directory/'frame.py')
    frame = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(frame)
    tree = ast.parse((directory/'score.py').read_text())
    targets = next(ast.literal_eval(node.value) for node in tree.body
                   if isinstance(node, ast.Assign)
                   and any(isinstance(t, ast.Name) and t.id == 'TARGETS' for t in node.targets))
    legs = [json.loads(line) for line in (directory/'legs.log').read_text().splitlines()]
    windows = [(f['still_from'], f['still_to']) for leg in legs for f in leg['facings']]
    database = directory/'world-2026-10-03-acceptance.db'
    inputs = [directory/n for n in ['frame.py', 'score.py', 'grid_now.json', 'legs.log']]
    inputs += [database, Path(__file__), ROOT/'experiments/entity_association/audit_depth_abstention.py',
               ROOT/'world_state/outline.py', ROOT/'world_state/oak.py', ROOT/'world_state/view.py']
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in inputs}
    rows, missing = [], []
    with sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        starts = {r['frame_id']: r['started_at'] for r in db.execute('SELECT * FROM inferences')}
        for target, entities in targets.items():
            for entity in entities:
                for row in db.execute('SELECT * FROM observations WHERE entity_id=?', (entity,)):
                    row = dict(row)
                    fid = row['frame_id']
                    frames = directory/'frames-2026-10-03-acceptance'
                    depth, photo = frames/(fid+'.depth.gz'), frames/(fid+'.jpg')
                    if not photo.exists() or not depth.exists() or not row['observer_pose_json']:
                        missing.append({'id': row['id'], 'target': target, 'frame_id': fid,
                                        'missing': 'photo, depth, or observer pose unavailable'})
                        continue
                    for p in [depth, photo]:
                        hashes[str(p.relative_to(ROOT))] = digest(p)
                    image, raw, header, inferred = load_depth(depth)
                    with Image.open(photo) as im:
                        size = im.size
                    bbox = json.loads(row['bbox_json'])
                    boxed = outline.box_range(image, bbox, size)
                    replay = outline.read(np, raw, image.width, image.height, image.lens,
                                          [(bbox, row['outline_blob'])], size)[0]
                    traced = (trace(image, row['outline_blob'], size, boxed.get('range_m'))
                              if replay.get('method') == 'outline' else None)
                    if replay.get('method') == 'outline':
                        assert traced and traced['range_m'] == replay['range_m']
                    pose = json.loads(row['observer_pose_json'])
                    heading = math.radians(pose['heading_deg'])
                    cx, cy = pose['x_m']+.02*math.cos(heading), pose['y_m']+.02*math.sin(heading)
                    tx, ty, tz = frame.TRUTH[target]
                    rx, ry = frame.W2M(*frame.RANGE_AT.get(target, (tx, ty)))
                    want = math.hypot(math.hypot(rx-cx, ry-cy), tz-.235 if tz is not None else 0)
                    at = starts.get(fid, row['observed_at'])
                    stationary = any(a <= at <= b for a, b in windows)
                    stored, reproduced = row['range_m'], replay.get('range_m')
                    delta = (float(reproduced)-stored
                             if reproduced is not None and stored is not None else None)
                    rows.append({'drive': '2026-10-03', 'id': row['id'], 'target': target,
                                 'frame_id': fid, 'stationary': stationary,
                                 'want_m': want, 'control_m': stored, 'replayed_m': reproduced,
                                 'range_delta_m': delta, 'stored_method': row['range_from'],
                                 'range_absent': row['range_absent'], 'trace': traced,
                                 'abstain': bool(stored is not None and traced and traced['abstain'])})
    stationary = [r for r in rows if r['stationary']]
    ranged = [r for r in stationary if r['control_m'] is not None]
    faithful = all(r['range_delta_m'] is not None and abs(r['range_delta_m']) <= .002 for r in ranged)
    result = {'stationary': stats(stationary), 'moving': stats([r for r in rows if not r['stationary']]),
              'stationary_ranges_reproduced_within_002m': faithful,
              'gross_error_removal_informative': stats(stationary)['control_gross_1m'] >= 2,
              'stationary_pass_retention': stats(stationary)['correct_retention'] is not None
                   and stats(stationary)['correct_retention'] >= .9,
              'by_target_stationary': {t: stats([r for r in stationary if r['target'] == t]) for t in targets},
              'rows': rows, 'missing': missing, 'input_sha256': hashes,
              'limits': ['Separately recorded same-room development data; not independent identity acceptance.',
                        'Zero-turn replay; moving cases reported separately and not used for stationary proof.',
                        'Stored missing ranges stay missing, even when a zero-turn replay could provide one.',
                        'The original taped-target mapping is kept unchanged.']}
    assert digest(database) == hashes[str(database.relative_to(ROOT))]
    output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ['rows', 'input_sha256', 'missing']}, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run(a.directory, a.output)
