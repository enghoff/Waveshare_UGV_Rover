"""Paired full-drive development replay; never claims an unlogged live schedule."""
from __future__ import annotations

import argparse
import contextlib
import copy
import io
import itertools
import json
import math
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline, perceive, replay
from world_state.store import WorldStore
from experiments.entity_association.audit_depth_abstention import (
    frozen_tape, iou, load_depth, trace, json_numpy)
from experiments.entity_association.diagnose_visual_evidence import digest


def run_replay(database, groups, reach):
    order = [r['id'] for g in groups for r in g]
    founders = {}
    real_create, real_attach = WorldStore.create_entity, WorldStore.attach
    born = set()
    def create(store, *args, **kwargs):
        eid = real_create(store, *args, **kwargs)
        born.add(eid)
        return eid
    def attach(store, eid, ids, why=''):
        if eid in born and eid not in founders:
            row = store.db.execute('SELECT placement_json FROM entities WHERE id=?', (eid,)).fetchone()
            founders[eid] = {'observation_ids': [order[i-1] for i in ids],
                             'why': why, 'initial_placement': json.loads(row[0] or 'null')}
        return real_attach(store, eid, ids, why)
    WorldStore.create_entity, WorldStore.attach = create, attach
    try:
        with contextlib.redirect_stdout(io.StringIO()):
            entities, observations = replay.replay(str(database), groups=copy.deepcopy(groups), reach=reach)
    finally:
        WorldStore.create_entity, WorldStore.attach = real_create, real_attach
    owners = {order[r['id']-1]: r['entity_id'] for r in observations}
    placements = {e['id']: json.loads(e['placement_json'] or 'null') for e in entities}
    # Compare memberships, not arbitrary entity IDs. Include waiting individually.
    members = {}
    for oid, eid in owners.items():
        if eid:
            members.setdefault(eid, []).append(oid)
    canonical = [{'members': sorted(ids), 'placement': placements[eid]}
                 for eid, ids in members.items()]
    canonical.sort(key=lambda r: r['members'])
    return {'owners': owners, 'placements': placements, 'canonical': canonical, 'founders': founders,
            'entities': len(entities), 'waiting': sum(e is None for e in owners.values())}


def score(result, labels, config, definitions):
    owners = result['owners']
    same, wrong = set(), set()
    for a, b in itertools.combinations(sorted(labels), 2):
        if owners.get(a) and owners.get(a) == owners.get(b):
            (same if labels[a] == labels[b] else wrong).add((a, b))
    transform = definitions['frame'](config['parks'])
    targets = {}
    for target in sorted(set(labels.values())):
        ids = [oid for oid in labels if labels[oid] == target]
        votes = {}
        for oid in ids:
            if owners.get(oid):
                votes[owners[oid]] = votes.get(owners[oid], 0)+1
        eid = max(votes, key=lambda e: (votes[e], e)) if votes else None
        p = result['placements'].get(eid)
        xy = transform(*definitions['TRUTH'][target][:2])
        error = math.hypot(p['x_m']-xy[0], p['y_m']-xy[1]) if p and p.get('x_m') is not None else None
        targets[target] = {'looks': len(ids), 'assigned': sum(votes.values()),
                           'fragments': len(votes), 'majority_members': votes.get(eid, 0),
                           'placement_error_m': error, 'placement': p}
    return {'same_pairs': sorted(same), 'cross_target_pairs': sorted(wrong), 'targets': targets,
            'entities': result['entities'], 'waiting': result['waiting']}


def compare(control, candidate):
    same = set(map(tuple, control['same_pairs']))
    kept = set(map(tuple, candidate['same_pairs']))
    wrong = set(map(tuple, candidate['cross_target_pairs']))-set(map(tuple, control['cross_target_pairs']))
    retention = len(same & kept)/len(same) if same else None
    assigned_ok = all(candidate['targets'][t]['assigned'] >= .9*r['assigned']
                      for t, r in control['targets'].items())
    errors, later, deltas = [], [], {}
    lost = []
    for t, r in control['targets'].items():
        a, b = r['placement_error_m'], candidate['targets'][t]['placement_error_m']
        if a is not None and b is None:
            lost.append(t)
        if a is not None and b is not None:
            errors.append(a); later.append(b); deltas[t] = b-a
    improvement = float(np.median(errors)-np.median(later)) if errors else None
    identity_pass = retention is not None and retention >= .98 and not wrong and assigned_ok
    placement_pass = (identity_pass and improvement is not None and improvement >= .1
                      and not lost and all(v <= .25 for v in deltas.values()))
    return {'same_pair_retention': retention, 'lost_same_pairs': sorted(same-kept),
            'new_same_pairs': sorted(kept-same), 'new_cross_target_pairs': sorted(wrong),
            'assigned_retention_pass': assigned_ok, 'identity_preservation_pass': identity_pass,
            'median_placement_improvement_m': improvement, 'target_error_delta_m': deltas,
            'lost_target_placements': lost, 'placement_benefit_pass': placement_pass}


def run(model, output):
    if output.exists():
        raise ValueError('choose a new output directory')
    output.mkdir(parents=True)
    import onnxruntime as ort
    options = ort.SessionOptions(); options.intra_op_num_threads = 2; options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model), sess_options=options, providers=['CPUExecutionProvider'])
    class Models:
        def regions(self, image):
            a = session.run(None, {'images': image})
            return a[0], a[1]
    finder = perceive.Perception(str(model.parent))
    finder._np, finder._cv2, finder._models = np, cv2, Models()
    definitions, tape = frozen_tape()
    grid = ROOT/'captures/m0-2026-10-02/grid.json'
    reach = replay.reach_from(str(grid))
    hashes = {str(tape.relative_to(ROOT)): digest(tape), str(grid.relative_to(ROOT)): digest(grid),
              'model': digest(model)}
    for directory in ['world_state', 'experiments/entity_association']:
        for p in (ROOT/directory).glob('*.py'):
            hashes[str(p.relative_to(ROOT))] = digest(p)
    report = {'drives': {}, 'input_sha256': hashes, 'limits': [
        'One resolve per inspection is a fixed reconstructed schedule, not recorded live calls.',
        'Original entity target mappings are proxy labels, not independent region-level truth.',
        'Same room development data; zero-turn reconstruction of old depth; no glass tape truth.',
        'Outline reranging is a separate control intervention; original missing ranges stay missing.']}
    for name, config in definitions['DRIVES'].items():
        database = ROOT/('captures/'+config['db'].split('captures/')[1])
        frames = database.parent/('frames-redo' if 'redo' in name else 'frames')
        hashes[str(database.relative_to(ROOT))] = digest(database)
        original = replay.inspections(str(database))
        labels = {r['id']: t for g in original for r in g
                  for t, ids in config['targets'].items() if r['entity_id'] in ids}
        groups = copy.deepcopy(original)
        records, flags = [], []
        # Per-frame perception is cached, and all ranged observations are considered.
        by_frame = {}
        for g in groups:
            for r in g:
                if r.get('range_m') is not None:
                    by_frame.setdefault(r['frame_id'], []).append(r)
        for number, (fid, rows) in enumerate(by_frame.items(), 1):
            photo, depth = frames/(fid+'.jpg'), frames/(fid+'.depth.gz')
            if not photo.exists() or not depth.exists():
                records.extend({'id': r['id'], 'unchanged': 'missing photo/depth'} for r in rows)
                continue
            hashes[str(photo.relative_to(ROOT))] = digest(photo)
            hashes[str(depth.relative_to(ROOT))] = digest(depth)
            image, raw, _, _ = load_depth(depth)
            rgb = cv2.imread(str(photo)); size = (rgb.shape[1], rgb.shape[0])
            boxes, _, masks, _ = finder._regions(rgb)
            for r in rows:
                bbox = json.loads(r['bbox_json'])
                best = max(range(len(boxes)), key=lambda j: iou(bbox, boxes[j])) if len(boxes) else None
                overlap = iou(bbox, boxes[best]) if best is not None else 0
                if overlap < .5 or masks is None:
                    records.append({'id': r['id'], 'unchanged': 'mask unavailable', 'overlap': overlap})
                    continue
                blob = outline.encode(np, masks.of(best), bbox)
                measured = outline.read(np, raw, image.width, image.height, image.lens, [(bbox, blob)], size)[0]
                traced = trace(image, blob, size, outline.box_range(image, bbox, size).get('range_m')) if measured.get('method') == 'outline' else None
                if measured.get('method') == 'outline':
                    assert traced and traced['range_m'] == measured['range_m'], (r['id'], traced, measured)
                rec = {'id': r['id'], 'frame_id': fid, 'original_m': r['range_m'],
                       'control': measured, 'trace': traced, 'overlap': overlap}
                records.append(rec)
                r['range_m'] = measured.get('range_m'); r['range_sigma_m'] = measured.get('sigma_m')
                r['range_absent'] = None if r['range_m'] is not None else 'offline outline measurement absent'
                if traced and traced['abstain'] and r['range_m'] is not None:
                    flags.append(r['id'])
            if number % 10 == 0:
                print(name, 'depth frames', number, '/', len(by_frame), flush=True)
        candidate = copy.deepcopy(groups)
        for g in candidate:
            for r in g:
                if r['id'] in flags:
                    r['range_m'] = r['range_sigma_m'] = None
                    r['range_absent'] = 'offline fixed minority-surface abstention'
        print(name, 'replay controls', flush=True)
        orig = run_replay(database, original, reach)
        repeat = run_replay(database, original, reach)
        assert orig['canonical'] == repeat['canonical'] and orig['owners'] == repeat['owners']
        control = run_replay(database, groups, reach)
        repeat = run_replay(database, groups, reach)
        assert control['canonical'] == repeat['canonical'] and control['owners'] == repeat['owners']
        changed = run_replay(database, candidate, reach)
        with sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True) as db:
            recorded_owners = dict(db.execute('select id,entity_id from observations'))
            recorded_placements = {eid: json.loads(p or 'null') for eid,p in db.execute('select id,placement_json from entities')}
        recorded = {'owners': recorded_owners, 'placements': recorded_placements,
                    'entities': len(recorded_placements), 'waiting': sum(v is None for v in recorded_owners.values())}
        memberships = sum(orig['owners'][oid] != eid for oid,eid in recorded_owners.items())
        # ID mismatch is a strict exact-state check, not a physical-identity score.
        exact_placements = orig['placements'] == recorded_placements
        scores = {n: score(r, labels, config, definitions) for n,r in
                  [('recorded', recorded), ('original', orig), ('outline', control), ('abstain', changed)]}
        result = {'observations': sum(map(len, original)), 'frames': len(original),
                  'range_records': records, 'flagged_ids': flags,
                  'original_control_repeat_exact': True, 'outline_control_repeat_exact': True,
                  'recorded_owner_id_differences': memberships, 'recorded_placements_exact': exact_placements,
                  'scores': scores, 'comparison': compare(scores['outline'], scores['abstain']),
                  'outline_vs_original': compare(scores['original'], scores['outline'])}
        report['drives'][name] = result
        for arm, data in [('original', orig), ('outline', control), ('abstain', changed)]:
            tag = name.replace(' ', '-')
            (output/(tag+'-'+arm+'.json')).write_text(json.dumps(data, indent=2, default=json_numpy)+'\n')
        (output/'result.json').write_text(json.dumps(report, indent=2, default=json_numpy)+'\n')
        print(name, 'completed', json.dumps(result['comparison'], default=json_numpy), flush=True)
    # Verify that the complete source recordings stayed unchanged.
    for config in definitions['DRIVES'].values():
        p = ROOT/('captures/'+config['db'].split('captures/')[1])
        assert digest(p) == hashes[str(p.relative_to(ROOT))]
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    run(args.model, args.output)
