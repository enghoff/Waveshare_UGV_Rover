"""Replay a later recording from its before-snapshot, with decision diagnostics.

The optional occupancy map is a frozen sensitivity, not a claim that the live map
was unchanged. Only disposable stores are written. No training takes place.
"""
from __future__ import annotations

import argparse
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import time
from contextlib import contextmanager

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import appearance, merging, replay, resolve
from world_state.store import EXEMPLARS, WorldStore


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


class AppearanceEvidence:
    """Existing merge weights applied to pairs with kept exemplar observations.

    This is an experiment, not a calibrated probability or a production API.
    Missing/ambiguous provenance abstains rather than comparing different backends.
    """
    def __init__(self, rows):
        self.by_plain = {}
        for row in rows:
            if row.get('dino_blob'):
                self.by_plain.setdefault(row['dino_blob'], []).append(row)
        self.cache = {}

    def score(self, store, entity_id, observation):
        vectors = tuple(store.exemplars(entity_id, width=len(observation.get('dino_blob') or b'')))
        key = (vectors, observation['id'])
        if key in self.cache:
            return self.cache[key]
        scores = []
        for vector in vectors:
            references = self.by_plain.get(vector, [])
            # No arbitrary selection if identical plain vectors have distinct
            # semantic/masked evidence or backend provenance.
            evidence = {(r.get('dino_alone_blob'), r.get('siglip_blob'), r.get('vectors_from'))
                        for r in references}
            if len(evidence) != 1:
                continue
            ref = references[0]
            if not ref.get('vectors_from') or ref['vectors_from'] != observation.get('vectors_from'):
                continue
            keys = ('dino_blob', 'dino_alone_blob', 'siglip_blob')
            if any(not ref.get(k) or not observation.get(k) or
                   len(ref[k]) != len(observation[k]) for k in keys):
                continue
            features = [appearance.similarity(ref[k], observation[k]) for k in keys]
            scores.append(sum(w*x for w,x in zip(merging.APPEARANCE_WEIGHTS, features))+
                          merging.APPEARANCE_OFFSET)
        result = sum(scores)/len(scores) if scores else None
        self.cache[key] = result
        return result


@contextmanager
def experimental_gate(store, candidate, evidence):
    original_collapsed = resolve.collapsed
    original_add = store.add_exemplar
    gate_stats = {'checks': 0, 'refusals': 0, 'missing': 0, 'updates_skipped': 0}

    def gate(active, entity_id, observation, seen):
        fell = original_collapsed(active, entity_id, observation, seen)
        if seen is None:
            return fell
        if candidate == 'evidence':
            value = evidence.score(active, entity_id, observation)
            threshold = merging.LOOKS_ALIKE_ABOVE
        elif candidate == 'masked-floor':
            value = appearance.alone(active, entity_id, observation.get('dino_alone_blob') or b'')
            threshold = resolve.DIFFERENT_THING
        else:
            return fell
        gate_stats['checks'] += 1
        if value is None:
            gate_stats['missing'] += 1
        elif (value <= threshold if candidate == 'evidence' else value < threshold):
            gate_stats['refusals'] += 1
            return resolve.COLLAPSED_ALONE
        return fell

    def trusted_add(entity_id, vector, keep=EXEMPLARS, alone=b''):
        seen = resolve.appearance(store, entity_id, vector)
        if seen is not None and seen < resolve.RECOGNISED:
            gate_stats['updates_skipped'] += 1
            return len(store.exemplars(entity_id, width=len(vector)))
        return original_add(entity_id, vector, keep=keep, alone=alone)

    if candidate in ('evidence', 'masked-floor'):
        resolve.collapsed = gate
    if candidate == 'trusted-updates':
        store.add_exemplar = trusted_add
    try:
        yield gate_stats
    finally:
        resolve.collapsed = original_collapsed
        store.add_exemplar = original_add


def diagnose(store, observation, entities, reach):
    ray = resolve.ray_of(observation, reach)
    candidates = []
    for entity in entities:
        point = entity.get('placement') or {}
        seen = resolve.appearance(store, entity['id'], observation.get('dino_blob') or b'')
        masked = appearance.alone(store, entity['id'], observation.get('dino_alone_blob') or b'')
        used = None if ray is None else resolve._allowance_used(point, ray)
        collapsed = resolve.collapsed(store, entity['id'], observation, seen)
        candidates.append({'entity': entity['id'], 'used': used, 'plain': seen,
                           'masked': masked, 'masked_drop': None if seen is None or masked is None else round(seen-masked,3),
                           'effective_drop_gate_value': collapsed,
                           'in_frame': entity['id'] in store.entities_in_frame(observation['inference_id']),
                           'placement': point, 'observation_count': entity['observation_count'],
                           'admissible': used is not None and (seen is None or seen >= resolve.DIFFERENT_THING)
                           and (collapsed is None or collapsed < resolve.COLLAPSED_ALONE)})
    return {'observation': observation['id'], 'ray': ray,
            'candidates': sorted(candidates, key=lambda x: -(x['plain'] or 0))}


def run(before, after, output, target=64638, map_path=None, stop_at_target=False,
        recorded_schedule=False, final_settle=False, candidate='none', labels_path=None,
        background_after=()):
    if output.exists() and any(output.iterdir()):
        raise ValueError('choose a new output directory to preserve previous results')
    output.mkdir(parents=True, exist_ok=True)
    if map_path and not map_path.is_file():
        raise ValueError('supplied occupancy map is missing')
    hashes = {'before': sha(before), 'after': sha(after)}
    reach = replay.reach_from(str(map_path)) if map_path else None
    with closing(sqlite3.connect(after.resolve().as_uri()+'?mode=ro', uri=True)) as db:
        db.row_factory = sqlite3.Row
        with closing(sqlite3.connect(before.resolve().as_uri()+'?mode=ro', uri=True)) as old:
            cutoff = old.execute('SELECT MAX(id) FROM observations').fetchone()[0]
        rows = [dict(r) for r in db.execute('SELECT * FROM observations WHERE id>? ORDER BY id', (cutoff,))]
        inferences = {r['id']: dict(r) for r in db.execute('SELECT * FROM inferences')}
        evidence = AppearanceEvidence([dict(r) for r in db.execute(
            'SELECT id,dino_blob,dino_alone_blob,siglip_blob,vectors_from FROM observations')])
        expected_all = dict(db.execute('SELECT id,entity_id FROM observations'))
        expected_exemplars = {r['id']: (r['exemplars'],r['exemplars_alone'])
                              for r in db.execute('SELECT id,exemplars,exemplars_alone FROM entities')}
    groups = {}
    for row in rows:
        groups.setdefault(row['inference_id'], []).append(row)
    expected = {r['id']: r['entity_id'] for r in rows}
    traces, frames = [], []
    current_frame = [0]
    real_by_look = resolve._by_look

    def traced(store, group, entities, session, taken_in, reach=None):
        targets = [r for r in group if r['id'] == target]
        if targets:
            store.db.backup(snapshot_db)
            traces.extend(diagnose(store, r, entities, reach) for r in targets)
            traces[-1]['settle_after_frame'] = current_frame[0]
        result = real_by_look(store, group, entities, session, taken_in, reach)
        if targets:
            traces[-1]['decision'] = [vars(d) for d in result if d.observation_id == target]
        return result

    began = time.monotonic()
    gate_totals = {'checks': 0, 'refusals': 0, 'missing': 0, 'updates_skipped': 0}
    resolve._by_look = traced
    try:
        with tempfile.TemporaryDirectory(prefix='ugv-attachment-replay-') as tmp:
            store = WorldStore(tmp)
            snapshot_db = sqlite3.connect(output/'pre-target.db')
            try:
                with closing(sqlite3.connect(before.resolve().as_uri()+'?mode=ro', uri=True)) as old:
                    old.backup(store.db)
                store._create()
                for n, (inference, group) in enumerate(groups.items(), 1):
                    current_frame[0] = n
                    with store.db:
                        for row in group:
                            columns = ['id', *replay.COLUMNS]
                            store.db.execute('INSERT INTO observations ('+','.join(columns)+') VALUES ('+
                                             ','.join('?' for _ in columns)+')', tuple(row.get(k) for k in columns))
                    detail = inferences[inference].get('detail') or ''
                    do_settle = not recorded_schedule or 'identity not settled yet' not in detail
                    with experimental_gate(store, candidate, evidence) as stats:
                        result = resolve.resolve(store, reach=reach) if do_settle else {
                            'matched': 0, 'created': 0, 'skipped': True}
                    for key in gate_totals:
                        gate_totals[key] += stats[key]
                    frames.append({'frame': n, 'inference': inference, 'result': result})
                    if n in background_after:
                        with experimental_gate(store, candidate, evidence) as stats:
                            background = resolve.resolve(store, reach=reach)
                        frames.append({'frame': n, 'inference': None, 'background': True, 'result': background})
                        for key in gate_totals:
                            gate_totals[key] += stats[key]
                    print(f'frame {n}/{len(groups)} inference {inference}: '+
                          f'{result["matched"]} matched, {result["created"]} created, '+
                          f'{time.monotonic()-began:.1f}s', flush=True)
                    if stop_at_target and store.db.execute(
                            'SELECT entity_id FROM observations WHERE id=?', (target,)).fetchone() is not None:
                        owner = store.db.execute('SELECT entity_id FROM observations WHERE id=?', (target,)).fetchone()[0]
                        if owner is not None:
                            break
                if final_settle:
                    with experimental_gate(store, candidate, evidence) as stats:
                        frames.append({'frame': 'final', 'inference': None, 'result': resolve.resolve(store, reach=reach)})
                    for key in gate_totals:
                        gate_totals[key] += stats[key]
                owners = {r['id']: r['entity_id'] for r in store.db.execute(
                    'SELECT id,entity_id FROM observations WHERE id>?', (cutoff,))}
                differences = [{'id': i, 'actual': owners[i], 'recorded': expected[i]}
                               for i in owners if owners[i] != expected[i]]
                actual_all = dict(store.db.execute('SELECT id,entity_id FROM observations'))
                all_differences = sum(i not in actual_all or actual_all[i] != entity
                                      for i,entity in expected_all.items())
                all_differences += len(set(actual_all)-set(expected_all))
                actual_exemplars = {r['id']: (r['exemplars'],r['exemplars_alone']) for r in
                                    store.db.execute('SELECT id,exemplars,exemplars_alone FROM entities')}
                exemplar_differences = [i for i,value in expected_exemplars.items()
                                        if actual_exemplars.get(i) != value]
                with closing(sqlite3.connect(output/'end.db')) as destination:
                    store.db.backup(destination)
            finally:
                snapshot_db.close()
                store.close()
    finally:
        resolve._by_look = real_by_look
    assert {'before': sha(before), 'after': sha(after)} == hashes, 'source snapshot changed'
    result = {'hashes': hashes, 'map_path': str(map_path) if map_path else None,
              'map_sha256': sha(map_path) if map_path else None,
              'map_limitation': 'Live occupancy maps per inspection were not recorded; supplied map is frozen sensitivity only.',
              'source_unchanged': True, 'independent_acceptance': False,
              'training_or_tuning': False, 'target': target, 'cutoff': cutoff,
              'recorded_schedule': recorded_schedule, 'final_settle': final_settle,
              'background_after_frames': list(background_after),
              'candidate': candidate, 'gate_totals': gate_totals,
              'frozen_weights': list(merging.APPEARANCE_WEIGHTS), 'frozen_offset': merging.APPEARANCE_OFFSET,
              'target_recorded_owner': expected[target], 'target_replayed_owner': owners.get(target),
              'owners': owners, 'differences': differences, 'frames': frames, 'trace': traces,
              'all_recorded_observations': len(expected_all),
              'all_membership_differences': all_differences,
              'exemplar_differences': exemplar_differences,
              'seconds': time.monotonic()-began}
    if labels_path:
        from experiments.entity_association.assess_recording import score
        doc = json.loads(labels_path.read_text())
        assert doc['source_database_sha256'] == hashes['after']
        labels = [r for r in doc['labels'] if r['verdict']=='object' and r['physical_object']]
        result['labels_sha256'] = sha(labels_path)
        result['primary'] = score([r for r in labels if r['review_status']=='draft_clear'], owners)
        result['tentative'] = score(labels, owners)
    (output/'result.json').write_text(json.dumps(result, indent=2)+'\n')
    print('RESULT', len(owners), 'rows,', len(differences), 'differences; target', owners.get(target), flush=True)
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--before', type=Path, required=True)
    parser.add_argument('--after', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--target', type=int, default=64638)
    parser.add_argument('--map', type=Path)
    parser.add_argument('--stop-at-target', action='store_true')
    parser.add_argument('--recorded-schedule', action='store_true')
    parser.add_argument('--final-settle', action='store_true')
    parser.add_argument('--candidate', choices=('none','evidence','masked-floor','trusted-updates'), default='none')
    parser.add_argument('--labels', type=Path)
    parser.add_argument('--background-after', default='', help='Comma-separated frame numbers; inferred schedule, not a live log.')
    args = parser.parse_args()
    run(args.before, args.after, args.output, args.target, args.map, args.stop_at_target,
        args.recorded_schedule, args.final_settle, args.candidate, args.labels,
        tuple(int(n) for n in args.background_after.split(',') if n))


if __name__ == '__main__':
    main()
