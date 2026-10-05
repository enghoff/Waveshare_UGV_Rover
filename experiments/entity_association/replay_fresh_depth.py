"""One timestamp-derived October 5 schedule; reject a non-reproducing control."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline, replay, resolve
from world_state.store import WorldStore
from experiments.entity_association.audit_depth_abstention import load_depth, trace, json_numpy
from experiments.entity_association.diagnose_visual_evidence import digest


def read(path, query):
    with sqlite3.connect(path.resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        return [dict(r) for r in db.execute(query)]


def state(path):
    return {'owners': {r['id']: r['entity_id'] for r in read(path, 'SELECT id,entity_id FROM observations')},
            'entities': {r['id']: {'placement': json.loads(r['placement_json'] or 'null'),
                                  'plain': bytes(r['exemplars'] or b'').hex(),
                                  'masked': bytes(r['exemplars_alone'] or b'').hex()}
                         for r in read(path, 'SELECT * FROM entities')}}


def compare(actual, expected):
    return {'membership_differences': sorted(i for i in set(actual['owners'])|set(expected['owners'])
                                             if i not in actual['owners'] or i not in expected['owners']
                                             or actual['owners'][i] != expected['owners'][i]),
            'entity_set_difference': sorted(set(actual['entities'])^set(expected['entities'])),
            'placement_differences': sorted(e for e in set(actual['entities'])&set(expected['entities'])
                if actual['entities'][e]['placement'] != expected['entities'][e]['placement']),
            'exemplar_differences': sorted(e for e in set(actual['entities'])&set(expected['entities'])
                if any(actual['entities'][e][k] != expected['entities'][e][k] for k in ['plain','masked']))}


def run(directory, output, only_observation=None):
    import numpy as np
    directory = directory.resolve()
    if output.exists():
        raise ValueError('choose a new output directory')
    output.mkdir(parents=True)
    before, after = directory/'before.db', directory/'after.db'
    old = read(before, 'SELECT * FROM observations')
    cutoff = max(r['id'] for r in old)
    rows = read(after, f'SELECT * FROM observations WHERE id>{cutoff} ORDER BY id')
    inferences = {r['id']: r for r in read(after, 'SELECT * FROM inferences')}
    groups = {}
    for r in rows:
        groups.setdefault(r['inference_id'], []).append(r)
    groups = list(groups.values())
    old_updates = {r['id']: r['placement_updated_at'] for r in read(before, 'SELECT * FROM entities')}
    times = sorted(r['placement_updated_at'] for r in read(after, 'SELECT * FROM entities')
                   if r['placement_updated_at'] and r['placement_updated_at'] != old_updates.get(r['id']))
    passes = []
    for t in times:
        if not passes or t-passes[-1][-1] > 2:
            passes.append([])
        passes[-1].append(t)
    schedule = []
    for event in passes:
        eligible = [n for n,g in enumerate(groups,1) if
                    inferences[g[0]['inference_id']]['started_at']+
                    inferences[g[0]['inference_id']]['duration_s'] <= event[0]]
        schedule.append({'after_frame': max(eligible, default=0), 'first_update_at': event[0],
                         'last_update_at': event[-1], 'surviving_updates': len(event)})
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in
              [before, after, directory/'nav_grid.json', directory/'physical-subjects-frozen.json']}
    records, flags = [], []
    # Validate all archived stored ranges without re-perception or vector changes.
    for g in groups:
        for r in g:
            depth = directory/'frames'/(r['frame_id']+'.depth.gz')
            if r.get('range_m') is None or not depth.exists():
                continue
            photo = directory/'frames'/(r['frame_id']+'.jpg')
            from PIL import Image
            size = Image.open(photo).size
            for p in [depth, photo]:
                hashes[str(p.relative_to(ROOT))] = digest(p)
            image, raw, _, _ = load_depth(depth)
            bbox = json.loads(r['bbox_json']); blob = r.get('outline_blob')
            measurement = outline.read(np, raw, image.width, image.height, image.lens, [(bbox, blob)], size)[0]
            exact = measurement.get('range_m') == r['range_m']
            traced = trace(image, blob, size, outline.box_range(image, bbox, size).get('range_m')) if measurement.get('method') == 'outline' else None
            if traced:
                assert traced['range_m'] == measurement['range_m']
            records.append({'id': r['id'], 'stored_m': r['range_m'], 'measurement': measurement,
                            'exact': exact, 'trace': traced})
            if exact and traced and traced['abstain']:
                flags.append(r['id'])
    report = {'cutoff': cutoff, 'schedule': schedule, 'range_records': records, 'flagged_ids': flags,
              'input_sha256': hashes, 'limits': ['Surviving placement timestamps omit overwritten/no-change calls.',
              'Initial occupancy grid is a frozen sensitivity; concurrent capture boundaries unlogged.',
              'Analyst physical-subject labels, no independent acceptance or fresh tape truth.']}
    reach = replay.reach_from(str(directory/'nav_grid.json'))
    def replay_arm(arm, withheld):
        with tempfile.TemporaryDirectory(prefix='ugv-fresh-depth-') as tmp:
            store = WorldStore(tmp)
            try:
                with sqlite3.connect(before.resolve().as_uri()+'?mode=ro', uri=True) as db:
                    db.backup(store.db)
                store._create()
                events = []
                for n in range(len(groups)+1):
                    if n:
                        with store.db:
                            for original in groups[n-1]:
                                r = dict(original)
                                if r['id'] in withheld:
                                    r['range_m'] = r['range_sigma_m'] = None
                                    r['range_absent'] = 'offline fixed minority-surface abstention'
                                columns = ['id', *replay.COLUMNS]
                                store.db.execute('INSERT INTO observations ('+','.join(columns)+') VALUES ('+
                                                 ','.join('?' for _ in columns)+')', tuple(r.get(k) for k in columns))
                    for event in schedule:
                        if event['after_frame'] == n:
                            outcome = resolve.resolve(store, reach=reach)
                            events.append({'after_frame': n, 'outcome': outcome})
                            print(arm, 'resolve after frame', n, flush=True)
                with sqlite3.connect(output/(arm+'.db')) as dst:
                    store.db.backup(dst)
            finally:
                store.close()
        actual = state(output/(arm+'.db'))
        (output/(arm+'-state.json')).write_text(json.dumps(actual, indent=2)+'\n')
        return actual, events
    expected = state(after)
    actual, events = replay_arm('control', set())
    repeated, _ = replay_arm('control-repeat', set())
    assert actual == repeated, 'control is not deterministic'
    differences = compare(actual, expected)
    exact = not any(differences.values())
    report.update({'control_repeat_exact': True, 'live_control_exact': exact,
                   'control_differences': differences, 'events': events})
    range_exact = all(r['exact'] for r in records)
    report['stored_range_reproduction_exact'] = range_exact
    if exact and range_exact:
        if only_observation is not None:
            assert only_observation in flags, 'requested observation does not satisfy fixed depth flag'
        withheld = flags if only_observation is None else [only_observation]
        report['withheld_ids'] = withheld
        candidate, events = replay_arm('abstain', set(withheld))
        labels = json.loads((directory/'physical-subjects-frozen.json').read_text())['subjects']
        from itertools import combinations
        subjects = {i: t for t, ids in labels.items() for i in ids}
        def pairs(s):
            same, wrong = set(), set()
            for a,b in combinations(sorted(subjects),2):
                if s['owners'].get(a) and s['owners'].get(a)==s['owners'].get(b):
                    (same if subjects[a]==subjects[b] else wrong).add((a,b))
            return same, wrong
        a,w = pairs(actual); b,v = pairs(candidate)
        report['candidate'] = {'events': events, 'same_pairs_control': sorted(a),
            'same_pairs_candidate': sorted(b), 'lost_same_pairs': sorted(a-b),
            'new_same_pairs': sorted(b-a), 'new_cross_subject_pairs': sorted(v-w),
            'same_pair_retention': len(a&b)/len(a) if a else None,
            'known_painting_control_owner': actual['owners'].get(68640),
            'known_painting_candidate_owner': candidate['owners'].get(68640),
            'state_differences': compare(candidate, actual)}
    else:
        report['candidate_skipped'] = 'control failed exact live state or stored-range reproduction; no policy score'
    report['source_sha256'] = {str(p.relative_to(ROOT)): digest(p) for directory_name in
        ['world_state', 'experiments/entity_association'] for p in (ROOT/directory_name).glob('*.py')}
    for name, sha in hashes.items():
        assert digest(ROOT/name) == sha, name
    (output/'result.json').write_text(json.dumps(report, indent=2, default=json_numpy)+'\n')
    print('live control exact:', exact, 'differences:', {k:len(v) for k,v in differences.items()}, flush=True)


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--only-observation', type=int, help='Post-score causal diagnostic: withhold one qualifying range only.')
    a = p.parse_args(); run(a.directory, a.output, a.only_observation)
