"""Explain frozen subject fragmentation while reproducing actual control calls."""
from __future__ import annotations
import argparse
import copy
import itertools
import json
from pathlib import Path
import sqlite3
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import resolve, locate, appearance
from experiments.entity_association.replay_call_recording import run as replay
from experiments.entity_association.diagnose_visual_evidence import digest


def run(directory, labels, output):
    subjects = json.loads(labels.read_text())['subjects']
    wanted = {i for ids in subjects.values() for i in ids}
    with sqlite3.connect((directory/'after.db').resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        rows = {r['id']: dict(r) for r in db.execute('SELECT * FROM observations')}
    relevant = {rows[i]['entity_id'] for i in wanted} - {None}
    traced, rays = [], {}
    originals = {k: getattr(resolve, k) for k in ['resolve', 'ray_of', '_by_look']}
    pass_number = [0]

    def ray_of(observation, reach=None):
        ray = originals['ray_of'](observation, reach)
        if observation['id'] in wanted and ray is not None:
            rays[observation['id']] = copy.deepcopy(ray)
        return ray

    def by_look(store, group, entities, session, taken_in, reach=None):
        targets = [o for o in group if o['id'] in wanted]
        snapshots = []
        if targets:
            already = set(store.entities_in_frame(group[0].get('inference_id')))
            for entity in entities:
                if entity['id'] in relevant:
                    eid = entity['id']
                    snapshots.append((copy.deepcopy(entity), eid in already,
                        store.exemplars(eid, width=len(targets[0].get('dino_blob') or b'')),
                        store.exemplars(eid, width=len(targets[0].get('dino_alone_blob') or b''), alone=True)))
        decisions = originals['_by_look'](store, group, entities, session, taken_in, reach)
        for o in targets:
            ray = rays.get(o['id'])
            candidates = []
            for entity, claimed, plain, masked in snapshots:
                point = entity.get('placement') or {}
                if ray is None or not point:
                    continue
                seen = appearance.between(plain, [o.get('dino_blob') or b''])
                alone = appearance.between(masked, [o.get('dino_alone_blob') or b''])
                tolerance = locate.match_tolerance(point, ray)
                used = resolve._allowance_used(point, ray)
                candidates.append({'entity_id':entity['id'], 'placement':point,
                    'already_in_frame':claimed, 'geometry_allowed':used is not None,
                    'allowance_used':used, 'cross_track_m':locate.cross_track_of(point['x_m'],point['y_m'],ray),
                    'tolerance_m':tolerance,
                    'beyond_reach':locate.beyond_reach(ray,(point['x_m'],point['y_m'])),
                    'height_agrees':locate.stands_as_high(point,ray),
                    'range_agrees':locate.stands_at_range(point,ray),
                    'appearance':seen, 'masked_appearance':alone,
                    'appearance_allowed':seen is None or seen>=resolve.DIFFERENT_THING,
                    'mask_drop':None if seen is None or alone is None else seen-alone})
            traced.append({'pass':pass_number[0], 'observation_id':o['id'], 'ray':ray,
                'candidates':candidates, 'decisions':[vars(d) for d in decisions if d.observation_id==o['id']]})
        return decisions

    def resolver(*args, **kwargs):
        pass_number[0] += 1
        return originals['resolve'](*args, **kwargs)

    resolve.ray_of, resolve._by_look, resolve.resolve = ray_of, by_look, resolver
    try:
        result = replay(directory, output)
    finally:
        for name, function in originals.items():
            setattr(resolve, name, function)
    pairs = {}
    for name, ids in subjects.items():
        pairs[name] = [{'a':a, 'b':b,
            'plain':appearance.similarity(rows[a]['dino_blob'],rows[b]['dino_blob']),
            'masked':appearance.similarity(rows[a]['dino_alone_blob'],rows[b]['dino_alone_blob']),
            'same_owner':rows[a]['entity_id'] is not None and rows[a]['entity_id']==rows[b]['entity_id']}
            for a,b in itertools.combinations(ids,2)]
    report = {'labels_sha256':digest(labels), 'control_exact':result['all_live_checkpoints_exact'],
        'input_sha256':result['input_sha256'], 'source_sha256':digest(Path(__file__)),
        'independent_acceptance':False, 'scope':'Pre-look candidate snapshots; recorded real rays; same-subject pair scores. No additional reach queries or matching intervention.',
        'traces':traced, 'subject_pairs':pairs}
    (output/'subject-trace.json').write_text(json.dumps(report,indent=2)+'\n')
    print('Exact instrumented control; subject traces:',len(traced), flush=True)
    return report


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--labels',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.directory,a.labels,a.output)
