"""Bounded development diagnostic: measured depth can extend mapped visibility."""
from __future__ import annotations
import argparse
import contextlib
import functools
import json
import math
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from experiments.entity_association.replay_call_recording import run
from experiments.entity_association.diagnose_visual_evidence import digest


_MATCHING = [False]


@contextlib.contextmanager
def matching_scope():
    from world_state import resolve
    originals={name:getattr(resolve,name) for name in ['_by_look','_against_known','_replace_placement']}
    def wrapped(function,enabled):
        @functools.wraps(function)
        def call(*args,**kwargs):
            previous=_MATCHING[0];_MATCHING[0]=enabled
            try:return function(*args,**kwargs)
            finally:_MATCHING[0]=previous
        return call
    for name,function in originals.items():
        setattr(resolve,name,wrapped(function,name!='_replace_placement'))
    try:yield
    finally:
        for name,function in originals.items():setattr(resolve,name,function)


def matching_visibility(ray):
    return measured_visibility(ray) if _MATCHING[0] else ray


def older_regression(source, output, known_only=False):
    from world_state import replay, resolve
    from experiments.entity_association.audit_depth_abstention import frozen_tape
    from experiments.entity_association.replay_depth_abstention import run_replay, score, compare
    assert not output.exists(), 'choose a new output directory'
    output.mkdir(parents=True)
    definitions, tape = frozen_tape()
    grid = ROOT/'captures/m0-2026-10-02/grid.json'
    reach = replay.reach_from(str(grid))
    report={'independent_acceptance':False,
        'candidate_scope':'existing-record matching only' if known_only else 'all resolver rays',
        'source_sha256':digest(Path(__file__)),
        'limits':['One pass per inspection, not recorded historical call order.',
                  'Historical target mappings are proxies, not independently reviewed region truth.',
                  'Original stored ranges retained; this does not validate distance truth.'],
        'input_sha256':{str(tape):digest(tape),str(grid):digest(grid)}, 'drives':{}}
    for name, config in definitions['DRIVES'].items():
        database=ROOT/('captures/'+config['db'].split('captures/')[1])
        report['input_sha256'][str(database)]=digest(database)
        groups=replay.inspections(str(database))
        labels={r['id']:t for group in groups for r in group
                for t,ids in config['targets'].items() if r['entity_id'] in ids}
        print(name,'original control',flush=True)
        control=run_replay(database,groups,reach)
        prior_path=source/(name.replace(' ','-')+'-original.json')
        prior=json.loads(prior_path.read_text())
        assert control['canonical']==prior['canonical']
        assert {str(k):v for k,v in control['owners'].items()}==prior['owners']
        report['input_sha256'][str(prior_path)]=digest(prior_path)
        original=resolve.ray_of
        def transformed(observation,reach=None):
            ray=original(observation,reach)
            transform=matching_visibility if known_only else measured_visibility
            return transform(ray) if ray is not None else None
        resolve.ray_of=transformed
        print(name,'measured visibility',flush=True)
        try:
            with matching_scope() if known_only else contextlib.nullcontext():
                candidate=run_replay(database,groups,reach)
        finally:
            resolve.ray_of=original
        scores={arm:score(result,labels,config,definitions)
                for arm,result in [('control',control),('candidate',candidate)]}
        report['drives'][name]={'control_exact':True,'scores':scores,
            'comparison':compare(scores['control'],scores['candidate'])}
        for arm,result in [('control',control),('candidate',candidate)]:
            (output/(name.replace(' ','-')+'-'+arm+'.json')).write_text(json.dumps(result,indent=2)+'\n')
        (output/'result.json').write_text(json.dumps(report,indent=2)+'\n')
    return report


def measured_visibility(ray):
    ray = dict(ray)
    distance, reach = ray.get('range_m'), ray.get('reach_m')
    if (distance is not None and reach is not None and
            math.isfinite(float(distance)) and float(distance)>0):
        ray['reach_m'] = max(float(reach), float(distance))
    return ray


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    inputs=p.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--directory',type=Path)
    inputs.add_argument('--older-source',type=Path)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--known-only',action='store_true',help='Extend visibility only during existing-record admission; keep discovery/refitting unchanged.')
    a=p.parse_args()
    if a.older_source:
        older_regression(a.older_source,a.output,a.known_only)
    else:
        with matching_scope() if a.known_only else contextlib.nullcontext():
            run(a.directory,a.output,map_invariant=True,
                candidate_ray_transform=matching_visibility if a.known_only else measured_visibility)
