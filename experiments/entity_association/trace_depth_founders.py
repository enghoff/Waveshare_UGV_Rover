"""Annotate the completed paired replay with actual founding calls, without re-perception."""
from __future__ import annotations
import argparse
import copy
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import replay
from experiments.entity_association.replay_depth_abstention import run_replay
from experiments.entity_association.audit_depth_abstention import frozen_tape
from experiments.entity_association.diagnose_visual_evidence import digest


def run(source, output):
    if output.exists():
        raise ValueError('choose a new output directory')
    output.mkdir(parents=True)
    doc = json.loads((source/'result.json').read_text())
    # Verify every archived input used for measurement. The trace wrapper is a new
    # version of the harness, so executable files have their own new hashes below.
    for name, sha in doc['input_sha256'].items():
        if name.startswith('captures/'):
            assert digest(ROOT/name) == sha, name
    definitions, _ = frozen_tape()
    reach = replay.reach_from(str(ROOT/'captures/m0-2026-10-02/grid.json'))
    summaries = {}
    for name, config in definitions['DRIVES'].items():
        database = ROOT/('captures/'+config['db'].split('captures/')[1])
        original = replay.inspections(str(database))
        control = copy.deepcopy(original)
        measured = {r['id']: r for r in doc['drives'][name]['range_records'] if 'control' in r}
        flags = set(doc['drives'][name]['flagged_ids'])
        for group in control:
            for row in group:
                if row['id'] in measured:
                    m = measured[row['id']]
                    assert row['range_m'] == m['original_m']
                    row['range_m'] = m['control'].get('range_m')
                    row['range_sigma_m'] = m['control'].get('sigma_m')
                    row['range_absent'] = None if row['range_m'] is not None else 'offline outline measurement absent'
        candidate = copy.deepcopy(control)
        for group in candidate:
            for row in group:
                if row['id'] in flags:
                    row['range_m'] = row['range_sigma_m'] = None
                    row['range_absent'] = 'offline fixed minority-surface abstention'
        tag = name.replace(' ', '-')
        results = {}
        for arm, groups in [('original', original), ('outline', control), ('abstain', candidate)]:
            print(name, arm, 'founding trace', flush=True)
            result = run_replay(database, groups, reach)
            prior = json.loads((source/(tag+'-'+arm+'.json')).read_text())
            assert result['canonical'] == prior['canonical'], (name, arm)
            assert {str(k): v for k,v in result['owners'].items()} == prior['owners'], (name, arm)
            (output/(tag+'-'+arm+'.json')).write_text(json.dumps(result, indent=2)+'\n')
            results[arm] = result
        summaries[name] = {'instrumented_states_exact': True,
            'outline_flagged_founders': {eid: f for eid,f in results['outline']['founders'].items()
                                        if flags.intersection(f['observation_ids'])},
            'candidate_flagged_founders': {eid: f for eid,f in results['abstain']['founders'].items()
                                          if flags.intersection(f['observation_ids'])},
            'founder_counts': {arm: len(r['founders']) for arm,r in results.items()}}
    report = {'drives': summaries, 'measurement_sha256': digest(source/'result.json'),
              'source_sha256': {str(p.relative_to(ROOT)): digest(p) for directory in
                               ['world_state', 'experiments/entity_association']
                               for p in (ROOT/directory).glob('*.py')}}
    (output/'result.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args(); run(a.source, a.output)
