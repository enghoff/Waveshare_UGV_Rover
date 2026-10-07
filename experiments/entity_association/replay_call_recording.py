"""Verify actual resolver checkpoints and reach queries from an optional recording."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import replay, resolve
from world_state.store import WorldStore
from world_state.call_recording import checkpoint
from experiments.entity_association.diagnose_visual_evidence import digest


def invariant_answer(contexts, arguments, sequence):
    answers = [fn(*arguments) if fn else None for fn in contexts]
    assert answers and all(answer == answers[0] for answer in answers), (
        'candidate map disagreement', sequence, arguments, answers)
    return answers[0]


def run(directory, output, withheld=(), map_invariant=False, candidate_ray_transform=None,
        prepare=None):
    """`prepare(store, reach)`, when given, changes a counterfactual arm's starting
    store after it is checked against the recording -- `replay_consolidated.py`
    merges records with it. The control arm is never prepared."""
    if output.exists():
        raise ValueError('choose a new output directory')
    manifest = json.loads((directory/'manifest.json').read_text())
    assert manifest['complete'] and not manifest.get('error'), 'incomplete/failed recorder'
    events = [json.loads(s) for s in (directory/'events.jsonl').read_text().splitlines()]
    assert [e['sequence'] for e in events] == list(range(1,len(events)+1))
    assert events[0]['kind']=='start' and events[-1]['kind']=='stop'
    with sqlite3.connect((directory/'after.db').resolve().as_uri()+'?mode=ro',uri=True) as db:
        db.row_factory = sqlite3.Row
        observations = {r['id']:dict(r) for r in db.execute('SELECT * FROM observations')}
    output.mkdir(parents=True)
    hashes = {p.name:digest(p) for p in [directory/'before.db',directory/'after.db',directory/'events.jsonl',directory/'manifest.json']}
    result = {'input_sha256':hashes, 'source_sha256':manifest['source_sha256'],
              'independent_acceptance':False, 'arms':{}}
    grid_functions={}
    prepared={}
    def grid_function(key):
        if key not in grid_functions:
            if key:
                path=directory/'maps'/(key+'.json')
                assert digest(path)==key, ('archived map digest',key)
                grid_functions[key]=replay.reach_from(str(path))
            else:
                grid_functions[key]=None
        return grid_functions[key]
    def arm(name, drop):
        counterfactual = name != 'control'
        with tempfile.TemporaryDirectory(prefix='ugv-call-replay-') as tmp:
            store = WorldStore(tmp)
            try:
                with sqlite3.connect((directory/'before.db').resolve().as_uri()+'?mode=ro',uri=True) as before:
                    before.backup(store.db)
                store._create()
                inserted = store.db.execute('SELECT COALESCE(MAX(id),0) FROM observations').fetchone()[0]
                assert checkpoint(store)==events[0]['checkpoint']
                if counterfactual and prepare is not None:
                    first=next((e['map_sha256'] for e in events if e['kind']=='reach'),None)
                    prepared[name]=prepare(store,grid_function(first))
                checks, reach_calls, passes = 0, 0, []
                index=1
                while index<len(events):
                    event=events[index]
                    maximum=event.get('max_observation_id',inserted)
                    with store.db:
                        for oid in sorted(i for i in observations if inserted<i<=maximum):
                            row=dict(observations[oid])
                            if oid in drop:
                                row['range_m']=row['range_sigma_m']=None
                                row['range_absent']='offline fixed minority-surface abstention'
                            columns=['id',*replay.COLUMNS]
                            store.db.execute('INSERT INTO observations ('+','.join(columns)+') VALUES ('+
                                ','.join('?' for _ in columns)+')',tuple(row.get(k) for k in columns))
                        inserted=maximum
                    if not counterfactual and 'checkpoint' in event:
                        assert checkpoint(store)==event['checkpoint'], ('input checkpoint',event['sequence'],event['kind'])
                        checks+=1
                    if event['kind']=='resolve_begin':
                        queries=[];last=index+1
                        while events[last]['kind']=='reach':
                            queries.append(events[last]);last+=1
                        assert events[last]['kind']=='resolve_end'
                        count=[0];reach_failures=[]
                        maps={q['map_sha256'] for q in queries}
                        grid=None;contexts=[]
                        if counterfactual:
                            if map_invariant:
                                contexts=[grid_function(key) for key in maps] or [None]
                            else:
                                assert len(maps)<=1, 'map changed during pass; no frozen counterfactual'
                                grid=grid_function(next(iter(maps),None))
                        def reach(x,y,bearing):
                            if counterfactual:
                                count[0]+=1
                                if map_invariant:
                                    try:
                                        return invariant_answer(contexts,[x,y,bearing],event['sequence'])
                                    except Exception as error:
                                        # ray_of intentionally catches map failures. Keep the
                                        # diagnostic failure outside that production boundary.
                                        reach_failures.append(error)
                                        raise
                                return grid(x,y,bearing) if grid else None
                            assert count[0]<len(queries), 'unrecorded reach query'
                            q=queries[count[0]];count[0]+=1
                            assert all(abs(float(a)-float(b))<1e-8 for a,b in zip([x,y,bearing],q['arguments'])), ('reach arguments',q['sequence'])
                            return q['result']
                        original_ray = resolve.ray_of
                        if counterfactual and candidate_ray_transform is not None:
                            def transformed_ray(observation, reach=None):
                                ray = original_ray(observation, reach)
                                return candidate_ray_transform(ray) if ray is not None else None
                            resolve.ray_of = transformed_ray
                        try:
                            actual=resolve.resolve(store,reach=reach if queries else None)
                        finally:
                            resolve.ray_of = original_ray
                        if reach_failures:
                            raise reach_failures[0]
                        if not counterfactual:
                            assert count[0]==len(queries), 'unused reach query'
                            assert actual==events[last]['result'], ('resolver outcome',events[last]['sequence'])
                            assert checkpoint(store)==events[last]['checkpoint'], ('output checkpoint',events[last]['sequence'])
                            checks+=1
                        passes.append({'sequence':event['sequence'],'outcome':actual,
                                       'maps':sorted(m for m in maps if m),
                                       'actual_reach_calls':count[0]})
                        reach_calls+=len(queries);index=last
                    index+=1
                with sqlite3.connect(output/(name+'.db')) as dest:
                    store.db.backup(dest)
                return {'checkpoint_checks':checks,'resolver_passes':passes,'recorded_reach_calls':reach_calls,
                        'owners':dict(store.db.execute('SELECT id,entity_id FROM observations')),
                        'final_checkpoint':checkpoint(store)}
            finally:
                store.close()
    control=arm('control',set());result['arms']['control']=control
    assert control['final_checkpoint']==events[-1]['checkpoint']
    result['all_live_checkpoints_exact']=True
    # Verify saved geometry even when no counterfactual is requested. Replaying
    # logged answers alone would not prove that the archived maps are usable.
    map_checks=0
    for event in events:
        if event['kind']!='reach':continue
        key=event['map_sha256']
        fn=grid_function(key);answer=fn(*event['arguments']) if fn else None
        assert answer==event['result'], ('map reach mismatch',event['sequence'],answer,event['result'])
        map_checks+=1
    result['archived_map_reach_checks']=map_checks
    result['all_archived_map_answers_exact']=True
    if withheld or candidate_ray_transform is not None or prepare is not None:
        arm_name=('ray_candidate' if candidate_ray_transform is not None
                  else 'consolidated' if prepare is not None else 'abstain')
        candidate=arm(arm_name,set(withheld));result['arms'][arm_name]=candidate
        result['candidate_map_handling']=('every candidate query invariant under all recorded pass grids'
                                         if map_invariant else 'single recorded grid per pass')
        result['withheld_ids']=list(withheld)
        result['owner_changes']=[i for i,e in control['owners'].items() if candidate['owners'].get(i)!=e]
        if prepare is not None:
            result['prepared']=prepared.get(arm_name)
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    for file,sha in hashes.items():assert digest(directory/file)==sha
    print('Every live identity checkpoint and reach query reproduced:',len(control['resolver_passes']),'passes;',control['recorded_reach_calls'],'reach calls',flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--withhold',default='')
    p.add_argument('--map-invariant',action='store_true',
                   help='Conditional diagnostic: reject any candidate query whose answer differs across recorded pass grids.')
    a=p.parse_args();run(a.directory,a.output,tuple(int(i) for i in a.withhold.split(',') if i),a.map_invariant)
