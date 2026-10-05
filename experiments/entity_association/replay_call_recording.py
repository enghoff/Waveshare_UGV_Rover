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


def run(directory, output, withheld=()):
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
    def arm(name, drop):
        with tempfile.TemporaryDirectory(prefix='ugv-call-replay-') as tmp:
            store = WorldStore(tmp)
            try:
                with sqlite3.connect((directory/'before.db').resolve().as_uri()+'?mode=ro',uri=True) as before:
                    before.backup(store.db)
                store._create()
                inserted = store.db.execute('SELECT COALESCE(MAX(id),0) FROM observations').fetchone()[0]
                assert checkpoint(store)==events[0]['checkpoint']
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
                    if not drop and 'checkpoint' in event:
                        assert checkpoint(store)==event['checkpoint'], ('input checkpoint',event['sequence'],event['kind'])
                        checks+=1
                    if event['kind']=='resolve_begin':
                        queries=[];last=index+1
                        while events[last]['kind']=='reach':
                            queries.append(events[last]);last+=1
                        assert events[last]['kind']=='resolve_end'
                        count=[0]
                        maps={q['map_sha256'] for q in queries}
                        grid=None
                        if drop:
                            assert len(maps)<=1, 'map changed during pass; no frozen counterfactual'
                            key=next(iter(maps),None)
                            grid=replay.reach_from(str(directory/'maps'/(key+'.json'))) if key else None
                        def reach(x,y,bearing):
                            if drop:
                                return grid(x,y,bearing) if grid else None
                            assert count[0]<len(queries), 'unrecorded reach query'
                            q=queries[count[0]];count[0]+=1
                            assert all(abs(float(a)-float(b))<1e-8 for a,b in zip([x,y,bearing],q['arguments'])), ('reach arguments',q['sequence'])
                            return q['result']
                        actual=resolve.resolve(store,reach=reach if queries else None)
                        if not drop:
                            assert count[0]==len(queries), 'unused reach query'
                            assert actual==events[last]['result'], ('resolver outcome',events[last]['sequence'])
                            assert checkpoint(store)==events[last]['checkpoint'], ('output checkpoint',events[last]['sequence'])
                            checks+=1
                        passes.append({'sequence':event['sequence'],'outcome':actual,'maps':sorted(m for m in maps if m)})
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
    maps={}
    map_checks=0
    for event in events:
        if event['kind']!='reach':continue
        key=event['map_sha256']
        if key not in maps:
            maps[key]=replay.reach_from(str(directory/'maps'/(key+'.json'))) if key else None
        fn=maps[key];answer=fn(*event['arguments']) if fn else None
        assert answer==event['result'], ('map reach mismatch',event['sequence'],answer,event['result'])
        map_checks+=1
    result['archived_map_reach_checks']=map_checks
    result['all_archived_map_answers_exact']=True
    if withheld:
        candidate=arm('abstain',set(withheld));result['arms']['abstain']=candidate
        result['withheld_ids']=list(withheld)
        result['owner_changes']=[i for i,e in control['owners'].items() if candidate['owners'].get(i)!=e]
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    for file,sha in hashes.items():assert digest(directory/file)==sha
    print('Every live identity checkpoint and reach query reproduced:',len(control['resolver_passes']),'passes;',control['recorded_reach_calls'],'reach calls',flush=True)
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--withhold',default='')
    a=p.parse_args();run(a.directory,a.output,tuple(int(i) for i in a.withhold.split(',') if i))
