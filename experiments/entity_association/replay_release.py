"""Disposable release/re-attachment sensitivity; never applies to a live store."""
import argparse
from collections import defaultdict
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import tempfile

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import replay,resolve
from world_state.incremental import IncrementalResolver
from world_state.store import WorldStore
from experiments.entity_association.assess_recording import score


def run(database,proposal,map_path,draft,mode,output,passes=3):
    if output.exists():raise ValueError('choose a new output path')
    digest=hashlib.sha256(database.read_bytes()).hexdigest()
    proposed=json.loads(proposal.read_text());labels=json.loads(draft.read_text())
    if not proposed.get('split_only') or proposed['database_sha256']!=digest or labels['source_database_sha256']!=digest:
        raise ValueError('need split-only proposals and labels for this source')
    groups=defaultdict(list)
    for i in proposed['released']:
        parent=proposed['before_owners'][str(i)]
        if parent is not None:groups[parent].append(i)
    clear=[r for r in labels['labels'] if r['verdict']=='object' and r['physical_object'] and r['review_status']=='draft_clear']
    tentative=[r for r in labels['labels'] if r['verdict']=='object' and r['physical_object']]
    reach=replay.reach_from(str(map_path));steps=[]
    with tempfile.TemporaryDirectory(prefix='ugv-release-replay-') as tmp:
        store=WorldStore(tmp)
        try:
            with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as source:
                source.backup(store.db)
            def checkpoint(step,result=None):
                owners={r['id']:r['entity_id'] for r in store.db.execute('SELECT id,entity_id FROM observations')}
                # At the original/baseline checkpoints these have not left yet;
                # this field means "currently in original", not a transition count.
                steps.append({'step':step,'result':result,'clear':score(clear,owners),'tentative':score(tentative,owners),
                    'released_owners':{i:owners[i] for ids in groups.values() for i in ids},
                    'returned_to_original':[i for parent,ids in groups.items() for i in ids if owners[i]==parent],
                    'target_64638':owners.get(64638)})
            checkpoint('original')
            if mode!='baseline':
                engine=IncrementalResolver(store);engine.session=store.map_session()
                for parent,ids in groups.items():engine._detach(parent,ids,'offline split singleton proposal')
                if mode=='guarded':
                    store.association_allowed=lambda eid,oid:(eid,oid) not in engine.rejected_pairs
                checkpoint('released')
            for number in range(1,passes+1):
                result=resolve.resolve(store,reach=reach)
                checkpoint(number,result)
                print(mode,'pass',number,'target',steps[-1]['target_64638'],'returned',len(steps[-1]['returned_to_original']),flush=True)
        finally:store.close()
    assert hashlib.sha256(database.read_bytes()).hexdigest()==digest
    answer={'database_sha256':digest,'proposal_sha256':hashlib.sha256(proposal.read_bytes()).hexdigest(),
            'draft_sha256':hashlib.sha256(draft.read_bytes()).hexdigest(),'map_sha256':hashlib.sha256(map_path.read_bytes()).hexdigest(),
            'mode':mode,'passes':passes,'steps':steps,'source_unchanged':True,'independent_acceptance':False,
            'limitations':[f'{passes} idle resolver passes, not a new-drive replay or a steady-state claim.',
                           'Uses the existing experimental detach path, rebuilding bounded exemplar history and placements.',
                           'The optional refusal set is temporary, not durable rejection memory.',
                           'Frozen occupancy map; no new perception or hardware validation.']}
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(answer,indent=2)+'\n')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['database','proposal','map','draft','output']:p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--mode',choices=['baseline','released','guarded'],required=True)
    a=p.parse_args();run(a.database,a.proposal,a.map,a.draft,a.mode,a.output)
