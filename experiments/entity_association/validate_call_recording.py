"""Exercise inspector recording boundaries against the exact October 5 reproduction."""
from __future__ import annotations
import argparse
import base64
import json
from pathlib import Path
import sqlite3
import sys
import types
import zlib
ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import replay
from world_state.store import WorldStore
from world_state.inspector import Inspector
from world_state.call_recording import checkpoint, REQUEST
from experiments.entity_association.replay_call_recording import run as verify
from experiments.entity_association.replay_fresh_depth import state, compare


def run(output):
    if output.exists():raise ValueError('choose a new output directory')
    output.mkdir(parents=True)
    directory=ROOT/'captures/2026-10-05-evidence-drive'
    before,after=directory/'before.db',directory/'after.db'
    with sqlite3.connect(before.resolve().as_uri()+'?mode=ro',uri=True) as db:
        cutoff=db.execute('SELECT MAX(id) FROM observations').fetchone()[0]
    groups=[g for g in replay.inspections(str(after)) if g[0]['id']>cutoff]
    schedule=[1,2,3,6,8,8,9,12,13,14]
    reach=replay.reach_from(str(directory/'nav_grid.json'))
    outputs={}
    for active in [False,True]:
        arm='recorded' if active else 'disabled'
        root=output/arm;store=WorldStore(str(root))
        try:
            with sqlite3.connect(before.resolve().as_uri()+'?mode=ro',uri=True) as db:db.backup(store.db)
            store._create()
            inspector=Inspector(store,None,lambda:None,lambda:None,reach=reach)
            import numpy as np
            grid=json.loads((directory/'nav_grid.json').read_text())
            cells=np.frombuffer(zlib.decompress(base64.b64decode(grid['data'])),dtype=np.int8).reshape(grid['height'],grid['width'])
            inspector.recording_grid=lambda: (grid['width'],grid['height'],grid['resolution_m'],
                                             grid['origin_x_m'],grid['origin_y_m'],cells)
            if active:(root/REQUEST).write_text(json.dumps({'session':'validation'})+'\n')
            def measured(self,settle=True,fresh=False,keep_depth=False):
                group=self.validation_group
                with store.db:
                    for row in group:
                        columns=['id',*replay.COLUMNS]
                        store.db.execute('INSERT INTO observations ('+','.join(columns)+') VALUES ('+
                            ','.join('?' for _ in columns)+')',tuple(row.get(k) for k in columns))
                return {'ok':True,'inference_id':group[0]['inference_id'],'stored':len(group)}
            inspector._inspect=types.MethodType(measured,inspector)
            for n,g in enumerate(groups,1):
                inspector.validation_group=g
                assert inspector.inspect(settle=False)['ok']
                for _ in range(schedule.count(n)):assert inspector.settle()['ok']
            if active:
                (root/REQUEST).unlink()
                with inspector.not_looking(5) as idle:
                    assert idle;inspector._sync_call_recording()
            outputs[arm]=checkpoint(store)
            with sqlite3.connect(root/'end.db') as dst:store.db.backup(dst)
        finally:store.close()
        assert not any(compare(state(root/'end.db'),state(after)).values()),arm
    assert outputs['recorded']==outputs['disabled']
    verified=verify(output/'recorded/recordings/validation',output/'replayed',withheld=[68640])
    assert verified['owner_changes']==[68640]
    assert verified['arms']['abstain']['owners'][68640]=='object:330'
    result={'both_arms_live_final_state_exact':True,'recorder_does_not_change_identity_state':True,
            'checkpoint':outputs['recorded'],'replay':verified,
            'limit':'Measured observation inserts supplied from real recording; no simulated camera/perception truth claim.'}
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print('Recorder enabled/disabled reproduce all 7,982 live memberships, placements and exemplars exactly.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();run(a.output)
