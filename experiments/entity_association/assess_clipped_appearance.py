"""Pair-ranking diagnostic for a full frozen draft; no fitting or threshold choice."""
import argparse
import hashlib
from itertools import combinations
import json
from pathlib import Path

import numpy as np
from sklearn.metrics import roc_auc_score


def assess(probe,draft):
    provenance=json.loads((probe/'summary.json').read_text())
    document=json.loads(draft.read_text())
    if document['source_database_sha256']!=provenance['database_sha256']:
        raise ValueError('draft and features describe different snapshots')
    with np.load(probe/'vectors.npz',allow_pickle=False) as saved:
        values={k:saved[k] for k in saved.files}
    ids=[int(i) for i in values.pop('ids')]
    labels={int(r['observation_id']):r for r in document['labels']}
    if len(set(ids))!=len(ids) or len(labels)!=len(document['labels']) or set(ids)!=set(labels):
        raise ValueError('need exactly one feature row per frozen draft observation')
    recorded={int(r['id']):r for r in provenance['observations']}
    if set(recorded)!=set(ids) or any(recorded[i]['frame_id']!=labels[i]['frame_id'] for i in ids):
        raise ValueError('frame provenance differs')
    if any(len(v)!=len(ids) or v.ndim!=2 or not np.isfinite(v).all() for v in values.values()):
        raise ValueError('feature rows must be finite and match IDs')
    matrices={k:v@v.T for k,v in values.items()}
    index={i:n for n,i in enumerate(ids)}
    result={}
    for scope in ('clear','tentative'):
        usable=[r for r in document['labels'] if r['verdict']=='object' and r['physical_object']
                and (scope=='tentative' or r['review_status']=='draft_clear')]
        pairs=[(index[int(a['observation_id'])],index[int(b['observation_id'])],
                a['physical_object']==b['physical_object']) for a,b in combinations(usable,2)]
        truth=[same for _,_,same in pairs]
        result[scope]={'regions':len(usable),'pairs':len(pairs),'same_pairs':sum(truth),
            'auc':{k:float(roc_auc_score(truth,[matrix[i,j] for i,j,_ in pairs])) if len(set(truth))==2 else None
                   for k,matrix in matrices.items()}}
    return {'source_database_sha256':provenance['database_sha256'],
            'draft_sha256':hashlib.sha256(draft.read_bytes()).hexdigest(),
            'vectors_sha256':hashlib.sha256((probe/'vectors.npz').read_bytes()).hexdigest(),
            'independent_acceptance':False,'fitted':False,'scopes':result,
            'limitations':['Correlated region pairs are not independent trials.',
                           'AUC does not specify an identity threshold or score downstream assignments.',
                           'Mask reconstruction and CPU differences remain as recorded by the probe.']}


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('probe','draft','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args()
    if a.output.exists():raise ValueError('choose a new result path')
    answer=assess(a.probe,a.draft);a.output.parent.mkdir(parents=True,exist_ok=True)
    a.output.write_text(json.dumps(answer,indent=2)+'\n')
