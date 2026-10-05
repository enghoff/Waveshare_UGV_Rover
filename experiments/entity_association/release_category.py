"""Does the kind of thing a release shows tell a right release from a wrong one?

Each observation's stored SigLIP2 image vector is scored against a fixed list of
phrases embedded by the rover's own text tower, and its kind is the best phrase. A
record's kind is the commonest kind among its largest kept cluster. A release whose
kind matches its record's is a candidate to keep rather than release. Never writes a
store; the phrase vectors come from a file fetched once from the rover.

The codebase's prior is against this: `perceive._semantic` records that the nearest
phrase in a fixed list scored 0.08-0.12 whatever the crop held. That was a statement
about the score's size; whether the ranking still separates kinds is what this tests.

PREDICATE and PHRASES were written and committed before any phrase was embedded.
"""
import argparse
import base64
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from experiments.entity_association.release_position import judged_by_labels, judged_by_review, score

KINDS=['a dining chair','an armchair','an office chair','a framed painting on a wall','a rug',
       'a tiled floor','a wall','a ceiling','a door','an open doorway','a window','a cabinet',
       'a ceiling fan','a hanging lamp','a floor lamp','a person','a bed','a wardrobe','a table',
       'a desk','a cardboard box','a cable','a bin','a shelf','a light switch','a kitchen']
PHRASES=[f'a photo of {k}' for k in KINDS]
PREDICATE={
    'statistic':'cosine of the release to its best phrase minus its cosine to its record\'s kind; 0 when the kinds match',
    'record_kind':'commonest best phrase among the record\'s largest kept cluster, ties to the higher mean cosine',
    'gate':0.0,'rule':'release only where the statistic exceeds the gate, that is where the kinds differ',
    'pass':'among judged right and wrong releases the gate keeps at least 50% of wrong releases and still releases at least 80% of right releases',
    'review':'frozen release_audit.json on the fresh proposal',
    'confirm':'the same rule and thresholds on both older folds, judged by the older look labels; both must pass',
    'auc':'reported only; no threshold or phrase list is chosen from it',
    'phrases':'PHRASES, fixed before embedding; written by an agent who had seen the review sheets'}


def unit(blob):
    v=np.frombuffer(blob,dtype='<f4').astype('float64');n=float(np.linalg.norm(v))
    return None if n<1e-9 else v/n


def load_phrases(path):
    got=json.loads(path.read_text())
    if got['phrases']!=PHRASES:raise ValueError('phrase file is not for this phrase list')
    return np.stack([unit(base64.b64decode(v)) for v in got['vectors']])


def measure(proposal_path,database,text):
    p=json.loads(proposal_path.read_text())
    if p['database_sha256']!=hashlib.sha256(database.read_bytes()).hexdigest():raise ValueError('proposal is for another store')
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as con:
        vectors={i:unit(b) for i,b in con.execute('SELECT id,siglip_blob FROM observations') if b}
    cos={i:text@v for i,v in vectors.items() if v is not None}
    def kind(i):return int(np.argmax(cos[i])) if i in cos else None
    before=p['before_owners'];kept={}
    for tile in p['tiles']:
        for c in tile['clusters']:kept.setdefault(before[str(c[0])],[]).append(c)
    out={}
    for i in p['released']:
        clusters=sorted(kept.get(before[str(i)],[]),key=len,reverse=True)
        members=[k for k in (clusters[0] if clusters else []) if k in cos]
        result={'parent':before[str(i)],'kind':KINDS[kind(i)] if i in cos else None}
        if members and i in cos:
            counts=Counter(kind(k) for k in members);top=max(counts.values())
            record=max((k for k in counts if counts[k]==top),key=lambda k:np.mean([cos[m][k] for m in members]))
            result.update(record_kind=KINDS[record],statistic=float(cos[i].max()-cos[i][record]),
                          best_cosine=float(cos[i].max()))
        out[i]=result
    return p,out


def run(fresh,older,database,older_database,phrases,review,labels,output):
    if output.exists():raise ValueError('choose a new output path')
    text=load_phrases(phrases);proposal,measured=measure(fresh,database,text)
    result={'predicate':PREDICATE,'phrases':PHRASES,'phrases_sha256':hashlib.sha256(phrases.read_bytes()).hexdigest(),
            'review':score(measured,judged_by_review(review),PREDICATE['gate']),'confirm':{},
            'review_sha256':hashlib.sha256(review.read_bytes()).hexdigest(),
            'labels_sha256':hashlib.sha256(labels.read_bytes()).hexdigest(),'measured':{str(k):v for k,v in measured.items()}}
    for path in older:
        p,m=measure(path,older_database,text);judged=judged_by_labels(labels,p)
        result['confirm'][path.name]={**score(m,judged,PREDICATE['gate']),'overlap_with_review':len(set(judged)&set(measured)),
                                      'measured':{str(k):{**v,'label':judged.get(k)} for k,v in m.items()}}
    result['passes']=result['review']['passes'] and all(c['passes'] for c in result['confirm'].values())
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=1)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('request').add_argument('--output',type=Path,required=True)
    t=sub.add_parser('test')
    for name in ['fresh','database','older_database','phrases','review','labels','output']:t.add_argument('--'+name.replace('_','-'),type=Path,required=True)
    t.add_argument('--older',type=Path,nargs='+',required=True)
    a=p.parse_args()
    if a.command=='request':a.output.write_text(json.dumps({'phrases':PHRASES}))
    else:
        r=run(a.fresh,a.older,a.database,a.older_database,a.phrases,a.review,a.labels,a.output)
        print(json.dumps({'review':r['review'],'passes':r['passes'],
                          'confirm':{n:{x:y for x,y in c.items() if x!='measured'} for n,c in r['confirm'].items()}},indent=1))
