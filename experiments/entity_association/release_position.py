"""Does position tell a right release from a wrong one? Never writes a store.

Each release is measured against where its record's object is: a placement refitted
from the record's largest kept cluster, started from the record's stored placement,
and the release's own ray scored against it (bearing, range and height, in the
units `repair_geometry.geometry` already uses). A release that agrees with that
placement is a candidate to keep rather than release.

PREDICATE was written and committed before any statistic was computed.
"""
import argparse
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from experiments.entity_association.release_audit import VERDICTS
from experiments.entity_association.repair_geometry import Look, fit, geometry
from world_state import replay, resolve
from world_state.store import _readable

PREDICATE={
    'statistic':'max of the available bearing, range and height z of the release against its record\'s largest kept cluster, refitted from the stored placement',
    'gate_z':2.5,'gate':'release only where the statistic exceeds gate_z; the repair\'s existing veto value, not tuned here',
    'unmeasured':'no ray or no fit: reported separately, never gated',
    'pass':'among measured right and wrong releases the gate keeps at least 50% of wrong releases and still releases at least 80% of right releases',
    'review':'frozen release_audit.json on the fresh proposal',
    'confirm':'the same gate and thresholds on both older folds, for releases whose identity the older look labels settle; both must pass',
    'auc':'reported only; no threshold is chosen from it'}
KEEP_WRONG=.5;RELEASE_RIGHT=.8


def measure(proposal,database,map_path):
    p=json.loads(proposal.read_text())
    if p['database_sha256']!=hashlib.sha256(database.read_bytes()).hexdigest():raise ValueError('proposal is for another store')
    reach=replay.reach_from(str(map_path))
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as con:
        con.row_factory=sqlite3.Row
        session=int(con.execute("SELECT value FROM meta WHERE key='map_session'").fetchone()[0])
        rows={r['id']:_readable(dict(r)) for r in con.execute('SELECT * FROM observations WHERE map_session=?',(session,))}
        placed={r['id']:json.loads(r['placement_json']) for r in con.execute(
            'SELECT id,placement_json FROM entities WHERE placement_map_session=? AND placement_json IS NOT NULL',(session,))}
    before=p['before_owners'];kept={}
    for tile in p['tiles']:
        for c in tile['clusters']:kept.setdefault(before[str(c[0])],[]).append(c)
    out={}
    for i in p['released']:
        parent=before[str(i)];clusters=sorted(kept.get(parent,[]),key=len,reverse=True)
        ray=resolve.ray_of(rows[i],reach) if i in rows else None
        result={'parent':parent,'kept':sum(map(len,clusters)),'has_ray':ray is not None,'has_range':bool(ray and ray.get('range_m') is not None)}
        point=fit([Look(rows[k],resolve.ray_of(rows[k],reach)) for k in clusters[0]],placed.get(parent)) if clusters and ray else None
        if point is not None:
            g=geometry(point,ray);zs=[g[k] for k in ('zb','zr','zh') if g[k] is not None]
            result.update({k:g[k] for k in ('zb','zr','zh')},statistic=max(zs) if zs else None)
        out[i]=result
    return p,out


def judged_by_review(review):
    r=json.loads(review.read_text())
    return {x['observation']:VERDICTS[x['verdict']] for x in r['items']}


def judged_by_labels(labels,proposal,excluded=(61656,)):
    """Right when the older labels put the release outside the object its record's
    largest kept cluster mostly is; wrong when they put it inside; else unjudged."""
    things=json.loads(labels.read_text())['things'];where={}
    for name,thing in things.items():
        for i in thing['main_looks']:where[i]=(name,'main')
        for i in thing['odd']:where[i]=(name,'odd')
    before=proposal['before_owners'];kept={}
    for tile in proposal['tiles']:
        for c in tile['clusters']:kept.setdefault(before[str(c[0])],[]).append(c)
    out={}
    for i in proposal['released']:
        if i in excluded or i not in where:continue
        clusters=sorted(kept.get(before[str(i)],[]),key=len,reverse=True)
        mains=Counter(where[k][0] for k in (clusters[0] if clusters else []) if k in where and where[k][1]=='main')
        if not mains:continue
        main=mains.most_common(1)[0][0];name,kind=where[i]
        out[i]='wrong' if (name,kind)==(main,'main') else 'right'
    return out


def auc(right,wrong):
    if not right or not wrong:return None
    return sum((r>w)+.5*(r==w) for r in right for w in wrong)/(len(right)*len(wrong))


def score(measured,judged,gate=PREDICATE['gate_z']):
    groups={g:[measured[i]['statistic'] for i,v in judged.items() if v==g and measured[i].get('statistic') is not None]
            for g in ('right','wrong')}
    unmeasured=Counter(v for i,v in judged.items() if v in ('right','wrong') and measured[i].get('statistic') is None)
    right,wrong=groups['right'],groups['wrong']
    released_right=sum(s>gate for s in right);kept_wrong=sum(s<=gate for s in wrong)
    return {'right':len(right),'wrong':len(wrong),'unmeasured':dict(unmeasured),
            'right_still_released':released_right,'wrong_kept':kept_wrong,
            'right_still_released_share':released_right/len(right) if right else None,
            'wrong_kept_share':kept_wrong/len(wrong) if wrong else None,
            'after_gate_released':{'right':released_right,'wrong':len(wrong)-kept_wrong},
            'auc':auc(right,wrong),
            'passes':bool(right and wrong and released_right>=RELEASE_RIGHT*len(right) and kept_wrong>=KEEP_WRONG*len(wrong))}


def run(fresh,older,database,older_database,map_path,review,labels,output):
    if output.exists():raise ValueError('choose a new output path')
    proposal,measured=measure(fresh,database,map_path)
    result={'predicate':PREDICATE,'review':score(measured,judged_by_review(review)),'confirm':{},
            'review_sha256':hashlib.sha256(review.read_bytes()).hexdigest(),
            'labels_sha256':hashlib.sha256(labels.read_bytes()).hexdigest(),'measured':{str(k):v for k,v in measured.items()}}
    for path in older:
        p,m=measure(path,older_database,map_path);judged=judged_by_labels(labels,p)
        result['confirm'][path.name]={**score(m,judged),'overlap_with_review':len(set(judged)&set(measured)),
                                      'measured':{str(k):{**v,'label':judged.get(k)} for k,v in m.items()}}
    result['passes']=result['review']['passes'] and all(c['passes'] for c in result['confirm'].values())
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=1)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ['fresh','database','older_database','map','review','labels','output']:p.add_argument('--'+name.replace('_','-'),type=Path,required=True)
    p.add_argument('--older',type=Path,nargs='+',required=True)
    a=p.parse_args()
    r=run(a.fresh,a.older,a.database,a.older_database,a.map,a.review,a.labels,a.output)
    print(json.dumps({k:({x:y for x,y in v.items() if x!='measured'} if k=='confirm' else v) for k,v in r.items() if k not in ('measured','predicate')} |
                     {'confirm':{n:{x:y for x,y in c.items() if x!='measured'} for n,c in r['confirm'].items()}},indent=1))
