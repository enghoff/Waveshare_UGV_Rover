"""Disjoint, bounded reconstruction of placed records; never writes the source.

Uses the previous neighbourhood bench's sum linkage and fixed thresholds.
Labels select fitting folds and scoring only, never tiles or proposed memberships.
No output is accepted identity, a live repair, or a navigation destination.
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import closing, nullcontext
import hashlib
import json
import math
from pathlib import Path
import sqlite3
import sys
import time
import tempfile

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from experiments.entity_association.audit import Evidence, LABELS
from experiments.entity_association.repair_geometry import Look, fit, geometry
from world_state import merging, replay, resolve
from world_state.store import _readable
from world_state.store import WorldStore

RADIUS = .75
MAX_ROWS = 512
THRESHOLD = 1.0
VETO_Z = 2.5


def covariance(point):
    major=max(float(point.get('error_major_m') or point.get('uncertainty_m') or .3),.05)
    minor=max(float(point.get('error_minor_m') or major),.05)
    angle=math.radians(float(point.get('error_major_deg') or 0))
    turn=np.array([[math.cos(angle),-math.sin(angle)],[math.sin(angle),math.cos(angle)]])
    return turn@np.diag([major**2,minor**2])@turn.T+np.eye(2)*(float(point.get('extent_m') or 0)/2)**2


def neighbourhood(a,b):
    delta=np.array([a['x_m']-b['x_m'],a['y_m']-b['y_m']])
    distance=float(np.linalg.norm(delta))
    return distance<=RADIUS or (distance<=3 and float(delta@np.linalg.solve(covariance(a)+covariance(b),delta))<4)


def tiles(placed, counts):
    remaining=set(placed)&set(counts);groups=[]
    while remaining:
        anchor=min(remaining,key=lambda key:(-counts[key],key))
        neighbours=sorted((k for k in remaining if k!=anchor and neighbourhood(placed[anchor],placed[k])),
                          key=lambda k:((placed[k]['x_m']-placed[anchor]['x_m'])**2+
                                        (placed[k]['y_m']-placed[anchor]['y_m'])**2,k))
        group=[anchor];size=counts[anchor]
        for other in neighbours:
            if size+counts[other]<=MAX_ROWS:
                group.append(other);size+=counts[other]
        groups.append(group);remaining.difference_update(group)
    assert len([k for g in groups for k in g])==len(set(k for g in groups for k in g))
    return groups


def cluster(pool, placed, model, threshold=THRESHOLD, veto=True, trace_ids=(), trace=None, extra=None):
    n=len(pool)
    if not n:return [],[],{}
    plain=np.stack([p.v for p in pool]);masked=np.stack([p.a if p.a is not None else p.v for p in pool])
    semantic=np.stack([p.g for p in pool])
    w=model['coef'];s=w[0]*(plain@plain.T)+w[1]*(masked@masked.T)+w[2]*(semantic@semantic.T)+model['offset']
    if len(w)==4:
        if extra is None:raise ValueError('four-channel model needs masked semantic features')
        added=np.stack([extra[p.id] for p in pool])
        s+=w[3]*(added@added.T)
    individual=s.copy() if trace is not None else None
    backend=[p.row.get('vectors_from') for p in pool]
    cannot=np.array([[(a.inference is not None and a.inference==b.inference) or
                      not backend[i] or backend[i]!=backend[j] for j,b in enumerate(pool)] for i,a in enumerate(pool)])
    np.fill_diagonal(cannot,True)
    s=np.where(cannot,0,s);counts=np.ones((n,n));alive=np.ones(n,bool)
    members=[[i] for i in range(n)];tried=np.zeros((n,n),bool);stats=Counter()
    while True:
        scores=np.where(cannot|tried|~alive[:,None]|~alive[None,:],-np.inf,s-threshold*counts)
        k=int(np.argmax(scores));i,j=divmod(k,n)
        if not np.isfinite(scores[i,j]) or scores[i,j]<0:break
        union=[pool[k] for k in members[i]+members[j]]
        owners=Counter(p.entity for p in union if p.entity in placed)
        start=placed[sorted(owners,key=lambda e:(-owners[e],e))[0]] if owners else None
        if veto:
            point=fit(union,start)
            small=members[i] if len(members[i])<=len(members[j]) else members[j]
            if point is None:
                stats['no_fit_veto']+=1;tried[i,j]=tried[j,i]=True;continue
            misses=[]
            for k in small:
                g=geometry(point,pool[k].ray)
                zs=[g[key] for key in ('zb','zr','zh') if g[key] is not None]
                if zs:misses.append(max(zs))
            if misses and float(np.median(misses))>VETO_Z:
                stats['geometry_veto']+=1;tried[i,j]=tried[j,i]=True;continue
        if trace is not None and any(p.id in trace_ids for p in union):
            pair_values=individual[np.ix_(members[i],members[j])].ravel()
            trace.append({'left':[pool[k].id for k in members[i]],
                          'right':[pool[k].id for k in members[j]],
                          'appearance_mean':float(pair_values.mean()),
                          'appearance_min':float(pair_values.min()),
                          'appearance_max':float(pair_values.max()),
                          'fraction_below_threshold':float(np.mean(pair_values<threshold)),
                          'small_side_geometry_median':float(np.median(misses)) if veto and misses else None,
                          'small_side_geometry_max':max(misses) if veto and misses else None})
        members[i]+=members[j];members[j]=[];alive[j]=False
        s[i,:]+=s[j,:];s[:,i]=s[i,:];counts[i,:]+=counts[j,:];counts[:,i]=counts[i,:]
        cannot[i,:]|=cannot[j,:];cannot[:,i]=cannot[i,:];cannot[i,i]=True
        tried[i,:]=False;tried[:,i]=False;stats['joins']+=1
    clusters=[[pool[k] for k in group] for group in members if len(group)>=2]
    released=[pool[k] for group in members if len(group)==1 for k in group]
    assert sorted(p.id for p in pool)==sorted([p.id for g in clusters for p in g]+[p.id for p in released])
    return clusters,released,dict(stats)


def regroup(database, looks, placed, owners, model, reach, bank=None):
    """The existing reader module over reconstructed records in an empty clone."""
    from world_state.reader_groups import preview
    weights,offset=merging.APPEARANCE_WEIGHTS,merging.APPEARANCE_OFFSET
    merging.APPEARANCE_WEIGHTS=tuple(model['coef'][:3]);merging.APPEARANCE_OFFSET=model['offset']
    by_owner={}
    for i,owner in owners.items():
        if owner is not None:by_owner.setdefault(owner,[]).append(looks[i])
    try:
        with tempfile.TemporaryDirectory(prefix='ugv-repair-groups-') as tmp:
            store=WorldStore(tmp)
            try:
                with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as con:
                    con.row_factory=sqlite3.Row
                    session=int(con.execute("SELECT value FROM meta WHERE key='map_session'").fetchone()[0])
                    with store.db:
                        store.db.execute("REPLACE INTO meta(key,value) VALUES('map_session',?)",(str(session),))
                        for raw in con.execute('SELECT * FROM observations WHERE map_session=?',(session,)):
                            row=dict(raw);row['entity_id']=owners[row['id']]
                            keys=list(row);store.db.execute('INSERT INTO observations ('+','.join(keys)+') VALUES ('+
                                                           ','.join('?' for _ in keys)+')',tuple(row[k] for k in keys))
                with store.db:
                    for owner,members in by_owner.items():
                        original=Counter(p.entity for p in members if p.entity in placed)
                        start=placed[sorted(original,key=lambda e:(-original[e],e))[0]] if original else None
                        point=fit(members,start)
                        store.db.execute('INSERT INTO entities(id,kind,label,canonical_description,created_at,last_seen_at,observation_count,placement_json,placement_map_session) '
                                         'VALUES(?,?,?,?,?,?,?,?,?)',(owner,'object','','',min(p.row['observed_at'] for p in members),max(p.row['observed_at'] for p in members),
                                                               len(members),json.dumps(point) if point else None,session))
                if bank is not None:
                    from experiments.entity_association.semantic_features import reader_channel
                with reader_channel(bank,model['coef'][3]) if bank is not None else nullcontext():
                    result=preview(store,reach=reach)
                assert result['ok'] and not result['stale'] and result['converged'],result
            finally:store.close()
    finally:merging.APPEARANCE_WEIGHTS=weights;merging.APPEARANCE_OFFSET=offset
    aliases={member:g['representative'] for g in result['groups'] for member in g['members']}
    return {i:aliases.get(owner,owner) for i,owner in owners.items()},result


def run(database, map_path, output, fold, labels=None, second_stage=False, trace_ids=(), appearance_columns=None, semantic_probe=None):
    if output.exists():raise ValueError('choose a new result path')
    digest=hashlib.sha256(database.read_bytes()).hexdigest()
    evidence=Evidence(excluded=[61656],same_person=True)
    bank=None
    if semantic_probe is not None:
        from experiments.entity_association.semantic_features import SemanticFeatures
        bank=SemanticFeatures(semantic_probe);bank.validate(evidence.rows.values())
    model=(evidence.fit(fold,columns=appearance_columns or (0,1,2,3),additional=bank.vectors) if bank is not None else
           evidence.fit(fold,columns=appearance_columns) if appearance_columns is not None else
           evidence.fit(fold) if fold is not None else
           {'coef':list(merging.APPEARANCE_WEIGHTS),'offset':merging.APPEARANCE_OFFSET})
    reach=replay.reach_from(str(map_path));began=time.monotonic()
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as con:
        con.row_factory=sqlite3.Row
        session=int(con.execute("SELECT value FROM meta WHERE key='map_session'").fetchone()[0])
        rows=[_readable(dict(r),vectors=True) for r in con.execute('SELECT * FROM observations WHERE map_session=? ORDER BY id',(session,))]
        placed={r['id']:json.loads(r['placement_json']) for r in con.execute(
            'SELECT id,placement_json FROM entities WHERE placement_map_session=? AND placement_json IS NOT NULL',(session,))}
    looks={r['id']:Look(r,resolve.ray_of(r,reach)) for r in rows}
    if bank is not None:missing_features=bank.validate(rows)
    owners={r['id']:r['entity_id'] for r in rows};before=owners.copy()
    by_entity={k:[p for p in looks.values() if p.entity==k] for k in placed}
    groups=tiles(placed,{k:len(v) for k,v in by_entity.items() if v})
    events=[];all_stats=Counter();untouched=[];released_ids=[];join_trace=[]
    for index,group in enumerate(groups):
        # Separate incomparable vector widths. Missing appearance stays unchanged.
        batches={}
        for entity in group:
            for look in by_entity[entity]:
                if (look.v is None or look.g is None or not look.row.get('vectors_from') or
                        (bank is not None and look.id not in bank.vectors)):
                    untouched.append(look.id);continue
                batches.setdefault((len(look.v),len(look.g),len(look.a) if look.a is not None else len(look.v)),[]).append(look)
        detail={'tile':index,'records':group,'regions':sum(len(by_entity[k]) for k in group),'clusters':[],'released':[]}
        for batch,pool in enumerate(batches.values()):
            clusters,released,stats=cluster(pool,placed,model,trace_ids=trace_ids,
                                           trace=join_trace if trace_ids else None,
                                           extra=bank.vectors if bank is not None else None);all_stats.update(stats)
            for number,members in enumerate(clusters):
                name=f'repair:{index}:{batch}:{number}'
                ids=[p.id for p in members];detail['clusters'].append(ids)
                for i in ids:owners[i]=name
            for look in released:owners[look.id]=None;released_ids.append(look.id);detail['released'].append(look.id)
        events.append(detail)
        if index<5 or index%10==9 or index==len(groups)-1:
            print(f'fold {fold}: tile {index+1}/{len(groups)}, {detail["regions"]} regions, {time.monotonic()-began:.1f}s',flush=True)
    result={'database_sha256':digest,'map_sha256':hashlib.sha256(map_path.read_bytes()).hexdigest(),
            'fold':fold,'model':model,'regions':len(rows),'tiles':events,'stats':dict(all_stats),
            'pending_original':sum(v is None for v in before.values()),'released':released_ids,
            'unscorable_unchanged':untouched,'before_owners':before,'owners':owners,
            'source_unchanged':hashlib.sha256(database.read_bytes()).hexdigest()==digest,
            'independent_acceptance':False,'preview_only':True,'radius_m':RADIUS,'max_tile_rows':MAX_ROWS,
            'threshold':THRESHOLD,'geometry_veto_z':VETO_Z,'seconds':time.monotonic()-began,
            'oversize_tiles':[t['tile'] for t in events if t['regions']>MAX_ROWS],
            'limitations':['Tiling boundaries prevent cross-tile repair; pending regions are unchanged.',
                           'Same-picture conflicts include head/body and other legitimate object parts.',
                           'Geometry veto tests the median of the smaller side, not every constituent.',
                           'No-fit unions are refused; prior selected-neighbourhood bench allowed them.',
                           'Reconstruction is a disposable proposal; no persistent IDs or reader destinations are created.']}
    if second_stage:
        grouped,preview=regroup(database,looks,placed,owners,model,reach,bank)
        result['grouped_owners']=grouped;result['grouping_preview']=preview
    if trace_ids:result['join_trace']=join_trace;result['trace_ids']=list(trace_ids)
    if bank is not None:
        result['masked_semantic_probe']={'vectors_sha256':bank.sha256,
            'model_sha256':bank.provenance['model_sha256'],'missing_observations':missing_features,
            'training_missing_observations':bank.validate(evidence.rows.values()),
            'columns':list(appearance_columns or (0,1,2,3)),
            'policy':'Missing features retain original owners and cannot support reader grouping.'}
    if appearance_columns is not None or bank is not None:
        result['appearance_columns']=list(appearance_columns or (0,1,2,3))
        result['training_labels_sha256']=hashlib.sha256(LABELS.read_bytes()).hexdigest()
        result['training_database_sha256']=hashlib.sha256(evidence.database.read_bytes()).hexdigest()
    result['source_unchanged']=hashlib.sha256(database.read_bytes()).hexdigest()==digest
    result['seconds']=time.monotonic()-began
    if fold is not None:
        result['before']=evidence.score(before,fold);result['after']=evidence.score(owners,fold)
        result['labels_sha256']=hashlib.sha256(LABELS.read_bytes()).hexdigest()
        if second_stage:result['grouped']=evidence.score(grouped,fold)
        def wrong_pairs(mapping):
            return {(i,j) for i,j,same,assigned,_,_ in evidence.pairs if assigned==fold and not same
                    and mapping.get(i) is not None and mapping.get(i)==mapping.get(j)}
        result['new_wrong_after']=sorted(wrong_pairs(owners)-wrong_pairs(before))
        if second_stage:
            result['new_wrong_grouped']=sorted(wrong_pairs(grouped)-wrong_pairs(before))
            result['wrong_restored_by_grouping']=sorted(wrong_pairs(grouped)-wrong_pairs(owners))
    if labels:
        from experiments.entity_association.assess_recording import score
        doc=json.loads(labels.read_text());assert doc['source_database_sha256']==digest
        usable=[r for r in doc['labels'] if r['verdict']=='object' and r['physical_object'] and r['review_status']=='draft_clear']
        result['fresh_before']=score(usable,before);result['fresh_after']=score(usable,owners)
        if second_stage:result['fresh_grouped']=score(usable,grouped)
        tentative=[r for r in doc['labels'] if r['verdict']=='object' and r['physical_object']]
        result['fresh_tentative']={'before':score(tentative,before),'after':score(tentative,owners)}
        if second_stage:result['fresh_tentative']['grouped']=score(tentative,grouped)
        result['draft_sha256']=hashlib.sha256(labels.read_bytes()).hexdigest()
    assert result['source_unchanged']
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--database',type=Path,required=True);p.add_argument('--map',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--fold',type=int,choices=[0,1]);p.add_argument('--labels',type=Path)
    p.add_argument('--second-stage',action='store_true')
    p.add_argument('--trace-observation',action='append',type=int,default=[])
    p.add_argument('--appearance-columns',type=int,nargs='+',choices=[0,1,2,3],
                   help='refit development channels (0 plain, 1 masked, 2 semantic, 3 masked semantic with --semantic-probe); no fold fits all development objects')
    p.add_argument('--semantic-probe',type=Path)
    args=p.parse_args();run(args.database,args.map,args.output,args.fold,args.labels,args.second_stage,args.trace_observation,args.appearance_columns,args.semantic_probe)
