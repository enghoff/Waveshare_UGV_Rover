"""Offline influence separation. Never import this proxy into a rover service."""
from collections import Counter
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys
import tempfile
import time

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from world_state import appearance, locate, replay, resolve
from world_state.store import WorldStore, _readable, EXEMPLARS


def witnesses(row, peers):
    frames={}; backend=row.get('vectors_from')
    if not backend or row.get('inference_id') is None:return []
    for peer in peers:
        frame=peer.get('inference_id')
        if frame is None or frame==row['inference_id'] or peer.get('vectors_from')!=backend:continue
        scores=[]
        for key in ('dino_blob','dino_alone_blob'):
            a,b=row.get(key),peer.get(key)
            if not a or not b or len(a)!=len(b):break
            scores.append(appearance.similarity(a,b))
        if len(scores)==2 and min(scores)>=resolve.RECOGNISED:frames.setdefault(frame,peer['id'])
    return list(frames.values())


class TentativeStore:
    """Archive memberships are hypotheses; confirmed rows alone train the model.

    Founder exemption and inherited trust are deliberately explicit limitations.
    No labels or unseen observations are available to this object.
    """
    def __init__(self,store):
        self.store=store
        self.confirmed={r[0] for r in store.db.execute('SELECT id FROM observations WHERE entity_id IS NOT NULL')}
        self.stats=Counter();self.events=[];self.last={};self.support={}

    def __getattr__(self,name):return getattr(self.store,name)

    def create_entity(self,*args,**kwargs):
        return self.store.create_entity(*args,**kwargs)

    def attach(self,entity_id,ids,why=''):
        existed=self.store.db.execute('SELECT 1 FROM observations WHERE entity_id=? LIMIT 1',(entity_id,)).fetchone()
        result=self.store.attach(entity_id,ids,why)
        self.last[entity_id]=list(ids)
        if not existed:
            self.confirmed.update(ids);self.stats['founders']+=len(ids)
            self.events.append({'kind':'founders','entity':entity_id,'ids':list(ids)})
        else:
            self.stats['tentative_attachments']+=len(ids)
        return result

    def observations(self,entity_id=None,*,limit=50,**kwargs):
        if entity_id is None:return self.store.observations(entity_id,limit=limit,**kwargs)
        # Filter BEFORE the history limit; tentative views cannot evict older support.
        rows=self.store.observations(entity_id,limit=2147483647,**kwargs)
        rows=[r for r in rows if r['id'] in self.confirmed][:limit]
        self.stats['position_history_reads']+=1
        assert all(r['id'] in self.confirmed for r in rows)
        return rows

    def add_exemplar(self,entity_id,vector,keep=EXEMPLARS,alone=b''):
        ids=self.last.get(entity_id,[])
        rows=self.store.observations(ids=ids,vectors=True) if ids else []
        approved=[r for r in rows if r['id'] in self.confirmed and r.get('dino_blob')==vector and (r.get('dino_alone_blob') or b'')==alone]
        if not approved:
            self.stats['tentative_exemplar_updates_withheld']+=1
            return len(self.store.exemplars(entity_id,width=len(vector)))
        self.stats['confirmed_exemplar_updates']+=1
        return self.store.add_exemplar(entity_id,vector,keep=keep,alone=alone)

    def promote(self,reach=None):
        # Snapshot confirmed peers: promotion in this round cannot bootstrap itself.
        groups={};pending=[]
        for raw in self.store.db.execute('SELECT * FROM observations WHERE entity_id IS NOT NULL ORDER BY observed_at,id'):
            row=_readable(dict(raw),vectors=True)
            if row['id'] in self.confirmed:groups.setdefault(row['entity_id'],[]).append(row)
            else:pending.append(row)
        approved=[]
        for row in pending:
            got=witnesses(row,groups.get(row['entity_id'],[])[-24:])
            if len(got)>=2:approved.append((row,got[:2]))
        return self.apply_promotions(approved,reach)

    def apply_promotions(self,approved,reach=None):
        changed=set()
        for row,got in approved:
            self.confirmed.add(row['id']);self.support[row['id']]=got;changed.add(row['entity_id'])
            self.events.append({'kind':'promoted','entity':row['entity_id'],'id':row['id'],'witnesses':got})
            if row.get('dino_blob'):
                self.store.add_exemplar(row['entity_id'],row['dino_blob'],alone=row.get('dino_alone_blob') or b'')
        for eid in sorted(changed):resolve._replace_placement(self,eid,self.map_session(),reach)
        self.stats['promoted']+=len(approved)
        return len(approved)


class BridgeStore(TentativeStore):
    """Permit anchored, independent tentative views to establish a new appearance."""
    def __init__(self,store):
        super().__init__(store)
        self.pair_scores={};self.pair_fixes={}

    def score_pair(self,a,b):
        key=tuple(sorted((a['id'],b['id'])))
        if key not in self.pair_scores:
            if not a.get('vectors_from') or a['vectors_from']!=b.get('vectors_from'):
                self.pair_scores[key]=None
            else:
                scores=[]
                for k in ('dino_blob','dino_alone_blob'):
                    x,y=a.get(k),b.get(k)
                    if not x or not y or len(x)!=len(y):break
                    scores.append(appearance.similarity(x,y))
                self.pair_scores[key]=min(scores) if len(scores)==2 else None
        return self.pair_scores[key]

    def promote(self,reach=None):
        import numpy as np
        from experiments.entity_association.repair_groups import covariance
        confirmed={};pending={};rows={}
        for raw in self.store.db.execute('SELECT * FROM observations WHERE entity_id IS NOT NULL ORDER BY observed_at,id'):
            r=_readable(dict(raw),vectors=True);rows[r['id']]=r
            (confirmed if r['id'] in self.confirmed else pending).setdefault(r['entity_id'],[]).append(r)
        approved={};bridge=[]
        for eid,waiting in pending.items():
            peers=confirmed.get(eid,[])[-24:];anchors={}
            for r in waiting:
                got=witnesses(r,peers)
                if len(got)>=2:approved[r['id']]=(r,got[:2])
                scored=[(self.score_pair(r,p),p['id']) for p in peers if p.get('inference_id') is not None and p['inference_id']!=r.get('inference_id')]
                scored=[x for x in scored if x[0] is not None]
                anchors[r['id']]=max(scored,default=(0.,None))
            raw=self.store.db.execute('SELECT placement_json FROM entities WHERE id=?',(eid,)).fetchone()
            if raw is None or not raw[0]:continue
            point=json.loads(raw[0]);core_cov=covariance(point)
            for r in waiting:
                if r['id'] in approved:continue
                if anchors[r['id']][0]<resolve.DIFFERENT_THING:
                    self.stats['bridge_no_anchor']+=1;continue
                for other in waiting[-24:]:
                    if other['id']==r['id'] or other.get('inference_id') is None or other['inference_id']==r.get('inference_id'):continue
                    if anchors[other['id']][0]<resolve.DIFFERENT_THING:continue
                    if max(anchors[r['id']][0],anchors[other['id']][0])<resolve.RECOGNISED:continue
                    score=self.score_pair(r,other)
                    if score is None or score<resolve.RECOGNISED:continue
                    key=tuple(sorted((r['id'],other['id'])))
                    if key not in self.pair_fixes:
                        a,b=resolve.ray_of(r,reach),resolve.ray_of(other,reach)
                        self.pair_fixes[key]=locate.fix(a,b) if a and b else None
                    fix=self.pair_fixes[key]
                    if fix is None:self.stats['bridge_no_fix']+=1;continue
                    delta=np.array([point['x_m']-fix['x_m'],point['y_m']-fix['y_m']])
                    distance2=float(delta@np.linalg.solve(core_cov+covariance(fix),delta))
                    if distance2>13.815510557964274:self.stats['bridge_geometry_refused']+=1;continue
                    for member,partner in ((r,other),(other,r)):
                        if member['id'] not in approved:
                            approved[member['id']]=(member,[anchors[member['id']][1],partner['id']])
                            bridge.append({'id':member['id'],'partner':partner['id'],'anchor':anchors[member['id']][1],
                                           'strong_anchor':max(anchors[r['id']],anchors[other['id']])[1],
                                           'pair_score':score,'ellipse_distance2':distance2,
                                           'baseline_m':fix['baseline_m'],'parallax_deg':fix['parallax_deg']})
                    break
        self.stats['bridge_promotions']+=len(bridge)
        for item in bridge:self.events.append({'kind':'bridge',**item})
        return self.apply_promotions([approved[i] for i in sorted(approved)],reach)


class FrozenStore(TentativeStore):
    """Explanatory extremes, not a confirmation rule or a production candidate."""
    def __init__(self,store,freeze_position=False):
        super().__init__(store)
        self.freeze_position=freeze_position
        self.positioned={r[0] for r in store.db.execute('SELECT id FROM entities WHERE placement_json IS NOT NULL')}
        self.founders=set()

    def attach(self,eid,ids,why=''):
        existed=self.store.db.execute('SELECT 1 FROM observations WHERE entity_id=? LIMIT 1',(eid,)).fetchone()
        result=super().attach(eid,ids,why)
        if not existed:self.founders.update(ids)
        self.confirmed.update(ids)
        return result

    def observations(self,*args,**kwargs):return self.store.observations(*args,**kwargs)

    def add_exemplar(self,eid,vector,keep=EXEMPLARS,alone=b''):
        rows=self.store.observations(ids=self.last.get(eid,[]),vectors=True)
        if not any(r['id'] in self.founders and r.get('dino_blob')==vector for r in rows):
            self.stats['appearance_updates_frozen']+=1
            return len(self.store.exemplars(eid,width=len(vector)))
        return self.store.add_exemplar(eid,vector,keep=keep,alone=alone)

    def place(self,eid,point,session):
        if self.freeze_position and eid in self.positioned:
            self.stats['position_updates_frozen']+=1
            return
        result=self.store.place(eid,point,session)
        if point is not None:self.positioned.add(eid)
        return result


def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def run(database,map_path,output,mode,before=None,draft=None):
    if output.exists():raise ValueError('choose a new output directory')
    output.mkdir(parents=True)
    hashes={'database':sha(database),'map':sha(map_path)}
    if before:hashes['before']=sha(before)
    reach=replay.reach_from(str(map_path))
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro',uri=True)) as con:
        con.row_factory=sqlite3.Row
        source=[dict(r) for r in con.execute('SELECT * FROM observations ORDER BY observed_at,id')]
        session=int(con.execute("SELECT value FROM meta WHERE key='map_session'").fetchone()[0])
        expected={r['id']:r['entity_id'] for r in source}
        expected_entities={r['id']:(r['placement_json'],r['exemplars'],r['exemplars_alone']) for r in con.execute('SELECT * FROM entities')}
        inferences={r['id']:dict(r) for r in con.execute('SELECT * FROM inferences')}
    cutoff=0
    if before:
        with closing(sqlite3.connect(before.resolve().as_uri()+'?mode=ro',uri=True)) as con:
            cutoff=con.execute('SELECT MAX(id) FROM observations').fetchone()[0]
    rows=[r for r in source if r['map_session']==session and r['id']>cutoff]
    groups={}
    for r in rows:groups.setdefault(r['inference_id'],[]).append(r)
    from experiments.entity_association.audit import Evidence
    from experiments.entity_association.assess_recording import score
    evidence=Evidence(excluded=[61656],same_person=True)
    labels=[]
    if draft:
        doc=json.loads(draft.read_text());assert doc['source_database_sha256']==hashes['database']
        labels=[r for r in doc['labels'] if r['verdict']=='object' and r['physical_object']]
        hashes['draft']=sha(draft)
    checkpoints=[];passes=[];started=time.monotonic()
    with tempfile.TemporaryDirectory(prefix='ugv-tentative-evidence-') as tmp:
        store=WorldStore(tmp)
        try:
            if before:
                with closing(sqlite3.connect(before.resolve().as_uri()+'?mode=ro',uri=True)) as con:con.backup(store.db)
                store._create()
            else:
                with store.db:store.db.execute("REPLACE INTO meta(key,value) VALUES('map_session',?)",(str(session),))
            active=(BridgeStore(store) if mode=='bridge' else TentativeStore(store) if mode=='candidate' else
                    FrozenStore(store,mode=='freeze-model') if mode.startswith('freeze-') else store)
            def checkpoint(frame):
                owners=dict(store.db.execute('SELECT id,entity_id FROM observations'))
                trusted={i:(eid if mode not in ('candidate','bridge') or i in active.confirmed else None) for i,eid in owners.items()}
                out={'frame':frame,'assigned':sum(v is not None for v in owners.values()),
                     'confirmed':sum(v is not None for v in trusted.values()),
                     'tentative':sum(owners[i] is not None and trusted[i] is None for i in owners),
                     'observations':len(owners)}
                if draft:
                    clear=[r for r in labels if r['review_status']=='draft_clear']
                    out['clear']={'hypotheses':score(clear,owners),'confirmed':score(clear,trusted)}
                    out['all_draft']={'hypotheses':score(labels,owners),'confirmed':score(labels,trusted)}
                else:
                    out['older']={str(f):{'hypotheses':evidence.score(owners,f),'confirmed':evidence.score(trusted,f)} for f in (0,1)}
                checkpoints.append(out)
                return owners,trusted
            cuts={max(1,round(len(groups)*n/5)) for n in range(1,6)}
            for n,(fid,group) in enumerate(groups.items(),1):
                with store.db:
                    for r in group:
                        columns=['id',*replay.COLUMNS]
                        store.db.execute('INSERT INTO observations ('+','.join(columns)+') VALUES ('+','.join('?' for _ in columns)+')',tuple(r.get(k) for k in columns))
                settle=not before or 'identity not settled yet' not in (inferences[fid].get('detail') or '')
                if settle:
                    result=resolve.resolve(active,reach=reach)
                    promoted=active.promote(reach) if mode in ('candidate','bridge') else 0
                    passes.append({'frame':n,'background':False,'matched':result['matched'],'created':result['created'],'promoted':promoted})
                if before and n in (7,13,16,22,25,27):
                    result=resolve.resolve(active,reach=reach)
                    promoted=active.promote(reach) if mode in ('candidate','bridge') else 0
                    passes.append({'frame':n,'background':True,'matched':result['matched'],'created':result['created'],'promoted':promoted})
                if n in cuts:checkpoint(n)
                if n<=3 or n%25==0 or n==len(groups):print(mode,'frame',n,'/',len(groups),'seconds',round(time.monotonic()-started,1),flush=True)
            owners,trusted=checkpoint('final')
            actual_entities={r['id']:(r['placement_json'],r['exemplars'],r['exemplars_alone']) for r in store.db.execute('SELECT * FROM entities')}
            original_measurements={r['id']:tuple(r.get(k) for k in replay.COLUMNS) for r in source if r['id'] in owners}
            actual_measurements={r['id']:tuple(r[k] for k in replay.COLUMNS) for r in store.db.execute('SELECT * FROM observations')}
            assert original_measurements==actual_measurements,'archived measurements changed or disappeared'
            differences=[i for i,eid in expected.items() if owners.get(i)!=eid]
            result={'mode':mode,'hashes':hashes,'source_unchanged':sha(database)==hashes['database'] and (not before or sha(before)==hashes['before']),
                    'independent_acceptance':False,'measurements_preserved':True,'cutoff':cutoff,'checkpoints':checkpoints,'passes':passes,
                    'owners':owners,'confirmed_owners':trusted,'membership_differences':differences,
                    'entity_state_differences':[eid for eid,state in expected_entities.items() if actual_entities.get(eid)!=state],
                    'seconds':time.monotonic()-started,'predicate_commit':'febca34' if mode=='bridge' else 'c0b1c42',
                    'candidate_stats':dict(active.stats) if mode!='control' else {},
                    'candidate_events':active.events if mode!='control' else [],
                    'limitations':['Founder and inherited evidence trusted unchanged.',
                                   'Tentative destinations retained and not yet reconsidered.',
                                   'Distinct frames are witnesses, not guaranteed independent viewpoints.',
                                   'Frozen occupancy map and inferred fresh background schedule.',
                                   'Development labels; no hardware validation.']}
            with closing(sqlite3.connect(output/'end.db')) as dest:store.db.backup(dest)
        finally:store.close()
    assert result['source_unchanged']
    (output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(mode,'done','membership differences',len(differences),'model state differences',len(result['entity_state_differences']),flush=True)
    return result


if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser()
    for name in ('database','map','output'):p.add_argument('--'+name,type=Path,required=True)
    for name in ('before','draft'):p.add_argument('--'+name,type=Path)
    p.add_argument('--mode',choices=('control','candidate','bridge','freeze-appearance','freeze-model'),required=True)
    a=p.parse_args();run(a.database,a.map,a.output,a.mode,a.before,a.draft)
