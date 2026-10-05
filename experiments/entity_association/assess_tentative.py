"""Score the predeclared influence experiment; never changes any world store."""
import argparse
from collections import defaultdict
from contextlib import closing
import hashlib
import json
from pathlib import Path
import sqlite3
import sys

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from experiments.entity_association.audit import Evidence
from world_state import replay


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def owners(d,key):return {int(i):v for i,v in d[key].items()}
def brief(s):return {k:s[k] for k in ('same_together','different_together','main_waiting','waiting','main_looks','regions','objects','per_object') if k in s}
def objects(s):
    per=s['per_object']
    missing=sum((v.get('waiting')==v.get('regions',v.get('looks'))) for v in per.values())
    split=sum((v['pieces_with_two_looks']>1 if 'pieces_with_two_looks' in v else sum(n>=2 for n in v['memberships'].values())>1) for v in per.values())
    return {'without_assigned_labelled_view':missing,'multiple_records_with_two_labelled_views':split}


def assess(control,candidate,evidence,fresh=False):
    b,c=control['checkpoints'][-1],candidate['checkpoints'][-1]
    bo,co,confirmed=owners(control,'owners'),owners(candidate,'owners'),owners(candidate,'confirmed_owners')
    if fresh:
        baseline=b['clear']['hypotheses'];hyp=c['clear']['hypotheses'];trusted=c['clear']['confirmed']
        base_wrong=set(map(tuple,baseline['wrong_pairs']))
        new_wrong=set(map(tuple,trusted['wrong_pairs']))-base_wrong
        new_hyp=set(map(tuple,hyp['wrong_pairs']))-base_wrong
        folds=None
    else:
        baseline=evidence.score(bo,None);hyp=evidence.score(co,None);trusted=evidence.score(confirmed,None)
        def wrong(mapping):return {(i,j) for i,j,same,_,_,_ in evidence.pairs if not same and mapping.get(i) is not None and mapping.get(i)==mapping.get(j)}
        base_wrong=wrong(bo);new_wrong=wrong(confirmed)-base_wrong;new_hyp=wrong(co)-base_wrong
        folds=c['older']
    checks={'wrong_confirmed_pairs_reduced':trusted['different_together']<baseline['different_together'],
            'no_new_wrong_confirmed_pairs':not new_wrong,
            'correct_confirmed_retention_at_least_90_percent':trusted['same_together']>=.9*baseline['same_together'],
            'correct_hypothesis_retention_at_least_95_percent':hyp['same_together']>=.95*baseline['same_together'],
            'no_more_missing_labelled_objects':objects(hyp)['without_assigned_labelled_view']<=objects(baseline)['without_assigned_labelled_view'],
            'no_more_fragmented_labelled_objects':objects(hyp)['multiple_records_with_two_labelled_views']<=objects(baseline)['multiple_records_with_two_labelled_views']}
    return {'baseline':brief(baseline),'hypotheses':brief(hyp),'confirmed':brief(trusted),
            'baseline_object_coverage':objects(baseline),'hypothesis_object_coverage':objects(hyp),'confirmed_object_coverage':objects(trusted),
            'correct_confirmed_retention':trusted['same_together']/baseline['same_together'],
            'correct_hypothesis_retention':hyp['same_together']/baseline['same_together'],
            'new_wrong_confirmed_pairs':sorted(map(list,new_wrong)),
            'new_wrong_hypothesis_pairs':sorted(map(list,new_hyp)),
            'checks':checks,'passes':all(checks.values()),'folds':folds}


def measurement_proof(source,end):
    def values(path):
        with closing(sqlite3.connect(path.resolve().as_uri()+'?mode=ro',uri=True)) as con:
            con.row_factory=sqlite3.Row
            return {r['id']:tuple(r[k] for k in replay.COLUMNS) for r in con.execute('SELECT * FROM observations')}
    a,b=values(source),values(end)
    assert a==b,'source measurements not all present byte-for-byte'
    return {'observations':len(a),'preserved':True}


def report(directory,output,previous_baseline):
    if output.exists():raise ValueError('choose a new summary path')
    names=['fresh-control','fresh-candidate','older-control','older-candidate','fresh-freeze-appearance','fresh-freeze-model']
    data={n:json.loads((directory/n/'result.json').read_text()) for n in names}
    evidence=Evidence(excluded=[61656],same_person=True)
    previous=json.loads(previous_baseline.read_text())
    assert data['older-control']['owners']==previous['owners_final'],'older control no longer reproduces prior replay'
    assert not data['fresh-control']['membership_differences'] and not data['fresh-control']['entity_state_differences']
    result={'requirements':{'R-WS-13':'open','R-WS-17':'proposed','R-WS-18':'proposed'},
            'predicate_commit':'c0b1c42','ablation_predicate_commit':'441bf76',
            'independent_acceptance':False,'deployed':False,'hardware_validated':False,
            'fresh':assess(data['fresh-control'],data['fresh-candidate'],evidence,True),
            'older':assess(data['older-control'],data['older-candidate'],evidence),
            'reproduction':{'fresh_exact_memberships':4026,'fresh_entity_state_differences':0,
                           'older_matches_prior_replay':True,'previous_baseline_sha256':digest(previous_baseline),
                           'older_live_snapshot_exactness':False,
                           'limitation':'The older rebuilt replay is the established baseline, not the raw live snapshot: it assigns 2,887 of 3,540 rows; the saved snapshot assigns 2,979. Names also differ.'},
            'arms':{}}
    for name,d in data.items():
        source=ROOT/('captures/2026-10-04-reader-validation/after.db' if name.startswith('fresh') else 'captures/2026-10-04-association-likelihood/world.db')
        assert digest(source)==d['hashes']['database'] and d['source_unchanged']
        checkpoint=d['checkpoints'][-1]
        result['arms'][name]={'hashes':d['hashes'],'result_sha256':digest(directory/name/'result.json'),
            'end_db_sha256':digest(directory/name/'end.db'),
            'measurement_proof':measurement_proof(source,directory/name/'end.db'),
            'final_counts':{k:checkpoint[k] for k in ('observations','assigned','confirmed','tentative')},
            'stats':d['candidate_stats'],
            'clear':{tier:brief(v) for tier,v in checkpoint.get('clear',{}).items()},
            'all_draft':{tier:brief(v) for tier,v in checkpoint.get('all_draft',{}).items()}}
    result['passes']=result['fresh']['passes'] and result['older']['passes']
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2)+'\n')
    print('candidate passes',result['passes'])
    for n in ('fresh','older'):print(n,result[n]['checks'])
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser()
    for name in ('directory','output','previous-baseline'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();report(a.directory,a.output,a.previous_baseline)
