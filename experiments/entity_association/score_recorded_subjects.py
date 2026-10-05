"""Score frozen development subject pairs without treating labels as acceptance."""
import argparse
import itertools
import json
from pathlib import Path


def score(result_path, labels_path, output):
    assert not output.exists(), 'choose a new output file'
    result=json.loads(result_path.read_text())
    labels=json.loads(labels_path.read_text())['subjects']
    same=[(a,b) for ids in labels.values() for a,b in itertools.combinations(ids,2)]
    cross=[(a,b) for (_,aa),(_,bb) in itertools.combinations(labels.items(),2) for a in aa for b in bb]
    report={'independent_acceptance':False,'same_pairs':len(same),'cross_pairs':len(cross),'arms':{}}
    for arm,data in result['arms'].items():
        owners=data['owners']
        def linked(pairs):
            return [[a,b] for a,b in pairs if owners[str(a)] is not None and owners[str(a)]==owners[str(b)]]
        report['arms'][arm]={'same_links':linked(same),'cross_links':linked(cross),
            'subjects':{name:[owners[str(i)] for i in ids] for name,ids in labels.items()}}
    baseline=set(map(tuple,report['arms']['control']['same_links']))
    for arm,data in report['arms'].items():
        if arm=='control':continue
        linked=set(map(tuple,data['same_links']))
        data['lost']=sorted(baseline-linked);data['gained']=sorted(linked-baseline)
        painting=set(itertools.combinations(labels['painting-behind-chairs'],2))
        data['visibility_development_pass']=not data['lost'] and not data['cross_links'] and bool((linked-baseline)&painting)
    report['owner_changes']=[{'id':i,**{a:d['owners'][str(i)] for a,d in result['arms'].items()}}
        for i in result.get('owner_changes',[])]
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(report,indent=2)+'\n')
    print({a:{'same_links':len(d['same_links']),'cross_links':len(d['cross_links']),
              'lost':len(d.get('lost',[])),'gained':len(d.get('gained',[])),
              'visibility_pass':d.get('visibility_development_pass')} for a,d in report['arms'].items()})
    return report


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--result',type=Path,required=True)
    p.add_argument('--labels',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();score(a.result,a.labels,a.output)
