"""Score observation memberships; never transfer a mixed entity verdict to a split."""
import argparse
from itertools import combinations, product
import json
from pathlib import Path


def score(result,labels,constraints):
    membership=result["membership"]
    expected={str(oid) for row in labels["things"].values() for oid in row["looks"]}
    if not expected.issubset(membership):
        raise ValueError("result does not contain the labelled observation IDs; this is not the labelled recording")
    def owner(oid):
        return membership.get(str(oid))
    same=retained=unresolved=split=0
    clean_whole=0
    for row in labels["things"].values():
        if row["verdict"]!="object":
            continue
        owners=[owner(oid) for oid in row["looks"]]
        clean_whole+=bool(owners) and None not in owners and len(set(owners))==1
        for a,b in combinations(row["looks"],2):
            same+=1
            if owner(a) is None or owner(b) is None:
                unresolved+=1
            elif owner(a)==owner(b):
                retained+=1
            else:
                split+=1
    cases=[]
    for case in constraints["cases"]:
        wrong=separate=pending=0
        for left,right in combinations(case["groups"],2):
            for a,b in product(left,right):
                if owner(a) is None or owner(b) is None:
                    pending+=1
                elif owner(a)==owner(b):
                    wrong+=1
                else:
                    separate+=1
        cases.append({"name":case["name"],"wrong_pairs":wrong,"separate_pairs":separate,"unresolved_pairs":pending,
                      "status":"mixed" if wrong else "unresolved" if pending else "separated"})
    return {"clean_pairs":same,"clean_pairs_retained":retained,"clean_pairs_split":split,
            "clean_pairs_unresolved":unresolved,"clean_retention_pct":100*retained/max(1,same),
            "clean_entities_whole":clean_whole,"clean_entities_total":sum(r["verdict"]=="object" for r in labels["things"].values()),
            "challenge_cases":cases,"cases_mixed":sum(c["status"]=="mixed" for c in cases),
            "cases_separated":sum(c["status"]=="separated" for c in cases),
            "cases_unresolved":sum(c["status"]=="unresolved" for c in cases)}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("results",nargs="+")
    args=parser.parse_args()
    here=Path(__file__).resolve().parent
    labels=json.loads((here/"labels/m0-2026-09-08.json").read_text())
    constraints=json.loads((here/"labels/incremental-2026-09-10.json").read_text())
    for filename in args.results:
        result=json.loads(Path(filename).read_text())
        result["quality"]=score(result,labels,constraints)
        Path(filename).write_text(json.dumps(result,indent=2),encoding="utf-8")
        print(filename,json.dumps(result["quality"],indent=2))


if __name__=="__main__":
    main()
