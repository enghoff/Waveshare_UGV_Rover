"""Build a compact, reproducible comparison and shareable plots from benchmark JSON."""
import argparse
import csv
import json
from pathlib import Path
import statistics

if __package__ in (None, ""):
    import sys
    sys.path.insert(0,str(Path(__file__).resolve().parent.parent))
    __package__="world_state"

from .score_incremental import score


def report(directory):
    directory=Path(directory)
    here=Path(__file__).resolve().parent
    labels=json.loads((here/"labels/m0-2026-09-08.json").read_text())
    constraints=json.loads((here/"labels/incremental-2026-09-10.json").read_text())
    rows=[]
    detailed={}
    for tag in ("07","08","acceptance"):
        for mode in ("baseline","shortlist","revision","negative"):
            path=directory/(mode+"-"+tag+".json")
            if not path.exists():
                continue
            result=json.loads(path.read_text())
            row={"drive":tag,"mode":mode,"complete":result["complete"],
                 "frames":result["frames"],"observations":result["observations"],
                 "seconds":round(result["seconds"],3),"p95_ms":round(result["p95_ms"],2),
                 "entities":len(result["entities"]),"placed_entities":sum(e["placement"] is not None for e in result["entities"].values()),
                 "attached":result["attached"],"pair_crossings":result["work"].get("fix",0),
                 "geometry_checks":result["work"].get("_allowance_used",0),
                 "revision_queue_remaining":result["engine"].get("revision_queue_remaining",0),
                 "negative_challenges":result["engine"].get("negative_challenges",0)}
            if tag=="08":
                quality=score(result,labels,constraints)
                result["quality"]=quality
                path.write_text(json.dumps(result,indent=2),encoding="utf-8")
                row.update({key:quality[key] for key in ("clean_retention_pct","clean_entities_whole","cases_mixed","cases_separated","cases_unresolved")})
                detailed[mode]=quality
            rows.append(row)
    growth=[]
    for mode in ("baseline","negative"):
        path=directory/(mode+"-growth.json")
        if not path.exists():
            continue
        result=json.loads(path.read_text())
        block=result["intended_frames"]//result["repeat"]
        for index in range(result["repeat"]):
            samples=result["trace"][index*block:(index+1)*block]
            if len(samples)!=block:
                continue
            times=[r["ms"] for r in samples]
            growth.append({"mode":mode,"pass":index+1,"cumulative_observations":samples[-1]["observations"],
                           "seconds":sum(times)/1000,"median_ms":statistics.median(times),
                           "p95_ms":sorted(times)[int(.95*(len(times)-1))]})
    base=detailed.get("baseline",{})
    candidate=detailed.get("negative",{})
    acceptable=(bool(base) and bool(candidate)
                and candidate["clean_retention_pct"]>=base["clean_retention_pct"]
                and candidate["cases_mixed"]<base["cases_mixed"]
                and candidate["cases_unresolved"]<=base["cases_unresolved"])
    decision=("Development candidate only: independent acceptance and host verification remain required."
              if acceptable else "Keep the production resolver: this comparison does not establish an acceptable identity improvement.")
    data={"runs":rows,"quality":detailed,"growth":growth,
          "quality_scope":"8 visually reviewed cannot-link cases; retention of same-object pairs within 52 previously labelled clean entities. Not an overall error rate.",
          "decision":decision}
    (directory/"comparison.json").write_text(json.dumps(data,indent=2),encoding="utf-8")
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with (directory/"comparison.csv").open("w",newline="",encoding="utf-8") as f:
        writer=csv.DictWriter(f,fieldnames=fields);writer.writeheader();writer.writerows(rows)
    plot(directory,rows,growth)
    print(json.dumps(data,indent=2))
    return data


def plot(directory,rows,growth):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import numpy as np
    selected=[r for r in rows if r["drive"]=="08"]
    mode_names={"baseline":"Current","shortlist":"Bounded search","revision":"+ Revision","negative":"+ Negative evidence"}
    names=[mode_names[r["mode"]] for r in selected]
    colors=["#34495e","#2496a8","#df8d32","#b8504e"]
    fig,axes=plt.subplots(1,3,figsize=(13,4.8))
    x=np.arange(len(selected))
    values=[[r["seconds"] for r in selected],
            [r["clean_retention_pct"] for r in selected],
            [r["cases_mixed"] for r in selected]]
    titles=["Replay time · lower is better","Correct pairs kept · higher is better","Known mix-ups remaining · lower is better"]
    labels=["seconds","percent of labelled same-object pairs","cases out of 8"]
    for ax,vals,title,label in zip(axes,values,titles,labels):
        ax.bar(x,vals,color=colors,width=.65)
        ax.set_xticks(x,names,rotation=25,ha="right")
        ax.set_title(title,fontsize=11,pad=15)
        ax.set_ylabel(label)
        ax.spines[["top","right"]].set_visible(False)
        ax.set_ylim(0,max(vals)*1.2)
        for i,val in enumerate(vals):ax.text(i,val+max(vals)*.03,f"{val:.1f}",ha="center",fontsize=10)
    fig.suptitle("Bounded entity fitting: runtime and identity",fontsize=15,y=.99)
    fig.text(.02,.025,"September 8 drive · 1,328 observations · workstation timings · each experimental mode leaves one reviewed case unresolved.\nQuality is a development sample; other drives have no comparable observation-level labels.",fontsize=9,color="#555555")
    fig.tight_layout(rect=(0,.12,1,.94))
    fig.savefig(directory/"comparison.png",dpi=180)
    fig.savefig(directory/"comparison.svg")
    plt.close(fig)
    if growth:
        fig,axes=plt.subplots(1,2,figsize=(10,4.5))
        for mode,color in (("baseline",colors[0]),("negative",colors[3])):
            group=[r for r in growth if r["mode"]==mode]
            for ax,metric,title in zip(axes,("seconds","p95_ms"),("Time per repeated drive (seconds)","95th percentile per frame (ms)")):
                ax.plot([r["cumulative_observations"] for r in group],[r[metric] for r in group],"o-",color=color,label="Current" if mode=="baseline" else "Bounded + revision + negative")
                ax.set_title(title,fontsize=11)
                ax.set_xlabel("Cumulative observations")
                ax.set_ylim(bottom=0)
                ax.spines[["top","right"]].set_visible(False)
        axes[0].legend(fontsize=9)
        fig.suptitle("Growth test: the same drive fed through twice",fontsize=14)
        fig.text(.03,.015,"A workload test, not independent accuracy evidence or proof of asymptotic complexity.",fontsize=9)
        fig.tight_layout(rect=(0,.06,1,.92))
        fig.savefig(directory/"growth.png",dpi=180)
        plt.close(fig)


if __name__=="__main__":
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory")
    report(parser.parse_args().directory)
