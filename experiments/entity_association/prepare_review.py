"""Make a blinded pilot label pack from available frames, without old entity IDs.

This is the development recording again, not an independent acceptance drive.
Missing source frames are reported rather than replaced by an entity crop sheet.
"""
import argparse
import csv
import json
from pathlib import Path
import random

from PIL import Image, ImageDraw

from audit import Evidence, ROOT


def prepare(output, frames=12, seed=104):
    evidence = Evidence()
    output.mkdir(parents=True, exist_ok=True)
    available = {p.stem: p for p in (ROOT / "captures").rglob("*.jpg")
                 if p.stem in {r["frame_id"] for r in evidence.rows.values()}}
    by_frame = {}
    for row in evidence.rows.values():
        if row["frame_id"] in available:
            by_frame.setdefault(row["frame_id"], []).append(row)
    chosen = random.Random(seed).sample(sorted(by_frame), min(frames, len(by_frame)))
    # The disputed frame is a separate review item and not part of random sampling.
    disputed = "20261003-091409-2e7688"
    entries = []
    html = ["<!doctype html><meta charset='utf-8'><title>Observation label review</title>",
            "<style>body{font:17px system-ui;max-width:1000px;margin:30px auto}img{max-width:100%}</style>",
            "<h1>Observation label review</h1>",
            "<p>This is a pilot on the development recording. It cannot establish independent acceptance.</p>",
            "<p>Enter a stable physical-object name in labels.csv for each numbered region. Use the same name "
            "across pictures only when you can tell it is the same physical object. Mark unclear, mixed, "
            "surface, glare, or indistinguishable chair when appropriate. A region can show part of an "
            "object; two regions of one picture are not automatically different identities.</p>"]
    for record,title in (("332","Seated person"),("385","Head of that person")):
        grid = ROOT/"captures/2026-10-03-merge-review/grids"/f"object_{record}.jpg"
        if grid.exists():
            name=f"part-whole-{record}.jpg"
            Image.open(grid).save(output/name,quality=94)
            html.append(f"<h2>Identity policy: {title}</h2><img src='{name}' alt='{title}'>")
    html.append("<p>The old evaluation treats these two records as different objects. "
                "They may instead be parts/views of one person. No verdict is prefilled; "
                "the replay reports both interpretations. These policy examples are separate "
                "from the random pilot frames below.</p>")
    for number, frame in enumerate([disputed] + [f for f in chosen if f != disputed], 1):
        rows = by_frame.get(frame, [])
        if not rows:
            continue
        image = Image.open(available[frame]).convert("RGB")
        draw = ImageDraw.Draw(image)
        for row in rows:
            x0,y0,x1,y1 = json.loads(row["bbox_json"])
            box = (round(x0*image.width),round(y0*image.height),round(x1*image.width),round(y1*image.height))
            color = "red" if row["id"] == 61656 else "cyan" if row["id"] == 61654 else "yellow"
            draw.rectangle(box, outline=color, width=2)
            label = str(row["id"])
            draw.rectangle((box[0],box[1],min(image.width,box[0]+len(label)*7+4),box[1]+13), fill="black")
            draw.text((box[0]+1,box[1]), label, fill=color)
            entries.append({"observation_id": row["id"], "frame_id": frame,
                            "bbox": row["bbox_json"], "physical_object": "", "verdict": "", "note": ""})
        name = f"frame-{number:02d}.jpg"
        image.save(output/name, quality=94)
        html.append(f"<h2>{'Disputed label' if frame == disputed else 'Pilot frame'} {number}</h2>"
                    f"<p>{frame}</p><img src='{name}' alt='Numbered observation regions'>")
    (output/"review.html").write_text("\n".join(html), encoding="utf-8")
    with (output/"labels.csv").open("w",newline="",encoding="utf-8") as f:
        writer = csv.DictWriter(f,fieldnames=list(entries[0]))
        writer.writeheader()
        writer.writerows(entries)
    (output/"manifest.json").write_text(json.dumps({"seed":seed,"available_frames":len(by_frame),
        "sample_frames":chosen,"observations":len(entries),"independent_acceptance":False,
        "excluded_missing_frames":len({r['frame_id'] for r in evidence.rows.values()})-len(by_frame)},indent=2)+"\n")
    print(output/"review.html", len(entries), "regions", flush=True)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--frames",type=int,default=12)
    args=parser.parse_args()
    prepare(args.output,args.frames)
