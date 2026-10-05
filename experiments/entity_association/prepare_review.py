"""Make a blinded pilot label pack from available frames, without old entity IDs.

This is the development recording again, not an independent acceptance drive.
Missing source frames are reported rather than replaced by an entity crop sheet.
"""
import argparse
import csv
import json
from pathlib import Path
import random
import sqlite3
import hashlib
from contextlib import closing

from PIL import Image, ImageDraw

from audit import Evidence, ROOT


def prepare(output, frames=12, seed=104, database=None, frames_dir=None, after_observation=0):
    pilot = database is None
    if pilot:
        observations = Evidence().rows
        image_root = ROOT / "captures"
    else:
        if frames_dir is None:
            raise ValueError("a supplied recording needs --frames-dir")
        with closing(sqlite3.connect(database.resolve().as_uri()+"?mode=ro", uri=True)) as db:
            db.row_factory = sqlite3.Row
            observations = {r["id"]: dict(r) for r in db.execute(
                "SELECT id,frame_id,bbox_json,bearing_deg FROM observations WHERE id>? ORDER BY id",
                (after_observation,))}
        image_root = frames_dir
    output.mkdir(parents=True, exist_ok=True)
    available = {p.stem: p for p in image_root.rglob("*.jpg")
                 if p.stem in {r["frame_id"] for r in observations.values()}}
    by_frame = {}
    for row in observations.values():
        if row["frame_id"] in available:
            by_frame.setdefault(row["frame_id"], []).append(row)
    chosen = random.Random(seed).sample(sorted(by_frame), min(frames, len(by_frame)))
    # The disputed frame is a separate review item and not part of random sampling.
    disputed = "20261003-091409-2e7688"
    entries = []
    html = ["<!doctype html><meta charset='utf-8'><title>Observation label review</title>",
            "<style>body{font:17px system-ui;max-width:1000px;margin:30px auto}img{max-width:100%}</style>",
            "<h1>Observation label review</h1>",
            ("<p>This is a pilot on the development recording. It cannot establish independent acceptance.</p>"
             if pilot else "<p>New recording, not yet labelled or accepted. Current entity assignments are hidden. "
             "No candidate result is shown. Review all regions, including unclear and unassigned ones.</p>"),
            "<p>Enter a stable physical-object name in labels.csv for each numbered region. Use the same name "
            "across pictures only when you can tell it is the same physical object. Mark unclear, mixed, "
            "surface, glare, or indistinguishable chair when appropriate. A region can show part of an "
            "object; two regions of one picture are not automatically different identities.</p>"]
    for record,title in ((("332","Seated person"),("385","Head of that person")) if pilot else ()):
        grid = ROOT/"captures/2026-10-03-merge-review/grids"/f"object_{record}.jpg"
        if grid.exists():
            name=f"part-whole-{record}.jpg"
            Image.open(grid).save(output/name,quality=94)
            html.append(f"<h2>Identity policy: {title}</h2><img src='{name}' alt='{title}'>")
    if pilot:
        html.append("<p>The old evaluation treats these two records as different objects. "
                "They may instead be parts/views of one person. No verdict is prefilled; "
                "the replay reports both interpretations. These policy examples are separate "
                "from the random pilot frames below.</p>")
    else:
        html.append("<p>Working policy: head/body views of one person use one physical identity. "
                    "Use unclear when you cannot distinguish individual similar chairs. "
                    "A blank identity is not evidence that two regions are different objects. "
                    "Mark an unambiguous object as object in verdict; use mixed, surface, glare or unclear otherwise.</p>")
    review_frames = [disputed] + [f for f in chosen if f != disputed] if pilot else sorted(chosen)
    for number, frame in enumerate(review_frames, 1):
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
        html.append(f"<h2>{'Disputed label' if pilot and frame == disputed else 'Review frame'} {number}</h2>"
                    f"<p>{frame}</p><img src='{name}' alt='Numbered observation regions'>")
    (output/"review.html").write_text("\n".join(html), encoding="utf-8")
    with (output/"labels.csv").open("w",newline="",encoding="utf-8") as f:
        writer = csv.DictWriter(f,fieldnames=["observation_id","frame_id","bbox","physical_object","verdict","note"])
        writer.writeheader()
        writer.writerows(entries)
    manifest = {"seed":seed,"available_frames":len(by_frame),
        "sample_frames":chosen,"observations":len(entries),"independent_acceptance":False,
        "development_pilot":pilot, "after_observation_id":after_observation,
        "source_observations":len(observations),
        "excluded_missing_frames":len({r['frame_id'] for r in observations.values()})-len(by_frame)}
    if database:
        manifest.update(database_sha256=hashlib.sha256(database.read_bytes()).hexdigest(),
                        regions_without_bearing=sum(r["bearing_deg"] is None for r in observations.values()))
    (output/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(output/"review.html", len(entries), "regions", flush=True)


if __name__ == "__main__":
    parser=argparse.ArgumentParser()
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--frames",type=int,default=12)
    parser.add_argument("--database",type=Path)
    parser.add_argument("--frames-dir",type=Path)
    parser.add_argument("--after-observation",type=int,default=0)
    args=parser.parse_args()
    prepare(args.output,args.frames,database=args.database,frames_dir=args.frames_dir,
            after_observation=args.after_observation)
