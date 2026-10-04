"""Render an analyst draft and grouped doubts using raw crops and saved outlines.

This does not label automatically, score a candidate or promote drafts to truth.
The reviewer can answer a few repeated issues without editing hundreds of rows.
"""
import argparse
from contextlib import closing
import hashlib
import html
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline


def prepare(draft, questions, database, frames_dir, output):
    document = json.loads(draft.read_text(encoding='utf-8'))
    issues = json.loads(questions.read_text(encoding='utf-8'))
    if hashlib.sha256(database.read_bytes()).hexdigest() != document['source_database_sha256']:
        raise ValueError('draft does not describe this database snapshot')
    if document.get('independent_acceptance') or document.get('confirmed_by_owner'):
        raise ValueError('this renderer requires an unconfirmed analyst draft')
    labels = {int(r['observation_id']): r for r in document['labels']}
    if len(labels) != len(document['labels']):
        raise ValueError('duplicate observation IDs in draft')
    with closing(sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True)) as db:
        db.row_factory = sqlite3.Row
        # Deliberately do not fetch entity assignments or model/candidate scores.
        regions = {r['id']: dict(r) for r in db.execute(
            'SELECT id,frame_id,bbox_json,outline_blob FROM observations') if r['id'] in labels}
    if set(regions) != set(labels):
        raise ValueError('draft contains observations absent from the snapshot')
    for i, label in labels.items():
        if label['frame_id'] != regions[i]['frame_id'] or json.loads(label['bbox']) != json.loads(regions[i]['bbox_json']):
            raise ValueError(f'draft changed source provenance for observation {i}')
    issue_ids = {q['id'] for q in issues}
    if len(issue_ids) != len(issues) or any(r['review_group'] and r['review_group'] not in issue_ids for r in labels.values()):
        raise ValueError('review groups must each have exactly one question')
    output.parent.mkdir(parents=True, exist_ok=True)
    crops = output.parent / 'first-pass-crops'
    crops.mkdir(exist_ok=True)
    escape = html.escape
    parts = ["<!doctype html><html lang='en'><meta charset='utf-8'>",
             "<meta name='viewport' content='width=device-width,initial-scale=1'>",
             "<title>First-pass identity review</title>",
             "<style>body{font:17px system-ui;max-width:1050px;margin:24px auto;padding:0 14px;color:#18212b}"
             "section{border-top:2px solid #ddd;margin-top:30px;padding-top:15px}"
             ".examples{display:grid;grid-template-columns:repeat(auto-fit,minmax(320px,1fr));gap:18px}"
             "figure{margin:0;padding:12px;background:#f4f5f6;border-radius:7px}"
             ".pair{display:flex;gap:5px;align-items:center;justify-content:center;min-height:180px}"
             ".pair img{max-width:48%;max-height:220px;object-fit:contain}"
             "textarea{box-sizing:border-box;width:100%;min-height:85px;font:inherit;padding:10px}"
             "button{font:inherit;padding:10px 16px;cursor:pointer}small{color:#46505c}</style>",
             f"<h1>First-pass identity labels: {len(issues)} review questions</h1>",
             f"<p>All {len(labels)} regions have draft judgments. "
             f"{document['review_status']['draft_clear']} appear clear; "
             f"{document['review_status']['needs_review']} need review, grouped below.</p>",
             "<p>These are coding-agent proposals, not independent ground truth. Original labels remain blank. "
             "Current entity assignments and candidate results were not consulted. "
             "Answers to doubts alone do not establish independent acceptance.</p>",
             "<p>For each example, the left image is the raw region and the right shows only the saved selected pixels. "
             "Grey pixels on the right were outside the detector's outline. Open the full frame for context.</p>",
             "<p><a href='labels-first-pass.csv'>Download all draft labels</a> · "
             "<a href='review.html'>All 27 numbered frames</a></p>"]
    for question in issues:
        affected = sorted(i for i, r in labels.items() if r['review_group'] == question['id'])
        parts.extend([f"<section><h2>{escape(question['title'])}</h2>",
                      f"<p>{escape(question['question'])}</p><p><small>{len(affected)} affected regions.</small></p>",
                      "<div class='examples'>"])
        for i in question['examples']:
            label, region = labels[i], regions[i]
            with Image.open(frames_dir/(region['frame_id']+'.jpg')) as opened:
                source = opened.convert('RGB')
            window = outline.window(json.loads(region['bbox_json']), source.size)
            raw_name, mask_name = f'{i}-raw.jpg', f'{i}-outline.jpg'
            source.crop(window).save(crops/raw_name, quality=96)
            decoded = outline.decode(np, region['outline_blob'])
            if decoded:
                x, y, stride, piece = decoded
                mask = Image.new('L', source.size)
                mask.paste(Image.fromarray((piece*255).astype('uint8')).resize(
                    (piece.shape[1]*stride, piece.shape[0]*stride), Image.Resampling.NEAREST), (x,y))
                selected = Image.composite(source, Image.new('RGB', source.size, '#444'), mask)
                selected.crop(window).save(crops/mask_name, quality=96)
            caption = f"{i}: {label['physical_object'] or label['verdict']} ({label['confidence']})"
            parts.extend([f"<figure><figcaption>{escape(caption)}</figcaption><div class='pair'>",
                          f"<img src='first-pass-crops/{raw_name}' alt='Raw region {i}'>",
                          f"<img src='first-pass-crops/{mask_name}' alt='Selected pixels {i}'>" if decoded
                          else "<small>No saved outline available.</small>",
                          f"</div><p><small>{escape(label['note'])}</small></p>",
                          f"<a href='frame-{int(label['frame_number']):02d}.jpg'>Full numbered frame</a></figure>"])
        parts.extend(["</div>", f"<p>{escape(question['answer_hint'])}</p>",
                      f"<textarea data-question='{escape(question['id'],quote=True)}' "
                      f"aria-label='Answer to {escape(question['title'],quote=True)}'></textarea>",
                      "<details><summary>All affected observation IDs</summary><p>"+
                      ', '.join(str(i) for i in affected)+"</p></details></section>"])
    parts.extend(["<p>You can reply in chat with the question numbers, or enter answers here and download them.</p>",
                  "<button id='download'>Download review answers</button><p id='saved' role='status'></p>",
                  "<script>const fields=[...document.querySelectorAll('textarea')];"
                  "document.getElementById('download').onclick=()=>{const answers={};"
                  "for(const f of fields)answers[f.dataset.question]=f.value;"
                  "const blob=new Blob([JSON.stringify({reviewer_answers:answers},null,2)],{type:'application/json'});"
                  "const url=URL.createObjectURL(blob);const a=document.createElement('a');a.href=url;"
                  "a.download='identity-review-answers.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);"
                  "document.getElementById('saved').textContent='Answers downloaded. Draft labels have not been changed.';};"
                  "</script></html>"])
    output.write_text('\n'.join(parts), encoding='utf-8')
    print(output)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    for name in ('draft','questions','database','frames-dir','output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    prepare(args.draft, args.questions, args.database, args.frames_dir, args.output)
