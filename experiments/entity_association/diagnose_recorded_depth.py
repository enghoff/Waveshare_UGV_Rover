"""Reproduce recorded depth answers without altering the live rover's policy."""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline
from experiments.entity_association.audit_depth_abstention import load_depth, json_numpy


def run(recording, frames, output, allow_missing_depth=False):
    if output.exists():
        raise ValueError('choose a new output file')
    manifest = json.loads((recording/'manifest.json').read_text())
    assert manifest['complete'] and not manifest.get('error'), 'incomplete recorder'
    events = [json.loads(s) for s in (recording/'events.jsonl').read_text().splitlines()]
    projections = [e for e in events if e['kind'] == 'depth_projection']
    assert projections, 'no recorded projection arguments'
    rows, missing = [], []
    with sqlite3.connect((recording/'after.db').resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        for projection in projections:
            fid = projection['frame_id']
            if not (frames/(fid+'.depth.gz')).exists() and allow_missing_depth:
                missing.append({'frame_id':fid, 'sequence':projection['sequence'],
                    'observations':[{'id':r['id'], 'range_m':r['range_m']}
                        for r in db.execute('SELECT id,range_m FROM observations WHERE frame_id=?', (fid,))],
                    'reason':'Raw depth not retained; sampler answer cannot be checked.'})
                continue
            image, raw, header, _ = load_depth(frames/(fid+'.depth.gz'))
            with Image.open(frames/(fid+'.jpg')) as photo:
                size = photo.size
            assert header.get('taken_at') is None or abs(header['taken_at']-projection['depth_taken_at']) < .0001
            for row in db.execute('SELECT * FROM observations WHERE frame_id=?', (fid,)):
                bbox = json.loads(row['bbox_json'])
                indices = [i for i,b in enumerate(projection['regions'])
                           if len(b) == len(bbox) and all(abs(x-y)<1e-7 for x,y in zip(b,bbox))]
                assert len(indices) == 1, ('ambiguous projection region', row['id'])
                expected = projection['answers'][indices[0]]
                original = outline.surface
                samples = []

                def observed(module, lengths):
                    found = original(module, lengths)
                    if found is not None and len(lengths):
                        samples.append({'share':float(found[2]/len(lengths)),
                                        'gap_m':float(module.median(lengths)-found[0]),
                                        'valid_samples':len(lengths)})
                    return found

                outline.surface = observed
                try:
                    answer = outline.read(np, raw, image.width, image.height, image.lens,
                        [(bbox, row['outline_blob'])], size,
                        turn_deg=projection['turn_deg'], pan_deg=projection['pan_deg'],
                        tilt_deg=projection['tilt_deg'])[0]
                finally:
                    outline.surface = original
                assert answer == expected, ('sampler answer', row['id'], answer, expected)
                assert row['range_m'] == answer.get('range_m'), ('stored range', row['id'])
                band = samples[-1] if answer.get('method') == 'outline' else None
                flag = bool(band and band['share'] < .5 and band['gap_m'] > .5
                            and row['range_m'] is not None)
                rows.append({'id':row['id'], 'frame_id':fid, 'bbox':bbox,
                             'range_m':row['range_m'], 'method':answer.get('method'),
                             'band':band, 'flag':flag})
    result = {'all_kept_sampler_answers_exact':not missing,
              'all_available_sampler_answers_exact':True,
              'missing_depth':missing, 'rows':rows,
              'flagged_ids':[r['id'] for r in rows if r['flag']],
              'independent_acceptance':False,
              'limit':'Reproduced sampler answers do not supply independent distance or identity truth.'}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, default=json_numpy)+'\n')
    print(f'{len(rows)} stored sampler answers exact; {sum(r["range_m"] is not None for r in rows)} ranges; {len(result["flagged_ids"])} flagged; {len(missing)} depth frames missing.')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--frames', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-missing-depth', action='store_true',
                        help='Report missing frames explicitly and check only retained depth; no complete-proof claim.')
    args = parser.parse_args()
    run(args.recording, args.frames, args.output, args.allow_missing_depth)
