"""Offline fixed minority-surface abstention against archived taped targets."""
from __future__ import annotations

import argparse
import gzip
import json
import math
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import depth_client, oak, outline, perceive
from experiments.entity_association.diagnose_visual_evidence import depth_samples, digest


def iou(a, b):
    area = max(0, min(a[2], b[2]) - max(a[0], b[0])) * max(
        0, min(a[3], b[3]) - max(a[1], b[1]))
    union = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - area
    return float(area / union) if union else 0.0


def json_numpy(value):
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'unsupported diagnostic value: {type(value).__name__}')


def trace(image, blob, size, guess=None):
    """Trace the two production projection iterations without changing their statistic."""
    decoded = outline.decode(np, blob)
    if decoded is None:
        return None
    x, y, stride, piece = decoded
    rr, cc = np.nonzero(piece)
    grid = outline.directions(np, size)
    dirs = grid[np.clip((y+rr*stride)//outline.STRIDE, 0, grid.shape[0]-1),
                np.clip((x+cc*stride)//outline.STRIDE, 0, grid.shape[1]-1)]
    dirs = dirs[~np.isnan(dirs).any(axis=1)]
    if len(dirs) < outline.RANGE_MIN_PIXELS:
        return None
    middle = dirs.mean(axis=0)
    middle = tuple(middle / (np.linalg.norm(middle) or 1.0))
    assumed = float(guess or oak.GUESS_RANGE_M)
    for _ in range(2):
        samples = depth_samples(image, blob, size, assumed)
        found = outline.surface(np, samples)
        if found is None:
            return None
        ranged = oak.range_from_gimbal([middle], found[0])
        if ranged is None or ranged <= 0:
            return None
        assumed = ranged
    median = float(np.median(samples))
    share = found[2] / len(samples)
    gap = median - found[0]
    return {'range_m': round(float(ranged), 3), 'band_share': share,
            'median_gap_m': gap, 'valid_samples': len(samples),
            'quantiles_m': np.quantile(samples, [.05, .2, .5, .8, .95]).tolist(),
            'abstain': bool(share < .5 and gap > .5)}


def load_depth(path):
    body = gzip.decompress(path.read_bytes())
    n = int.from_bytes(body[:4], 'little')
    header = json.loads(body[4:4+n])
    inferred = 'lens' not in header
    lens = depth_client.Lens(**(header.get('lens') or
        dict(fx=500.3, fy=500.17, cx=321.23, cy=190.69, width=640, height=360)))
    image = outline.DepthImage(np, body[4+n:], header['width'], header['height'], lens)
    return image, body[4+n:], header, inferred


def frozen_tape():
    """Load the historical mapping/geometry definitions, without its scoring side effects."""
    path = ROOT / 'captures/m0-2026-10-02/residuals.py'
    text = path.read_text().split('looks, places = [], []')[0]
    definitions = {}
    exec(compile(text, str(path), 'exec'), definitions)
    return definitions, path


def stats(rows):
    ranged = [r for r in rows if r['control_m'] is not None]
    good = [r for r in ranged if abs(r['control_m']-r['want_m']) <= .25]
    bad = [r for r in ranged if abs(r['control_m']-r['want_m']) >= 1]
    removed = [r for r in ranged if r['abstain']]
    return {'looks': len(rows), 'control_ranged': len(ranged),
            'control_correct_025': len(good), 'control_gross_1m': len(bad),
            'abstained': len(removed), 'lost_correct_025': sum(r['abstain'] for r in good),
            'removed_gross_1m': sum(r['abstain'] for r in bad),
            'correct_retention': 1-sum(r['abstain'] for r in good)/len(good) if good else None,
            'gross_removal': sum(r['abstain'] for r in bad)/len(bad) if bad else None,
            'lost_correct_ids': [[r['drive'], r['id']] for r in good if r['abstain']]}


def run(model, fresh, output):
    if output.exists():
        raise ValueError('choose a new output directory')
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model), sess_options=options,
                                  providers=['CPUExecutionProvider'])
    class Models:
        def regions(self, blob):
            outputs = session.run(None, {'images': blob})
            return outputs[0], outputs[1]
    finder = perceive.Perception(str(model.parent))
    finder._np, finder._cv2, finder._models = np, cv2, Models()
    definitions, tape_path = frozen_tape()
    hashes = {str(tape_path.relative_to(ROOT)): digest(tape_path), 'model': digest(model)}
    for source in [Path(__file__), ROOT/'world_state/outline.py', ROOT/'world_state/oak.py',
                   ROOT/'world_state/view.py', ROOT/'world_state/perceive.py',
                   ROOT/'experiments/entity_association/diagnose_visual_evidence.py']:
        hashes[str(source.relative_to(ROOT))] = digest(source)
    records, archived = [], []
    names = {'2026-10-01': ('10-01', 'frames'),
             '2026-10-02 first': ('10-02-first', 'frames'),
             '2026-10-02 redo': ('10-02-redo', 'frames-redo')}
    for name, config in definitions['DRIVES'].items():
        database = ROOT / ('captures/' + config['db'].split('captures/')[1])
        tag, frame_dir = names[name]
        frames = database.parent / frame_dir
        cached_path = ROOT / f'captures/2026-10-02-single-look-rules/ranges-{tag}-raw.json'
        cached = json.loads(cached_path.read_text())
        hashes[str(database.relative_to(ROOT))] = digest(database)
        hashes[str(cached_path.relative_to(ROOT))] = digest(cached_path)
        w2m = definitions['frame'](config['parks'])
        by_frame = {}
        with sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True) as db:
            db.row_factory = sqlite3.Row
            for target, entities in config['targets'].items():
                for entity in entities:
                    for row in db.execute('SELECT * FROM observations WHERE entity_id=?', (entity,)):
                        row = dict(row)
                        if row['observer_pose_json'] and str(row['id']) in cached:
                            by_frame.setdefault(row['frame_id'], []).append((target, row))
        for fid, target_rows in sorted(by_frame.items()):
            photo, depth_path = frames/(fid+'.jpg'), frames/(fid+'.depth.gz')
            if not photo.exists() or not depth_path.exists():
                raise ValueError(f'missing archived target frame {fid}')
            for p in [photo, depth_path]:
                hashes[str(p.relative_to(ROOT))] = digest(p)
            image, raw, header, inferred = load_depth(depth_path)
            rgb = cv2.imread(str(photo))
            size = (rgb.shape[1], rgb.shape[0])
            boxes, _, masks, _ = finder._regions(rgb)
            for target, row in target_rows:
                bbox = json.loads(row['bbox_json'])
                best = max(range(len(boxes)), key=lambda j: iou(bbox, boxes[j])) if len(boxes) else None
                overlap = iou(bbox, boxes[best]) if best is not None else 0
                blob = (outline.encode(np, masks.of(best), bbox)
                        if best is not None and overlap >= .5 and masks is not None else None)
                replay = outline.read(np, raw, image.width, image.height, image.lens,
                                      [(bbox, blob)], size)[0]
                boxed = outline.box_range(image, bbox, size)
                traced = (trace(image, blob, size, boxed.get('range_m'))
                          if replay.get('method') == 'outline' else None)
                if replay.get('method') == 'outline':
                    assert traced and traced['range_m'] == replay['range_m'], (row['id'], traced, replay)
                pose = json.loads(row['observer_pose_json'])
                heading = math.radians(pose['heading_deg'])
                cx = pose['x_m'] + .02*math.cos(heading)
                cy = pose['y_m'] + .02*math.sin(heading)
                tz = definitions['TRUTH'][target][2]
                rx, ry = w2m(*definitions['RANGE_AT'].get(target, definitions['TRUTH'][target][:2]))
                want = math.hypot(math.hypot(rx-cx, ry-cy)-definitions['FRONT'][target],
                                  tz-definitions['CAM_H'] if tz is not None else 0)
                old = cached[str(row['id'])]['hybrid']
                control = float(replay['range_m']) if replay.get('range_m') is not None else None
                record = {'drive': name, 'target': target, 'id': row['id'], 'frame_id': fid,
                          'want_m': want, 'control_m': control,
                          'historical_control_m': old, 'replay': replay,
                          'trace': traced, 'abstain': bool(traced and traced['abstain']),
                          'mask_iou': overlap, 'mask_regenerated': blob is not None,
                          'inferred_old_lens': inferred, 'stored_range_m': row['range_m'],
                          'historical_control_delta_m': control-old if control is not None and old is not None else None}
                records.append(record)
                archived.append(dict(record, control_m=old, abstain=False))
            print(name, fid, len(target_rows), 'target regions', flush=True)
        assert digest(database) == hashes[str(database.relative_to(ROOT))]
    new_db = fresh/'after.db'
    hashes['fresh/after.db'] = digest(new_db)
    with sqlite3.connect(new_db.resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        row = dict(db.execute('SELECT * FROM observations WHERE id=68640').fetchone())
    depth_path = fresh/'frames'/(row['frame_id']+'.depth.gz')
    photo = depth_path.with_suffix('').with_suffix('.jpg')
    hashes['fresh/depth'] = digest(depth_path)
    hashes['fresh/photo'] = digest(photo)
    image, raw, header, inferred = load_depth(depth_path)
    with Image.open(photo) as im:
        size = im.size
    known_box = outline.box_range(image, json.loads(row['bbox_json']), size)
    known = trace(image, row['outline_blob'], size, known_box.get('range_m'))
    assert known['range_m'] == row['range_m']
    summary = stats(records)
    faithful = [r for r in records if r['historical_control_delta_m'] is not None]
    summary.update({'known_failure_abstains': known['abstain'],
                    'pass_retention': summary['correct_retention'] is not None and summary['correct_retention'] >= .9,
                    'pass_gross_removal': summary['gross_removal'] is not None and summary['gross_removal'] >= .5,
                    'old_replay_comparable_ranges': len(faithful),
                    'old_replay_within_002m': sum(abs(r['historical_control_delta_m']) <= .02 for r in faithful),
                    'mask_matches_below_05': sum(r['mask_iou'] < .5 for r in records),
                    'historical_baseline': stats(archived)})
    result = {'summary': summary, 'known_failure': known, 'rows': records,
              'by_drive': {name: stats([r for r in records if r['drive'] == name]) for name in names},
              'by_target': {name: stats([r for r in records if r['target'] == name]) for name in definitions['TRUTH']},
              'input_sha256': hashes, 'model_path': str(model), 'runtime': ort.__version__,
              'limits': ['Reused development tape and target mapping, not independent acceptance.',
                        'Older masks regenerated with CPU ONNX; recorded boxes preserved.',
                        'Older depth replay uses zero shutter turn, matching its historical replay convention.',
                        'Some older headers lack a lens; documented historical calibration assumed.',
                        'Range retention only; no identity or placement replay.',
                        'Transparent-table examples require separate visual review.']}
    text = json.dumps(result, indent=2, default=json_numpy)+'\n'
    output.mkdir(parents=True)
    (output/'result.json').write_text(text)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', type=Path, required=True)
    p.add_argument('--fresh', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    run(a.model, a.fresh, a.output)
