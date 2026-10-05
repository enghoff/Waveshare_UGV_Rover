"""Diagnose frozen physical subjects using saved vectors and depth; never resolve."""
from __future__ import annotations
import argparse
import gzip
import hashlib
import itertools
import json
from pathlib import Path
import sqlite3
import sys

import numpy as np
from PIL import Image

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import depth_client, locate, outline, resolve
from experiments.entity_association.repair_geometry import unit


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def cosine(first, second, column):
    a, b = unit(first.get(column)), unit(second.get(column))
    if a is None or b is None or a.shape != b.shape:
        return None
    if first['vectors_from'] != second['vectors_from']:
        return None
    return float(a @ b)


def depth_samples(image, blob, size, distance):
    """The production projection at its final assumed distance, before band selection."""
    x0, y0, stride, piece = outline.decode(np, blob)
    rows, columns = np.nonzero(piece)
    grid = outline.directions(np, size)
    rr = np.clip((y0 + rows * stride) // outline.STRIDE, 0, grid.shape[0] - 1)
    cc = np.clip((x0 + columns * stride) // outline.STRIDE, 0, grid.shape[1] - 1)
    pointing = grid[rr, cc]
    pointing = pointing[~np.isnan(pointing).any(axis=1)]
    matrix, offset = outline.into_oak(np)
    points = (pointing * distance - offset) @ matrix.T
    points = points[points[:, 2] > 1e-6]
    lens = image.lens
    u = (lens.cx + lens.fx * points[:, 0] / points[:, 2]) * image.sx
    v = (lens.cy + lens.fy * points[:, 1] / points[:, 2]) * image.sy
    inside = (u >= 0) & (u < image.width) & (v >= 0) & (v < image.height)
    keys = np.unique(v[inside].astype(int) * image.width + u[inside].astype(int))
    return image.lengths(keys % image.width, keys // image.width)[0]


def run(directory, output):
    if output.exists():
        raise ValueError('choose a new output path')
    database = directory / 'after.db'
    subjects_path = directory / 'physical-subjects-frozen.json'
    masks_path = directory / 'mask-review-frozen.json'
    input_hashes = {str(p.relative_to(directory)): digest(p) for p in
                    [database, directory / 'before.db', subjects_path, masks_path,
                     directory / 'visual-protocol.json']}
    subjects = json.loads(subjects_path.read_text())['subjects']
    ids = {i for one in subjects.values() for i in one}
    rows = {}
    with sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        for i in ids:
            row = dict(db.execute('SELECT * FROM observations WHERE id=?', (i,)).fetchone())
            row['pose'] = json.loads(row['observer_pose_json'])
            rows[i] = row
    pairs = {}
    for subject, members in subjects.items():
        pairs[subject] = []
        for a, b in itertools.combinations(members, 2):
            ra, rb = resolve.ray_of(rows[a]), resolve.ray_of(rows[b])
            baseline = locate.baseline_m(ra, rb)
            parallax = locate.parallax_deg(ra, rb)
            pairs[subject].append({
                'ids': [a, b], 'plain': cosine(rows[a], rows[b], 'dino_blob'),
                'masked': cosine(rows[a], rows[b], 'dino_alone_blob'),
                'baseline_m': baseline, 'parallax_deg': parallax,
                'independent_geometry': baseline >= locate.MIN_BASELINE_M
                    and parallax >= locate.MIN_PARALLAX_DEG,
                'fix': locate.fix(ra, rb)})
    negatives = []
    chairs = subjects['front-left-dining-chair'] + subjects['front-right-dining-chair']
    for a, b in itertools.product(subjects['painting-behind-chairs'], chairs):
        negatives.append({'ids': [a, b], 'plain': cosine(rows[a], rows[b], 'dino_blob'),
                          'masked': cosine(rows[a], rows[b], 'dino_alone_blob')})
    depth = []
    for i, row in sorted(rows.items()):
        frame = directory / 'frames' / (row['frame_id'] + '.jpg')
        input_hashes[str(frame.relative_to(directory))] = digest(frame)
        path = frame.with_suffix('.depth.gz')
        if not path.exists():
            depth.append({'id': i, 'missing': 'no depth map archived'})
            continue
        input_hashes[str(path.relative_to(directory))] = digest(path)
        body = gzip.decompress(path.read_bytes())
        n = int.from_bytes(body[:4], 'little')
        header = json.loads(body[4:4+n])
        lens = depth_client.Lens(**header['lens'])
        image = outline.DepthImage(np, body[4+n:], header['width'], header['height'], lens)
        size = Image.open(frame).size
        replay = outline.read(np, body[4+n:], header['width'], header['height'], lens,
                              [(json.loads(row['bbox_json']), row['outline_blob'])], size)[0]
        one = {'id': i, 'stored_range_m': row['range_m'], 'stored_sigma_m': row['range_sigma_m'],
               'stored_method': row['range_from'], 'replayed': replay,
               'depth_header': header,
               'stationary_projection': 'zero turn; selected manual looks recorded no shutter motion'}
        if replay.get('range_m') is not None and row['range_m'] is not None:
            one['range_delta_m'] = float(replay['range_m']) - row['range_m']
            if replay.get('method') == 'outline':
                samples = depth_samples(image, row['outline_blob'], size, float(replay['range_m']))
                found = outline.surface(np, samples)
                one['valid_samples'] = int(samples.size)
                one['quantiles_m'] = dict(zip(['p05', 'p20', 'p50', 'p80', 'p95'],
                                             np.quantile(samples, [.05, .2, .5, .8, .95]).tolist()))
                one['chosen_band_share'] = found[2] / samples.size
                one['under_2_5m_share'] = float((samples < 2.5).mean())
                one['over_3m_share'] = float((samples > 3).mean())
                one['initial_guess_sensitivity'] = [
                    {'guess_m': guess, 'answer': outline.outline_range(
                        image, row['outline_blob'], size, guess=guess)}
                    for guess in [1, 2, 3, 4, 5, 6]]
        depth.append(one)
    result = {'input_sha256': input_hashes, 'same_subject_pairs': pairs,
              'painting_chair_pairs': negatives, 'depth': depth,
              'limitations': ['Analyst development labels; not independent acceptance.',
                  'No recorded exact background resolver sequence; no resolver replay claim.',
                  'Saved outlines are clipped and half resolution, unlike the full appearance mask.',
                  'No tape ground truth; range-band ambiguity is reproduced, not a calibration verdict.',
                  'This diagnostic neither fits a gate nor changes observations or entities.']}
    assert digest(database) == input_hashes['after.db']
    output.write_text(json.dumps(result, indent=2) + '\n')
    return result


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = run(args.directory, args.output)
    print('Diagnosed', sum(len(v) for v in result['same_subject_pairs'].values()),
          'same-subject pairs and', len(result['painting_chair_pairs']), 'painting/chair pairs')
