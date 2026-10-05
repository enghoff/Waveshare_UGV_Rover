"""Reproduce the fixed depth rule on a separately reviewed qualitative example."""
import argparse
import base64
import json
from pathlib import Path
import sqlite3
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import outline, perceive
from experiments.entity_association.audit_depth_abstention import digest, iou, json_numpy, load_depth, trace


def run(database, frames, model, observation, output):
    if output.exists():
        raise ValueError('choose a new output file')
    import onnxruntime as ort
    options = ort.SessionOptions()
    options.intra_op_num_threads = 2
    options.inter_op_num_threads = 1
    session = ort.InferenceSession(str(model), sess_options=options, providers=['CPUExecutionProvider'])
    class Models:
        def regions(self, blob):
            outputs = session.run(None, {'images': blob})
            return outputs[0], outputs[1]
    with sqlite3.connect(database.resolve().as_uri()+'?mode=ro', uri=True) as db:
        db.row_factory = sqlite3.Row
        row = dict(db.execute('SELECT * FROM observations WHERE id=?', (observation,)).fetchone())
    photo, depth = frames/(row['frame_id']+'.jpg'), frames/(row['frame_id']+'.depth.gz')
    finder = perceive.Perception(str(model.parent))
    finder._np, finder._cv2, finder._models = np, cv2, Models()
    rgb = cv2.imread(str(photo))
    boxes, _, masks, _ = finder._regions(rgb)
    bbox = json.loads(row['bbox_json'])
    which = max(range(len(boxes)), key=lambda j: iou(bbox, boxes[j]))
    overlap = iou(bbox, boxes[which])
    if overlap < .5:
        raise ValueError('region regeneration does not match the recorded box')
    blob = outline.encode(np, masks.of(which), bbox)
    image, raw, header, inferred = load_depth(depth)
    size = (rgb.shape[1], rgb.shape[0])
    boxed = outline.box_range(image, bbox, size)
    control = outline.read(np, raw, image.width, image.height, image.lens, [(bbox, blob)], size)[0]
    traced = trace(image, blob, size, boxed.get('range_m'))
    assert traced and traced['range_m'] == control['range_m']
    review = frames/'glass-review-frozen.json'
    result = {'id': observation, 'frame_id': row['frame_id'], 'bbox': bbox,
              'mask_iou': overlap, 'production_replay': control, 'trace': traced,
              'outline_base64': base64.b64encode(blob).decode(),
              'input_sha256': {str(p): digest(p) for p in [database, photo, depth, model, review, Path(__file__)]},
              'limit': 'Qualitative example; no taped range truth and not an acceptance score.'}
    output.write_text(json.dumps(result, indent=2, default=json_numpy)+'\n')
    print(json.dumps(traced))


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__)
    for name in ['database', 'frames', 'model', 'output']:
        p.add_argument('--'+name, type=Path, required=True)
    p.add_argument('--observation', type=int, required=True)
    a = p.parse_args()
    run(a.database, a.frames, a.model, a.observation, a.output)
