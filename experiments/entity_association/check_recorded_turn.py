"""Check a local costmap snapshot for an in-place turn; sends no commands.

The current rover configuration uses a 0.20 m circular body. Use a containing
polygon and every 5-degree sample. Check the centre against the inflated body
band and the physical footprint against lethal cells; applying the inflation
band to every body cell would inflate the body twice. Unknown/out-of-bounds cells, stale data and
missing/frame-inconsistent transforms refuse the check. This is preparation
evidence for one snapshot, not authorization or future collision protection.
"""
import argparse
import base64
import json
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'ros_nav'))
import goal_fit


def check(snapshot, *, now=None):
    if now is not None:
        age = now - snapshot['captured_at']
        if not math.isfinite(age) or age < -0.1 or age > 5:
            raise ValueError('Capture is stale; obtain a new snapshot before turning.')
    grid = snapshot['grids']['local']
    pose = grid['body_pose']
    if pose['frame'] != grid['frame']:
        raise ValueError('Body pose must be in the local costmap frame.')
    ages = [grid['age_s'], pose['age_s']]
    if any(not math.isfinite(age) or age < -0.1 or age > 2 for age in ages):
        raise ValueError('Local costmap or body transform is stale.')
    orientation = grid['origin_orientation']
    if any(abs(v) > 1e-6 for v in orientation[:3]) or abs(abs(orientation[3])-1) > 1e-6:
        raise ValueError('Rotated costmap origin is unsupported; do not ignore it.')
    raw = base64.b64decode(grid['data'], validate=True)
    if len(raw) != grid['width'] * grid['height']:
        raise ValueError('Incomplete costmap.')
    if not math.isfinite(grid['resolution']) or grid['resolution'] <= 0:
        raise ValueError('Invalid costmap resolution.')
    x, y = pose['x_m'], pose['y_m']
    if not all(math.isfinite(v) for v in (x, y)):
        raise ValueError('Invalid body pose.')
    costmap = goal_fit.CostGrid(grid['width'], grid['height'], grid['resolution'],
                               *grid['origin'], list(raw))
    footprint = goal_fit.polygon_from('', 0.20 / math.cos(math.pi / 32), sides=32)
    centre_cost = costmap.cost(*costmap.cell_of(x, y))
    costs = [costmap.cost(col, row) for heading in range(-180, 181, 5)
             for col, row in goal_fit.covered(costmap, footprint, x, y,
                                              math.radians(heading))]
    worst = max(costs)
    return {'ok': centre_cost < 253 and worst < 254,
            'centre_cost': centre_cost, 'max_covered_cost': worst,
            'frame': grid['frame'], 'body_pose': pose,
            'costmap_age_s': grid['age_s'], 'body_radius_m': 0.20,
            'limit': 'Snapshot only; Nav2 and live sensor checks still govern execution.'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshot', type=Path, required=True)
    parser.add_argument('--live', action='store_true',
                        help='also refuse captures older than five seconds')
    args = parser.parse_args()
    try:
        result = check(json.loads(args.snapshot.read_text(encoding='utf-8-sig')),
                       now=time.time() if args.live else None)
    except (ValueError, KeyError, TypeError) as error:
        result = {'ok': False, 'error': str(error)}
    print(json.dumps(result))
    return 0 if result['ok'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
