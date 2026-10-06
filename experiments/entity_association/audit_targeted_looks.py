"""Read frozen episode/world stores; trace targeted looks by exact frame ID.

Current observation owners are explicitly snapshot-time owners, not historical
assignments or physical-object truth. No timestamp-nearest frame matching.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import re
import sqlite3


def audit(directory):
    def connect(name):
        db = sqlite3.connect((directory/name).resolve().as_uri()+'?mode=ro', uri=True)
        db.row_factory = sqlite3.Row
        return db
    episodes, world = connect('episodes.db'), connect('world.db')
    generation = world.execute("SELECT value FROM meta WHERE key='generation'").fetchone()[0]
    generations = dict(episodes.execute('SELECT id,world_generation FROM episodes'))
    targets = {r[0] for r in world.execute('SELECT id FROM entities')}
    def target_at(digest, target):
        snapshot = episodes.execute('SELECT body_json FROM snapshots WHERE digest=?',(digest,)).fetchone()
        if not snapshot:
            return None
        state = json.loads(snapshot[0])
        linked = state.get('world_and_map')
        if not linked:
            return None
        objects = json.loads(episodes.execute('SELECT body_json FROM snapshots WHERE digest=?',
                                             (linked,)).fetchone()[0])
        return next((e for e in objects['entities'] if e['id']==target),None)
    rows = []
    for decision in episodes.execute("SELECT * FROM events WHERE kind='decision' ORDER BY id"):
        chosen = json.loads(decision['body_json'])
        match = re.fullmatch(r'improve_geometry:object:(\d+)@.+', chosen.get('chose',''))
        if not match:
            continue
        target = 'object:'+match[1]
        events = [(r, json.loads(r['body_json'])) for r in episodes.execute(
            'SELECT * FROM events WHERE episode_id=? ORDER BY seq', (decision['episode_id'],))]
        calls = [(r,b) for r,b in events if r['kind']=='call' and b.get('call')=='world_inspect']
        measured = next((b for r,b in events if r['kind']=='measured' and
                         'placement_uncertainty_before_m' in b), None)
        base = dict(episode=decision['episode_id'], target=target, decision_at=decision['at'],
                    generation=generations[decision['episode_id']],
                    target_exists_in_snapshot=target in targets,
                    target_before=target_at(chosen['inputs'],target),
                    rationale=chosen['why'], measured=measured)
        if base['generation'] != generation:
            rows.append(dict(base, category='different_world_generation'))
            continue
        if not calls:
            rows.append(dict(base, category='no_inspection_call'))
        for event, call in calls:
            result = call.get('result') or {}
            row = dict(base, call_event=event['id'], call_at=event['at'], call=call)
            frame = result.get('frame_id')
            if not call.get('ok'):
                row['category'] = 'inspection_failed'
            elif not frame:
                row['category'] = ('unchanged_picture' if result.get('status')=='unchanged'
                                   else 'no_frame_in_reply')
            else:
                obs = [dict(r) for r in world.execute('''SELECT id,entity_id,inference_id,
                    observed_at,bearing_deg,range_m,range_absent,bearing_sigma_deg,
                    bbox_json,observer_pose_json,map_session FROM observations WHERE frame_id=?''',(frame,))]
                owners = Counter(str(o['entity_id']) for o in obs)
                row.update(frame_id=frame, observations=obs, snapshot_owners=dict(owners),
                           target_observations=[o['id'] for o in obs if o['entity_id']==target],
                           ranged_observations=[o['id'] for o in obs if o['range_m'] is not None])
                row['category'] = ('frame_missing_from_world_snapshot' if not obs else
                    'target_no_longer_in_world_snapshot' if target not in targets else
                    'target_has_frame_observation' if row['target_observations'] else
                    'frame_observations_elsewhere_or_unassigned')
            rows.append(row)
    counts = Counter(r['category'] for r in rows)
    exact = [r for r in rows if r.get('observations')]
    ranged = [r for r in exact if r['ranged_observations']]
    surviving = [r for r in exact if r['target_exists_in_snapshot']]
    return {'world_generation':generation,
            'input_sha256':{n:hashlib.sha256((directory/n).read_bytes()).hexdigest()
                           for n in ['episodes.db','world.db']},
            'limits':['Owners are those in the copied world store, not at call time.',
                      'Physical identity and missed detection require image review.',
                      'Unchanged replies are not joined to a nearby frame by time.',
                      'Stores were backed up consecutively, not atomically together.'],
            'counts':dict(counts), 'decisions':len({r['episode'] for r in rows}),
            'exact_frames':len(exact), 'unique_exact_frames':len({r['frame_id'] for r in exact}),
            'exact_frames_with_surviving_target':len(surviving),
            'frames_with_depth':len(ranged),
            'frames_with_depth_on_target':sum(any(o['entity_id']==r['target'] and o['range_m'] is not None
                                               for o in r['observations']) for r in ranged),
            'rows':rows}


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--directory',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    args=p.parse_args()
    result=audit(args.directory)
    args.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'},indent=2))
