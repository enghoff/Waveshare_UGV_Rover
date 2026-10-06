"""Offline diagnostic: apply the existing rival veto to whole-frame matching.

No production source changes or rover calls. This combines the frozen matching-only
visibility diagnostic with the existing OUTCLASSED_LEAD, without fitting a threshold.
The veto is evaluated before frame assignment by marking that candidate forbidden.
"""
from __future__ import annotations
import argparse
import contextlib
import json
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from world_state import resolve
from experiments.entity_association.replay_call_recording import run
from experiments.entity_association.replay_measured_visibility import matching_scope, matching_visibility


@contextlib.contextmanager
def frame_rival_veto(enabled, original_rays=None):
    original_frame, original_drop = resolve._by_look, resolve.collapsed
    active = []
    refusals = []

    def frame(store, group, entities, session, taken_in, reach=None):
        active.append((entities, {}))
        try:
            return original_frame(store, group, entities, session, taken_in, reach)
        finally:
            active.pop()

    def drop(store, entity_id, observation, seen):
        value = original_drop(store, entity_id, observation, seen)
        if active and enabled():
            entities, found = active[-1]
            if original_rays is not None:
                ray = original_rays[observation['id']]
                placement = next(e for e in entities if e['id'] == entity_id).get('placement') or {}
                if resolve._allowance_used(placement, ray) is not None:
                    return value
            rival = resolve._outclassed(store, entity_id, observation, seen, entities, found)
            if rival is not None:
                refusals.append({'observation_id': observation['id'], 'entity_id': entity_id,
                                 'rival': rival, 'seen': seen, 'original_drop': value})
                return max(value or 0.0, resolve.COLLAPSED_ALONE)
        return value

    resolve._by_look, resolve.collapsed = frame, drop
    try:
        yield refusals
    finally:
        resolve._by_look, resolve.collapsed = original_frame, original_drop


def main():
    p = argparse.ArgumentParser(description=__doc__)
    inputs = p.add_mutually_exclusive_group(required=True)
    inputs.add_argument('--directory', type=Path)
    inputs.add_argument('--older-source', type=Path)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--new-visibility-only', action='store_true',
                   help='Apply extra veto only to candidates forbidden by the original ray geometry.')
    a = p.parse_args()
    # Keep the extra veto disabled through exact control. The replay first calls
    # the transform when it starts the candidate, before that arm's frame costs.
    candidate_active = [False]
    original_rays = {}
    def transform(ray):
        candidate_active[0] = True
        original_rays[ray['observation_id']] = dict(ray)
        return matching_visibility(ray)
    with frame_rival_veto(lambda: candidate_active[0],
                          original_rays if a.new_visibility_only else None) as refusals:
        if a.older_source:
            from experiments.entity_association import replay_depth_abstention as depth_replay
            from experiments.entity_association import replay_measured_visibility as visibility
            original_run, original_transform = depth_replay.run_replay, visibility.matching_visibility
            calls = [0]
            def older_arm(*args, **kwargs):
                calls[0] += 1
                candidate_active[0] = calls[0] % 2 == 0
                original_rays.clear()
                return original_run(*args, **kwargs)
            depth_replay.run_replay, visibility.matching_visibility = older_arm, transform
            try:
                result = visibility.older_regression(a.older_source, a.output, known_only=True)
                assert calls[0] == 2 * len(result['drives']), 'control/candidate ordering changed'
            finally:
                depth_replay.run_replay, visibility.matching_visibility = original_run, original_transform
        else:
            with matching_scope():
                result = run(a.directory, a.output, map_invariant=True, candidate_ray_transform=transform)
    report = {'independent_acceptance': False, 'threshold': resolve.OUTCLASSED_LEAD,
              'new_visibility_only': a.new_visibility_only,
              'candidate': 'Matching-only measured visibility plus existing rival veto in frame costs',
              'refusals': refusals, 'owner_changes': result.get('owner_changes', []),
              'older_regression': bool(a.older_source)}
    (a.output/'rival-veto.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
