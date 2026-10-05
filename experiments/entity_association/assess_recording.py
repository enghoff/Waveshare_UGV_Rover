"""Score a frozen grouping preview on fresh analyst labels without fitting anything.

Labels are judgments, not independent acceptance. Only fresh/fresh pairs count;
the preview still uses the store's full history, including development evidence.
"""
from __future__ import annotations

import argparse
from collections import Counter
from contextlib import closing
import hashlib
from itertools import combinations
import json
from pathlib import Path
import sqlite3
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def score(labels, owners):
    counts = Counter()
    wrong = []
    same_apart = []
    for a, b in combinations(labels, 2):
        i, j = int(a['observation_id']), int(b['observation_id'])
        same = a['physical_object'] == b['physical_object']
        together = owners.get(i) is not None and owners.get(i) == owners.get(j)
        counts[('same' if same else 'different') + ('_together' if together else '_apart')] += 1
        if together and not same:
            wrong.append([i, j])
        if same and not together:
            same_apart.append([i, j])
    per_object = {}
    for name in sorted({r['physical_object'] for r in labels}):
        ids = [int(r['observation_id']) for r in labels if r['physical_object'] == name]
        pieces = Counter(owners[i] for i in ids if owners.get(i) is not None)
        per_object[name] = {'regions': len(ids), 'waiting': sum(owners.get(i) is None for i in ids),
                            'pieces': len(pieces), 'memberships': dict(pieces)}
    return {**{k: counts[k] for k in ('same_together', 'same_apart', 'different_together',
                                     'different_apart')},
            'regions': len(labels), 'objects': len(per_object),
            'waiting': sum(owners.get(int(r['observation_id'])) is None for r in labels),
            'wrong_pairs': wrong, 'same_apart_pairs': same_apart, 'per_object': per_object}


def compare(labels, owners, aliases):
    grouped = {i: aliases.get(owner, owner) for i, owner in owners.items()}
    before, after = score(labels, owners), score(labels, grouped)
    return {'baseline': before, 'grouped': after,
            'added_same_pairs': after['same_together'] - before['same_together'],
            'added_wrong_pairs': sorted(map(list, set(map(tuple, after['wrong_pairs'])) -
                                           set(map(tuple, before['wrong_pairs']))))}


def assess(database, draft, cutoff):
    from world_state.reader_groups import preview
    from world_state.store import WorldStore

    digest = hashlib.sha256(database.read_bytes()).hexdigest()
    document = json.loads(draft.read_text())
    if document['source_database_sha256'] != digest:
        raise ValueError('draft does not describe this database')
    labels = document['labels']
    ids = [int(r['observation_id']) for r in labels]
    if len(set(ids)) != len(ids) or any(i <= cutoff for i in ids):
        raise ValueError('labels must be unique fresh observations')
    with closing(sqlite3.connect(database.resolve().as_uri() + '?mode=ro', uri=True)) as con:
        con.row_factory = sqlite3.Row
        rows = {r['id']: dict(r) for r in con.execute(
            'SELECT id,frame_id,bbox_json,entity_id FROM observations WHERE id>?', (cutoff,))}
        if set(rows) != set(ids):
            raise ValueError('labels must cover exactly the fresh observations')
        for lab in labels:
            row = rows[int(lab['observation_id'])]
            if row['frame_id'] != lab['frame_id'] or json.loads(row['bbox_json']) != json.loads(lab['bbox']):
                raise ValueError('label source provenance changed')
        with tempfile.TemporaryDirectory(prefix='ugv-assess-recording-') as tmp:
            store = WorldStore(tmp)
            try:
                con.backup(store.db)
                store._create()  # upgrade only our temporary copy
                result = preview(store)
            finally:
                store.close()
    if not result.get('ok') or result['stale'] or not result['converged']:
        raise ValueError('preview did not produce a current converged result')
    aliases = {member: g['representative'] for g in result['groups'] for member in g['members']}
    owners = {i: r['entity_id'] for i, r in rows.items()}
    eligible = [r for r in labels if r['verdict'] == 'object' and r['physical_object']]
    clear = [r for r in eligible if r['review_status'] == 'draft_clear']
    if hashlib.sha256(database.read_bytes()).hexdigest() != digest:
        raise ValueError('source database changed')
    return {'source_database_sha256': digest, 'labels_sha256': hashlib.sha256(draft.read_bytes()).hexdigest(),
            'after_observation_id': cutoff, 'source_unchanged': True,
            'independent_acceptance': False, 'training_or_tuning': False,
            'label_author': 'coding_agent_first_pass', 'fresh_regions': len(ids),
            'preview_uses_prior_history': True, 'preview': result,
            'primary_clear_only': compare(clear, owners, aliases),
            'tentative_label_sensitivity': compare(eligible, owners, aliases),
            'excluded_primary_regions': sorted(set(ids) - {int(r['observation_id']) for r in clear}),
            'limitations': ['Labels are analyst judgments, not owner-confirmed or independent truth.',
                            'Only fresh/fresh pairs count; old regions in the snapshot still influence grouping.',
                            'The same room and many physical objects were present in development.',
                            'Unknown/mixed/surface regions do not count as correct or incorrect pairs.',
                            'No missed-object census or placement accuracy is measured.',
                            'Tentative sensitivity deliberately includes uncertain identities; it is not truth.']}


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--database', type=Path, required=True)
    p.add_argument('--draft', type=Path, required=True)
    p.add_argument('--after-observation', type=int, required=True)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    result = assess(args.database, args.draft, args.after_observation)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2), encoding='utf-8')
    for name in ('primary_clear_only', 'tentative_label_sensitivity'):
        value = result[name]
        print(name, json.dumps({k: value[k] for k in ('added_same_pairs', 'added_wrong_pairs')}))


if __name__ == '__main__':
    main()
