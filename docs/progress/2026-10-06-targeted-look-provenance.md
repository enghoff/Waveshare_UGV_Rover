# A targeted look can reach a duplicate instead of the record it was meant to improve

The newer autonomy record supplies useful identity evidence without another drive.
One reviewed painting look detects and ranges the intended physical painting but
belongs to another existing record of it. Failure to improve the requested record
is therefore not, by itself, failure to see that object. R-WS-13 remains open;
R-WS-17 and R-WS-18 remain proposed. No resolver change or automatic merge follows.

## Frozen record and counts

SQLite backups of the episode and world stores are retained under
`captures/entity-target-audit-20261006/`, with verified hashes. They were copied
consecutively, not as an atomic pair. The read-only
`audit_targeted_looks.py` joins explicit returned frame identifiers, never the
nearest timestamp, and checks world generation. Entity owners mean the owners in
this copied store, not necessarily the owners when the call returned.

This is a larger cohort than the earlier
[171 evaluated goals](2026-10-06-looks-seldom-reach-their-thing.md): it includes
285 chosen geometry goals, including attempts which failed before evaluation.

| Recorded outcome | Goals |
|---|---:|
| No inspection call | 24 |
| Inspection failed | 16 |
| Picture reported unchanged, no frame returned | 88 |
| Exact frame retained, target no longer present in copied world store | 58 |
| Exact frame retained, surviving target has none of its observations | 92 |
| Exact frame retained, surviving target has an observation | 7 |

All 157 exact frame identifiers are unique. Of these, 153 contain at least one
depth-ranged region; only three contain a ranged region currently assigned to
the requested record. These are provenance counts, not detection accuracy or
physical-identity labels. In particular, the 58 absent targets cannot be counted
as failed attachments to a surviving entity. Unchanged calls may have been
preceded by a background look: they are not evidence that the camera saw nothing.

## Reviewed case: episode 858

The request targeted `object:328`. Frame `20261006-170523-d27ce8` contains
observation 72149, now attached to `object:351`, with stored range 2.664 m.
The box spans [0.2406, 0.1292, 0.3710, 0.3902] in the full photograph and shows
the visible dining-room landscape painting beside the foreground chair.

Earlier target observation 70929, frame `20261006-125013-67e5da`, spans
[0.1581, 0.1243, 0.2884, 0.3806] and shows the same physical painting from a
different position. It belongs to `object:328`. This review used full photographs
and stored boxes after assignments were known; it is exploratory, not blind
acceptance truth. It does not establish that every view in either record is pure.

The earlier stored range is 1.280 m. Both raw depth frames were recovered and
`replay_targeted_depth.py` reproduces both stored answers and methods exactly.
The earlier answer used the bounding-box fallback; the later used the outline.
The later outline's selected surface contains 73.5% of valid samples, with a
0.074 m median gap, so the previously tested minority-surface rule would retain it.
Neither distance is independently validated here. A reproduced depth answer in
the correct photograph is not proof that it measured the painting rather than
foreground material. This case distinguishes sampler reproducibility from truth.

The requested record's uncertainty stayed at 1.082 m in the immediate evaluation.
That fact alone missed the useful detection stored under its duplicate. The
[measurement](2026-10-06-targeted-look-provenance.json) records input hashes,
the complete case, boxes, photo hashes and limitations; the full per-goal audit
is retained with the frozen databases.

## Decision

Keep the narrower visibility candidate provisional and preserve its earlier
mixed-crop failure. Do not infer that all missing target improvement is bad
heading, missing depth, or failure to recognise the physical object. Separate
duplicate identity, range correctness and evaluation of one record's uncertainty.
The new case strengthens the need for physical-object labels and retained depth;
it cannot replace the exact call/depth replay and independent clear/occluded
coverage required by the [run plan](../plans/entity-evidence-drive.md).
