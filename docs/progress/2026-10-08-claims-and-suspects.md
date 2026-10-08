# Aimed filing: a disagreeing range now widens the claim, and suspects use filing's own bar

**Two weaknesses M4 session 1 showed are fixed and deployed (1f0a0d1, world_state
and the daemon).** A kitchen cabinet whose two aimed ranges landed 1.25 m apart
had claimed 0.20 m, and now claims 0.88. The rug under the dining table has seven
records and no filing named any other; at the new bar each of its filings
names the other six. R-WS-13 stays open.

## A range that stops agreeing

The claim from aimed depth took its spread only over ranges that still point at
the bearing placement. As that placement moved, object:367's claim went 0.24,
0.74, 0.62 and 0.20 m: its second range, 1.25 m from the first, stopped
agreeing, and one range alone claims the 0.20 m floor. Reproduced on a copy of
the rover's store with the deployed code (0.20 m), then with the change
(0.88 m). A range that no longer agrees now gives no position but is counted in
the spread.

On the taped targets of 2026-10-03, with every ranged look treated as aimed
(`experiments/entity_association/claim_honesty.py`), counting such ranges
changed no claim's median (0.20 m) and the tape was inside every claim (98.8%
before). The floor itself holds up there: single ranges missed the tape by a
median 0.16 m, 0.18 m at the 68th percentile and 0.32 m at the 90th, and a
claim resting on one ranged viewpoint covered the tape 11 times in 12. A
large or mixed record is the case the floor cannot cover, and the spread is
what catches it.

Put through the 157 recorded aimed looks of 10-02 to 10-06
(`replay_aimed.py`), the same 24 regions are filed; some claims now rise where
aimed ranges disagree, one from 0.22 to 0.96 m.

## Suspects at filing's own bar

A record the filed region also fits was named a same-object suspect only if the
region looked 0.70 like it (`RECOGNISED`), while the region itself was filed at
0.55 (`DIFFERENT_THING`). The rug's filed regions looked 0.57 to 0.68 like its
other records, so none was named, and the run went back to the rug by a second
record. The bar is now 0.55: a record the region could itself have been filed to.

On the session's 7 filings this adds 13 suspects. By their photographs (two
recent sightings each, analyst's judgment):

| Filing | Added suspects | Same object |
|---|---|---|
| the rug (object:338) | 6 | 6, all records of the rug |
| the black cabinet (object:461) | 7 | 5; the others are the painting above it and a person in front of it |

The two chair filings add 3 and 5 more, not judged. Over the 157 recorded looks
the 24 filings name 100 suspects against 46, and 23 filings name at least one
against 20. A wrong suspect costs its record 15 minutes set aside; nothing is
merged. Filings made before the change keep the suspects they were given.

## On the rover

Deployed with `deploy.py` at 1f0a0d1; both self-tests passed there
(world_state 1,081 here). With the deployed modules, on a copy of the rover's
store: object:367's claim is 0.88 m, and the rug's filing names objects 514,
336, 471, 353, 348 and 417.
