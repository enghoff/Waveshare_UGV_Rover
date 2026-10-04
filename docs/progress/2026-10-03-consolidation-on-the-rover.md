# EM consolidation is on the rover, applied once for review, and can be rolled back

**The rover can now re-solve a map session's things all at once by EM, and put them back
exactly.** It was deployed at e0ecddd and applied once to map session 67, as
consolidation 1, for the owner to review in the console. It took the rover from 128 things
to 71, and every one of its 2,625 looks is still stored. The door the resolver had split
into `object:96`, `object:100` and `object:102` is now one thing, `object:17`, holding all
eight of the door's looks. Whether the rest is better is the review's to say:
[the replay](2026-10-03-whole-session-em.md) found that EM also merges different objects
and drops real things. [R-WS-13](../requirements/world-state.md#r-ws-13) stays `open`.

## What was applied

`world_state_consolidate {"apply": true}` on the Orin, with autonomy checked to be off
first. It took 17.9 s with the rover's looks held off, against 10 s at a desk.

- **17 things took in others**, 23 in all. Besides the door, `object:107` took
  `object:28` and `object:65`, and `object:16` took `object:108` and `object:117`.
- **34 things went**, their looks fitting nothing better than scenery. The largest were
  `object:88` (25 looks), `object:125` (20) and `object:19` (19).
- **1,256 looks changed thing**, 124 were let go and 67 waiting looks were taken up.
  Most of the changes are not merges: neighbours that appear in the same pictures, such
  as `object:51` and `object:3` 7 cm apart, had their looks split between them again.

## Rolling it back

`world_state_consolidate {"rollback": true}` puts consolidation 1 back. Rehearsed on a copy of the
rover's store taken at 15:38, applying took 0.5 s and rolling back 0.4 s. Afterwards every
thing and every look's thing and note were byte-identical to the copy. Looks recorded
after applying stay where the resolver puts them. Things the resolver founds from looks
EM let go are removed by the rollback.

## Addendum, 2026-10-04: rolled back, and the call removed

The owner had both consolidations rolled back the same evening and the store rebuilt by
the resolver from its looks, which gave 182 things. The per-look tests that followed
([cleaning things up, tested on recordings](2026-10-03-cleaning-things-tested-on-recordings.md))
found nothing that beats the resolver on this data, so the call, its journal and its
rollback were taken out of the rover. The EM itself survives as the bench
`world_state/bench_whole.py`. The rover's database still holds the two runs' journal
tables, which nothing reads; the store never drops recorded history.
