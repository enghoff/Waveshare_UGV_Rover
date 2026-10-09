# M4's attempts and their scoring

What [M4](../../docs/plans/autonomous-curiosity.md) asks is whether a look from the
viewpoint the rover chose improves where it knows a thing to be more than a look
again from where it stood when it chose. The case for measuring it that way is
[m4-measures-where-things-are.md](../../docs/decisions/m4-measures-where-things-are.md).

| File | What it does |
|---|---|
| `start_trial.py truth.json "purpose" [area]` | on the rover: opens a run that may look only at the records `truth.json` names, copies the world store before each attempt and re-looks before each drive |
| `score_attempts.py attempts ...` | at a desk: files each attempt's chosen look and its re-look into their own copies of that attempt's snapshot, scores the target's placement against the tape before and after, and reports criteria 3, 5, 6 and 8 with intervals that resample things rather than attempts |
| `score_attempts.py plan RATE RATES...` | how many pairs the comparison needs, for given rates at which chosen looks and re-looks improve their thing |
| `test_score_attempts.py` | the scoring's checks, on a room drawn by hand |

**The truth file** names each taped thing's position in map coordinates and the
store's records that are it, labelled from their photographs before any attempt:

    {"name": "M4 acceptance", "things": {"P1": {"x_m": -18.2, "y_m": -12.9,
      "records": ["object:248"], "where": "dining room"}}}

A thing may also carry a `footprint`, and is then scored by a placement's distance
to the thing rather than to the taped point: a disc (`radius_m`), a stretch of wall
(`width_m` along `along_deg`), or a box `width_m` across its front and `depth_m`
back from the taped point towards `into_deg`. On 2026-10-09 every acceptance
attempt that scored worse against a point was a look landing on big furniture
half a metre from the centre of its front (`distance_to`;
[the re-scoring](../../docs/progress/2026-10-09-m4-scored-from-the-thing.md)).

**After a run**, copy from the rover the episode store
(`~/.ugv/autonomy/episodes.db`), the world store (`~/.ugv/world/world.db`), each
through sqlite's own backup since both keep a write-ahead log, and the snapshots
(`~/.ugv/world/snapshots/`, about 200 MB each). Then:

    python experiments/m4/score_attempts.py attempts --episodes episodes.db \
        --store world.db --snapshots snapshots/ --truth truth.json --out report.json

An attempt without a snapshot, whose snapshot was not copied, or aimed at a record
no taped thing names is reported as not scorable and says why. Snapshots can be
deleted from the rover once scored.

**Not yet done:** the new taped set and its labels, and the development attempts on
the six things taped on 2026-10-03 that set how many acceptance pairs to ask for.
On 2026-10-09 two records that each hold one of the 2026-10-03 looks at a taped
thing, the cabinet and a painting, stood 2.0 and 1.7 m from its tape, and every
aimed look at them on 2026-10-08 filed nothing. A record is labelled as a taped
thing from its photographs and its position, not from a look it happens to hold.
