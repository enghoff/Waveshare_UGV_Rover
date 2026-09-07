# Why a refit could not find a rover whose heading was 152 degrees out

The rover came up on its restored map with the right position and a badly wrong
heading, and the console's "refit to map" refused with a number that reads like a
threshold problem: the best pose it could find put 80% of the scan on a wall
where a fit needs 90%. **The threshold was not the problem.** At the rover's true
pose the scan scores 0.99. The search window is what stood in the way -- it looks
45 degrees either side of where the rover believes it is, and the error was 152.5
degrees, so the answer was never inside the box being searched.

Widening the heading window recovered the rover on the first attempt, on the
existing thresholds and with margin to spare. Lowering the threshold instead
would have moved the rover 1.1 m and 29.5 degrees the wrong way and then written
that pose over the good saved map.

Two other things came out of the same sitting: the rover turned about 146 degrees
when asked for 90, and the world state recorded 34 observations against the wrong
heading with nothing in its path to stop it.

## What was measured

The rover was parked, mapping, on restored map `7660766cbfd4` at world session
67, with `map_settled` false -- the restore had anchored the graph 0.10 m and 37
degrees from where the map said the rover was left, and nothing had confirmed it
since. Its believed pose was `(-16.360, -17.539, 107.3)`; the saved note on disk
said it was parked at `(-16.265, -17.553, 114.0)`.

The live `/map`, `/scan` and pose were captured off the rover into one JSON file
and fitted on a workstation with [`ros_nav/refit.py`](../../ros_nav/refit.py)
itself, so what is reported below is the deployed matcher and not a second one
written to agree with it. The grid was 257x335 cells at 0.05 m; the scan carried
267 usable returns of 360, the rest being no-echo. The sweep was the full circle
at 1 degree, over plus or minus 1.5 m at 10 cm, scored by `refit.field` and
`refit._scores` unchanged.

## What the numbers were

Every distinct peak in the whole circle, best first. "Score" is what the matcher
reports -- the share of the scan on a wall, averaged over only those returns
landing on ground the map has an opinion about. "Of" is how many returns that
was. "Whole" is the same quantity taken over the entire scan instead.

| Turn from belief | Score | Of 267 | Whole |
|---|---|---|---|
| +152 | 0.962 | 267 | 0.962 |
| -30 | 0.905 | 98 | 0.332 |
| -28 | 0.800 | 109 | 0.327 |
| -121 | 0.750 | 142 | 0.399 |
| -53 | 0.741 | 65 | 0.180 |
| -28 | 0.732 | 115 | 0.315 |

Where the rover believed it was scored 0.197 over 206 returns. The saved parked
pose scored 0.176 over 209 -- so the note on disk was no better, which is worth
recording because centring the search there is the rescue that exists for a badly
anchored restore and it would not have helped either.

Then the real `refit.fit`, given the windows it could have been given, centred on
the rover's belief:

| Window | Score | Rival | Margin | Turn | Moved | Returns | Verdict |
|---|---|---|---|---|---|---|---|
| +-45 deg | 0.807 | 0.715 | 1.13 | -27.5 | 1.12 m | 109 of 267 | refused |
| +-90 deg | 0.807 | 0.715 | 1.13 | -27.5 | 1.12 m | 109 of 267 | refused |
| +-135 deg | 0.807 | 0.715 | 1.13 | -27.5 | 1.12 m | 109 of 267 | refused |
| +-180 deg | 0.982 | 0.791 | 1.24 | +152.5 | 0.12 m | 267 of 267 | accepted |

Centring on the parked note instead gives the same four verdicts within a
hundredth. Widening to 90 or 135 degrees changes nothing at all, because the
answer is past both; only the full circle reaches it.

On the rover, `refit_pose` with `window_deg: 180` was accepted: 0.99 of the scan
on a wall over 269 of 269 returns, against a rival of 0.776, turning the rover
152.5 degrees and moving it 0.149 m. slam_toolbox landed it at `(-16.441,
-17.664, -100.4)`. Afterwards `map_settled` was true, `map_id` unchanged, and the
note on disk carried the corrected heading -- so the next boot starts from a good
pose. No code was changed to get this; the window is already a parameter the
control call accepts.

## Two failures the search made visible

**The refusal message pointed at the wrong knob.** It offered "a room that has
changed since it was mapped or a rover that has been moved further than this can
see", which is what sent a reader to the threshold. It had better evidence in
hand and did not use it. The winning pose sat at a 1.00 m offset in x within a
plus-or-minus 1.00 m search, and a peak pressed against the boundary is the
signature of a maximum outside it. It also explained 109 of 267 returns where the
true pose explains all of them.

**The score's normalisation does not survive rotation.** Returns over unmapped
ground are excluded from the average, and the code argues for this from
translation: over a metre, which candidate pose is chosen barely changes which
returns land on mapped ground. That is true and it does not carry across
heading, where rotating the scan sweeps an entirely different part of the house.
The table above is what that costs -- every impostor keeps a rump of 65 to 142
returns and scores well on it, while `MIN_POINTS` of 60 waves all of them
through. Taken over the whole scan the true pose scores 0.962 and the best
impostor 0.399, a margin of 2.4 rather than 1.06.

So a coverage guard is what would pay for a wider heading search rather than what
would be risked by it. That is a proposal and not a measurement: nothing here was
changed, and the wider window has been demonstrated on exactly one scan in one
room.

## The turn, and what it is evidence for

The saved note was written before the restart and says the rover was parked
facing 114.0 degrees. It truly faced 259.6. One commanded 90 degree turn in place
ran in between and reported "arrived". So the rover physically turned about 146
degrees when asked for 90, and its own instruments recorded 108 of that -- the
map heading was -0.8 before the turn and 107.3 after.

Both figures are inferred from stored poses rather than read from an independent
reference, and the pre-turn truth rests on the saved note being correct when it
was written. This is therefore evidence bearing on
[R-NAV-9](../requirements/navigation.md#r-nav-9) and not a measurement of it; that
requirement still wants a declared tolerance and a deliberate sweep from both
directions.

## What the world state did meanwhile

34 observations were recorded between the restart and the fit, every one stamped
with a heading that was eventually shown to be 152.5 degrees wrong. None of them
was placed: all 34 carry a null entity, and the most recent placement in the
store predates the restart by more than an hour. **The placed record is clean**,
and it is clean by luck rather than by design -- a parked rover cannot cross
bearings for want of parallax, which is
[R-WS-2](../requirements/world-state.md#r-ws-2) doing its job for a reason that
has nothing to do with the pose being wrong. The 34 rows remain in the pending
pool and become crossable the moment the rover drives.

Nothing in `world_state` or `rover_world.py` consults `map_settled`.
`rover_world._world_pose` refuses a direction when there is no map identity or no
fresh transform, gating on `position_trusted` -- which answers whether a pose
exists and is recent, never whether it is right. This is the code gap recorded as
[R-WS-16](../requirements/world-state.md#r-ws-16), and this entry is the
measurement it was owed.

## Which requirements moved

None changed state.

- [R-WS-16](../requirements/world-state.md#r-ws-16) stays `open`, and its
  blocked-by can now cite this entry rather than the code gap alone. The 34
  observations and their 152.5 degrees are measured here.
- [R-NAV-2](../requirements/navigation.md#r-nav-2) **held**, exactly as written.
  `map_settled` stayed false through a restore the mapper could not confirm, the
  saved pose was protected from being overwritten by the bad one, and it was
  rewritten only once a fit had settled the argument. Had the guard not been
  there, the wrong heading would have reached disk and the next boot would have
  restored to it.
- [R-NAV-3](../requirements/navigation.md#r-nav-3) **held**. The fit ran because
  somebody asked for it. Nothing searched on its own.
- [R-NAV-9](../requirements/navigation.md#r-nav-9) stays `open`, with the
  commanded-90-achieved-146 figure above as supporting evidence rather than as
  the measurement it asks for.

## What has to happen next

1. **Decide whether the heading window should open.** The evidence for it is that
   every localization failure this rover has recorded is a large heading error
   with a small position error: 81 degrees with 41 cm on 2026-09-06, 37 degrees
   with 10 cm at this morning's restore, 152.5 degrees with 15 cm here. The
   window allows a whole metre of position and 45 degrees of heading, which is
   generous where the errors are small and mean where they are large. It should
   not open without the coverage guard, and the pair of changes needs the
   recorded tuning set rerun, not one scan in one room.
2. **Give the refusal the evidence it already has.** A winning pose on the
   boundary of the search, or one explaining a small share of the scan, should
   say so instead of suggesting the room has changed.
3. **Decide what happens to the 34 observations.** They are unplaced and harmless
   while the rover is parked, and they carry a heading now proved wrong by 152.5
   degrees. Dropping them is a destructive edit to the store and is the owner's
   call.
4. R-WS-16's own fix is unchanged by any of this: withhold the direction when the
   pose is unconfirmed, and keep the picture.
