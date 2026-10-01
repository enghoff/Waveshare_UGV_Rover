# A look's heading is now checked against the map, and the rover's driving is untouched

**After it turns, the rover still believes a heading tens of degrees wrong, but the
direction stamped on each look is now within a few degrees of a tape.** At three
moments in a 22-turn sequence, the rover believed headings 43, 40 and 17 degrees
out. The headings the world state stored for those looks measured -3.2, -3.5 and
-1.5 degrees against the owner's two-wall frame, scored by colour on the frames
the rover kept, as in [the turning measurement](2026-10-01-heading-after-turning.md).
The bucket reads about 3 degrees low throughout and the tissue box 0.4 high, which
looks like the bucket's taped point rather than the heading.

Deployed as `672a1b4` and `6dd4881` (ros_nav, world_state, rover_daemon; on-host
suites passing: ros_nav 550, world_state 898, rover_daemon 985). It is described in
[world_state/README.md](../../world_state/README.md) and
[headingcheck.py](../../world_state/headingcheck.py). Data and scripts are in
`captures/2026-10-01-heading/turn6` and `turn7`.

**It replaces the in-move refit that was [rolled back](2026-10-01-heading-check-rolled-back.md)
the same day, and it is built not to repeat that.** It moves nothing, corrects
nothing in navigation, takes no move mutex and waits behind no graph write. The
measurement answered in 0.08 to 0.26 s, a look took 0.7 to 0.9 s as before, and a
half turn stopped 0.5 s in still let go 0.04 s after the stop.

Two things the first deployment got wrong, found on the rover the same hour and
fixed in `6dd4881`:

- **Moving looks all went without a direction.** They waited for a check that
  found the heading right, and with navigation no longer corrected every check
  corrects instead. A moving look now takes the last check's correction while
  that check is fresh: under 15 degrees of turning since, and under half a metre
  of travel when the check had to correct.
- **The drift outran the search.** After two circles the heading was 46 degrees
  out, past the 45-degree window, and the look was withheld. The search now starts
  from the last correction found. On the rerun it found corrections up to 58.5
  degrees.

The first rerun is not used: the rover was caught in its charging cable and
dragged up to half a metre.

## What this does not show

It is turning on the spot at one place. A driven run through the room, with looks
taken on the way, is the acceptance test for
[R-WS-10](../requirements/world-state.md#r-ws-10). The carried rover of
[R-WS-16](../requirements/world-state.md#r-ws-16) should fail the check, because its
scan does not fit near where it believes it is. That is unproven until a rover is
carried.

## Requirements

None moved; both stay `failing` until the drive and the carry.
