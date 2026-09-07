# A look only gets a direction from a state that was measured

Two gates now stand between an inspection and a bearing, and both are on the
rover. A look is given a direction only when navigation says the rover's place
on its map has been *confirmed*, not merely that a pose exists; and only when
the gimbal was inside the pan range its own calibration covers. Outside either,
the picture, the regions and the appearance vectors are all kept and the
direction is withheld. A third condition — an angle reached from the wrong side
of the pan servo's backlash — is measured rather than unmeasured, so it widens
the bearing instead of removing it.

These are [M0](../plans/autonomous-curiosity.md)'s criteria 10 and 11. Neither
milestone gate is passed by this entry: criterion 11 still owes a demonstration
on hardware, and criterion 10 needs an envelope wide enough for the inspection
cases that matter, which this shows it is not.

## Confirmed poses, and why the old check could not catch this

The capture path already refused a direction when there was no pose, no map
identity or a stale transform. None of those asks whether the pose is *correct*,
and navigation answers that separately. After a restore the rover's place on its
map is the mapper's anchor until something confirms it, and an anchor that landed
somewhere else is neither confirmed nor disproved.

The recorded fault is the [refit window](2026-09-07-refit-window.md): a rover
came up on a restored map it could not place and recorded 34 looks between the
restart and the fit that corrected it, every one from a heading later shown to
be 152.5 degrees out. It had a fresh transform, a real map identity and a
confidently wrong direction, so every existing test passed throughout.

`rover_daemon/rover_world.py` now also requires navigation's `map_settled`, and
treats the answer being absent as unconfirmed rather than assuming the best. A
map the rover drew itself reports settled, because there the coordinates are its
own and there is nothing to doubt.

**The withholding is permanent, which is the half that is easy to get wrong.**
An observation's row is written once and nothing ever puts a pose back onto it,
so confirming the rover an hour later cannot make a bearing recorded before it
true. That is why the fix is a gate at capture and not a filter at read time,
and there is a check that says so.

## The gimbal envelope, and what the store says it costs

The pan campaign of 2026-09-07 validated commanded pan −20 to +20 degrees at
tilt zero, reached from the ascending direction. Two separate faults live outside
that, and they deserve different answers.

*Beyond the pan range the direction is withheld.* Past ±20 the servo's gain
error is not merely larger but unmeasured — one to two degrees by pan 30 on the
only sweep that went that far, and nothing at all is known about the pan 145 the
store holds eleven looks at. Inventing a cone for that would be worse than
recording none.

*Reached from the wrong side of the backlash the direction is widened.* The two
approaches to one commanded angle were measured to differ by 1.19 to 2.23
degrees, which is worse than the 1.5 the geometry is promised but is still a
number. It is carried as `bearing_sigma_deg` and spent by `locate`, exactly as a
look taken while turning already is. Refusing these instead would cost the rover
every bearing it takes while tracking a face, which moves the gimbal both ways by
its nature.

**Which way the pan servo last travelled is recorded now**, which it was not:
`world_state/README.md` had it as "recorded nowhere, so it cannot be corrected
afterwards". It is knowable only at the instant the command goes out, so that is
where it is captured, and it rides on the frame to the inspector.

**Rest is now an ascending arrival on purpose.** `Rover.centre_gimbal`
undershoots by 30 degrees and comes back up, the same manoeuvre the calibration
bench makes before every sample it takes. Of the 2162 observations this rover had
recorded, 1952 were taken at pan zero — leaving the approach to chance would have
put nearly every bearing it records on the unmeasured side of the backlash.

### What it costs, counted on the rover's own store

| | observations | share |
|---|---:|---:|
| pan inside ±20 | 1998 | 92.4% |
| pan outside it | 164 | 7.6% |
| of those, pan 30 | 126 | |
| of those, pan 52 to 145 | 20 | |

So the envelope costs the existing recording under a tenth of its looks. That is
the good news and it is not the whole picture.

## The envelope does not cover where the rover actually looks

The pan result was measured at tilt zero and only there. The rover rests at tilt
+20, because the camera is low and level fills the frame with floor. Counted on
the store:

| gimbal tilt | observations |
|---|---:|
| +20 (rest) | 1825 |
| 0 | 310 |
| +45 | 20 |
| +10 | 7 |

**84% of every look this rover has taken was at a tilt the pan campaign never
visited.** The lens itself is fitted across tilts up to 20 and the bearing
arithmetic undoes the tilt properly, so this is not an uncorrected geometric
error; what is uncharacterised is the *servo's* behaviour — its backlash and its
gain — at the tilt the rover actually uses. It is the same pan axis and the same
gearing, so it is likely to carry, but likely is not measured, and the point of
P0 is not asserting what has not been.

The gate is therefore deliberately on pan alone. Gating on tilt as well would
withhold the direction from more than nine looks in ten and satisfy criterion 10
by abstaining, which that criterion explicitly forbids. **Repeating the pan
campaign at tilt +20 is the cheapest thing that would close this**, and it is
the same procedure already written down, run with the gimbal 20 degrees up.

## Proved on the rover

`world_state` and `rover_daemon` deployed at 6bfd63a; their suites pass on the
Orin at 767 and 856 checks. Navigation reported `map_settled` true, with "the map
from the last session is back, and the rover is where it was parked", so the
confirmed-pose gate is passing ordinary traffic rather than blocking everything.

Four inspections through the deployed path, the rover standing at one pose the
whole time so nothing else could account for the differences:

| commanded pan | arrived | pose recorded | bearing | sigma | picture |
|---|---|---|---:|---:|---|
| 35 | from below | none | none | none | kept |
| −5 | from above | yes | −133.0 | 2.30 | kept |
| 12 | from below | yes | −129.4 | 0.00 | kept |
| 8 | from above | yes | −127.8 | 2.30 | kept |

The four looks at pan 35 have a frame each and no pose at all, while the rows
either side of them carry the same pose the rover held throughout. The status
line named the reason in each case, including "the camera reached that angle from
above, leaving the bearing good to 2.3 deg".

## What was deliberately left alone

`look_at` does not undershoot. Establishing the ascending approach costs a
settle, and a face being tracked cannot be reached from one side by definition —
so an aimed look records its approach honestly and pays for it in a wider
bearing, rather than being slowed down or refused.

## The store as it was before any of this

`~/.ugv/archive/` on the rover holds `world-2026-09-07-pre-clear.db` and
`frames-2026-09-07-pre-clear/`: 2165 observations, 131 entities, 841 of them
carrying a range, 428 frames, one map session. It is outside the deploy tree, so
a deploy will not touch it. It exists because clearing the semantic world takes
its pictures with it, and both the 838 ranges taken through the old mount and the
association decisions reviewed on 2026-09-07 are worth being able to replay.

## Requirements

- [R-WS-16](../requirements/world-state.md#r-ws-16) stays `open`. The gate is
  written, deployed and covered offline, and what it still owes is the hardware
  half the criterion asks for: a restart or refit on the rover shown to withhold
  directions while keeping the pictures.
- [R-WS-10](../requirements/world-state.md#r-ws-10) stays `failing` and is
  untouched. Nothing here measures the gimbal; it only stops the rover claiming
  accuracy in states the gimbal was never measured in.
- No other requirement moved.

## Next, in order

1. **Repeat the pan campaign at tilt +20**, which is where 84% of looks are
   taken. Same procedure as
   [the runbook](../runbooks/p0-gimbal-calibration.md), gimbal 20 degrees up.
   Until then the envelope covers a tilt the rover almost never uses.
2. **Restart navigation once during the next session** and confirm that looks
   taken before the map settles keep their pictures and record no direction.
   That is R-WS-16's hardware demonstration and it costs one restart.
3. **Clear the store, not the map** — `world_state_clear` empties the semantic
   world and leaves the pose graph alone — then take the acceptance recording
   with measured targets in the room, so criterion 2 can be checked rather than
   asserted.
4. Floor and background eligibility ([R-WS-12](../requirements/world-state.md#r-ws-12))
   and the movement-eligible identity band
   ([R-WS-13](../requirements/world-state.md#r-ws-13)) are analyses of a
   recording rather than changes to how one is taken, and both are better done
   on the fresh recording than on the store that is about to be cleared.
