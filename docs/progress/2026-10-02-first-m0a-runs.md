# The first supervised M0a runs: no wrong answers, but too few answers to pass

**Across three supervised runs, 14 hypothesis inspections all ended inside their limits,
and none gave a wrong answer. But only two of them answered at all.** Both of those
answers were correct: something does stand at the toolbox and at the tissue box. Eight
attempts ended "can't tell", and four were refused before the rover moved. The minimum
sample, at least 20 attempts at 10 places with three absent and three occluded cases, was
not reached. The rover ran out of places it had not already answered or used up, and it
could not drive further to find more, because it was tethered to its charger. The three
places where the owner removed an object were attempted four times and never answered.
[R-AUT-12](../requirements/autonomy.md#r-aut-12) stays `open`.

The permission-expiry trial that M0a needs first passed the same session; see
`captures/m3-stop-2026-10-02/MANIFEST.txt`. With a 2 s permit and its program killed
mid-leg, the rover was stopped by the permit running out: at rest 0.54 s and 0.23 m after
expiry, half a metre short of its goal. The record of every attempt is in
`captures/m0a-2026-10-02/RUNS.txt`, with the executive's logs.

## What happened

A rebuild drive placed the six targets fresh under today's corrections. The toolbox came
out 0.11 m from the tape at 0.90 m high, against 0.845 taped. The painting above the
cabinet came out at 1.54 m, against 1.65. Then the executive ran with `--m0a`:

| Run | Conditions | Attempts | Outcome |
|---|---|---|---|
| 1 | the room as it stood | 7 | 2 supported, both correct; 3 unresolved; 2 refused at dispatch |
| 2 | the bucket and two dining chairs removed | 4 | 2 unresolved; 2 refused; ended on the battery at 10.92 V under load |
| 3 | the same, tethered, fenced to 2.2 m | 3 | 3 unresolved; then nothing left to try |

Why the eight "can't tell" answers happened:

- **Four had no depth picture.** The depth service was refusing connections, because
  the depth camera was dropping off USB and restarting; it did so even while parked on the
  charger. A supervisor change (`56b2729`, from another session) now restarts it in about
  2 s instead of 15. This session's `b73bcb3` retakes a check look once when its depth
  service was down. Neither had a chance to show itself before the runs ran out of places.
- **Three were outside the depth camera's view**, at both removed chairs. Those looks
  were taken at the raised tilt, and the place did not land in the depth picture. This is
  the open fault that keeps absent-target cases unanswered.
- **One found a surface but no region on the line.** That was the painting placed 0.68 m
  out in front of the cabinet: a wrong crossing, with the cabinet behind it.

The four refused attempts were refused by navigation, not the check. Two goals reached the
fence's stopping margin. Two had nowhere the rover's body fitted, because the map still
held the chairs that had been taken away.

## What the runs say about the hardware

When the check answers, it has been right. It mostly cannot answer, because each look
needs one depth picture of the place from 1 to 1.6 m away, and today that failed for two
reasons:

- the depth camera's reliability;
- aiming at places low or close to the floor.

The battery also shaped the session. Under load the pack sags below the 11.2 V reserve
every autonomous run keeps, so runs ended long before the battery was empty. The owner
asked for runs to go on until the rover shuts itself off. Lowering that reserve is a
change to a safety limit, so it was left for the owner to make.

## What is left for M0a

- Find out why check looks at low places miss the depth picture, and fix it.
- A drive that places more things, so there are fresh places to check, and absent cases
  made where the rover can see them: above the floor, in plain view, with half a metre
  clear around each.
- Enough battery to finish, either by charging or by the owner deciding on the reserve.
- Then the remaining attempts, to at least 20 at 10 places.

## Requirements

[R-AUT-12](../requirements/autonomy.md#r-aut-12) stays `open`: 14 attempts at about 8
places, no wrong answers, every limit held, and the absent and occluded minimums not met.
[R-WS-11](../requirements/world-state.md#r-ws-11) has its first on-rover height under the
correction: the toolbox at +0.06 m, and the painting at -0.11 m with its placement 0.25 m
off. That is two targets, too few to move it.
