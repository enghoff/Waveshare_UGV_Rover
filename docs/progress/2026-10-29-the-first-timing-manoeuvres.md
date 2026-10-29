# The first timing manoeuvres: the lidar holds against the OAK, the board does not, and odometry ran 2% short while turning

**Three recording sessions on 2026-10-29, at manual exposures of 10, 5 and 20 ms,
each opening and closing with Perception's timing manoeuvres, dropped nothing and
passed Perception's checker on order, boundary and schema.** The checker's timing
item is the finding. The lidar's lag against the OAK's gyro held within a few
milliseconds from each session's opening to its closing. The board's could not
be estimated: its lag against the OAK came out 32 to 105 ms from one turn to the
next. Odometry averaged 19.43 updates a second in the 20 ms session, under the
19.5 that [R-CTL-13](../requirements/control.md#r-ctl-13) asks; whether
recording causes that or turning does it anyway is not yet known. The owner
handed the rover over unplugged at 14:13 IST, with the bedroom door shut; the
recorder was at `07a7ab1`
([plan](../plans/recording-for-perception.md)).

## What was run

Parked at the charger, from this desktop. The recording calls went through the
console's `/recording` endpoint, which the page's buttons use, and the turns and
the gimbal's centring went to the daemon. Each session:

- *Chassis turns*, opening and closing: six turns on the spot, 30 degrees left
  and right alternately, 1.5 s still between, the gimbal locked at its centre.
- *Gimbal steps*, opening and closing: 8 pan steps of 16 degrees around pan −10
  at tilt 20, then 8 tilt steps between 20 and 10, equal numbers each way,
  facing the dining set and the paintings, which is the textured part of the
  view from the charger.
- In the 10 ms session only, *blur sets*: 16-degree pan steps at the servos'
  fastest (T:133 SPD 0), then at 400 and 150 counts a second (T:134).

A 2.5 ms session was refused: the camera's shortest exposure is 5 ms. All
frames in a short 10 ms session the same afternoon were marked manual, and the
camera went back to automatic at its end.

## What the checker found

| Lag | What it says |
|---|---|
| Board against the OAK | unknown in all three sessions: per-turn lags of 32 to 105 ms, and the board's lines not steady enough to use a clock of their own |
| Lidar against the OAK | held from opening to closing within −2.4 to +2.7 ms; "unresolved" only because its interval reaches past the margin |
| Lidar against the board | disagrees turn to turn, the board's spread again |
| Camera arrival against the OAK | 14 to 24 ms over the gimbal steps, one step at −20 ms |

The opening and closing turn units were ineligible, and that was the script's
doing: it marked the first unit about a second after the session's start and
ended the last 0.02 s before the session's end, so the checker's windows (a
press's two-second uncertainty and a further second either side) reached before
the ROS streams had begun and past the session's end. It now waits 4 s either
side.

What is the rover's:

- *Odometry 19.43 a second* in the 20 ms session against the board's feedback
  at 19.88, with no gap longer than 0.15 s, so a steady shortfall rather than
  stalls. Parked, on 2026-10-28, odometry ran at 19.82 while recording. A run of
  the same turns with nothing recording would tell whether recording costs it.
- *Camera gaps* of 0.2 to 0.27 s at the starts and ends of moves, a few per
  session.
- *Turns over-run.* Twelve alternating 30-degree turns left the rover 34
  degrees left of where it began, and an 8-degree correction turned it 33. It
  was turned back to within about 3 degrees of its starting heading.

## What it does not show

Nothing drove anywhere: the turns were on the spot. The sessions are copies on
this desktop and on the rover in `~/recordings/`.
