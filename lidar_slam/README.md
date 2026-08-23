# `lidar_slam/` — 2D SLAM and self-driving on the rover's own Pi

Scan-matched localisation, an occupancy grid, and a drive controller that will not
run into things — from the D500 lidar alone, on the Raspberry Pi 1 bolted to the
rover. It exists because that Pi already has both sensors wired to it and no map to
show for them, and because a map is only worth computing somewhere that can act on
it — which means on the rover itself.

The scope is deliberately small. This finds where the rover is relative to where it
started, keeps a grid of what is solid around it, drives it there without hitting
anything, and describes the result in terms a language model can use. It is not
enough to close a loop and it does not try — see
[What is deliberately missing](#what-is-deliberately-missing).

Measured on the rover's Pi against the real sensor, 20 seconds, stationary:

```
198 revolutions in 20.0s (9.9 Hz), 0 dropped, 0 matches rejected, worst loop 41.9 ms
```

## Why this is C

Because the host is a 700 MHz single-core ARM1176 with scalar VFP and **no NEON**:

```
Features : half thumb fastmult vfp edsp java tls
```

No SIMD, one core, and 474 MB of RAM. The arithmetic in a correlative scan matcher
is not large, but it is spread over arrays of a few hundred elements, which is
exactly where numpy's per-call overhead stops being amortised. Measured on this
host, same algorithm, same 400×400 grid at 5 cm, 500-point scans:

| stage | Python + numpy 2.2.4 | C, `gcc -O2` |
|---|---|---|
| parse + CRC-8, one revolution of 42 packets | 24.7 ms | 0.05 ms |
| scan match, 637 candidate poses | 477 ms | 87.7 ms |
| scan match, coarse→fine, 90 poses | 90.8 ms | 13.2 ms |
| occupancy update, 500 rays | 114.7 ms | 9.4 ms |
| **parse + coarse→fine match + map update** | **231 ms** | **22.8 ms** |

A 10 Hz sensor gives you 100 ms, so the Python column does not fit and the C column
does. The parse is the most lopsided row at roughly 500×, and it is worth
understanding why rather than filing it under "C is faster": checking a CRC is 46
sequential table lookups that cannot be vectorised, so the numpy version is 46
numpy calls on a 42-element array and came out *slower* than the plain Python loop
it was meant to replace. Reaching for numpy is the instinct that fails here.

What ships is not that benchmark's configuration — the search window and point count
were tuned afterwards — so `selftest` measures the real thing end to end:

```
parse + CRC, 35 packets                  0.30 ms
map update + 1 pose                      3.98 ms
scan match                              17.27 ms   (300 poses, 0.058 ms each)
TOTAL per revolution                    21.55 ms    21.5% of one core -> fits
```

**That 21.5% is the number to plan around, not the 100 ms budget.** The core is
already committed elsewhere: forwarding 640×480 MJPEG over the WiFi dongle costs
about 30% of it, and the link saturates the CPU at 72% before it saturates itself.
SLAM and video at full frame rate do not both fit, and that is the trade to make
consciously rather than discover.

## What is deliberately missing

**Loop closure, and with it any globally consistent map.** The cost is not marginal.
Each candidate pose costs a measured 0.058 ms, and the search window `slam_toolbox`
uses by default — `loop_search_space_dimension: 8.0` at `resolution: 0.05`, so
160×160 offsets across 13 angles — is 332,800 poses, or about **19 seconds for one
closure attempt** on this host. Branch-and-bound of the kind Cartographer uses would
change that answer, and hand-rolling it is a much larger project than this.

So the pose here drifts, monotonically and for ever. Drive a circuit and the two
ends of it will not meet. Everything downstream should treat the pose as good
locally and untrustworthy globally: fine for "is there a wall 40 cm ahead", wrong
for "return to where I started".

**An unbounded map.** The grid is allocated once at a fixed size — 800×800 cells at
5 cm, so 40 m square with the rover starting at the middle of it, which is 20 m of
reach in every direction from wherever it was switched on. Past that edge a room is
driven through and not written down: the beams fall outside the array and are
dropped, so the map shows a dead straight line with never-seen grey beyond it. That
line is the map running out, not a wall.

It is a fixed size rather than a growing one because there is nothing to gain from
growing it. A revolution's work is the scan and not the map — each beam is walked a
cell at a time and each hit stamps a small kernel around itself, so the cost follows
the ranges the sensor reported and not how much grid is lying around them. Widening
it therefore costs memory and nothing else: two bytes a cell, 1.3 MB at this size,
against 474 MB of RAM. Measured on the Pi, doubling the grid from 400 cells moved the
cost per revolution from 58.23 ms to 58.10 ms, which is to say not at all — the
matcher's working set is the room around the rover either way. So it is set large
enough for a floor of a house and left alone. `run_slam.py --cells` changes it for a
run; `slam2d_default_config` is where a rover that needs a different one should say
so, and `test_timing` in `selftest.c` is what to read afterwards.

## Building and running

The library is compiled per-machine and is not committed — nothing else in this
repository is armv6, so there is no cross-compiler and a checked-in binary would
only ever be wrong.

```bash
scp lidar_slam/* rpi:~/ugv/lidar_slam/
ssh rpi 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
ssh rpi 'cd ~/ugv/lidar_slam && python3 run_slam.py --seconds 30 --sectors 37 --map room.pgm'
```

`run_slam.py` prints a line a second and draws clearance as a bar with forward in
the middle:

```
t=  6.0s scan   60  x=+0.000 y=+0.000 th=  +0.0deg  score 0.98  pts 278  ahead  2.63m  drop 0
   @@@@  @@@.. .   ::-- --   :..... ..@@
```

`score` is the mean likelihood under the matched scan, 0 to 1: 0.98 stationary
against a mature map, and a run where it sags toward `min_match_score` (0.15) is a
run that is losing its position. It is **not** a sufficient health check on its own
— a scan that has snapped onto the wrong alignment scores high, because scoring high
is why that pose won — so see [When the match is
wrong](#when-the-match-is-wrong-and-how-it-says-so) for the two numbers that catch
what it cannot. `drop` counts revolutions thrown away because the loop fell behind —
it should stay at 0, and if it does not, the Pi is oversubscribed rather than the
SLAM being slow.

If nothing arrives at all, the rover's power switch is the first thing to check: the
port enumerates without it, because the CH343 is USB-powered, but the lidar's motor
runs off the 5 V rail behind the switch. A live port and no packets means the switch
is off, not that the cable is wrong.

## The ports, which are not the ones the docs used to name

Two separate serial ports, and they are easy to confuse:

| | port | baud | what |
|---|---|---|---|
| lidar | `/dev/ttyACM0` | 230400 | D500 point stream, one-way, unprompted |
| driver board | `/dev/ttyAMA0` | 115200 | `T:1001` telemetry and motor commands |

The lidar is a **`ttyACM`**, not a `ttyUSB`: it is a CH343 (`1a86:55d3`) behind an
FE1.1S hub (`1a40:0101`), and `cdc_acm` claims it. [`docs/hosts.md`](../docs/hosts.md)
asserted for a while that this Pi had neither, which was simply wrong and is now
fixed.

`ttyAMA0` can only have one owner, and `rover_daemon.py` normally *is* that owner.
That is why `run_slam.py` does not touch it unless asked, and it is the main reason
this code should eventually be called from inside the daemon rather than run beside
it.

## Frames, since a sign error here is invisible and expensive

The sensor reports a **left-handed** bearing: zero at the front of the sensor, angle
growing clockwise, in hundredths of a degree. The rover frame is right-handed with
**x forward, y left**, and yaw counter-clockwise. The lidar is also mounted 90° off
the chassis.

Both corrections collapse into one angle, `phi = mount_deg - bearing`, applied once
when the sin/cos lookup table is built, so no sign fixing survives past
`build_lut()`. With `mount_deg` at 90 this reduces to `x = r·sin(bearing)`,
`y = r·cos(bearing)` — the same geometry [`lidar/lidar_view.py`](../lidar/lidar_view.py)
draws with its `VIEW_ROTATION_DEG = 90`.

`selftest` pins this down rather than trusting it. It synthesises scans inside a
6 × 3 m room and checks that the sector astride each axis reports that wall's real
distance:

```
sector 0, straight ahead (+x wall)     4.0000 (want 4.0000 +/- 0.0500)  ok
sector 18, rover's left (+y wall)      1.5000 (want 1.5000 +/- 0.0500)  ok
sector 36, behind (-x wall)            2.0000 (want 2.0000 +/- 0.0500)  ok
sector 54, rover's right (-y wall)     1.5000 (want 1.5000 +/- 0.0500)  ok
```

A mirrored frame or a wrong mount offset permutes those four and nothing else
notices.

## How the matching works

A **correlative scan matcher** over a likelihood field, in two passes.

The field is a `uint8` grid alongside the occupancy grid. A beam that ends in a cell
stamps a Gaussian at σ = 1 cell over it, taking the *maximum* rather than adding, so
a wall seen on twenty consecutive revolutions is not twenty times more attractive
than one seen once. A beam that passes *through* a cell decays that cell's
likelihood, which is what lets an obstacle that moves away stop pulling the match
toward where it used to be.

That 10 cm of smear is much wider than the sensor's ±20 mm accuracy, and it is not
modelling the sensor — it is widening the basin the search has to fall into, so the
coarse pass can step a whole 5 cm cell without walking past the peak.

The coarse pass spans ±0.10 m and ±9°, which at 10 Hz is 1.0 m/s and 90°/s — the
angular window earns its width because a rotation past the edge comes back
*under-reported* rather than rejected, and a controller closing on that keeps
turning. The fine pass then spans one coarse step. 5×5×7 + 5×5×5 = 300 poses, and
points are thinned to 300 of the ~419 the sensor delivers because every point costs
a cache miss in every one of those poses; thinning bought 25 ms a revolution.

A match whose mean likelihood falls below `min_match_score` is **rejected** and the
motion prior is used instead. Writing is a higher bar: `min_write_score` (0.35) and
a winner that did not sit on the rim of the window. Believe and write are different
questions — a 0.16 fit is still better than the prior, but it is not a pose to stamp
walls from, and the first scan — the first that saw anything, a revolution with no
returns does not count — is exempt because it *is* the map. That first scan has to
be a real revolution. The port opens onto a sensor that is already spinning, so the
first wrap is a remnant of the turn we joined in the middle of; stamping that
remnant is how a restart used to leave a wedge of map that later full scans would
not match (a 0.16 score, mapping held) until someone cleared it. A wrap that has
not covered 270 degrees is discarded, and the next one becomes the map. This
matters more than it looks: dead reckoning drifts predictably, whereas a
confidently wrong match teleports the rover and then corrupts the map it will be
matched against next.

## When the match is wrong, and how it says so

The score is not a health check on its own, and treating it as one is what produced
maps with the room stamped in twice at an angle to itself. **A scan that has snapped
onto the wrong-but-self-consistent alignment scores high** — scoring high is
precisely why that pose beat the others — so a confident mistake and a good fix are
indistinguishable in it. `min_match_score` catches "this scan matched nothing
anywhere", which is a dead sensor or an empty map, not "this scan matched the wrong
thing". Worse, the update then stamps the bad pose into the map, after which the map
agrees with the error and the score *recovers*.

Two more numbers come out of the same search, which already computed them and used
to throw them away.

**`slam2d_match_edge`** is 1 when the winning coarse candidate sat on the rim of the
lattice. The window spans what the rover can move in one revolution, so a winner
against its edge means the true pose was probably outside it and what came back is
the boundary of what was searched rather than a fit. This is the failure the coarse
window's own comment warns about — rotation past the window comes back
*under-reported* rather than rejected — made visible instead of silent.

**`slam2d_ambiguity`** is the best rival peak as a fraction of the winner, comparing
only headings at least `ambiguity_sep_deg` (20°) away. At the centre of a rectangular
room a half turn maps the room exactly onto itself; the self-test measures 0.97 there
against 0.58 from an off-centre pose. Both fit beautifully and the score cannot say
which is right. It reads 0.0 during ordinary tracking, because a ±9° sweep cannot
hold a rival 20° out — this is a number to read after a recovery search, not every
revolution. A turn treats 0.60 as the bar for believing the heading; mapping takes
the winner once the pose holds still, rather than staying paused in a rectangle.

**`slam2d_angle_profile`** hands back the whole correlation curve against heading,
which is the artifact worth logging when a map comes out wrong: a peak against the
end of the sweep is a window too narrow, two comparable peaks are a room that does
not say which way round the rover is, and one low broad hump is a scan with nothing
in it worth matching.

### Two rules that keep a bad pose out of the map

**A rejected match is never written, and neither is a weak one, and neither is one
that won against the rim of the window.** It used to be that anything not rejected
was stamped. The likelihood field takes the maximum, so a bad stamp lands at full
strength — as attractive to the next revolution as a wall seen all afternoon — and
from then on the wrong answer has evidence for it. One stamp is enough, `lik_decay`
needs about thirty-two clearing passes to erase it, and with no loop closure nothing
ever repairs what is left. The pose is still allowed to follow an edge winner, so
the window can walk toward the truth; it is just not allowed to draw while it does.

**Mapping can be suspended without suspending the matching.** `slam2d_set_mapping`
is for the caller that has just moved the pose itself and cannot yet vouch for where
it put it. Matching goes on, so the matcher can find its way back; nothing is
written until the caller says so. The cost of a wrong re-seed becomes a few
revolutions of pose and no map at all. The navigator also holds the map on the first
untrustworthy revolution *outside* a turn: C already refused to stamp that one, but
the next will not get a wide search unless mapping is paused.

### The recovery search

`slam2d_request_recovery` widens the next coarse pass, once, to ±60° and ±0.05 m —
41 candidate headings against the tracking window's 7. It exists because the tracking
window is sized for what the rover can move in 100 ms and is hopeless for re-finding
a pose somebody else moved: a dead-reckoned turn on this rover has been observed 48°
out, five times the coarse window, which the match cannot climb back from on its own.

Wide in angle and deliberately narrow in translation, because a rover turning on the
spot errs by tens of degrees in heading and by centimetres in position — and because
cost goes as the *square* of the linear steps and only linearly in the angular ones.
369 poses against the coarse pass's 175, so a recovery revolution costs about half as
much again as a normal one (measured 97 ms against 59 ms, both with the daemon also
running, so both inflated against the idle table above). It is asked for until the
first healthy match, then the confirming revolution uses the ordinary tracking
window: two independent ±60° answers can lock onto different peaks, both scoring
beautifully, which is not the matcher agreeing with itself.

**Only for a rover that was moved without being watched**, which in practice means a
dead-reckoned turn. Narrow in translation is the right trade for that rover and the
wrong one for a rover that lost the pose at 0.25 m/s: ±5 cm is less than a driving
rover covers between two matched revolutions, so the sweep lands on the rim, which
holds the map, which asks for another sweep. Five recorded drives, 999 revolutions:

| the revolution matched with | outran its translation window | landed on the rim |
|---|---|---|
| the tracking window, ±10 cm | 0% | 11% |
| the recovery sweep, ±5 cm | 49% | 80% |

A third of every drive ran the sweep and mapping was held for 38–67% of each move,
almost all of it that latch rather than anything wrong with the room: 334 of the 335
vetoed revolutions were on the rim, and nothing else came close — six also scored
below the write threshold and one was rejected outright. So the navigator now holds
the map *without* widening the search (`WIDEN_AFTER_LOST`): a rover the matcher was
tracking a revolution ago is a few degrees out, not tens, and the ordinary window
reaches that. It widens after four revolutions of tracking that cannot find the
pose — which is the case the sweep was built for — and a burst still gets it at once.

Measured in the self-test, on a pose deliberately put 35° out:

```
  normal window                               35.0 deg out, score 0.14
the ordinary window cannot reach the answer                                    ok
and says so: the winner sat on the rim of the window                           ok
  recovery window                              0.5 deg out, score 0.90
```

## The gyro: a prior, and a witness

The driver board's `T:1001` telemetry has always carried rather more than the lidar
can offer — a 9-DoF IMU in `ax/ay/az`, `gx/gy/gz`, a magnetometer in `mx/my/mz` that
the OAK-D-Lite's BMI270 does not have, and wheel encoders in `odl`/`odr` — and for a
long time nothing read any of it. [`odometry.py`](odometry.py) reads it now, and does
two jobs with it that are worth keeping apart, because they have opposite requirements
and only one of them was ever blocked on a measurement.

### As a prior, which needs a scale factor

The prior only centres the search window, so it is genuinely optional — at a walking
crawl the true motion is inside the coarse window anyway, and every measurement above
was taken with no prior at all. `selftest` covers both regimes, including the one
where the prior stops being optional:

```
--- driving past the search window: 30 cm a revolution ---
  without prior     worst x error 3.000 m
  with prior        worst x error 0.000 m
```

**Two scale factors are needed**: the encoders' counts per metre, which depends on
gearbox and wheel, and the gyro's LSB per deg/s, which depends on the full-scale range
the firmware picked. Guessing either produces a prior that looks plausible while
quietly dragging the match off true — strictly worse than no prior — so until they
have been measured the prior is exactly zero and the rover drives as it always has.

They measure themselves, out of moves the rover makes anyway, and that is the pleasing
part: the scan matcher's heading is the absolute reference the gyro has never had, so
a `turn_in_place` the rover made for its own reasons is also a calibration. `_refind`
already confirms every burst — a recovery sweep and the tracking revolution after it
landing within 5° and 5 cm of each other — and it is that confirmation, not the size
of the turn or how close it came to what was asked, that makes the heading a
measurement rather than the re-seed repeating itself back. Drives do the same for the
wheels. The numbers land in `~/ugv/odometry.json`, outside `lidar_slam/` so that
`scp lidar_slam/*.py` cannot overwrite a measurement, and the moves behind each fit
are saved with it so a restart continues the measurement instead of starting a new
one from whatever the next three moves happen to be.

The gyro's scale is kept **signed**, and the sign is the point: nothing on this rover
documents whether a positive `gz` is a left turn or a right one. It turns out to be
counter-clockwise, the same way round as the pose, and the first confirmed turn
settles that along with the magnitude.

**A drive is measured against the path, not against the distance between its ends.**
That was the difference between measuring the wheels and never measuring them: this
chassis wanders, and a 1.5 m drive that arrives 23° off its start heading had its
`travelled` — the straight line — refused by a curvature gate for three attempts
running. The wheels rolled every centimetre of the wander, so what they should be
compared against is the path the matcher traced, accumulated a revolution at a time
along the heading of the moment. Signed along the heading rather than as a distance,
because taking the absolute value of each step rectifies the matcher's own
few-millimetre noise into centimetres of travel that never happened.

**Neither fit is published until its moves differ from each other**, which mattered
more than how many there were. Three 175° turns the same way round agreed with each
other to 2% and disagreed with a mixed set by 10% — the residual was measuring how
well three near-identical manoeuvres repeat, which is a different question from
whether they are right. So a fit is held back until its moves either go both ways or
differ in size by a factor of 1.8, and `nav_status` reports `gyro_varied` and
`ticks_varied` beside the values.

### What the rover actually measured

Driven on 2026-08-22, five varied turns and five varied drives:

```
gyro   15.34 LSB per deg/s   5 turns,  worst residual 8.7%
wheels  108   ticks/metre     5 drives, worst residual 14.5%
```

Both are far better than the prior needs. Its job is to centre a window spanning
±9° and ±10 cm, so a 9% error on a 5° step is half a degree — the accuracy that
matters here is "within a fraction of the window", not "to the last percent".

Two structures are visible in the raw pairs, and both are worth knowing before
anybody tries to tighten these numbers:

- **The gyro-to-matcher ratio depends on which way the rover turns.** The two
  left turns read 16.5 and 16.7 LSB per deg/s; the two right turns read 14.5 and
  14.2. That is not scatter, it is a sign asymmetry, and a single signed slope
  averages it to 15.3 and is then about 8% wrong in each direction, alternating.
  Fixing it means two scale factors, or finding out whether the asymmetry is in the
  gyro or in the matcher's own heading.
- **Short drives read long.** 0.51 m came out at 124 ticks/metre and 1.49 m at 103,
  monotonically in between. That is a fixed overhead per drive — the slip while the
  wheels break away and while they stop — being divided by a shorter distance. The
  fit is weighted by move size and so lands near the long-drive asymptote, which is
  the right answer for a rover that is driving rather than starting. Modelling it
  properly means fitting an offset as well as a slope, rather than a ratio through
  the origin.

This is also why the first attempt at the wheel scale was nonsense: three 0.5 m hops
gave 223 ticks/metre and five gave 117, because a hop that short is mostly overhead.
Long drives were what settled it.

### As a witness, which needs nothing

This is the more valuable half. The match score cannot detect a scan that has snapped
onto a wrong-but-self-consistent alignment — scoring high is precisely why that pose
won — and once the map is stamped from the bad pose the map agrees with the error and
the score *recovers*. That is the whole mechanism behind a room welded in twice at an
angle, and every number the matcher produces is downstream of the same search, so none
of them can contradict it. The gyro is the only thing on this rover that is not the
scan matcher.

It needs no scale factor for this, only a threshold, and the threshold measures itself
while the rover stands still — which is most of what it does. Measured live on the
rover on 2026-08-22: a resting bias of **7.3 to 7.6 LSB** on `gz`, consistent with the
6.9 recorded a week earlier, and a resting spread of **1.8 to 2.3 LSB**, learnt over
several hundred stationary revolutions within a few seconds of start-up. Six of those
spreads is the bar for believing the chassis turned. The bias is re-learnt
continuously and deliberately *not* saved between runs, because it drifts with
temperature and a stale one would be believed.

Two contradictions are checked, and the second is the one that is easy to miss:

- **A jump.** The match moves the heading five degrees or more in one revolution
  while the chassis, according to the gyro, did not turn at all.
- **A creep.** The match accumulates fifteen degrees across at least eight
  consecutive revolutions the chassis spent standing still. Two degrees a revolution
  never trips the first test and is twenty degrees wrong in ten seconds. An honest
  match's own noise is a few tenths of a degree and cancels rather than accumulating,
  so it does not reach the bar.

A third, comparing the *direction* the two report, lights up once a turn has
established the sign convention.

The response to any of them is deliberately the one a weak match already gets: hold
the map, keep matching, resume when two revolutions agree. A disagreement does not say
which of the two is wrong, only that they cannot both be right, and holding the map is
the safe reading of that. The witness is only consulted while the map is being written
and no recovery sweep ran — a sweep and a post-turn re-seed both move the heading tens
of degrees over a stationary chassis quite legitimately, and both live on the
already-paused side of that test.

`unknown` is a third answer and is not a failure: no threshold learnt yet, a hole in
the telemetry, a board on WiFi with no stream to integrate. A caller reading it as
"the chassis was still" would manufacture the very disagreement this exists to detect,
on every revolution, from a cold start.

### Reading the board without paying for it

The stream runs at **~20 Hz**, measured at 19.9 by draining `in_waiting` in bulk — a
`readline` loop with a 0.2 s timeout reports 17, and the missing sixth is the reader's
fault rather than the firmware's. Even 20 Hz is slow for a gyro: a 60°/s turn advances
3° between samples.

Getting that stream onto this host cost more thought than reading it did, and the
numbers are worth keeping because the obvious design is the expensive one. The core is
single and the scan matcher has most of it, so what a second reader costs is not the
work it does but the number of times it wakes: every wakeup preempts the matching loop
and forces a hand-off of the interpreter lock. Measured against the rover's real rate,
40 seconds a round, daemon otherwise idle:

| how the board's stream is drained | scan rate | revolutions dropped |
|---|---|---|
| nothing reads it, as it was before | 9.6 Hz | 2% |
| a read that returns as soon as a byte lands | 8.7 Hz | 10% |
| drained on a 50 ms clock | 9.34 Hz | 5% |
| drained by the navigator's own loop | 9.34 Hz | 5% |

The second row looks like the attentive design and is the trap: at 115200 baud a line
takes 13 ms to clock in, so a reader that returns the moment a byte arrives spends
that whole 13 ms going round again for the next one, twenty times a second — 12% of
the core spent re-reading a line still in flight. What ships drains from the
navigator's loop, which was going to run anyway and so costs no wakeup at all, with a
250 ms thread as a backstop for a daemon started without `--lidar`, where the pack
voltage is all anyone wants out of the stream.

**So reading the gyro costs about 3% of the scan rate**, and that is the standing
price of everything above.

Two details in the folding are load-bearing rather than tidy. Lines drained together
share the elapsed interval between them rather than each being stamped when it was
parsed: the board samples on its own fixed clock, so two lines pulled from the buffer
at once were taken 50 ms apart however close together they were read, and stamping on
arrival would hand the first one the whole interval and the second none of it. And an
interval nothing was awake for is *counted* rather than integrated, because a yaw rate
multiplied by a gap is rotation that never happened; a span containing one of those is
refused outright by both the prior and the witness. An interval of zero is neither a
gap nor a sample — two pumpers share this port, so two drains landing in the same
instant are ordinary, and calling one a hole would quietly switch off the prior and
the witness for the span around it. That last one only showed up on the Pi: the same
test passed on the workstation, which was fast enough that the two drains never landed
in the same microsecond.

## Driving

`navigator.py` owns the lidar, the SLAM core and a 10 Hz control loop, and turns a
request like "forward 1.5 m" into motor PWM. It does not own the driver board: the
caller passes in something with a `.send(dict)`, which on the Pi is the daemon's
`SerialLink`, so there is still exactly one owner of the UART.

**The scan matcher is the encoder this rover does not have.** Driving is open-loop
PWM in ±255 because there are no wheel encoders fitted, equal PWM is not equal speed,
and below `MIN_PWM` the motors only buzz. A speed in metres per second would be
meaningless — except that the matcher measures the real displacement every 100 ms, so
speed, distance and turn angle all close on that instead of on the motors. It is also
what makes `turn_in_place(90)` mean ninety degrees rather than a guess at a duration.

**Avoidance reads the live scan, never the map.** The map drifts and holds geometry
that has since moved; the current revolution is 100 ms old and still right when the
pose estimate is not.

Three checks stand between a request and the wheels:

- **A swept-arc corridor, not a standoff circle.** `slam2d_arc_clearance` asks how far
  the rover can travel along the arc it is actually about to follow before that
  corridor is blocked. A circle would forbid rotating away from a wall inside it,
  which is the one manoeuvre that gets a rover out of a corner, and would read a wall
  met at a shallow angle as an obstacle when it is a thing to slide along.
- **Braking distance, not a trip wire.** Speed is capped at what can still stop by the
  30 cm standoff, and the decision point is 15 cm short of it: a revolution is 100 ms
  of sweep, `slam2d` only completes it when the next one begins, and the motors then
  take time to stop. Measured, that chain is over 200 ms, which is 7 cm at full speed
  before spin-down is counted.
- **Unknown is not clear.** Roughly a sixth of this sensor's beams return nothing, and
  a beam returns nothing off matt black fabric as readily as off open air. Empty
  sectors read as unknown and cap the speed at a crawl rather than reading as free.

Turning on the spot is allowed in tighter places than driving is, because it does not
translate — the test there is the chassis' circumscribed radius, not the standoff.
That test looks back over the last five revolutions rather than the newest one, and
that is not conservatism for its own sake: testing only the newest let a turn start
beside something 0.13 m away and run for nearly four seconds before a scan happened
to catch it again. On a bench that would have been the rover spinning off the edge.

**A turn is dead reckoned, and then checked.** The matcher cannot follow 170°/s, so
each burst runs blind with matching suspended, and the pose is re-seeded afterwards
from the rates in `TURN_RATES`. That re-seed *tells* the matcher where it is and it
cannot argue — which is how a turn that physically managed 42° of a requested 90 came
back reported as 90. So the re-seed is treated as a hypothesis rather than an answer:
mapping stays suspended, a recovery search runs, and the map is written again only
once a recovery sweep and the tracking revolution after it have landed within 5° and
5 cm of each other. A rival heading (ambiguity at or above 0.60) still makes the
turn come back `lost`, because that number was added to catch a rectangle that fits
two ways round; mapping takes the winner rather than staying held for the rest of
the run.

Checked **per revolution**, which is the part that used to lose. The old code slept
0.7 s and then read the score once, by which time seven scans had already been folded
into the map at whatever heading the re-seed invented — and since `integrate` ran
whether or not the match had been rejected, being lost was no protection. That is the
whole mechanism behind a map with a second copy of the room welded in at an angle.

Long turns go in bursts of `TURN_BURST_MAX_DEG` with a measurement between them,
because a dead-reckoned error is a fraction of the burst it came from, and a whole
180 guessed in one go can land outside what the search can undo.

If the re-seed is never confirmed the move comes back `lost`, saying which of the
three things went wrong. The map is held until two revolutions agree on a pose, even
when those two were the better of two answers the room offered. `status()` carries
`mapping`, `match_edge`, `heading_ambiguity` and a short `slam_events` history, which
is what lets the two failures be told apart after the fact — a rover that could not
keep up shows dropped revolutions and an answer against the rim of the window, while
a room that looks the same two ways round shows a rival peak and no drops at all.

The rim is the one worth watching. Every pose jump over 6 cm in the recordings sat on
it, 166 of 178 window overruns were rotation rather than travel, and it was the sole
reason mapping was ever held. Two things feed it and both are about the loop being
slower than the sensor: `MAX_TURN_DPS` assumed a revolution every 100 ms and the
recordings measured 138 at the median and 236 at the ninetieth percentile, which
turns 45°/s into 10.6° of rotation against a ±9° window — see the turn cap below —
and the recovery sweep's ±5 cm, above.

Steering is follow-the-gap — the heading with the most room, penalised for departing
from the one asked for. Wall-following at a shallow angle falls out of that rather
than being a special case with a threshold to tune.

Going to a *place* rather than a distance — `drive_to`, which is what a tap on the
map becomes — plans on the occupancy grid first: A* at cell resolution over the map
inflated by 45 cm, with unknown treated as blocked, thinned to the handful of
corners that change heading. 45 cm is the distance at which the follower actually
stops (30 cm standoff plus 15 cm of reaction). If that ring has no route, planning
falls back to 25 cm — a sideways gap, not the along-track brake, because inflating
every attempt by 45 cm asked for a 90 cm opening and refused pinches the chassis
still fits, with the live scan clear down the middle. The follower still keeps
30 cm ahead and brakes 15 cm early; it just is not asked to pretend an 85 cm
doorway is closed. A soft toll beyond the keep-out is a nudge toward the middle of
a gap, not the thing that keeps corners at arm's length: two extra cells of path
was a cheap price to scrape a corner when going around cost metres.

**A place can be said two ways, and which one is used decides whether a click can
interrupt a move.** `ahead_m` and `left_m` are measured from wherever the rover is
standing when the call arrives, which is what a caller looking at the room in front
of it wants. `x_m` and `y_m` name a point in the map's own frame — the frame the
pose is reported in and the frame the map picture is drawn in — and that is what a
caller wants when the rover may move between choosing the place and the call
running. The console's map is exactly that case: a click that interrupts a move has
to stop the move first, and the rover keeps driving until the stop lands, so the
same pixel means one fixed place absolutely and a place adrift by most of a metre
relatively — further out, the faster it was going. So every tap goes as a point.
Only a console is offered the pair, because nothing a model can see says where the
rover is in that frame; see `_tool_drive_to` in
[rover_daemon/rover_nav.py](../rover_daemon/rover_nav.py).

**A single route is capped at 15 m**, and the cap is a property of the map rather
than a policy: the grid reaches 20 m in every direction from where the rover was
switched on, and a route is allowed most of that. It was 8 m, which was honest while
the grid was 20 m across and reached 10 m, and on the wider grid it refused places
the map plainly showed — a tap 11.2 m away came back *that is 11.2 m away and a
single route is capped at 8 m* with the room it pointed at drawn on screen. The time
budget moves with the distance: 200 s, because 15 m at the default 0.22 m/s is 68 s
of driving before a single corner is turned and a route is a polyline rather than a
straight line. Planning grows with the length but only linearly, since the margin
either side is capped at 2.5 m rather than being a fraction of the run — a 15 m route
searches a 20 × 5 m window against a 5 m route's 10 × 5 m, which is twice the cells
and not nine times.

**Planning is the slowest thing the rover does**, and the numbers are only visible
on the rover: measured over seventeen plans it really made, the same call takes 2–6
ms on a desk and 2.2–15.3 *seconds* on the Pi. A heap pop costs 1.3 µs here and
580 µs there — a factor of 450 that no clock speed explains, and the reason is that
A*'s flat lists blow a 128 kB L2 cache, so every `g[j]` is a pointer chase into
SDRAM. Cost therefore follows the *size of the window searched*, not the length of
the route (correlation +0.74), and two things follow from that:

- The window is sized to the route. It used to be the two points plus a flat 2.5 m
  either side, so a 34 cm replan searched a 5 m square and took 1.1 s of standing
  still. It is now `CROP_MARGIN_FRACTION` of the distance, floored at two keep-out
  radii — a window narrower than that is all keep-out once inflated and can hold no
  route at all — and the full 2.5 m is tried again before any route is refused or
  any clearance given up. A search that finds nothing has opened every cell it could
  reach, so the small window is a bet: `CROP_MARGIN_WORTH` declines it unless it
  saves enough area to be worth losing. Without that gate one 5.3 m route went from
  two passes to three and came out 40% slower.
- The keep-out is inflated once per distinct run length. The disc is a union of
  horizontal runs, one per row offset, and the same length turns up on several rows;
  growing each length out of the one below it and then placing the rows takes about
  forty whole-array passes where offset-by-offset took 253. Same disc, bit for bit,
  and the self-test checks it against the version it replaces.

On the Pi: a 0.18 m replan 1.41 s → 0.14 s, a 0.34 m one 1.12 s → 0.15 s, and the
long routes 1.2–1.4×. Every route is unchanged — same waypoints, same length, same
turning, same clearance — over both the recorded grids and the room sweep.

Octile distance is the tighter A* heuristic and was tried here for the same reason.
It opened 5% fewer cells and saved 2%, and it broke ties differently: four of twelve
test rooms came back with a different route of the *same* tolled cost, one passing
4.7 cm closer to a table. Equally optimal is not equally good to drive, and 2% does
not buy a change in where the rover goes, so the heuristic stays the straight line.

**Then the route is pulled straight, because most of its corners are the grid and
not the room.** A* steps in eight directions and measures in octile steps, so every
monotone staircase between two cells costs exactly the same and which one comes
back is down to the order the heap popped. On empty floor that produced a four-metre
run straight ahead followed by a 25° kink for no reason at all, and thinning cannot
undo it — the kink is a real 40 cm departure from the straight line, and keeping
departures that large is the whole job of thinning. So runs of corners are replaced
by the line between their ends wherever that line is clear of the same keep-out A*
was given and costs no more under the same toll, with a credit of 30 cm of path for
each corner it removes: a corner past the follower's turn-in-place threshold is a
full stop and a dead-reckoned spin, about 1.8 s for a right angle, which is more
driving than it saves. The credit is withheld on a fallback route through a pinch,
where the keep-out is already inside the distance the follower brakes at and the
toll is the last thing holding the route off the wall.

Over 347 routes through randomly generated rooms that is 1.1 fewer waypoints, 39° less turning and half a
stop-and-spin per route, for 8 cm less path — and no route anywhere came out nearer
to anything than the keep-out it was planned with, which is the invariant that makes
shortening one safe at all.

The follower's carrot stays on the current segment. Looking a metre past a vertex
*onto the next leg* is how a route that gave a corner room still drove the chord and
arrived at the brake distance; a sharp corner is a turn on the spot, which is the
move this rover already has. What it does do at a *gentle* corner, once the vertex
has come inside 30 cm, is run the aim point on past it along the line of the leg
being driven — which is a different thing, because extending the line the rover is
already on cannot bend it towards the inside of the corner. Without that the carrot
collapses onto the vertex and the bearing to it becomes pure cross-track error and
pose wobble: 5 cm to the side of a carrot 5 cm ahead is 45° of heading error out of
nothing, past the turn-in-place threshold, so the rover stopped and spun a hand's
breadth short of a corner it was tracking cleanly. Two thirds of the heading a
simulated route threw away went on exactly that.

A corner past the turn-in-place threshold keeps the collapsing carrot it always had,
and so does the last waypoint — for different reasons. At a sharp corner the rover is
going to stop and spin whatever happens, so triggering it a few centimetres early
costs nothing, while running on past a right angle would have it arrive at the corner
still under power, and a corner is where the route has the least room to spare. At
the goal the collapsing bearing is doing real work: it is what swings the rover round
to a place it would otherwise sail past a little to one side of.

Both changes were measured by driving the real `_step_goto` around simulated rooms at
the 10 Hz it runs at, with the pose wobble the matcher really has — whole journeys,
replans included, not single legs. Over 310 of them: four more arrivals out of 310,
a second off the average journey, 23% less heading swing, fewer replans, and the
committed pair's four timeouts and one "gave up replanning" gone entirely. The rover
also ends up *further* from things rather than nearer — tightest approach 0.19 m to
0.20 m, and half as many journeys inside 0.25 m — because the time it no longer spends
replanning and unsticking is time it was spending close to something.

And because turning is always legal, even with the nose in a wall, a heading that
looks into the keep-out starts the route with a hop off that heading so the first
thing the rover does is turn, rather than drive the chord through the blocked cell.

The polyline is a sketch, not a promise — the live scan stays in the loop while
following it, and the route is thrown away and planned again when the room
disagrees. A route is not thrown away for being blocked, though: the planner reads
the pose and the map, so a rover that has stopped gets the same route back and
refuses it again a revolution later. It turns to look for room instead, and asks
for a new route only once it has moved somewhere the planner can answer from. `planner.py` is pure Python but shaped for this host: the
inflation is one whole-array pass per disc offset rather than one write per blocked
cell, and A* runs on flat Python lists because a numpy scalar read costs several
list indexes. The first version did neither and took 7–10 **seconds** a route on
this Pi, paid again at every replan; it now takes about 0.2 s a keep-out. Trying
the 45 cm ring first and falling back to 25 cm is two of those when the wide
ring has no route, and one when it has.

A rover that has ended up inside the inflation ring of a wall plans from the nearest free
cell instead of being refused — that is the wedged case, and refusing it is how a
planner strands the thing it steers.

One move at a time, enforced with a lock rather than assumed: tool calls arrive on
whichever connection thread carried them, and a turn racing a drive would interleave
PWM. The second request is refused as "busy" — queueing it would drive the rover
somewhere the first caller has since made wrong. `stop()` takes no lock and always
gets through.

Every move returns **why it stopped**, which matters more than the pose: "stopped
after 40 cm because something was 32 cm ahead" is actionable and "done" is not.

**And says what it is doing before it is over.** That answer arrives once, at the
end, and a route can take a minute to reach the end of — so on its own it left
anything watching with a stopwatch and no idea whether the planner had accepted the
request, refused it outright, or was three replans into carrying it out. A move now
publishes each of its turns into `status()` under `move`: planning, the route it
came back with and how many corners are in it, the reason there is none, every
replan with what provoked it, and how it ended. `MoveReport` holds it, one sentence
at a time rather than a queue — a watcher that misses a phase wants the one
happening now, not a backlog — and each carries a counter, which is what lets a
console poll this three times a second and still write one line per thing the rover
said. [voice_chat/drive_web.py](../voice_chat/drive_web.py) is what reads it,
under the map you clicked on.

`dryrun.py` exercises all of it against the real lidar with a stub link, so the
control loop, the clearance checks and the PWM arithmetic can be tested on live scans
with nothing reaching the motors. Run it before the first real move.

### Recording a journey, when the map afterwards is not enough

A route that comes out convoluted cannot be diagnosed from the map image, and it
cannot honestly be reproduced in simulation either, because every simulation is a
guess about which part is going wrong. There are at least four candidates and they
call for opposite fixes: the planner drew a bad route through a good map; the map
was ragged so no good route existed; the route was fine and the follower wove along
it; or the rover drove straight and the *pose* moved, so the blue line is a drawing
of the estimate rather than of the rover. That last one is not exotic here — most
of the recent work on the matcher is about poses that are confidently wrong — and
nothing downstream can tell it apart from the other three.

So `journey.py` records the inputs and the decisions rather than the conclusions:
the occupancy grid and pose handed to every plan, the route that came back, every
replan with what provoked it, each dead-reckoned burst against what the matcher
saw of it, and one row per revolution carrying the pose, the match score and
ambiguity, what the follower asked the wheels for and what it measured them doing.

**Recording is armed by making the directory and stopped by removing it**, which is
the whole control surface — no flag, no new tool, and above all no restart, because
the daemon's arguments live in a crontab entry and relaunching it by hand is how the
rover silently loses them:

```bash
ssh rpi 'mkdir -p ~/ugv/journeys'     # record the next few moves
ssh rpi 'ls ~/ugv/journeys'           # newest five are kept
scp rpi:'~/ugv/journeys/journey-*.npz' .
python3 journey.py journey-20260821-141332.npz            # the timeline
python3 journey.py journey-20260821-141332.npz --replan   # replan those same maps
ssh rpi 'rm -rf ~/ugv/journeys'       # stop
```

`--replan` is the reason the grids are kept: a change to `planner.py` can be tried
against the rooms that actually produced a bad route, instead of against rooms
invented to look like them.

Nothing in it may throw into the control loop — a diagnostic that can stop the rover
is worse than none — so every entry point swallows its own errors and goes quiet.

The one question it exists to answer is the last of the four, and it asks it by
arithmetic rather than by eye: between two revolutions the rover cannot have moved
further than the faster of what it asked for and what the matcher measured, so
anything beyond that came from the estimate changing its mind. Revolutions spanning
a dead-reckoned burst are excluded, because a burst suspends matching and re-seeds
the heading on purpose — 53° of legitimate step across a 60° burst, and counting it
would make every healthy turn look broken.

## Telling a model where it is

Two tools, and the text one does most of the work.

`Slam2D.describe()` segments the scan into **walls, objects and gaps** rather than
handing over a list of ranges — a model will confabulate over 36 numbers and can
reason about "a flat surface 1.6 m to your left". The segmentation clusters at range
discontinuities, splits the clusters at corners so a rectangular room comes back as
four walls instead of one lumpy ring, and reports openings the rover would actually
fit through.

The one inference it makes is grouping: four narrow objects in a square metre is the
signature of furniture, and no single reading says so. That is the whole answer to
"navigate around the table" — **the lidar never sees a table, it sees four legs**, and
the description says as much in words, so the model can do the naming that geometry
cannot.

`mapimg.py` renders the grid as a PNG using nothing but `zlib` and `struct`, because
there is no image library on this Pi at all — no OpenCV, no PIL. The camera hands over
MJPEG already encoded, so nothing here ever needed one. What goes out is cropped to a
few metres, scaled up, and marked with the rover, its heading, its track and a one
metre scale bar; a raw 400×400 occupancy grid shown to a vision model is a field of
grey speckle that invites confident nonsense. The caption travels as the tool result
whether or not the picture arrives, so a refused image degrades to a worse answer
rather than to an invented one.

Two things in there turn metres into pixels — the array of cells, and `to_px` for
everything drawn over it — and they have to agree, because the grid's axes are
forward and left rather than row and column. They did not, for a while: an extra
transpose reflected the walls about the diagonal and left the rover, its heading and
its track alone, so the track ran across a corridor instead of down it. Each half
looked plausible by itself, and the mock rover draws both halves with one function of
its own, so only the real map showed it. `python mapimg.py` now asserts that a wall
straight ahead and a track that drove into it come out as a vertical line meeting a
horizontal one, that a wall to the left is drawn on the left, and that the arrow
swings counter-clockwise when the heading says left.

The picture is in colour for the same reason. Occupancy wants to be a lightness ramp
from solid black to empty white, which leaves nothing for the things drawn on top:
in grey, the track and the rover were both dark pixels over dark obstacles, and the
two hardest things to find in the picture were where the rover is and where it has
been. Now hue carries the overlay and lightness carries the occupancy — a red arrow
for the rover, tip forward, with a yellow dot at the exact pose, and a blue line for
the path. The arrow replaced a dot with a whisker off it, which at three pixels per
cell was two pixels wide and left the heading to be guessed. Nothing on the rover can
draw text, so the caption names the colours for the model and
[voice_chat/drive_web.py](../voice_chat/drive_web.py) builds its key out of this
file's palette rather than its own.

A client can zoom, and zooming keeps the picture the size it was. `map_png` in the
daemon takes how many metres to show and how big a picture to send back, and works
pixels-per-cell out from the two; `render` still takes it directly, since by then the
question has been settled. That way round matters. Taking a magnification instead
means the picture grows every time the view widens, which is rescaling the window
rather than zooming — asked for a steady 480 px, the console's ladder now comes back
465–492 px from 1.5 m across to 12 m, where fixing the magnification gave 240 px to
1200 px over the same range. Sizes are only reachable to within a few percent because
a cell must be a whole number of pixels, and past 12 m across it is down to two, which
is why the ladder stops there. Drawing is interpreted Python — there is no library to
hand it to — so a picture costs roughly its own area, and the total is capped.

A violet wedge shows where the camera is pointing and how much of the room is in
shot. That is the only thing in the picture that did not come off the lidar, and it
is there because the map otherwise says nothing at all about the other sensor: the
two point in different directions most of the time — the gimbal pans a long way
either side and sweeps continuously while face tracking runs — and the rover's own
arrow says nothing about where the camera got to. It reaches across the crop rather
than a fixed number of metres, so it reads the same at every zoom: it is a direction
and a width, not a range.

It is washed over the map at a quarter strength and then outlined at full. The fill
is what makes it read as one lit area rather than as three unrelated violet lines,
and a quarter is as far as it can go in the other direction: the interesting part of
the map is precisely the part inside the cone, so a heavier wash would hide what the
cone is there to point at — a wall under it has to stay as legible as a wall beside
it. The outline goes on top so the edges stay exact whatever the fill lands on. A
translucent fill is the one shape here that has to *read* what is underneath it
rather than overwrite it, which would be a per-pixel Python loop on a Pi 1 — so the
colour and the fraction, both fixed for the whole shape, are folded into a 256-entry
table per channel and `bytes.translate` applies it a row at a time in C. `python
mapimg.py` checks that more of the picture changed than the outline accounts for,
because every other check there finds the cone by its exact colour and would pass an
outline with nothing inside it.

**The two angle conventions are opposite, and that sign is the whole risk here.**
The gimbal counts pan positive to the right; the lidar, the map and everything else
in this directory count bearings positive to the left, counter-clockwise from
straight ahead. So the daemon hands the renderer minus the pan, in one place —
`_camera_cone` — and both `rover_daemon/selftest.py` and `python mapimg.py` check
the *direction* rather than the value, because a mirrored cone draws perfectly
ordinarily over the wrong half of the room and nothing about the picture gives it
away. The caption says which way and how wide in words as well, since a wedge on its
own cannot say whether it is 40 degrees or 90.

The width comes from the daemon's `--camera-fov`, and for this rover's camera it has
been measured at 132 degrees across — by
[`usb_cameras/calibrate_fov.py`](../usb_cameras/calibrate_fov.py), which sweeps the
gimbal and fits the lens to how far the room slides. It stood at 65 degrees for a
long time as a guess at a generic webcam, and the guess was out by more than a factor
of two: the module is a fisheye. A cone that narrow is not a small error to leave in
a picture whose whole job is saying which part of the room is in shot.

## Throwing the map away

`slam2d_reset` empties the occupancy grid and the likelihood field and stands the
rover at the origin of what is left; `Navigator.clear_map` wraps it, and the daemon
offers it as a control call that no model is shown. It exists because drift here is
permanent. There is no loop closure on this hardware and there will not be — a
closure attempt costs about 19 seconds at the rate this Pi searches poses — so a room
that has come out a few degrees out of true with itself, or a corridor stamped in
twice from two passes, stays that way for as long as the daemon runs. Past a certain
point the map is worse than no map: the planner routes on it and refuses gaps that
are really there. An empty map is at least true, and this rover fills one back in
within a revolution or two of standing still.

Three things about it are deliberate.

**Both grids, not just the picture.** The occupancy grid is what gets drawn and the
likelihood field is what the matcher actually slides a scan over. Clearing only the
first would blank the map while the old room went on deciding where the rover is.

**The pose goes back to the origin.** The grid is finite and centred on the origin,
so a rover that has driven six metres from where it started would otherwise get a
blank map with a third of it already behind it. The cost is that everything anyone
holds in world coordinates — a route, the driven track, somewhere worth going back
to — refers to an origin that no longer exists, so the track is thrown away with the
map and the navigator refuses to clear at all while a move is running. The route
being followed is written in the very frame the clear discards.

**The scan count goes back to zero**, which is what makes the next revolution take
the same first-scan path as the first one after startup and get stamped straight in.
Leave it alone and that revolution is matched against an empty likelihood field,
scores nothing, is rejected, and the map is never written again — a rover dead
reckoning through a permanently blank map, which looks exactly like a dead lidar and
is not one. `selftest.c` drives a metre and a half, clears, and then checks that the
grid is empty, that the pose and the count are back to zero, and that the next dozen
revolutions are mapped and matched.

`rover_up` picks which way is up: the heading the rover started with, so the room
holds still and the arrow turns, or the heading it has now, so the arrow holds still
and the room turns underneath it. The second needs the grid *sampled* through a
rotation rather than sliced, which incidentally fixed something the slicing got wrong
— a crop running off the edge of the grid used to come back as a smaller, and
sometimes not even square, picture. Off-grid now reads as never-seen, which is what
it is. A rotation is exactly the sort of thing that looks plausible while being a
quarter turn or a mirror out, so `python mapimg.py` checks it against a wall whose
real bearing is known, at each of the four quarter turns.

Most of that cost turned out to be avoidable, and finding it needed measurement
rather than reading. Colours were being packed from a tuple into `bytes` once per
pixel, which cost more than compressing the whole image; every shape now reduces to
horizontal runs, one slice assignment each, and packs its colour once. The border
alone had been twice the price of the PNG encoder. Worst was the rover's own track:
4000 poses 5 cm apart, most of them outside the crop, the rest drawn over their own
pixels again and again — 86 seconds for a single map late in a session against 2
seconds for the same map early in one. It is clipped, thinned, and then thinned
again to a fixed budget of points, which makes it a fixed cost instead of one that
grows all afternoon. Together those took the default map from 2.3 s to about 0.7 s,
and the rendering is pixel-for-pixel identical wherever the budget does not bite.

## Files

```
slam2d.h      the API, and the reasoning behind each config field
slam2d.c      parser, scan matcher, occupancy and likelihood grids, segmentation
selftest.c    correctness against a synthetic room and a synthetic table
build.sh      builds libslam2d.so and selftest, on the machine that runs them
slam2d.py     ctypes binding, and describe(); checks its struct layout each load
navigator.py  Navigator facade over the mixins below; `python3 navigator.py` self-tests the move commentary
nav_types.py  constants, Outcome, MoveReport, find_lidar
nav_drive.py  drive, goto, turn, PWM
nav_sense.py  pose trust and the lidar loop
navigator_selftest.py  the move-commentary checks, run via navigator.py
odometry.py   the board's gyro and wheel counts, as a prior and as a witness; self-tests
planner.py    a route through the occupancy grid, as a few corners; `python3 planner.py` self-tests
planner_selftest.py    the planner checks, run via planner.py
mapimg.py     a PNG encoder and the map rendering, in colour, stdlib only
run_slam.py   mapping on its own: pose, clearance, a PGM
dryrun.py     the whole driving stack on live scans, with nothing wired to the motors
calibrate_turn.py  measures real turns against the lidar profile, outside the matcher
journey.py    records what a move was handed and what it decided, and reads it back
usbreset.py   replugs the lidar in software when it drops off the USB bus; self-tests
99-rover-usb-reset.rules  what lets the daemon do that without being root
install-udev.sh           installs that rule; needs root, once, per rover
```

`libslam2d.so` and `selftest` are build products and are not committed.

The map comes out as a binary PGM, which needs no image library and which anything
can open. Occupied is black, free is light, and never-seen is mid grey, so an
unexplored map reads as unknown rather than as confidently empty. One ambiguity is
inherent to keeping a single log-odds value per cell with no visit count: a cell hit
once and later cleared back to exactly zero is indistinguishable from one never seen.

## When the lidar drops off the bus

It does, and not rarely. The sensor's serial adapter hangs off a small hub, on
another hub, on the Pi's own hub -- three deep -- and the whole branch goes away
under motor load:

```
usb 1-1.3.3: USB disconnect, device number 17
usb 1-1.3-port3: Cannot enable. Maybe the USB cable is bad?
usb 1-1.3-port3: attempt power cycle
usb 1-1.3-port3: unable to enumerate USB device
```

Read the last three lines carefully, because they are the whole reason this section
exists: the kernel notices, tries a port power cycle of its own, fails, and **stops
trying**. The port stays dead until something resets it. Everything above this in
the stack behaves correctly and uselessly — `find_lidar` looks for a port that is not
there, `lidar_ok` goes false so nothing drives on a stale pose, the console shows a
scan age climbing — and the rover sits blind until somebody walks over and pulls the
plug. The run that prompted this had been blind for sixteen minutes.

`usbreset.py` is the plug, in software. `USBDEVFS_RESET` on a device re-enumerates
that device; on a hub it re-enumerates the hub and everything below it, which is the
only thing that reaches a port too wedged to enumerate at all.

**The ladder is nearest-first, and it escalates only on evidence.** On this rover it
comes out as

```
1-1.3.3.2 (USB Single Serial) -> 1-1.3.3 (USB 2.0 Hub) -> 1-1.3 (USB2.0 Hub)
```

and it stops there rather than continuing to `1-1`, which is the Pi's built-in hub
and carries the ethernet and the wifi dongle: resetting that would cut the wire the
request to reset arrived over. `_carries_the_network` works that out by walking each
candidate's subtree for a net device, so re-plugging the wifi somewhere else does not
silently make the rover cut itself off.

Escalation matters as much as the ladder. A reset can succeed and change nothing —
the ioctl returns cleanly against a device that is enumerated but dead — so a
recovery that only ever reset the device would spend the afternoon repeating the one
act already shown not to work. The navigator therefore counts: nothing came back, so
reach one rung higher. Only when the ladder is exhausted does it start backing off,
doubling from a minute to a quarter of an hour, because at that point it is a cable
and knocking the camera out every minute will not change that.

**The rungs and the thresholds.** Six seconds of silence closes the port and opens
it again — that is sixty missed revolutions and it fixes the case the by-id name
exists for, an adapter that re-enumerated under a running daemon. Thirty seconds
reaches for USB. Neither happens while the wheels are turning: the reset takes the
camera and the OAK down with it for a few seconds, and a move that has lost the
sensor is already being stopped by the watchdog for the same silence.

Measured on the rover, with the adapter taken off the bus by deauthorising it and
nothing touched afterwards:

```
   3s  live=False age=3.06   resets=0
  33s  live=False age=33.2   resets=1   reset 1-1.3.3.2 (USB Single Serial)
  93s  live=False age=93.49  resets=2   reset 1-1.3.3 (USB 2.0 Hub)
  99s  live=True  age=0.02   resets=2
```

Rung one had no effect, which is what a deauthorised device does and what a wedged
one does; rung two brought it back, and the daemon found the port again on its own
six seconds later.

**It needs a udev rule, once per rover.** `/dev/bus/usb/BBB/DDD` is `root:root 0664`
and the reset ioctl needs the node open for writing, so without the rule every
attempt comes back "not allowed to reset /dev/bus/usb/001/005" and names the node it
could not open. `install-udev.sh` puts the rule in place and reapplies it to what is
already plugged in:

```bash
cat secrets/rpi-sudo.key | ssh rpi 'sudo -S -p "" ~/ugv/lidar_slam/install-udev.sh'
```

The action there has to be `udevadm trigger --action=add` and not `change`: udev sets
a node's owner and mode when the node is created, and re-running the rules under
`change` matched the rule, reported `GROUP 46, MODE 0660` in `udevadm test`, and left
every node `root:root 0664` — which looks exactly like a rule that did not match.

`nav_status` reports `lidar_resets` and what the last one said, and the console shows
both, because the number is the diagnosis: a rover that has replugged its own lidar
four times in an afternoon has a cable working loose, and nothing else would ever say
so.

## What is not done yet

- **Turning has been driven for real; straight-line speed control barely has.** The
  open-loop turn rates in `navigator.py` came off the floor via `calibrate_turn.py`,
  which measures against the lidar profile rather than the matcher under test, and
  the wheel sense is both asserted in `dryrun.py` and confirmed by those runs. The
  speed loop is still a single scale factor nudged by the matcher's measured speed,
  clamped tight because at 10 Hz anything eager will oscillate — expect to tune it,
  and expect the straight-line trim to matter, since equal PWM is not equal speed on
  this chassis.
- **The magnetometer is still unused.** `mx/my/mz` is the one absolute heading
  reference on this rover — the OAK-D-Lite's BMI270 has no equivalent — and it is the
  obvious answer to a pose whose heading drifts for ever, subject to the usual caution
  about the rover's own motors distorting it. The gyro beside it is read now; see
  [The gyro](#the-gyro-a-prior-and-a-witness).
- **The gyro's scale is one number where the rover wants two**, and the wheels' is a
  ratio where the rover wants a ratio and an offset. Both are measured and both are
  good enough for a prior; see [What the rover actually
  measured](#what-the-rover-actually-measured) for the asymmetry and the overhead
  that a single figure each is averaging over.
- **The turns undershoot, consistently.** Asked for 175° the rover managed 159;
  asked for 140 it managed 127; asked for 45 it managed 11. That is the open-loop
  `TURN_RATES` table being optimistic, which was already known — but the gyro now
  makes it measurable independently of the matcher, so re-deriving that table is a
  smaller job than it was.
- **The map picture has not been seen by the model.** The frame server stashes bytes
  without decoding and the upload declares no media type, so a PNG ought to be as
  acceptable as a JPEG — but that is reasoning, not a test. If it turns out to be
  refused, the caption still answers and the fix is a JPEG encoder or a service-side
  change.
- **Driving and face tracking are mutually exclusive**, enforced by the daemon parking
  tracking for the duration of a move — and parking it now releases the camera as
  well as the loop reading it, which are two different things. That is a real
  limitation and not a bug: SLAM is a third of the core, MJPEG forwarding is another
  third, and oversubscribing the one core makes the scan matcher drop revolutions —
  degrading exactly the thing keeping the rover off the walls. Measured with the
  rover stationary and one picture taken, 9.94 revolutions/s and no losses with the
  camera shut against 7.52/s and 22.1% dropped with it streaming. A single
  photograph does not cost this any more: `look` and friends capture three frames
  and let `v4l2-ctl` exit rather than holding the feed open, so only tracking
  competes with driving now.
- **The lidar sees one horizontal slice** and cannot see a step, a drop, a low sill or
  a table top. Thirty centimetres from a wall is safe; thirty centimetres from a stair
  is not. Nothing in software fixes that, and an unattended rover needs either a
  second sensor or a rule about where it may run.
