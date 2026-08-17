# `lidar_slam/` — 2D SLAM and self-driving on the rover's own Pi

Scan-matched localisation, an occupancy grid, and a drive controller that will not
run into things — from the D500 lidar alone, on the Raspberry Pi 1 bolted to the
rover. It exists because that Pi already has both sensors wired to it and no map to
show for them: the ROS 2 stack in [`vm/`](../vm) does the mapping far better, but it
lives in a VM on VMware's NAT segment and cannot see the rover at all, so nothing it
computes can ever steer anything.

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
parse + CRC, 35 packets                  0.33 ms
map update + 1 pose                      4.70 ms
scan match                              28.48 ms   (370 poses, 0.077 ms each)
TOTAL per revolution                    33.50 ms    33.5% of one core -> fits
```

**That 33.5% is the number to plan around, not the 100 ms budget.** The core is
already committed elsewhere: forwarding 640×480 MJPEG over the WiFi dongle costs
about 30% of it, and the link saturates the CPU at 72% before it saturates itself.
SLAM and video at full frame rate do not both fit, and that is the trade to make
consciously rather than discover.

## What is deliberately missing

**Loop closure, and with it any globally consistent map.** The cost is not marginal.
Each candidate pose costs a measured 0.077 ms, and the search window `slam_toolbox`
uses in the VM — `loop_search_space_dimension: 8.0` at `resolution: 0.05`, so
160×160 offsets across 13 angles — is 332,800 poses, or about **26 seconds for one
closure attempt** on this host. Branch-and-bound of the kind Cartographer uses would
change that answer, and hand-rolling it is a much larger project than this.

So the pose here drifts, monotonically and for ever. Drive a circuit and the two
ends of it will not meet. Everything downstream should treat the pose as good
locally and untrustworthy globally: fine for "is there a wall 40 cm ahead", wrong
for "return to where I started".

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

`score` is the mean likelihood under the matched scan, 0 to 1, and it is the health
indicator worth watching: 0.98 stationary against a mature map, and a run where it
sags toward `min_match_score` (0.15) is a run that is losing its position. `drop`
counts revolutions thrown away because the loop fell behind — it should stay at 0,
and if it does not, the Pi is oversubscribed rather than the SLAM being slow.

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

The coarse pass spans ±0.15 m and ±6°, which at 10 Hz is 1.5 m/s and 60°/s. The fine
pass then spans one coarse step. 7×7×5 + 5×5×5 = 370 poses, and points are thinned
to 300 of the ~419 the sensor delivers because every point costs a cache miss in
every one of those poses; thinning bought 25 ms a revolution.

A match whose mean likelihood falls below `min_match_score` is **rejected** and the
motion prior is used instead. This matters more than it looks: dead reckoning drifts
predictably, whereas a confidently wrong match teleports the rover and then corrupts
the map it will be matched against next.

## The motion prior, and the two numbers nobody has measured

The prior only centres the search window, so it is genuinely optional — at a walking
crawl the true motion is inside the coarse window anyway, and every measurement above
was taken with no prior at all. `selftest` covers both regimes, including the one
where the prior stops being optional:

```
--- driving past the search window: 30 cm a revolution ---
  without prior     worst x error 3.000 m
  with prior        worst x error 0.000 m
```

The driver board's `T:1001` telemetry has everything needed to supply it, and rather
more than the VM's filter gets: a 9-DoF IMU in `ax/ay/az`, `gx/gy/gz` and — unlike
the OAK-D-Lite's BMI270 — a magnetometer in `mx/my/mz`, plus wheel encoders in
`odl`/`odr`.

**Two scale factors are needed and neither has been measured on this rover**: the
encoders' counts per metre, which depends on gearbox and wheel, and the gyro's LSB
per deg/s, which depends on the full-scale range the firmware picked. `run_slam.py`
therefore takes them as `--ticks-per-metre` and `--gyro-lsb-per-dps` and contributes
nothing to the prior without them. Guessing either would produce a prior that looks
plausible while quietly dragging the match off true — strictly worse than no prior.

Two further things are known about that stream and worth planning around. It runs at
**~20 Hz**, measured at 19.9 by draining `in_waiting` in bulk — a `readline` loop
with a 0.2 s timeout reports 17, and the missing sixth is the reader's fault, not the
firmware's. Even 20 Hz is slow for a gyro: a 60°/s turn advances 3° between samples.
And the bias is worth removing — `Telemetry` averages the first ~34 samples, about
1.7 s, with the rover held still, for the same reason `vm/config/ekf.yaml` does. It
measured 6.9 LSB on `gz`.

There is a pleasing way out of the calibration problem, not implemented here: once
scan matching works, *its* heading is the reference the gyro lacks, so the rover can
calibrate its own gyro scale by turning on the spot and comparing.

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

Steering is follow-the-gap — the heading with the most room, penalised for departing
from the one asked for. Wall-following at a shallow angle falls out of that rather
than being a special case with a threshold to tune.

Every move returns **why it stopped**, which matters more than the pose: "stopped
after 40 cm because something was 32 cm ahead" is actionable and "done" is not.

`dryrun.py` exercises all of it against the real lidar with a stub link, so the
control loop, the clearance checks and the PWM arithmetic can be tested on live scans
with nothing reaching the motors. Run it before the first real move.

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
horizontal one.

## Files

```
slam2d.h      the API, and the reasoning behind each config field
slam2d.c      parser, scan matcher, occupancy and likelihood grids, segmentation
selftest.c    correctness against a synthetic room and a synthetic table
build.sh      builds libslam2d.so and selftest, on the machine that runs them
slam2d.py     ctypes binding, and describe(); checks its struct layout each load
navigator.py  the drive controller: avoidance, steering, speed, PWM
mapimg.py     a greyscale PNG encoder and the map rendering, stdlib only
run_slam.py   mapping on its own: pose, clearance, a PGM
dryrun.py     the whole driving stack on live scans, with nothing wired to the motors
```

`libslam2d.so` and `selftest` are build products and are not committed.

The map comes out as a binary PGM, which needs no image library and which anything
can open. Occupied is black, free is light, and never-seen is mid grey, so an
unexplored map reads as unknown rather than as confidently empty. One ambiguity is
inherent to keeping a single log-odds value per cell with no visit count: a cell hit
once and later cleared back to exactly zero is indistinguishable from one never seen.

## What is not done yet

- **Nothing has actually been driven.** This is the big one. Every real-sensor
  measurement here was taken with the rover stationary on a bench, and every figure
  for tracking, speed control and steering comes either from synthetic scans or from
  `dryrun.py` with the motors disconnected. The first real move wants a clear floor,
  a hand near the power switch, and `--seconds` kept short.
- **The direction the wheels turn is inferred, not observed.** The PWM pairing copies
  `driver_board/drive_gamepad.py`, which is known to work, and the counter-clockwise
  sense is asserted in `dryrun.py` — but no motor has yet turned under this code.
- **The speed loop has never seen a moving rover.** With no encoders it is a single
  scale factor nudged by the matcher's measured speed, clamped tight because at 10 Hz
  anything eager will oscillate. Expect to tune it, and expect the straight-line trim
  to matter, since equal PWM is not equal speed on this chassis.
- **The gyro and the magnetometer are still unused.** The two scale factors in
  [The motion prior](#the-motion-prior-and-the-two-numbers-nobody-has-measured) remain
  unmeasured. Once driving works, `turn_in_place` calibrates the gyro for free by
  comparing its integral against the matcher's heading.
- **The map picture has not been seen by the model.** The frame server stashes bytes
  without decoding and the upload declares no media type, so a PNG ought to be as
  acceptable as a JPEG — but that is reasoning, not a test. If it turns out to be
  refused, the caption still answers and the fix is a JPEG encoder or a service-side
  change.
- **Driving and face tracking are mutually exclusive**, enforced by the daemon parking
  tracking for the duration of a move. That is a real limitation and not a bug: SLAM
  is a third of the core, MJPEG forwarding is another third, and oversubscribing the
  one core makes the scan matcher drop revolutions — degrading exactly the thing
  keeping the rover off the walls.
- **The lidar sees one horizontal slice** and cannot see a step, a drop, a low sill or
  a table top. Thirty centimetres from a wall is safe; thirty centimetres from a stair
  is not. Nothing in software fixes that, and an unattended rover needs either a
  second sensor or a rule about where it may run.
