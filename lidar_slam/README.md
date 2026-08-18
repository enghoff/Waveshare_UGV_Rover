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
uses in the VM — `loop_search_space_dimension: 8.0` at `resolution: 0.05`, so
160×160 offsets across 13 angles — is 332,800 poses, or about **19 seconds for one
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

The coarse pass spans ±0.10 m and ±9°, which at 10 Hz is 1.0 m/s and 90°/s — the
angular window earns its width because a rotation past the edge comes back
*under-reported* rather than rejected, and a controller closing on that keeps
turning. The fine pass then spans one coarse step. 5×5×7 + 5×5×5 = 300 poses, and
points are thinned to 300 of the ~419 the sensor delivers because every point costs
a cache miss in every one of those poses; thinning bought 25 ms a revolution.

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

Going to a *place* rather than a distance — `drive_to`, which is what a tap on the
map becomes — plans on the occupancy grid first: A* at cell resolution over the map
inflated by 25 cm, with unknown treated as blocked, thinned to the handful of
corners that change heading. That radius is a sideways gap, not the along-track
brake: inflating by the standoff plus the 15 cm reaction asked for a 90 cm opening
and refused pinches the chassis still fits, with the live scan clear down the
middle. The follower still keeps 30 cm ahead and brakes 15 cm early; it just is
not asked to pretend a 85 cm doorway is closed.

The hard ring alone has the opposite vice: A* hugs it, so every corner is passed
at exactly the distance the follower brakes at and an ordinary pose error turns a
legal route into a stop. So beyond the keep-out there is a toll — travel within
55 cm of anything blocked costs up to three times its length, fading to nothing at
the edge. In the open the route arcs wide of a corner (measured in the self-test:
24 cm of clearance becomes 63 cm, for 0.7 m more path); through a narrow gap both
sides charge every route the same, so the squeeze is still taken. The toll shapes
the route, the keep-out decides what exists at all.

The polyline is a sketch, not a promise — the live scan stays in the loop while
following it, and the route is thrown away and planned again when the room
disagrees. A route is not thrown away for being blocked, though: the planner reads
the pose and the map, so a rover that has stopped gets the same route back and
refuses it again a revolution later. It turns to look for room instead, and asks
for a new route only once it has moved somewhere the planner can answer from. `planner.py` is pure Python but shaped for this host: the
inflation is one whole-array pass per disc offset rather than one write per blocked
cell, and A* runs on flat Python lists because a numpy scalar read costs several
list indexes. The first version did neither and took 7–10 **seconds** a route on
this Pi, paid again at every replan; it now takes about 0.2 s, same routes. A rover
that has ended up inside the inflation ring of a wall plans from the nearest free
cell instead of being refused — that is the wedged case, and refusing it is how a
planner strands the thing it steers.

One move at a time, enforced with a lock rather than assumed: tool calls arrive on
whichever connection thread carried them, and a turn racing a drive would interleave
PWM. The second request is refused as "busy" — queueing it would drive the rover
somewhere the first caller has since made wrong. `stop()` takes no lock and always
gets through.

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
[voice_chat/drive_console.py](../voice_chat/drive_console.py) builds its key out of
this file's palette rather than its own.

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
— a crop running off the edge of the 20 m grid used to come back as a smaller, and
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
navigator.py  the drive controller: avoidance, steering, speed, PWM
planner.py    a route through the occupancy grid, as a few corners; `python3 planner.py` self-tests
mapimg.py     a PNG encoder and the map rendering, in colour, stdlib only
run_slam.py   mapping on its own: pose, clearance, a PGM
dryrun.py     the whole driving stack on live scans, with nothing wired to the motors
calibrate_turn.py  measures real turns against the lidar profile, outside the matcher
```

`libslam2d.so` and `selftest` are build products and are not committed.

The map comes out as a binary PGM, which needs no image library and which anything
can open. Occupied is black, free is light, and never-seen is mid grey, so an
unexplored map reads as unknown rather than as confidently empty. One ambiguity is
inherent to keeping a single log-odds value per cell with no visit count: a cell hit
once and later cleared back to exactly zero is indistinguishable from one never seen.

## What is not done yet

- **Turning has been driven for real; straight-line speed control barely has.** The
  open-loop turn rates in `navigator.py` came off the floor via `calibrate_turn.py`,
  which measures against the lidar profile rather than the matcher under test, and
  the wheel sense is both asserted in `dryrun.py` and confirmed by those runs. The
  speed loop is still a single scale factor nudged by the matcher's measured speed,
  clamped tight because at 10 Hz anything eager will oscillate — expect to tune it,
  and expect the straight-line trim to matter, since equal PWM is not equal speed on
  this chassis.
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
