# Full-resolution evidence: a still look keeps its view at 2592x1944, and the models keep reading 640x480

**Since `c5ceec0`, deployed at 12:05, every still look through the gimbal camera
keeps a 2592x1944 picture of its own view beside the 640x480 frame its models
read.** That is sixteen times the pixels the rover recorded before, at about
0.35 s more per look. The models were not moved onto the larger picture: reduced
to 640x480, it changed their vectors far more than two grabs of the same view
do, and every vector in the store came off the camera's 640x480 mode. The first
step of the [perception plan](../plans/perception-that-keeps-what-it-learns.md),
done at the owner's request
([R-WS-19](../requirements/world-state.md#r-ws-19)).

## What the camera does at each size

Grabbed on the rover, parked on the charger, with the daemon's own one-shot
capture (three frames, the newest kept), interleaved in three rounds:

| Size | Grab | JPEG | Brightness |
|---|---|---|---|
| 640x480 | 0.35-0.57 s | 34 kB | 139-140 |
| 1280x960 | 0.36-0.37 s | 100 kB | 139-140 |
| 2048x1536 | 0.37-0.38 s | 194 kB | 140 |
| 2592x1944 | 0.39-0.41 s | 292 kB | 139-140 |

The full size costs no more time to grab than the small one, and exposes the
same. One earlier run of the same script lost every grab after its second, each
failing in 10 ms with nothing said; the daemon's own grab worked straight after,
and seven raw grabs and the twelve above all worked. The cause was not found;
something else holding the camera for a few seconds would look like this, and
the inspector already treats a failed grab as a lost picture, not a lost look.

## The sizes are not the same window

Each pair, taken seconds apart with nothing moving, was matched by SIFT features
and fitted with an affine map. 236-296 matches survived per pair, at a residual
of 0.22-0.26 pixels:

- **1280x960 is the 640x480 window** at twice the pixels: scale 0.4998-0.5001,
  and in place to within 0.4 pixels.
- **2592x1944 is not.** The 640x480 picture is a window of 2580 by 1934 of it,
  starting about a pixel and a half in, scale 0.24803-0.24831, consistently over
  the three rounds. So the full frame holds about eleven more columns at the
  right and eight more rows at the bottom. 2048x1536 is another window again,
  2039 by 1529.

A reduction through that window (`evidence.NATIVE_WINDOW`) lands on the
camera's own 640x480 picture within 0.2 pixels, about 0.04 degrees. The lens
model would have treated 2592x1944 as the 640x480 lens scaled up by shape, which
is 0.5% out in scale and a pixel and a half in position. `face_tracking/lens.py`
now carries the 2592x1944 lens through the measured window, and a check gives
every direction within 0.05 degrees of the 640x480 lens.

## Why the models still read 640x480

The perception sidecar on the rover was given the camera's 640x480 picture and
the full one reduced through the window, for each of the three views. Regions
were matched by box overlap of 0.7 or more, 9 to 10 a view:

| | DINOv2, median cosine (lowest) | SigLIP2, median cosine (lowest) |
|---|---|---|
| 640x480 against the reduced full picture | 0.88-0.90 (0.79-0.81) | 0.93-0.94 (0.88-0.91) |
| two 640x480 grabs of the same view | 0.98 (0.90-0.95) | 0.98-0.99 (0.96) |

The reduction is sharper and differently compressed than what the camera's own
scaler makes, and both encoders notice. A look's appearance is compared with
the vectors already held, all of them from the 640x480 mode. A held-out look
already matches its own object at 0.70 or better only 43% of the time
([world_state](../../world_state/README.md#checking-a-hypothesis-with-one-look)), so
a shift of 0.1 would cost identity. So the full picture is a second grab, taken
after the look's shutter bracket has closed. It is kept only when the rover has
moved under 2 cm and turned under half a degree since the look, and the gimbal
is where it was. A look on the move, through the OAK, or while face tracking
holds the camera keeps none.

## On the rover

Deployed with `world_state` and `rover_daemon` at `c5ceec0`, the rover's suite
passing. Afterwards navigation was still trusted and settled, and autonomy off
and not latched.

- The looking loop's first look after the restart kept a full picture on its
  own: 286 kB, taken 0.34 s after the frame.
- Six looks asked for since all kept theirs: 2592x1944, 286-289 kB, 0.33-0.68 s
  after their frames, each read back by `world_state_frame` with `full`, the
  frame beside it still the camera's 640x480 at 34 kB.
- Without the identity pass, four looks took 1.20-1.28 s, depth camera off.
  Comparable looks this morning, depth camera off and identity not settled, took
  0.82-0.98 s. So the full picture costs about 0.35 s a look, which on an
  autonomous run of about 50 looks is about 20 s.
- Asked with the identity pass, as `world_inspect` does without arguments, a
  look took 10.2-10.7 s. That was the pass over the 405 looks waiting for a view
  from elsewhere, not the picture: the perception itself was 0.34 s, and the
  looking loop's own looks do not ask for the pass.

At about a thousand looks on a busy day, the full pictures add about 0.3 GB a
day. The store's disk has 800 GB free. A snapshot links them and a clear removes
them with their frames, because they end in `.jpg` beside them.
