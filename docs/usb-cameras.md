# The host's USB cameras

Exercised by [`usb_cameras/`](../usb_cameras). Two scripts, both needing only
OpenCV — no depthai, no pyserial. `preview_usb_cameras.py` covers every UVC device on
the machine driving the rover: the host's own webcams and, since it plugs in over USB
like any other, the rover's camera module too. The OAK-D-Lite is the exception — it is
not a UVC device and will not appear here; use
[`oak_camera/preview_rgb.py`](oak-d-lite.md#preview_rgbpy) for that one.
`calibrate_fov.py` measures how wide a camera actually sees, and is described in
[Measuring the field of view](#measuring-the-field-of-view).

## How it finds cameras

It probes capture indices 0–7 — DirectShow hands out dense indices from 0, so a
small range is enough — and keeps the ones that actually deliver a frame, allowing
three reads because some webcams hand back nothing until the stream warms up.
DirectShow is tried first and MSMF only as a fallback for cameras DirectShow cannot
see: DirectShow opens in tens of milliseconds where MSMF can take seconds per
device. It requests 1280×720 on open, then MJPG — in that order, for the reasons
in [A black frame is usually the format](#a-black-frame-is-usually-the-format).

OpenCV exposes no device names, so friendly names are borrowed positionally from
Windows' `Win32_PnPEntity` list, **sorted by `PNPDeviceID`**. That sort is not
cosmetic. DirectShow builds its list from the registry's capture interface class,
whose subkeys are named after the device path and so enumerate lexicographically;
`Get-CimInstance` returns its own order, which on this machine was Integrated
Camera, Brio 500, USB Camera against DirectShow's Brio 500, USB Camera, Integrated
Camera. Unsorted, every label was wrong by a place — the rover's USB camera was
presented as the Brio, and a dark frame from it read as the Brio failing rather
than as a naming fault. Sorting by `PNPDeviceID` reproduces DirectShow's order,
that field being the same device path the interface keys are named after.

The list still includes devices that refuse to stream such as IR sensors, which is
why labels are indexed by capture index rather than by list position — and any
oddity in the PnP list shifts every label, so treat the names as a hint, not an
identity. Check the picture against the name before concluding a camera is at
fault.

Only one camera is held open at a time, so the preview still works when several
cameras share a USB controller's bandwidth. A camera that vanishes mid-stream —
unplugged, or grabbed by another app — returns failed reads forever rather than
erroring, so it gives up after 30 failures (about a second) and moves to the next.

## A black frame is usually the format

The rover's own USB camera (`0abd:8050`, a plain UVC module) offers 1280×720 twice
over: **MJPG at 30 fps and YUY2 at 10 fps**. It runs no auto-exposure whatever in
the YUY2 one. DirectShow prefers uncompressed formats, so requesting the size and
nothing else lands squarely in the broken mode — which is what the script used to
do, and why this camera appeared to "randomly lose exposure".

Measured on one camera, one scene, seconds apart:

| Request | Negotiated | Rate | Mean brightness |
|---|---|---|---|
| 1280×720, size only | YUY2 | 7.8 fps | **1.10** / 255 |
| 1280×720, MJPG after the size | MJPG | 20.1 fps | **81.8** / 255 |
| 1280×720, MJPG *before* the size | YUY2 | 7.8 fps | 0.80 / 255 |
| 640×480, size only | YUY2 | 20.1 fps | 69.6 / 255 |

Three things follow, and each cost time to learn:

* **Order matters.** `CAP_PROP_FOURCC` set *before* the width and height is
  silently ignored and the stream stays YUY2. It must come after.
* **Re-applying auto cannot save you.** In the bad mode the exposure sits at
  1/64 s no matter how often `a` is pressed, and a fresh open inherits it —
  `CAP_PROP_EXPOSURE` even survives release and reopen. Only the format change
  lifts it.
* **The status words will not warn you.** This camera answers `-1` to
  `CAP_PROP_AUTO_EXPOSURE`, so the overlay reads `exposure set` — the driver's
  word, not a reading — while the sensor is pinned. A black frame beside a
  confident `exposure set` is this failure, not a dead sensor.

The MJPG request is best effort: at 640×480 this same camera keeps YUY2 and is
perfectly well exposed, and cameras with nothing to switch to simply carry on. The
caption carries the negotiated format for exactly this reason — if a camera looks
dark, read the format before suspecting the hardware.

## Forcing auto, and reading the result

Every camera is put into full auto on open — autofocus, auto exposure and auto
white balance — because whatever app used it last may have left it pinned to
manual. UVC has no separate auto-gain or auto-shutter control: one auto-exposure
flag drives both shutter time and gain, so those three cover everything a webcam
automates. The first frame after opening is discarded, since it was exposed under
the previous user's settings, and some drivers only accept control changes once the
stream is live.

The overlay reports what each driver actually accepted:

| Status | Meaning |
|---|---|
| `on` | the driver confirms auto is engaged |
| `set` | accepted, but the driver will not report the flag back — its word, not a reading |
| `refused` | accepted the call, still reports manual |
| `unsupported` | no such control on this camera |

The `set` case exists because DirectShow answers `-1` for auto-exposure on every
camera here. `a` re-applies auto and prints a fresh report; `s` opens the driver's
own settings dialog, which is the escape hatch for controls OpenCV cannot reach —
the Brio's focus, for one. That is DirectShow-only and a no-op elsewhere.

## Controls

Left-click or `n`/space for the next camera, right-click or `p` for the previous,
`a` re-applies auto, `s` opens the driver settings dialog, `q` or Esc quits.
Closing the window also quits.

## Measuring the field of view

`calibrate_fov.py` answers "how much of the room is actually in this picture", which
matters because the map draws the camera's cone from that number and a guessed one
puts the wedge over the wrong part of the room. It needs no chart, no tape measure and
nobody holding a chessboard: the rover already owns two things that turn its camera by
a known angle, so the room it happens to be standing in is the target.

```bash
python usb_cameras/calibrate_fov.py --selftest         # no rover needed
python usb_cameras/calibrate_fov.py sweep/             # pan the gimbal
python usb_cameras/calibrate_fov.py sweep/ --by rover  # turn the whole chassis
python usb_cameras/calibrate_fov.py sweep/ --axis tilt
python usb_cameras/calibrate_fov.py sweep/ --fit-only  # re-measure saved frames
```

It talks to [`rover_daemon`](../rover_daemon/README.md) over TCP, using the same
`RoverClient` the voice stack uses, and keeps every frame it took beside a
`sweep.json` of the angles — so a sweep can be re-fitted later without going back to
the hardware.

**Two independent references, and running both is the point.** `--by gimbal` trusts
the pan servo's degrees. `--by rover` turns the chassis instead and takes the angle
from the lidar's scan match, which is measured against the walls rather than asked
for. They share no mechanism, so when they agree the servo is honest as well as the
lens known. On the rover's camera they came out at 132.4 and 131.7 degrees — half a
percent apart, which is the servo being vouched for by the lidar.

**It fits a lens model rather than averaging pixel shifts, and that is not fussiness.**
The obvious method — pixels moved, divided into degrees turned — reads high on a wide
lens, because a feature near the top of the frame slides less under a pan than one on
the centreline, for the same reason a degree of longitude is shorter away from the
equator. `--selftest` renders a synthetic room through a lens of known width and
measures it back, and it quotes both numbers: the fit recovers 136.0 degrees from a
136 degree lens, and averaging shifts over the same frames says 142.3.

That selftest is also the only thing that checks the sign conventions. A pan fitted
backwards, or the tilt axis used for a pan, gives a small residual and a quietly wrong
answer; rendering the sweep through the same rotation the fit inverts is what catches
it.

### What this camera turned out to be

| | measured, at 640x480 |
|---|---|
| horizontal | 130 degrees across 640 px |
| vertical | 96 degrees across 480 px |
| on the axis | 11.8 arcmin per pixel |
| distortion term | +0.03, so slightly wider than equidistant at the edge |
| centre of the lens | 316, 227 px — about 13 px above the middle of the picture |

Near enough an equidistant fisheye: the angular scale is nearly the same everywhere
in the frame, which is why straight walls bow so obviously in anything it takes. The
daemon's cone had been drawn at 65 degrees, a guess at a generic webcam, and was
therefore claiming a third of what the camera could see.

Those figures are from two sweeps taken the same day, 2026-08-19, one panning and one
tilting: they agree on the scale to half a percent, and each pins the coordinate of
the centre that it moves along while saying almost nothing about the other, so `cx`
comes from the pan run and `cy` from the tilt run. **Run both.** An earlier pan-only
run put the centre 97 px off in `cy` and nobody would have known.

The centre is the row worth reading twice. The lens axis is not the middle of the
picture, and [`face_tracking/aiming.py`](../face_tracking/aiming.py) needs it to be
told apart from the middle, because the thing it is trying to do is put a face in the
*middle* — aiming at the axis instead would leave everyone two and a half degrees
high in every frame the rover took.

## Whether the aiming can actually use it

[`calibrate_aim.py`](../usb_cameras/calibrate_aim.py) asks the question the tracking
loop depends on, which the field of view does not answer: given a face at some pixel,
do the degrees `aiming.py` works out put it in the middle **in one move**?

```bash
python usb_cameras/calibrate_aim.py --selftest       # no rover needed
python usb_cameras/calibrate_aim.py aim/ --from -20 30
python usb_cameras/calibrate_aim.py aim/ --mirror    # the other side of the frame
```

It cuts a textured patch out at a known pixel, commands what the model says would
centre it, and measures where the patch really ended up — rectifying both frames
through the fitted lens onto a common tangent plane first, so that the fisheye's own
squeeze towards the edges is not read as aiming error. `--selftest` renders a camera
whose lens and kinematics are known and recovers a rendered residual to a tenth of a
degree, which is what makes the hardware numbers worth anything.

What it found on this rover, 2026-08-19, is in `aiming.lens_recipe()`: the separable
pixels-times-a-gain model that used to fly left a face up to 20 degrees out towards
the corners and got worse the more the camera was already tilted. Solving the
pan-then-tilt geometry instead lands within a few degrees everywhere.

**Measure in a room with something far away in it.** The gimbal pivots a few
centimetres behind the lens, so a subject nearer than about a metre moves by parallax
as well as by rotation, and no rotation-only model can take that out. A first run of
this test was done against a sofa 40 cm away and produced a confident, wrong answer
in the opposite direction.
