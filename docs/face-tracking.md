# Face tracking

Exercised by [`face_tracking/`](../face_tracking). This is the one part of the
suite that runs two components against each other rather than exercising one: the
rover's USB camera module supplies the picture, OpenCV's YuNet detector finds a
face in it, and the pan/tilt's two servos are steered to keep that face in the
middle of the frame. Reach for it after both halves have been checked on their
own, not instead of checking them.

It never touches the wheels. The only command it sends that moves anything is
`{"T":134,...}`, which reaches the camera servos and nothing else, and it leaves
the firmware's heartbeat at its default — that timer exists to stop the base, and
this never starts it.

Two programs run the same loop with the pieces in different places.
`track_face.py` runs on the workstation with the camera on its own USB and the
board over WiFi; `track_face_pi.py` runs on the rover's Pi with the camera and
the control law there, the detector on another machine over HTTP, and the servos
down the GPIO UART. They share every constant through `aiming.py`, because two
copies of a calibrated control law are two different robots.

## Running it

```powershell
python face_tracking\track_face.py             # finds the board and the camera itself
python face_tracking\track_face.py --no-move   # detect and draw, command nothing
python face_tracking\track_face.py --no-scan   # stay put when no one is in shot
python face_tracking\track_face.py --scan-rate 40   # sweep faster while looking
```

`c` recentres, space re-targets the largest face, `h` holds position, `q` quits.

`--no-move` runs everything and commands nothing, which is the way to check the
picture and the detections before letting it move the camera; rejected detections
are drawn thin with their scores, so it shows what was seen and passed over.

From the rover instead, with a [face detection service](../face_detect/README.md)
reachable on the network:

```bash
python3 track_face_pi.py                        # camera, UART, default detector
python3 track_face_pi.py --service HOST:8768    # name the detector
python3 track_face_pi.py --no-move
```

It finds both halves itself: the driver board the way
[`drive_gamepad.py`](driver-board.md#finding-the-board) does, and the camera by
its USB id (`0abd:8050`) among the machine's other webcams, with `--camera` to
name one. The detector's model is a 230 kB ONNX file OpenCV does not ship,
fetched once on the first run to sit beside the script — Haar cascades would need
no download, but OpenCV 5 dropped them from the wheel entirely.

## Aiming is calibrated, not assumed

Taking a patch of the scene, commanding a known move and finding that patch again
by template matching gives **9.65 px of image shift per commanded degree in pan
and 9.5 in tilt** at 1280×720, symmetric in both directions, from which `+X` pans
right and `+Y` tilts up. A half frame is 640 px, so 66 of those degrees.

That first suggested the firmware's "degrees" were about half a real one, on the
grounds that no sane lens is 132° wide — but the
[firmware source](https://github.com/waveshareteam/ugv_base_general/blob/main/General_Driver/gimbal_module.h)
maps them `×11.375` into the ST3215's 4096 counts per turn, so a commanded degree
*is* a real degree and the lens really is that wide. The barrel distortion in any
frame it takes confirms it. The controller works in the measured units either way
and never needs the lens FOV, which is how the wrong inference survived so long.

## Dead time is what makes this hard

The loop is closed through the world but open around the servos, which report
nothing back, so the angles are a model kept true by centring at startup and on
exit. The number that governs everything else is the delay between a command
going out and the picture showing any sign of it: **266 ms**, measured over five
50° steps, which at 30 fps is **eight frames**. Everything commanded inside that
window is still in flight and invisible.

A controller that simply corrects what it can see therefore issues the same
correction eight times over, and the result is not sluggishness but divergence —
the camera sails past the face, corrects harder the other way, and pins itself
against a limit. From outside it looks precisely like a camera avoiding people,
and that is what this did before the compensation went in. The fix is to correct
from where the camera *was when the frame was exposed*, so motion already in
flight is subtracted instead of commanded again. Measured against a fixed target
at a −55° offset:

| | error, start → settled |
|---|---|
| correcting against the current angle | 0.80 → 0.67, swinging the full frame, ended pinned at pan +180 |
| correcting against the angle at exposure | **0.80 → −0.01**, no overshoot |

On a live subject it reaches centre in under a second and holds a lock for ten
seconds at a stretch, with nothing pinned at a limit. A sign error looks the same
from the outside as too much gain, so tell them apart by watching one correction
from a standstill: the wrong sign moves away immediately, too much gain moves the
right way first and overshoots.

## Running the loop from the rover

`track_face_pi.py` puts the camera and the control law on the Pi, the detector on
another machine, and the servos on the GPIO UART.

The Pi cannot detect anything itself. An ARM1176 with no NEON measures
0.039 GFLOP/s on conv-shaped work, so YuNet would cost it about a second a frame
against 6.5 ms on a desktop CPU. It never even decodes the picture: one 640×480
JPEG costs it 93 ms to decode, and forwarding those exact bytes untouched costs
30% of the core at 30 fps.

Two things change for the better in the move. The dead time stops being a
constant — V4L2 stamps every buffer at start of exposure, the stamp rides out to
the detector and comes back attached to the boxes, and the controller is told
exactly which moment it is answering rather than assuming 266 ms. And the loop
gets shorter: measured end to end through an SSH tunnel it runs at 11 fps with
boxes in hand ~145 ms after the light, and a direct path should roughly halve the
round-trip half of that, against 266 ms of dead time with the camera on the
workstation's own USB.

One measurement in that chain is worth knowing about, because it was wrong here
for a while. Exposure to a complete frame in hand looked like 98 ms, and 25 ms of
that was `v4l2-ctl`'s own stdio: its stdout is a pipe, so libc buffers it and the
tail of each frame waits for the next one to push it out. Under `stdbuf -o0` it
is 41 ms, barely varying between 320×240 and 720p — which is what identifies the
rest as the camera's own pipeline. The number the control law is most sensitive
to was being set by a buffering default.

Frames are dropped on purpose: the camera runs at 30 fps, a round trip does not,
and only the newest frame is ever sent. A queue here would not be slowness, it
would be a rover aiming where somebody used to be. The count is displayed, since
a silently decimated stream looks identical to a healthy one.

## Scanning, and how fast it may sweep

With nobody in shot it sweeps its whole range — pan end to end, then a step of
tilt and back the other way — and follows the moment a face appears. Two tilt
levels cover the range because the frame takes in 76° of the 120° available, so
levels a half-frame inside each end reach both limits and overlap.

The sweep rate is the pacing question, and it is answered with measurements
rather than taste. Motion smear was calibrated by blurring a still frame until
its sharpness matched what the moving camera produced: **~2 px at 25°/s**,
against a detector that still finds a 50 px face (someone across a room) under
9 px of smear and a 100 px face under 27. Faster detects perfectly well — 90°/s
still finds faces — but smear grows with exposure time and a dim room lengthens
it, and the sweep is visibly rougher above about this speed: at 25°/s the picture
moves 8.0 px a frame against the 7.5 expected with 1–4% of frames not moving at
all, while at 45°/s it delivers 11 px of an expected 13 and stalls on nearly a
fifth. Slower is smoother and sees no less. `--scan-rate` moves it; `--no-scan`
stops it sweeping.

One trap worth recording: `dt` multiplies the sweep step, so a single slow frame
commands a large jump, the board takes longer to answer a large jump, and the
next frame is slower still. That spiral took the loop from 25 commands a second
to 0.9. Clamping `dt` breaks it, after which every rate tried held 25 fps with
nothing lost.

## Two thresholds, not one

A false positive here is not cosmetic: the camera locks onto the wrong thing and,
unlike a person, a sofa never walks away. In one room the arm of a black sofa
against a yellow wall scored **0.79**, while real faces scored 0.88–0.91 and a
distant half-profile one 0.73. No single threshold separates those, so acquiring
a target needs 0.85 while keeping one needs only 0.60 — and the low bar is safe
because a weak detection is accepted only close to where the face already was.

There is no recognition behind any of this. YuNet returns a box and a confidence
and nothing else, so the lock is kept by proximity rather than identity, and
somebody who leaves the frame and comes back is a stranger to it. See
[rover_daemon](../rover_daemon/README.md#what-it-cannot-do-and-will-not-pretend-to)
for what that rules out.

## Why the camera used to step from pose to pose

Both gimbal commands carry a speed, and the firmware passes it to the servo in
the servo's own units — `map(spd, 0, 360, 0, 4095)`, which is plain degrees per
second. Sending `SPD: 0` means *unlimited*, so every correction was "get there as
fast as you can": a 1° lunge at the servo's full 130°/s, over in 8 ms, followed
by 25 ms of standing still, thirty times a second. Naming the speed the motion
actually wants fixes it, and `T:134` is used rather than `T:133` because it takes
the two axes separately — otherwise a barely-moving tilt is dragged along at
whatever pan needed. Measured, to calibrate the units: `SPD` 20 gave 20.4°/s, 40
gave 40.2, 80 gave 75.3 and 150 gave 114.6, against a ceiling of about 130.

Two limits of the mechanism remain, and no amount of commanding gets around
either. The firmware truncates angles to whole degrees, so **the smallest
possible move is 9.65 px** of picture — fractional angles are silently rounded,
and half-degree steps produce one jump per *pair* of commands. And there is
**about 2° of backlash**: after a change of direction the first two commanded
degrees produce no motion at all, then it tracks linearly again. The deadband
sits just outside that, which is what stops the camera dithering across it.
