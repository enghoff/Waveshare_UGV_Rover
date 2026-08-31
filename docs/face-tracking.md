# Face tracking: local YuNet + gimbal aiming

Face tracking is a local rover capability. The gimbal's UVC camera supplies MJPEG
frames, `face_tracking/yunet.py` detects faces on the Banana Pi, `aiming.py`
turns a detected face into pan/tilt targets, and the daemon sends those targets
to the driver board.

There is no remote face-detection service in the current system.

```text
gimbal UVC camera
      |
      | MJPEG 640x480
      v
face_tracking/yunet.py      LocalDetector / OpenCV YuNet
      |
      | [x, y, w, h, score]
      v
face_tracking/aiming.py     Target + Gimbal + Scan
      |
      v
rover_daemon -> driver-board UART -> ST3215 pan/tilt servos
```

The same aiming law is also used by `face_tracking/track_face.py` on a workstation
and `track_face_pi.py` on the rover. Keeping the geometry and control law in one
file is intentional: two programs controlling the same gimbal must not carry two
sets of gains, lens parameters or scan rules.

## Detector

The current detector is OpenCV's YuNet (`FaceDetectorYN`), using the
`face_detection_yunet` ONNX model. On the rover the wrapper is
`face_tracking/yunet.py::LocalDetector`.

Current defaults in code:

- capture/detection width: 640 pixels;
- normal rover frame: 640×480 MJPEG;
- detector threads: 3;
- acquisition/keep thresholds: owned by `aiming.py`/`yunet.py`, not duplicated in
  documentation;
- model file: `face_detection_yunet.onnx` beside the deployed tracking code.

The thread count is deliberately three rather than four. The detector can use all
four cores, but a rover whose mapping/navigation and hardware services share the
same board benefits more from leaving one core available than from the last few
milliseconds of detector throughput.

Measured on the Banana Pi M4 Zero on 2026-08-23, 640×480:

| OpenCV threads | YuNet detect time |
|---:|---:|
| 1 | ~310 ms |
| 2 | ~180 ms |
| 4 | ~146 ms |

JPEG decode on this board is about 7 ms for the same frame, so detection rather
than decode dominates the local vision cost. The running code uses three threads
as the compromise between face rate and leaving scheduling room for the rest of
the rover.

`LocalDetector` deliberately presents the same simple interface to the tracking
loop every frame:

```python
faces = detector.detect(jpeg, exposed_at)
```

An empty list means the image was processed and contained no accepted face.
`None` means the detector could not produce a trustworthy answer. The distinction
matters: "there is nobody here" and "nobody looked at this frame" require
different control behaviour.

## OpenCV on the rover

The Banana Pi had neither `pip` nor `python3-venv`; the Jetson has pip and this
still does not use it, because one install path that works on both boards is
worth more than a second that works on one. The rover does not rely on a system
OpenCV package either. `face_tracking/install_opencv.sh` downloads a pinned
aarch64 `opencv-python-headless` wheel, verifies its SHA-256 and unpacks it into
`vendor/` beside the deployed tracking code.

```bash
ssh orin '~/ugv/install_opencv.sh'
```

The installer finishes by importing OpenCV and constructing `LocalDetector`, so
"the wheel unpacked" is not mistaken for "YuNet can actually load".

`yunet.py` first tries an ordinary `import cv2`; if that fails it adds the local
`vendor/` directory. A workstation with a normal OpenCV installation therefore
uses that installation, while the rover uses its unpacked wheel.

## Why MJPEG stays in the camera path

The gimbal camera can produce uncompressed frames, but the running rover keeps the
feed as MJPEG. On the Banana Pi JPEG decode is cheap, while uncompressed 640×480
at camera rate consumes much more of the shared USB path and can build a stale
FIFO when a reader cannot drain at camera speed.

For a feedback loop, a stale frame is worse than a skipped frame. The tracking
path therefore keeps the newest complete picture and drops pictures it cannot act
on in time rather than building a queue of old observations.

One-shot tools such as `look`/`camera_jpeg` use the bounded snapshot path rather
than keeping a 30 fps feed alive. The continuous feed is for face tracking, where
continuous frames are actually needed.

## Timing: a pixel only means something at one camera pose

The camera sits on the gimbal it is controlling. A face centre in one image is
therefore not a fixed point in the world: every pan/tilt command changes where
that same face will land in the next frame.

`aiming.py` solves in angles rather than repeatedly reusing old pixels. The loop
records where the gimbal was commanded over time and associates a detection with
the exposure time of the frame that produced it. That is why the camera path
carries V4L2 exposure timing and why a fabricated "fresh" timestamp is worse than
an explicitly uncertain one.

The control loop also distinguishes:

- a fresh face measurement;
- a brief missed detection while an existing target is still held;
- no target, where the scan pattern may search for one;
- detector/camera failure, where blind sweeping should stop rather than pretending
  the last observation is current.

With no target the loop has two behaviours rather than one, and which is right
depends on the wheels. Parked, it sweeps (`Scan`). Under way it holds the camera
straight ahead at the scanning height and stops sweeping (`Ahead`): the pan would
otherwise add to the rover's own motion, smearing the picture past what the sweep
rate was measured for, and the direction it looks at least often is the one the
rover is driving into. A face found while driving is followed exactly as it would
be standing still. The loop asks the navigator once a frame -- `Rover.driving`,
which is the move mutex and not a flag kept beside it -- so a move that begins or
ends between two frames is picked up by the next one. `tracking_status()` reports
which of the two is running as `searching`.

The exact gains, deadband, scan speed and target grace are executable policy in
`face_tracking/aiming.py`. Treat that file as authoritative if a prose value in an
older investigation differs.

## Lens and aiming calibration

Two separate questions need measuring:

1. **What direction does each image coordinate represent?** — lens/FOV/principal
   point, measured with `usb_cameras/calibrate_fov.py`.
2. **Does commanding the derived angle actually centre a target?** — end-to-end
   aiming, measured with `usb_cameras/calibrate_aim.py`.

Typical workflow:

```bash
python usb_cameras/calibrate_fov.py --selftest
python usb_cameras/calibrate_fov.py sweep/ --axis pan
python usb_cameras/calibrate_fov.py sweep/ --axis tilt
python usb_cameras/calibrate_aim.py aim/
```

Do aiming calibration against something sufficiently far away. At close range the
lens translates as the gimbal rotates, so parallax becomes a significant part of
where the target moves and a pure angular calibration can look wrong even when the
rotation geometry is right.

The map's camera cone and the tracking solution should derive from the same lens
model rather than keeping independent FOV numbers. If the camera/lens changes,
re-measure the lens first and then the aiming.

## Diagnostics

Useful layers to test independently:

**Can YuNet see the face in a known picture?** The daemon exposes its diagnostic
`detect_in` control call (not a model tool) for exactly this separation. It runs
YuNet repeatedly over a supplied JPEG so detection can be checked without the
camera moving underneath the experiment.

**Does the camera supply current frames?** Tracking status includes frame age and
recent detector timing. A low frame rate can be compute; a high frame age is a
buffering/timing problem and needs a different fix.

**Does the geometry centre correctly?** Use `calibrate_aim.py`, not a live
tracking run whose detector, timing, servo and target selection all change at once.

**Does the target stay locked?** Only after the three questions above are known
should gain/smoothing/grace be tuned from a live run.

## Bench tracking

`face_tracking/track_face.py` remains a useful workstation instrument. It opens the
UVC camera directly, runs YuNet locally and sends only gimbal commands to the
Waveshare board. It never drives the wheels.

`track_face_pi.py` is the headless rover-side loop used as a standalone diagnostic.
Do not run it while the daemon owns the UART/camera.

The normal deployed path is through `rover_daemon`, which imports the same
`LocalDetector` and aiming code.
