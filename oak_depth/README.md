# OAK-D-Lite depth service

The OAK-D-Lite supplies aligned colour and stereo depth on loopback port 8770.
World-state perception uses it to range visual regions. Navigation continues to
use the lidar.

DepthAI uploads firmware whenever the device opens. The camera has no persistent
application firmware, so the service must stay alive. Switching power off closes
the device while keeping HTTP available; switching on opens a new pipeline and
takes several seconds.

## Current pipeline

- DepthAI 2.32.0.0, pinned because tested 3.x releases fail stereo on this unit.
- USB2 (`HIGH`) for stability on the rover's shared USB path.
- 640x360 MJPEG colour and 320x180 aligned depth at 15 fps.
- Valid stereo range: 0.2 to 6 m.
- Colour field of view 65.2 degrees by 39.6, from the stored calibration cut
  to the window the 1080p mode actually reads (`colour_lens.py`).
- `/health` publishes the colour matrix and stored distortion coefficients so a
  calibration bench can measure the error from treating this near-pinhole lens
  as a pinhole.

The colour lens is worked out for the sensor mode as well as the output size.
depthai's `getCameraIntrinsics(socket, w, h)` assumes a mode reads the sensor's
full width, and 1080p reads only its middle 3840 x 2160, so until 2026-09-30 this
service published a focal length 9.6% short (456.5 pixels where the pixels obey
500.3) and a 70.1-degree field of view. Matching the colour picture against the
right mono camera confirmed the corrected lens to 0.2%. The device's own depth
alignment was never affected. `python oak_depth/test_colour_lens.py` checks the
arithmetic at a desk.
- Stereo baseline: 7.5 cm.

Depth is aligned to the colour camera. A normalized box from `/frame` can be sent
unchanged to `/ranges`.

## HTTP API

- `GET /health` reports device, firmware, USB speed, frame age and power state.
- `GET /depth` returns a coarse depth grid and sector ranges.
- `GET /depth.png` returns the latest depth image for a person.
- `GET /frame` returns paired JPEG colour plus age and size headers.
- `GET /power` reports `on`, `off` or `waking`.
- `POST /power` accepts `{"on": true}` or `{"on": false}`.
- `POST /ranges` accepts normalized `[left, top, right, bottom]` boxes.

Range extraction finds the near surface inside each box and reports both metres
and estimated uncertainty. The disparity error model is plausible but has not
been validated against tape-measured targets.

## Install and run

```bash
ssh orin 'sh ~/ugv/oak_depth/install.sh'
ssh orin '~/ugv/oak_depth/restart.sh'
ssh orin 'curl -s http://127.0.0.1:8770/health'
```

The installer unpacks the pinned wheel beside the component and installs the
udev rule for the `03e7` device. `run_oak_depth.sh` supervises the process and
reopens the camera after a fault. Only one process can own the OAK.

A server that had been running for half a minute or more is restarted as soon
as the camera is back on USB; one that failed straight away waits 15 s, so a
missing camera does not fill the log. The fast path exists because the OAK drops
off USB mid-stream: five times on 2026-10-02, driving and parked, with the
gimbal camera on the same hub unaffected. The fixed 15 s wait made each drop
about twenty seconds without depth. The cause of the drops is not known yet:
power and the cable up the gimbal are both open.

To test the hardware directly, stop the service first. There is no stop option;
stopping the supervisor stops the server with it, and the bracket keeps the
pattern from matching the ssh command that carries it:

```bash
ssh orin 'pkill -f "oak_depth/run_oak_depth[.]sh"'
ssh orin 'python3 ~/ugv/oak_depth/selftest.py --frames 60'
ssh orin '~/ugv/oak_depth/restart.sh'
```

The first program to open the device afterwards reports that it "has crashed"
and saves a crash dump. That is the device's watchdog firing when the service
let go of it, which it always does, not a fault in the test.

The direct self-test proves library import, udev access, enumeration, USB speed,
pipeline upload and live frames. It is a hardware test and is not expected to run
on the workstation.

The camera normally powers down after the rover has been still for 30 seconds and
wakes for movement or inspection. An intentionally off camera is healthy; a
camera expected to be on with stale frames is not.
