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
- Colour field of view from device intrinsics: 70.1 degrees by 43.0 degrees.
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

To test the hardware directly, stop the service first:

```bash
ssh orin '~/ugv/oak_depth/restart.sh --stop'
ssh orin 'python3 ~/ugv/oak_depth/selftest.py --frames 60'
ssh orin '~/ugv/oak_depth/restart.sh'
```

The direct self-test proves library import, udev access, enumeration, USB speed,
pipeline upload and live frames. It is a hardware test and is not expected to run
on the workstation.

The camera normally powers down after the rover has been still for 30 seconds and
wakes for movement or inspection. An intentionally off camera is healthy; a
camera expected to be on with stale frames is not.
