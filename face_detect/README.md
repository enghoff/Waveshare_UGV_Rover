# Face detection service

YuNet on MEDIA's CPU, behind an HTTP request. JPEG in, boxes out. It exists so
the rover can see: a Pi 1 cannot run this detector at any useful rate, but it can
forward a picture and steer servos, which is what
[face_tracking/track_face_pi.py](../face_tracking/track_face_pi.py) does.

```
  admin@rpi (on the rover)                    root@media
  ------------------------                    ----------
  USB camera --MJPEG--> forward, never decode
                            |  POST /detect, one frame in flight
                            |                 YuNet, 4 threads, ~6 ms
  ST3215 servos <--UART-- aiming.py  <--boxes + the frame's own timestamp
```

## Why this one is on the CPU

The other three services in `/opt` share one 8 GB card and are mutually exclusive
because of it — `~/switch_service.sh` stops whichever is running before starting
the next. With `voice-chat` up the card reads 6656 of 8192 MiB used.

Face detection does not join that queue. YuNet is a 230 kB CNN; a 5700G runs it
in 6.5 ms and never notices. So this service is simply always there, concurrent
with whatever owns the card, and **it is deliberately not in
`switch_service.sh`** — adding it would put face tracking back into the interlock
and mean the rover could only see while nobody was talking to it. For the same
reason it is enabled at boot: of the other three only `qwen3-vl` is, so an
instance that has restarted comes back on vision, and this must not depend on
somebody having switched anything.

Measured here, on a 640x480 frame, against `cv2.setNumThreads`:

| threads | 640x480 | 320x240 |
|---|---|---|
| 1 | 14.5 ms | 3.6 ms |
| 2 | 9.5 | 2.2 |
| **4** | **6.5** | **1.7** |
| 8 | 5.9 | 1.4 |

Four is where the curve flattens. The eighth thread buys 0.6 ms and costs half of
what is left of the box, which is a bad trade for a service whose whole argument
is being a good neighbour.

## The protocol

```
POST /detect?ts=<opaque>&score=<float>&width=<int>     body: one JPEG
  -> {"ts": <echoed>, "w": 640, "h": 480,
      "faces": [[x, y, w, h, score], ...],
      "decode_ms": 1.0, "detect_ms": 6.2}

GET /health
  -> {"ok": true, "frames": ..., "detect_ms_median": ..., ...}
```

Boxes come back in **full-frame** pixels even when `width` caused a downscale, so
the caller never has to know whether one happened.

`ts` is echoed untouched and never parsed. That is the point of it: the rover
stamps each frame with V4L2's start-of-exposure time and gets it back attached to
the boxes, so its control loop knows exactly which moment those boxes describe
instead of assuming a fixed dead time. The two machines' clocks never have to
agree, because the stamp only ever means anything on the rover.

`score` is the caller's threshold, not a policy here. The rover runs two — a high
one to acquire a face and a low one to keep following one, both in
[face_tracking/aiming.py](../face_tracking/aiming.py) — and only it knows which
applies, so it asks for everything above its low bar and decides for itself.

One frame is in flight at a time, by the caller's construction rather than any
rule here: that is the backpressure, and it is what stops a queue of stale frames
forming when the loop is slower than the camera.

## Deploying

```bash
ssh root@media 'mkdir -p /opt/face_detect && /root/.local/bin/uv venv --python 3.10 /opt/face_detect/.venv'
ssh root@media 'VIRTUAL_ENV=/opt/face_detect/.venv /root/.local/bin/uv pip install opencv-python-headless numpy'
scp face_detect/server.py root@media:/opt/face_detect/
scp face_tracking/face_detection_yunet.onnx root@media:/opt/face_detect/
scp face_detect/face-detect.service root@media:/etc/systemd/system/
ssh root@media 'systemctl daemon-reload && systemctl enable --now face-detect'
```

The model is the same 230 kB ONNX the workstation script uses, copied rather than
duplicated in the repo; `server.py` will fetch it from the OpenCV zoo if it is
absent, so a fresh box does not fail to start for want of it.

No FastAPI and no uvicorn, unlike `voice_chat/`. A turn here is one request and
one response with nothing to stream, so the stdlib's threading server does it in
a fraction of the dependencies — which matters on a box where the other three
services each carry a multi-gigabyte CUDA stack.

## The firewall — this is the part that will catch you

MEDIA's WSL runs `networkingMode=mirrored`, which puts a **Hyper-V firewall** in
front of it whose `DefaultInboundAction` is `Block`. A listening socket alone is
therefore reached by nobody: the only reason `ssh root@media` works from the LAN
at all is a rule someone already made, `OpenPI WSL SSH TCP 22 (LAN only)`.

So this port needs a rule, in an **elevated** PowerShell on the MEDIA desktop
(WSL cannot make one — `New-NetFirewallRule` returns "Access is denied" through
interop). One rule covers the block the services here live in, `8765-8774`,
rather than one rule per service:

```powershell
New-NetFirewallRule -DisplayName "MEDIA services TCP 8765-8774 (LAN only)" `
  -Direction Inbound -Action Allow -Protocol TCP -LocalPort 8765-8774 `
  -RemoteAddress LocalSubnet -Profile Any
```

Six of those ten are unclaimed, which is the point — a new service on 8769 is
reachable without a second trip to an elevated prompt. The cost of that
convenience is worth naming: anything that binds a port in this range is on the
LAN the moment it starts, with `LocalSubnet` as the only thing in front of it.
Nothing here authenticates. Bind loopback deliberately for anything that should
not be, the way `voice-chat` does.

Until that exists, the rover can still reach the service through SSH, since it
already has a key on MEDIA:

```bash
ssh 192.168.1.47 'ssh -f -N -L 8768:127.0.0.1:8768 media'
ssh 192.168.1.47 'cd ~/ugv/face_tracking && python3 track_face_pi.py --service 127.0.0.1:8768'
```

That works, and it costs about **50 ms of round trip** — measured, and not the
encryption, which the Pi does at 25 MB/s. It is the tunnel itself: a bodyless
`GET /health` through it takes 51 ms, against 10 ms to a stub on the Pi's own
loopback.

Whole, through the tunnel, a round trip measures 85 ms and the tracking loop
settles at **11 fps**. The parts of a direct round trip measure 13 ms (the Pi's
HTTP stack) + 9 (network) + 6 (detection) = 28, so the rule should roughly double
the loop rate — but that is a sum of parts, not an end-to-end measurement, and it
is worth re-taking once the port is open.

## Checks

```bash
# from anywhere that can reach it
curl -s http://192.168.1.3:8768/health

# a photograph with a known face in it
curl -sSLo /tmp/lena.jpg https://raw.githubusercontent.com/opencv/opencv/4.x/samples/data/lena.jpg
curl -s -X POST --data-binary @/tmp/lena.jpg -H 'Content-Type: image/jpeg' \
  'http://192.168.1.3:8768/detect?ts=1234.5&score=0.6'
# -> {"ts": "1234.5", "w": 512, "h": 512, "faces": [[207.8, 182.5, 145.9, 206.9, 0.909]], ...}
```

A garbage body returns `{"error": "not a decodable image"}` with the `ts` still
echoed, rather than a 500 — the rover treats any unusable answer the same way and
should not have to tell them apart.
