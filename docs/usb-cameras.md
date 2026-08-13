# The host's USB cameras

Exercised by [`usb_cameras/`](../usb_cameras). One script,
`preview_usb_cameras.py`, which needs only OpenCV — no depthai, no pyserial. It
covers every UVC device on the machine driving the rover: the host's own webcams
and, since it plugs in over USB like any other, the rover's camera module too. The
OAK-D-Lite is the exception — it is not a UVC device and will not appear here; use
[`oak_camera/preview_rgb.py`](oak-d-lite.md#preview_rgbpy) for that one.

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
