# The host's USB cameras

Exercised by [`usb_cameras/`](../usb_cameras). One script,
`preview_usb_cameras.py`, which needs only OpenCV — no depthai, no pyserial. This
covers the UVC webcams on the machine driving the rover, not the rover's own
hardware. The OAK-D-Lite is not a UVC device and will not appear here; use
[`oak_camera/preview_rgb.py`](oak-d-lite.md#preview_rgbpy) for that one.

## How it finds cameras

It probes capture indices 0–7 — DirectShow hands out dense indices from 0, so a
small range is enough — and keeps the ones that actually deliver a frame, allowing
three reads because some webcams hand back nothing until the stream warms up.
DirectShow is tried first and MSMF only as a fallback for cameras DirectShow cannot
see: DirectShow opens in tens of milliseconds where MSMF can take seconds per
device. It requests 1280×720 on open.

OpenCV exposes no device names, so friendly names are borrowed positionally from
Windows' `Win32_PnPEntity` list. DirectShow enumerates in the same order, including
devices that refuse to stream such as IR sensors, which is why labels are indexed
by capture index rather than by list position — but any oddity in the PnP list
shifts every label, so treat the names as a hint, not an identity.

Only one camera is held open at a time, so the preview still works when several
cameras share a USB controller's bandwidth. A camera that vanishes mid-stream —
unplugged, or grabbed by another app — returns failed reads forever rather than
erroring, so it gives up after 30 failures (about a second) and moves to the next.

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
