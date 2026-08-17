# The OAK-D-Lite

Exercised by [`oak_camera/`](../oak_camera). Two companion documents cover the
findings that constrain how these scripts are written: the
[USB link](oak-usb-link.md) and the [depthai version pin](depthai-version-pin.md).

## What the board is

It reports itself as **OAK-D-LITE**, product **OAK-D-LITE-AF** — an OAK-D-Lite
with the autofocus colour module, not a full OAK-D. What `probe_device.py`
reported for one unit on 2026-08-11, which is also the shape of what it prints:

| | |
|---|---|
| VPU | Movidius MyriadX, RVC2 |
| CAM_A | IMX214, autofocus, 4208×3120, colour, 69° horizontal FOV |
| CAM_B / CAM_C | OV7251, fixed focus, 640×480, mono, 73° horizontal FOV |
| Stereo pair | CAM_B (left) / CAM_C (right), baseline 7.50 cm |
| IMU | BMI270 |
| IR drivers | none — the Lite has no dot projector or flood illuminator |
| Idle temperature | ~37 °C |

No driver install is needed on Windows: the camera enumerates as a WinUSB device
(`USB\VID_03E7&PID_2485`) in the MyriadX ROM bootloader state, and depthai uploads
firmware over USB every time a pipeline opens. A device showing as `Movidius
MyriadX` in Device Manager but never as a model name has simply not been booted
yet; that is the idle state, not a fault.

## The tools, in triage order

Each adds one layer to the last, so the first one that fails localises the
problem.

### `probe_device.py`

Enumerates attached OAK devices and boots each one, because the USB descriptor
says only "Movidius MyriadX" — model, populated sensors and calibration all live
on the device. Prints board and product name, USB speed, bootloader version, IMU,
IR drivers, chip temperature, every camera socket with sensor and resolution, the
stereo pair, the baseline and per-socket FOV.

It is the deliberate exception to the USB-speed rule: it asks for `SUPER_PLUS`
first and falls back to `HIGH`, printing which it got, so it is how you find out
whether a cable or port change actually bought you USB3. Its `wait_unbooted`
helper polls up to 45 s for a device to drop back to `X_LINK_UNBOOTED` after a
failed boot.

### `inspect_calibration.py`

Prints per-socket intrinsics (fx, fy, cx, cy with the resolution they were
calibrated at) and all fourteen distortion coefficients, for the user calibration
in use *and* the factory copy, so an overwritten or corrupt user calibration is
obvious side by side. On this unit the two are byte-identical. Reads stored state
only — it opens no streams, which is what makes it useful for separating a device
fault from a pipeline fault.

### `read_crash_dump.py`

Reads the device's stored firmware crash dump with `clearCrashDump=True`, prints
the per-core cause — error id, assert file/line/function, hardware trap, plus the
firmware console buffer — and saves the full JSON under `oak_camera/crash_dumps/`
named by device id and timestamp. It parses both schema shapes, since 2.x nests
the cause under `crashReports[].errorSourceInfo` and 3.x flattens it into
`reports[]`.

Run it after any device-side crash to get the actual cause, and to clear stale
dumps that would otherwise keep `hasCrashDump()` true and point the next
diagnostic at an old crash. On depthai 3.x clearing is not optional — see the
[crash-dump trap](depthai-version-pin.md#the-crash-dump-trap-on-3x).

### `preview_depth.py`

Live stereo depth from the mono pair at 640×480, 15 fps, HIGH_DENSITY preset with
left-right check and subpixel, turbo-colour-mapped over 200–4000 mm with invalid
pixels black and the distance under the cursor printed. Unaligned, so its origin
is the right mono camera rather than the colour camera. Exercises CAM_B, CAM_C and
the stereo engine and nothing else.

By default each pixel holds its last valid depth and darkens to black over 1 s
rather than going black the frame the device stops reporting it; `--fade SECONDS`
resizes that window and `--fade 0` turns it off. On weak texture the valid pixels
flicker in and out frame to frame and an unheld view strobes; holding them smooths
that out while still letting a genuinely lost pixel disappear. Ageing is by wall
clock, not frame count, so a stalled link fades out rather than freezing the last
frame, and the cursor readout appends the age of a held value so stale depth never
reads as fresh.

### `preview_rgb.py`

Live 960×540 colour from CAM_A at 30 fps. With `--depth` it also builds the stereo
pair and blends depth over the colour frame at 15 fps; `--fps` overrides either
default. With `--depth` it is the heaviest thing here — all three sensors plus
alignment, which is the most the USB link is ever asked to carry.

The alignment is two calls — `setDepthAlign(CAM_A)` warps disparity into the
colour camera's geometry and `setOutputSize(960, 540)` emits it at the preview's
size — so `depth[y, x]` and `rgb[y, x]` are the same ray and blending is a plain
`addWeighted` with no host-side remapping. Aligning this direction works because
the mono pair's 73° horizontal FOV is wider than the colour camera's 69°; the
other direction would leave the edges uncovered. The two streams come from
different nodes, so they are paired by `getSequenceNum()` rather than assumed to
arrive in lockstep, and the render loop only re-blends when something changed
instead of recolouring the same pair at `waitKey` rate.

Sizing is set by the link, not by the request — see
[the throughput ceiling](oak-usb-link.md#what-fits-on-the-link).

## Depth semantics

Depth is millimetres along the optical axis from the *right* mono camera's optical
centre, not from the front face of the housing; `setDepthAlign` moves that origin
to the colour camera, which is why `preview_rgb.py --depth` and `preview_depth.py`
disagree on the same scene. Zero means no valid disparity for that pixel — closer
than the pair can triangulate, textureless, or occluded in one view.

The 7.5 cm baseline sets the practical range: below roughly 20 cm the two views
stop overlapping, and beyond a few metres disparity quantisation makes each step
tens of millimetres. A desk scene reads a median of about 1.6 m with 74–82 % of
pixels valid.

## Calibration oddity

CAM_B's distortion coefficients are far larger than CAM_C's — `k1=+41.95,
k2=-36.68` against `k1=-1.66, k2=-0.76`. That is what the factory wrote (user and
factory copies match exactly) and both sensors stream fine on 2.x, so it is not a
fault. Treat undistortion output on the left camera with suspicion before trusting
it, and re-check against a straight-edge target if it matters.
