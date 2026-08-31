# `lidar_slam/` — the lidar's parser, the map's renderer, and the plug

**The name is wrong and is kept on purpose.** This directory was 2D SLAM and
self-driving on the rover's own host: a correlative scan matcher, an occupancy
grid, a route planner and a drive controller, all fed by the D500 lidar. The rover
navigates with [`ros_nav/`](../ros_nav) now — `slam_toolbox` maps with loop
closure, Nav2 plans and follows — so all of that has been deleted rather than left
to rot beside code that no longer calls it.

What is left is the part that had no replacement, and four files hold it:

- **the LD19 parser**, in C, because it is 0.3 ms here against 25 in Python and
  `lidar_node.py` runs it ten times a second;
- **the room described in words**, also in C, because a language model handed 360
  ranges will confabulate over them and a model told about walls, objects and gaps
  can say something true;
- **the map renderer**, which turns an occupancy grid into a PNG using nothing but
  `zlib` and `struct`, because there is no image library on this board;
- **the plug**, which re-enumerates the lidar in software when it drops off the USB
  bus, which it does, and not rarely.

A dozen deploy lines and `sys.path` fixups name this directory, and `nav_types.py`
keeps its own name for the same reason. Neither name describes its contents.

## Why the C is still C

The first host was a 700 MHz single-core ARM1176 with scalar VFP and **no NEON**.
The board is a Jetson Orin Nano now — aarch64, six cores — and was a quad-core
Banana Pi M4 Zero in between, but the row that justified this file was never about
the core count:

| stage | Python + numpy 2.2.4 | C, `gcc -O2` |
|---|---|---|
| parse + CRC-8, one revolution of 42 packets | 24.7 ms | 0.05 ms |

Roughly 500×, and it is worth understanding rather than filing under "C is
faster": checking a CRC is 46 sequential table lookups that cannot be vectorised,
so the numpy version is 46 numpy calls on a 42-element array and came out *slower*
than the plain Python loop it was meant to replace. Reaching for numpy is the
instinct that fails here.

`selftest` measures the real thing end to end, on the host it was built on, and
prints the per-revolution cost against the 100 ms a 10 Hz sensor allows. The scan
matcher used to be 17 ms of that and the map update 4 ms; both are gone, which is
why what is left reads as a rounding error rather than as a fifth of a core.

## Building and running

The library is compiled per-machine and is not committed — the ABI is the host's —
so there is no cross-compiler and a checked-in binary would only ever be wrong.
`scp` adds and never removes, so leftover files from when this directory still
held a scan matcher would stay on the host; mirror it instead, and keep the
per-host library:

```bash
rsync -a --delete --exclude 'libslam2d.so' --exclude selftest \
    lidar_slam/ orin:~/ugv/lidar_slam/
ssh orin 'cd ~/ugv/lidar_slam && ./build.sh && ./selftest'
```

Nothing here opens a serial port by itself any more.
[`ros_nav/lidar_node.py`](../ros_nav/lidar_node.py) owns the lidar, feeds bytes in
with `feed()` and publishes what comes back out of `scan_xy()` as `/scan`. To look
at a scan without ROS, `python3 mapimg.py` and `python3 slam2d.py` both run their
own checks, and [`lidar/lidar_view.py`](../lidar/lidar_view.py) draws the sensor
live from a desk.

If nothing arrives at all, the rover's power switch is the first thing to check:
the port enumerates without it, because the CH343 is USB-powered, but the lidar's
motor runs off the 5 V rail behind the switch. A live port and no packets means the
switch is off, not that the cable is wrong.

## The ports, which are not the ones the docs used to name

Two separate serial ports, and they are easy to confuse:

| | port | baud | what |
|---|---|---|---|
| lidar | `/dev/ttyACM0` | 230400 | D500 point stream, one-way, unprompted |
| driver board | `/dev/ttyTHS1` | 115200 | `T:1001` telemetry and motor commands |

The lidar is a **`ttyACM`**, not a `ttyUSB`: it is a CH343 (`1a86:55d3`) behind an
FE1.1S hub (`1a40:0101`), and `cdc_acm` claims it. Neither port is opened from this
directory. The lidar belongs to `lidar_node.py` and the board to `rover_daemon.py`,
which lends it to ROS over loopback — see
[`rover_daemon/board_bridge.py`](../rover_daemon/board_bridge.py). Only one process
can hold either, and the failure when two try is silence rather than an error.

## Frames, since a sign error here is invisible and expensive

The sensor reports a **left-handed** bearing: zero at the front of the sensor,
angle growing clockwise, in hundredths of a degree. The rover frame is
right-handed with **x forward, y left**, and yaw counter-clockwise. The lidar is
also mounted 90° off the chassis.

Both corrections collapse into one angle, `phi = mount_deg - bearing`, applied once
when the sin/cos lookup table is built, so no sign fixing survives past
`build_lut()`. With `mount_deg` at 90 this reduces to `x = r·sin(bearing)`,
`y = r·cos(bearing)` — the same geometry [`lidar/lidar_view.py`](../lidar/lidar_view.py)
draws with its `VIEW_ROTATION_DEG = 90`.

`selftest` pins this down rather than trusting it. It synthesises scans inside a
6 × 3 m room and checks that the sector astride each axis reports that wall's real
distance:

```
sector 0, straight ahead (+x wall)     4.0000 (want 4.0000 +/- 0.0500)  ok
sector 18, rover's left (+y wall)      1.5000 (want 1.5000 +/- 0.0500)  ok
sector 36, behind (-x wall)            2.0000 (want 2.0000 +/- 0.0500)  ok
sector 54, rover's right (-y wall)     1.5000 (want 1.5000 +/- 0.0500)  ok
```

A mirrored frame or a wrong mount offset permutes those four and nothing else
notices.

## The rover's own body, which it was reporting as an obstacle

Two mount posts sit behind the lidar, 12 to 16 cm out, and the sensor sees them.
They were coming back as the nearest obstacle in 59% of revolutions. A ring of
minimum range cannot fix that — 16 cm is well beyond anything it would be safe to
blind the sensor to in *front* — so the mask is a box behind the lidar, measured
from what the sensor actually reports of the rover rather than from a drawing:
returns span 8.5 to 11.2 cm behind and 8.2 to 10.7 cm to each side over 397
revolutions, and `body_back_m` and `body_half_width_m` are those bounds with about
5 cm of margin.

It is applied at `add_point`, the one place a return enters, so everything
downstream agrees about what is real. `test_body_mask` in `selftest.c` feeds a ring
of equal returns at 13 cm and checks that the whole rear half goes and the whole
front half stays, then does it again at 60 cm and checks that nothing goes at all.

## Decimation, which is now the one setting that can quietly make this worse

`max_points` used to default to 300 against a sensor that delivers ~419 a
revolution, and that was a budget rather than a measurement: every point cost a
cache miss in each of 300 candidate poses, so thinning the scan bought 25 ms a
revolution off the scan match. With the matcher gone the scan is no longer an input
to something expensive — it *is* the output, published for `slam_toolbox` and Nav2
— and throwing away a third of it to save arithmetic nobody does any more is simply
a coarser sensor. The default is 600 now, and `lidar_node.py` asks for 1200.

## Telling a model where it is

`Slam2D.describe()` segments the scan into **walls, objects and gaps** rather than
handing over a list of ranges. The segmentation clusters at range discontinuities,
splits the clusters at corners so a rectangular room comes back as four walls
instead of one lumpy ring, and reports openings the rover would actually fit
through.

The one inference it makes is grouping: four narrow objects in a square metre is
the signature of furniture, and no single reading says so. That is the whole answer
to "navigate around the table" — **the lidar never sees a table, it sees four
legs** — and the description says as much in words, so the model can do the naming
that geometry cannot.

Nothing in that reply says where the rover *is*, and that is deliberate. It used to
carry a pose, a match score and a scan count out of the scan matcher; with the
matcher gone those would all have been zeros arriving looking perfectly plausible,
and a console row labelled "scans" reading 0 on a rover whose lidar is spinning
happily is exactly the kind of number somebody debugs for an hour.
[`ros_nav/nav_bridge.py`](../ros_nav/nav_bridge.py) fills them in from
`slam_toolbox` and the transform tree, which do know.

## Drawing the map

`mapimg.py` renders an occupancy grid as a PNG using nothing but `zlib` and
`struct`, because there is no image library on this board at all — no OpenCV, no
PIL. The camera hands over MJPEG already encoded, so nothing here ever needed one.
The grid comes from `slam_toolbox` now, by way of
[`rover_daemon/ros_navigator.py`](../rover_daemon/ros_navigator.py), which shapes it
into the six things this renderer asks a map for. Rendering on the ROS side would
have meant a second renderer, and two renderers become two different pictures of
one room.

What goes out is cropped to a few metres, scaled up, and marked with the rover, its
heading, its track and a one metre scale bar; a raw 400×400 occupancy grid shown to
a vision model is a field of grey speckle that invites confident nonsense. The
caption travels as the tool result whether or not the picture arrives, so a refused
image degrades to a worse answer rather than to an invented one.

Two things in there turn metres into pixels — the array of cells, and `to_px` for
everything drawn over it — and they have to agree, because the grid's axes are
forward and left rather than row and column. They did not, for a while: an extra
transpose reflected the walls about the diagonal and left the rover, its heading and
its track alone, so the track ran across a corridor instead of down it. Each half
looked plausible by itself, and the mock rover draws both halves with one function of
its own, so only the real map showed it. `python mapimg.py` now asserts that a wall
straight ahead and a track that drove into it come out as a vertical line meeting a
horizontal one, that a wall to the left is drawn on the left, and that the arrow
swings counter-clockwise when the heading says left.

The picture is in colour for the same reason. Occupancy wants to be a lightness ramp
from solid black to empty white, which leaves nothing for the things drawn on top:
in grey, the track and the rover were both dark pixels over dark obstacles, and the
two hardest things to find in the picture were where the rover is and where it has
been. Now hue carries the overlay and lightness carries the occupancy — a red arrow
for the rover, tip forward, with a yellow dot at the exact pose, and a blue line for
the path. The arrow replaced a dot with a whisker off it, which at three pixels per
cell was two pixels wide and left the heading to be guessed. Nothing on the rover can
draw text, so the caption names the colours for the model and
[drive_web/drive_web.py](../drive_web/drive_web.py) builds its key out of this
file's palette rather than its own. Empty floor the rover can reach from where it
stands is green; empty that is cut off by a wall stays the cream "empty" of the
occupancy ramp, which is why those two sit next to each other on the key.

A client can zoom, and zooming keeps the picture the size it was. `map_png` in the
daemon takes how many metres to show and how big a picture to send back, and works
pixels-per-cell out from the two; `render` still takes it directly, since by then the
question has been settled. That way round matters. Taking a magnification instead
means the picture grows every time the view widens, which is rescaling the window
rather than zooming — asked for a steady 480 px, the console's ladder now comes back
465–492 px from 1.5 m across to 12 m, where fixing the magnification gave 240 px to
1200 px over the same range. Sizes are only reachable to within a few percent because
a cell must be a whole number of pixels, and past 12 m across it is down to two, which
is why the ladder stops there.

A violet wedge shows where the camera is pointing and how much of the room is in
shot. That is the only thing in the picture that did not come off the lidar, and it
is there because the map otherwise says nothing at all about the other sensor: the
two point in different directions most of the time — the gimbal pans a long way
either side and sweeps continuously while face tracking runs — and the rover's own
arrow says nothing about where the camera got to. It reaches across the crop rather
than a fixed number of metres, so it reads the same at every zoom: it is a direction
and a width, not a range.

It is washed over the map at a quarter strength and then outlined at full. The fill
is what makes it read as one lit area rather than as three unrelated violet lines,
and a quarter is as far as it can go in the other direction: the interesting part of
the map is precisely the part inside the cone, so a heavier wash would hide what the
cone is there to point at. The outline goes on top so the edges stay exact whatever
the fill lands on. A translucent fill is the one shape here that has to *read* what
is underneath it rather than overwrite it, which would be a per-pixel Python loop —
so the colour and the fraction, both fixed for the whole shape, are folded into a
256-entry table per channel and `bytes.translate` applies it a row at a time in C.
`python mapimg.py` checks that more of the picture changed than the outline accounts
for, because every other check there finds the cone by its exact colour and would
pass an outline with nothing inside it.

**The two angle conventions are opposite, and that sign is the whole risk here.**
The gimbal counts pan positive to the right; the lidar, the map and everything else
count bearings positive to the left, counter-clockwise from straight ahead. So the
daemon hands the renderer minus the pan, in one place — `_camera_cone` — and both
`rover_daemon/selftest.py` and `python mapimg.py` check the *direction* rather than
the value, because a mirrored cone draws perfectly ordinarily over the wrong half of
the room and nothing about the picture gives it away. The caption says which way and
how wide in words as well, since a wedge on its own cannot say whether it is 40
degrees or 90.

The width comes from the daemon's `--camera-fov`, and for this rover's camera it has
been measured at 132 degrees across — by
[`usb_cameras/calibrate_fov.py`](../usb_cameras/calibrate_fov.py), which sweeps the
gimbal and fits the lens to how far the room slides. It stood at 65 degrees for a
long time as a guess at a generic webcam, and the guess was out by more than a factor
of two: the module is a fisheye. A cone that narrow is not a small error to leave in
a picture whose whole job is saying which part of the room is in shot.

## When the lidar drops off the bus

It does, and not rarely. The sensor's serial adapter hangs off a small hub, on
another hub, on the host's own hub — three deep — and the whole branch goes away
under motor load:

```
usb 1-1.3.3: USB disconnect, device number 17
usb 1-1.3-port3: Cannot enable. Maybe the USB cable is bad?
usb 1-1.3-port3: attempt power cycle
usb 1-1.3-port3: unable to enumerate USB device
```

Read the last three lines carefully, because they are the whole reason this section
exists: the kernel notices, tries a port power cycle of its own, fails, and **stops
trying**. The port stays dead until something resets it. Everything above this in
the stack behaves correctly and uselessly — the node looks for a port that is not
there, `lidar_ok` goes false so nothing drives on a stale pose, the console shows a
scan age climbing — and the rover sits blind until somebody walks over and pulls the
plug. The run that prompted this had been blind for sixteen minutes.

`usbreset.py` is the plug, in software. `USBDEVFS_RESET` on a device re-enumerates
that device; on a hub it re-enumerates the hub and everything below it, which is the
only thing that reaches a port too wedged to enumerate at all. It is reached through
the daemon's `reset_lidar` control call — see
[`rover_daemon/ros_navigator.py`](../rover_daemon/ros_navigator.py) — because the
daemon is what a console can talk to and the ROS node is not.

**The ladder is nearest-first, and it escalates only on evidence.** On this rover it
comes out as

```
1-1.3.3.2 (USB Single Serial) -> 1-1.3.3 (USB 2.0 Hub) -> 1-1.3 (USB2.0 Hub)
```

and it stops there rather than continuing to `1-1`, which is the host's built-in hub
and carries the ethernet and the wifi dongle: resetting that would cut the wire the
request to reset arrived over. `_carries_the_network` works that out by walking each
candidate's subtree for a net device, so re-plugging the wifi somewhere else does not
silently make the rover cut itself off.

Escalation matters as much as the ladder. A reset can succeed and change nothing —
the ioctl returns cleanly against a device that is enumerated but dead — so a
recovery that only ever reset the device would spend the afternoon repeating the one
act already shown not to work. The caller therefore counts: nothing came back, so
reach one rung higher. Only when the ladder is exhausted does it start backing off,
doubling from a minute to a quarter of an hour, because at that point it is a cable
and knocking the camera out every minute will not change that.

Measured on the rover, with the adapter taken off the bus by deauthorising it and
nothing touched afterwards:

```
   3s  live=False age=3.06   resets=0
  33s  live=False age=33.2   resets=1   reset 1-1.3.3.2 (USB Single Serial)
  93s  live=False age=93.49  resets=2   reset 1-1.3.3 (USB 2.0 Hub)
  99s  live=True  age=0.02   resets=2
```

Rung one had no effect, which is what a deauthorised device does and what a wedged
one does; rung two brought it back, and the port was found again six seconds later.

**It needs a udev rule, once per rover.** `/dev/bus/usb/BBB/DDD` is `root:root 0664`
and the reset ioctl needs the node open for writing, so without the rule every
attempt comes back "not allowed to reset /dev/bus/usb/001/005" and names the node it
could not open. `install-udev.sh` puts the rule in place and reapplies it to what is
already plugged in:

```bash
cat secrets/jetson-orin.key | ssh orin 'sudo -S -p "" sh ~/ugv/lidar_slam/install-udev.sh'
```

The action there has to be `udevadm trigger --action=add` and not `change`: udev sets
a node's owner and mode when the node is created, and re-running the rules under
`change` matched the rule, reported `GROUP 46, MODE 0660` in `udevadm test`, and left
every node `root:root 0664` — which looks exactly like a rule that did not match.

`nav_status` reports `lidar_resets` and what the last one said, and the console shows
both, because the number is the diagnosis: a rover that has replugged its own lidar
four times in an afternoon has a cable working loose, and nothing else would ever say
so.

## Files

```
slam2d.h      the API, and the reasoning behind each config field
slam2d.c      the LD19 parser, the sector query and the segmentation
selftest.c    correctness and cost against a synthetic room and a synthetic table
build.sh      builds libslam2d.so and selftest, on the machine that runs them
slam2d.py     ctypes binding, and describe(); checks its struct layout each load
nav_types.py  the driver board's protocol, the chassis fallback, Outcome, MoveReport
mapimg.py     a PNG encoder and the map rendering, in colour, stdlib only
usbreset.py   replugs the lidar in software when it drops off the USB bus; self-tests
99-rover-usb-reset.rules  what lets the daemon do that without being root
install-udev.sh           installs that rule; needs root, once, per rover
```

`libslam2d.so` and `selftest` are build products and are not committed.

## What is not done yet

- **`nav_types.py` is in the wrong place and has the wrong name.** It is the driver
  board's protocol constants and a fallback chassis curve, read by `ros_nav/` and by
  the daemon; nothing about it is a nav type and nothing about it belongs under a
  directory called `lidar_slam`. Moving it is a rename across six import sites and a
  deploy path, which is why it has not happened yet rather than because it is right.
- **The map picture has not been seen by the model.** The frame server stashes bytes
  without decoding and the upload declares no media type, so a PNG ought to be as
  acceptable as a JPEG — but that is reasoning, not a test. If it turns out to be
  refused, the caption still answers and the fix is a JPEG encoder or a service-side
  change.
- **The magnetometer is still unused.** `mx/my/mz` off the driver board is the one
  absolute heading reference on this rover, and `slam_toolbox`'s loop closure has
  taken most of the urgency out of it — but a rover that has just been picked up and
  put down somewhere else still has nothing to say about which way it is facing.
- **The lidar sees one horizontal slice** and cannot see a step, a drop, a low sill or
  a table top. Thirty centimetres from a wall is safe; thirty centimetres from a stair
  is not. Nothing in software fixes that, and an unattended rover needs either a
  second sensor or a rule about where it may run.
