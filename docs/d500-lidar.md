# The D500 lidar

The lidar mounted on the Waveshare UGV Rover, wired to its driver board. Exercised
by [`lidar/`](../lidar). One script, `lidar_view.py`, which validates the
whole chain at once: the rover's 5 V rail, the driver board's USB-UART, and the
sensor itself. It needs pyserial and nothing from depthai.

## Power

The D500 (STL-19P / LD19) has no supply of its own. Its ZH1.5T-4P connector
carries Tx, PWM, GND and P5V — 5 V, 290 mA, 1.45 W. A PH2.0→ZH1.5 cable runs to the
header marked **LIDAR** on the rover's Waveshare *General Driver for Robots* board,
which acts as the adapter board. That 5 V comes off the board's DC-DC rail, fed from the
3S 18650 UPS through the main power switch — **not** from USB.

Measured 2026-08-11: the serial port enumerates over USB with the robot off, and
the lidar only starts streaming when the switch is on. So an empty window with a
live COM port means the switch is off, not that the cable is wrong.

## Data path

The header's Tx goes to an onboard USB-UART and out the Type-C port labelled
*LIDAR* (the other Type-C is the ESP32's) — a COM port on Windows, and
`/dev/ttyACM0` on the Pi, since the CH343 below is claimed by `cdc_acm` rather than
by a `ttyUSB` driver. 230400-8-N-1, one-way: no command needed, it streams once
powered. PWM is left unconnected, so the motor uses internal speed control at
10 Hz.

This board revision enumerates as a **CH343 behind an FE1.1S hub**
(`USB\VID_1A86&PID_55D3`, parent `VID_1A40&PID_0101`), not the two CP2102Ns in
Waveshare's published schematic. Autodetection in `lidar_view.py` accepts both, and
a port can always be passed explicitly.

## Protocol

47-byte packets: header `54 2C`, speed (deg/s), start angle, 12×(uint16 distance
mm, uint8 intensity), end angle, timestamp, CRC-8. The CRC is polynomial `0x4d`,
init 0, no reflection — checking it is what separates real frames from a `54` that
happens to fall in a distance field. Angles are hundredths of a degree in a
left-handed frame: zero is the front of the sensor and angle grows clockwise.

Measured over 1.5 s on 2026-08-11: 625 packets, zero CRC failures, 19.4 kB/s
(417 packets/s × 47 B), rotation 9.96 Hz, 96 % of points non-zero, 23 mm to
8857 mm.

## What `lidar_view.py` draws

A top-down view of the point cloud at 30 fps with range rings. The window opens
820 px square and is resizable; each frame is drawn at the window's current size
rather than scaled up from a fixed canvas, so nothing is ever stretched, and the
margin, dot size and text scale with it.

The window is held square, in two layers.

highgui has no aspect constraint, and a Win32 resize drag runs inside the
system's own modal loop — `waitKey` does not return until the mouse is released,
so nothing on the Python side gets to redraw while the window is being dragged and
Windows simply stretches the last frame, which is what turned the disc into an
ellipse. Squaring up afterwards fixes the picture but not the drag. So on Windows
`keep_window_square` subclasses the window procedure and intercepts `WM_SIZING`,
squaring the rectangle Windows is about to apply: the edge being dragged sets the
side and the opposite edge stays put, so dragging the right edge widens *and*
heightens. The offset between the outer window and the image area is measured
once, so it is the image that comes out square whatever the border and title bar
cost. The original procedure is restored on exit, and anything unexpected inside
the callback falls through to highgui untouched — it runs inside Windows' own
message dispatch, so it cannot be allowed to raise.

`SquareWindow` is the second layer and owns the window: on any change to the rect
that leaves it non-square it resizes to whichever side moved further. That covers
what `WM_SIZING` never sees — maximising, Win+arrow snapping — and is the whole
mechanism on platforms where the subclassing does not apply. It fires only when
the rect actually changed, so a window manager that declines is asked once rather
than every frame.

Underneath both, the frame is always drawn square at the shorter side and padded,
so a window that is momentarily not square shows bars instead of a stretched
picture. `s` saves that square plot, without the padding.

The window's closed state is checked *before* each draw, not after: `imshow` on a
window the user has closed silently creates a replacement — and an `AUTOSIZE` one,
which is not resizable — so checking afterwards would resurrect the view unresizable
and leave the loop with nothing to exit on.

Points are binned at 0.1° and expire after 60 packets — a little over one
revolution, which keeps the picture complete without smearing when the robot turns.
The HUD shows point count, range and measured rotation rate. `s` writes
`lidar-<timestamp>.png` to the working directory.

## View orientation

The lidar sits 90° off the chassis, so its zero is not the rover's forward
direction. `VIEW_ROTATION_DEG` swings the drawing counter-clockwise to cancel that
and is set to 90, which puts the rover's **0° heading straight up** the window —
the vertical green line marks it. The sensor's own zero therefore lands on the left
and its rear on the right.

The green line is drawn at heading 0 rather than at the sensor's zero, so it stays
vertical for any `VIEW_ROTATION_DEG`; change the constant and the cloud turns
underneath it. The rotation is applied to the point geometry, not the finished
canvas, so the HUD and range-ring labels stay upright — and the parsed bearings are
untouched, so a distance read off a point means the same thing at any setting.

## It sees the rover it is bolted to

Two of the lidar's own mount posts are inside its field of view, and it reports
them as an obstacle 13 cm away. Over 397 stationary revolutions:

| | behind the lidar | to the side | range | seen in |
|---|---|---|---|---|
| the post at −135° | 8.5–11.2 cm | 8.2–10.7 cm right | 0.120–0.155 m | 59% of revolutions |
| the post at +135° | 8.5–8.8 cm | 8.4–8.6 cm left | 0.120–0.123 m | 3% of revolutions |

Nothing forward of the sensor was ever this close: all 251 short returns were
behind it. That asymmetry is what identifies them — a real object at 13 cm would
not confine itself to two bearings 90° apart and never appear anywhere else.

They matter more than a stray return should, for three reasons. They are **inside
the rover**, since the chassis is 34 cm wide and these sit 13 cm from its centre,
so no external object can ever be where they are. They **move with the rover**, so
they were stamped into the occupancy grid at each new pose, painting a trail of
phantom obstacles down the middle of the map. And because the navigator takes the
nearest return in any direction as its clearance, and takes the *minimum* over the
last few revolutions to be pessimistic about dropouts, a post seen in 59% of
revolutions was effectively always the nearest thing — which held every turn down
to the slow rate and, before turning was made unrefusable, would refuse it outright.

The minimum-range filter cannot do this job: at 12–16 cm the posts are well beyond
anything it would be safe to blind the sensor to in front, where a return that
close is something the rover is about to hit. So `slam2d.c` masks a box behind the
lidar instead — `body_back_m` deep by `body_half_width_m` either side, 16 cm by
14 cm, fitted to the measurements above with about 5 cm of margin and still inside
the chassis' own 17 cm half-width. It is applied where a return first enters, so
the matcher, the map, the sector query and the feature segmentation all agree about
what is real; filtering further out would have left the map corrupted, and the map
is the part that does not recover. `test_body_mask` in `selftest.c` feeds a ring of
returns at 13 cm and checks that the rear half goes and the front half stays.

Re-measure after moving the sensor or its bracket. The signature is a clearance
that sits at some small constant whatever the room, and a `describe_surroundings`
that disagrees with it.
