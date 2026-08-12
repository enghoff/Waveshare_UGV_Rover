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
*LIDAR* (the other Type-C is the ESP32's) — a COM port on Windows, `/dev/ttyUSB0`
on the Pi. 230400-8-N-1, one-way: no command needed, it streams once powered. PWM
is left unconnected, so the motor uses internal speed control at 10 Hz.

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

A top-down view of the point cloud at 30 fps into an 820 px canvas with range
rings. Points are binned at 0.1° and expire after 60 packets — a little over one
revolution, which keeps the picture complete without smearing when the robot turns.
The HUD shows point count, range and measured rotation rate. `s` writes the canvas
to `lidar-<timestamp>.png` in the working directory.

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
