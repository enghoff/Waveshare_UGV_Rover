# What a continuous recording would cost: the camera feed is cheap, the clocks are wall time, the OAK dropped once

**Holding the gimbal camera's continuous feed open at 30 frames a second, parked,
cost the Orin about a tenth of one core and left navigation's rates where they
were: 10 scans, 20 odometry updates and 5 localizer steps a second with the feed
and without it.** The 22% of lidar revolutions lost to a held-open feed, which
the camera code still quotes, was the Pi's. Since the OAK's cable was reseated on
2026-10-03 its service has seen one drop, while driving, in about 13½ hours of
streaming. ROS on the rover stamps with the wall clock, each stamp being arrival
time moved by a fixed amount. Measured for Perception's recorder questions
([plan](../plans/recording-for-perception.md)); nothing moved, nothing was
deployed.

## How it was read

2026-10-28, 07:34-07:45 IST, six minutes after a boot, parked, at `a62709d`. The
daemon, navigation (lidar node, localizer, mapper, Nav2, bridge, change watch)
and the console were running; face tracking was off and nobody was talking to
the rover. Each probe ran from `/tmp` and was removed afterwards.

## The camera feed

A separate process opened the feed the way face tracking does
(`uvc_camera.Camera`, 640x480 MJPEG) and took every frame for 25 s, after 25 s
with the camera closed. Navigation's topics were counted every second
throughout, and processor time read from `/proc` at each boundary.

| | Camera closed | Feed open |
|---|---|---|
| Whole board busy (six cores) | 42.6% | 44.7% |
| Daemon | 57% of a core | 66% |
| Lidar node / base node / localizer / mapper | 16 / 11 / 21 / 35% | 16 / 12 / 22 / 35% |
| Scans / odometry / map corrections a second | 10 / 20 / 20 | 10 / 20 / 20 |
| Localizer steps a second | 5 | 5 |

The feed gave 747 frames in 24.9 s (30.0 a second), none dropped, 29.7 kB on
average, each in hand 33 ms after its buffer stamp. The rise of 2.1 points of the
board is about 0.13 of a core, the daemon's own variation included. It does not
cover writing the frames to disk, a feed inside the daemon rather than beside
it, or driving.

## The camera's controls

`v4l2-ctl --list-ctrls-menus` on the gimbal camera: automatic exposure is
"aperture priority" (3) or manual (1); manual exposure runs 50 to 10,000 in units
of 100 µs (5 ms to 1 s) in steps of 10; there is **no gain control**; and
`exposure_dynamic_framerate` is on, so in automatic mode the camera may slow below
30 frames a second in a dim room. 640x480 MJPEG offers one frame interval, 30 a
second, so a lower recording rate means taking every third frame, not setting
the camera.

## The clocks

A subscriber compared each message's stamp with the wall clock on arrival, over
6 s:

| Topic | Rate | Arrival minus stamp |
|---|---|---|
| `/odom`, `odom -> base_link` | 19.8 a second | 50-54 ms |
| `/scan` | 9.9 | 101-104 ms |
| `map -> odom` | 20.0 | -97 to -99 ms |

The stamps are on the wall clock (the monotonic clock differs from it by 1.79e9
s), and each is the moment of arrival moved by the node's fixed allowance: 48 ms
back for odometry, 0.1 s back for a scan, 0.1 s ahead for the localizer's
correction. None is a sensor's own time.

## The OAK since the reseat

The depth service's log (`~/ugv/oak_depth/oak_depth.log`, from 2026-08-31) has,
after the supervisor's start at 09:30 on 2026-10-03:

- **one drop:** 2026-10-13 13:26 IST, "Device crashed, but no crash dump could be
  extracted", then no depth for 5.9 s until the watchdog let the supervisor reopen
  the device, which came back. The battery log's wheel counts show the rover
  driving at the time (about 90 ticks a second);
- one deliberate stop (exit 143, 2026-10-17), and no "communication exception",
  the message every drop before the reseat ended with (16 in the log);
- about 727,000 depth frames between the switch-offs, roughly 13½ hours at 15 a
  second (a floor: the count is taken at each switch-off).

So the reseat did not end the drops; it made them rare. One in 13 hours is one
event, not a rate. The kernel's journal holds nothing for the minute of the drop.

## The board at rest

Memory 3.5 GB available of 7.5, no swap; the root NVMe 788 GB free of 915.
`~/ugv/odometry.json` holds the gyro scale 16.14613 and 107.206 ticks a metre
that base node uses.
