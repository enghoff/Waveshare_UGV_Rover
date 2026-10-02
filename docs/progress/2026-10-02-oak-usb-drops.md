# The OAK drops off USB mid-stream, and neither supply sag nor heat explains it

**The OAK-D-Lite has been falling off USB while it streams, and it does so on a
parked rover on its charger with the Orin's supply flat and the chip at 57-60 °C.**
Each drop takes the depth service down. Until 56b2729 that meant about twenty
seconds without depth each time; now it is a few seconds. The cause is not
found. What is left is power or contact at the camera end of its cable, the USB
port, or the camera itself, and those need hands on the rover.

No requirement changes state. [R-CTL-6](../requirements/control.md#r-ctl-6) held:
the wheels still switch the camera, and these drops are not those switches.

## What the owner saw

The console printed "ConnectionRefusedError: [Errno 111] Connection refused"
under the battery while the gimbal camera's picture kept refreshing. The refusal
was the depth service's port while the process was down after a drop, not the
network and not the camera being switched off to save power. A camera switched
off by the wheels rule leaves the service running and answering `off`.

## The drops

Kernel log, `usb 1-2.1`, which is the OAK on the Orin's onboard USB 2.0 hub,
beside the gimbal camera on `1-2.2`. The gimbal camera never dropped.

- Before today, mid-stream drops were rare. The thousands of failed starts on
  2026-08-31 to 09-04 were the Orin migration ("Cannot find any device"), and
  2026-10-01 had none.
- 2026-10-02, in use: 11:41 and 12:00 parked, then 13:12, 13:13 and 13:14 during
  a drive of short legs and turns with the gimbal at rest. The supervised M0a
  runs that followed lost depth on four check looks at 13:23 and 13:30
  ([that entry](2026-10-02-first-m0a-runs.md) counts 4 unresolved for no depth).
- The device's crash dumps report `errorId 9001` with no assert or trap, and the
  firmware prints in them start from boot. They name no cause.

## Two soaks on the charger

The OAK was kept on by hand on the parked rover for ten minutes each time.
`VDD_IN`, the Orin module's 5 V input from the INA3221, was sampled at 10 Hz,
the depth service's state at 1 Hz, and the kernel's disconnects of the booted
camera (`03e7:f63b`) counted as drops. The bootloader's (`2485`) are the
firmware hand-over on every wake.

| | First, 14:08 | Second, 14:21 |
|---|---|---|
| Drops | 9 | 2 |
| Running before each drop | 6 to 41 s, once 212 s | 217 s and 402 s |
| `VDD_IN` | 5024-5040 mV, 1216-1512 mA, no dip at any drop | 5032-5040 mV, 1208-1496 mA, no dip at any drop |
| Chip temperature | not yet reported | 54.4-60.0 °C; 57.1 and 59.7 at the drops |
| Depth service down | 82 of 588 s | 4 of 587 s |

**Supply sag at the Orin is not needed for a drop.** The Orin's 5 V rail is
flat to 16 mV through every one. The camera draws through its cable and the
hub, so a voltage drop at the camera end would not show here and is not ruled
out.

**Heat is unlikely.** 57-60 °C is far below anything that throttles a Myriad X,
and in the first soak most drops came 6-15 s after a fresh start rather than
after the chip had warmed. The reference figure of about 37 °C is idle, not
streaming.

**The restart change is most of the difference in time lost.** The first soak
ran under 56b2729's 30 s threshold, so drops soon after a start fell back to the
15 s wait. The second had longer runs and every restart took the fast path. Two
soaks are too few to say whether the drop rate itself changed.

## What changed on the rover

- 6d1fd33: the console says "depth service down" in its depth panel instead of
  printing a socket error under the battery. It asks again every 2 s rather than
  every 30, so the panel recovers with the service.
- 56b2729: a camera that had run 30 s or more is reopened as soon as it is back
  on USB instead of after a flat 15 s.
- 5442140: the depth service reports the chip's temperature in `/health` as
  `chip_temp_c`, and on every stop, drop and switch-off line in its log.

## Owed

A hardware check: reseat or swap the OAK's USB cable, try another port on the
Orin, and try powering the camera from a powered hub or a Y-cable. Then soak again
the same way to see whether the drops stop.
