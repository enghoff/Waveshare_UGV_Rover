# The virtual USB hub, and why a sound card stopped the lidar

On 2026-08-13 the lidar streamed for about a minute at a time and then went
silent, over and over. `/scan` stayed advertised, `ros2 topic list` looked
normal, and nothing in the stack reported an error worth reading — rf2o printed
`Waiting for laser_scans....` and the lidar node printed `get ldlidar data is
time out` at 10 Hz, forever. The lidar itself was fine. So was the driver.

## What was actually happening

The guest gets its sensors through VMware's virtual USB 1.1 controller
(`uhci_hcd`, bus 2), and both the lidar and a JMTek USB audio dongle sat on the
same virtual hub:

    2-2.1   1a86:55d3   CH343 serial -> the D500 lidar
    2-2.2   0c76:1229   JMTek USB PnP Audio Device

The dongle faults on that hub. Every 60–80 seconds:

    usb 2-2.2: reset full-speed USB device number 18 using uhci_hcd
    usb 2-2.2: device descriptor read/64, error -71
    usb 2-2.1: USB disconnect, device number 26        <- the lidar, collateral
    usb 2-2.1: new full-speed USB device number 27

19 lidar disconnects in the first hour. The OAK is untouched by this — it is on
the EHCI controller (bus 1, 480M), not on this hub.

The reason it looked like a driver problem is that `ldlidar_stl_ros2_node`
cannot notice. It opens the port once at startup and never reopens it; on a
vanished device `demo.cpp` gets `DATA_TIME_OUT`, logs, and loops. The node keeps
its handle on a deleted inode — visible as `/proc/<pid>/fd/26 ->
/dev/ttyACM0 (deleted)` — and spins at ~78% of a core producing nothing.

## Two things that do not fix it

**`authorized=0` on a bound device.** It leaves the existing `snd-usb-audio`
bindings in place and the driver re-probes after every reset. Measured: 7
disconnects in the following 4 minutes, worse than the ~3 before.

**`authorized=0` from udev, at add time, before any interface exists.** Cleaner
— no `2-2.2:*` interfaces are ever created, nothing binds — and it still does
not help: 39 kernel events on the port and 3 more lidar disconnects in the next
4.5 minutes. USB core resets the device on its own account. Whether the guest is
talking to it is irrelevant.

The device has to be *gone*, not idle.

## The guest's half: `vm/setup/disable_usb_audio.sh`

A udev rule that unbinds every interface and writes the device's `remove` on
each add, through `systemd-run --no-block` so the udev worker is not left
waiting on the event it is itself processing.

This works, and it holds across reboots — but it cannot be the whole fix,
because the host keeps giving the device back. With `usb.generic.autoconnect`
set, VMware re-attaches the dongle within a few minutes of every removal, and
**the attach itself resets the hub**. The lidar drops 4 seconds *before* the
dongle appears, so udev cannot get there first:

    19:12:09  usb 2-2.1: USB disconnect, device number 54
    19:12:13  usb 2-2.2: New USB device found, idVendor=0c76, idProduct=1229
    19:12:13  usb-drop-device[9848]: detached 2-2.2

The rule turns a fault every 60–80 seconds into one drop per VMware re-attach
(3 in 8 minutes, and two of those were VMware chasing a manual removal). It does
not get to zero.

## The host's half: `ugv-rover.vmx`

    usb.generic.autoconnect = "FALSE"

The two sensors do not depend on this setting. They are remembered devices and
reconnect through their own entries:

    usb.autoConnect.device0 = "vid:03e7 pid:2485 autoclean:0"   # OAK bootloader
    usb.autoConnect.device1 = "vid:03e7 pid:f63b autoclean:0"   # OAK
    usb.autoConnect.device2 = "vid:1a86 pid:55d3 autoclean:0"   # lidar

`usb.generic.autoconnect` governs only *newly plugged* devices — which is how a
sound card that nothing here asked for ended up on the rover's hub in the first
place.

**Edit it with the VM powered off.** VMware holds the configuration in memory
and rewrites the `.vmx` on power-off, so an edit made while the guest is running
is silently discarded.

Unplugging the dongle from the host works too, and works immediately.

## The stack's half: `vm/bin/lidar_watchdog.sh`

Neither of the above helps a lidar that has *already* lost its port, and the
same wedge appears without any dongle involved — a USB glitch, or the node
winning a race against udev at startup and opening the previous `ttyACM`.

So the lidar node now runs with `respawn=True`, and a watchdog compares the tty
the node holds against what `/dev/rover-lidar` resolves to. A mismatch — which a
deleted handle always is — means the node can never recover on its own, so it is
killed and launch starts a fresh one. Verified against forced re-enumerations:

    19:05:39 lidar holds '/dev/ttyACM0 (deleted)' but /dev/rover-lidar is '/dev/ttyACM1' -- restarting node

Recovery takes 10–25 seconds: two 5-second confirmations, a 2-second respawn
delay, and the lidar's own spin-up. That is a repair, not a substitute for the
device staying put.

## What is left after all three

The dongle is gone and stays gone — the first boot with
`usb.generic.autoconnect = "FALSE"` never saw `0c76` at all. The 60–80 second
fault is over.

The lidar still drops occasionally on its own. One burst on that boot, four
disconnects between 19:29:55 and 19:30:34, then nothing for the rest of the
session. These look nothing like the dongle's: no `2-2.2` in sight, no reset, no
`-71`, no warning of any kind before the disconnect line. Something drops the
CH343 — the board's own hub, the cable, or the host's passthrough.

Worth knowing about that burst: only the first disconnect was spontaneous. The
other three interleave with the watchdog's restarts, closely enough that
reopening the port looks like it can provoke another re-enumeration of its own.
The loop does converge — 40 seconds, two restarts, stable afterwards — but a
heal is not always a single clean event, and a burst of them in the log is not
necessarily a burst of independent faults.

Unrelated but easy to mistake for a cause: rviz2 trips a `vmwgfx` WARNING
("Command buffer error", `vmw_cmdbuf_ctx_process`) shortly after every launch.
It is the virtual GPU, not USB, and the stack runs fine through it.
