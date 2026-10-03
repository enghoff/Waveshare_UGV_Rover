# After reseating the OAK's cable, a ten-minute soak with no drops

**The owner reseated the depth camera's USB cable on 2026-10-03, and a ten-minute
soak on the charger afterwards had no drops at all.** The two soaks of 2026-10-02 had
9 and 2 ([the soaks](2026-10-02-oak-usb-drops.md)). One soak does not show the fault
is gone, and a parked rover does not shake its cable the way a driving one does.
[R-CTL-6](../requirements/control.md#r-ctl-6) is unchanged.

The camera was switched on through the depth service's own switch, on the parked
rover, and its health was read once a second for 601 s. It streamed 8,969 frames,
about 15 a second. It was not ready for 3 seconds, all of them while it woke at the
start, and the service did not exit once. Earlier that day, before the reseat, it had
dropped once, a minute into the acceptance drive at 09:13. Yesterday it dropped 40
times. The script is `/tmp/soak.py` on the rover. It differs from yesterday's in
counting the service's own exits rather than the kernel's disconnects, and it did not
sample the supply rail.

## What would show it is fixed

Drives. The first supervised autonomy sessions stream the camera while the rover moves,
and the depth service's log counts every drop.
