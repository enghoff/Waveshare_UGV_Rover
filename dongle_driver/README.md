# The USB radio's driver

The rover's second radio is a Realtek RTL8188FTV USB dongle (`0bda:f179`). This
directory builds the kernel driver it needs, with one patch, and registers it
with DKMS so a JetPack kernel update rebuilds it rather than silently dropping
it.

```bash
python deploy/deploy.py --only dongle_driver --system
```

Run it again after a kernel update or a change to the patch. It is idempotent.

## Why the driver is built here at all

NVIDIA's L4T kernel is compiled with `CONFIG_RTL8XXXU` unset and ships no
`drivers/net/wireless/realtek` directory at all, so the dongle sits on the USB
bus with no driver bound and no interface. The driver it needs is in-tree and
has supported this exact device for several releases; it simply was not
compiled. So `install.sh` fetches the driver sources for whatever kernel version
the rover is running — `6.8.12-1021-tegra` is kernel.org's 6.8.12 with NVIDIA's
patches on top, and this directory of the tree is identical — and builds them
out of tree. The kernel does not enforce module signatures, which is why an
unsigned out-of-tree module loads at all.

The sources are fetched rather than vendored here: they are somebody else's GPL
driver, they have to match whatever kernel the next JetPack update leaves
behind, and the only part that is ours is one patch.

There is also a firmware trap the script handles. This kernel is built without
`CONFIG_FW_LOADER_COMPRESS` while Ubuntu 24.04 ships every blob in
`linux-firmware` as `.zst`, so the driver finds the device, identifies it
correctly, and then fails with `Direct firmware load for
rtlwifi/rtl8188fufw.bin failed with error -2` beside a `.zst` that plainly
exists. Read "no such file" as "the file is there, compressed". Expect the same
from any other firmware-loading device added to this board.

## The patch

`rx-urb-recovery.patch`, one change to the receive path, and not specific to
this chip — the same code is in v6.19 today.

The driver keeps a pool of thirty-two receive buffers. When one completes with a
USB error the driver frees it and never makes another. So a device that produces
the occasional transient error — which this one does, sharing a bus with the
camera and the lidar — retires its buffers one at a time until it has none, and
then receives nothing at all. The failure is silent and total: the interface
stays up, the device stays on the bus, `iw scan` succeeds and returns zero
access points, and only reloading the module brings it back.

The patch puts a buffer back to work when the error was recoverable, dropping
only the junk that arrived in it. Teardown errors — `ENOENT`, `ECONNRESET`,
`ESHUTDOWN`, `ENODEV` — still free the buffer, because the device is gone or the
driver is stopping and something is waiting for the pool to drain.

### Seeing what the receive path is doing

This kernel has neither dynamic debug nor kprobes, so the driver had to be made
to report on itself. Three module parameters come with the patch:

```bash
cat /sys/module/rtl8xxxu/parameters/rx_urb_errors    # buffers that erred
cat /sys/module/rtl8xxxu/parameters/rx_urb_retired   # buffers given up on
cat /sys/module/rtl8xxxu/parameters/rx_urb_recover   # the new behaviour, on/off
```

`rx_urb_recover` is writable, so the old behaviour can be restored on a running
rover without rebuilding anything. That is how the difference was measured
rather than argued about, and it is how to check the patch is still earning its
place after a kernel update:

```bash
ssh orin 'sudo -S -p "" sh -c "echo N > /sys/module/rtl8xxxu/parameters/rx_urb_recover"' \
    < secrets/jetson-orin.key
```

`rx_urb_retired` reaching thirty-two is the radio dying. With recovery on it
should stay at zero while `rx_urb_errors` climbs.

## The keeper

`keeper.sh`, run every minute by `dongle-keeper.timer`. If the spare radio has
not been associated to anything for three checks running, it reloads the driver
and lets NetworkManager reconnect.

This is a backstop, not the fix, and it is here because the original fault was
silent and total for six hours on a rover with nothing watching. A radio nobody
watches is not a spare, whatever its driver does.

Two things keep it safe. **It only ever touches the spare**: it finds its
interface by asking which one the `rtl8xxxu` driver has bound, never by name,
and the rover's primary radio is a different driver on a different bus
(`rtl88x2ce`, PCIe), so the worst this can do is disturb a radio that is already
not working — never the link a console or an SSH session is arriving over. And
it is slow to act: three consecutive misses, then a five-minute cooldown, so a
legitimate two-second roam is never mistaken for a fault and a reload is never
piled on top of the last one.

`selftest.sh` drives it through all of that with no radio present — working,
roaming, dead, dead again too soon, and the module not loaded at all. The states
that matter most are the ones where it must do nothing.

```bash
sh dongle_driver/selftest.sh                     # anywhere
ssh orin 'systemctl status dongle-keeper.timer'  # is it armed
ssh orin 'journalctl -u dongle-keeper -n 20'     # what it has been deciding
ssh orin '/usr/local/sbin/dongle-keeper.sh -n'   # what it would do right now
```

Install it switched off with `KEEPER=off`, for a rover being worked on where a
driver reload arriving mid-measurement would make the measurement a lie.
