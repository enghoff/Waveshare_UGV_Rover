#!/usr/bin/env python3
"""Bring a USB device back by resetting it, or the nearest hub still above it.

This exists because of a failure the rover actually has. The lidar hangs off a
CH343 serial adapter, on a small hub, on another hub, on the host's own hub -- three
deep -- and the whole branch drops off the bus from time to time:

    usb 1-1.3.3: USB disconnect, device number 17
    usb 1-1.3-port3: Cannot enable. Maybe the USB cable is bad?
    usb 1-1.3-port3: attempt power cycle
    usb 1-1.3-port3: unable to enumerate USB device

The kernel tries a port power cycle by itself, and when that fails it gives up for
good: the port stays dead until something resets it or somebody walks over and pulls
the plug. Meanwhile the rover is blind. It cannot drive, it cannot map, and the only
sign of it is a scan age counting up in the console -- 980 seconds, on the run that
prompted this.

`USBDEVFS_RESET` is what the plug does, in software. Issued on the *device*, it
re-enumerates that device; issued on a hub, it re-enumerates the hub and everything
below it, which is the only thing that reaches a port too wedged to enumerate at
all. Measured on the rover: resetting the hub above a lidar that had been gone for
sixteen minutes brought `ttyACM0` back within four seconds, and the daemon found it
on its next reopen without being told.

Two rules keep that from being worse than the fault.

**Never the hub the network is on.** The reset is reached over ssh or over the
console's own TCP connection, and resetting the hub carrying the wifi dongle would
cut the wire the request arrived on. `_carries_the_network` walks each candidate's
subtree looking for a net device and refuses it -- which on this rover rules out the
Pi's built-in hub, and would rule out whatever else somebody plugs the wifi into
later without this file having to be told.

**Nearest first.** Resetting a hub takes everything below it with it, and on this
rover that hub also carries the camera, the OAK and the Bluetooth dongle. So the
device's own node is tried before its parent, and the parent before *its* parent:
the shallowest reset that can reach the fault, rather than the biggest hammer.

Nothing here needs root, but it does need write access to `/dev/bus/usb/...`, which
is `root:root 0664` by default. `99-rover-usb-reset.rules` beside this file is what
grants it to the `plugdev` group; without it every reset returns "not allowed to
reset" and says which node it could not open.
"""
from __future__ import annotations

import glob
import os

# `fcntl` is imported where it is used rather than here, and that is not tidiness:
# it does not exist on Windows, and this module is reached from `navigator.py`,
# which the desk imports for its move commentary and its selftests. Everything else
# in here is path and string work that runs anywhere -- a glob over `/sys` simply
# finds nothing -- so the one Linux-only line is the one that issues the ioctl.

#: `_IO('U', 20)` out of `linux/usbdevice_fs.h`: no argument, so the number is just
#: the type in the high byte and the ordinal in the low one.
USBDEVFS_RESET = ord("U") << 8 | 20

SYSFS = "/sys/bus/usb/devices"

#: The serial adapters this rover's lidar has been seen behind. The CH343 is what is
#: fitted; the Silicon Labs id is here because the spare cable uses one, and finding
#: the device by id is only a fallback for when nothing remembered where it was.
LIDAR_USB_IDS = (("1a86", "55d3"), ("1a86", "7523"), ("10c4", "ea60"))


class Attempt:
    """What one call to :func:`revive` did, in a form worth logging or reporting.

    `ok` says a reset was issued and the ioctl returned, not that the device came
    back -- nothing here waits around to find out, because the caller is a control
    loop that will notice the port reappear on its own within a couple of seconds.
    """

    def __init__(self, ok: bool, what: str, why: str = "",
                 rung: int = 0, rungs: int = 0) -> None:
        self.ok = ok
        self.what = what      # which device was reset, as its sysfs name
        self.why = why        # in plain words, for the console and the log
        self.rung = rung      # how far up the ladder this went
        self.rungs = rungs    # and how far it could have gone
        #: Whether anything bigger is left to try. A caller that resets, waits, and
        #: finds the sensor still silent needs to know the difference between "try
        #: the next rung up" and "this is as far as software goes, it is a cable".
        self.more = rung + 1 < rungs

    def __repr__(self) -> str:
        return (f"Attempt(ok={self.ok}, what={self.what!r}, rung={self.rung}"
                f"/{self.rungs}, why={self.why!r})")


def _read(path: str) -> str:
    try:
        with open(path) as handle:
            return handle.read().strip()
    except OSError:
        return ""


def usb_path_for(tty: str) -> str:
    """The sysfs name of the USB device behind a tty, or "".

    `/dev/ttyACM0` -> `1-1.3.3.2`. The tty's `device` link lands on the *interface*
    (`1-1.3.3.2:1.0`), so this walks up until it reaches something with an
    `idVendor` -- which is the device itself, and the thing a reset applies to.

    The symlink is followed first, because the name the navigator opens the lidar by
    is deliberately *not* `/dev/ttyACM0` -- it is the by-id name that carries the
    adapter's serial number and survives a replug. There is no
    `/sys/class/tty/usb-1a86_USB_Single_Serial_5B79023845-if00`, so taking the
    basename of what was passed in found nothing and quietly returned "". That
    matters more than it looks: this is the one lookup that has to have happened
    *before* the device disappears, because once it has gone there is nothing left
    on the bus to find it by.
    """
    node = os.path.realpath(f"/sys/class/tty/{os.path.basename(os.path.realpath(tty))}"
                            f"/device")
    for _ in range(8):
        if os.path.exists(os.path.join(node, "idVendor")):
            return os.path.basename(node)
        parent = os.path.dirname(node)
        if parent == node or parent == "/":
            break
        node = parent
    return ""


def find_by_ids(ids=LIDAR_USB_IDS) -> str:
    """The sysfs name of the first device matching one of `ids`, or "".

    For the case where nothing remembered where the lidar was -- a daemon that
    started with the device already missing -- and it came back on its own since.
    """
    for entry in sorted(glob.glob(f"{SYSFS}/*")):
        pair = (_read(f"{entry}/idVendor"), _read(f"{entry}/idProduct"))
        if pair in ids:
            return os.path.basename(entry)
    return ""


def parents(name: str):
    """`1-1.3.3.2` -> `1-1.3.3`, `1-1.3`, `1-1`. Nearest first, root hub excluded.

    The root hub is left out deliberately: it is the controller rather than
    anything anyone plugged in, and resetting it is a reboot of the bus.
    """
    while "." in name:
        name = name.rsplit(".", 1)[0]
        yield name


def present(name: str) -> bool:
    return bool(name) and os.path.isdir(f"{SYSFS}/{name}")


def is_hub(name: str) -> bool:
    return _read(f"{SYSFS}/{name}/bDeviceClass") == "09"


def describe(name: str) -> str:
    """What a device calls itself, for a log line somebody has to read."""
    product = _read(f"{SYSFS}/{name}/product")
    ids = f"{_read(f'{SYSFS}/{name}/idVendor')}:{_read(f'{SYSFS}/{name}/idProduct')}"
    return f"{name} ({product or ids})"


def _carries_the_network(name: str) -> bool:
    """Is a network interface anywhere below this device?

    Asked of every candidate, because resetting a hub takes everything under it
    down -- and if that includes the wifi dongle, the reset cuts the connection the
    request to reset arrived on. The rover is reached over that wire and nothing
    else, so this is a refusal rather than a warning.
    """
    root = f"{SYSFS}/{name}"
    if not os.path.isdir(root):
        return False
    if glob.glob(f"{root}/*/net/*") or glob.glob(f"{root}/net/*"):
        return True
    # Depth is bounded by the bus, not by this walk: names get one `.N` longer per
    # hub, so a subtree is at most a handful of levels and always finite.
    for child in glob.glob(f"{root}/{os.path.basename(root)}*"):
        if os.path.isdir(child) and _carries_the_network(os.path.basename(child)):
            return True
    return False


def node_for(name: str) -> str:
    """The `/dev/bus/usb/BBB/DDD` node a reset is issued on, or "".

    Read from sysfs rather than assembled from a remembered number, because the
    device number changes on every enumeration and a stale one names whatever
    plugged in after it.
    """
    bus, dev = _read(f"{SYSFS}/{name}/busnum"), _read(f"{SYSFS}/{name}/devnum")
    if not (bus and dev):
        return ""
    return f"/dev/bus/usb/{int(bus):03d}/{int(dev):03d}"


def reset(name: str) -> Attempt:
    """Reset one USB device by its sysfs name. The plug, in software."""
    import fcntl

    node = node_for(name)
    if not node:
        return Attempt(False, name, f"{name} is not on the bus")
    try:
        fd = os.open(node, os.O_WRONLY)
    except PermissionError:
        return Attempt(False, name,
                       f"not allowed to reset {node} -- install "
                       f"99-rover-usb-reset.rules")
    except OSError as error:
        return Attempt(False, name, f"cannot open {node}: {error}")
    try:
        fcntl.ioctl(fd, USBDEVFS_RESET, 0)
    except OSError as error:
        return Attempt(False, name, f"reset of {describe(name)} refused: {error}")
    finally:
        os.close(fd)
    return Attempt(True, name, f"reset {describe(name)}")


def ladder(known: str) -> list:
    """What there is to reset, in order of how much it disturbs.

    The device itself first, if the bus can still see it -- that costs nothing but
    the lidar. Then each hub above it that is still enumerated, because a port too
    wedged to enumerate has no device node of its own and the only thing that
    reaches it is a reset of the hub carrying it. And that is where it stops: the
    first hub with the network behind it ends the ladder rather than appearing on
    it, since resetting that cuts the wire the request arrived over.
    """
    rungs = []
    if present(known) and not is_hub(known):
        rungs.append(known)
    for parent in parents(known):
        if not present(parent):
            continue
        if _carries_the_network(parent):
            break
        rungs.append(parent)
    return rungs


def revive(known: str = "", rung: int = 0, ids=LIDAR_USB_IDS) -> Attempt:
    """Reset the lidar, or something above it, and say what else there was.

    `known` is where the device was last seen, as a sysfs name -- the caller keeps
    that from when the port was last open, because once the device has gone there is
    nothing left to look it up by. Without it the device is searched for by id,
    which only helps if it is present but wedged.

    `rung` is how far up to reach, and it exists because a reset can succeed and
    change nothing. The ioctl returns fine against a device that is enumerated but
    dead, and the caller then waits, finds the sensor still silent, and would
    otherwise spend the rest of the afternoon resetting the same device that did not
    respond the first time. So the caller counts: nothing came back, so ask for one
    rung higher, and this reaches for the hub instead. Past the top of the ladder it
    stays there, and `Attempt.more` is False so the caller can say out loud that
    software has run out and it is a cable.
    """
    where = known or find_by_ids(ids)
    if not where:
        return Attempt(False, "", "nothing to reset: the lidar has never been seen "
                                  "on this bus, so there is no port to look above")
    rungs = ladder(where)
    if not rungs:
        return Attempt(False, where,
                       f"nothing at or above {where} is still on the bus to reset, "
                       f"or what is left carries the network")
    at = min(max(0, rung), len(rungs) - 1)
    attempt = reset(rungs[at])
    attempt.rung, attempt.rungs = at, len(rungs)
    attempt.more = at + 1 < len(rungs)
    return attempt


def _selftest() -> int:
    """Checkable without a lidar, and worth checking: the parts that decide *what*
    to reset are pure string and filesystem work, and getting them wrong resets the
    wrong device."""
    failures = []

    def check(claim, got, want):
        if got != want:
            failures.append(f"{claim}: got {got!r}, wanted {want!r}")
        print(f"  {'ok  ' if got == want else 'FAIL'} {claim}")

    check("a device's parents come out nearest first",
          list(parents("1-1.3.3.2")), ["1-1.3.3", "1-1.3", "1-1"])
    check("...and the root hub is not among them", list(parents("1-1")), [])
    check("a device with no parents yields none", list(parents("usb1")), [])

    check("the reset ioctl is the one in usbdevice_fs.h", USBDEVFS_RESET, 0x5514)

    check("a device nobody has ever seen is refused rather than guessed at",
          revive("", ids=(("dead", "beef"),)).ok, False)

    # Against the real bus, if there is one. Nothing is reset -- these only ask
    # what would be, which is the half that can be wrong quietly.
    if os.path.isdir(SYSFS):
        hubs = [os.path.basename(p) for p in sorted(glob.glob(f"{SYSFS}/*"))
                if is_hub(os.path.basename(p))]
        print(f"  note hubs on this host: {', '.join(hubs) or 'none'}")
        for hub in hubs:
            if _carries_the_network(hub):
                print(f"  note {describe(hub)} carries the network and is refused")
        where = find_by_ids()
        if where:
            climb = ladder(where)
            print(f"  note the ladder from {where}: "
                  f"{' -> '.join(describe(r) for r in climb) or 'nothing'}")
            check("the ladder starts at the device itself", climb[:1], [where])
            check("...and every rung above it is a hub",
                  all(is_hub(r) for r in climb[1:]), True)
            check("...and none of them carries the network",
                  any(_carries_the_network(r) for r in climb), False)
            # With `reset` stood down, because a selftest that actually resets
            # the bus is a selftest that unplugs the rover's lidar to prove it
            # can -- which is how the run before this one came back with three
            # failures about a device that was busy re-enumerating.
            global reset
            real, reset = reset, lambda name: Attempt(True, name, "pretended")
            try:
                check("asking past the top of the ladder stays at the top",
                      revive(where, 99).what, climb[-1])
                check("...and asking below the bottom stays at the bottom",
                      revive(where, -5).what, climb[0])
                check("...and each rung in between is the one asked for",
                      [revive(where, i).what for i in range(len(climb))], climb)
            finally:
                reset = real
        found = where
        print(f"  note lidar adapter: {describe(found) if found else 'not on the bus'}")
        if found:
            check("...and its node is nameable", bool(node_for(found)), True)
            # Both names for the same port have to lead back to the same device.
            # The by-id one is what the navigator actually opens, and it is the one
            # that used to lead nowhere -- silently, leaving nothing remembered for
            # the reset that has to happen after the device has gone.
            names = sorted(glob.glob("/dev/serial/by-id/*") + glob.glob("/dev/ttyACM*"))
            led_back = {name: usb_path_for(name) for name in names}
            check(f"every serial name leads back to a device ({len(names)} of them)",
                  sorted(set(led_back.values())) != [""] and "" not in led_back.values(),
                  True)
            check("...including the by-id name the navigator opens",
                  found in led_back.values(), True)

    print(f"\n{'all passed' if not failures else str(len(failures)) + ' failed'}")
    return 1 if failures else 0


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "--selftest":
        raise SystemExit(_selftest())
    # Otherwise: do it, and say what happened. `python3 usbreset.py [sysfs-name]`
    outcome = revive(sys.argv[1] if len(sys.argv) > 1 else "")
    print(outcome.why)
    raise SystemExit(0 if outcome.ok else 1)
