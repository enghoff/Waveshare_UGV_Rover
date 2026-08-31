"""Wi-Fi scan/join helpers and the daemon tools that expose them."""
from __future__ import annotations

import json
import os
import socket
import subprocess
import threading
import time
from typing import Any

from rover_util import _flag, _number

# The rover's own network, for the console to show and to choose from. Three
# access points are configured here, and the rover joins exactly one of them by
# itself -- see wifi_roam/README.md. The other two are reachable only through
# this file, because somebody at the console asked for one.
#
# Scanning and switching are privileged -- polkit grants NetworkManager control to
# an active local session, which a daemon is not -- so they go through a helper
# with a passwordless sudo rule for that one path. See wifi_roam/README.md.
WIFI_CTL = "/usr/local/sbin/wifi_ctl.sh"
# The name to fall back on when this host has no wireless interface to find --
# which on a rover means the radio is not on the bus at all. It is the name both
# earlier boards used, and hard-coding it is what made a Jetson with two working
# radios report no network whatsoever: neither of them is called `wlan0` here
# (the onboard Realtek is `wlP1p1s0` and the dongle keeps the kernel's
# `wlx002e2d3074d0`), so every read below went to an interface that did not
# exist and the panel was left with nothing to show. The radio is found rather
# than assumed now; see _wifi_iface.
WIFI_IFACE = "wlan0"
# Where the radios and the routing table are read from. Named so the self-test
# can point them at a directory it built, since the interesting case -- a board
# whose radio is not called wlan0 -- cannot be arranged on the machine running
# the test.
SYS_NET = "/sys/class/net"
PROC_ROUTE = "/proc/net/route"
# How long the list of access points is served for. Long, because one nmcli call
# costs 1.8 s of wall time and half a second of CPU on this Pi and a console polls
# this: what actually moves while the rover drives is the signal strength, and
# that is read out of /proc instead, for nothing. The list is only stale in the
# sense that a neighbour's router might have appeared in the last few seconds.
WIFI_MAX_AGE_S = 20.0
# A scan takes seconds and interrupts the link while the radio is off channel;
# a switch takes a DHCP round on top. Neither is ever waited on by a
# caller holding a socket that the switch is about to break.
WIFI_SCAN_TIMEOUT_S = 20.0
WIFI_JOIN_TIMEOUT_S = 60.0
# And how long the list of networks this rover holds a passphrase for is served
# for. Minutes, not seconds, because it changes only when somebody installs a new
# passphrase -- while asking costs another 2.6 s of nmcli, which was being spent on
# top of every scan and took one scan to 15.2 s, past the patience of the console
# that asked for it.
WIFI_PROFILE_MAX_AGE_S = 300.0

def _wireless_ifaces() -> list[str]:
    """Every radio on this host, by the name the kernel gave it.

    A wireless interface is the one with a `wireless/` directory beside it in
    sysfs, which is the same question `iw` and NetworkManager ask, and it costs
    a directory listing rather than a process.
    """
    try:
        names = sorted(os.listdir(SYS_NET))
    except OSError:
        return []
    return [name for name in names
            if os.path.isdir(os.path.join(SYS_NET, name, "wireless"))]


def _default_route_ifaces() -> list[str]:
    """The interfaces holding a default route, the preferred one first.

    Preferred meaning lowest metric, which is how the kernel itself chooses
    between them -- and this rover genuinely has two, one per radio.
    """
    routes: list[tuple[int, str]] = []
    try:
        with open(PROC_ROUTE, encoding="ascii") as handle:
            for line in handle:
                fields = line.split()
                # Destination 00000000 is the default route; the header row has
                # the word "Destination" there and falls out here.
                if len(fields) > 6 and fields[1] == "00000000":
                    try:
                        routes.append((int(fields[6]), fields[0]))
                    except ValueError:
                        continue
    except OSError:
        return []
    return [iface for _, iface in sorted(routes)]


def _wifi_iface() -> str:
    """Which radio the answers below are about.

    The one carrying the rover's traffic, if the routing table says which that
    is; otherwise one that at least has an address; otherwise the first radio
    there is. This rover has a second radio on the bus -- the USB dongle, which
    NetworkManager is told to leave alone -- so all three of those questions have
    to be asked rather than assuming the only interface found is the right one.
    """
    radios = _wireless_ifaces()
    if not radios:
        return WIFI_IFACE
    for iface in _default_route_ifaces():
        if iface in radios:
            return iface
    for iface in radios:
        if _iface_address(iface) is not None:
            return iface
    return radios[0]


def _terse_fields(line: str) -> list[str]:
    """Split one row of `nmcli -t` output, honouring its backslash escapes.

    `nmcli -t` separates fields with colons and escapes any colon or backslash
    inside a field. A plain `split(":")` therefore works until somebody's access
    point has a colon in its name, at which point it silently reports the wrong
    signal for the wrong network -- and an SSID is a string a stranger chose.
    """
    fields: list[str] = []
    current: list[str] = []
    escaped = False
    for char in line:
        if escaped:
            current.append(char)
            escaped = False
        elif char == "\\":
            escaped = True
        elif char == ":":
            fields.append("".join(current))
            current = []
        else:
            current.append(char)
    fields.append("".join(current))
    return fields


# How many times to read /proc before believing it has nothing, and how long to
# wait between tries. The M4 Zero's driver leaves the dBm column at -256 -- "not
# filled in" -- for part of the time: measured at 20 ms over 8 s, 19 of 395
# samples were unfilled, in unbroken runs of one to three samples, so the state
# lasts 20 to 60 ms. The gap matters more than the count. Back-to-back reads
# return the identical figure because the driver refreshes it on a timer, so four
# reads in a row are one read repeated; four spaced 30 ms apart span 90 ms and
# clear the longest stretch seen. Nothing waits unless the column is unfilled.
PROC_LEVEL_TRIES = 4
PROC_LEVEL_GAP_S = 0.03


def _proc_level_dbm(iface: str | None = None) -> int | None:
    """One read of the driver's dBm column, or None if it was not filled in."""
    iface = iface or _wifi_iface()
    try:
        with open("/proc/net/wireless", encoding="ascii") as handle:
            for line in handle:
                name, _, rest = line.partition(":")
                if name.strip() != iface:
                    continue
                # status, link quality, signal level, noise. Some drivers leave
                # the dBm column at -256, which is "not filled in", not a reading.
                level = int(float(rest.split()[2]))
                if -120 < level < 0:
                    return level
    except (OSError, ValueError, IndexError):
        pass
    return None


def _wifi_level_dbm(iface: str | None = None) -> int | None:
    """The driver's own idea of the link, in dBm, for the cost of one file read.

    Worth preferring over anything nmcli reports: measured on this rover, a scan's
    0-100 figure wandered from 74 to 88 for the same association while this held
    steady within a couple of dB. It is also the only number here that is free, and
    the only one that moves while the rover drives.
    """
    iface = iface or _wifi_iface()
    for attempt in range(PROC_LEVEL_TRIES):
        level = _proc_level_dbm(iface)
        if level is not None:
            return level
        if attempt + 1 < PROC_LEVEL_TRIES:
            time.sleep(PROC_LEVEL_GAP_S)
    return _iw_signal_dbm(iface)


def _iw_argv(iface: str) -> list[list[str]]:
    return [
        ["iwgetid", "-r", iface],
        ["iw", "dev", iface, "link"],
        ["/sbin/iw", "dev", iface, "link"],
        ["/usr/sbin/iw", "dev", iface, "link"],
    ]


def _iw_signal_dbm(iface: str | None = None) -> int | None:
    """`iw` reports dBm even when /proc/net/wireless leaves the column empty."""
    for argv in _iw_argv(iface or _wifi_iface()):
        if argv[0].endswith("iwgetid"):
            continue
        try:
            done = subprocess.run(argv, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            continue
        for line in done.stdout.decode("utf-8", "replace").splitlines():
            stripped = line.strip()
            if not stripped.startswith("signal:"):
                continue
            try:
                level = int(float(stripped.split()[1]))
            except (IndexError, ValueError):
                continue
            if -120 < level < 0:
                return level
    return None


def _wifi_ssid(iface: str | None = None) -> str | None:
    """The associated SSID, without NetworkManager.

    The privileged helper is how this rover lists neighbours and switches, but
    "which network am I on" is a kernel fact and has to keep working on a host
    that never got `wifi_roam/install.sh`.
    """
    for argv in _iw_argv(iface or _wifi_iface()):
        try:
            done = subprocess.run(argv, stdout=subprocess.PIPE,
                                  stderr=subprocess.DEVNULL, timeout=2)
        except (OSError, subprocess.TimeoutExpired):
            continue
        text = done.stdout.decode("utf-8", "replace")
        if argv[0] == "iwgetid":
            ssid = text.strip()
            if ssid:
                return ssid
            continue
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("SSID:"):
                ssid = stripped[5:].strip()
                if ssid:
                    return ssid
    return None


def _quality(dbm: float | None) -> int:
    """A signal in dBm on the 0-100 scale every list in this file is ranked by.

    NetworkManager's scale, so that a row built here sorts and reads the same as
    one that came from a scan: the associated access point at -46 becomes 100
    and a neighbour at -68 becomes 64.
    """
    return max(0, min(100, int(2 * ((-100 if dbm is None else dbm) + 100) + 0.5)))


def _wifi_from_kernel(iface: str | None = None) -> dict[str, Any] | None:
    """What the radio is doing right now, when the helper cannot list neighbours.

    The one network in the list is the one the rover is on, and it is not offered
    as somewhere to join: without the helper there is nothing here that could
    perform a join, and a button that cannot work is worse than no button.
    """
    iface = iface or _wifi_iface()
    ssid = _wifi_ssid(iface)
    level = _wifi_level_dbm(iface)
    address = _iface_address(iface)
    if ssid is None and level is None and address is None:
        return None
    networks = []
    if ssid:
        networks.append({"ssid": ssid, "signal": _quality(level),
                         "security": "", "in_use": True, "configured": True,
                         "dbm": level})
    return {
        "interface": iface,
        "connected": ssid,
        "level_dbm": level,
        "address": address,
        "networks": networks,
        "configured": [ssid] if ssid else [],
        "scanned": False,
        "list_age_s": 0.0,
    }


def _iface_address(iface: str | None = None) -> str | None:
    """The interface's IPv4 address, or None while it has none.

    Asked of the kernel directly rather than by running `ip`, because "no address"
    is one of the states this has to be able to report and spawning a process to
    learn it costs more than the answer. Absent an address the interface is
    associated but DHCP has not answered, which looks like being online and is not.
    """
    iface = iface or _wifi_iface()
    try:
        import fcntl
        import struct

        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            packed = fcntl.ioctl(sock.fileno(), 0x8915,      # SIOCGIFADDR
                                 struct.pack("256s", iface.encode()[:15]))
        return socket.inet_ntoa(packed[20:24])
    except (ImportError, OSError):
        return None


def _wifi_ctl(action: str, *args: str, timeout: float) -> tuple[bool, str]:
    """Run the wifi helper. Returns whether it succeeded and what it said.

    `sudo` for the two actions that need root and not for the one that does not,
    so that a rover whose sudoers rule was never installed can still say which
    network it is on -- the panel that matters most is the one that only reads.
    """
    argv = [WIFI_CTL, action, *args]
    if action in ("scan", "join"):
        argv = ["sudo", "-n", *argv]
    try:
        done = subprocess.run(argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              timeout=timeout)
    except FileNotFoundError:
        return False, (f"{WIFI_CTL} is not installed on this rover; "
                       f"run wifi_roam/install.sh")
    except subprocess.TimeoutExpired:
        return False, f"the wifi helper did not finish {action} in {timeout:.0f} s"
    except OSError as error:
        return False, f"could not run the wifi helper: {error}"
    if done.returncode != 0:
        said = done.stderr.decode("utf-8", "replace").strip()
        return False, said or f"the wifi helper failed {action} ({done.returncode})"
    return True, done.stdout.decode("utf-8", "replace")


def _wifi_networks(rows: str, configured: set[str]) -> list[dict[str, Any]]:
    """One entry per network from `IN-USE:SSID:SIGNAL:SECURITY` rows.

    One per *network*, not one per radio: a router with two of them beacons the
    same SSID from several BSSIDs, and a list that showed each of those would offer
    a person three identical choices with different numbers beside them. The
    strongest sighting is the one that would be associated with, so it is the one
    shown.

    Hidden networks are dropped. They come back with an empty SSID, which is
    nothing a person can choose and nothing this rover has a passphrase for.
    """
    best: dict[str, dict[str, Any]] = {}
    for line in rows.splitlines():
        if not line.strip():
            continue
        fields = _terse_fields(line)
        if len(fields) < 3 or not fields[1]:
            continue
        ssid = fields[1]
        try:
            signal = int(fields[2])
        except ValueError:
            continue
        entry = best.get(ssid)
        if entry is None:
            best[ssid] = {"ssid": ssid, "signal": signal,
                          "security": fields[3] if len(fields) > 3 else "",
                          "in_use": fields[0] == "*",
                          "configured": ssid in configured}
        else:
            entry["signal"] = max(entry["signal"], signal)
            entry["in_use"] = entry["in_use"] or fields[0] == "*"
    # Configured networks first and the loudest at the top of each group, because
    # the ones with a passphrase on this rover are the only ones it can join and a
    # chooser should not bury them under the neighbours.
    return sorted(best.values(),
                  key=lambda n: (not n["configured"], -n["signal"], n["ssid"]))


class RoverWifi:
    """Scan and join, mixed into Rover. Not offered to the model."""

    # --- the network --------------------------------------------------------
    #
    # Two calls, and neither is offered to the model. They are dispatched like
    # tools because that is the only protocol this daemon speaks, and kept out of
    # :meth:`tools` for the reason `set_vision` is: a model that decided to move
    # the rover onto another access point would be cutting the wire its own
    # conversation is arriving on, and no phrasing of a description makes that a
    # good idea. A person at a console, who can see which network they are on and
    # will notice the reconnect, is a different matter.

    def _wifi_configured(self, now: float) -> set[str]:
        """The networks this rover holds a passphrase for, remembered for minutes.

        Kept far longer than the list of access points because it answers a
        different kind of question: the neighbours come and go, while this changes
        only when somebody runs `wifi_roam/install.sh`. It is worth remembering
        because it is not free -- 2.6 s of nmcli here -- and it was being paid on
        top of every scan, which is most of what made a scan too slow to wait for.

        A helper that fails leaves the last good answer standing rather than
        replacing it with an empty set, which would label every network in the
        panel as one there is no passphrase for.
        """
        if (self._wifi_profiles is None
                or now - self._wifi_profiles_at > WIFI_PROFILE_MAX_AGE_S):
            ok, said = _wifi_ctl("profiles", timeout=WIFI_SCAN_TIMEOUT_S)
            if not ok:
                return self._wifi_profiles or set()
            self._wifi_profiles = {line.strip() for line in said.splitlines()
                                   if line.strip()}
            self._wifi_profiles_at = now
        return self._wifi_profiles

    def _tool_wifi_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Which access point the rover is on, and what else it can hear.

        The strength of the current link is read fresh every time, out of
        `/proc/net/wireless`, because that is the number that changes as the rover
        drives and it costs a file read. The list of access points comes from nmcli,
        which costs 1.8 s here, so it is cached for WIFI_MAX_AGE_S; which networks
        have a passphrase is another nmcli call again, and is cached for far longer
        than that, since it only changes when one is installed.

        `scan` asks the radio to look again rather than reporting what
        NetworkManager already knew. It is not the default and nothing polls it: a
        scan goes off channel for several seconds, which interrupts the link the
        caller is very likely asking through. Without it the list is whatever was last heard, which on a healthy rover is
        mostly just the access point it is on -- so an empty-looking list is the
        answer to "nobody has scanned recently", not to "there is nothing there".
        """
        scan = bool(arguments.get("scan"))
        now = time.monotonic()

        with self._wifi_lock:
            fresh = (self._wifi is not None
                     and now - self._wifi_at < WIFI_MAX_AGE_S)
            if scan or not fresh:
                ok, said = _wifi_ctl("scan" if scan else "list",
                                     timeout=WIFI_SCAN_TIMEOUT_S)
                if not ok:
                    # No list is not the same as no answer: the signal and the
                    # address are still worth having, and on a rover whose sudoers
                    # rule is missing -- or whose OS has no NetworkManager --
                    # they are all that is available.
                    if self._wifi is None:
                        live = _wifi_from_kernel()
                        if live is None:
                            return {"ok": False, "error": said}
                        live["ok"] = True
                        live["note"] = said
                        return live
                else:
                    self._wifi = _wifi_networks(said,
                                                self._wifi_configured(now))
                    self._wifi_at = now
            networks = list(self._wifi or [])
            age = now - self._wifi_at

        connected = next((n["ssid"] for n in networks if n["in_use"]), None)
        # Found once and passed down, so that the signal and the address are
        # certainly about the same radio even if the routing table changes
        # between two reads.
        iface = _wifi_iface()
        reading = {
            "ok": True,
            "interface": iface,
            "connected": connected,
            "level_dbm": _wifi_level_dbm(iface),
            "address": _iface_address(iface),
            "networks": networks,
            "configured": [n["ssid"] for n in networks if n["configured"]],
            "scanned": scan,
            "list_age_s": round(age, 1),
        }
        if self._wifi_join is not None:
            reading["last_join"] = dict(self._wifi_join)
        return reading

    def _tool_wifi_join(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Move the rover onto one of its configured networks.

        **This answers before it has done it**, which is not sloppiness. Bringing
        another access point up takes the current one down, so the reply to a
        caller waiting on a socket would be written into a connection that no
        longer exists -- the caller would see a timeout whether the switch worked or
        not. So the switch runs on its own thread, the answer says which network is
        being joined, and what actually happened is reported by the next
        `wifi_status` as `last_join` once the caller has reconnected.

        Refusing an unconfigured network here as well as in the helper is not
        belt-and-braces: this one can explain itself, and a console showing a list
        of the neighbourhood needs to know that most of what it is showing is not
        somewhere this rover can go.
        """
        ssid = arguments.get("ssid")
        if not isinstance(ssid, str) or not ssid.strip():
            return {"ok": False, "error": "wifi_join wants an ssid"}
        ssid = ssid.strip()
        configured = self._wifi_configured(time.monotonic())
        if configured and ssid not in configured:
            known = ", ".join(sorted(configured)) or "none"
            return {"ok": False,
                    "error": f"there is no passphrase for {ssid} on this rover, so "
                             f"it cannot join it. Configured networks: {known}"}

        def switch() -> None:
            began = time.time()
            ok, said = _wifi_ctl("join", ssid, timeout=WIFI_JOIN_TIMEOUT_S)
            with self._wifi_lock:
                # The list held the old access point as the one in use, and that is
                # now the one thing about it that is certainly wrong.
                self._wifi = None
                self._wifi_join = {"ssid": ssid, "ok": ok,
                                   "at": round(began, 1),
                                   "seconds": round(time.time() - began, 1),
                                   "said": said.strip()[:200]}

        threading.Thread(target=switch, daemon=True, name="wifi-join").start()
        return {"ok": True, "joining": ssid,
                "note": (f"joining {ssid}. Every connection to this rover is about "
                         f"to drop, including this one; reconnect in a few seconds "
                         f"and wifi_status will say how it went.")}
