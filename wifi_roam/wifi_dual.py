#!/usr/bin/env python3
"""Two radios, one rover, and a link that is already up before it is needed.

The rover has been wifi-only since it lost its ethernet socket, and the way it
falls off the network is not a radio failing -- it is a *handover*. The three
house routers beacon six different SSIDs between them, so `wpa_supplicant`
cannot roam from one to another at all: leaving TheGreatViking for TheGreatLord
is a full disconnect, a scan of every channel, a fresh association and a DHCP
round. Measured by `netwatch/` over one afternoon: 92 outages averaging 49
seconds, most of them beginning at an excellent signal.

The recording of one of them is the whole argument for this file:

    00:10:51  BEACON-LOSS               sig -78, associated, gateway answering
    00:10:54  ... eight more beacon losses over the next 43 seconds ...
    00:11:34  DISCONNECTED reason=4  -> SCANNING -> no link at all
    00:11:35  re-associated with TheGreatLord, which was sitting at -40 dBm

Forty-three seconds of warning, and a much better access point audible
throughout. Nothing was wrong with the decision the supplicant eventually made;
what was missing was a second radio already associated with the answer, so that
the move could be a change of routing rather than a change of association.

So: the onboard Broadcom radio and the USB dongle stay associated at the same
time, deliberately with two different routers, and this process decides which of
them carries the rover's traffic. A failover is then an address moving between
two interfaces and a gratuitous ARP -- tens of milliseconds, and TCP connections
live through it -- instead of a scan and a DHCP lease.

    wifi_dual.py                # run the manager (root; this is the service)
    wifi_dual.py --once         # one tick, then print the status and exit
    wifi_dual.py --dry-run      # decide everything, change nothing
    wifi_dual.py --restore      # hand both radios back to wpa_supplicant and exit
    wifi_dual.py --status       # print what the running manager last wrote

The design this implements is docs/bpi_dual_wifi_redundancy.md, with four
amendments that the hardware forced or offered:

- **No NetworkManager.** That document pins connections to BSSIDs through
  `nmcli`; this board has netplan, `systemd-networkd` and one `wpa_supplicant`
  per interface, so a radio is pinned by enabling exactly one of the networks in
  its supplicant and disabling the rest.
- **BSSID pinning is not needed.** The six SSIDs are three routers times two
  bands, so "keep the two radios on different access points" is answered by
  choosing different names. What does need care is the opposite mistake:
  TheGreatLord and TheGreatLord 5G are one router, and putting one radio on each
  of them would look like redundancy and provide none. See :func:`router`.
- **The two radios are not interchangeable.** The onboard BCM4345/6 is dual band
  and reaches 31 dBm; the RTL8188FTV dongle is 2.4 GHz, 1T1R, 0 dBm. That maps
  straight onto the document's own advice -- 5 GHz for bandwidth, 2.4 GHz for
  reach -- so the onboard radio gets first pick of routers and a small bonus in
  ties, and the dongle takes the best of what is left.
- **Scanning stops costing anything.** A scan takes the radio off channel for
  seconds, which is why `wifi_roam.sh` only ever scans when the link is already
  in trouble, and why a burst of scans once took this rover off the network for
  an afternoon. With two radios the standby does all the scanning and the active
  one is never interrupted. This is the largest single improvement here, and it
  is free.

**Nothing in this file ever switches a radio off**, for the reason nothing in
`wifi_roam.sh` does: a soft rfkill block is saved and restored across reboots by
systemd, and this board has no ethernet socket to repair it through. The
self-test asserts the absence across every scenario it runs.

Everything the manager touches goes through :class:`Platform`, which exists so
that `wifi_world.py` can hand it a model of a house with three routers in it and
drive the whole thing off a recording. See the "Reproduce it in simulation" rule
in CLAUDE.md; this file was not deployed until that reproduction failed the same
way the rover did.
"""
from __future__ import annotations

import argparse
import json
import os
import select
import socket
import struct
import subprocess
import sys
import time

# --- where the rover lives --------------------------------------------------
GATEWAY = os.environ.get("GATEWAY", "192.168.1.1")

# The address the rest of the world uses for this rover, moved between the two
# interfaces so that TCP connections survive a failover. It is deliberately
# below the DHCP pool -- a sweep of the subnet found .80 silent while the live
# hosts sat at .1, .2, .13, .16, .22, .135, .188, .206 and .232 -- and it is
# ARP-probed before it is ever claimed, every time. Anything answering for it
# means somebody has been given it since, and the manager then runs without a
# service address rather than starting an address war on the house LAN. That is
# not a failure: it degrades exactly to the document's Option 1, two DHCP
# addresses and a route metric, which fails over just as fast and only costs the
# connections that were already open.
SERVICE_IP = os.environ.get("SERVICE_IP", "192.168.1.80")

# --- how often things happen ------------------------------------------------
#
# The document suggests half a second to two seconds. One second is the whole
# loop including both pings: reading a link is two files and one `iw` call, and
# the pings are raw sockets rather than processes.
TICK_S = float(os.environ.get("TICK_S", "1.0"))
# How often the standby radio looks around. Every scan is free in the sense that
# matters -- it never interrupts the link carrying traffic -- but the dongle
# shares a USB bus with the camera, the audio device and the lidar's serial
# adapter, and a burst of scans on that bus is what took this rover off the
# network for an afternoon in August. So it stays a tunable, and raising it is
# how to throttle the dongle without touching this file.
SCAN_EVERY_S = float(os.environ.get("SCAN_EVERY_S", "30"))

# --- how a link is graded ---------------------------------------------------
#
# Everything is scored in dB-equivalents, so that the document's "prefer a new
# access point when it is roughly 8-10 dB better" is expressible directly rather
# than through some invented unit. A link's score starts at its signal strength
# in dBm and has penalties subtracted, which keeps a score readable: -78 is a
# link as good as a clean -78 dBm one, however it got there.
#
# The weights are set so that the document's own worked example comes out the
# way it says it should. It offers AP1 at -56 dBm / 5 ms / 0% against AP2 at
# -49 dBm / 80 ms / 15%, and says AP1 is the better link despite being 7 dB
# quieter. Here AP2 pays (80-10)/5 = 14 dB for latency and 15*0.5 = 7.5 dB for
# loss, scoring -70.5 against AP1's -56: AP1 wins by 14.5 dB, comfortably past
# the switching margin. `test_wifi_dual.py` asserts that arithmetic, so a change
# of weights that quietly reverses the document's example is caught.
LATENCY_FREE_MS = float(os.environ.get("LATENCY_FREE_MS", "10"))
LATENCY_MS_PER_DB = float(os.environ.get("LATENCY_MS_PER_DB", "5"))
LOSS_DB_PER_PCT = float(os.environ.get("LOSS_DB_PER_PCT", "0.5"))
# The onboard radio wins a tie. It is dual band, it is not on the USB bus, and
# it is not the adapter that failed its own PHY, RF and LLT initialisation and
# dropped off the bus in August. Three dB is small enough that a genuinely
# better dongle link still takes the traffic.
PRIMARY_BONUS_DB = float(os.environ.get("PRIMARY_BONUS_DB", "3"))
# And 5 GHz wins a tie when placing a radio, because the band carries several
# times the throughput at the same reported signal. It deliberately plays no
# part in choosing which of two associated links carries traffic: by then
# latency and loss have been measured, and a measurement beats a preference.
BAND_BONUS_DB = float(os.environ.get("BAND_BONUS_DB", "3"))

# --- when to actually move --------------------------------------------------
#
# The document's numbers, and its reasons for them: do not move for 1-3 dB,
# prefer something 8-10 dB better, require the candidate to have been better for
# a second or two, and hold down afterwards so a rover parked in an overlap does
# not oscillate.
#
# The hold-down is shorter than `wifi_roam.sh`'s three minutes, and that is not
# a change of mind. What that script's cooldown protects against is the cost of
# a re-association -- a scan, an authentication and a DHCP round. Moving traffic
# between two radios that are both already associated costs an address and three
# ARP frames, so it can be undone cheaply and need not be feared.
#
# It is a minute rather than the twenty seconds first written here, and the
# recording is what changed it. Replaying 2026-08-24T00:02-00:04 -- the rover
# driving between two routers, so its recorded link swung from -40 to -52 dBm
# inside twenty seconds -- the shorter hold-down produced three failovers in
# fifty-one seconds, the last of them for a nine-decibel difference that had
# reversed by the time it landed. At sixty seconds the same recording produces
# one considered move and then one emergency, and the marginal bounce is gone.
# Neither setting left the rover carrying nothing for a single second, so this
# buys quiet rather than uptime -- but a failover rewrites the ARP cache of
# every device in the house, and doing that for nine decibels is rude.
MARGIN_DB = float(os.environ.get("MARGIN_DB", "8"))
HOLD_S = float(os.environ.get("HOLD_S", "2"))
COOLDOWN_S = float(os.environ.get("COOLDOWN_S", "60"))
# How much of the recent past a link is judged on. Ten pings at one a second, so
# loss resolves to 10% over ten seconds -- fine enough to catch the sustained
# loss the document wants failover on, coarse enough that one dropped packet on
# a busy channel is 5 dB of penalty and not a reason to move on its own.
WINDOW = int(os.environ.get("WINDOW", "10"))
PING_TIMEOUT_S = float(os.environ.get("PING_TIMEOUT_S", "1.0"))
# How many pings in a row have to go unanswered before the link is called dead
# rather than merely bad. This is separate from the window above and it has to
# be: loss over ten seconds is what grades a link that is working badly, and
# waiting for all ten of them to fail before admitting a link has stopped
# working would put ten seconds of retransmits into every hard failure. The
# document asks for exactly this distinction -- fail over rapidly on sustained
# loss, immediately when the access point becomes unreachable.
#
# Three, because one lost packet on a busy 2.4 GHz channel is ordinary and two
# in a row are not rare, while three consecutive seconds of silence from a
# gateway that answers in five milliseconds is not a busy channel.
DEAD_PINGS = int(os.environ.get("DEAD_PINGS", "3"))
# How many scans a placement decision is made from. Three, taken thirty seconds
# apart, so a single wild reading is outvoted and a real change is followed
# within a minute and a half. See Manager.placement_score for the measurement
# this comes from.
SCANS_REMEMBERED = int(os.environ.get("SCANS_REMEMBERED", "3"))

# --- giving up --------------------------------------------------------------
#
# The one thing this process must never do is leave a wifi-only rover with no
# way back. If neither radio has reached the gateway for this long, the manager
# stops trying to be clever: it hands both radios back to their supplicants,
# drops the service address, restores an ordinary default route, and keeps its
# hands off until something answers again. That is strictly worse than what it
# was doing, and strictly better than a rover carried to a socket because a
# manager stayed certain about a plan that was not working.
DEADMAN_S = float(os.environ.get("DEADMAN_S", "120"))

# The dead-man above is about both radios at once. This one is about one radio
# that cannot join the network it is being held on. Holding is `select_network`,
# which disables every other network the radio knows, so a radio held on an
# access point that will not keep it has nothing else it is allowed to try -- it
# retries the same one, loses it, and retries it again, while the console shows
# a spare that is "not associated" beside a list of strong networks. After this
# long with no link at all, the radio gets every network back and may take
# whatever it can hold: being on the air somewhere beats being on the router the
# placement rules would have preferred.
STRANDED_S = float(os.environ.get("STRANDED_S", "90"))
# And it is not sent straight back to the one that would not have it. Ten
# minutes, the same figure as a person's choice, because that is roughly how
# long an access point's bad spell lasts and this rover's radios have several a
# day.
REFUSED_S = float(os.environ.get("REFUSED_S", "600"))

STATUS_PATH = os.environ.get("STATUS_PATH", "/run/wifi-dual.json")

# Where a person at the console asks for a particular network. A file rather
# than a socket because the thing writing it is a shell script the daemon calls
# through one narrow sudo rule, and adding a listening socket to a process that
# holds the rover's network would be a second way in for no benefit.
#
# **The request goes to the spare radio unless it names one**, and that is the
# whole reason this exists rather than the console just calling `wifi_ctl.sh
# join` as it always did. A join has always meant "drop every connection to this
# rover, including the one you are asking through, and hope". With two radios it
# can instead mean: put the standby on the network you asked for, wait until it
# is actually working, and then move the traffic onto it. Nothing drops, and the
# person watching sees the rover change networks under them without the page
# reconnecting.
REQUEST_PATH = os.environ.get("REQUEST_PATH", "/run/wifi-dual.request")
# How long a network somebody chose outranks the one the scoring would pick. Ten
# minutes, which is long enough to be a decision and short enough that a rover
# left alone goes back to looking after itself.
STICKY_S = float(os.environ.get("STICKY_S", "600"))

# Routing tables for the two radios' own addresses, so a packet arriving on the
# standby is answered out of the standby rather than out of whichever interface
# the main table happens to point at. Numeric because this board has no
# /etc/iproute2/rt_tables, and adding one would be another file to keep in step.
TABLE_BASE = 101


def router(ssid: str) -> str:
    """Which physical access point an SSID belongs to.

    TheGreatLord and TheGreatLord 5G are one router with two radios in it.
    Treating them as two access points is the failure this whole file exists to
    avoid: the two adapters would sit on the two bands of the same box, the
    status page would show a healthy active and a healthy standby, and
    unplugging that one box would take both of them at once.
    """
    name = (ssid or "").strip()
    for suffix in (" 5G", "-5G", "_5G", " 5Ghz", " 5GHz", " 5GHZ"):
        if name.endswith(suffix):
            return name[: -len(suffix)].strip()
    return name


def band_of(freq_mhz) -> str:
    try:
        freq = int(freq_mhz)
    except (TypeError, ValueError):
        return "?"
    return "5" if freq >= 4900 else "2.4"


def usable_dbm(dbm):
    """A signal reading, or None when the driver did not fill one in.

    -256 is the sentinel both of this rover's radios park in the level column of
    /proc/net/wireless when they have nothing to report, and reading it as a
    number is how `wifi_roam.sh` once decided a link measuring -42 dBm had
    fallen 200 dB. A reading outside what a radio can physically report is not a
    weak signal; it is no signal reported, and the right answer is to keep the
    previous one rather than to act on it.
    """
    try:
        value = int(dbm)
    except (TypeError, ValueError):
        return None
    return value if -110 < value < 0 else None


def link_score(dbm, rtt_ms, loss_pct):
    """Grade one association in dB-equivalents. None when it is not usable.

    Signal, then a penalty for the round trip and another for what did not come
    back. Everything the caller compares -- the two radios against each other,
    and each against the margin -- is in these units.
    """
    level = usable_dbm(dbm)
    if level is None:
        return None
    score = float(level)
    if rtt_ms is not None and rtt_ms > LATENCY_FREE_MS:
        score -= (rtt_ms - LATENCY_FREE_MS) / LATENCY_MS_PER_DB
    score -= (loss_pct or 0.0) * LOSS_DB_PER_PCT
    return score


class Seen:
    """One access point heard in a scan."""

    __slots__ = ("ssid", "bssid", "freq", "dbm")

    def __init__(self, ssid, bssid=None, freq=None, dbm=None):
        self.ssid, self.bssid, self.freq, self.dbm = ssid, bssid, freq, dbm

    @property
    def router(self):
        return router(self.ssid)

    def as_dict(self):
        return {"ssid": self.ssid, "bssid": self.bssid, "freq": self.freq,
                "dbm": self.dbm, "router": self.router,
                "band": band_of(self.freq)}


class Link:
    """What a radio is associated with right now."""

    __slots__ = ("ssid", "bssid", "freq", "dbm")

    def __init__(self, ssid=None, bssid=None, freq=None, dbm=None):
        self.ssid, self.bssid, self.freq, self.dbm = ssid, bssid, freq, dbm

    @property
    def associated(self):
        return bool(self.ssid)


class Platform:
    """Everything this manager does to the machine it is running on.

    One class, so that the whole of `wifi_world.py` can stand in for it and the
    manager cannot reach round the back of the model to touch a real radio. That
    is not tidiness: the rule in CLAUDE.md is that a fix is a guess until it has
    been reproduced, and a manager that read one real file in the middle of a
    simulated failover would be untestable in exactly the interesting places.

    Nothing here is clever. `iw` and `wpa_cli` are shelled out to because they
    are what is installed, and `ip` because pyroute2 is not on this board and a
    netlink implementation would be a second thing to get wrong. The two
    exceptions are the ping and the ARP, which are raw sockets: `arping` is not
    installed here at all, and a ping per radio per second through a process
    spawn would cost more than the rest of the loop put together.
    """

    def __init__(self, dry_run=False):
        self.dry_run = dry_run
        self._ping_seq = 0

    # --- clocks and noises --------------------------------------------------
    def now(self):
        return time.monotonic()

    def sleep(self, seconds):
        time.sleep(seconds)

    def log(self, message):
        sys.stderr.write(message.rstrip() + "\n")
        sys.stderr.flush()

    # --- looking ------------------------------------------------------------
    def _run(self, argv, timeout=10):
        try:
            done = subprocess.run(argv, stdout=subprocess.PIPE,
                                  stderr=subprocess.PIPE, timeout=timeout)
        except (OSError, subprocess.TimeoutExpired):
            return None
        if done.returncode != 0:
            return None
        return done.stdout.decode("utf-8", "replace")

    def wireless_interfaces(self):
        """Every wifi interface the kernel has, in a stable order.

        `wlan0` before `wlan1` before anything else, so that the onboard radio
        is considered first on a board where both `.link` files applied, and so
        that a dongle whose rename did not take is still managed rather than
        ignored -- it comes back as `wlx002e2d3074d0` and belongs in the list.
        """
        found = []
        try:
            for name in sorted(os.listdir("/sys/class/net")):
                if os.path.isdir("/sys/class/net/%s/wireless" % name):
                    found.append(name)
        except OSError:
            return []
        return sorted(found, key=lambda n: (n != "wlan0", n != "wlan1", n))

    def is_usb(self, iface):
        """Whether this radio hangs off USB, which is what makes it the spare."""
        try:
            path = os.path.realpath("/sys/class/net/%s/device" % iface)
        except OSError:
            return False
        return "/usb" in path or "usb" in os.path.basename(path)

    def mac(self, iface):
        try:
            with open("/sys/class/net/%s/address" % iface) as handle:
                return handle.read().strip()
        except OSError:
            return None

    def operstate(self, iface):
        """`up`, `dormant`, `down`, or `absent` for an interface that is gone.

        `operstate` and not `carrier`, which is the more obvious question and
        took this rover off the network for an evening: reading `carrier` on an
        interface that is not administratively up fails with EINVAL rather than
        answering 0. `operstate` is a word and is readable in every state.
        """
        try:
            with open("/sys/class/net/%s/operstate" % iface) as handle:
                return handle.read().strip()
        except OSError:
            return "absent"

    def link(self, iface):
        """What this radio is associated with, from `iw dev <iface> link`."""
        text = self._run(["iw", "dev", iface, "link"], timeout=4)
        if text is None:
            text = self._run(["/sbin/iw", "dev", iface, "link"], timeout=4)
        if not text or "Not connected" in text:
            return Link()
        link = Link()
        first = text.splitlines()[0] if text.splitlines() else ""
        if "Connected to" in first:
            parts = first.split()
            if len(parts) > 2:
                link.bssid = parts[2]
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("SSID:"):
                link.ssid = stripped[5:].strip()
            elif stripped.startswith("freq:"):
                try:
                    link.freq = int(float(stripped.split()[1]))
                except (IndexError, ValueError):
                    pass
            elif stripped.startswith("signal:"):
                try:
                    link.dbm = int(float(stripped.split()[1]))
                except (IndexError, ValueError):
                    pass
        if link.dbm is None:
            link.dbm = self._proc_level(iface)
        return link

    def _proc_level(self, iface):
        try:
            with open("/proc/net/wireless") as handle:
                for line in handle:
                    name, _, rest = line.partition(":")
                    if name.strip() != iface:
                        continue
                    return int(float(rest.split()[2]))
        except (OSError, ValueError, IndexError):
            pass
        return None

    def ipv4(self, iface):
        """The interface's own DHCP address, asked of the kernel directly.

        Not by running `ip`, because "no address" is one of the states this has
        to report and spawning a process to learn it costs more than the answer.
        A secondary address -- the service address, when this radio is holding
        it -- is deliberately not what comes back here: SIOCGIFADDR returns the
        primary, which is the DHCP lease, and that is the one being asked about.
        """
        try:
            import fcntl
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                packed = fcntl.ioctl(sock.fileno(), 0x8915,      # SIOCGIFADDR
                                     struct.pack("256s", iface.encode()[:15]))
            return socket.inet_ntoa(packed[20:24])
        except (ImportError, OSError):
            return None

    def scan(self, iface):
        """Ask this radio to look around. Only ever called on the standby.

        `iw scan` rather than `wpa_cli scan` plus a poll, because the caller is
        the standby radio and has nothing to lose by blocking: it is carrying no
        traffic, and a scan that takes eight seconds is eight seconds of a spare
        being spare. On the active radio this call is never made at all, which
        is the point of having two.
        """
        text = self._run(["iw", "dev", iface, "scan"], timeout=25)
        if text is None:
            text = self._run(["/sbin/iw", "dev", iface, "scan"], timeout=25)
        if not text:
            return []
        return parse_iw_scan(text)

    # --- moving -------------------------------------------------------------
    def _wpa(self, iface, *args):
        for binary in ("wpa_cli", "/sbin/wpa_cli", "/usr/sbin/wpa_cli"):
            text = self._run([binary, "-i", iface, *args], timeout=8)
            if text is not None:
                return text
        return None

    def networks(self, iface):
        """SSID -> wpa_supplicant network id, for the networks this radio holds.

        The list comes from the supplicant rather than from a constant here, so
        that a network somebody adds to netplan is one this manager can use
        without being edited -- and so that a radio whose supplicant never
        started reports an empty list and is treated as a radio that cannot be
        pinned anywhere, rather than one that should be.
        """
        text = self._wpa(iface, "list_networks")
        if not text:
            return {}
        found = {}
        for line in text.splitlines()[1:]:
            fields = line.split("\t")
            if len(fields) < 2 or not fields[1].strip():
                continue
            try:
                found.setdefault(fields[1].strip(), int(fields[0]))
            except ValueError:
                continue
        return found

    def pin(self, iface, ssid):
        """Make this radio associate with exactly this network and stay there.

        `select_network` is what does it, and it works by disabling every other
        configured network -- which is the one way this file could strand a
        wifi-only rover, so it is worth being plain about. While the manager is
        running that is exactly what is wanted: each radio is held on the router
        the manager chose for it, and a radio free to wander would undo the only
        guarantee this design makes, that the two are never on the same box.

        What makes it safe is that the disabling is never left behind.
        :meth:`release` undoes it, the manager calls that on every exit path
        including a signal, the dead-man calls it when nothing is working at
        all, and `ExecStopPost=` in the unit calls it again in case the process
        died without getting the chance.
        """
        ids = self.networks(iface)
        if ssid not in ids:
            return False
        if self.dry_run:
            return True
        text = self._wpa(iface, "select_network", str(ids[ssid]))
        return bool(text and "OK" in text)

    def release(self, iface):
        """Hand this radio back to its supplicant: every network enabled again.

        Called on every way out of this program. A radio left with one network
        enabled and the manager gone is a rover holding one access point it may
        not be able to reach and forbidden from trying the two it can.
        """
        if self.dry_run:
            return True
        return bool(self._wpa(iface, "enable_network", "all"))

    # --- addresses and routes ----------------------------------------------
    def _ip(self, *args):
        if self.dry_run:
            return True
        return self._run(["ip", *args], timeout=5) is not None

    def addresses(self, iface):
        text = self._run(["ip", "-4", "-o", "addr", "show", "dev", iface],
                         timeout=5) or ""
        found = []
        for line in text.splitlines():
            fields = line.split()
            if "inet" in fields:
                found.append(fields[fields.index("inet") + 1].split("/")[0])
        return found

    def add_service_ip(self, iface, address):
        return self._ip("addr", "add", "%s/32" % address, "dev", iface)

    def del_service_ip(self, iface, address):
        return self._ip("addr", "del", "%s/32" % address, "dev", iface)

    def set_default(self, iface, gateway, src=None, metric=50):
        argv = ["route", "replace", "default", "via", gateway, "dev", iface,
                "metric", str(metric)]
        if src:
            argv += ["src", src]
        return self._ip(*argv)

    def set_interface_table(self, iface, address, table):
        """Answer out of the interface a packet arrived on.

        Two radios on one subnet means the main routing table can only point at
        one of them, so a packet reaching the standby on its own DHCP address
        would be answered out of the active radio -- which works on a bridged
        LAN and stops working the moment an access point does client isolation
        or checks the source MAC. One rule and one table per radio makes the
        answer leave the way the question arrived.
        """
        subnet = address.rsplit(".", 1)[0] + ".0/24"
        self._ip("rule", "del", "from", address, "lookup", str(table))
        ok = self._ip("rule", "add", "from", address, "lookup", str(table))
        ok = self._ip("route", "replace", subnet, "dev", iface, "scope", "link",
                      "src", address, "table", str(table)) and ok
        return self._ip("route", "replace", "default", "via", GATEWAY, "dev",
                        iface, "src", address, "table", str(table)) and ok

    def clear_interface_table(self, address, table):
        self._ip("rule", "del", "from", address, "lookup", str(table))
        self._ip("route", "flush", "table", str(table))

    def set_source_rule(self, address, table):
        """Send one address's replies out of a radio whose lease it is not.

        The service address is the only address here that moves, and it gets no
        table of its own: it borrows the table of whichever radio is holding it,
        because what it needs is exactly what that radio's own address needs --
        this subnet and this gateway, out of this interface. Only the rule moves.

        The `src` on those routes stays the radio's own address, which is
        correct: `src` picks a source for a packet that has not chosen one, and
        a reply from the service address has already chosen.
        """
        self._ip("rule", "del", "from", address, "lookup", str(table))
        return self._ip("rule", "add", "from", address, "lookup", str(table))

    def clear_source_rule(self, address, table):
        self._ip("rule", "del", "from", address, "lookup", str(table))

    def sysctl(self, key, value):
        """arp_ignore and arp_announce, without which two radios on one subnet
        answer for each other and the bridges upstream learn the wrong port."""
        if self.dry_run:
            return True
        try:
            with open("/proc/sys/" + key.replace(".", "/"), "w") as handle:
                handle.write(str(value))
            return True
        except OSError:
            return False

    # --- ICMP, without a process per ping ----------------------------------
    def ping(self, pairs, timeout=PING_TIMEOUT_S):
        """One echo out of each named interface at once. Milliseconds or None.

        `pairs` is (iface, target). Both radios are pinged in the same second
        and waited for together, so the loop costs one timeout rather than two,
        and a standby that has gone silent does not delay the active radio's
        reading by a second every second.

        SO_BINDTODEVICE rather than a source address, because the point is to
        test *that radio's* path to the gateway: a bound source address would
        still be routed by the main table and would answer the question the
        active radio already answered.
        """
        sockets = {}
        sent_at = {}
        results = {iface: None for iface, _ in pairs}
        for iface, target in pairs:
            self._ping_seq = (self._ping_seq + 1) & 0xFFFF
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_RAW,
                                     socket.IPPROTO_ICMP)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BINDTODEVICE,
                                iface.encode())
                sock.setblocking(False)
                packet = _icmp_echo(os.getpid() & 0xFFFF, self._ping_seq)
                sock.sendto(packet, (target, 0))
            except OSError:
                try:
                    sock.close()
                except (OSError, UnboundLocalError, NameError):
                    pass
                continue
            sockets[sock] = iface
            sent_at[iface] = time.monotonic()
        deadline = time.monotonic() + timeout
        try:
            while sockets and time.monotonic() < deadline:
                ready, _, _ = select.select(list(sockets), [], [],
                                            max(0.0, deadline - time.monotonic()))
                if not ready:
                    break
                for sock in ready:
                    iface = sockets.pop(sock)
                    try:
                        sock.recv(1024)
                        results[iface] = (time.monotonic() - sent_at[iface]) * 1000.0
                    except OSError:
                        pass
                    sock.close()
        finally:
            for sock in list(sockets):
                sock.close()
        return results

    # --- ARP, because `arping` is not installed on this board ---------------
    def _arp_socket(self, iface):
        sock = socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                             socket.htons(0x0806))
        sock.bind((iface, socket.htons(0x0806)))
        return sock

    def garp(self, iface, address, times=3):
        """Tell the LAN that this address is over here now.

        The whole of the failover, as far as everything else on the network is
        concerned. Three announcements a tenth of a second apart, because the
        one that matters is whichever the upstream bridge happens to catch, and
        a single broadcast lost to a busy channel would leave every ARP cache in
        the house pointing at the radio that just lost the traffic.
        """
        if self.dry_run:
            return True
        mac = self.mac(iface)
        if not mac:
            return False
        try:
            hardware = bytes(int(part, 16) for part in mac.split(":"))
            packed = socket.inet_aton(address)
        except (ValueError, OSError):
            return False
        frame = (b"\xff" * 6 + hardware + b"\x08\x06"
                 + struct.pack("!HHBBH", 1, 0x0800, 6, 4, 1)
                 + hardware + packed + b"\x00" * 6 + packed)
        try:
            with self._arp_socket(iface) as sock:
                for index in range(times):
                    sock.send(frame)
                    if index + 1 < times:
                        time.sleep(0.1)
            return True
        except OSError:
            return False

    def arp_probe(self, iface, address, timeout=0.6):
        """Ask whether anybody else already holds this address. RFC 5227.

        Sender address 0.0.0.0, so that the question cannot itself be mistaken
        for a claim -- which matters, because the answer this is looking for is
        somebody saying yes, and a probe that announced would poison the very
        caches it is checking.
        """
        mac = self.mac(iface)
        if not mac:
            return None
        try:
            hardware = bytes(int(part, 16) for part in mac.split(":"))
            packed = socket.inet_aton(address)
        except (ValueError, OSError):
            return None
        frame = (b"\xff" * 6 + hardware + b"\x08\x06"
                 + struct.pack("!HHBBH", 1, 0x0800, 6, 4, 1)
                 + hardware + b"\x00\x00\x00\x00" + b"\x00" * 6 + packed)
        try:
            with self._arp_socket(iface) as sock:
                sock.setblocking(False)
                sock.send(frame)
                deadline = time.monotonic() + timeout
                while time.monotonic() < deadline:
                    ready, _, _ = select.select(
                        [sock], [], [], max(0.0, deadline - time.monotonic()))
                    if not ready:
                        break
                    try:
                        reply = sock.recv(2048)
                    except OSError:
                        break
                    if len(reply) < 42:
                        continue
                    opcode = struct.unpack("!H", reply[20:22])[0]
                    sender_ip = reply[28:32]
                    sender_mac = reply[22:28]
                    if opcode == 2 and sender_ip == packed and sender_mac != hardware:
                        return ":".join("%02x" % byte for byte in sender_mac)
        except OSError:
            return None
        return None


def _icmp_checksum(data):
    if len(data) % 2:
        data += b"\x00"
    total = 0
    for index in range(0, len(data), 2):
        total += (data[index] << 8) + data[index + 1]
    total = (total >> 16) + (total & 0xFFFF)
    total += total >> 16
    return ~total & 0xFFFF


def _icmp_echo(identifier, sequence):
    header = struct.pack("!BBHHH", 8, 0, 0, identifier, sequence)
    payload = b"ugv-wifi-dual-0"
    return (struct.pack("!BBHHH", 8, 0, _icmp_checksum(header + payload),
                        identifier, sequence) + payload)


def parse_iw_scan(text):
    """`iw dev X scan` into a list of :class:`Seen`, strongest sighting kept.

    One entry per SSID rather than per BSSID, for the reason the console's list
    already does it: a router with two radios beacons the same name from several
    addresses, and the strongest of them is the one that would be associated
    with, so it is the one worth scoring.
    """
    best = {}
    ssid = bssid = None
    freq = dbm = None

    def keep():
        if not ssid:
            return
        current = best.get(ssid)
        if current is None or (dbm is not None and current.dbm is not None
                               and dbm > current.dbm):
            best[ssid] = Seen(ssid, bssid, freq, dbm)

    for line in text.splitlines():
        stripped = line.strip()
        if stripped.startswith("BSS "):
            keep()
            ssid = None
            freq = dbm = None
            bssid = stripped[4:].split("(")[0].strip()
            continue
        if stripped.startswith("SSID:"):
            ssid = stripped[5:].strip()
        elif stripped.startswith("freq:"):
            try:
                freq = int(float(stripped.split()[1]))
            except (IndexError, ValueError):
                pass
        elif stripped.startswith("signal:"):
            try:
                dbm = int(float(stripped.split()[1]))
            except (IndexError, ValueError):
                pass
    keep()
    return [entry for entry in best.values() if entry.ssid]


class Radio:
    """One adapter, what it is doing, and how well it is doing it."""

    def __init__(self, iface, usb=False, mac=None):
        self.iface = iface
        self.usb = usb
        self.mac = mac
        self.link = Link()
        self.address = None
        self.operstate = "down"
        self.pings = []
        self.seen = []
        # What the last few scans said about each network, oldest first. Kept
        # because a single scan is not evidence on this hardware -- see
        # Manager.placement_score, which takes the median of these.
        self.heard = {}
        self.scanned_at = -1e9
        self.intent = None          # the SSID this radio is being held on
        self.pinned = None          # ...and the one the supplicant was told
        self.held_dbm = None        # last reading the driver actually filled in
        self.gone = False
        self.asked_ssid = None      # what a person asked this radio for
        self.asked_at = -1e9
        self.linked_at = None       # last time it was associated and addressed
        self.refused = {}           # ssid -> when it would not keep this radio

    # --- readings -----------------------------------------------------------
    @property
    def dbm(self):
        return self.held_dbm

    @property
    def loss_pct(self):
        if not self.pings:
            return 0.0
        lost = sum(1 for rtt in self.pings if rtt is None)
        return 100.0 * lost / len(self.pings)

    @property
    def rtt_ms(self):
        """The median round trip, which is the one a rover should be judged on.

        A mean would let one 400 ms retransmission on a busy channel stand in
        for a link that is answering in four, and moving the rover's traffic on
        the strength of a single retry is how a hysteresis gets defeated.
        """
        got = sorted(rtt for rtt in self.pings if rtt is not None)
        if not got:
            return None
        return got[len(got) // 2]

    @property
    def usable(self):
        """Whether this link could carry the rover's traffic at all.

        Three separate facts, and the interesting failures are the ones where
        two of them hold: associated, addressed, and a gateway that has answered
        at least once in the window. An association with no address is DHCP not
        having answered, and it looks exactly like being online from anywhere
        except here.

        **Neither the signal nor the SSID is one of them**, and both of those
        are corrections the recording forced rather than choices. Scored against
        9,437 samples the rover wrote down over two days:

        - requiring a readable signal scored 91.3%, and every disagreement was
          a link the rover had up and answering the gateway in a few
          milliseconds on which the driver had not filled the level column in;
        - requiring a named SSID as well scored 99.4%, and the remaining 59
          disagreements were all the same thing one field along -- up,
          addressed, pinging in 2 ms, and `iw` naming no network.

        `wifi_roam.sh` learned half of this once already and wrote it down: a
        reading a radio cannot produce is no signal reported, not a signal of
        zero. The general form is the one that matters here. Association,
        address, signal and reachability are four facts gathered from four
        places, and three of those places drop the occasional answer. A link
        that is carrying packets is a working link whatever the driver will
        admit to at that instant, so the ping is what this rests on and the rest
        is commentary -- which is also why a stale address left behind by a
        vanished access point does not fool it: nothing answers, and it goes.
        """
        if self.gone or self.operstate != "up" or not self.address:
            return False
        if not self.pings:
            return True
        recent = self.pings[-DEAD_PINGS:]
        if len(recent) >= DEAD_PINGS and not any(rtt is not None for rtt in recent):
            return False
        return self.loss_pct < 100.0

    # What to grade a link as when it is plainly working and the driver will not
    # say how well. Pessimistic enough that a radio in this state never wins a
    # failover on signal alone, and generous enough that it is never abandoned
    # for one: with the 8 dB margin, something genuinely better can still take
    # the traffic, and nothing merely ordinary can.
    UNGRADED_DBM = -70

    @property
    def score(self):
        level = usable_dbm(self.held_dbm)
        if level is None and not self.usable:
            return None
        return link_score(self.UNGRADED_DBM if level is None else level,
                          self.rtt_ms, self.loss_pct)

    def remember(self, seen, keep=SCANS_REMEMBERED):
        """File one scan's readings, and forget what has not been heard lately.

        Dropping the networks this scan did not mention matters as much as
        keeping the ones it did: an access point that has been switched off
        would otherwise go on contributing its last three good readings to a
        median for another minute and a half, which is exactly long enough to
        send the spare at something that is no longer there.
        """
        heard = {}
        for entry in seen:
            level = usable_dbm(entry.dbm)
            if level is None:
                continue
            previous = self.heard.get(entry.ssid, [])
            heard[entry.ssid] = (previous + [level])[-keep:]
        self.heard = heard

    def effective(self, primary_bonus=PRIMARY_BONUS_DB):
        """The score this radio is compared on: its grade, plus who it is.

        The bonus goes to the onboard radio and only in this comparison, so that
        two links a decibel apart do not hand the rover's traffic to the adapter
        that failed its own initialisation and fell off the USB bus in August.
        """
        base = self.score
        if base is None:
            return None
        return base + (0.0 if self.usb else primary_bonus)

    def as_dict(self, role):
        return {
            "iface": self.iface,
            "kind": "usb" if self.usb else "onboard",
            "mac": self.mac,
            "ssid": self.link.ssid,
            "bssid": self.link.bssid,
            "router": router(self.link.ssid) if self.link.ssid else None,
            "freq": self.link.freq,
            "band": band_of(self.link.freq),
            "dbm": self.held_dbm,
            "address": self.address,
            "operstate": "absent" if self.gone else self.operstate,
            "associated": self.link.associated,
            "usable": self.usable,
            "rtt_ms": None if self.rtt_ms is None else round(self.rtt_ms, 1),
            "loss_pct": round(self.loss_pct, 1),
            "score": None if self.score is None else round(self.score, 1),
            "role": role,
            "intent": self.intent,
            # Forgotten by the manager once the choice stops being honoured, so
            # the panel stops saying "you asked for this" at the moment it stops
            # being true rather than for the rest of the session.
            "asked": self.asked_ssid,
            "seen": [entry.as_dict() for entry in
                     sorted(self.seen, key=lambda s: -(s.dbm or -999))],
        }


class Manager:
    """Keep both radios associated, and decide which one carries the traffic.

    The two decisions are deliberately on different clocks and different
    evidence, which is the same split `wifi_roam.sh` arrived at: whether to
    leave is decided from the driver, and where to go is decided from a scan.

    - **Which link is active** is re-decided every second from measurements --
      signal, round trip and loss, on links that are both already up. It is
      cheap to act on, so it acts quickly.
    - **Where the standby sits** is re-decided every half minute from a scan,
      which can only report a signal. It costs an association, so it needs a
      real reason.

    The active radio's association is never touched. That is the guarantee the
    whole design rests on, and every method here is written so that the worst it
    can do to a working link is move an address off it.
    """

    def __init__(self, platform, service_ip=SERVICE_IP, gateway=GATEWAY,
                 margin_db=MARGIN_DB, hold_s=HOLD_S, cooldown_s=COOLDOWN_S,
                 scan_every_s=SCAN_EVERY_S, deadman_s=DEADMAN_S,
                 window=WINDOW, status_path=STATUS_PATH,
                 request_path=REQUEST_PATH, sticky_s=STICKY_S,
                 stranded_s=STRANDED_S, refused_s=REFUSED_S):
        self.platform = platform
        self.service_ip = service_ip or None
        self.gateway = gateway
        self.margin_db = margin_db
        self.hold_s = hold_s
        self.cooldown_s = cooldown_s
        self.scan_every_s = scan_every_s
        self.deadman_s = deadman_s
        self.window = window
        self.status_path = status_path
        self.request_path = request_path
        self.sticky_s = sticky_s
        self.stranded_s = stranded_s
        self.refused_s = refused_s

        self.radios = []
        self.active = None
        self.service_on = None
        self.service_refused = None     # the MAC that answered for the address
        self.better_since = None
        self.promote = None             # a radio a person asked the traffic onto
        # What each radio's rule and routing table were last written for. The
        # kernel deletes a route when the address it hangs off goes away, so a
        # lease changing under a radio empties that radio's table with nothing
        # said anywhere; this is how the manager notices and puts it back.
        self.applied = {}               # iface -> the address it was written for
        self.switched_at = -1e9
        self.switches = 0
        self.history = []
        self.started = platform.now()
        self.last_usable_at = platform.now()
        self.surrendered = False
        self.note = ""
        self.discover()

    # --- setting up ---------------------------------------------------------
    def discover(self):
        """Find the radios, and prepare the kernel for two of them on one LAN.

        `arp_ignore` and `arp_announce` are the whole of that preparation, and
        without them the design does not work at all: by default Linux answers
        an ARP request for *any* local address on *any* interface, so the
        standby radio would answer for the active radio's address, the access
        point it is on would learn that address, and the upstream bridge would
        deliver the rover's traffic to whichever radio spoke last. That is not a
        subtle degradation -- it is the rover's address ending up on the radio
        that is not carrying its traffic.
        """
        names = self.platform.wireless_interfaces()
        self.radios = [Radio(name, usb=self.platform.is_usb(name),
                             mac=self.platform.mac(name)) for name in names]
        self.platform.sysctl("net.ipv4.conf.all.arp_ignore", 1)
        self.platform.sysctl("net.ipv4.conf.all.arp_announce", 2)

    def radio(self, iface):
        for radio in self.radios:
            if radio.iface == iface:
                return radio
        return None

    @property
    def standby(self):
        others = [radio for radio in self.radios
                  if radio is not self.active and not radio.gone]
        if not others:
            return None
        return max(others, key=lambda r: (r.usable, r.effective() or -999))

    # --- one pass -----------------------------------------------------------
    def tick(self):
        self.sample()
        self.probe()
        if self.check_deadman():
            self.write_status()
            return
        self.read_request()
        self.choose_active()
        self.refresh_routes()
        self.free_stranded()
        self.place_radios()
        self.hold_intents()
        self.write_status()

    def read_request(self):
        """Take a network somebody asked for, and put the spare radio on it.

        Read and then deleted, so a request is acted on once. What it does with
        it depends on which radio it names, and the default -- naming none -- is
        the interesting one: the standby goes to the requested network, and the
        traffic follows only once that radio is associated, addressed and
        answering the gateway. The connection the request arrived through is
        never the one moved, so nothing drops.

        A request for a network this radio holds no passphrase for is refused
        here as well as in `wifi_ctl.sh`, because this one can say what it does
        know about, and a console showing the whole neighbourhood needs to be
        told that most of it is not somewhere this rover can go.
        """
        if not self.request_path:
            return
        try:
            with open(self.request_path) as handle:
                blob = handle.read()
        except OSError:
            return
        try:
            os.unlink(self.request_path)
        except OSError:
            pass
        try:
            want = json.loads(blob)
        except ValueError:
            self.say("could not read the request in %s" % self.request_path)
            return
        ssid = str(want.get("ssid") or "").strip()
        if not ssid:
            return
        radio = self.radio(want.get("iface")) if want.get("iface") else None
        if radio is None or radio.gone:
            standby = self.standby
            radio = standby if standby is not None and not standby.gone \
                else self.active
        if radio is None:
            return
        if ssid not in self.platform.networks(radio.iface):
            self.note = ("there is no passphrase for %s on %s"
                         % (ssid, radio.iface))
            self.say(self.note)
            return
        radio.intent = ssid
        radio.pinned = None
        radio.asked_ssid = ssid
        radio.asked_at = self.platform.now()
        self.promote = radio.iface if want.get("carry", True) else None
        self.note = ""
        self.say("asked for %s on %s%s" % (ssid, radio.iface,
                 "; the traffic will follow once it is working"
                 if self.promote else ""))

    def sample(self):
        for radio in self.radios:
            state = self.platform.operstate(radio.iface)
            radio.operstate = state
            if state == "absent":
                # The dongle falling off the USB bus is a thing this adapter has
                # actually done, twice, and it must cost nothing: the radio is
                # marked gone, the traffic is somewhere else already, and no
                # further call is made against an interface that is not there.
                if not radio.gone:
                    self.say("%s is no longer present" % radio.iface)
                radio.gone = True
                radio.link = Link()
                radio.address = None
                radio.pinned = None
                continue
            if radio.gone:
                self.say("%s is back" % radio.iface)
                radio.gone = False
            radio.link = self.platform.link(radio.iface)
            radio.address = self.platform.ipv4(radio.iface)
            level = usable_dbm(radio.link.dbm)
            if level is not None:
                radio.held_dbm = level
            elif not radio.link.associated:
                radio.held_dbm = None

    def probe(self):
        pairs = [(radio.iface, self.gateway) for radio in self.radios
                 if not radio.gone and radio.link.associated and radio.address]
        results = self.platform.ping(pairs) if pairs else {}
        for radio in self.radios:
            if radio.iface not in results:
                if not radio.gone and (not radio.link.associated
                                       or not radio.address):
                    radio.pings = []
                continue
            radio.pings.append(results[radio.iface])
            if len(radio.pings) > self.window:
                del radio.pings[:-self.window]

    # --- the two decisions --------------------------------------------------
    def choose_active(self):
        usable = [radio for radio in self.radios if radio.usable]
        if not usable:
            return
        self.last_usable_at = self.platform.now()

        # Somebody asked for a network and asked for the traffic to follow it.
        # That outranks the scoring and skips both the margin and the hold-down:
        # a person watching the console who has chosen an access point is not
        # served by being told it was only four decibels better. It still has to
        # be a link that is actually working, which is the whole difference
        # between this and the old join -- the wait is for the new radio to be
        # usable, not for the old one to be gone.
        if self.promote is not None:
            wanted = self.radio(self.promote)
            if wanted is None or wanted.gone:
                self.promote = None
            elif wanted.usable and wanted.link.ssid == wanted.asked_ssid:
                self.promote = None
                if wanted is not self.active:
                    self.make_active(wanted, "somebody asked for %s"
                                     % wanted.asked_ssid)
                return

        best = max(usable, key=lambda r: r.effective())

        if self.active is None or self.active.gone or not self.active.usable:
            why = ("nothing was carrying traffic yet" if self.active is None
                   else "%s lost the gateway" % self.active.iface)
            self.make_active(best, why)
            return

        if best is self.active:
            self.better_since = None
            return

        if (self.active.asked_ssid
                and self.active.link.ssid == self.active.asked_ssid
                and self.platform.now() - self.active.asked_at < self.sticky_s):
            # Somebody put the traffic here on purpose and recently. Leaving is
            # then only for a link that has stopped working -- handled above,
            # before this -- and not for one that is merely quieter than the
            # alternative. Without this the choice survived exactly one
            # hold-down: the model promoted the asked-for radio, waited a
            # minute, found the other one 38 dB louder and undid the whole
            # thing, which from the console looks like the button not working.
            self.better_since = None
            return

        gap = best.effective() - self.active.effective()
        if gap < self.margin_db:
            # One to three decibels is not a reason to move anything, and this
            # is where a rover parked in the overlap between two cells would
            # otherwise spend its afternoon.
            self.better_since = None
            return
        now = self.platform.now()
        if self.better_since is None or self.better_since[0] != best.iface:
            self.better_since = (best.iface, now)
            return
        if now - self.better_since[1] < self.hold_s:
            return
        if now - self.switched_at < self.cooldown_s:
            return
        self.make_active(best, "%s is %.0f dB better" % (best.iface, gap))

    def place_radios(self):
        """Decide where each radio that is free to move should sit.

        **The rule about scanning is the whole shape of this method**: a radio
        that is both carrying traffic and associated is never scanned and never
        moved, and everything else may be. That covers the standby, which is the
        usual case and is why the active link is never interrupted -- and it
        covers the case that would otherwise deadlock, which is the one that
        actually turned up the first time this was run against a model of a
        rover at boot: two radios associated with nothing, no active radio, and
        placement that only ran when there already was one. Nothing bootstrapped
        at all. An active radio with no association is carrying nothing by
        definition, so scanning it costs nothing either.

        The onboard radio is placed first -- the list is ordered so -- which is
        how it gets first pick of the routers, and the dongle then takes the
        best of what is left.
        """
        now = self.platform.now()
        for radio in sorted(self.radios, key=lambda r: (r.usb, r.iface)):
            if radio.gone:
                continue
            if radio is self.active and radio.link.associated:
                continue
            if now - radio.scanned_at < self.scan_every_s:
                continue
            radio.scanned_at = now
            seen = self.platform.scan(radio.iface)
            if seen:
                radio.seen = seen
                radio.remember(seen)
            self.place_one(radio)

    def place_one(self, radio):
        """Put one free radio on the loudest router no other radio is claiming."""
        if radio.asked_ssid:
            if self.platform.now() - radio.asked_at < self.sticky_s:
                # Somebody chose this one. Ten minutes of not being argued with
                # is the least a deliberate choice deserves.
                radio.intent = radio.asked_ssid
                return
            # And then the rover goes back to looking after itself, rather than
            # sitting for the rest of the afternoon on a network chosen for one
            # moment by somebody who has since closed the page. Forgotten here
            # rather than in the status, so the panel stops saying "you asked
            # for this" at the same moment it stops being true.
            radio.asked_ssid = None
        known = self.platform.networks(radio.iface)
        if not known:
            return
        heard = [entry for entry in radio.seen
                 if entry.ssid in known
                 and usable_dbm(entry.dbm) is not None]
        now = self.platform.now()
        candidates = [entry for entry in heard
                      if now - radio.refused.get(entry.ssid, -1e9)
                      >= self.refused_s]
        if not candidates and heard:
            # Everything this radio can hear has turned it away recently, which
            # is more likely to be the radio than the routers. Forgetting and
            # trying again beats sitting still with a list of grudges.
            radio.refused.clear()
            candidates = heard
        if not candidates:
            return
        # Whatever the other radios are on, or are being moved to. Written as a
        # set of routers rather than "the active one's SSID" so that it is also
        # right at boot, when there is no active radio and the two are choosing
        # at the same moment.
        taken = set()
        for other in self.radios:
            if other is radio or other.gone:
                continue
            claim = other.intent or other.link.ssid
            if claim:
                taken.add(router(claim))
        free = [entry for entry in candidates if entry.router not in taken]
        if not free:
            # Every router this radio can hear is already spoken for. If it is
            # associated, leave it alone -- moving it would gain nothing. If it
            # is associated with nothing, put it beside the other one anyway: a
            # second radio on the same router is no defence against that router
            # failing, but it is still a defence against an adapter failing, and
            # this dongle has failed twice.
            if radio.link.associated:
                radio.intent = radio.link.ssid
                return
            free = candidates
        best = max(free, key=lambda entry: self.placement_score(radio, entry))
        if radio.intent == best.ssid and radio.pinned == best.ssid:
            # Already being held exactly there, and still trying to get on.
            # Saying it again would mean another `select_network`, and
            # `select_network` restarts the association -- so a radio slow to
            # join would be interrupted every time this ran and never finish,
            # while the log filled with "moving from nothing to" lines about a
            # radio that was not being moved anywhere.
            return
        here = radio.link.ssid
        if here == best.ssid:
            radio.intent = best.ssid
            return
        collided = bool(here and router(here) in taken)
        if here and not collided:
            current = next((entry for entry in radio.seen
                            if entry.ssid == here), None)
            if current is not None and usable_dbm(current.dbm) is not None:
                if (self.placement_score(radio, best)
                        < self.placement_score(radio, current) + self.margin_db):
                    radio.intent = here
                    return
        why = ("it was on the same router as the other radio" if collided
               else "it is the loudest router no other radio is on")
        self.say("moving %s from %s to %s at %s dBm, because %s"
                 % (radio.iface, here or "nothing", best.ssid, best.dbm, why))
        radio.intent = best.ssid
        radio.pinned = None         # so hold_intents re-pins it this tick

    def placement_score(self, radio, entry):
        """Where to send a radio, from scans -- which can only report signal.

        The band bonus lives here and nowhere else. A scan cannot say what the
        latency through an access point will be, so the only thing worth adding
        to a signal reading is a standing preference for the band that carries
        more when the signals are comparable.

        **The reading is the median of the last few scans, not the latest one**,
        and that is the whole of the fix for something the rover did an hour
        after this was first armed:

            08:58:01  moving wlan1 from TheMaharaja to TheGreatViking at -50 dBm
            08:58:30  moving wlan1 from TheGreatViking to TheMaharaja at -66 dBm

        Twenty-nine seconds apart in a house where nothing had moved. Twelve
        consecutive scans measured on the rover afterwards put TheGreatViking
        between -74 and -84, so the -50 was one sample wrong by nearly thirty
        decibels -- the same excursion this directory's README already records
        about `nmcli`, one access point reading 50 and then 97 inside a minute.

        A median over three scans throws a single excursion away outright and
        still follows a real change within a minute and a half, which is the
        right trade for a radio that is carrying nothing: what protects the rover
        is the *active* link's health, and that is measured every second from the
        driver and the gateway rather than from any scan.
        """
        levels = [level for level in radio.heard.get(entry.ssid, [])
                  if usable_dbm(level) is not None]
        if not levels:
            level = usable_dbm(entry.dbm)
            if level is None:
                return -999.0
            levels = [level]
        levels.sort()
        median = levels[len(levels) // 2]
        return median + (BAND_BONUS_DB if band_of(entry.freq) == "5" else 0.0)

    def free_stranded(self):
        """Give up holding a radio on a network it has not managed to join.

        The clock is the last moment the radio was associated *and* addressed,
        and it is deliberately not restarted by re-pinning: the failure this
        exists for looks like progress from the outside, because the manager
        re-chooses the same loudest access point after every scan and pins it
        again, so a deadline measured from the last pin would never expire.

        Freeing it is `release`, the same call every exit path makes: every
        network the radio knows is enabled again and the supplicant may take
        whatever it can actually hold. The network that would not have it is
        remembered so the next placement does not send it straight back.

        Only ever the radio that has no link at all. A radio that is associated
        and addressed but cannot reach the gateway is a different fault with a
        different answer -- moving the traffic, which `choose_active` has
        already done by the time this runs.
        """
        now = self.platform.now()
        for radio in self.radios:
            if radio.gone:
                continue
            if radio.linked_at is None:
                radio.linked_at = now
            if radio.link.associated and radio.address:
                radio.linked_at = now
                continue
            if not radio.pinned or now - radio.linked_at < self.stranded_s:
                continue
            self.say("%s has been held on %s for %.0f s without joining it, so "
                     "every network is enabled again and it may take what it "
                     "can" % (radio.iface, radio.pinned,
                              now - radio.linked_at))
            radio.refused[radio.pinned] = now
            self.platform.release(radio.iface)
            radio.pinned = None
            radio.intent = None
            radio.linked_at = now

    def hold_intents(self):
        """Keep each radio where it was put, and never move the active one.

        The active radio's intent is simply wherever it already is: pinning it
        there costs nothing while it holds, and it is what stops a supplicant
        that gets disconnected from re-associating with the access point the
        standby is using and quietly collapsing the redundancy to one router.
        """
        for radio in self.radios:
            if radio.gone:
                continue
            if radio is self.active and radio.link.associated:
                radio.intent = radio.link.ssid
            if not radio.intent or radio.pinned == radio.intent:
                continue
            if self.platform.pin(radio.iface, radio.intent):
                radio.pinned = radio.intent
            else:
                radio.intent = None

    # --- acting -------------------------------------------------------------
    def make_active(self, radio, why):
        was = self.active.iface if self.active else None
        self.active = radio
        self.better_since = None
        self.switched_at = self.platform.now()
        if was is not None:
            self.switches += 1
            self.history.insert(0, {"at": round(time.time(), 1),
                                    "t": round(self.platform.now(), 1),
                                    "from": was, "to": radio.iface, "why": why})
            del self.history[8:]
            self.say("traffic moves from %s to %s (%s), now on %s at %s dBm"
                     % (was, radio.iface, why, radio.link.ssid, radio.held_dbm))
        else:
            self.say("%s carries the traffic (%s), on %s at %s dBm"
                     % (radio.iface, why, radio.link.ssid, radio.held_dbm))
        self.claim_service_ip(radio)
        self.route_through(radio)

    def claim_service_ip(self, radio):
        """Move the rover's stable address onto this radio, if it may have it.

        Probed before every claim rather than once at startup, because the
        reason to check is somebody being handed the address by DHCP in the
        meantime, and that can happen between two failovers as easily as before
        the first. An address that answers is not taken: the manager says so and
        runs without one, which costs the connections that were already open and
        nothing else.
        """
        if not self.service_ip:
            return
        holder = self.platform.arp_probe(radio.iface, self.service_ip)
        if holder:
            self.service_refused = holder
            self.note = ("%s is already answered for by %s, so this rover has "
                         "no stable address" % (self.service_ip, holder))
            self.say(self.note)
            self.clear_service_rules()
            if self.service_on:
                self.platform.del_service_ip(self.service_on, self.service_ip)
                self.service_on = None
            self.service_ip = None
            return
        if self.service_on and self.service_on != radio.iface:
            self.platform.del_service_ip(self.service_on, self.service_ip)
        self.platform.add_service_ip(radio.iface, self.service_ip)
        self.service_on = radio.iface
        # And this is the failover, as far as the rest of the house is
        # concerned: everything else is bookkeeping.
        self.platform.garp(radio.iface, self.service_ip)

    def route_through(self, radio):
        src = self.service_ip if self.service_on == radio.iface else radio.address
        self.platform.set_default(radio.iface, self.gateway, src=src, metric=50)
        for index, other in enumerate(self.radios):
            now = None if other.gone else other.address
            was = self.applied.get(other.iface)
            if was and was != now:
                # The old rule names an address this radio no longer has. Left
                # behind it matches nothing and its table has already been
                # emptied by the kernel, which is how a rule for a lease three
                # renewals old ends up in `ip rule` pointing at nothing.
                self.platform.clear_interface_table(was, TABLE_BASE + index)
                self.applied.pop(other.iface, None)
            if not now:
                continue
            self.platform.set_interface_table(other.iface, now,
                                              TABLE_BASE + index)
            self.applied[other.iface] = now
        self.route_service_ip()

    def refresh_routes(self):
        """Put back what a DHCP renewal quietly took away.

        Everything above is written once, when the traffic moves between
        radios. That was enough until it was noticed what a changing lease does
        to it: **the kernel deletes a route when the address it is anchored to
        goes away**, silently, in every table at once. `kernel_route_lifetime.sh`
        measures it on this board -- two routes before the address is removed,
        none afterwards -- and removing the `src` does not save them either,
        because the gateway they point through sits in the prefix that went with
        it. So they have to be rebuilt, and the only question is when.

        This house makes that an hourly event rather than a curiosity. A second
        DHCP server answers alongside the router -- a TP-Link extender at
        192.168.1.232 -- and whichever replies first decides, so the rover's
        addresses genuinely change several times an hour without a radio ever
        losing its association. On 2026-08-27 the dongle went .47, .100, .47,
        .100, .47 inside eight minutes.

        Until this method existed, each of those emptied the table that the
        service address's policy rule points at, and nothing rebuilt it until
        the next failover happened to. With one radio sick and the other
        healthy -- exactly the rover's state that afternoon -- there is no next
        failover, and the rule then falls through to the main table, which holds
        one connected route per radio at the same metric and breaks the tie the
        same way every time. That is the eleven-and-a-half-minute fault of
        2026-08-26 with a renewal as its cause instead of a handover.

        A tick's worth of that window remains and cannot be closed from here:
        the kernel acts the moment the lease lands and nothing tells this
        process. One second of replies leaving by the wrong radio on a bridged
        LAN is a different animal from eleven minutes of it, and the model
        holds the fix to that bound.
        """
        if self.surrendered or self.active is None or self.active.gone:
            return
        for radio in self.radios:
            now = None if radio.gone else radio.address
            if self.applied.get(radio.iface) != now:
                self.route_through(self.active)
                return

    def route_service_ip(self):
        """Make the address that moves leave by the radio it moved to.

        Each radio's own address gets a rule above, so a packet reaching the
        standby is answered out of the standby. The service address had none,
        and fell through to the main table -- which holds one connected route
        per radio at the same metric and breaks the tie the same way every time.
        So a failover put the address on the healthy radio and went on answering
        it out of the sick one, which is invisible from the rover: every link
        check here is bound to a radio and they all pass.

        That is the eleven and a half minutes of 2026-08-26. The rover was up,
        associated, carrying traffic and completely unreachable at the only
        address anything bookmarks.

        Rewritten rather than moved, because the address can be on one radio
        only: every other radio's copy goes first and the holder's is added
        afterwards, so a failover interrupted halfway leaves nothing behind
        pointing at a radio that is not carrying anything.
        """
        if not self.service_ip or not self.service_on:
            return
        holder = None
        for index, radio in enumerate(self.radios):
            if radio.iface == self.service_on:
                holder = index
            else:
                self.platform.clear_source_rule(self.service_ip,
                                                TABLE_BASE + index)
        if holder is not None:
            self.platform.set_source_rule(self.service_ip,
                                          TABLE_BASE + holder)

    def clear_service_rules(self):
        """Every trace of the rule above, for when the address is given up."""
        if not self.service_ip:
            return
        for index, _radio in enumerate(self.radios):
            self.platform.clear_source_rule(self.service_ip, TABLE_BASE + index)

    # --- giving up, and coming back ----------------------------------------
    def check_deadman(self):
        """Hand the radios back when the plan has demonstrably stopped working.

        Two minutes with neither radio reaching the gateway is not a bad link;
        it is evidence that whatever this manager is doing is not the thing that
        will fix it. So it undoes all of it -- both radios free to associate
        wherever they can, no service address, no policy routes -- and waits.
        The rover then has exactly what it had before any of this was written,
        which is a supplicant that will eventually find something.
        """
        now = self.platform.now()
        if any(radio.usable for radio in self.radios):
            self.last_usable_at = now
            if self.surrendered:
                self.surrendered = False
                self.note = ""
                self.say("a radio is answering again; taking the link back")
            return False
        if self.surrendered:
            return True
        if now - self.last_usable_at < self.deadman_s:
            return False
        self.surrendered = True
        self.note = ("neither radio has reached the gateway for %.0f s, so both "
                     "have been handed back to wpa_supplicant"
                     % (now - self.last_usable_at))
        self.say(self.note)
        self.restore(keep_running=True)
        return True

    def restore(self, keep_running=False):
        """Undo everything, in the order that leaves the rover reachable.

        The service address goes first and the radios are freed last, so that
        there is never a moment with an address on an interface nothing is
        allowed to associate. `arp_ignore` is deliberately left where it is: two
        radios on one subnet want it whether or not this manager is running, and
        putting it back would restore the cross-answering rather than the
        status quo.
        """
        if self.service_ip and self.service_on:
            self.clear_service_rules()
            self.platform.del_service_ip(self.service_on, self.service_ip)
            self.service_on = None
        for index, radio in enumerate(self.radios):
            # By what was installed rather than by what the radio holds now: a
            # lease that changed since leaves the rule under the old address.
            address = self.applied.pop(radio.iface, None) or radio.address
            if address:
                self.platform.clear_interface_table(address,
                                                    TABLE_BASE + index)
            radio.intent = None
            radio.pinned = None
            if not radio.gone:
                self.platform.release(radio.iface)
        if not keep_running:
            self.active = None

    # --- saying so ----------------------------------------------------------
    def say(self, message):
        self.platform.log("wifi_dual: " + message)

    def status(self):
        active = self.active.iface if self.active else None
        standby = self.standby
        roles = {}
        for radio in self.radios:
            if radio.gone:
                roles[radio.iface] = "absent"
            elif radio is self.active:
                roles[radio.iface] = "active"
            elif standby is not None and radio is standby:
                roles[radio.iface] = "standby"
            else:
                roles[radio.iface] = "spare"
        return {
            "at": round(time.time(), 1),
            "uptime_s": round(self.platform.now() - self.started, 1),
            "gateway": self.gateway,
            "service_ip": self.service_ip,
            "service_on": self.service_on,
            "service_refused_by": self.service_refused,
            "active": active,
            "standby": standby.iface if standby else None,
            "since_switch_s": round(self.platform.now() - self.switched_at, 1)
                              if self.switches or self.active else None,
            "switches": self.switches,
            "surrendered": self.surrendered,
            "note": self.note,
            "margin_db": self.margin_db,
            "radios": [radio.as_dict(roles[radio.iface])
                       for radio in self.radios],
            "history": list(self.history),
        }

    def write_status(self):
        """Leave the whole picture where anything can read it without root.

        The console asks the daemon, the daemon reads this file, and neither
        spends a `sudo` or a process to do it. That is a real change from the
        single-radio panel, which cost about two seconds of `nmcli` per look and
        was cached for twenty because of it.
        """
        if not self.status_path:
            return
        blob = json.dumps(self.status(), sort_keys=False)
        try:
            temporary = self.status_path + ".new"
            with open(temporary, "w") as handle:
                handle.write(blob + "\n")
            os.chmod(temporary, 0o644)
            os.replace(temporary, self.status_path)
        except OSError:
            pass

    def run(self, tick_s=TICK_S, until=None):
        while True:
            began = self.platform.now()
            self.tick()
            if until is not None and until(self):
                return
            rest = tick_s - (self.platform.now() - began)
            if rest > 0:
                self.platform.sleep(rest)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--once", action="store_true",
                        help="one pass, print the status, exit")
    parser.add_argument("--dry-run", "-n", action="store_true",
                        help="decide everything, change nothing")
    parser.add_argument("--restore", action="store_true",
                        help="hand both radios back to wpa_supplicant and exit")
    parser.add_argument("--status", action="store_true",
                        help="print what the running manager last wrote")
    parser.add_argument("--service-ip", default=SERVICE_IP,
                        help="the address that moves between the radios; "
                             "empty to run on route metrics alone")
    parser.add_argument("--tick", type=float, default=TICK_S)
    args = parser.parse_args(argv)

    if args.status:
        try:
            with open(STATUS_PATH) as handle:
                sys.stdout.write(handle.read())
            return 0
        except OSError as error:
            sys.stderr.write("no status from a running manager: %s\n" % error)
            return 1

    platform = Platform(dry_run=args.dry_run)
    # A dry run writes no status file, which is not tidiness. `wifi_ctl.sh` reads
    # that file's age to decide whether a manager is running, and a join lands in
    # a request file only a running manager ever reads -- so a dry run that left
    # a fresh status behind would make every console join on that rover vanish
    # silently for the next fifteen seconds.
    manager = Manager(platform, service_ip=args.service_ip or None,
                      status_path=None if args.dry_run else STATUS_PATH,
                      request_path=None if args.dry_run else REQUEST_PATH)

    if args.restore:
        # Deliberately usable on a rover where the manager was never running:
        # the point of it is to be the thing somebody can reach for when a
        # radio has been left pinned, and it must not need a process to talk to.
        manager.restore()
        manager.say("both radios handed back to wpa_supplicant")
        return 0

    if args.once:
        manager.tick()
        json.dump(manager.status(), sys.stdout, indent=2)
        sys.stdout.write("\n")
        return 0

    import signal

    def stop(signum, frame):
        # Every way out of this program frees the radios. A manager that exits
        # with one network enabled and the rest disabled is the one shape of
        # this file that could strand a wifi-only rover.
        manager.say("stopping on signal %d" % signum)
        manager.restore()
        raise SystemExit(0)

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    try:
        manager.run(tick_s=args.tick)
    except SystemExit:
        raise
    except BaseException:
        manager.restore()
        raise
    return 0


if __name__ == "__main__":
    sys.exit(main())
