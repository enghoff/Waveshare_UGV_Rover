#!/usr/bin/env python3
"""A house with three routers in it, for `wifi_dual.py` to be wrong in cheaply.

The rule this file exists to satisfy is in CLAUDE.md: a fix for a fault nobody
has reproduced is a guess, and three fixes have already been deployed to this
rover on reasoning alone and left the fault exactly where it was. So before the
dual-radio manager touches the running system it is driven round a model of the
building, through every failure the design claims to survive, and -- more
importantly -- it is held against a recording of the real fault.

There are two halves here and they do different jobs.

**`World` is a model, and a model is only as good as its last calibration.** It
places three routers in a house, puts the rover somewhere, and computes a signal
from a path-loss law. It is how a scripted scenario gets written -- drive the
rover out of one cell and into another, switch a router off, pull the dongle out
-- and its propagation law is a convenience, not evidence. Nothing about the
manager is proved by it alone.

**`Trace` is a recording, and that is what earns the model the right to be
believed.** `netwatch/` has been writing a line every ten seconds since August:
signal, round trip, association, address and the supplicant's own account of
every disconnection. Replaying one of those through the manager's own scoring
and asking whether it calls the link healthy exactly where the rover called it
healthy is the calibration -- the same thing `ros_nav/dwb_replay.py` does for the
drive controller, and for the same reason. `test_wifi_dual.py` refuses to report
a result from a scenario until that replay has matched.

    python3 wifi_world.py --replay netwatch.log        # what a recording says
    python3 wifi_world.py --replay netwatch.log --dual # ...and what two radios
                                                       # would have done with it
"""
from __future__ import annotations

import argparse
import json
import math
import sys

import wifi_dual
from wifi_dual import Link, Seen, band_of, router

# --- the modelled house ------------------------------------------------------
#
# Signal is a log-distance path loss: a reference level at one metre and an
# exponent for how fast it falls off through walls. n = 2.6 is the middle of the
# usual indoor range, and the reference is set so that the model reproduces the
# three readings actually taken from the rover where it sits today: the onboard
# radio on TheGreatLord 5G at -30 dBm, and the dongle hearing TheGreatLord at
# -40 and TheMaharaja at -72.
#
# This is the part of the file that is a convenience rather than evidence, and
# it is worth being blunt about which conclusions may lean on it. It decides
# where the rover can hear what, which is how a scenario gets *written*. It
# decides nothing about how the manager grades a link or when it moves the
# traffic -- those come from the recording.
REFERENCE_DBM = -30.0
PATH_EXPONENT = 2.6
# 5 GHz costs about 7 dB more than 2.4 GHz over the same distance through the
# same walls, which is why the dual-band radio is the one worth putting there
# and the 2.4-only dongle is the one worth keeping for reach.
FIVE_GHZ_PENALTY_DB = 7.0

# How long the modelled hardware takes to do things, from the recording of
# 2026-08-24: a disconnect at 00:11:34, associated with the next access point at
# 00:11:35, and an address by 00:11:44. Sampling was every ten seconds, so the
# DHCP figure is an upper bound rather than a measurement, and the model takes
# the pessimistic reading on purpose -- a failover that only looks good because
# the model let DHCP finish quickly would be a failover that looks good for the
# wrong reason.
ASSOCIATE_S = 1.0
DHCP_S = 8.0
SCAN_S = 3.0

# How wrong a scan's signal figure is, and how often it changes its mind. Both
# from what this rover has actually been measured doing rather than from a
# distribution: the scan is the noisy instrument here and the driver's own
# reading is the steady one, which is why the manager decides *whether to leave*
# from the driver and *where to go* from a scan, and never compares the two.
# Measured over twelve consecutive standby scans on 2026-08-25: the access
# points the spare was choosing between sat within 2 to 10 dB of themselves, so
# the ordinary jitter is small. What is not small is the excursion below.
SCAN_NOISE_DB = 4.0
SCAN_NOISE_PERIOD_S = 20.0
# One scan in twelve reports one access point far louder than it is. Both
# numbers are from the rover: TheGreatViking was reported at -50 dBm at 08:58 on
# 2026-08-25 and read -74 to -84 across every scan twenty minutes later, and the
# manager spent a re-association on that single sample.
SCAN_EXCURSION_IN = 12
SCAN_EXCURSION_DB = 26.0


class AccessPoint:
    def __init__(self, ssid, x, y, freq=2437, up=True, latency_ms=2.5,
                 loss_pct=0.0, reaches_lan=True):
        self.ssid = ssid
        self.x, self.y = x, y
        self.freq = freq
        self.up = up
        self.latency_ms = latency_ms
        self.loss_pct = loss_pct
        # An access point with a signal and no path to the LAN is the failure
        # the document singles out, and the one that RSSI alone cannot see. It
        # is modelled separately from `up` precisely so a scenario can build it.
        self.reaches_lan = reaches_lan

    @property
    def router(self):
        return router(self.ssid)

    def dbm_at(self, x, y):
        distance = max(0.8, math.hypot(self.x - x, self.y - y))
        level = REFERENCE_DBM - 10.0 * PATH_EXPONENT * math.log10(distance)
        if band_of(self.freq) == "5":
            level -= FIVE_GHZ_PENALTY_DB
        return int(round(level))


class SimRadio:
    def __init__(self, iface, usb=False, mac="02:00:00:00:00:01",
                 bands=("2.4", "5"), present=True, knows=()):
        self.iface = iface
        self.usb = usb
        self.mac = mac
        self.bands = tuple(bands)
        self.present = present
        # The networks this radio's supplicant holds a passphrase for. Modelled
        # per radio rather than globally because that is how netplan generates
        # it -- one wpa_supplicant per interface, each with its own list -- and a
        # manager that assumed both radios knew the same networks would work in
        # the model and fail on a board where only one stanza was installed.
        self.knows = set(knows)
        self.enabled = set(knows)
        self.ssid = None
        self.address = None
        self.associating_until = None
        self.dhcp_until = None
        self.pinned = None      # what the manager told this radio to hold
        self.trying = None      # ...and what it is actually associating with

    def can_use(self, ap):
        return band_of(ap.freq) in self.bands


class World:
    def __init__(self, aps, radios, x=0.0, y=0.0, gateway="192.168.1.1"):
        self.aps = list(aps)
        self.radios = {radio.iface: radio for radio in radios}
        self.x, self.y = x, y
        self.gateway = gateway
        self.t = 0.0
        self.log = []
        self.rfkill_asked = []      # asserted empty by the self-test
        self.claimed = {}           # address -> iface, so a double claim shows
        self.arp_owner = {}         # address -> mac already on the LAN
        self.script = []            # (time, callable(world))
        self.lies = {}              # ssid -> (until, dB), a bad spell
        self.lies_once = {}         # ssid -> dB, spent by the next scan
        # Whether scans get the occasional wild reading of their own.
        # Switched off by a scenario that supplies exactly one, so that
        # the deliberate fault is the only one in the run: a natural
        # excursion landing on the *same* scan masked the injected one
        # and made a broken manager look fixed.
        self.excursions = True
        self.next_octet = 140
        # The routing the manager installs, kept because the fault of
        # 2026-08-26 lives here and nowhere else: an address can be on the right
        # radio and still be answered out of the wrong one.
        self.rules = {}             # source address -> routing table
        self.tables = {}            # routing table -> iface it points at
        self.stranded = []          # (t, address, holder, the radio it leaves by)

    # --- driving it ---------------------------------------------------------
    def at(self, when, action):
        self.script.append((when, action))
        return self

    def advance(self, seconds):
        end = self.t + seconds
        while self.script and self.script[0][0] <= end:
            self.script.sort(key=lambda item: item[0])
            when, action = self.script[0]
            if when > end:
                break
            self.script.pop(0)
            self.t = max(self.t, when)
            action(self)
        self.t = end
        self.settle()
        self.check_service_path()

    def egress(self, address):
        """Which radio a reply from this address actually leaves by.

        A source rule wins, if the address has one and that table has anything
        in it. Otherwise the main table decides, and the main table holds one
        connected route per radio at the same metric, so the kernel breaks the
        tie by interface order and picks the same radio every time regardless of
        which one is carrying traffic.

        That last part is calibrated, not assumed. With the service address on
        `wlan1` the rover answered `ip route get 192.168.1.206 from
        192.168.1.80` with `dev wlan0`.
        """
        table = self.rules.get(address)
        if table is not None and table in self.tables:
            return self.tables[table]
        for iface, radio in self.radios.items():
            if radio.present and radio.address:
                return iface
        return None

    def check_service_path(self):
        """An address on one radio and answered out of another is stranded.

        Checked every tick rather than at the end of a scenario, because the
        interesting version of this fault is transient: it opens at a failover
        and closes again when something unrelated moves the traffic back.
        """
        for address, holder in self.claimed.items():
            radio = self.radios.get(holder)
            if radio is None or not radio.present or not radio.address:
                continue
            leaves_by = self.egress(address)
            if leaves_by is not None and leaves_by != holder:
                self.stranded.append((round(self.t, 1), address, holder,
                                      leaves_by))

    def settle(self):
        for radio in self.radios.values():
            if not radio.present:
                radio.ssid = radio.address = None
                radio.associating_until = radio.dhcp_until = None
                continue
            if radio.associating_until is not None and self.t >= radio.associating_until:
                radio.associating_until = None
                if radio.trying and self.ap(radio.trying) and self.ap(radio.trying).up:
                    radio.ssid = radio.trying
                    radio.dhcp_until = self.t + DHCP_S
            if radio.dhcp_until is not None and self.t >= radio.dhcp_until:
                radio.dhcp_until = None
                radio.address = "192.168.1.%d" % self.next_octet
                self.next_octet += 1
            ap = self.ap(radio.ssid)
            if radio.ssid and (ap is None or not ap.up):
                # The access point went away under an associated radio, which is
                # the one thing a supplicant notices by itself.
                radio.ssid = None
                radio.address = None
            self.reassociate(radio)

    def reassociate(self, radio):
        """What `wpa_supplicant` does by itself when nothing is holding it.

        This is not decoration. It is the safety net the manager's dead-man
        depends on: when nothing has worked for two minutes the manager frees
        both radios and stands back, and the whole value of standing back is
        that the supplicant then goes and finds something. A model without this
        would show the manager surrendering and the rover never coming back,
        which is both wrong and the most alarming possible way to be wrong.

        With a network pinned it retries that one and only that one, which is
        what `select_network` means and why leaving a radio pinned after the
        manager has gone would be the one way this design could strand a rover.
        """
        if radio.ssid or radio.associating_until is not None:
            return
        options = [ap for ap in self.aps
                   if ap.up and radio.can_use(ap) and ap.ssid in radio.enabled]
        if not options:
            return
        best = max(options, key=lambda ap: ap.dbm_at(self.x, self.y))
        radio.trying = best.ssid
        radio.associating_until = self.t + ASSOCIATE_S

    def ap(self, ssid):
        for ap in self.aps:
            if ap.ssid == ssid:
                return ap
        return None

    def visible(self, radio):
        seen = []
        for ap in self.aps:
            if not ap.up or not radio.can_use(ap):
                continue
            level = ap.dbm_at(self.x, self.y) + self.scan_noise(ap)
            until, offset = self.lies.get(ap.ssid, (0.0, 0.0))
            if self.t < until:
                level += offset
            if ap.ssid in self.lies_once:
                level += self.lies_once.pop(ap.ssid)
            if level < -92:         # below what either of these radios reports
                continue
            seen.append(Seen(ap.ssid, "02:%s" % ap.ssid[:5], ap.freq,
                             int(round(level))))
        return seen

    def lie_once(self, ssid, decibels):
        """Make the *next* scan report one access point wrongly, and only that one.

        This is the shape the rover actually produced and so it is the shape the
        reproduction has: twelve consecutive standby scans measured on
        2026-08-25 put every access point within 2 to 10 dB of itself, and the
        reading that cost a re-association twenty minutes earlier was a single
        sample nearly thirty decibels out. A bad *spell* is a different fault
        with a different answer -- :meth:`lie` builds that one -- and testing the
        fix for one against the other would prove nothing about either.
        """
        self.lies_once[ssid] = float(decibels)

    def lie(self, ssid, decibels, seconds=25.0):
        """Make scans report one access point wrongly for a while.

        The general noise below is a distribution and is therefore a matter of
        luck: a scenario that relies on it happening to fire for the right access
        point at the right moment is a scenario that passes or fails for reasons
        nobody chose. This is the same fault stated exactly -- one scan, one
        access point, wrong by a known amount, at a known time -- which is what a
        reproduction of something the rover actually did should look like.
        """
        self.lies[ssid] = (self.t + seconds, float(decibels))

    def scan_noise(self, ap):
        """What a scan gets wrong, which is a great deal more than a driver does.

        This is not garnish. A scan figure on this hardware is wild: the
        wifi_roam README records consecutive scans putting the *same*
        association anywhere from 74 to 88 on NetworkManager's 0-100 scale --
        seven decibels -- and one access point swinging from 50 to 97 and back
        inside a minute, which is twenty-three. A model whose scans return the
        propagation law exactly is a model in which no placement decision can
        ever be wrong, and the first thing the real rover did when this manager
        was armed was flap its spare radio between two routers every thirty
        seconds, which the noiseless model had said nothing about.

        Deterministic in the scan's time and the access point's name rather than
        random, so a scenario that fails does so identically the next time.
        """
        if not SCAN_NOISE_DB:
            return 0.0
        seed = (int(self.t) // max(1, int(SCAN_NOISE_PERIOD_S))) * 2654435761
        for char in ap.ssid:
            seed = (seed * 31 + ord(char)) & 0xFFFFFFFF
        seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF
        jitter = ((seed >> 8) % 1000 / 1000.0 - 0.5) * 2 * SCAN_NOISE_DB
        # And the excursion, which is the shape that actually matters and which
        # a symmetric spread does not contain. Measured on the rover on
        # 2026-08-25: over twelve consecutive standby scans the access points sat
        # within 2 to 10 dB of themselves -- and twenty minutes earlier one of
        # those same access points had been reported at -50 dBm where it now
        # reads -78. One sample, wrong by nearly thirty decibels, and it cost the
        # spare radio a re-association. That is the same thing this directory's
        # README already records about `nmcli`: one AP swinging from 50 to 97 and
        # back inside a minute.
        if self.excursions and (seed >> 20) % SCAN_EXCURSION_IN == 0:
            return jitter + SCAN_EXCURSION_DB
        return jitter


class SimPlatform:
    """`wifi_dual.Platform`, answered out of a :class:`World`.

    Every method here is the same shape as the real one, and the manager cannot
    tell them apart. That is the whole point: a manager with one real file read
    left in it would be untestable in exactly the places worth testing, which
    are the ones that only happen when something has already gone wrong.
    """

    def __init__(self, world, quiet=True):
        self.world = world
        self.quiet = quiet
        self.dry_run = False
        self.scans = 0
        self.garps = []
        self.pins = []

    # --- clocks and noises --------------------------------------------------
    def now(self):
        return self.world.t

    def sleep(self, seconds):
        self.world.advance(seconds)

    def log(self, message):
        self.world.log.append(message)
        if not self.quiet:
            sys.stderr.write(message + "\n")

    # --- looking ------------------------------------------------------------
    def wireless_interfaces(self):
        return [name for name, radio in sorted(self.world.radios.items())
                if radio.present]

    def is_usb(self, iface):
        radio = self.world.radios.get(iface)
        return bool(radio and radio.usb)

    def mac(self, iface):
        radio = self.world.radios.get(iface)
        return radio.mac if radio else None

    def operstate(self, iface):
        radio = self.world.radios.get(iface)
        if radio is None or not radio.present:
            return "absent"
        if radio.ssid:
            return "up"
        return "dormant"

    def link(self, iface):
        radio = self.world.radios.get(iface)
        if radio is None or not radio.present or not radio.ssid:
            return Link()
        ap = self.world.ap(radio.ssid)
        if ap is None or not ap.up:
            return Link()
        return Link(ap.ssid, "02:%s" % ap.ssid[:5], ap.freq,
                    ap.dbm_at(self.world.x, self.world.y))

    def ipv4(self, iface):
        radio = self.world.radios.get(iface)
        return radio.address if radio and radio.present else None

    def scan(self, iface):
        radio = self.world.radios.get(iface)
        if radio is None or not radio.present:
            return []
        self.scans += 1
        # A scan takes the radio off channel, so it costs time even here -- the
        # manager must not be able to pass a test by scanning for free.
        self.world.advance(SCAN_S)
        return self.world.visible(radio)

    # --- moving -------------------------------------------------------------
    def networks(self, iface):
        radio = self.world.radios.get(iface)
        if radio is None or not radio.present:
            return {}
        return {ssid: index for index, ssid in enumerate(sorted(radio.knows))}

    def pin(self, iface, ssid):
        radio = self.world.radios.get(iface)
        if radio is None or not radio.present or ssid not in radio.knows:
            return False
        self.pins.append((round(self.world.t, 1), iface, ssid))
        radio.pinned = ssid
        # select_network disables every other configured network, which is the
        # one way this design could strand the rover, so the model does it too.
        radio.enabled = {ssid}
        if radio.ssid != ssid:
            radio.ssid = None
            radio.address = None
            radio.trying = ssid
            radio.associating_until = self.world.t + ASSOCIATE_S
        return True

    def release(self, iface):
        radio = self.world.radios.get(iface)
        if radio is None:
            return False
        radio.enabled = set(radio.knows)
        radio.pinned = None
        return True

    # --- addresses and routes ----------------------------------------------
    def addresses(self, iface):
        radio = self.world.radios.get(iface)
        found = [radio.address] if radio and radio.address else []
        found += [ip for ip, on in self.world.claimed.items() if on == iface]
        return found

    def add_service_ip(self, iface, address):
        # A model that let the same address sit on two interfaces would hide the
        # exact bug worth catching, so it refuses instead.
        holder = self.world.claimed.get(address)
        if holder is not None and holder != iface:
            raise AssertionError("%s claimed on %s while still on %s"
                                 % (address, iface, holder))
        self.world.claimed[address] = iface
        return True

    def del_service_ip(self, iface, address):
        if self.world.claimed.get(address) == iface:
            del self.world.claimed[address]
        return True

    def set_default(self, iface, gateway, src=None, metric=50):
        self.world.log.append("route: default via %s dev %s src %s"
                              % (gateway, iface, src))
        return True

    def set_interface_table(self, iface, address, table):
        self.world.rules[address] = table
        self.world.tables[table] = iface
        return True

    def clear_interface_table(self, address, table):
        # The real one flushes the table and leaves any other rule pointing at
        # it, which then matches nothing and falls through to the main table.
        # Modelled the same way, so a rule left behind fails here too.
        self.world.rules.pop(address, None)
        self.world.tables.pop(table, None)
        return True

    def set_source_rule(self, address, table):
        self.world.rules[address] = table
        return True

    def clear_source_rule(self, address, table):
        if self.world.rules.get(address) == table:
            del self.world.rules[address]
        return True

    def sysctl(self, key, value):
        self.world.log.append("sysctl: %s = %s" % (key, value))
        if "rfkill" in key:
            self.world.rfkill_asked.append(key)
        return True

    # --- ICMP and ARP -------------------------------------------------------
    def ping(self, pairs, timeout=1.0):
        results = {}
        for iface, _target in pairs:
            radio = self.world.radios.get(iface)
            results[iface] = None
            if radio is None or not radio.present or not radio.ssid:
                continue
            ap = self.world.ap(radio.ssid)
            if ap is None or not ap.up or not ap.reaches_lan:
                continue
            if not radio.address:
                continue
            # Loss is deterministic on the tick number rather than random, so a
            # scenario that fails does so the same way twice.
            if ap.loss_pct > 0:
                bucket = int(self.world.t) % 100
                if bucket < ap.loss_pct:
                    continue
            results[iface] = ap.latency_ms
        return results

    def garp(self, iface, address, times=3):
        self.garps.append((round(self.world.t, 1), iface, address))
        self.world.arp_owner[address] = self.mac(iface)
        return True

    def arp_probe(self, iface, address, timeout=0.6):
        owner = self.world.arp_owner.get(address)
        mine = self.mac(iface)
        others = {radio.mac for radio in self.world.radios.values()}
        if owner and owner != mine and owner not in others:
            return owner
        return None


# --- the recording -----------------------------------------------------------

def parse_netwatch(path, limit=None):
    """`netwatch` records into dictionaries, keeping the ones with a link in.

    The format is `key=value` separated by spaces, one record per line, and the
    `kind` says what sort of record it is. Only `sample` and `change` carry a
    link reading; `wpa` and `kmsg` carry the supplicant's and the kernel's own
    accounts, which are kept because the disconnection reason is the thing that
    says a fault was a handover rather than a radio failing.
    """
    records = []
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            fields = {}
            for token in line.split():
                key, sep, value = token.partition("=")
                if sep:
                    fields[key] = value
            if not fields.get("kind"):
                continue
            records.append(fields)
            if limit and len(records) >= limit:
                break
    return records


def _number(fields, key):
    try:
        return float(fields[key])
    except (KeyError, TypeError, ValueError):
        return None


class Trace:
    """One recorded run of the rover's real radio, as a sequence of readings.

    Each entry is what a single radio was doing at that moment -- associated or
    not, at what signal, answering the gateway in how many milliseconds -- taken
    from what the rover actually wrote down rather than from any model. Feeding
    these through the manager's own scoring and comparing its verdict with the
    verdict `netwatch` recorded is the calibration; see
    :func:`replay_health`.
    """

    def __init__(self, records):
        self.samples = []
        self.events = []
        for fields in records:
            kind = fields.get("kind")
            if kind in ("wpa", "kmsg"):
                self.events.append((fields.get("t"), fields.get("ev")
                                    or fields.get("msg") or ""))
                continue
            if kind not in ("sample", "change"):
                continue
            self.samples.append({
                "t": fields.get("t"),
                "state": fields.get("state"),
                "ssid": fields.get("ssid"),
                "bssid": fields.get("bssid"),
                "freq": _number(fields, "freq"),
                "dbm": _number(fields, "sig"),
                "rtt_ms": _number(fields, "rtt"),
                "address": fields.get("ip"),
                "wpa": fields.get("wpa"),
            })

    def __len__(self):
        return len(self.samples)

    def window(self, first, last):
        return Trace.from_samples(self.samples[first:last])

    @classmethod
    def from_samples(cls, samples):
        trace = cls([])
        trace.samples = list(samples)
        return trace


def recorded_usable(sample):
    """The rover's own verdict on whether that link was carrying traffic.

    Not `state=` on its own, which is worth being careful about because it is
    the obvious thing to compare against and it is the wrong question.
    `netwatch` writes `state` straight out of `/sys/class/net/<iface>/operstate`,
    so it is the link-layer answer and says nothing at all about whether
    anything upstream replied. Its README is explicit that association, an
    address, a route and a gateway that answers are four separate facts and that
    the interesting failures are the ones where three of them hold.

    So the comparison is against the conjunction the rover actually recorded:
    associated, addressed, and the gateway answering the single ping taken with
    that sample. That is the same question :attr:`wifi_dual.Radio.usable` asks,
    and comparing it with `state` alone was scoring the model against a verdict
    it was never trying to reproduce.
    """
    return bool(sample["state"] == "up" and sample["address"]
                and sample["rtt_ms"] is not None)


def replay_health(trace):
    """Does the manager's grading agree with what the rover recorded?

    This is the whole of the validation, and it is deliberately a narrow
    question. For every sample in the recording it builds the radio state that
    sample describes, asks :class:`wifi_dual.Radio` whether that is a link worth
    carrying traffic on, and compares the answer with :func:`recorded_usable`. A
    model that says a link is fine where the rover found it carrying nothing --
    or the reverse -- has no business judging a fix.

    Returns (agreed, total, disagreements).
    """
    agreed = 0
    total = 0
    wrong = []
    for sample in trace.samples:
        if sample["state"] not in ("up", "down", "dormant"):
            continue
        radio = wifi_dual.Radio("wlan0")
        radio.link = Link(sample["ssid"], sample["bssid"],
                          None if sample["freq"] is None else int(sample["freq"]),
                          None if sample["dbm"] is None else int(sample["dbm"]))
        radio.held_dbm = wifi_dual.usable_dbm(sample["dbm"])
        radio.operstate = sample["state"]
        radio.address = sample["address"]
        radio.pings = [sample["rtt_ms"]] if sample["rtt_ms"] else [None]
        total += 1
        if radio.usable == recorded_usable(sample):
            agreed += 1
        else:
            wrong.append((sample["t"], sample["state"], radio.usable,
                          sample["dbm"], sample["rtt_ms"], sample["address"]))
    return agreed, total, wrong


class TraceRadio(SimRadio):
    """A modelled radio whose signal and latency come out of the recording."""

    def __init__(self, iface, trace, **kwargs):
        super().__init__(iface, **kwargs)
        self.trace = trace
        self.index = 0


class TraceWorld(World):
    """The recorded run, with a second radio added that was not there.

    This is the counterfactual the whole design is an argument for, and it is
    built so that it cannot flatter itself: the recorded radio's signal, latency
    and association are replayed exactly as the rover wrote them down, and only
    the second radio is modelled. If two radios would not have covered this
    outage, this is where that shows up.
    """

    def __init__(self, trace, second, seconds_per_sample=10.0, **kwargs):
        super().__init__(**kwargs)
        self.trace = trace
        self.seconds_per_sample = seconds_per_sample
        self.second = second

    def sample_at(self, when):
        index = int(when // self.seconds_per_sample)
        index = max(0, min(index, len(self.trace.samples) - 1))
        return self.trace.samples[index]


class TracePlatform(SimPlatform):
    """`SimPlatform`, with one interface answered from the recording."""

    def __init__(self, world, replayed="wlan0", **kwargs):
        super().__init__(world, **kwargs)
        self.replayed = replayed

    def operstate(self, iface):
        if iface != self.replayed:
            return super().operstate(iface)
        # The recorded operstate, verbatim. Deriving it from whether an SSID
        # was written down would invent a disassociation on the 59 samples where
        # `iw` named no network on a link that was pinging in two milliseconds.
        return self.world.sample_at(self.world.t)["state"] or "down"

    def link(self, iface):
        if iface != self.replayed:
            return super().link(iface)
        sample = self.world.sample_at(self.world.t)
        if not sample["ssid"]:
            return Link()
        return Link(sample["ssid"], sample["bssid"],
                    None if sample["freq"] is None else int(sample["freq"]),
                    None if sample["dbm"] is None else int(sample["dbm"]))

    def ipv4(self, iface):
        if iface != self.replayed:
            return super().ipv4(iface)
        return self.world.sample_at(self.world.t)["address"]

    def ping(self, pairs, timeout=1.0):
        results = super().ping([pair for pair in pairs
                                if pair[0] != self.replayed], timeout=timeout)
        for iface, _target in pairs:
            if iface != self.replayed:
                continue
            sample = self.world.sample_at(self.world.t)
            results[iface] = sample["rtt_ms"]
        return results

    def scan(self, iface):
        if iface == self.replayed:
            # The recording cannot say what the radio would have heard, and
            # inventing it would be the model flattering itself. The replayed
            # radio is never the standby in these scenarios anyway.
            return []
        return super().scan(iface)

    def pin(self, iface, ssid):
        if iface == self.replayed:
            # The recording is what it is; the manager may hold an intent
            # against this radio but cannot move it.
            self.pins.append((round(self.world.t, 1), iface, ssid))
            return True
        return super().pin(iface, ssid)


# --- a house, ready made ------------------------------------------------------

def house(**kwargs):
    """The three real routers, at distances that reproduce the real readings.

    TheGreatLord is the one the rover is beside today: its 5 GHz radio reads -30
    dBm and its 2.4 GHz radio -40, which is what the board reported on
    2026-08-25. TheMaharaja read -72 on the dongle from the same spot, and
    TheGreatViking is placed where the 2026-08-24 recording found it -- audible
    at -78 and fading.
    """
    aps = [
        AccessPoint("TheGreatLord", 2.4, 0.0, freq=2427),
        AccessPoint("TheGreatLord 5G", 2.4, 0.0, freq=5805),
        AccessPoint("TheMaharaja", 33.0, 0.0, freq=2437),
        AccessPoint("TheMaharaja 5G", 33.0, 0.0, freq=5220),
        AccessPoint("TheGreatViking", 0.0, 55.0, freq=2437),
        AccessPoint("TheGreatViking 5G", 0.0, 55.0, freq=5180),
    ]
    known = [ap.ssid for ap in aps]
    radios = [
        SimRadio("wlan0", usb=False, mac="ac:6a:a3:41:53:53",
                 bands=("2.4", "5"), knows=known),
        SimRadio("wlan1", usb=True, mac="00:2e:2d:30:74:d0",
                 bands=("2.4",), knows=known),
    ]
    return World(aps, radios, **kwargs)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--replay", help="a netwatch log to score against")
    parser.add_argument("--dual", action="store_true",
                        help="also run two radios over the same recording")
    parser.add_argument("--from", dest="first", type=int, default=0)
    parser.add_argument("--to", dest="last", type=int, default=0)
    args = parser.parse_args(argv)

    if not args.replay:
        parser.error("nothing to do without --replay; the scenarios live in "
                     "test_wifi_dual.py")

    trace = Trace(parse_netwatch(args.replay))
    if args.last:
        trace = trace.window(args.first, args.last)
    agreed, total, wrong = replay_health(trace)
    print("%d of %d samples graded the way the rover recorded them (%.1f%%)"
          % (agreed, total, 100.0 * agreed / max(1, total)))
    for entry in wrong[:10]:
        print("  disagreed at %s: recorded %s, model says usable=%s "
              "(sig %s, rtt %s, ip %s)" % entry)
    if not args.dual:
        return 0 if agreed == total else 1

    result = replay_dual(trace, quiet=False)
    print("recording:  %5.0f s carrying nothing, out of %.0f"
          % (result["recorded_dark_s"], result["span_s"]))
    print("two radios: %5.0f s carrying nothing, %d failover(s)"
          % (result["dark_s"], result["switches"]))
    for entry in result["history"]:
        print("  moved %s -> %s: %s" % (entry["from"], entry["to"], entry["why"]))
    return 0


def replay_dual(trace, standby_on="TheGreatLord", quiet=True, rover=(0.0, 0.0)):
    """Run the manager over a recording, with the second radio it never had.

    The recorded radio is replayed exactly: its signal, its round trip, its
    association and its address are read straight out of what the rover wrote
    down, and the manager cannot move it. Only the standby is modelled, and it
    is put on the access point the rover itself re-associated with when the link
    finally broke -- so the counterfactual is not "what if there had been a
    better network", it is "what if the network the rover found by hand had
    already been associated".

    Returns how many seconds each arrangement spent carrying nothing.
    """
    aps = house().aps
    known = [ap.ssid for ap in aps]
    replayed = SimRadio("wlan0", usb=False, mac="ac:6a:a3:41:53:53",
                        knows=known)
    spare = SimRadio("wlan1", usb=True, mac="00:2e:2d:30:74:d0",
                     bands=("2.4",), knows=known)
    spare.ssid = standby_on
    spare.address = "192.168.1.144"
    world = TraceWorld(trace, second="wlan1", aps=aps,
                       radios=[replayed, spare], x=rover[0], y=rover[1])
    platform = TracePlatform(world, replayed="wlan0", quiet=quiet)
    manager = wifi_dual.Manager(platform, status_path=None)

    span = len(trace) * 10.0
    dark = 0.0
    while world.t < span:
        manager.tick()
        if manager.active is None or not manager.active.usable:
            dark += 1.0
        world.advance(1.0)
    recorded_dark = sum(10.0 for sample in trace.samples
                        if not recorded_usable(sample))
    return {"span_s": span, "recorded_dark_s": recorded_dark, "dark_s": dark,
            "switches": manager.switches, "history": list(manager.history),
            "garps": list(platform.garps), "log": list(world.log)}


if __name__ == "__main__":
    sys.exit(main())
