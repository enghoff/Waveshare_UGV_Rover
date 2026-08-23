#!/usr/bin/env python3
"""Why the rover fell off the network, and what it was doing at the time.

    sudo netwatch.py                  # the service: sample, follow, append
    sudo netwatch.py --once           # one sample to stdout, write nothing
    netwatch_report.py                # read the log back as episodes

A rover that has to be power-cycled to come back has already destroyed the
evidence for why. This is the thing that writes the evidence down first, and it
is deliberately the dullest process on the board: it opens no camera, holds no
serial port, spawns nothing on the common path, and cannot move the link it is
watching. It only ever answers questions.

Three questions, specifically, because they need different evidence and the
first two look identical from a desk:

* **Was the board up?** A boot record with no `stop` record before it means the
  board went down without being asked to -- a hard reset or a power cut, not a
  `reboot`. The last sample before it is then what it was doing while it died.
* **Was the radio associated?** `wlan0` can be up, associated and holding an
  address while nothing on the LAN can reach it. Association, address, default
  route and a gateway that answers are four separate facts and this records all
  four, because the fault that matters is usually the gap between two of them.
* **What did the driver say?** wpa_supplicant announces every disconnection with
  a reason code and the kernel announces every USB fault, and both scroll out of
  a 20 MB journal in a day. They are copied here beside the samples so the
  answer is one file rather than a correlation exercise.

**Nothing here is written to /var/log.** Armbian mounts that as a zram ramlog
and syncs it to disk on a schedule, so the last minutes before a hard reset --
the only minutes that matter -- are exactly what it loses. This writes to
`/var/lib/netwatch/`, on the SD card, and calls `fsync` on every transition and
every heartbeat, because the root filesystem here is mounted `commit=120` and
would otherwise hold two minutes of samples in RAM for the reset to take.
"""

import argparse
import errno
import glob
import os
import re
import select
import signal
import socket
import struct
import subprocess
import sys
import time

# The interface address is an ioctl, and there is no fcntl on Windows. Importing
# this on a desk is worth keeping possible even so: `selftest.py` drives every
# parser in here against a fake /proc, and a self-test that only runs on the
# machine it is meant to protect is a self-test nobody runs.
try:
    import fcntl
except ImportError:
    fcntl = None

IFACE = os.environ.get("NETWATCH_IFACE", "wlan0")
LOGDIR = os.environ.get("NETWATCH_DIR", "/var/lib/netwatch")
LOGNAME = "netwatch.log"

# Ten seconds of resolution is enough to see a link fail and cheap enough to run
# for weeks: a sample is about 250 bytes, so a day is under 2 MB. The heartbeat
# is what bounds how much a hard reset can take with it -- everything since the
# last fsync -- and 30 s of samples is a fair trade against three fsyncs a
# minute on an SD card. Transitions do not wait for it; they sync as they land.
SAMPLE_S = float(os.environ.get("NETWATCH_SAMPLE", 10))
FSYNC_S = float(os.environ.get("NETWATCH_FSYNC", 30))
PROBE_S = float(os.environ.get("NETWATCH_PROBE", 30))
ROTATE_BYTES = int(os.environ.get("NETWATCH_ROTATE", 16 * 1024 * 1024))
KEEP = 2

# The gateway is the cheapest thing on the LAN that always answers and is not
# this repository's own code. Pinging the workstation instead would make "the
# desk is asleep" look like "the rover is off the air".
GATEWAY = os.environ.get("NETWATCH_GATEWAY", "192.168.1.1")

WPA_CTRL = os.environ.get("NETWATCH_WPA", "/run/wpa_supplicant")

PROC = os.environ.get("NETWATCH_PROC", "/proc")
SYS = os.environ.get("NETWATCH_SYS", "/sys")

# Kernel lines worth keeping. The USB tree is the first suspect on this board --
# the wifi dongle, the camera, the lidar and the OAK share one weakly fused bus --
# so every disconnect, reset and enumeration on it is evidence, as is anything
# the wifi driver says about itself and anything the kernel says about running
# out of memory or being late.
KMSG_KEEP = re.compile(
    r"usb|rtl8|wlan|cfg80211|ieee80211|voltage|undervolt|oom|Out of memory|"
    r"watchdog|hung task|thermal|brownout|xhci|ehci|dwc",
    re.I,
)


def _read(path, default=""):
    """A file, or a default. Every input here is /proc or /sys and may vanish."""
    try:
        with open(path) as fh:
            return fh.read()
    except OSError:
        return default


def _read_int(path, default=-1):
    try:
        return int(_read(path).strip())
    except ValueError:
        return default


class Link:
    """Everything about wlan0 that can be had without spawning a process."""

    def __init__(self, iface=IFACE):
        self.iface = iface
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def operstate(self):
        # operstate rather than carrier, which is the more obvious question and
        # is unreadable on an interface that is not up -- see wifi_roam/README.
        return _read(SYS + "/class/net/" + self.iface + "/operstate", "gone").strip()

    def address(self):
        """The IPv4 address, by ioctl rather than by parsing `ip`."""
        if fcntl is None:
            return ""
        try:
            packed = fcntl.ioctl(
                self._sock.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack("256s", self.iface.encode()[:15]),
            )
            return socket.inet_ntoa(packed[20:24])
        except OSError:
            return ""

    def default_route(self):
        """The gateway this interface has a default route to, or ''."""
        for line in _read(PROC + "/net/route").splitlines()[1:]:
            f = line.split()
            if len(f) > 2 and f[0] == self.iface and f[1] == "00000000":
                return socket.inet_ntoa(struct.pack("<I", int(f[2], 16)))
        return ""

    def wireless(self):
        """Signal and the driver's own discard counters, from /proc/net/wireless.

        The level column carries -256 when this dongle has nothing to report,
        which is not a signal 200 dB down and must not be averaged with the ones
        that are. It comes back as None so a caller cannot accidentally do
        arithmetic on it.
        """
        out = {"qual": None, "sig": None, "noise": None, "misc": None, "beacon": None}
        for line in _read(PROC + "/net/wireless").splitlines():
            if not line.strip().startswith(self.iface + ":"):
                continue
            f = line.split(":", 1)[1].split()
            if len(f) < 10:
                continue

            def num(tok):
                try:
                    v = int(float(tok.rstrip(".")))
                except ValueError:
                    return None
                return None if v <= -110 else v

            out["qual"] = num(f[1])
            out["sig"] = num(f[2])
            out["noise"] = num(f[3])
            out["misc"] = num(f[8])
            out["beacon"] = num(f[9])
        return out

    def counters(self):
        base = SYS + "/class/net/" + self.iface + "/statistics"
        keys = ("rx_bytes", "tx_bytes", "rx_packets", "tx_packets",
                "rx_dropped", "tx_dropped", "rx_errors", "tx_errors")
        return dict((k, _read_int(base + "/" + k, 0)) for k in keys)


class Wpa:
    """wpa_supplicant's control socket, spoken directly.

    `wpa_cli` is on this board but a subprocess three times a minute is not, and
    the events are the point: an association that ends says why it ended exactly
    once, in an unsolicited message, and nothing polls that back afterwards.

    Two sockets, the way wpa_cli itself does it -- one for commands and one
    ATTACHed for events -- because a reply and an unsolicited message arriving on
    one socket cannot be told apart without guessing.
    """

    def __init__(self, iface=IFACE, ctrl=WPA_CTRL):
        self.path = os.path.join(ctrl, iface)
        self.cmd = None
        self.evt = None
        self.attached = False
        self.why = ""
        self._cmd_path = self._evt_path = None

    def _open(self, tag):
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        local = "/tmp/netwatch-%s-%d" % (tag, os.getpid())
        try:
            os.unlink(local)
        except OSError:
            pass
        sock.bind(local)
        sock.connect(self.path)
        sock.settimeout(1.0)
        return sock, local

    def connect(self):
        """Attach, or say why not. Failure here is not fatal: the /proc half of
        the sampler works without wpa_supplicant, and a supplicant that is being
        restarted is exactly when the rest of the record matters most."""
        self.close()
        try:
            self.cmd, self._cmd_path = self._open("cmd")
            self.evt, self._evt_path = self._open("evt")
            self.evt.send(b"ATTACH")
            self.attached = self.evt.recv(64).startswith(b"OK")
            return self.attached
        except OSError as exc:
            self.why = str(exc)
            self.close()
            return False

    def close(self):
        for sock, path in ((self.cmd, self._cmd_path), (self.evt, self._evt_path)):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
            if path:
                try:
                    os.unlink(path)
                except OSError:
                    pass
        self.cmd = self.evt = None
        self._cmd_path = self._evt_path = None
        self.attached = False

    def status(self):
        """ssid, bssid and frequency, or {} if the supplicant is not talking."""
        if self.cmd is None:
            return {}
        try:
            self.cmd.settimeout(1.0)
            self.cmd.send(b"STATUS")
            raw = self.cmd.recv(4096).decode("utf-8", "replace")
        except OSError:
            return {}
        out = {}
        for line in raw.splitlines():
            if "=" in line:
                k, v = line.split("=", 1)
                out[k] = v
        return out

    def events(self):
        """Whatever the supplicant has said since last asked. Never blocks."""
        got = []
        if self.evt is None:
            return got
        while True:
            try:
                self.evt.settimeout(0)
                raw = self.evt.recv(4096)
            except (BlockingIOError, socket.timeout):
                break
            except OSError:
                self.attached = False
                break
            text = raw.decode("utf-8", "replace").strip()
            # <3>CTRL-EVENT-DISCONNECTED bssid=.. reason=3 ..
            if text.startswith("<"):
                text = text.split(">", 1)[-1]
            if text:
                got.append(text)
        return got


class Kmsg:
    """New kernel messages, filtered. Reopened if the buffer laps us."""

    def __init__(self):
        self.fd = None
        self.open()

    def open(self):
        self.close()
        try:
            self.fd = os.open("/dev/kmsg", os.O_RDONLY | os.O_NONBLOCK)
            os.lseek(self.fd, 0, os.SEEK_END)   # only what happens from now on
        except OSError:
            self.fd = None

    def close(self):
        if self.fd is not None:
            try:
                os.close(self.fd)
            except OSError:
                pass
        self.fd = None

    def read(self):
        out = []
        if self.fd is None:
            return out
        while True:
            try:
                raw = os.read(self.fd, 8192)
            except OSError as exc:
                if exc.errno == errno.EAGAIN:
                    break
                if exc.errno == errno.EPIPE:
                    # The ring lapped while we were not looking. Nothing to
                    # recover; say so rather than silently skipping.
                    out.append("netwatch: kmsg ring overran, messages lost")
                    self.open()
                    continue
                self.close()
                break
            if not raw:
                break
            text = raw.decode("utf-8", "replace")
            body = text.split(";", 1)[-1].split("\n", 1)[0].strip()
            if body and KMSG_KEEP.search(body):
                out.append(body)
        return out


class Cpu:
    """Busy fraction per core between two calls, and the board's temperature."""

    def __init__(self):
        self.prev = self._raw()

    def _raw(self):
        out = []
        for line in _read(PROC + "/stat").splitlines():
            if line.startswith("cpu") and line[3:4].isdigit():
                f = [int(x) for x in line.split()[1:]]
                out.append((sum(f), f[3] + (f[4] if len(f) > 4 else 0)))
        return out

    def busy(self):
        now = self._raw()
        out = []
        for (t1, i1), (t0, i0) in zip(now, self.prev):
            dt, di = t1 - t0, i1 - i0
            out.append(0 if dt <= 0 else max(0, min(100, round(100 * (dt - di) / dt))))
        self.prev = now
        return out

    @staticmethod
    def temp_c():
        milli = _read_int(SYS + "/class/thermal/thermal_zone0/temp", -1)
        return round(milli / 1000.0, 1) if milli > 0 else None

    @staticmethod
    def mhz():
        khz = _read_int(SYS + "/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", -1)
        return khz // 1000 if khz > 0 else None


def mem_available_mb():
    for line in _read(PROC + "/meminfo").splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) // 1024
    return -1


def usb_devices():
    """How many devices are on the USB tree. A dongle that has fallen off the
    bus is a different fault from a dongle that cannot associate, and this is
    the one number that tells them apart without root or lsusb."""
    return len(glob.glob(SYS + "/bus/usb/devices/*-*"))


ROVER_PROCS = (("daemon", "rover_daemon.py"),
               ("oak", "depth_server.py"),
               ("web", "drive_web.py"))


def rover_procs():
    """Which of the rover's three long-lived processes are alive, by cmdline.

    Worth a couple of hundred small reads every ten seconds because the load
    hypothesis needs to know whether the board was doing its actual job at the
    time, and because a daemon that died an hour before the network did is a
    different story entirely.
    """
    alive = dict((name, 0) for name, _ in ROVER_PROCS)
    try:
        entries = os.listdir(PROC)
    except OSError:
        return alive
    for entry in entries:
        if not entry.isdigit():
            continue
        cmd = _read(PROC + "/" + entry + "/cmdline").replace("\0", " ")
        for name, needle in ROVER_PROCS:
            if needle in cmd:
                alive[name] = 1
    return alive


def ping(host, timeout=1):
    """Round trip to the gateway in ms, or None. The one spawn in this program.

    It answers the question none of /proc can: whether a link that looks
    perfectly associated actually carries a packet. That is the failure this
    rover keeps having -- ssh connecting and never sending a banner -- and from
    the inside it is invisible in every file above.
    """
    try:
        out = subprocess.run(
            ["ping", "-n", "-c", "1", "-W", str(timeout), host],
            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            timeout=timeout + 2, text=True,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    m = re.search(r"time=([\d.]+) ms", out.stdout)
    return round(float(m.group(1)), 1) if m else None


def boot_id():
    return _read(PROC + "/sys/kernel/random/boot_id").strip()[:8]


def uptime_s():
    try:
        return round(float(_read(PROC + "/uptime").split()[0]), 1)
    except (ValueError, IndexError):
        return -1.0


def fmt(kind, fields):
    """One record, one line. `key=value`, space separated, values never spaced.

    Chosen over JSON because this file is read by a person on a slow ssh session
    at least as often as by the report tool, and over CSV because the fields
    that matter change with the record kind.
    """
    parts = ["t=" + time.strftime("%Y-%m-%dT%H:%M:%S"), "kind=" + kind]
    for k, v in fields.items():
        if v is None or v == "":
            continue
        if isinstance(v, float):
            v = "%g" % v
        v = str(v).replace(" ", "_")
        parts.append("%s=%s" % (k, v))
    return " ".join(parts)


class Log:
    """Append-only, rotated, and synced when it matters.

    The two things this has to survive are the two this board does: a hard reset
    that takes unsynced writes with it, and a filesystem filling up over weeks of
    running. Hence fsync on transitions and rotation at a fixed size.
    """

    def __init__(self, directory=LOGDIR, name=LOGNAME):
        self.dir = directory
        self.path = os.path.join(directory, name)
        os.makedirs(directory, exist_ok=True)
        self.fh = open(self.path, "a", buffering=1)
        self.dirty = False
        self.last_sync = time.monotonic()

    def write(self, line, sync=False):
        self.fh.write(line + "\n")
        self.dirty = True
        if sync:
            self.sync()

    def sync(self):
        if not self.dirty:
            return
        self.fh.flush()
        os.fsync(self.fh.fileno())
        self.dirty = False
        self.last_sync = time.monotonic()

    def maybe_rotate(self):
        try:
            if self.fh.tell() < ROTATE_BYTES:
                return
        except OSError:
            return
        self.sync()
        self.fh.close()
        for n in range(KEEP, 0, -1):
            older = "%s.%d" % (self.path, n)
            newer = "%s.%d" % (self.path, n - 1) if n > 1 else self.path
            if os.path.exists(newer):
                os.replace(newer, older)
        self.fh = open(self.path, "a", buffering=1)

    def tail(self, limit=64 * 1024):
        """The end of the previous run's log, for classifying how it ended."""
        try:
            with open(self.path, "rb") as fh:
                fh.seek(0, os.SEEK_END)
                fh.seek(max(0, fh.tell() - limit))
                return fh.read().decode("utf-8", "replace").splitlines()
        except OSError:
            return []


def classify_previous(lines):
    """Did the last run end because somebody asked, or because the board died?

    This is the whole reason the service writes a `stop` record. A `reboot`, a
    `systemctl stop` and a shutdown all deliver SIGTERM and get one. A hard
    reset, a brownout and a wedged kernel do not, so a log whose last word is an
    ordinary sample is a log describing a board that was still talking when the
    power went. The last sample is then the most useful line in the file, and is
    handed back with the verdict.
    """
    last_stop = last_any = None
    for line in lines:
        if " kind=" not in line:
            continue
        last_any = line
        if " kind=stop" in line:
            last_stop = line
    if last_any is None:
        return "unknown", None
    if last_stop is not None and last_stop == last_any:
        return "clean", last_any
    return "hard", last_any


def sample(link, wpa, cpu, prev_counters):
    """One row of everything cheap, plus whatever the supplicant will say."""
    st = wpa.status() if wpa.attached else {}
    w = link.wireless()
    counters = link.counters()
    busy = cpu.busy()
    loadavg = _read(PROC + "/loadavg").split()
    fields = {
        "up": uptime_s(),
        "state": link.operstate(),
        "ip": link.address(),
        "gw": link.default_route(),
        "ssid": st.get("ssid", ""),
        "bssid": st.get("bssid", ""),
        "freq": st.get("freq", ""),
        "wpa": st.get("wpa_state", "" if wpa.attached else "noctrl"),
        "sig": w["sig"],
        "qual": w["qual"],
        "misc": w["misc"],
        "beacon": w["beacon"],
        "load": loadavg[0] if loadavg else "",
        "cpu": ",".join(str(b) for b in busy),
        "mhz": cpu.mhz(),
        "temp": cpu.temp_c(),
        "memfree": mem_available_mb(),
        "usbn": usb_devices(),
    }
    for key, short in (("rx_bytes", "rxkb"), ("tx_bytes", "txkb")):
        delta = counters[key] - prev_counters.get(key, counters[key])
        fields[short] = max(0, delta) // 1024
    for key in ("rx_dropped", "rx_errors", "tx_errors"):
        if counters[key]:
            fields[key.replace("_", "")] = counters[key]
    fields.update(rover_procs())
    return fields, counters


def key_state(fields):
    """The part of a sample worth a line of its own the moment it changes.

    The console is deliberately not in here. `drive_web.py` is started on demand
    and restarted by its supervisor on a fifteen-second timer, so it comes and
    goes in normal operation, and putting it in this tuple made every ordinary
    reload a synced `change` record. The daemon and the depth server are in,
    because neither of those two is supposed to go anywhere.
    """
    return (fields.get("state"), fields.get("ip"), fields.get("gw"),
            fields.get("ssid"), fields.get("bssid"), fields.get("wpa"),
            fields.get("usbn"), fields.get("daemon"), fields.get("oak"))


def run(args):
    link, wpa, cpu, kmsg = Link(), Wpa(), Cpu(), Kmsg()
    log = Log(args.dir)

    verdict, last_line = classify_previous(log.tail())
    wpa.connect()
    log.write(fmt("boot", {
        "boot": boot_id(),
        "up": uptime_s(),
        "prev": verdict,
        "kernel": _read(PROC + "/sys/kernel/osrelease").strip(),
        "wpactrl": 1 if wpa.attached else 0,
        "sample_s": SAMPLE_S,
    }), sync=True)
    if verdict == "hard" and last_line:
        # Quoted rather than parsed: the point is to put the board's last words
        # next to the verdict, in the same file, so nobody has to go looking.
        log.write(fmt("lastwords", {"line": last_line.replace(" ", "|")}), sync=True)

    stopping = {"now": False}

    def on_term(_signum, _frame):
        stopping["now"] = True

    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    prev_counters = link.counters()
    prev_key = None
    prev_wall = time.time()
    prev_mono = time.monotonic()
    next_sample = 0.0
    next_probe = 0.0
    rtt = None

    while not stopping["now"]:
        # The loop sleeps on the event socket rather than on the clock, so a
        # disconnection is written the moment the supplicant says it rather than
        # up to ten seconds later, which is the difference between "the link
        # went and then the load rose" and the other way round.
        waits = [fd for fd in (wpa.evt.fileno() if wpa.evt else None, kmsg.fd)
                 if fd is not None]
        timeout = max(0.2, min(next_sample - time.monotonic(), 2.0))
        if waits:
            try:
                select.select(waits, [], [], timeout)
            except OSError:
                time.sleep(timeout)
        else:
            time.sleep(timeout)

        for event in wpa.events():
            log.write(fmt("wpa", {"ev": event.replace(" ", "_")}), sync=True)
        for line in kmsg.read():
            log.write(fmt("kmsg", {"msg": line.replace(" ", "_")}), sync=True)

        # A board with no RTC boots at whatever fake-hwclock last wrote and jumps
        # when NTP answers, so every wall-clock timestamp before that jump is
        # wrong. Saying so once is what stops an hour of samples looking like an
        # hour-long outage in the report.
        now_mono, now_wall = time.monotonic(), time.time()
        drift = (now_wall - prev_wall) - (now_mono - prev_mono)
        if abs(drift) > 5:
            log.write(fmt("clock", {"jump_s": round(drift, 1),
                                    "to": time.strftime("%Y-%m-%dT%H:%M:%S")}),
                      sync=True)
        prev_wall, prev_mono = now_wall, now_mono

        if now_mono >= next_probe:
            next_probe = now_mono + PROBE_S
            rtt = ping(args.gateway)

        if now_mono < next_sample:
            if log.dirty and now_mono - log.last_sync >= FSYNC_S:
                log.sync()
            continue
        next_sample = now_mono + SAMPLE_S

        if not wpa.attached:
            wpa.connect()          # cheap, and the supplicant does get restarted
        fields, prev_counters = sample(link, wpa, cpu, prev_counters)
        fields["rtt"] = rtt if rtt is not None else "none"

        this_key = key_state(fields)
        changed = prev_key is not None and this_key != prev_key
        prev_key = this_key
        log.write(fmt("change" if changed else "sample", fields), sync=changed)
        log.maybe_rotate()
        if log.dirty and time.monotonic() - log.last_sync >= FSYNC_S:
            log.sync()

    log.write(fmt("stop", {"up": uptime_s(), "why": "signal"}), sync=True)
    os.system("sync")
    wpa.close()
    kmsg.close()
    return 0


def once(args):
    link, wpa, cpu = Link(), Wpa(), Cpu()
    wpa.connect()
    time.sleep(0.2)
    fields, _ = sample(link, wpa, cpu, link.counters())
    rtt = ping(args.gateway)
    fields["rtt"] = rtt if rtt is not None else "none"
    print(fmt("sample", fields))
    if not wpa.attached:
        print("# no wpa_supplicant control socket (%s): %s"
              % (wpa.path, wpa.why or "not root?"), file=sys.stderr)
    wpa.close()
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--once", action="store_true", help="one sample to stdout")
    p.add_argument("--dir", default=LOGDIR, help="where the log lives")
    p.add_argument("--gateway", default=GATEWAY, help="what to ping")
    args = p.parse_args(argv)
    if args.once:
        return once(args)
    return run(args)


if __name__ == "__main__":
    sys.exit(main())
