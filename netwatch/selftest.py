#!/usr/bin/env python3
"""Every parser in netwatch, against a fake /proc. Runs anywhere, needs no radio.

    python3 selftest.py

The interesting branches here are the ones that only happen when something has
gone wrong -- a dongle that has fallen off the bus, an interface that is not up,
a driver reporting a signal it does not have -- and waiting for a real rover to
go wrong is not a test strategy. So the whole of /proc and /sys that this program
reads is a directory of files written by this file, and the failures are made to
order.

Two of the scenarios are here because they are the ones that would quietly turn
this instrument into a liar. A `-256` in the level column is the dongle saying it
has no reading, not a link 200 dB down, and averaging it into a report would
manufacture an outage that never happened. And a log whose last record is an
ordinary sample means the board died mid-sentence, which is the single fact this
whole exercise exists to establish -- if that came out as "clean" the report
would say the rover had been shut down politely every time it fell over.
"""

import os
import shutil
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

ROOT = tempfile.mkdtemp(prefix="netwatch-selftest-")
os.environ["NETWATCH_PROC"] = os.path.join(ROOT, "proc")
os.environ["NETWATCH_SYS"] = os.path.join(ROOT, "sys")

import netwatch          # noqa: E402  -- after the environment, on purpose
import netwatch_report   # noqa: E402

PROC = os.environ["NETWATCH_PROC"]
SYS = os.environ["NETWATCH_SYS"]

PASSED = 0
FAILED = []


def check(name, got, want):
    global PASSED
    if got == want:
        PASSED += 1
    else:
        FAILED.append("%s: got %r, wanted %r" % (name, got, want))


def check_true(name, got):
    check(name, bool(got), True)


def write(path, text):
    full = os.path.join(ROOT, path)
    os.makedirs(os.path.dirname(full), exist_ok=True)
    with open(full, "w") as fh:
        fh.write(text)


WIRELESS_OK = (
    "Inter-| sta-|   Quality        |   Discarded packets\n"
    " face | tus | link level noise |  nwid  crypt   frag  retry   misc | beacon\n"
    " wlan0: 0000   70.  -38.  -256        0      0      0      0    707        0\n"
)
# The same dongle, mid-blink: the level column carries the no-value sentinel it
# keeps permanently in the noise column. Five of these in a row once carried the
# old rover off an association measuring -42 dBm.
WIRELESS_SENTINEL = WIRELESS_OK.replace("-38.", "-256")


def fake_board(state="up", route=True, wireless=WIRELESS_OK, usb=9, procs=True):
    """A whole board's worth of /proc and /sys, in whatever condition is wanted."""
    shutil.rmtree(ROOT, ignore_errors=True)
    write("proc/net/wireless", wireless)
    write("proc/net/route",
          "Iface\tDestination\tGateway\n"
          + ("wlan0\t00000000\t0101A8C0\t0003\n" if route else "")
          + "wlan0\t0001A8C0\t00000000\t0001\n")
    write("proc/uptime", "1834.75 7011.20\n")
    write("proc/loadavg", "0.49 0.44 0.66 1/158 5431\n")
    write("proc/meminfo", "MemTotal: 4013000 kB\nMemAvailable: 3651584 kB\n")
    write("proc/sys/kernel/osrelease", "6.18.44-current-sunxi64\n")
    write("proc/sys/kernel/random/boot_id", "3e6e58d6-85e6-4cfa-b691-4cfa5d266a81\n")
    write("proc/stat", "cpu  100 0 100 800 0 0 0\n"
                       "cpu0 25 0 25 200 0 0 0\ncpu1 25 0 25 200 0 0 0\n"
                       "cpu2 25 0 25 200 0 0 0\ncpu3 25 0 25 200 0 0 0\n")
    write("sys/class/net/wlan0/operstate", state + "\n")
    for key, value in (("rx_bytes", 8953384), ("tx_bytes", 56001423),
                       ("rx_packets", 33007), ("tx_packets", 51492),
                       ("rx_dropped", 22), ("tx_dropped", 0),
                       ("rx_errors", 0), ("tx_errors", 0)):
        write("sys/class/net/wlan0/statistics/" + key, str(value) + "\n")
    write("sys/class/thermal/thermal_zone0/temp", "48905\n")
    write("sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq", "1416000\n")
    for n in range(usb):
        write("sys/bus/usb/devices/3-%d/idVendor" % (n + 1), "0bda\n")
    if procs:
        write("proc/981/cmdline", "python3\0/home/admin/ugv/rover_daemon.py\0--vision\0")
        write("proc/982/cmdline", "python3\0/home/admin/ugv/oak_depth/depth_server.py\0")
    write("proc/1/cmdline", "/sbin/init\0")


class FakeSock:
    """A wpa_supplicant control socket that says whatever the test wants.

    Injected rather than opened, so this runs on a desk with no AF_UNIX and no
    supplicant -- the parsing is what is under test, not the kernel's sockets.
    """

    def __init__(self, messages):
        self.messages = list(messages)

    def settimeout(self, _t):
        pass

    def recv(self, _n):
        if not self.messages:
            raise BlockingIOError()
        return self.messages.pop(0)

    def fileno(self):
        return -1


def test_link():
    fake_board()
    link = netwatch.Link("wlan0")
    check("operstate up", link.operstate(), "up")
    check("default route", link.default_route(), "192.168.1.1")
    w = link.wireless()
    check("signal", w["sig"], -38)
    check("quality", w["qual"], 70)
    check("noise sentinel is not a number", w["noise"], None)
    check("discarded misc", w["misc"], 707)
    check("counters", link.counters()["rx_dropped"], 22)

    fake_board(wireless=WIRELESS_SENTINEL)
    check("level sentinel is not a signal", netwatch.Link("wlan0").wireless()["sig"], None)

    fake_board(state="down", route=False)
    link = netwatch.Link("wlan0")
    check("operstate down", link.operstate(), "down")
    check("no default route", link.default_route(), "")

    shutil.rmtree(ROOT, ignore_errors=True)
    link = netwatch.Link("wlan0")
    check("missing interface reads as gone", link.operstate(), "gone")
    check("missing /proc is not a crash", link.wireless()["sig"], None)


def test_board():
    fake_board(usb=9)
    check("usb devices", netwatch.usb_devices(), 9)
    check("uptime", netwatch.uptime_s(), 1834.8)
    check("memory", netwatch.mem_available_mb(), 3566)
    check("temperature", netwatch.Cpu.temp_c(), 48.9)
    check("clock", netwatch.Cpu.mhz(), 1416)
    alive = netwatch.rover_procs()
    check("daemon seen", alive["daemon"], 1)
    check("oak seen", alive["oak"], 1)
    check("console not running", alive["web"], 0)

    # The dongle falling off the bus is a different fault from the dongle failing
    # to associate, and this is the number that tells them apart.
    fake_board(usb=8, procs=False)
    check("usb device lost", netwatch.usb_devices(), 8)
    check("daemon gone", netwatch.rover_procs()["daemon"], 0)

    fake_board()
    cpu = netwatch.Cpu()
    write("proc/stat", "cpu  200 0 200 900 0 0 0\n"
                       "cpu0 25 0 75 250 0 0 0\ncpu1 50 0 50 250 0 0 0\n"
                       "cpu2 50 0 50 250 0 0 0\ncpu3 50 0 50 250 0 0 0\n")
    busy = cpu.busy()
    check("four cores", len(busy), 4)
    check("first core busy", busy[0], 50)


def test_format():
    line = netwatch.fmt("sample", {"ssid": "The Great Lord", "sig": -38,
                                   "gone": None, "empty": "", "f": 1.5})
    check("spaces never split a field", " ssid=The_Great_Lord " in " " + line + " ", True)
    check("absent fields are absent", "gone=" in line, False)
    check("empty fields are absent", "empty=" in line, False)
    check("floats stay short", "f=1.5" in line, True)
    check("kind is present", "kind=sample" in line, True)


def test_wpa_events():
    wpa = netwatch.Wpa()
    wpa.evt = FakeSock([
        b"<3>CTRL-EVENT-DISCONNECTED bssid=b0:19:21:b9:4e:fe reason=3 locally_generated=1",
        b"<3>CTRL-EVENT-BEACON-LOSS ",
    ])
    events = wpa.events()
    check("priority prefix stripped", events[0].startswith("CTRL-EVENT-DISCONNECTED"), True)
    check("reason kept", "reason=3" in events[0], True)
    check("both events", len(events), 2)
    check("no socket is not a crash", netwatch.Wpa().events(), [])


def test_kmsg_filter():
    keep = ["usb 3-1.1: USB disconnect, device number 4",
            "rtl8xxxu: Firmware not ready",
            "Under-voltage detected! (0x00050005)",
            "wlan0: deauthenticating from b0:19:21:b9:4e:fe by local choice",
            "Out of memory: Killed process 981 (python3)"]
    drop = ["random: crng init done", "EXT4-fs (mmcblk2p1): mounted filesystem"]
    for line in keep:
        check_true("kmsg keeps %r" % line[:24], netwatch.KMSG_KEEP.search(line))
    for line in drop:
        check("kmsg drops %r" % line[:24], bool(netwatch.KMSG_KEEP.search(line)), False)


def test_previous_boot():
    clean = ["t=1 kind=boot prev=unknown", "t=2 kind=sample up=10",
             "t=3 kind=stop up=20 why=signal"]
    hard = ["t=1 kind=boot prev=clean", "t=2 kind=sample up=10",
            "t=3 kind=sample up=20 load=3.9 temp=61.2"]
    check("a shutdown is a clean end", netwatch.classify_previous(clean)[0], "clean")
    verdict, last = netwatch.classify_previous(hard)
    check("no shutdown is a hard end", verdict, "hard")
    check("and the last words come back", "temp=61.2" in last, True)
    check("an empty log says so", netwatch.classify_previous([])[0], "unknown")
    # A stop followed by anything at all means the service came back and died
    # again without being asked -- still hard, and the common case after a
    # `systemctl restart` that the board then survived for a while.
    check("stop then more is hard",
          netwatch.classify_previous(clean + ["t=4 kind=sample up=30"])[0], "hard")


def test_log():
    directory = os.path.join(ROOT, "log")
    log = netwatch.Log(directory)
    log.write("t=1 kind=boot prev=unknown", sync=True)
    for n in range(200):
        log.write("t=%d kind=sample up=%d" % (n, n))
    log.sync()
    check("everything is on disk", len(log.tail()), 201)
    check("tail is the end", log.tail()[-1].endswith("up=199"), True)

    netwatch.ROTATE_BYTES, keep = 512, netwatch.ROTATE_BYTES
    log.maybe_rotate()
    log.write("t=2 kind=sample up=200", sync=True)
    netwatch.ROTATE_BYTES = keep
    check("rotated aside", os.path.exists(log.path + ".1"), True)
    check("and started again", len(log.tail()), 1)


def test_report():
    log = [
        "t=2026-08-23T12:00:00 kind=boot boot=aaaa prev=unknown up=40",
        "t=2026-08-23T12:00:10 kind=sample up=50 state=up ip=192.168.1.47 gw=192.168.1.1 rtt=3.1 sig=-38 ssid=TheGreatLord load=0.4 temp=48.9 cpu=10,5,3,2",
        "t=2026-08-23T12:00:20 kind=sample up=60 state=up ip=192.168.1.47 gw=192.168.1.1 rtt=2.9 sig=-40 ssid=TheGreatLord load=0.5 temp=49.1 cpu=12,6,4,2",
        "t=2026-08-23T12:00:25 kind=wpa ev=CTRL-EVENT-DISCONNECTED_bssid=b0:19:21:b9:4e:fe_reason=4",
        "t=2026-08-23T12:00:30 kind=change up=70 state=down wpa=SCANNING sig=-77 load=3.9 temp=61.2 usbn=9 cpu=99,98,97,96 rtt=none",
        "t=2026-08-23T12:00:40 kind=sample up=80 state=down wpa=SCANNING sig=-79 load=3.8 temp=61.5 usbn=9 cpu=99,97,96,95 rtt=none",
        "t=2026-08-23T12:00:50 kind=change up=90 state=up ip=192.168.1.47 gw=192.168.1.1 rtt=4.0 sig=-52 ssid=TheMaharaja bssid=aa:bb:cc:dd:ee:ff load=1.2 temp=58.0 cpu=30,20,10,5",
        "t=2026-08-23T12:01:00 kind=sample up=100 state=up ip=192.168.1.47 gw=192.168.1.1 rtt=3.5 sig=-51 ssid=TheMaharaja load=0.9 temp=57.0 cpu=20,10,5,5",
        "t=2026-08-23T12:30:00 kind=boot boot=bbbb prev=hard up=41",
        "t=2026-08-23T12:30:10 kind=sample up=51 state=up ip=192.168.1.47 gw=192.168.1.1 rtt=3.0 sig=-38 ssid=TheMaharaja load=0.3 temp=47.0 cpu=5,5,5,5",
    ]
    records = [netwatch_report.parse(line) for line in log]
    records = [r for r in records if r]
    check("every record parsed", len(records), len(log))

    check("a healthy sample is healthy", netwatch_report.healthy(records[1]), True)
    check("an unassociated one is not", netwatch_report.healthy(records[4]), False)
    check("a boot record is neither", netwatch_report.healthy(records[0]), None)

    # An address with no gateway answer is a link that carries nothing, and it
    # has to count as an outage or the report will call a dead link healthy.
    silent = dict(records[1], rtt="none")
    check("a link that carries nothing is an outage",
          netwatch_report.healthy(silent), False)
    check("and says why", netwatch_report.why(silent), "gateway did not answer")

    bs = netwatch_report.boots(records)
    check("two runs", len(bs), 2)
    # Two different boot ids, so this really was the board restarting and not the
    # service being reinstalled -- the distinction the report has to keep.
    check("on two different boards' worth of uptime", bs[0]["boot"] != bs[1]["boot"], True)
    check("the first ended badly", bs[0]["ended"], "hard")
    check("because the second said so", bs[1]["prev"], "hard")
    check("the second is still running", bs[1]["ended"], "running")

    eps = netwatch_report.episodes(records, window=1)
    check("one outage", len(eps), 1)
    check("it lasted 10 s", netwatch_report.secs(eps[0]["samples"][0]["up"],
                                                 eps[0]["samples"][-1]["up"]), 10)
    check("it says why", netwatch_report.why(eps[0]["samples"][0]),
          "not associated (state=down)")
    check("the supplicant's reason is inside it",
          any("reason=4" in e.get("ev", "") for e in eps[0]["events"]), True)
    check("the load at the time is there", eps[0]["samples"][0]["load"], "3.9")
    check("and it came back on another network", eps[0]["after"][0]["ssid"], "TheMaharaja")

    check("uptime is the clock that is safe", netwatch_report.secs("50", "90"), 40)
    check("a missing one is not a zero", netwatch_report.secs(None, "90"), None)
    check("durations read like durations", netwatch_report.human(3725), "1h02m")


def test_probe_verdicts():
    sys.path.insert(0, HERE)
    import netprobe
    check("gone", netprobe.verdict(None, "timeout", "unreachable"), "gone")
    # The one this exists for: ping answers, TCP connects, sshd never speaks.
    check("stalled", netprobe.verdict(3.2, "silent", "timeout"), "stalled")
    check("no daemon", netprobe.verdict(3.2, "ok", "refused"), "no-daemon")
    check("healthy", netprobe.verdict(3.2, "ok", "open"), "ok")


def main():
    for test in (test_link, test_board, test_format, test_wpa_events,
                 test_kmsg_filter, test_previous_boot, test_log, test_report,
                 test_probe_verdicts):
        try:
            test()
        except Exception as exc:                      # noqa: BLE001
            FAILED.append("%s raised %s: %s" % (test.__name__, type(exc).__name__, exc))
    shutil.rmtree(ROOT, ignore_errors=True)
    for line in FAILED:
        print("FAIL  " + line)
    print("%d assertions, %d failed" % (PASSED + len(FAILED), len(FAILED)))
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
