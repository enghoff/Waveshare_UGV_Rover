"""Network checks: which radio is read, and what an unfinished scan says.

The rover has more than one radio and the service address moves between them, so
nothing here may assume a device name. A scan that has not finished is the other
case: a missing signal column is a moment in time, not a verdict about a network.
"""
from __future__ import annotations

import os

from test_fakes import FakeLink
from test_harness import check

def test_the_radio_is_found_and_not_assumed_to_be_wlan0():
    """The Jetson has two radios and neither of them is called `wlan0`.

    This is the fault the console showed on the day the rover became a Jetson:
    the network panel had nothing in it at all, not even the access point the
    console itself was arriving over. Everything this file reads about the link
    -- the signal, the SSID, the address -- went to an interface named here, and
    a board whose onboard Realtek is `wlP1p1s0` and whose dongle is
    `wlx002e2d3074d0` does not have it. So the radio has to be found.

    It cannot be found on the machine running this test, which is the point of
    building a sysfs and a routing table in a directory: the interesting board
    is precisely the one this is not being run on.
    """
    import shutil
    import tempfile

    import rover_wifi

    root = tempfile.mkdtemp(prefix="rover-net-")
    net = os.path.join(root, "net")
    for name, radio in (("enP8p1s0", False), ("lo", False),
                        ("wlP1p1s0", True), ("wlx002e2d3074d0", True)):
        os.makedirs(os.path.join(net, name, "wireless") if radio
                    else os.path.join(net, name))
    route = os.path.join(root, "route")
    header = ("Iface\tDestination\tGateway \tFlags\tRefCnt\tUse\tMetric\tMask"
              "\t\tMTU\tWindow\tIRTT\n")

    def write_route(*rows: str) -> None:
        with open(route, "w", encoding="ascii") as handle:
            handle.write(header)
            handle.writelines(rows)

    real_net, real_route = rover_wifi.SYS_NET, rover_wifi.PROC_ROUTE
    real_ssid, real_level = rover_wifi._wifi_ssid, rover_wifi._wifi_level_dbm
    real_addr = rover_wifi._iface_address
    rover_wifi.SYS_NET, rover_wifi.PROC_ROUTE = net, route
    try:
        check("the wired interface is not mistaken for a radio",
              rover_wifi._wireless_ifaces(),
              ["wlP1p1s0", "wlx002e2d3074d0"])

        # Two default routes, one per radio, which is what this rover actually
        # has: both associated, to different routers. The kernel sends the
        # rover's traffic out of the cheaper one, so that is the one the panel
        # is reporting on.
        write_route("wlx002e2d3074d0\t00000000\t0101A8C0\t0003\t0\t0\t600"
                    "\t00000000\t0\t0\t0\n",
                    "wlP1p1s0\t00000000\t0101A8C0\t0003\t0\t0\t100"
                    "\t00000000\t0\t0\t0\n")
        check("the radio carrying the traffic is the one reported on",
              rover_wifi._wifi_iface(), "wlP1p1s0")

        # And when that one loses its route, the answer moves with the traffic
        # rather than staying on a radio that is no longer carrying any.
        write_route("wlx002e2d3074d0\t00000000\t0101A8C0\t0003\t0\t0\t600"
                    "\t00000000\t0\t0\t0\n")
        check("...and it moves when the traffic does",
              rover_wifi._wifi_iface(), "wlx002e2d3074d0")

        # With the helper missing there is no list of neighbours to be had, and
        # the one thing left worth saying is what the radio carrying the traffic
        # is on. The dongle is on the bus and NetworkManager is told to leave it
        # alone, so it has no network to report and reporting it as one of the
        # rover's would be inventing a second way in that does not exist.
        heard = {"wlP1p1s0": ("TheGreatLord", -47),
                 "wlx002e2d3074d0": ("TheGreatViking", -68)}
        rover_wifi._wifi_ssid = lambda iface=None: heard[iface][0]
        rover_wifi._wifi_level_dbm = lambda iface=None: heard[iface][1]
        rover_wifi._iface_address = lambda iface=None: "192.168.1.77"
        live = rover_wifi._wifi_from_kernel()
        check("without the helper the associated radio still reports itself",
              (live or {}).get("connected"), "TheGreatViking")
        check("...and it is the only network named",
              [n["ssid"] for n in (live or {}).get("networks", [])],
              ["TheGreatViking"])
        check("...on the same 0-100 scale as every other list here",
              [n["signal"] for n in (live or {}).get("networks", [])], [64])
        check("...and is not offered as somewhere to join, since it could not",
              [n["in_use"] for n in (live or {}).get("networks", [])],
              [True])
    finally:
        rover_wifi.SYS_NET, rover_wifi.PROC_ROUTE = real_net, real_route
        rover_wifi._wifi_ssid, rover_wifi._wifi_level_dbm = real_ssid, real_level
        rover_wifi._iface_address = real_addr
        shutil.rmtree(root, ignore_errors=True)


def test_wifi_status_without_the_helper_still_reports_the_link():
    """The console's network panel has to work without NetworkManager.

    `wifi_ctl.sh` is the privileged helper. When it is missing the page used to
    show only that sentence -- no SSID, no address -- even though the kernel
    already knew both.

    """
    import rover_daemon
    import rover_wifi

    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    real_ctl = rover_wifi._wifi_ctl
    real_live = rover_wifi._wifi_from_kernel
    rover_wifi._wifi_ctl = lambda *args, **kwargs: (
        False, "/usr/local/sbin/wifi_ctl.sh is not installed on this rover; "
               "run wifi_roam/install.sh")
    rover_wifi._wifi_from_kernel = lambda iface="wlan0": {
        "interface": iface, "connected": "TheGreatLord", "level_dbm": -47,
        "address": "192.168.1.47",
        "networks": [{"ssid": "TheGreatLord", "signal": -47, "security": "",
                      "in_use": True, "configured": True}],
        "configured": ["TheGreatLord"], "scanned": False, "list_age_s": 0.0,
    }
    try:
        got = rover.call("wifi_status", {})
    finally:
        rover_wifi._wifi_ctl = real_ctl
        rover_wifi._wifi_from_kernel = real_live
    check("wifi_status still answers", got.get("ok"), True)
    check("...with the associated network", got.get("connected"), "TheGreatLord")
    check("...and the address", got.get("address"), "192.168.1.47")
    check("...and says the helper is missing", "install" in str(got.get("note", "")), True)


def test_an_unfilled_signal_column_is_a_moment_not_an_answer():
    """The M4 Zero's driver leaves /proc/net/wireless at -256 now and then.

    -256 is "not filled in" rather than a reading, and the next read is usually
    good, so it is worth re-reading before falling back. The fallback is no use on
    this board anyway -- `iw` reports `signal: 0 dBm` here, which is not a level
    either -- so giving up on one sample cost the console its signal entirely.
    """
    import rover_wifi

    reads = []

    def fake(iface="wlan0"):
        reads.append(iface)
        return None if len(reads) < 3 else -41

    slept = []
    real_proc, real_iw = rover_wifi._proc_level_dbm, rover_wifi._iw_signal_dbm
    real_sleep = rover_wifi.time.sleep
    rover_wifi._proc_level_dbm = fake
    rover_wifi._iw_signal_dbm = lambda iface="wlan0": "asked iw"
    rover_wifi.time.sleep = slept.append
    try:
        check("an unfilled column is read again", rover_wifi._wifi_level_dbm(), -41)
        check("...and iw is not asked while /proc still answers", len(reads), 3)
        # The driver refreshes the figure on a timer, so a re-read that does not
        # wait is the same read again and cannot come back different.
        check("...having waited between the tries", slept, [rover_wifi.PROC_LEVEL_GAP_S] * 2)
        reads.clear()
        del slept[:]
        rover_wifi._proc_level_dbm = lambda iface="wlan0": reads.append(iface)
        check("a column that never fills falls back to iw",
              rover_wifi._wifi_level_dbm(), "asked iw")
        check("...after trying /proc a few times",
              len(reads), rover_wifi.PROC_LEVEL_TRIES)
        check("...and does not wait after the last try",
              len(slept), rover_wifi.PROC_LEVEL_TRIES - 1)
    finally:
        rover_wifi._proc_level_dbm, rover_wifi._iw_signal_dbm = real_proc, real_iw
        rover_wifi.time.sleep = real_sleep


def test_reading_the_network():
    """Parsing what nmcli says, and refusing a network there is no key for.

    The parsing earns a check of its own because its input is a string a stranger
    chose. An SSID may contain a colon, `nmcli -t` escapes it, and a `split(":")`
    gets away with that until the day somebody's router is called something
    awkward -- at which point the panel shows the wrong signal against the wrong
    name and looks like it is working.

    The rest is what happens on a machine with no wifi helper installed at all,
    which is every machine but the rover: a refusal in words, and never a
    traceback, because both calls are wired to live buttons in a window.
    """
    import rover_daemon

    check("an escaped colon stays inside its field",
          rover_daemon._terse_fields(r"*:My\:Net:84:WPA2"),
          ["*", "My:Net", "84", "WPA2"])
    check("an escaped backslash does too",
          rover_daemon._terse_fields(r"\\:x:1"), ["\\", "x", "1"])

    rows = "\n".join(("*:TheGreatLord:52:WPA2",
                      " :TheGreatLord:40:WPA2",      # the same router's other radio
                      " :TheMaharaja:84:WPA2",
                      " :Stranger:99:WPA2",
                      " ::70:WPA2"))                 # a hidden network
    seen = rover_daemon._wifi_networks(rows, {"TheGreatLord", "TheMaharaja"})
    check("one row per network, not per radio",
          [n["ssid"] for n in seen], ["TheMaharaja", "TheGreatLord", "Stranger"])
    check("...at the strongest signal that network was heard on",
          [n["signal"] for n in seen if n["ssid"] == "TheGreatLord"], [52])
    check("...still marked as the one in use",
          [n["in_use"] for n in seen if n["ssid"] == "TheGreatLord"], [True])
    check("...and a hidden network is not offered as a choice",
          any(not n["ssid"] for n in seen), False)
    check("the ones with a passphrase come first",
          [n["configured"] for n in seen], [True, True, False])

    # And the two calls themselves, which answer differently depending on where
    # this is run -- so both worlds are checked rather than the convenient one.
    # On the rover the helper is installed and this reads the real radio; anywhere
    # else it is absent, and what comes back has to be a sentence rather than a
    # traceback, because both calls are wired to live buttons in a window.
    rover = rover_daemon.Rover(FakeLink(), "unused", device=None)
    check("wifi_join wants an ssid", rover.call("wifi_join", {})["ok"], False)
    # And one *with* a name, which is the only path a console button ever takes and
    # the only one that reaches the rest of the call. The check above returns two
    # lines in, so it was covering none of it -- which is how a wifi_join that
    # raised TypeError before it did anything went unnoticed on a rover that had
    # been asked to switch networks by hand. `call` turns that into a sentence like
    # any other refusal, so it reads as the rover declining rather than as a bug.
    #
    # A name no rover holds a passphrase for, so this is safe to run on the rover
    # itself: there it is refused before the radio is touched, and on a desk, where
    # there is no helper to ask, the request is accepted and the thread behind it
    # finds nothing to run.
    named = rover.call("wifi_join", {"ssid": "NoSuchNetworkHere"})
    check("wifi_join with a name is answered rather than raising",
          named.get("ok") is True or "no passphrase" in str(named.get("error", "")),
          True)
    asked = rover.call("wifi_status", {})
    if asked["ok"]:
        check("wifi_status answers with every field the panel reads",
              [key for key in ("interface", "connected", "level_dbm", "address",
                               "networks", "configured") if key not in asked], [])
    else:
        check("wifi_status without the helper says how to install it",
              "install" in asked["error"], True)


TESTS = (
    test_the_radio_is_found_and_not_assumed_to_be_wlan0,
    test_wifi_status_without_the_helper_still_reports_the_link,
    test_an_unfilled_signal_column_is_a_moment_not_an_answer,
    test_reading_the_network,
)
