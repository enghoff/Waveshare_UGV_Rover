"""Bluetooth checks: what the console is shown, and pairing's three outcomes.

There is no system bus here, so BlueZ is a dictionary of objects shaped the way
`GetManagedObjects` answers on the rover, and every call made on it is recorded.
What matters is that a scan in a crowded building shows only what could be
paired with a rover, that a pair leaves the device trusted and the adapter not
pairable whatever happened, and that a device BlueZ has dropped since the scan
is looked for again rather than reported missing.
"""
from __future__ import annotations

import os
import shutil
import tempfile
import time

from test_fakes import FakeLink
from test_harness import check

HCI = "/org/bluez/hci0"
PAD = "AA:BB:CC:00:11:22"
SPEAKER = "11:22:33:44:55:66"
HEADSET = "77:88:99:AA:BB:CC"
PHONE = "DE:AD:BE:EF:00:01"


def _device(address, name, icon, **props):
    body = {"Address": address, "Alias": name, "Icon": icon, "Paired": False,
            "Trusted": False, "Connected": False, "UUIDs": []}
    body.update(props)
    return {"org.bluez.Device1": body}


class DBusError(Exception):
    def __init__(self, name, message):
        super().__init__(message)
        self.name, self.message = name, message

    def get_dbus_name(self):
        return self.name

    def get_dbus_message(self):
        return self.message


class FakeBluez:
    """The daemon's view of bluetoothd: objects, and a log of what was asked."""

    def __init__(self):
        self.objects = {
            "/org/bluez": {"org.bluez.AgentManager1": {}},
            HCI: {"org.bluez.Adapter1": {"Address": "F0:68:E3:A8:B4:86",
                                         "Powered": True, "Pairable": False,
                                         "Discovering": False}},
        }
        self.calls = []
        self.fail = {}          # method name -> exception it raises
        self.on_discover = None
        self.closed = 0

    def add(self, path, body):
        self.objects[path] = body

    def link(self):
        return _Conn(self)


class _Conn:
    def __init__(self, bluez):
        self.bluez = bluez

    def objects(self):
        return {p: {i: dict(v) for i, v in ifaces.items()}
                for p, ifaces in self.bluez.objects.items()}

    def call(self, path, iface, method, *args, timeout=10.0):
        self.bluez.calls.append((method, path))
        if method in self.bluez.fail:
            raise self.bluez.fail[method]
        if method == "StartDiscovery" and self.bluez.on_discover:
            self.bluez.on_discover()
        if method == "Pair":
            self.bluez.objects[path]["org.bluez.Device1"]["Paired"] = True
        if method == "Connect":
            self.bluez.objects[path]["org.bluez.Device1"]["Connected"] = True

    def remove(self, adapter, device):
        self.bluez.calls.append(("RemoveDevice", device))
        self.bluez.objects.pop(device, None)

    def set(self, path, iface, name, value):
        self.bluez.calls.append((f"set {name}={value}", path))
        self.bluez.objects[path][iface][name] = value

    def close(self):
        self.bluez.closed += 1


def _rover(bluez):
    import rover_daemon

    rover = rover_daemon.Rover(FakeLink(), "unused", device="/dev/null")
    rover.BT_LINK = bluez.link
    return rover


def _settle(rover, seconds=5.0):
    """Wait for the pair or scan thread to finish."""
    until = time.monotonic() + seconds
    while time.monotonic() < until:
        thread = rover._bt_scan_thread
        if rover._bt_busy is None and (thread is None or not thread.is_alive()):
            return
        time.sleep(0.02)


def _crowd(bluez):
    bluez.add(f"{HCI}/dev_pad", _device(PAD, "Xbox Wireless Controller",
                                        "input-gaming", RSSI=-58))
    bluez.add(f"{HCI}/dev_spk", _device(
        SPEAKER, "BT Speaker", "audio-card", Paired=True, Trusted=True,
        Connected=True, UUIDs=["0000110b-0000-1000-8000-00805f9b34fb"]))
    bluez.add(f"{HCI}/dev_hs", _device(
        HEADSET, "Headset", "audio-headset", Paired=True,
        UUIDs=["0000111e-0000-1000-8000-00805f9b34fb",
               "0000110b-0000-1000-8000-00805f9b34fb"]))
    bluez.add(f"{HCI}/dev_phone", _device(PHONE, "Somebody's phone", "phone",
                                          RSSI=-40))


def test_the_console_is_shown_what_could_be_paired_and_nothing_else():
    bluez = FakeBluez()
    _crowd(bluez)
    rover = _rover(bluez)
    status = rover.call("bt_status", {})
    check("bt_status answers", status.get("ok"), True)
    devices = status["devices"]
    check("connected first, then paired, then what a scan found; the phone "
          "is counted and not listed",
          [(d["name"], d["kind"]) for d in devices],
          [("BT Speaker", "speaker"), ("Headset", "headset"),
           ("Xbox Wireless Controller", "controller")])
    check("...and it was heard", status["heard"], 4)
    check("a headset is the one with a microphone",
          [d["mic"] for d in devices], [False, True, False])
    check("every connection is closed after use", bluez.closed, 1)

    bluez.objects.pop(HCI)
    check("no adapter is said, not shown as nothing paired",
          (rover.call("bt_status", {}).get("ok"),
           rover.call("bt_status", {}).get("supported")), (False, False))

    def broken():
        raise DBusError("org.freedesktop.DBus.Error.ServiceUnknown",
                        "The name org.bluez was not provided")
    rover.BT_LINK = broken
    gone = rover.call("bt_status", {})
    check("a bluetoothd that is not running is an error with its reason",
          (gone.get("ok"), "ServiceUnknown" in gone.get("error", "")), (False, True))


def test_a_connected_controller_shows_the_joystick_it_became():
    import rover_bluetooth

    root = tempfile.mkdtemp(prefix="rover-input-")
    was = rover_bluetooth.SYS_INPUT
    try:
        for name, uniq, children in (("input3", "", ["event3"]),
                                     ("input9", PAD.lower(), ["event7", "js0"])):
            os.makedirs(os.path.join(root, name))
            for child in children:
                os.makedirs(os.path.join(root, name, child))
            with open(os.path.join(root, name, "uniq"), "w") as handle:
                handle.write(uniq + "\n")
        rover_bluetooth.SYS_INPUT = root
        bluez = FakeBluez()
        bluez.add(f"{HCI}/dev_pad", _device(PAD, "Pad", "input-gaming",
                                            Paired=True, Connected=True))
        devices = _rover(bluez).call("bt_status", {})["devices"]
        check("the controller's joystick is found by its address",
              devices[0]["input"], "js0")
    finally:
        rover_bluetooth.SYS_INPUT = was
        shutil.rmtree(root, ignore_errors=True)


def test_a_scan_is_held_and_then_let_go():
    bluez = FakeBluez()
    rover = _rover(bluez)
    started = rover.call("bt_scan", {"seconds": 5})
    check("a scan answers at once", started.get("scanning"), True)
    time.sleep(0.1)
    check("...and is under way", rover.call("bt_status", {})["scanning"], True)
    rover._bt_scan_until = 0.0
    _settle(rover)
    check("discovery is started and stopped on one connection",
          [m for m, _ in bluez.calls], ["StartDiscovery", "StopDiscovery"])
    check("...and the scan is over", rover.call("bt_status", {})["scanning"], False)


def test_pairing_leaves_it_trusted_and_the_adapter_not_pairable():
    import rover_bluetooth

    bluez = FakeBluez()
    _crowd(bluez)
    rover = _rover(bluez)
    check("a bad address is refused",
          rover.call("bt_pair", {"address": "pad"}).get("ok"), False)
    asked = rover.call("bt_pair", {"address": PAD.lower()})
    check("a pair answers at once, by name",
          (asked.get("pairing"), asked.get("name")), (PAD, "Xbox Wireless Controller"))
    _settle(rover)
    check("pairable, pair, trust, connect, and pairable again only for the pair",
          [m for m, _ in bluez.calls],
          ["set Pairable=True", "Pair", "set Trusted=True", "Connect",
           "set Pairable=False"])
    last = rover.call("bt_status", {})["last"]
    check("...and the next status says it worked",
          (last["what"], last["ok"], last["connected"]), ("pair", True, True))

    # A pair the device refuses: still not pairable afterwards, and why.
    bluez.calls.clear()
    bluez.objects[f"{HCI}/dev_pad"]["org.bluez.Device1"]["Paired"] = False
    bluez.fail["Pair"] = DBusError("org.bluez.Error.AuthenticationFailed",
                                   "Authentication Failed")
    rover.call("bt_pair", {"address": PAD})
    _settle(rover)
    last = rover.call("bt_status", {})["last"]
    check("a refused pair says why",
          (last["ok"], last["said"]), (False, "AuthenticationFailed: Authentication Failed"))
    check("...and leaves the adapter as it found it",
          bluez.calls[-1][0], "set Pairable=False")
    check("...and is not trusted", "set Trusted=True" in
          [m for m, _ in bluez.calls], False)

    # Paired but switched off: the pair stands, and the connect error is said.
    bluez.fail = {"Connect": DBusError("org.bluez.Error.Failed",
                                       "Page Timeout")}
    rover.call("bt_pair", {"address": PAD})
    _settle(rover)
    last = rover.call("bt_status", {})["last"]
    check("a pair that could not connect is still a pair",
          (last["ok"], last["connected"], last["said"]),
          (True, False, "Failed: Page Timeout"))

    # Dropped from BlueZ's list since the scan: looked for again.
    bluez.fail = {}
    bluez.calls.clear()
    late = "C0:FF:EE:00:00:01"
    bluez.on_discover = lambda: bluez.add(
        f"{HCI}/dev_late", _device(late, "Late pad", "input-gaming"))
    rover.call("bt_pair", {"address": late})
    _settle(rover)
    check("a device gone from the list is discovered again and paired",
          [m for m, _ in bluez.calls][:3], ["StartDiscovery", "StopDiscovery",
                                            "set Pairable=True"])
    check("...under its own name", rover.call("bt_status", {})["last"]["name"],
          "Late pad")

    was = rover_bluetooth.BT_FIND_S
    rover_bluetooth.BT_FIND_S = 0.2
    try:
        bluez.on_discover = None
        rover.call("bt_pair", {"address": "C0:FF:EE:00:00:02"})
        _settle(rover)
        last = rover.call("bt_status", {})["last"]
        check("one that is never heard says so",
              (last["ok"], "pairing mode" in last["said"]), (False, True))
    finally:
        rover_bluetooth.BT_FIND_S = was


def test_unpairing_forgets_the_device():
    bluez = FakeBluez()
    _crowd(bluez)
    rover = _rover(bluez)
    gone = rover.call("bt_unpair", {"address": SPEAKER})
    check("unpair removes the device", (gone.get("ok"), gone.get("name")),
          (True, "BT Speaker"))
    check("...through the adapter", bluez.calls, [("RemoveDevice", f"{HCI}/dev_spk")])
    check("...and it is gone from the list",
          SPEAKER in [d["address"] for d in rover.call("bt_status", {})["devices"]],
          False)
    check("an address the rover does not know is refused",
          rover.call("bt_unpair", {"address": SPEAKER}).get("ok"), False)


TESTS = (
    test_the_console_is_shown_what_could_be_paired_and_nothing_else,
    test_a_connected_controller_shows_the_joystick_it_became,
    test_a_scan_is_held_and_then_let_go,
    test_pairing_leaves_it_trusted_and_the_adapter_not_pairable,
    test_unpairing_forgets_the_device,
)
