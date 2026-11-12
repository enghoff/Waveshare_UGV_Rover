"""Bluetooth: pairing a game controller or an audio device, for the console.

Four control calls, and none of them is offered to the model, for the reason the
network calls are not: what the rover is paired with is a decision about the
rover, made by a person holding the device in question, and a model that paired
or forgot one would be acting on hardware it cannot see.

    bt_status   what the adapter knows: paired devices and what a scan heard
    bt_scan     look for devices for half a minute; answers at once
    bt_pair     pair, trust and connect one; answers at once, `last` says how
    bt_unpair   forget one, which also disconnects it

**Everything goes over BlueZ's D-Bus API, as the `jetson` user.** The system bus
policy on this Ubuntu lets any user talk to `org.bluez`, so unlike the Wi-Fi
helper there is no sudo rule and nothing to install. Each call opens its own
private connection and closes it again, which keeps two threads off one libdbus
connection and survives `bluetoothd` being restarted underneath the daemon.

**A scan is held, not fired.** BlueZ ends a discovery session when the client that
started it disconnects, so the scan runs on its own thread holding its own
connection for BT_SCAN_S and then lets go. Devices it heard stay in BlueZ's list
for about half a minute after that (`TemporaryTimeout`), so a pair that finds its
device gone looks for it again rather than failing at once.

**Pairing uses no agent.** A controller, a speaker or a headset pairs with "just
works" pairing, which the kernel accepts by itself when the rover is the one
asking; a device that wants a PIN typed is refused, and says so in `last`. The
adapter is made pairable only for the length of one pairing, because with no
agent registered a pairable adapter would also accept a stranger who asked; it is
not pairable at rest (`Pairable: no` on this rover), and an LE controller paired
while it is not would pair without bonding and never reconnect by itself.
"""
from __future__ import annotations

import os
import re
import threading
import time
from typing import Any

BLUEZ = "org.bluez"
ADAPTER = "org.bluez.Adapter1"
DEVICE = "org.bluez.Device1"
OBJECT_MANAGER = "org.freedesktop.DBus.ObjectManager"
PROPERTIES = "org.freedesktop.DBus.Properties"

#: How long a press of "look" keeps the radio discovering. Long enough to put a
#: controller into pairing mode after pressing it; a device found in the first
#: seconds is in the list, with a pair button, while the rest runs.
BT_SCAN_S = 30.0
#: The longest a pair waits for the device to answer, which covers a controller
#: that is slow to bring up its link and a person who pressed pair a moment before
#: holding the controller's own pairing button.
BT_PAIR_TIMEOUT_S = 40.0
#: Connecting after pairing. A controller usually connects as part of the pair;
#: a headset brings up its audio profiles here, and that can take a few seconds.
BT_CONNECT_TIMEOUT_S = 20.0
#: How long a pair looks for a device BlueZ has dropped from its list since the
#: scan that found it -- it keeps unpaired devices about 30 s after a scan ends.
BT_FIND_S = 20.0

#: Profiles that say what a device can do with sound. A headset that offers the
#: hands-free or headset profile has a microphone; a speaker offers only A2DP.
HANDSFREE = "0000111e-0000-1000-8000-00805f9b34fb"
HEADSET = "00001108-0000-1000-8000-00805f9b34fb"
A2DP_SINK = "0000110b-0000-1000-8000-00805f9b34fb"
#: BlueZ's own icon for a device class or LE appearance, which is how a gamepad
#: identifies itself before it has been paired and its profiles are known.
AUDIO_ICONS = ("audio-headset", "audio-headphones", "audio-card")

#: Where the kernel lists input devices, so that a connected controller can be
#: shown with the joystick it became. Named so that the self-test can build one.
SYS_INPUT = "/sys/class/input"

_ADDRESS = re.compile(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$")


def _plain(value: Any) -> Any:
    """dbus-python's types as plain Python, so that a reply can go into JSON.

    `dbus.Boolean` is an int subclass, so it has to be caught by name before
    anything treats it as a number.
    """
    if isinstance(value, bool) or type(value).__name__ == "Boolean":
        return bool(value)
    if isinstance(value, dict):
        return {str(k): _plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(v) for v in value]
    if isinstance(value, str):
        return str(value)
    if isinstance(value, int):
        return int(value)
    if isinstance(value, float):
        return float(value)
    return value


def _said(error: Exception) -> str:
    """A D-Bus error as one short line: BlueZ's error name and its message."""
    name = getattr(error, "get_dbus_name", lambda: None)()
    message = (getattr(error, "get_dbus_message", lambda: None)()
               or str(error)).strip()
    if name:
        short = name.rsplit(".", 1)[-1]
        return (f"{short}: {message}" if message and message != short
                else short)[:200]
    return message[:200] or type(error).__name__


class BluezLink:
    """One private connection to BlueZ, for one call or one thread."""

    def __init__(self) -> None:
        import dbus

        self.dbus = dbus
        self.bus = dbus.SystemBus(private=True)

    def objects(self) -> dict[str, dict[str, dict[str, Any]]]:
        manager = self.dbus.Interface(
            self.bus.get_object(BLUEZ, "/", introspect=False), OBJECT_MANAGER)
        return _plain(manager.GetManagedObjects(timeout=5.0))

    def call(self, path: str, iface: str, method: str, *args: Any,
             timeout: float = 10.0) -> None:
        target = self.dbus.Interface(
            self.bus.get_object(BLUEZ, path, introspect=False), iface)
        getattr(target, method)(*args, timeout=timeout)

    def remove(self, adapter: str, device: str) -> None:
        self.call(adapter, ADAPTER, "RemoveDevice", self.dbus.ObjectPath(device))

    def set(self, path: str, iface: str, name: str, value: Any) -> None:
        props = self.dbus.Interface(
            self.bus.get_object(BLUEZ, path, introspect=False), PROPERTIES)
        if isinstance(value, bool):
            value = self.dbus.Boolean(value)
        props.Set(iface, name, value, timeout=5.0)

    def close(self) -> None:
        try:
            self.bus.close()
        except Exception:
            pass


def _kind(props: dict[str, Any]) -> tuple[str | None, bool]:
    """What a device is, for the console, and whether it has a microphone."""
    icon = str(props.get("Icon") or "")
    uuids = {str(u).lower() for u in props.get("UUIDs") or []}
    mic = HANDSFREE in uuids or HEADSET in uuids
    if icon == "input-gaming":
        return "controller", False
    if icon in AUDIO_ICONS or A2DP_SINK in uuids or mic:
        if mic:
            return "headset", True
        return ("headphones" if icon == "audio-headphones" else "speaker"), False
    return None, False


def _input_node(address: str) -> str | None:
    """The joystick (or event device) the kernel made of a connected controller.

    Matched on `uniq`, which BlueZ fills with the controller's own address for
    both classic and LE controllers; a controller that is paired and connected
    but has no node here is one the kernel has no driver for.
    """
    want = address.lower()
    try:
        names = os.listdir(SYS_INPUT)
    except OSError:
        return None
    for name in sorted(names):
        if not name.startswith("input"):
            continue
        try:
            with open(os.path.join(SYS_INPUT, name, "uniq"), encoding="ascii") as f:
                if f.read().strip().lower() != want:
                    continue
            children = sorted(os.listdir(os.path.join(SYS_INPUT, name)))
        except OSError:
            continue
        for prefix in ("js", "event"):
            for child in children:
                if child.startswith(prefix) and child[len(prefix):].isdigit():
                    return child
    return None


def _adapter(objects: dict[str, Any]) -> tuple[str | None, dict[str, Any]]:
    for path, ifaces in sorted(objects.items()):
        if ADAPTER in ifaces:
            return path, ifaces[ADAPTER]
    return None, {}


def _device_path(objects: dict[str, Any], address: str) -> tuple[str | None, dict]:
    for path, ifaces in objects.items():
        props = ifaces.get(DEVICE)
        if props and str(props.get("Address", "")).upper() == address:
            return path, props
    return None, {}


def bt_devices(objects: dict[str, Any]) -> tuple[list[dict[str, Any]], int]:
    """The devices worth showing, and how many the adapter knows in all.

    Worth showing means paired, or a controller or an audio device. A scan in a
    block of flats hears dozens of phones, televisions and beacons, none of which
    anybody means to pair with a rover, and they are counted rather than listed.
    """
    shown, heard = [], 0
    for _path, ifaces in objects.items():
        props = ifaces.get(DEVICE)
        if not props:
            continue
        heard += 1
        kind, mic = _kind(props)
        paired = bool(props.get("Paired"))
        if kind is None and not paired:
            continue
        address = str(props.get("Address", "")).upper()
        connected = bool(props.get("Connected"))
        shown.append({
            "address": address,
            "name": str(props.get("Alias") or props.get("Name") or address),
            "kind": kind or "other",
            "mic": mic,
            "paired": paired,
            "trusted": bool(props.get("Trusted")),
            "connected": connected,
            "rssi": props.get("RSSI"),
            "input": _input_node(address) if connected and kind == "controller"
                     else None,
        })
    # Connected first, then paired, then what a scan found, loudest first.
    shown.sort(key=lambda d: (not d["connected"], not d["paired"],
                              -(d["rssi"] if isinstance(d["rssi"], int) else -999),
                              d["name"]))
    return shown, heard


class RoverBluetooth:
    """Pairing, mixed into Rover. Not offered to the model."""

    #: Replaced by a fake in the self-test, which has no system bus.
    BT_LINK = BluezLink

    def _bt_init(self) -> None:
        self._bt_lock = threading.Lock()
        self._bt_scan_until = 0.0
        self._bt_scan_thread: threading.Thread | None = None
        self._bt_scan_stop = threading.Event()
        # The pairing in flight, if any: one at a time, since the adapter can
        # only bring up one bond at once and the console shows one.
        self._bt_busy: dict[str, Any] | None = None
        # What the last pair, unpair or scan came to, for whoever asks next:
        # a pair takes far longer than the console's call waits.
        self._bt_last: dict[str, Any] | None = None

    def _bt_read(self) -> tuple[str | None, dict, dict[str, Any]]:
        link = self.BT_LINK()
        try:
            objects = link.objects()
        finally:
            link.close()
        path, props = _adapter(objects)
        return path, props, objects

    @staticmethod
    def _bt_address(arguments: dict[str, Any]) -> str | None:
        address = str(arguments.get("address") or "").strip().upper()
        return address if _ADDRESS.match(address) else None

    def _tool_bt_status(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """The adapter, the devices worth showing, and what is under way."""
        try:
            path, adapter, objects = self._bt_read()
        except Exception as error:
            return {"ok": False, "error": f"Bluetooth is not answering: {_said(error)}"}
        if path is None:
            return {"ok": False, "supported": False,
                    "error": "this rover has no Bluetooth adapter"}
        devices, heard = bt_devices(objects)
        now = time.monotonic()
        with self._bt_lock:
            left = max(0.0, self._bt_scan_until - now)
            scanning = (self._bt_scan_thread is not None
                        and self._bt_scan_thread.is_alive())
            busy = dict(self._bt_busy) if self._bt_busy else None
            last = dict(self._bt_last) if self._bt_last else None
        reading = {"ok": True, "adapter": str(adapter.get("Address", "")),
                   "powered": bool(adapter.get("Powered")),
                   "discovering": bool(adapter.get("Discovering")),
                   "scanning": scanning, "scan_left_s": round(left, 1),
                   "devices": devices, "heard": heard}
        if busy:
            reading["busy"] = busy
        if last:
            reading["last"] = last
        return reading

    def _tool_bt_scan(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Start looking for devices, or keep looking for longer."""
        try:
            seconds = float(arguments.get("seconds") or BT_SCAN_S)
        except (TypeError, ValueError):
            seconds = BT_SCAN_S
        seconds = max(5.0, min(120.0, seconds))
        with self._bt_lock:
            if self._bt_busy:
                return {"ok": False, "error": f"busy pairing "
                                              f"{self._bt_busy.get('name')}"}
            self._bt_scan_until = time.monotonic() + seconds
            if self._bt_scan_thread is not None and self._bt_scan_thread.is_alive():
                return {"ok": True, "scanning": True, "seconds": seconds}
            self._bt_scan_stop.clear()
            self._bt_scan_thread = threading.Thread(
                target=self._bt_scan_run, daemon=True, name="bt-scan")
            self._bt_scan_thread.start()
        return {"ok": True, "scanning": True, "seconds": seconds}

    def _bt_scan_run(self) -> None:
        link = None
        try:
            link = self.BT_LINK()
            path, adapter = _adapter(link.objects())
            if path is None:
                raise RuntimeError("no Bluetooth adapter")
            if not adapter.get("Powered"):
                link.set(path, ADAPTER, "Powered", True)
            link.call(path, ADAPTER, "StartDiscovery")
            while (time.monotonic() < self._bt_scan_until
                   and not self._bt_scan_stop.wait(0.25)):
                pass
            try:
                link.call(path, ADAPTER, "StopDiscovery")
            except Exception:
                pass        # closing the connection ends it anyway
        except Exception as error:
            with self._bt_lock:
                self._bt_last = {"what": "scan", "ok": False, "said": _said(error),
                                 "at": round(time.time(), 1)}
        finally:
            if link is not None:
                link.close()
            with self._bt_lock:
                self._bt_scan_until = 0.0

    def _bt_stop_scan(self) -> None:
        thread = self._bt_scan_thread
        if thread is not None and thread.is_alive():
            self._bt_scan_stop.set()
            thread.join(timeout=3.0)

    def _tool_bt_pair(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Pair, trust and connect a device; answers before it has done it.

        A pair can take most of a minute, so it runs on its own thread and the
        next `bt_status` carries how it went as `last`. Trusted is what lets a
        controller reconnect by itself when it is next switched on.
        """
        address = self._bt_address(arguments)
        if address is None:
            return {"ok": False, "error": "bt_pair wants an address like "
                                          "AA:BB:CC:DD:EE:FF"}
        try:
            _path, _adapter_props, objects = self._bt_read()
        except Exception as error:
            return {"ok": False, "error": f"Bluetooth is not answering: {_said(error)}"}
        _device, props = _device_path(objects, address)
        name = str(props.get("Alias") or props.get("Name") or address)
        with self._bt_lock:
            if self._bt_busy:
                return {"ok": False, "error": f"busy pairing "
                                              f"{self._bt_busy.get('name')}"}
            self._bt_busy = {"what": "pair", "address": address, "name": name,
                             "since": round(time.time(), 1)}
        threading.Thread(target=self._bt_pair_run, args=(address, name),
                         daemon=True, name="bt-pair").start()
        return {"ok": True, "pairing": address, "name": name}

    def _bt_pair_run(self, address: str, name: str) -> None:
        began = time.time()
        outcome: dict[str, Any] = {"what": "pair", "address": address,
                                   "name": name, "ok": False, "said": ""}
        link = None
        adapter = None
        try:
            # Inquiry and paging share the radio, so a scan still running would
            # slow the pair down or make it fail.
            self._bt_stop_scan()
            link = self.BT_LINK()
            objects = link.objects()
            adapter, _props = _adapter(objects)
            if adapter is None:
                raise RuntimeError("no Bluetooth adapter")
            device, props = _device_path(objects, address)
            if device is None:
                device, props = self._bt_find(link, adapter, address)
            if device is None:
                outcome["said"] = "not heard -- is it still in pairing mode?"
                return
            outcome["name"] = str(props.get("Alias") or props.get("Name") or name)
            link.set(adapter, ADAPTER, "Pairable", True)
            if not props.get("Paired"):
                link.call(device, DEVICE, "Pair", timeout=BT_PAIR_TIMEOUT_S)
            link.set(device, DEVICE, "Trusted", True)
            outcome["ok"] = True
            try:
                link.call(device, DEVICE, "Connect", timeout=BT_CONNECT_TIMEOUT_S)
                outcome["connected"] = True
            except Exception as error:
                # Paired and trusted is the part that lasts; a device that did
                # not connect now will when it is next switched on.
                outcome["connected"] = False
                outcome["said"] = _said(error)
        except Exception as error:
            outcome["said"] = _said(error)
        finally:
            if link is not None:
                if adapter is not None:
                    try:
                        link.set(adapter, ADAPTER, "Pairable", False)
                    except Exception:
                        pass
                link.close()
            outcome["at"] = round(began, 1)
            outcome["seconds"] = round(time.time() - began, 1)
            with self._bt_lock:
                self._bt_busy = None
                self._bt_last = outcome

    def _bt_find(self, link: Any, adapter: str,
                 address: str) -> tuple[str | None, dict]:
        """Discover again until `address` is heard, for at most BT_FIND_S."""
        try:
            link.call(adapter, ADAPTER, "StartDiscovery")
        except Exception:
            pass        # already discovering is as good
        try:
            until = time.monotonic() + BT_FIND_S
            while time.monotonic() < until:
                device, props = _device_path(link.objects(), address)
                if device is not None:
                    return device, props
                time.sleep(0.5)
            return None, {}
        finally:
            try:
                link.call(adapter, ADAPTER, "StopDiscovery")
            except Exception:
                pass

    def _tool_bt_unpair(self, arguments: dict[str, Any]) -> dict[str, Any]:
        """Forget a device: its bond goes, and so does its connection."""
        address = self._bt_address(arguments)
        if address is None:
            return {"ok": False, "error": "bt_unpair wants an address like "
                                          "AA:BB:CC:DD:EE:FF"}
        with self._bt_lock:
            if self._bt_busy and self._bt_busy.get("address") == address:
                return {"ok": False, "error": f"still pairing "
                                              f"{self._bt_busy.get('name')}"}
        link = None
        try:
            link = self.BT_LINK()
            objects = link.objects()
            adapter, _props = _adapter(objects)
            device, props = _device_path(objects, address)
            if adapter is None or device is None:
                return {"ok": False, "error": f"{address} is not known to this rover"}
            name = str(props.get("Alias") or props.get("Name") or address)
            link.remove(adapter, device)
        except Exception as error:
            return {"ok": False, "error": _said(error)}
        finally:
            if link is not None:
                link.close()
        outcome = {"what": "unpair", "address": address, "name": name, "ok": True,
                   "said": "", "at": round(time.time(), 1)}
        with self._bt_lock:
            self._bt_last = outcome
        return {"ok": True, "removed": address, "name": name}
