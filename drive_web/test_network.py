"""The network panel: which access point the rover is on, and moving it.

Over a real socket rather than against the class, because the shape of these
answers is the whole contract between the daemon and the panel. The unscanned
case is the one worth pinning down: nothing polls for a scan, so a list of one is
what a console normally sees, and treating that as "there is nothing else out
there" would be wrong in the ordinary case rather than the rare one.
"""
from __future__ import annotations

import _paths  # noqa: F401 -- puts drive_web and voice_chat on the path
from test_harness import SKIP, check


def test_choosing_a_network() -> None:
    """Which access point the rover is on, and moving it to another.

    Over a real socket rather than against the class, because the shape of these
    two answers is the whole contract between the daemon and the console's network
    panel, and the mock is the only place that shape can be checked without a Pi
    and three routers.

    The unscanned case is the one worth pinning down. Nothing polls for a scan --
    it takes the radio off channel for seconds, and the caller is very likely
    asking through it -- so what a console normally sees is a list of one, and a panel that treated
    that as "there is nothing else out there" would be wrong in the ordinary case
    rather than the rare one.
    """
    try:
        import mock_rover
        import rover_tools
    except ImportError as exc:
        SKIP.append(f"choosing a network ({type(exc).__name__})")
        return

    rover = mock_rover.Rover(None, None)
    server = mock_rover.serve(rover, "127.0.0.1", 0, quiet=True)
    client = rover_tools.RoverClient(f"127.0.0.1:{server.server_address[1]}")
    try:
        quiet = client.call("wifi_status", {})
        check("it says which network it is on",
              quiet.get("connected"), "TheGreatLord")
        check("...with a signal from the driver, in dBm",
              -90 <= quiet.get("level_dbm", 0) <= -20, True)
        check("...and an address, since being associated is not being online",
              quiet.get("address"), "192.168.1.47")
        check("without a scan the list is only what was last heard",
              [n["ssid"] for n in quiet["networks"]], ["TheGreatLord"])
        check("...and that row is marked as the one in use",
              quiet["networks"][0]["in_use"], True)

        looked = client.call("wifi_status", {"scan": True})
        check("a scan finds the neighbours", len(looked["networks"]) > 1, True)
        check("...and says which of them this rover has a passphrase for",
              [n["ssid"] for n in looked["networks"] if n["configured"]],
              ["TheGreatLord", "TheMaharaja", "TheGreatViking"])

        refused = client.call("wifi_join", {"ssid": "Alister"})
        check("a network with no passphrase is refused", refused.get("ok"), False)
        check("...by name, so the panel can say why",
              "no passphrase for Alister" in refused.get("error", ""), True)
        check("and so is a join with no network at all",
              client.call("wifi_join", {}).get("ok"), False)

        moved = client.call("wifi_join", {"ssid": "TheMaharaja"})
        check("a configured network is accepted", moved.get("joining"), "TheMaharaja")
        check("...and the answer warns that the link is about to go",
              "drop" in moved.get("note", ""), True)
        after = client.call("wifi_status", {})
        check("...and afterwards it is on it", after.get("connected"), "TheMaharaja")
        check("...with the outcome kept for whoever reconnects",
              after.get("last_join", {}).get("ok"), True)
    finally:
        client.close()
        server.shutdown()
        server.server_close()


def test_signal_verdict() -> None:
    """One word for a dBm reading, which is what gets the colour in the panel."""
    try:
        import console_model
    except ImportError as exc:
        SKIP.append(f"signal verdict ({type(exc).__name__})")
        return

    verdict = console_model.wifi_verdict
    check("a strong link", verdict(-41), "good")
    check("a fading one", verdict(-68), "fair")
    check("one the wifi keeper is about to act on", verdict(-77), "poor")
    # No reading at all is the interface not reporting a signal, which is not good
    # news and must not be coloured as though it were.
    check("and no reading at all", verdict(None), "poor")


def test_the_network_panel() -> None:
    """The network panel, and what pressing `join` costs on a rover with one radio.

    It costs the page. The rover takes its link down and brings another up, so
    every connection this console holds dies with it -- which is why the panel
    warns before it happens, why a reconnect is scheduled rather than waited
    for, and why nothing here ever chooses a network on its own.
    """
    import drive_web

    session = drive_web.Session(None, 3.0, 480)
    session.show_wifi({"ok": True, "connected": "TheGreatViking",
                       "level_dbm": -38, "address": "192.168.1.80",
                       "list_age_s": 0.0,
                       "networks": [
                           {"ssid": "TheGreatViking", "signal": 84,
                            "in_use": True, "configured": True, "startup": True,
                            "security": "WPA2"},
                           {"ssid": "TheGreatLord", "signal": 50,
                            "in_use": False, "configured": True,
                            "security": "WPA2"},
                           {"ssid": "Alister", "signal": 40,
                            "in_use": False, "configured": False,
                            "security": "WPA1 WPA2"},
                           {"ssid": "Free Cafe", "signal": 30,
                            "in_use": False, "configured": False, "security": ""},
                           {"ssid": "Office", "signal": 20, "in_use": False,
                            "configured": False, "security": "WPA2 802.1X"}]})
    check("the reading is the network the rover is on",
          session.wifi["text"].startswith("TheGreatViking"), True)
    check("...and the address to reach it at",
          "192.168.1.80" in session.wifi["where"], True)
    # The other two house networks are the ones a person may choose; the
    # neighbour's is shown so the list is honest and is not offered.
    rows = {n["ssid"]: n for n in session.wifi_networks}
    offered = [n["ssid"] for n in session.wifi_networks if n["joinable"]]
    check("the stored networks it is not on are the ones offered a join",
          offered, ["TheGreatLord"])
    check("a secured network with nothing stored gets a passphrase box",
          [s for s, n in rows.items() if n["addable"]], ["Alister"])
    check("...an open one does not, and says it is open",
          (rows["Free Cafe"]["addable"], rows["Free Cafe"]["note"]), (False, "open"))
    check("...nor one that wants a user name too",
          (rows["Office"]["addable"], rows["Office"]["note"]),
          (False, "not supported"))
    check("forget is offered on a stored network that is neither in use nor the "
          "one the rover comes up on",
          [s for s, n in rows.items() if n["forgettable"]], ["TheGreatLord"])

    session.watch = object()
    sent: list[tuple] = []
    session.watch_call = lambda name, args: sent.append((name, args))
    session.wifi_add("Alister", "kettle on the hob")
    check("a typed passphrase goes to the rover with the join",
          sent[0], ("wifi_add", {"ssid": "Alister", "password": "kettle on the hob"}))
    check("...which the panel treats as a join: the rover is about to go away",
          (session.wifi["joining"], session.rejoin_at > 0.0), ("Alister", True))
    check("...and nothing the page is sent holds the passphrase",
          "kettle" in repr(session.snapshot()), False)
    session.wifi_answered("wifi_add", {"ok": False,
                                       "error": "a passphrase is 8 to 63 characters"})
    check("a refused add cancels the wait, since the link was never touched",
          (session.wifi["joining"], session.rejoin_at), (None, 0.0))
    check("...and says why", session.wifi["note"],
          "not joined: a passphrase is 8 to 63 characters")
    session.wifi_forget("TheGreatLord")
    check("forget is sent", sent[-1], ("wifi_forget", {"ssid": "TheGreatLord"}))
    session.wifi_answered("wifi_forget", {"ok": True, "forgot": "TheGreatLord"})
    check("...and said, and the list asked for again",
          (session.wifi["note"], session.wifi_at), ("forgot TheGreatLord", 0.0))
    session.wifi_answered("wifi_forget",
                          {"ok": False, "error": "TheGreatViking is the network in use"})
    check("a refused forget says why",
          session.wifi["note"], "not forgotten: TheGreatViking is the network in use")
    session.show_wifi({"ok": True, "connected": "TheGreatViking", "list_age_s": 0.0,
                       "networks": [], "last_join": {"ssid": "Alister", "ok": False,
                                                     "added": True}})
    check("a typed join that failed says nothing was kept",
          "could not join Alister -- not stored" in session.wifi["where"], True)

    session.watch = object()
    calls: list[tuple] = []
    session.watch_call = lambda name, args: calls.append((name, args))
    session.wifi_join("TheGreatLord")
    check("a join is sent", calls[0][0], "wifi_join")
    check("...and says the rover is about to go away",
          "unreachable" in session.wifi["note"], True)
    check("...so the page schedules its own reconnect",
          session.rejoin_at > 0.0, True)

    # ...and it stops claiming to be joining once the rover answers from the
    # network it was sent to, rather than saying so for the rest of the session.
    session.show_wifi({"ok": True, "connected": "TheGreatLord",
                       "level_dbm": -55, "address": "192.168.1.80",
                       "list_age_s": 0.0, "networks": []})
    check("the join is seen to have landed", session.wifi["joining"], None)
    check("...and said so", "on TheGreatLord now" in session.wifi["note"], True)


TESTS = (
    test_choosing_a_network,
    test_signal_verdict,
    test_the_network_panel,
)
