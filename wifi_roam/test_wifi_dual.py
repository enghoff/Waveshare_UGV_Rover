#!/usr/bin/env python3
"""Drive the dual-radio manager through everything it claims to survive.

    python3 test_wifi_dual.py            # anywhere: the workstation, the rover

Two kinds of check, and the order between them is the point.

**The calibration comes first and gates the rest.** `fixtures/outage-2026-08-24.log`
is 322 lines the rover wrote down while it lost the network twice in fourteen
minutes. Every sample in it is fed through the manager's own idea of whether a
link is carrying traffic, and compared with what the rover recorded at the time.
If that disagrees anywhere, this file stops: a model that grades a link
differently from the machine it is modelling has no business judging a fix, and
printing a scenario result underneath a failed calibration is how a reproduction
starts being believed for the wrong reason.

**Then the scenarios**, which are the failures the design is an argument for and
which no recording contains, because they have not happened yet: an access point
that has a signal and no path to the LAN, a router switched off underneath an
associated radio, the dongle falling off the USB bus, a rover driving out of one
cell and into another, a service address somebody else has taken, and both
radios landing on the same router.

Three assertions span every scenario in the file rather than living in one of
them, because each is about what must never happen anywhere:

- **no scenario ever asks for a radio to be switched off.** A soft rfkill block
  is saved and restored across reboots by systemd and this board has no ethernet
  socket, so an `off` that was interrupted does not cost one boot, it costs every
  boot after it. `wifi_roam.sh` carries the same assertion for the same reason.
- **no scenario ever leaves the service address on two interfaces at once.** The
  model refuses it outright rather than checking afterwards, so a manager that
  added before it removed would fail where it did it rather than somewhere later.
- **no scenario ever answers the service address out of a radio it is not on.**
  Being on the right radio is not the same as being reachable through it, and on
  2026-08-26 the difference was eleven and a half minutes.
"""
from __future__ import annotations

import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import wifi_dual
import wifi_world as W
from wifi_world import AccessPoint, SimPlatform, SimRadio, World

CHECKS = 0
FAILED = []
WORLDS = []


def check(claim, message):
    global CHECKS
    CHECKS += 1
    if not claim:
        FAILED.append(message)
        print("  FAIL: " + message)


def build(aps=None, radios=None, x=0.0, y=0.0, **kwargs):
    """A world, a platform and a manager, wired up and ready to tick."""
    aps = list(aps if aps is not None else W.house().aps)
    known = [ap.ssid for ap in aps]
    if radios is None:
        radios = [SimRadio("wlan0", usb=False, mac="ac:6a:a3:41:53:53",
                           knows=known),
                  SimRadio("wlan1", usb=True, mac="00:2e:2d:30:74:d0",
                           bands=("2.4",), knows=known)]
    world = World(aps, radios, x=x, y=y)
    WORLDS.append(world)
    platform = SimPlatform(world)
    manager = wifi_dual.Manager(platform, status_path=None, **kwargs)
    return world, platform, manager


def settle(manager, world, seconds, tick=1.0):
    """Run the loop for a while, and say how much of it carried nothing."""
    dark = 0.0
    end = world.t + seconds
    while world.t < end:
        manager.tick()
        if manager.active is None or not manager.active.usable:
            dark += tick
        world.advance(tick)
    return dark


# --- the calibration, which gates everything below it ------------------------

def calibration():
    here = os.path.dirname(os.path.abspath(__file__))
    path = os.path.join(here, "fixtures", "outage-2026-08-24.log")
    trace = W.Trace(W.parse_netwatch(path))
    agreed, total, wrong = W.replay_health(trace)
    print("calibration: %d of %d recorded samples graded as the rover graded them"
          % (agreed, total))
    for entry in wrong[:5]:
        print("   disagreed at %s: recorded %s, model says usable=%s "
              "(sig %s, rtt %s, ip %s)" % entry)
    check(total > 50, "the recording should have samples in it")
    check(agreed == total,
          "the model disagreed with the recording on %d samples" % (total - agreed))
    return trace, agreed == total


def the_recorded_outage(trace):
    """What two radios would have done with the outage the rover actually had."""
    result = W.replay_dual(trace)
    print("recorded outage: %.0f s carrying nothing became %.0f s, %d failovers"
          % (result["recorded_dark_s"], result["dark_s"], result["switches"]))
    check(result["recorded_dark_s"] >= 200,
          "the fixture should contain the real outage")
    check(result["dark_s"] == 0,
          "two radios still spent %.0f s carrying nothing" % result["dark_s"])
    check(result["switches"] <= 4,
          "%d failovers over fourteen minutes is thrashing" % result["switches"])
    check(any(garp[2] == wifi_dual.SERVICE_IP for garp in result["garps"]),
          "the service address should have been announced")


# --- the scenarios -----------------------------------------------------------

def two_radios_land_on_two_routers():
    world, platform, manager = build()
    settle(manager, world, 120)
    routers = {wifi_dual.router(radio.link.ssid)
               for radio in manager.radios if radio.link.associated}
    check(len(routers) == 2,
          "both radios ended up on %s" % (routers or "nothing"))
    check(manager.active is not None and not manager.active.usb,
          "the onboard radio should carry the traffic when it can")
    check(manager.standby is not None and manager.standby.usb,
          "the dongle should be the standby")
    check(world.claimed.get(wifi_dual.SERVICE_IP) == manager.active.iface,
          "the service address should sit on the active radio")


def signal_but_no_lan():
    """The failure the document singles out, and the one RSSI cannot see."""
    aps = W.house().aps
    world, platform, manager = build(aps=aps)
    settle(manager, world, 90)
    was = manager.active.iface
    on = manager.active.link.ssid
    for ap in world.aps:
        if ap.ssid == on:
            ap.reaches_lan = False
    broke_at = world.t
    dark = settle(manager, world, 120)
    check(manager.active.iface != was,
          "traffic stayed on %s after it stopped reaching the LAN" % was)
    # Not zero any more, and that is the price of SURRENDER_WORSE_S rather than
    # a regression that crept in. The radio left here is forty decibels worse --
    # in this house they always are, being on routers at 2 m and 33 m -- so the
    # manager now waits out the failure before surrendering to it. What must not
    # grow is the wait itself, which is what the two bounds below hold.
    check(dark <= wifi_dual.SURRENDER_WORSE_S + wifi_dual.DEAD_PINGS + 4,
          "%.0f s with nothing carrying traffic at all" % dark)
    check(manager.active.usable, "the radio it moved to is not usable either")
    # The number that matters is not how long the manager was confused -- it was
    # never confused -- but how many seconds of the rover's traffic went into a
    # link that had stopped delivering it. That is the consecutive-loss run, and
    # bounding it here is what stops somebody widening it back to the ten-second
    # window and putting ten seconds of retransmits into every hard failure.
    moved_at = manager.switched_at
    check(moved_at - broke_at
          <= wifi_dual.DEAD_PINGS + 2 + wifi_dual.SURRENDER_WORSE_S,
          "took %.0f s to notice the active link had stopped delivering"
          % (moved_at - broke_at))


def a_link_that_goes_slow_rather_than_dead():
    """The shape a real degradation turned out to have, which is not the one above.

    `signal_but_no_lan` makes an access point stop answering altogether, and
    that scenario is worth having -- but it is not what the hardware did when it
    was actually broken on 2026-08-25. The test poisoned the gateway's MAC on
    the active radio expecting silence, and got this instead:

        p +3.9   wlan0  -24 dBm  rtt 4.3 ms    loss 0%
        p +6.0   wlan0  -22 dBm  rtt 313.8 ms  loss 0%
        p +12.0  wlan0  -22 dBm  rtt 313.8 ms  loss 10%   -> moved to wlan1

    The pings kept coming back. Frames addressed to a MAC the upstream bridge
    has never seen are *flooded* to every port rather than dropped, so they
    reached the gateway anyway -- slowly, and at a rate the bridge was in no
    hurry about. So the link was not dead, it was two orders of magnitude
    slower, and what caught it was the latency penalty rather than the loss.

    That is exactly the case the document warns about, arrived at from an
    unexpected direction: an access point with an excellent signal that is not a
    usable path. A model that can only make an access point die cannot show
    whether the manager handles it, so this scenario makes one go slow.
    """
    world, platform, manager = build()
    settle(manager, world, 150)
    was = manager.active.iface
    on = manager.active.link.ssid
    for ap in world.aps:
        if ap.ssid == on:
            ap.latency_ms = 314.0
    broke_at = world.t
    dark = settle(manager, world, 180)
    check(manager.active.iface != was,
          "traffic stayed on a link answering in 314 ms")
    check(dark == 0, "%.0f s carrying nothing while moving off a slow link" % dark)
    moved_at = manager.switched_at
    check(moved_at - broke_at <= 20,
          "took %.0f s to move off a link two orders of magnitude slower"
          % (moved_at - broke_at))
    # And the reason has to be the latency, not something incidental: a link
    # with a perfect signal and no loss must be leavable on its round trip
    # alone, or the whole scoring scheme is decoration.
    check(any("better" in entry["why"] for entry in manager.history),
          "it did not move on the score")


def the_active_router_is_switched_off():
    world, platform, manager = build()
    settle(manager, world, 90)
    was = manager.active.iface
    for ap in world.aps:
        if wifi_dual.router(ap.ssid) == wifi_dual.router(manager.active.link.ssid):
            ap.up = False
    settle(manager, world, 150)
    check(manager.active.iface != was, "traffic stayed on a router that is off")
    check(manager.active.usable, "nothing usable after the router went off")
    others = [radio for radio in manager.radios if radio is not manager.active]
    check(all(radio.link.ssid != was for radio in others),
          "the demoted radio was left on the router that is off")


def the_dongle_falls_off_the_bus():
    """It has done this twice. Now that it is the spare, it should cost nothing."""
    world, platform, manager = build()
    settle(manager, world, 90)
    active_before = manager.active.iface
    check(active_before == "wlan0", "the onboard radio should have been active")
    world.radios["wlan1"].present = False
    dark = settle(manager, world, 120)
    check(dark == 0, "%.0f s carrying nothing because the spare vanished" % dark)
    check(manager.active.iface == "wlan0", "the traffic moved for no reason")
    check(manager.radio("wlan1").gone, "the manager did not notice it had gone")
    world.radios["wlan1"].present = True
    settle(manager, world, 90)
    check(not manager.radio("wlan1").gone, "the manager did not notice it return")


def the_dongle_is_the_only_one_left():
    """One radio is not a reason to refuse to work."""
    known = [ap.ssid for ap in W.house().aps]
    radios = [SimRadio("wlan1", usb=True, mac="00:2e:2d:30:74:d0",
                       bands=("2.4",), knows=known)]
    world, platform, manager = build(radios=radios)
    settle(manager, world, 120)
    check(manager.active is not None and manager.active.iface == "wlan1",
          "a single radio should still carry the traffic")
    check(manager.standby is None, "there is no standby with one radio")
    check(world.claimed.get(wifi_dual.SERVICE_IP) == "wlan1",
          "the service address should still be held")


def both_radios_start_on_one_router():
    """Two adapters on one box looks like redundancy and is not."""
    known = [ap.ssid for ap in W.house().aps]
    radios = [SimRadio("wlan0", mac="ac:6a:a3:41:53:53", knows=known),
              SimRadio("wlan1", usb=True, mac="00:2e:2d:30:74:d0",
                       bands=("2.4",), knows=known)]
    radios[0].ssid, radios[0].address = "TheGreatLord 5G", "192.168.1.141"
    radios[1].ssid, radios[1].address = "TheGreatLord", "192.168.1.142"
    world, platform, manager = build(radios=radios)
    manager.tick()                  # long enough to pick one and leave it alone
    world.advance(1.0)
    carrying = manager.active.iface
    stays_on = manager.active.link.ssid
    settle(manager, world, 150)
    routers = [wifi_dual.router(radio.link.ssid) for radio in manager.radios
               if radio.link.associated]
    check(len(set(routers)) == 2,
          "both radios are still on %s" % (routers or "nothing"))
    # Which of the two ends up carrying traffic is not the interesting part --
    # the dongle's 2.4 GHz reading beats the onboard radio's 5 GHz one by more
    # than the tiebreak here, and legitimately so, since a signal is a signal.
    # What must hold is that whichever one was chosen was not the one moved.
    check(manager.active.iface == carrying,
          "the traffic moved while sorting the radios out")
    check(manager.active.link.ssid == stays_on,
          "it re-associated the radio that was carrying traffic")


def a_rover_that_drives_away():
    world, platform, manager = build()
    settle(manager, world, 120)
    # Out of TheGreatLord's room and into TheMaharaja's, over a minute.
    for step in range(12):
        world.x = 2.4 + (33.0 - 2.4) * (step + 1) / 12.0
        settle(manager, world, 10)
    settle(manager, world, 180)
    check(manager.active is not None and manager.active.usable,
          "nothing was carrying traffic after driving across the house")
    near = wifi_dual.router(manager.active.link.ssid)
    check(near == "TheMaharaja",
          "ended up on %s rather than the router it drove to" % near)


def somebody_else_has_the_service_address():
    world, platform, manager = build()
    world.arp_owner[wifi_dual.SERVICE_IP] = "de:ad:be:ef:00:01"
    settle(manager, world, 60)
    check(manager.service_ip is None,
          "it claimed an address another host was answering for")
    check(wifi_dual.SERVICE_IP not in world.claimed,
          "the address was configured anyway")
    check(manager.active is not None and manager.active.usable,
          "refusing the service address should not stop it working")
    check("already answered" in manager.note, "it did not say why")


def the_spare_stays_put():
    """The spare must not wander between routers because a scan changed its mind.

    This scenario exists because the rover did it. Five minutes after the
    manager was first armed, the journal read:

        08:58:01  moving wlan1 from TheMaharaja to TheGreatViking at -50 dBm
        08:58:30  moving wlan1 from TheGreatViking to TheMaharaja at -66 dBm

    Twenty-nine seconds apart, in a house where nothing had moved. That is the
    scan noise this repository has measured before -- the same access point read
    twenty-three decibels apart inside a minute -- and it matters more here than
    it looks: a spare that is re-associating is a spare that is not ready, and
    each of those moves costs about ten seconds of association and DHCP. A
    standby flapping every thirty seconds is a hot spare a third of the time.

    So: nothing moving, nothing failing, ten minutes. The spare is allowed to
    settle somewhere at the start and then expected to stay there.
    """
    world, platform, manager = build()
    settle(manager, world, 120)
    spare = manager.standby
    check(spare is not None and spare.link.associated,
          "the spare should have settled somewhere first")
    landed_on = spare.link.ssid
    before = len([pin for pin in platform.pins if pin[1] == spare.iface])

    # First the general case: nothing moving, nothing failing, ten minutes of
    # ordinary scan jitter.
    settle(manager, world, 600)
    moves = len([pin for pin in platform.pins if pin[1] == spare.iface]) - before
    check(moves <= 1,
          "the spare re-associated %d times in ten quiet minutes" % moves)

    check(manager.standby is not None and manager.standby.link.associated,
          "the spare ended up associated with nothing")
    check(wifi_dual.router(manager.standby.link.ssid)
          != wifi_dual.router(manager.active.link.ssid),
          "the two radios ended up on one router")
    check(landed_on is not None, "it never landed anywhere to begin with")


def one_wrong_scan_reading():
    """The fault the rover produced, stated exactly rather than waited for.

    An hour after the manager was first armed the journal read:

        08:58:01  moving wlan1 from TheMaharaja to TheGreatViking at -50 dBm
        08:58:30  moving wlan1 from TheGreatViking to TheMaharaja at -66 dBm

    Twenty-nine seconds apart, in a house where nothing had moved. Twelve
    consecutive standby scans measured afterwards put TheGreatViking between -74
    and -84, so the -50 was one sample wrong by nearly thirty decibels. Two
    re-associations bought with it, and a spare that is re-associating is a spare
    that is not ready.

    So: one scan, one access point, twenty-six decibels out. The world's own
    occasional wild readings are switched off for this run, because one of them
    landing on the same scan as the injected one raised the reading the spare was
    being compared *against* and made a manager that had not been fixed look as
    though it had.
    """
    world, platform, manager = build()
    world.excursions = False
    settle(manager, world, 150)
    spare = manager.standby
    check(spare is not None and spare.link.associated,
          "the spare should have settled somewhere first")
    held = spare.link.ssid
    elsewhere = next(
        (entry.ssid for entry in spare.seen
         if wifi_dual.router(entry.ssid) not in (
             wifi_dual.router(manager.active.link.ssid),
             wifi_dual.router(held))),
        None)
    check(elsewhere is not None, "there should be a third router to lie about")
    if not elsewhere:
        return
    before = len([pin for pin in platform.pins if pin[1] == spare.iface])
    world.lie_once(elsewhere, +26)
    settle(manager, world, 180)
    after = len([pin for pin in platform.pins if pin[1] == spare.iface])
    check(after == before,
          "one wrong scan reading moved the spare off %s onto %s" % (held, elsewhere))
    check(manager.standby.link.ssid == held,
          "the spare did not stay where it was")


def somebody_asks_for_a_network(tmp):
    """Choosing a network from the console, without dropping the console.

    The old `wifi_join` had one meaning: take the link down, bring another one
    up, and hope the person reconnects. With two radios it can mean something
    much better, and this is the check that it does -- the spare is moved, the
    traffic follows only once the spare is genuinely working, and the radio the
    request arrived through is never touched.
    """
    world, platform, manager = build(request_path=tmp, sticky_s=200)
    settle(manager, world, 120)
    carrying = manager.active.iface
    spare = manager.standby.iface
    check(carrying != spare, "there should be two radios to choose between")

    with open(tmp, "w") as handle:
        handle.write('{"ssid": "TheGreatViking", "carry": true}\n')
    dark = settle(manager, world, 90)

    check(not os.path.exists(tmp), "the request file should be consumed")
    check(dark == 0,
          "%.0f s carrying nothing while changing network by hand" % dark)
    check(manager.active.iface == spare,
          "the traffic did not move onto the radio that was asked to move")
    check(manager.active.link.ssid == "TheGreatViking",
          "ended up on %s rather than the network somebody asked for"
          % manager.active.link.ssid)
    check(manager.radio(carrying).link.ssid is not None,
          "the radio that was carrying traffic got re-associated anyway")

    # And it is not argued with while the choice is fresh, even though the
    # scoring would put that radio somewhere much louder.
    settle(manager, world, 60)
    check(manager.active.link.ssid == "TheGreatViking",
          "the scoring overrode a choice a person had just made")

    with open(tmp, "w") as handle:
        handle.write('{"ssid": "NoSuchNetwork"}\n')
    settle(manager, world, 30)
    check("no passphrase" in manager.note,
          "a network this rover has no passphrase for should be refused")


def nothing_works_at_all():
    """The dead-man. Worse behaviour, deliberately, rather than a stranded rover."""
    world, platform, manager = build(deadman_s=30)
    settle(manager, world, 90)
    check(any(radio.pinned for radio in world.radios.values()),
          "the radios should be pinned while the manager is happy")
    for ap in world.aps:
        ap.up = False
    settle(manager, world, 120)
    check(manager.surrendered, "it did not give up when nothing worked")
    check(all(radio.enabled == radio.knows for radio in world.radios.values()),
          "a radio was left with its other networks disabled")
    check(wifi_dual.SERVICE_IP not in world.claimed,
          "the service address was left on a dead interface")
    for ap in world.aps:
        ap.up = True
    settle(manager, world, 150)
    check(not manager.surrendered, "it never took the link back")
    check(manager.active is not None and manager.active.usable,
          "it did not recover once the routers came back")


def stopping_frees_the_radios():
    world, platform, manager = build()
    settle(manager, world, 90)
    manager.restore()
    check(all(radio.enabled == radio.knows for radio in world.radios.values()),
          "a radio was left pinned after the manager stopped")
    check(wifi_dual.SERVICE_IP not in world.claimed,
          "the service address outlived the manager")


def the_documents_worked_example():
    """The document's own numbers must come out the way it says they do."""
    ap1 = wifi_dual.link_score(-56, 5.0, 0.0)
    ap2 = wifi_dual.link_score(-49, 80.0, 15.0)
    check(ap1 > ap2,
          "the document's AP1 (%.1f) should beat its AP2 (%.1f)" % (ap1, ap2))
    check(ap1 - ap2 > wifi_dual.MARGIN_DB,
          "and by more than the switching margin, or it would never move")
    check(wifi_dual.link_score(-256, 2.0, 0.0) is None,
          "-256 is not a signal 200 dB down, it is no signal reported")
    check(wifi_dual.router("TheGreatLord 5G") == "TheGreatLord",
          "the two bands of one router are one access point")
    check(wifi_dual.router("TheMaharaja") == "TheMaharaja",
          "and a name without a band suffix is left alone")
    check(wifi_dual.band_of(5805) == "5" and wifi_dual.band_of(2437) == "2.4",
          "the bands should be told apart by frequency")


def scoring_prefers_the_onboard_radio_only_in_ties():
    onboard = wifi_dual.Radio("wlan0", usb=False)
    dongle = wifi_dual.Radio("wlan1", usb=True)
    for radio, level in ((onboard, -60), (dongle, -60)):
        radio.operstate = "up"
        radio.address = "192.168.1.1"
        radio.held_dbm = level
        radio.pings = [2.0]
    check(onboard.effective() > dongle.effective(),
          "an even match should go to the onboard radio")
    dongle.held_dbm = -50
    check(dongle.effective() > onboard.effective(),
          "ten decibels should beat the tiebreak")


def the_service_address_follows_the_failover():
    """2026-08-26, 16:50: the failover was right and the address stayed behind.

    The radio carrying the traffic kept its association and quietly stopped
    delivering anything. The manager did the right thing and did it in seconds:
    it moved the traffic to the standby, and moved the service address with it.
    The rover then went on answering that address out of the radio that had just
    died, for eleven and a half minutes, until an unrelated re-association moved
    the traffic back and fixed it by accident.

    So what is checked here is not where the address sits, which was right the
    whole time, but which radio a reply to it leaves by.
    """
    world, platform, manager = build()
    settle(manager, world, 90)
    was = manager.active.iface
    on = manager.active.link.ssid
    check(world.claimed.get(wifi_dual.SERVICE_IP) == was,
          "the service address did not start on the radio carrying traffic")
    for ap in world.aps:
        if ap.ssid == on:
            ap.reaches_lan = False
    settle(manager, world, 120)
    now = manager.active.iface
    check(now != was, "traffic stayed on the radio that stopped delivering")
    check(world.claimed.get(wifi_dual.SERVICE_IP) == now,
          "the service address did not move with the traffic")
    leaves_by = world.egress(wifi_dual.SERVICE_IP)
    check(leaves_by == now,
          "the service address sits on %s and is answered out of %s, which is "
          "the radio that just died" % (now, leaves_by))


def a_lease_that_changes_under_the_active_radio():
    """2026-08-27: a DHCP renewal emptied the table the service address uses.

    The rover spent that afternoon in the state this builds: the onboard radio
    associated and passing nothing, the dongle carrying everything perfectly
    well, and therefore no failover due for as long as that lasted. Meanwhile
    the dongle's lease kept changing -- .47, .100, .47, .100, .47 in eight
    minutes, because a second DHCP server answers on this LAN alongside the
    router -- and every one of those changes took the dongle's routing table
    away with it, measured on the board in `kernel_route_lifetime.sh`.

    The manager wrote those routes only when traffic moved between radios. So
    the service address's rule was left pointing at an empty table, which falls
    through to the main table, which answers out of whichever radio comes first
    -- the sick one. Nothing was going to rebuild it, because the thing that
    rebuilt it was the failover that was never coming.
    """
    world, platform, manager = build()
    settle(manager, world, 120)
    # Put the rover in the state it was actually in: the onboard radio is
    # associated and useless, and the dongle is carrying the traffic.
    sick = manager.active
    for ap in world.aps:
        if ap.ssid == sick.link.ssid:
            ap.reaches_lan = False
    # Long enough to cover the wait before an emergency move to a much worse
    # radio; this scenario is about what happens after the failover, not about
    # how quickly it comes.
    settle(manager, world, 30 + wifi_dual.SURRENDER_WORSE_S)
    carrier = manager.active.iface
    check(carrier != sick.iface, "the traffic never left the radio that died")
    check(world.egress(wifi_dual.SERVICE_IP) == carrier,
          "the service address was already answered out of the wrong radio")

    was = manager.active.address
    now = world.renew(carrier)
    check(was != now, "the model did not actually change the lease")
    settle(manager, world, 20)

    check(manager.active.iface == carrier,
          "the renewal moved the traffic, which is not what a renewal is")
    check(world.egress(wifi_dual.SERVICE_IP) == carrier,
          "after the renewal the service address sits on %s and is answered "
          "out of %s" % (carrier, world.egress(wifi_dual.SERVICE_IP)))

    # The window that remains is one tick, and it is not closable from the
    # manager: the kernel takes the routes the instant the lease lands and
    # tells nobody. A second of replies leaving by the wrong radio on a bridged
    # LAN is a different animal from the eleven and a half minutes of
    # 2026-08-26, and this is where that difference is held.
    longest = run = 0
    previous = None
    for when, _address, _holder, _leaves in world.stranded:
        run = run + 1 if previous is not None and when - previous <= 1.5 else 1
        longest = max(longest, run)
        previous = when
    check(longest <= 2,
          "the service address was answered out of the wrong radio for %d "
          "consecutive ticks after the renewal" % longest)
    # Cleared deliberately, having been checked here with a bound. The sweep
    # below stays at zero tolerance for the version of this fault that lasts.
    world.stranded = []


def the_service_address_is_answered_out_of_the_radio_it_sits_on():
    """The assertion that spans the file, and the one the rover paid for.

    Every world every scenario above ran in was asked, on every tick, whether
    the address callers use would have been answered out of the radio holding
    it. A failover that moves the address and not its routing looks perfect in
    every other check here -- right radio, right address, traffic flowing -- and
    is completely unreachable from outside.
    """
    guilty = [world for world in WORLDS if world.stranded]
    for world in guilty[:2]:
        when, address, holder, leaves_by = world.stranded[0]
        check(False,
              "at t=%.0fs %s sat on %s and was answered out of %s (%d tick(s) "
              "like it)" % (when, address, holder, leaves_by,
                            len(world.stranded)))
    check(not guilty,
          "%d of %d scenarios answered the service address out of the wrong "
          "radio" % (len(guilty), len(WORLDS)))
    check(bool(WORLDS), "no scenario ran, so this assertion proves nothing")


def a_ten_second_blip_with_a_much_worse_spare():
    """2026-08-27: a router that went quiet cost the rover three failovers.

    The rover had been driven across the house and parked beside TheGreatLord at
    -20 dBm, with the dongle hearing TheMaharaja at -78. The gateway then went
    quiet for about ten seconds at a time. Each time, the manager handed the
    service address to the dongle, the dongle lost the gateway itself moments
    later and handed it back, and every one of those moves rewrote the ARP cache
    of every device in the house -- over a fault that healed on its own.

    The rule under test is that an emergency move looks at what it is moving to.
    Ten seconds of silence from a strong link is not a reason to hand the rover
    to one fifteen decibels down; the other half of the rule, that thirty still
    is, is `a_failure_that_does_not_heal_still_moves` below.
    """
    world, platform, manager = build(aps=W.house().aps)
    settle(manager, world, 150)
    was = manager.active.iface
    on = manager.active.link.ssid
    switches_before = manager.switches
    spare = manager.standby
    check(spare is not None and spare.effective() is not None,
          "the scenario needs a standby with a grade of its own")
    check(manager.active.effective() - spare.effective()
          >= wifi_dual.SURRENDER_DROP_DB,
          "the spare is not enough worse for this scenario to mean anything")

    for ap in world.aps:
        if ap.ssid == on:
            ap.reaches_lan = False
    settle(manager, world, 10)
    for ap in world.aps:
        if ap.ssid == on:
            ap.reaches_lan = True
    settle(manager, world, 60)

    check(manager.active.iface == was,
          "the traffic moved to a much worse radio over a ten-second blip")
    check(manager.switches == switches_before,
          "%d failover(s) for a fault that healed itself"
          % (manager.switches - switches_before))
    check(manager.active.usable, "the radio it stayed on never recovered")


def a_failure_that_does_not_heal_still_moves():
    """Waiting is not refusing, which is the half that keeps the rover safe.

    The same setup as above with the access point left broken. However much
    worse the only remaining radio is, the traffic has to end up on it: a rover
    on a poor link beats a rover on none, and the wait is bounded for exactly
    that reason.
    """
    world, platform, manager = build(aps=W.house().aps)
    settle(manager, world, 150)
    was = manager.active.iface
    on = manager.active.link.ssid
    for ap in world.aps:
        if ap.ssid == on:
            ap.reaches_lan = False
    broke_at = world.t
    settle(manager, world, wifi_dual.SURRENDER_WORSE_S + 60)
    check(manager.active.iface != was,
          "the traffic never left a link that stayed dead")
    check(manager.active.usable, "it moved to a radio that is not usable")
    moved_at = manager.switched_at
    bound = wifi_dual.SURRENDER_WORSE_S + wifi_dual.DEAD_PINGS + 4
    check(moved_at - broke_at <= bound,
          "took %.0f s to give up on a dead link, with a bound of %.0f"
          % (moved_at - broke_at, bound))


def a_router_that_will_not_hold_the_standby():
    """2026-08-26: the spare unassociated for minutes, with strong routers listed.

    Holding a radio on one network is done by disabling every other one, which
    is what `select_network` means. So a radio held on a router that will not
    keep it has nowhere to fall back to: it joins, loses carrier, retries the
    same access point, and the manager re-pins it there after every scan because
    it is still the loudest router the other radio is not on. The console shows
    a spare that is "not associated" beside a list of networks at full strength
    that it is not permitted to try.

    Being associated to something beats being on the router the placement rules
    would prefer, so what is checked is that it ends up on the air at all.
    """
    world, platform, manager = build()
    settle(manager, world, 120)
    spare = manager.standby
    check(spare is not None and spare.link.associated,
          "the spare never associated at all, so this proves nothing")
    iface, refused_by = spare.iface, spare.link.ssid
    for ap in world.aps:
        if ap.ssid == refused_by:
            ap.refuses.add(iface)
    settle(manager, world, 420)
    spare = manager.radio(iface)
    check(spare.link.associated,
          "%s spent seven minutes unassociated, still held on %s with every "
          "other network disabled" % (iface, refused_by))
    check(spare.link.ssid != refused_by,
          "%s is still on %s, which will not keep it" % (iface, refused_by))


def nothing_ever_switches_a_radio_off():
    """The assertion that spans the whole file, and keeps the rover rebootable.

    Two forms of it, because the strong one is easy to weaken by accident. The
    manager's own source is searched for any way of asking for a radio to be
    switched off -- there is deliberately no method on
    :class:`wifi_dual.Platform` that could -- and every world every scenario
    above ran in is then asked whether anything reached for one anyway.
    """
    import ast

    here = os.path.dirname(os.path.abspath(__file__))
    source = open(os.path.join(here, "wifi_dual.py"), encoding="utf-8").read()
    tree = ast.parse(source)
    # Docstrings are exempt, and have to be: the file explains at length why it
    # never blocks a radio, and a search of the raw text finds that explanation
    # and calls it the offence. What is searched is the strings the code could
    # actually hand to something.
    docstrings = set()
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.ClassDef, ast.FunctionDef,
                             ast.AsyncFunctionDef)):
            first = node.body[0] if node.body else None
            if (isinstance(first, ast.Expr)
                    and isinstance(first.value, ast.Constant)
                    and isinstance(first.value.value, str)):
                docstrings.add(id(first.value))
    live = [node.value for node in ast.walk(tree)
            if isinstance(node, ast.Constant) and isinstance(node.value, str)
            and id(node) not in docstrings]
    for phrase in ("rfkill", "radio wifi off", "block wifi"):
        offenders = [text for text in live if phrase in text]
        check(not offenders,
              "wifi_dual.py can pass %r to something, and that survives a "
              "reboot: %s" % (phrase, offenders[:2]))
    check(all(not world.rfkill_asked for world in WORLDS),
          "a scenario asked for a radio to be switched off")
    check(bool(WORLDS), "no scenario ran, so this assertion proves nothing")


class FakeNmcli:
    """A NetworkManager that answers `nmcli` and remembers what it was told.

    Only the four questions the manager asks, and it is deliberately not a
    generous fake: a profile that is not named after its network, and one pinned
    to the other radio, because both of those exist on the rover and both would
    be invisible to a fake that named everything tidily.
    """

    def __init__(self, profiles=None, on=None):
        # profile name -> (ssid, pinned interface or "")
        self.profiles = profiles if profiles is not None else {
            "TheGreatLord": ("TheGreatLord", ""),
            "TheMaharaja": ("TheMaharaja", ""),
            "TheGreatViking-dongle": ("TheGreatViking", ""),
        }
        self.on = dict(on or {})        # iface -> ssid it is associated with
        self.calls = []

    def run(self, argv, timeout=10):
        self.calls.append(list(argv))
        if argv[0] != "nmcli":
            return None
        args = argv[1:]
        if args[:4] == ["-t", "-f", "TYPE,NAME", "con"]:
            rows = ["802-11-wireless:" + name for name in self.profiles]
            rows.append("802-3-ethernet:Wired connection 1")
            return "\n".join(rows) + "\n"
        if args[:2] == ["-g", "802-11-wireless.ssid"]:
            return self.profiles.get(args[-1], ("", ""))[0] + "\n"
        if args[:2] == ["-g", "connection.interface-name"]:
            return self.profiles.get(args[-1], ("", ""))[1] + "\n"
        if "con" in args and "up" in args:
            name = args[args.index("up") + 1]
            iface = args[args.index("ifname") + 1]
            ssid = self.profiles.get(name, ("", ""))[0]
            if not ssid:
                return None
            # Whatever else held this profile loses it: NetworkManager will not
            # have one connection active on two devices.
            for other, was in list(self.on.items()):
                if was == ssid and other != iface:
                    del self.on[other]
            self.on[iface] = ssid
            return "Connection successfully activated\n"
        if args[:3] == ["dev", "set"] or args[:2] == ["dev", "set"]:
            return "\n"
        return "\n"


def _nm_platform(fake, on=None):
    """A real Platform with its shell replaced and its stack forced."""
    platform = wifi_dual.Platform()
    platform._backend = "nm"
    platform._run = fake.run
    platform.link = lambda iface: wifi_dual.Link(
        ssid=fake.on.get(iface)) if fake.on.get(iface) else wifi_dual.Link()
    return platform


def the_networkmanager_translation():
    """The three calls that differ between the two boards this has run on.

    Everything else in Platform is `iw`, `/sys`, `/proc`, `ip` and raw sockets,
    which mean the same thing on netplan and on NetworkManager. These three do
    not, and they are what the port is, so they are checked directly rather than
    through the world model -- the model stands in for the whole of Platform and
    would never exercise either translation.
    """
    fake = FakeNmcli()
    platform = _nm_platform(fake)

    known = platform.networks("wlP1p1s0")
    check(set(known) == {"TheGreatLord", "TheMaharaja", "TheGreatViking"},
          "the networks a radio may use came back as %s" % sorted(known))
    check(known.get("TheGreatViking") == "TheGreatViking-dongle",
          "a profile that is not named after its network was not found by the "
          "network it is for")

    # A profile pinned to the other radio is one this radio cannot use, and
    # offering it would produce a placement that fails every time it is tried.
    fake.profiles["TheMaharaja"] = ("TheMaharaja", "wlx002e2d3074d0")
    platform._profiles.clear()
    known = platform.networks("wlP1p1s0")
    check("TheMaharaja" not in known,
          "a profile pinned to another interface was offered to this one")
    known = platform.networks("wlx002e2d3074d0")
    check("TheMaharaja" in known,
          "a profile pinned to this interface was hidden from it")

    # Pinning is `con up` on the profile, naming the radio.
    fake.profiles["TheMaharaja"] = ("TheMaharaja", "")
    platform._profiles.clear()
    fake.calls = []
    check(platform.pin("wlP1p1s0", "TheGreatViking"),
          "pinning a network the rover has a passphrase for failed")
    ups = [c for c in fake.calls if "up" in c]
    check(len(ups) == 1, "pinning made %d activations" % len(ups))
    check("TheGreatViking-dongle" in ups[0] and "wlP1p1s0" in ups[0],
          "the activation named %s" % ups[0])
    check(fake.on.get("wlP1p1s0") == "TheGreatViking",
          "the radio did not end up on the network it was pinned to")

    # And the call that must not happen. `nmcli con up` on a connection that is
    # already active reactivates it, taking the association down for the ten
    # seconds it spends coming back -- so a manager restarting, which re-pins
    # everything it finds, would interrupt both radios for no reason.
    fake.calls = []
    check(platform.pin("wlP1p1s0", "TheGreatViking"),
          "re-pinning a radio that is already there reported failure")
    check(not [c for c in fake.calls if "up" in c],
          "a radio already on the right network was reactivated anyway")

    # A network with no profile is refused rather than attempted.
    fake.calls = []
    check(not platform.pin("wlP1p1s0", "Alister"),
          "pinning a network with no passphrase claimed to work")
    check(not [c for c in fake.calls if "up" in c],
          "an activation was attempted for a network with no profile")

    # Releasing disables nothing, because pinning disabled nothing. What it has
    # to do is make sure NetworkManager may reconnect the radio itself, which is
    # the safety net that replaces `enable_network all`.
    fake.calls = []
    check(platform.release("wlP1p1s0"), "releasing a radio reported failure")
    said = " ".join(fake.calls[-1]) if fake.calls else ""
    check("autoconnect" in said and "yes" in said,
          "releasing a radio did not hand it back to NetworkManager: %s" % said)


def the_service_address_survives_a_dhcp_renewal_stripping_it():
    """NetworkManager removes addresses it did not put there. This puts it back.

    The port found this rather than predicting it: NetworkManager owns the
    addresses on the devices it manages, and reconfiguring one -- which a DHCP
    renewal does, several times an hour in this house -- takes off anything it
    does not know about. The service address is added with `ip` on purpose,
    because it belongs to whichever radio is *active* while a profile moves
    between radios independently, so it cannot be written into a profile and
    left there.

    Under netplan this never happened and the check would have looked like
    paranoia. Here it is the difference between a stable address and one that
    quietly disappears mid-afternoon.
    """
    world, platform, manager = build()
    settle(manager, world, 90)
    on = manager.active.iface
    check(world.claimed.get(wifi_dual.SERVICE_IP) == on,
          "the service address did not start on the radio carrying traffic")

    # Something else takes it off, without telling anybody.
    del world.claimed[wifi_dual.SERVICE_IP]
    garps = len(platform.garps)
    settle(manager, world, 3)
    check(world.claimed.get(wifi_dual.SERVICE_IP) == on,
          "the service address was taken off %s and never came back" % on)
    check(len(platform.garps) > garps,
          "the address came back without telling the rest of the house, so "
          "nothing would send traffic to it")
    check(world.egress(wifi_dual.SERVICE_IP) == on,
          "the address came back but replies to it leave by the wrong radio")

def main():
    print("wifi_dual: driving the manager round a model of the house\n")
    trace, calibrated = calibration()
    if not calibrated:
        print("\ncalibration failed, so no scenario below is worth reading.")
        print("%d checks, %d failed" % (CHECKS, len(FAILED)))
        return 1
    print()

    the_recorded_outage(trace)
    print()

    scenarios = [
        ("both radios find two routers", two_radios_land_on_two_routers),
        ("a signal with no path to the LAN", signal_but_no_lan),
        ("a link that goes slow rather than dead",
         a_link_that_goes_slow_rather_than_dead),
        ("the active router is switched off", the_active_router_is_switched_off),
        ("the dongle falls off the USB bus", the_dongle_falls_off_the_bus),
        ("only the dongle is left", the_dongle_is_the_only_one_left),
        ("both radios start on one router", both_radios_start_on_one_router),
        ("the rover drives across the house", a_rover_that_drives_away),
        ("the spare stays put when nothing changes", the_spare_stays_put),
        ("one wrong scan reading", one_wrong_scan_reading),
        ("somebody else has the service address",
         somebody_else_has_the_service_address),
        ("somebody asks for a network", lambda: somebody_asks_for_a_network(
            os.path.join(tempfile.mkdtemp(), "wifi-dual.request"))),
        ("nothing works at all", nothing_works_at_all),
        ("stopping frees the radios", stopping_frees_the_radios),
        ("the document's worked example", the_documents_worked_example),
        ("the onboard radio wins ties only",
         scoring_prefers_the_onboard_radio_only_in_ties),
        ("the service address follows the failover",
         the_service_address_follows_the_failover),
        ("a lease that changes under the active radio",
         a_lease_that_changes_under_the_active_radio),
        ("a router that will not hold the standby",
         a_router_that_will_not_hold_the_standby),
        ("a ten-second blip with a much worse spare",
         a_ten_second_blip_with_a_much_worse_spare),
        ("a failure that does not heal still moves",
         a_failure_that_does_not_heal_still_moves),
        ("nothing ever switches a radio off",
         nothing_ever_switches_a_radio_off),
        ("the service address is answered out of its own radio",
         the_service_address_is_answered_out_of_the_radio_it_sits_on),
        ("the NetworkManager translation", the_networkmanager_translation),
        ("a DHCP renewal strips the service address",
         the_service_address_survives_a_dhcp_renewal_stripping_it),
    ]
    for name, scenario in scenarios:
        before = len(FAILED)
        print("- " + name)
        scenario()
        if len(FAILED) == before:
            pass
    print()
    print("%d checks, %d failed" % (CHECKS, len(FAILED)))
    for message in FAILED:
        print("  " + message)
    return 1 if FAILED else 0


if __name__ == "__main__":
    sys.exit(main())
