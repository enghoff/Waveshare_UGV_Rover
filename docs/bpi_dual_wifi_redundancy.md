# Banana Pi M4 Zero Dual-Wi-Fi Redundancy for a Rover

> **This was built on 2026-08-25 and is running on the rover.** What follows is
> the design as it was proposed; the implementation is
> [wifi_roam/wifi_dual.py](../wifi_roam/wifi_dual.py), and
> [wifi_roam/README.md](../wifi_roam/README.md) is where the account of what it
> actually does now lives. Five things came out differently, and they are worth
> knowing before reading the rest of this as a specification:
>
> - **There is no NetworkManager on this board**, so nothing below about `nmcli`
>   connections or BSSID-pinned profiles applies. It is netplan,
>   `systemd-networkd` and one `wpa_supplicant` per interface, and a radio is
>   held on a network by enabling that one in its supplicant and disabling the
>   rest.
> - **BSSID pinning turned out not to be needed.** This document assumes one
>   SSID shared by three access points; the house actually has six SSIDs, being
>   three routers with a 2.4 and a 5 GHz name each, so keeping the two radios
>   apart is a matter of choosing different names. The real hazard is the
>   opposite one, which the document does not mention: `TheGreatLord` and
>   `TheGreatLord 5G` are *one box*, and a radio on each would look like
>   redundancy and provide none.
> - **The two radios are not interchangeable**, so "best AP to the active radio,
>   second best to the standby" is not quite the policy. The onboard BCM4345/6
>   is dual band at 31 dBm; the USB dongle is 2.4 GHz, 1T1R, 0 dBm, and is the
>   adapter that failed its own initialisation and fell off the bus in August.
>   The onboard radio therefore gets first pick and wins ties by 3 dB, and the
>   unreliable one is the spare, where its failing costs nothing.
> - **Option 2 was implemented and Option 1 is what it falls back to.** The
>   rover answers on 192.168.1.80, moved between the interfaces with a
>   gratuitous ARP. It is ARP-probed before every claim rather than once, and if
>   anything answers for it the manager runs without a service address — which
>   is Option 1 exactly, and costs only the connections that were already open.
> - **The ROS 2 section does not apply to this rover.** Everything DDS does here
>   is on loopback: the navigation stack, the driver-board bridge and the
>   navigation bridge all live on the board and talk over 127.0.0.1. No ROS 2
>   traffic crosses the wifi at all, so discovery during failover is not a
>   question that arises.
>
> One thing the document does not claim, which turned out to be the largest
> practical gain: with two radios the **standby does all the scanning**, so a
> scan never interrupts the link carrying traffic. That removes the single worst
> hazard in the single-radio keeper this replaces.

## Goal

Use the Banana Pi M4 Zero's onboard Wi-Fi together with a USB Wi-Fi dongle to provide redundant connectivity for a mobile rover.

The site has three Wi-Fi access points:

- AP1
- AP2
- AP3

All three APs are connected to the **same Layer-2 LAN / IP subnet**, but their coverage areas do not completely overlap.

The objective is to keep the rover connected as it moves by:

1. Keeping two Wi-Fi radios active simultaneously.
2. Associating each radio with a different AP.
3. Selecting the best usable connection for rover traffic.
4. Keeping the second radio ready as a hot standby.
5. Switching rapidly if the active link degrades or disappears.
6. Ideally keeping the rover reachable through one stable IP address.

---

## Proposed topology

```text
                         SAME LAN
              ┌────────────┼────────────┐
              │            │            │
             AP1          AP2          AP3
              ▲            ▲
              │            │
           wlan0         wlan1
          onboard       USB dongle
              \            /
               \          /
              Banana Pi M4 Zero
                     │
                  Rover
```

Both Wi-Fi interfaces are active at the same time.

For example:

```text
wlan0 -> AP1
wlan1 -> AP2
```

The two radios should normally be associated with the two best available APs rather than both connecting to the same AP.

---

## Why two radios are preferable to ordinary Wi-Fi roaming

With one Wi-Fi interface, roaming typically works like this:

```text
connected to AP1
      ↓
AP1 becomes weak
      ↓
disconnect / roam
      ↓
associate with AP2
      ↓
traffic resumes
```

There may be a period during which the rover has no usable link.

With two radios:

```text
wlan0 -> AP1 -> currently carrying traffic
wlan1 -> AP2 -> already connected and tested
```

If AP1 fails:

```text
wlan1 -> AP2 -> immediately becomes active
wlan0 -> searches for the next-best AP
```

The replacement connection can therefore be established **before** the current connection is abandoned.

This is particularly useful for a moving robot where loss of SSH, ROS 2 traffic, telemetry, or teleoperation should be minimized.

---

## AP selection

If all APs use the same SSID, the BPI should still identify and select them individually by **BSSID**.

Example:

```text
SSID: rover-net

AP1 BSSID: AA:AA:AA:AA:AA:01
AP2 BSSID: AA:AA:AA:AA:AA:02
AP3 BSSID: AA:AA:AA:AA:AA:03
```

NetworkManager can create connections pinned to individual BSSIDs.

This makes it possible to deliberately maintain:

```text
wlan0 -> AP1
wlan1 -> AP2
```

rather than allowing both adapters to independently choose AP1.

---

# Link scoring

Do not choose the active AP using RSSI alone.

A strong RF signal does not guarantee a good network path.

For example:

```text
AP1  RSSI -56 dBm   latency 5 ms    packet loss 0%
AP2  RSSI -49 dBm   latency 80 ms   packet loss 15%
AP3  RSSI -65 dBm   latency 8 ms    packet loss 0%
```

Although AP2 has the strongest signal, AP1 may provide the best connection.

A useful link score can include:

```text
RSSI
+ gateway reachability
+ latency
+ packet-loss penalty
+ association state
+ optional application-level health check
```

Conceptually:

```text
score =
    RSSI score
    - latency penalty
    - packet-loss penalty
    - connectivity penalty
```

---

## Suggested thresholds

Exact values should be tuned experimentally, but reasonable starting behavior is:

- Avoid switching for differences of only 1-3 dB.
- Prefer a new AP when it is approximately 8-10 dB better.
- Immediately fail over if the current AP becomes unreachable.
- Fail over rapidly on sustained packet loss.
- Require the candidate link to remain healthy for roughly 1-2 seconds before a non-emergency switch.
- Add a hold-down period after switching to prevent oscillation.

This prevents:

```text
AP1 -> AP2 -> AP1 -> AP2 -> AP1
```

when the rover is near the overlap between two cells.

---

# Recommended operating model

At any time, maintain:

```text
best AP       -> active radio
second-best AP -> standby radio
third AP       -> candidate
```

Example:

```text
AP1  -47 dBm
AP2  -59 dBm
AP3  -81 dBm

wlan0 -> AP1   ACTIVE
wlan1 -> AP2   STANDBY
```

As the rover moves:

```text
AP1  -72 dBm
AP2  -51 dBm
AP3  -58 dBm

wlan1 -> AP2   ACTIVE
wlan0 -> AP3   STANDBY
```

The former active interface can then be repositioned onto the next-best AP.

---

# Option 1: Separate IP addresses and route metrics

The simplest implementation is to allow each interface to have its own LAN address.

For example:

```text
wlan0 = 192.168.1.80
wlan1 = 192.168.1.81
```

Linux routing metrics determine which interface carries outbound traffic.

Example:

```text
wlan0 default route metric 50
wlan1 default route metric 150
```

The lower metric wins.

On failover:

```text
wlan0 metric 200
wlan1 metric 50
```

### Advantages

- Simple.
- Works well with NetworkManager.
- Easy to diagnose.
- No unusual Layer-2 behavior.

### Disadvantage

Existing TCP sessions may break when traffic moves to the other interface because the source IP changes.

This may affect:

- SSH
- TCP telemetry
- browser sessions
- some ROS 2 traffic
- long-running service connections

For some rover workloads this is acceptable; for teleoperation it may not be ideal.

---

# Option 2: Stable rover service IP

Because all three APs are on the **same LAN**, a better rover-oriented design is possible.

Expose one stable rover address, for example:

```text
192.168.1.80
```

and associate that service address with whichever Wi-Fi interface is currently active.

Conceptually:

```text
                 192.168.1.80
                       │
          ┌────────────┴────────────┐
          │                         │
     wlan0 ACTIVE              wlan1 STANDBY
         │                         │
        AP1                       AP2
```

After failover:

```text
                 192.168.1.80
                       │
          ┌────────────┴────────────┐
          │                         │
     wlan0 STANDBY             wlan1 ACTIVE
         │                         │
        AP3                       AP2
```

After moving the address to another interface, send a **gratuitous ARP** so other LAN devices update their ARP caches promptly.

Conceptually:

```bash
arping -U -I wlan1 192.168.1.80
```

The exact command and address setup should be validated on the final Linux image.

### Why this is useful

The remote side still sees:

```text
rover = 192.168.1.80
```

before and after failover.

TCP sessions therefore have a much better chance of surviving a short Wi-Fi interruption than when switching between two different source IP addresses.

### Important caveat

Moving one IP between two Wi-Fi interfaces is more complex than simple route-metric failover.

Care must be taken with:

- Linux routing rules
- ARP behavior
- reverse-path filtering
- DHCP
- NetworkManager attempting to manage the same address
- interface-specific routes
- source address selection

For a production rover, this should be tested under repeated failover while SSH, ROS 2, telemetry, and video streams are active.

---

# Suggested architecture

A small rover-specific service can control the policy.

```text
                     wifi-manager
                          │
       ┌──────────────────┼──────────────────┐
       │                  │                  │
    scan APs          test links         select path
       │                  │                  │
       └──────────────────┼──────────────────┘
                          │
                 NetworkManager / ip
                          │
                  ┌───────┴───────┐
                  │               │
                wlan0           wlan1
```

The service can run under `systemd`.

---

## Main loop

A simple state machine would perform:

```text
1. Scan known AP BSSIDs.
2. Read RSSI for each visible AP.
3. Verify association state.
4. Ping the LAN gateway through each connected interface.
5. Measure packet loss and latency.
6. Score each usable path.
7. Keep the two radios on different APs.
8. Select the best path as ACTIVE.
9. Keep the second-best path connected as STANDBY.
10. Fail over if ACTIVE becomes unhealthy.
11. Move/re-advertise the stable rover IP if using that design.
12. Reassociate the free radio with the next-best AP.
```

A monitoring interval around 0.5-2 seconds is probably sufficient for rover use.

---

# State example

```text
KNOWN APS
----------------------------------------
AP1    BSSID aa:aa:aa:aa:aa:01
AP2    BSSID aa:aa:aa:aa:aa:02
AP3    BSSID aa:aa:aa:aa:aa:03


CURRENT LINKS
----------------------------------------
wlan0  AP1   RSSI -54   loss 0%   ACTIVE
wlan1  AP2   RSSI -62   loss 0%   STANDBY


VISIBLE
----------------------------------------
AP1 -54
AP2 -62
AP3 -76
```

Later:

```text
wlan0 / AP1
RSSI -78
packet loss 30%

wlan1 / AP2
RSSI -55
packet loss 0%
```

The controller performs:

```text
promote wlan1
move rover service IP if required
send gratuitous ARP
demote wlan0
associate wlan0 with AP3 if AP3 is now the second-best candidate
```

---

# Failure behavior

## Active AP disappears

```text
AP1 OFFLINE
```

Expected:

```text
wlan1/AP2 promoted immediately
wlan0 searches AP3
```

---

## Active interface/dongle fails

If `wlan1` itself disappears:

```text
USB Wi-Fi failure
```

the onboard Wi-Fi continues carrying traffic.

This provides redundancy against both:

- AP failure
- Wi-Fi adapter failure

---

## One AP has RF signal but no LAN connectivity

RSSI-only selection might continue using it.

The health checker instead sees:

```text
RSSI       good
gateway    unreachable
```

and marks the link unusable.

---

# AP configuration

Where possible, configure AP1/AP2/AP3 on different non-overlapping channels.

This improves resilience against localized interference.

For 2.4 GHz, this normally means using non-overlapping channel planning such as:

```text
AP1 -> channel 1
AP2 -> channel 6
AP3 -> channel 11
```

For 5 GHz, choose appropriately separated channels according to the local regulatory domain and AP capabilities.

If possible, use 5 GHz for high-bandwidth rover traffic, while retaining 2.4 GHz where its longer range is useful.

---

# USB Wi-Fi dongle considerations

The second adapter should have good Linux driver support.

Important properties are:

- reliable Linux kernel support
- ability to scan while associated, if possible
- stable operation under continuous load
- 5 GHz support if needed
- external antenna support if useful
- modest USB power consumption

A dongle using a chipset with an in-kernel Linux driver is preferable to one requiring a vendor DKMS driver.

---

# Video traffic

If the BPI is also carrying video from devices such as an OAK-D or gimbal camera, failover behavior should be tested under realistic traffic loads.

UDP video generally handles a short interruption differently from TCP:

- packets during the switch are simply lost
- there is no TCP reconnect delay
- the decoder may recover rapidly if keyframes are frequent

For low-latency teleoperation, application-level recovery characteristics therefore matter as much as IP failover.

---

# ROS 2 considerations

ROS 2 DDS discovery and transport can behave differently depending on the DDS implementation and network interface configuration.

With two simultaneously active Wi-Fi interfaces, it may be useful to explicitly control which interfaces DDS uses rather than allowing discovery over both.

The stable-IP design can simplify this.

Testing should include:

- discovery during failover
- existing publishers/subscribers
- service calls
- actions
- large sensor topics
- camera/video topics

---

# Suggested implementation stages

## Stage 1 — basic dual connectivity

Configure:

```text
wlan0 -> AP1
wlan1 -> AP2
```

Verify both can independently reach:

```text
LAN gateway
control workstation
rover services
```

---

## Stage 2 — route failover

Use separate interface IPs and routing metrics.

Test:

```text
disconnect AP1
```

and verify traffic switches automatically to AP2.

This establishes the hardware and driver reliability first.

---

## Stage 3 — AP scoring

Add the monitoring daemon:

```text
RSSI
latency
packet loss
association
gateway reachability
```

and automatically select the active interface.

---

## Stage 4 — standby AP management

Make the inactive radio track the second-best AP.

This gives:

```text
best AP       -> ACTIVE
second best   -> STANDBY
```

at all times.

---

## Stage 5 — stable rover IP

If preserving active sessions is important, implement the stable service IP and gratuitous ARP handover.

Stress-test it with:

```text
continuous ping
SSH
ROS 2
video
web UI
teleoperation
```

while repeatedly disabling APs and driving between coverage areas.

---

# Recommended final design

For this rover, the preferred architecture is:

```text
                       SAME LAN
             ┌───────────┼───────────┐
             │           │           │
            AP1         AP2         AP3
             ▲           ▲
             │           │
          onboard        USB
           wlan0        wlan1
             \           /
              \         /
             Banana Pi M4 Zero
                    │
             stable rover IP
                    │
          ┌─────────┼─────────┐
          │         │         │
         SSH       ROS2      Web/video
```

Policy:

```text
ACTIVE  = best healthy AP
STANDBY = second-best healthy AP

switch only when:
    candidate is materially better
OR
    active link is degraded
OR
    active link fails
```

The most important design principle is that the **standby connection is already associated and tested before it is needed**.

That should produce substantially faster and more predictable recovery than relying on conventional single-radio roaming alone.
