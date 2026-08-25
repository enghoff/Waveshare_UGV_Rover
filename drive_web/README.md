# Drive console, on the rover

A page with the driving tools, hosted here so a phone or another machine opens
it without a desk process. The daemon is unchanged: this is still a client of
TCP 8769, just on loopback.

```
  phone / desk browser
          |
          |  HTTP :8771   (page, event stream, /map.png, /frame.jpg)
          v
  drive_web.py          --idle, from boot
          |
          |  six TCP connections to 127.0.0.1:8769
          v
  rover_daemon.py
```

A Pi 1 could not afford this and was never asked to. The Banana Pi M4 Zero can:
the HTTP and the JSON are cheap, and the expensive work (drawing a map, taking a
frame) is the daemon's. `--idle` is why a process that lives from boot is not
that client overnight -- it opens the six connections when a browser arrives and
drops them when the last tab has been gone a couple of seconds.

**8771, not 8770.** 8770 is [oak_depth](../oak_depth/README.md). There is no
password on the driving controls; anyone on the LAN who can load the page can
drive the rover. The microphone is the one exception, and
[the microphone](#the-microphone) says why.

The pacing and the English live in
[voice_chat/console_model.py](../voice_chat/console_model.py); the wire is
[voice_chat/rover_tools.py](../voice_chat/rover_tools.py). Both are copied next
to this on deploy, because the tests still import them from `voice_chat/`.

## Running it

`crontab` for `admin`, beside the daemon's own entry, for the reason
[run_daemon.sh](../rover_daemon/run_daemon.sh) gives -- a system unit would need
a sudo password no script has, and cron needs none:

```
@reboot /home/admin/ugv/run_daemon.sh --vision --board-bridge --ros-nav
@reboot /home/admin/ugv/oak_depth/run_oak_depth.sh
@reboot /home/admin/ugv/drive_web/run_drive_web.sh
```

```bash
scp drive_web/*.py drive_web/*.html drive_web/*.sh drive_web/README.md bpi-m4zero:~/ugv/drive_web/
# console_model and rover_tools are the console's; session.py is the omni
# protocol the microphone runs. server.py is not imported there, only parsed,
# because it is where the prompt is written.
scp voice_chat/{console_model,rover_tools,session,talk_frames,prompts,server}.py \
    bpi-m4zero:~/ugv/drive_web/
ssh bpi-m4zero '~/ugv/drive_web/install.sh'              # crontab, once
ssh bpi-m4zero 'sh ~/ugv/drive_web/install_websockets.sh'  # a wheel, once
ssh bpi-m4zero '~/ugv/drive_web/restart.sh'              # prints /health
python drive_web/selftest.py
```

**Use `restart.sh` rather than typing the `pkill` yourself.** The pattern that
matches the server also matches the ssh command carrying it.

```
https://bpi-m4zero.local:8771/
https://192.168.1.139:8771/
```

Windows often cannot resolve `.local`; the number works. Plain `http://` on the
same port is answered with a redirect into `https://`, so an old bookmark still
lands in the right place.

## Why it speaks TLS now

**One browser rule, and nothing about secrecy.** `getUserMedia` -- the call that
opens a microphone -- is refused outside a secure context, and a secure context
is HTTPS or `localhost` and nothing else. A phone on the sofa is neither. So
[make_cert.sh](make_cert.sh) makes a certificate and the console serves it, and
without one the page still loads and the microphone is simply not offered.

```bash
ssh bpi-m4zero '~/ugv/drive_web/make_cert.sh'          # idempotent
ssh bpi-m4zero '~/ugv/drive_web/make_cert.sh --force'  # start again
```

It writes to `~/.ugv/tls/`, outside `~/ugv`, because a deploy lands on `~/ugv`
and a private key a deploy can overwrite -- or carry back into the repository --
is a key in the wrong place. `run_drive_web.sh` runs it at every boot: this board
is wifi-only and moves between three house networks, and a certificate is checked
against the address that was *typed*, so the address it names has to be the one
the board woke up on. Since 2026-08-25 there is a steadier answer to type:
`https://192.168.1.80:8771/` is the rover's service address, held by whichever
of its two radios is carrying traffic, so it stays right across a failover and
across a reboot. The certificate names it alongside whatever the board woke up
on.

## The network panel, and what a join costs now

The panel shows two radios: the onboard one and the USB dongle, each with the
access point it is on, its signal, its round trip and whether it is `active` —
carrying the rover's traffic — or `standby`, which means associated, tested and
idle. A standby in that state is why a failover is an address moving rather than
a scan and a DHCP round, and the line under the rows says which address that is
and how many times it has moved.

Two things about the panel changed with it, and both are the panel telling the
truth about something that used to be true and is not any more.

**The list of networks is always fresh and nobody has to ask for it.** It used
to be whatever was last heard, because a scan takes the radio off channel for
several seconds on a bus it shares with the camera and the lidar. The standby
does the scanning now, so the list costs the link nothing and arrives on its
own.

**Pressing `join` no longer takes the page down.** It used to mean: drop every
connection to the rover including this one, and reconnect in a few seconds. Now
the rover puts its *spare* radio on the network you chose, waits until that
radio is associated, addressed and answering the gateway, and only then moves
the traffic across. The browser does not reconnect, and the panel says so rather
than warning about an outage that is not going to happen. On a rover with one
radio — the Pi, or this board with the manager stopped — it still means the old
thing, and the panel still says the old thing.

See [wifi_roam/README.md](../wifi_roam/README.md) for what is deciding all this.

**Both schemes arrive on one port**, which is worth a sentence because it is not
the usual arrangement. A TLS connection opens with a handshake record -- the byte
0x16 -- and no HTTP request line starts with that, so the first byte is peeked at
and the connection either becomes TLS or is sent a 308 and closed. The
alternative was a second port, and a browser's trust decision is per origin, so
that would have meant clicking through a second certificate warning on a page
whose whole purpose is that the first one bought a microphone.

There is a tiny certificate authority in `~/.ugv/tls/console-ca.crt`, and it is
there for devices rather than for the rover: a self-signed leaf can be clicked
past in any browser, but Android's "install a certificate" dialogue takes CA
certificates and refuses everything else. Install that one file and the console
gets an ordinary padlock; skip it and all that is lost is the clicking.

## The microphone

The rover holds a live conversation with Alibaba's realtime omni model, and this
page is its microphone and its speaker.
[omni_bridge.py](omni_bridge.py) runs the session and
[voice_chat/session.py](../voice_chat/session.py)'s `Session` is what actually speaks
the protocol -- unchanged, with a browser supplied where a sound card used to be,
because everything measured about that file lives in it and a fork would drift.

What this buys over running the same client on a desk is where the work happens.
The daemon is on loopback, so the tools are local calls; the frame server is on
loopback too, so `look`'s JPEG goes from the camera to the model without crossing
the house wifi at all. What crosses the wifi is the audio: 32 kB a second each
way, against 35 kB for a single picture.

It also buys three more tools, which are the one consequence of this move that
is not merely about latency. Running code is refused on anything but loopback,
because submitting code is a different proposition from flashing the headlights
on a port that authenticates nothing, and a desk holding the conversation was
outside that gate. A session on the rover is inside it, so the model can write a
small program when what was asked for is a sequence rather than an act:
`run_script` for one that finishes while it waits, in a child process with a
fifteen-second wall clock; `start_script` for one with no end written into it,
which runs until it is stopped and hands back a handle instead of an answer; and
`script_stop`, which is what stops it. See
[docs/scripting.md](../docs/scripting.md).

* **On demand, and closed again.** There is no session until the button is
  pressed. It ends when the button is pressed again, when the tab goes, after two
  quiet minutes with nothing attached, or at the service's own two-hour ceiling.
  The account behind the key is free-quota-only, so an idle session is not free.
* **One microphone at a time.** Two browsers pushing audio into one context is
  two people talking into the same sentence. The newest wins and the older is
  told it lost it.
* **A token, on this control only.** Driving stays as open as it has always been;
  what changed is that the page can now spend an account's quota, which is a
  different bargain. The token is made on first run and lives in
  `~/.ugv/console.token`; the page keeps it in `localStorage` once it is entered.
* **The key is on the rover**, at `~/.ugv/alibaba.key`, mode 600 -- a deliberate
  exception to this repository's rule that credentials never leave the
  workstation, made because the alternative is the desk having to be switched on
  for the rover to talk. Outside `~/ugv` so no deploy can carry it back.

Interruption is the part with a network hop in the middle of it. When somebody
talks over the rover, the model believes it said the whole reply, and the only
correction available is how many milliseconds were audible -- which is a fact the
browser holds and this end has to be told. The page reports its playback cursor
five times a second, and what it reports is clamped to what was actually sent;
send *that* number instead and every interruption teaches the model it said more
than anybody heard.

`GET /health` is what `restart.sh` waits on. `watching` is how many event
streams are open; `rover` is empty until one of them has made this a client.

Against an invented room, with no rover:

```
python voice_chat/mock_rover.py --drive
python drive_web/drive_web.py --no-idle --bind 127.0.0.1
```
