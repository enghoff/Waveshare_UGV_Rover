# The rover daemon

One process on the Banana Pi that owns the driver board and the camera, and hands out
what can be done with them as tools. It exists because the rover's hardware does
not divide: there is one UART to the ESP32 and one `/dev/video0`, so two programs
that both want to command servos or look through the lens are two programs
corrupting each other.

```
  root@media                              admin@bpi-m4zero
  ----------                              ----------------
  voice-chat  <--speech--  talk.py  --TCP 8769-->  rover_daemon.py
  face-detect <---------------- JPEG ------------- | camera
                              boxes -------------> | UART -> ESP32
  voice-chat  <------- JPEG, POST /frame ---------- | (only with --vision)
```

That was not a hypothetical. `talk_pi.py` held the UART for the headlights while
`track_face_pi.py` held it for the gimbal, and running both meant interleaved
JSON on one wire and two processes fighting for one camera.

## What it offers

| tool | what it does |
|---|---|
| `set_lights(level)` | headlights, 0–255, both channels as one |
| `get_lights()` | the last level set — the board cannot be read back |
| `battery()` | how much charge is left: percent, volts, and a word for it |
| `look_at(pan, tilt)` | aim the camera in degrees; stops tracking first |
| `center_camera()` | straight ahead and level; stops tracking |
| `count_faces()` | one look: how many people, and roughly where each is |
| `look()` | take a picture and show it to the model — only with `--vision` |
| `start_tracking()` | follow a face, sweeping to find one |
| `stop_tracking()` | stop, and return the camera to centre |
| `track_next()` | let go of this person and take the next face |
| `tracking_status()` | running? following anyone? where is it pointing? |

`list_tools` returns the schemas themselves. That is the point of it: no client
carries a copy, so adding a tool is a change to [tool_schemas.py](tool_schemas.py)
and the handler on Rover, and nothing else is redeployed. [voice_chat/talk.py](../voice_chat/talk.py) asks on connect and hands
the answer straight to the model.

That list is built rather than constant, which is how `look` can come and go:
started without `--vision` there is nowhere to send a picture, so the tool is not
offered at all — a tool that can only fail is worse than a missing one, and the
model is not told about a camera it cannot look through. Adding or dropping the
flag and restarting the daemon is the whole of it; nothing is redeployed and no
client is restarted, because every client asks again on every connection.

### Looking

`look` takes one frame and POSTs it to the voice service's `/frame`, which holds
it for the turn that asked. **The picture goes straight to the model's host** —
it does not travel back through the client holding the conversation, which is on
a desk and has no use for it. What crosses that desk is the name the frame was
filed under, in an ordinary tool result. It is the road
[face_detect](../face_detect/README.md) frames already take, thirty times a
second, so there is nothing new about it but the port.

Nothing here decodes the picture; decoding one 640×480 JPEG costs 7 ms on this
machine and the picture is not for us. The one thing it does check is the two
bytes at the front: a frame read from a stream that was joined mid-picture ends
at an end-of-image marker without starting at a start-of-image one, and sending
that fragment would cost a round trip to be told it is not an image.

While tracking is running the loop owns the camera, so `look` sends the loop's
newest frame — which is the one the camera is actually pointing at — and refuses
if that frame is more than two seconds old rather than passing off something
stale as now.

```bash
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py --vision'              # 192.168.1.3:8767
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py --vision media.local:8767'
```

Where the camera is pointed also goes onto the map. `show_map` and the console's
`map_png` draw the gimbal's cone as a violet wedge — which way the lens is aimed and
how much of the room is inside the frame — because the map is otherwise entirely the
lidar's account of the room and says nothing about where the photographs are of. The
two sensors rarely agree on a direction: the gimbal pans a long way either side and
sweeps continuously while face tracking runs, and the rover's own arrow says nothing
about any of that.

**The gimbal counts pan positive to the right and the map counts bearings positive to
the left**, so `_camera_cone` hands over minus the pan. That is the whole conversion,
it happens in exactly one place, and `selftest.py` checks its direction rather than
its value — a sign error draws an ordinary-looking wedge over the wrong half of the
room. Started with `--no-camera` there is no cone at all, because a wedge drawn for a
lens that is not fitted is the map making a claim the hardware cannot keep.

`--camera-fov` sets how wide the wedge is, and the default is now the measured 132
degrees. It was 65 for a long time, on the reasoning that this is a generic USB
webcam — and that was wrong by more than a factor of two, because the module actually
fitted is a fisheye. A cone drawn at 65 degrees covered a third of what the camera
could see, so the map said the photographs were of one chair when they had the whole
end of the room in them. [`usb_cameras/calibrate_fov.py`](../usb_cameras/calibrate_fov.py)
is what measured it: sweep the gimbal across the room, track how far the room slides
for each degree of pan, and fit the lens. Run it again if the camera is ever changed.

**That flag is a starting position, not a setting.** The destination was a
constant once, and being a constant is exactly how it went wrong: the model moved
off MEDIA, the daemon kept posting pictures to MEDIA, and `look` failed with
`No route to host` while every other tool worked perfectly — which is a hard
thing to debug, because nothing about the rover is broken. So a client says where
it is listening, on every connection, with the control call below; the flag only
decides where pictures go until somebody says otherwise.

### The battery

`battery` is the one tool here that reads the board rather than telling it
something. The ESP32 streams one JSON object per line whether or not anybody asked
— `{"T":1001, ...}`, about seventeen times a second — and `v` in it is the pack
voltage in hundredths of a volt. Everything else in that line the daemon throws
away: there is a 9-DoF IMU, a magnetometer and wheel encoders in there, and the
lidar's scan matcher is a better odometer than any of them. The voltage has no
second source at all, which is the whole reason this reads the port.

Three 18650 cells in series, so 12.6 V is full and about 9.9 V is empty, and the
percentage comes off a discharge curve rather than a straight line between the two
— lithium-ion is nearly flat through the middle of its range, where 40% to 70% is
a tenth of a volt per cell, and interpolating linearly reads twenty points high
for most of a run. It is still an estimate under load and it says so: what it is
good for is watching the number fall over an afternoon, not comparing two runs.

**The two ends of that curve are Waveshare's numbers, not ours.** The pack is the
[UPS Module 3S](https://www.waveshare.com/wiki/UPS_Module_3S), which ships with a
12.6 V 2 A charger and carries three HY2213-BB3A balancing chips — those start
bleeding a cell at 4.200 ± 0.025 V, so full is 4.2 V per cell by construction rather
than by convention. The empty end is theirs too: the `INA219.py` demo published for
that module computes `(volts - 9) / 3.6 * 100`, putting 0% at 3.0 V per cell. This
table keeps both of those ends and disagrees only about the middle, where that
straight line is exactly the twenty-point error described above. Underneath sits an
S-8254AA protection IC, but the schematic names only the family and its variants cut
off anywhere between 2.0 and 3.0 V per cell, so 0% here is deliberately set at the
safe top of that range.

**What this cannot see is charging.** The board sends voltage and nothing else —
there is no current anywhere in `T:1001` — and the INA219 on the UPS module, which
does measure charge current, is not read here: the host can see that chip on
header I²C, but the ESP32 already owns the bus — see [docs/i2c.md](../docs/i2c.md).
So a pack sitting on the charger and not taking any looks exactly like a pack at rest, and
the only thing that tells them apart is the module's own LED — red while charging,
green when full.

Under 6 V there is no pack at all. The ESP32 runs perfectly well from USB with the
battery out or the main switch off, and reports a few tenths of a volt when it
does, so that gets its own answer — `"state": "absent"`, with no percentage
attached — because a flat battery and a missing one are different things to go and
do something about.

**Reading the port does not take the lock that writes to it.** The two directions
of a serial line do not interfere; the *waiting* would. A whole line can take four
tenths of a second to turn up, and the write lock is what the navigator holds to
keep PWM going to the wheels, so a rover that stopped steering because somebody
asked about the battery would be a worse rover than one whose reading is a few
seconds old. For the same reason a reading is cached for five seconds and its age
is reported with it: a console polls this, and a board that has gone quiet should
show up as a number getting old rather than as a number.

### `set_vision`, and why it is not a tool

```
-> {"call": "set_vision", "arguments": {"address": "192.168.1.206:8767"}}
<- {"ok": true, "vision": "http://192.168.1.206:8767/frame", "tools": [...]}
-> {"call": "set_vision", "arguments": {"address": null}}
<- {"ok": true, "vision": null, "tools": [ ...without `look`... ]}
```

It is dispatched like a tool because that is the only protocol this daemon
speaks, and it is deliberately absent from `list_tools`, so no model is ever
shown it or can call it. The client is the one that knows where its own frame
server is; the model has no business knowing there is one.

Naming no address switches the picture path off, which withdraws `look` — a tool
that cannot reach the model's host is worse than a missing one, for the reason
this file repeats: the model says it has done the thing, and nothing happens.

The address a client should send is the one *its own socket to this daemon* is
bound to, not whatever `hostname -I` says. A desk has several addresses and only
one of them is on the way here, and which one that is changes when the rover
leaves its dock and starts answering on wlan0. See `local_address` in
[voice_chat/rover_tools.py](../voice_chat/rover_tools.py).

### The calls no model is shown

Five things are dispatched exactly like tools and are deliberately absent from
`list_tools`, so no model is ever offered one: `set_vision` above, and four written
for [drive_web/drive_web.py](../drive_web/drive_web.py), the console somebody
drives this rover from by hand -- hosted on this machine at
[drive_web/](../drive_web/README.md), TCP 8771.

| call | what it does |
|---|---|
| `nav_status` | every number the driving loop has — PWM, measured turn rate, scan age |
| `map_png` | the map as base64 PNG **in the reply**, at a given extent and picture size |
| `camera_jpeg` | one frame as base64 JPEG in the reply |
| `clear_map` | throw the SLAM map away and stand the rover at the origin of an empty one |

The first two are covered in [voice_chat/README.md](../voice_chat/README.md). The
other two are worth a note each.

`camera_jpeg` is to `look` what `map_png` is to `show_map`: a tool result cannot
carry an image into a conversation, so `look` posts the frame to the model's host and
returns the name it was filed under, while a window on a desk has no such problem and
routing a picture through a frame server to reach the screen of the machine that
asked for it would be silly. The practical difference is what each one needs.
`look` needs somewhere to post and is withdrawn without `--vision`; `camera_jpeg`
needs only a camera, so a daemon started with no vision host can still be asked for a
picture. The bytes are the camera's own and nothing here decodes them — there is no
image library on this Pi, which is the same reason face detection happens on another
host — so what arrives is MJPEG and turning it into something a widget can show is
the caller's problem.

`clear_map` is kept from models for a different reason than danger. The rover fills a
map back in within a revolution or two; the trouble is that a model handed this will
reach for it. Told there is no route to somewhere, the obliging thing to do is clear
the map and try again — which throws away the only account anyone has of the room,
including the walls the route was refused for. Whether a map has drifted past being
worth keeping is a judgement made by looking at it. The navigator refuses while a
move is running, because the route being followed is written in the frame the clear
discards; see [lidar_slam/README.md](../lidar_slam/README.md).

## What it cannot do, and will not pretend to

**There is no face recognition here.** YuNet is a detector: it returns a box and
a confidence, and [face_detect/server.py](../face_detect/server.py) discards even
the five landmarks it produces. There is no embedding model, no identity, no
attribute classifier anywhere in this repo. So:

- **Nobody can be told apart.** `aiming.py` keeps its lock by *proximity* — the
  nearest detection to where the face was, within about 1.5 face-widths — and
  acquires by taking the largest strong one. Somebody who leaves the frame and
  comes back is a stranger to it.
- **`track_next` therefore cannot promise a different person.** It suppresses
  detections near whoever is being followed for `SKIP_FOR_S` and lets `Target`
  acquire the next largest face. If nobody else is in view it will find the same
  person again once the suppression lapses, and its result says so in as many
  words, because the model would otherwise claim it had found somebody new.
- **"Find someone with glasses" has no model behind it.** Nor age, expression or
  anything else about a face. Adding one is a project, not an exposure.

**Face tracking needs a detector, and will say so when it has not got one.** It
runs in this process now — YuNet on the board's own four cores, see
[face_tracking/yunet.py](../face_tracking/yunet.py) — where it used to be a
service on the GPU box on port 8768, and briefly the OAK camera's VPU on loopback.
`--service host:port` still puts it back on a service. The tracking loop is
written to hold still through a detector being away rather than to die, which is
right for a loop already running and wrong for one being started: it would start,
hold still, report itself as tracking, and the model would say "I started tracking
people" while the camera never moved. That is this file's own worst-case failure
arriving from underneath the prompt written to prevent it. So `start_tracking`
checks the detector answers before it claims anything — for the in-process one,
that the library and the model load, which is where a rover deployed without
OpenCV finds out:

```
{"ok": false, "error": "the face detector at 192.168.1.3:8768 is not answering
 (TimeoutError), so tracking a face is not possible right now"}
```

A refusal is instant and a host that is off costs `DETECT_PROBE_S`, which is why
the check is bounded rather than left to the first frame.

**Driving is deliberately not exposed.** The firmware stops the base if it hears
nothing for `HEARTBEAT_MS`, so "drive forward" is a control loop with a stop
condition rather than a call — and speech is ~1.5s each way, which makes a spoken
"stop" a poor way to stop a moving rover. The gamepad's View button and the
firmware heartbeat are the real stop, and a voice tool sitting next to them would
be a worse one that somebody might trust. The gimbal is fine by contrast: `T:133`
does not feed the heartbeat, so aiming is not mistaken for driving.

## Running it

```bash
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py'
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py --no-camera'     # lights and gimbal only
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py --host 192.168.1.22'   # board over wifi
ssh bpi-m4zero 'cd ugv && python3 rover_daemon.py --lidar --camera-fov 58'
```

It centres the gimbal at startup, like every other script that commands it: the
angles are a model kept true by putting the camera where the code thinks it is,
since the firmware's `getGimbalFeedback()` is not reachable over JSON.

Do **not** run it alongside `drive_gamepad_pi.py` or `track_face_pi.py`. Those
are still standalone and still take the UART directly; the whole point of this
is that only one thing does.

Binding is `0.0.0.0:8769` with no authentication, the same trade
[face_detect](../face_detect/README.md) makes and with the same warning. The
Banana Pi has no firewall in front of it.

## The protocol

Newline-delimited JSON over TCP. One request, one reply, no streaming:

```
-> {"call": "set_lights", "arguments": {"level": 255}}
<- {"ok": true, "level": 255, "on": true}
-> {"call": "list_tools"}
<- {"ok": true, "tools": [ ...schemas... ]}
```

A failure is a reply, not a dropped connection: `{"ok": false, "error": ...}`.
That shape matters because the result goes into a language model's context
verbatim and gets paraphrased out loud, so it has to read as an explanation
rather than a traceback. It is also why arguments are coerced rather than
validated strictly — `"255"`, `255.0`, `"on"` and `"50%"` all mean something
obvious, and a 4B model at int4 produces all of them.

## How it shares

- **The board** is behind a lock, because the tracking loop commands servos on
  its own thread while tool calls arrive on connection threads. Two interleaved
  writes are one line of JSON the ESP32 cannot parse.
- **The camera** is opened two different ways, and the difference is not the
  picture but what is still running afterwards. A single picture — `look`,
  `camera_jpeg`, `count_faces` — starts `v4l2-ctl` for three frames and lets it
  exit, which is 0.6 s and leaves nothing behind. Only face tracking opens the
  30 fps feed, because only face tracking wants every frame. While tracking is
  running the loop owns the camera, so `count_faces` answers from what the loop
  last saw rather than trying to take a second look — an honest answer beats a
  contended one. `CAMERA_IDLE_S` still exists but nothing reaches it in ordinary
  running; it is the backstop for a feed left behind by a crash.
- **Driving outranks looking**, and that is a rule about this one core rather
  than about the camera. Holding the feed open costs the scan matcher about a
  quarter of the lidar's revolutions — measured, stationary, one picture taken:
  9.94 revolutions/s and no losses with the camera shut, 7.52/s and 22.1%
  dropped with it streaming. Since the matcher is the only odometer this rover
  has, a photograph taken on the move used to corrupt the measurement `drive`
  closes its loop on, for the whole twenty seconds the camera stayed warm. Two
  things keep that from happening: a one-shot picture never opens the feed, and
  the navigator calls `park_tracking` before the wheels turn, which puts the
  tracking loop down **and** releases the camera. The `stand_aside` niceness in
  `track_face_pi` is the weaker half of the same rule — within one interpreter
  the GIL decides who runs, not the scheduler, so not asking for frames nobody
  wants matters more than asking politely.
- **The gimbal** cannot be aimed by two things at once, so `look_at` and
  `center_camera` stop tracking first and say so in their result. That is what a
  person means by "look left" while the rover is following somebody.

## Checks

```bash
ssh bpi-m4zero 'cd ugv && python3 selftest.py'    # no board and no camera
python rover_daemon/selftest.py            # the same, from the repo
```

It covers argument coercion, the servo limits, and — the one worth having — that
every schema the model is shown has a handler behind it and every handler is
offered. A name that does not match fails as "no such tool" in the middle of a
conversation, which is a poor place to find out.

## Deploying

```bash
scp rover_daemon/*.py bpi-m4zero:~/ugv/
```

Flat into `~/ugv/`, which is the layout already there and is what lets this
import `aiming.py` and `track_face_pi.py` from the same directory. Those imports
are deliberately made lazily, inside the functions that need them, so the module
loads — and its selftest runs — on a machine with no camera and no `aiming.py`
beside it.
