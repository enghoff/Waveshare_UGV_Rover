# The rover daemon

One process on the Pi that owns the driver board and the camera, and hands out
what can be done with them as tools. It exists because the rover's hardware does
not divide: there is one UART to the ESP32 and one `/dev/video0`, so two programs
that both want to command servos or look through the lens are two programs
corrupting each other.

```
  root@media                              admin@rpi
  ----------                              ---------
  voice-chat  <--speech--  talk.py  --TCP 8769-->  rover_daemon.py
  face-detect <---------------- JPEG ------------- | camera
                              boxes -------------> | UART -> ESP32
```

That was not a hypothetical. `talk_pi.py` held the UART for the headlights while
`track_face_pi.py` held it for the gimbal, and running both meant interleaved
JSON on one wire and two processes fighting for one camera.

## What it offers

| tool | what it does |
|---|---|
| `set_lights(level)` | headlights, 0–255, both channels as one |
| `get_lights()` | the last level set — the board cannot be read back |
| `look_at(pan, tilt)` | aim the camera in degrees; stops tracking first |
| `center_camera()` | straight ahead and level; stops tracking |
| `count_faces()` | one look: how many people, and roughly where each is |
| `start_tracking()` | follow a face, sweeping to find one |
| `stop_tracking()` | stop, and return the camera to centre |
| `track_next()` | let go of this person and take the next face |
| `tracking_status()` | running? following anyone? where is it pointing? |

`list_tools` returns the schemas themselves. That is the point of it: no client
carries a copy, so adding a tool is a change to this file and nothing else is
redeployed. [voice_chat/talk.py](../voice_chat/talk.py) asks on connect and hands
the answer straight to the model.

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

**Driving is deliberately not exposed.** The firmware stops the base if it hears
nothing for `HEARTBEAT_MS`, so "drive forward" is a control loop with a stop
condition rather than a call — and speech is ~1.5s each way, which makes a spoken
"stop" a poor way to stop a moving rover. The gamepad's View button and the
firmware heartbeat are the real stop, and a voice tool sitting next to them would
be a worse one that somebody might trust. The gimbal is fine by contrast: `T:133`
does not feed the heartbeat, so aiming is not mistaken for driving.

## Running it

```bash
ssh rpi 'cd ugv && python3 rover_daemon.py'
ssh rpi 'cd ugv && python3 rover_daemon.py --no-camera'     # lights and gimbal only
ssh rpi 'cd ugv && python3 rover_daemon.py --host 192.168.1.22'   # board over wifi
```

It centres the gimbal at startup, like every other script that commands it: the
angles are a model kept true by putting the camera where the code thinks it is,
since the firmware's `getGimbalFeedback()` is not reachable over JSON.

Do **not** run it alongside `drive_gamepad_pi.py` or `track_face_pi.py`. Those
are still standalone and still take the UART directly; the whole point of this
is that only one thing does.

Binding is `0.0.0.0:8769` with no authentication, the same trade
[face_detect](../face_detect/README.md) makes and with the same warning. The Pi
has no firewall in front of it.

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
- **The camera** is opened on demand and released `CAMERA_IDLE_S` after the last
  thing that wanted it. While tracking is running the loop owns it, so
  `count_faces` answers from what the loop last saw rather than trying to take a
  second look — an honest answer beats a contended one.
- **The gimbal** cannot be aimed by two things at once, so `look_at` and
  `center_camera` stop tracking first and say so in their result. That is what a
  person means by "look left" while the rover is following somebody.

## Checks

```bash
ssh rpi 'cd ugv && python3 selftest.py'    # 60 checks, no board and no camera
python rover_daemon/selftest.py            # the same, from the repo
```

It covers argument coercion, the servo limits, and — the one worth having — that
every schema the model is shown has a handler behind it and every handler is
offered. A name that does not match fails as "no such tool" in the middle of a
conversation, which is a poor place to find out.

## Deploying

```bash
scp rover_daemon/{rover_daemon.py,selftest.py} rpi:~/ugv/
```

Flat into `~/ugv/`, which is the layout already there and is what lets this
import `aiming.py` and `track_face_pi.py` from the same directory. Those imports
are deliberately made lazily, inside the functions that need them, so the module
loads — and its selftest runs — on a machine with no camera and no `aiming.py`
beside it.
