# Rover daemon

`rover_daemon` is the single owner of the rover's driver-board UART and gimbal
camera. It runs on the rover's Jetson Orin Nano and exposes the rover as a small
JSON-lines tool protocol on TCP 8769.

One owner is a correctness requirement, not a style choice. The ESP32 has one
UART and the gimbal camera is one device; separate processes independently
opening them can interleave motor/gimbal commands or race for the camera.

```text
browser / Alibaba Qwen session / diagnostics
                 |
                 | TCP 8769
                 v
           rover_daemon
            /        \
 driver-board UART   gimbal UVC camera
       |                 |
 motors/lights/IMU       +-- local YuNet face tracking
       |
       +-- loopback 8772 <-> ROS 2 / Nav2 <-> loopback 8773
```

## Current startup

The daemon is supervised from the `jetson` crontab through `run_daemon.sh` with:

```text
--vision --board-bridge --ros-nav
```

Use [`restart.sh`](restart.sh) rather than relaunching `run_daemon.sh` manually;
the supervisor is where those arguments live.

Normal deployment:

```bash
python deploy/deploy.py --only rover_daemon
```

The component is assembled from `rover_daemon/`, `driver_board/` and
`face_tracking/` and currently lands flat in `~/ugv/`. That deployment shape is
historical and intentionally left alone in the current cleanup.

## Tools

The exact schemas live in [`tool_schemas.py`](tool_schemas.py) and are the source
of truth. `list_tools` returns the current set so clients do not maintain their
own copies.

Core hardware/vision tools include:

| Tool | Purpose |
|---|---|
| `set_lights(level)` | set both headlights |
| `get_lights()` | return the last commanded light level |
| `battery()` | board voltage plus estimated charge state |
| `look_at(pan, tilt)` | point the gimbal; stops face tracking |
| `center_camera()` | back to rest — straight ahead, 10° above level; stops tracking |
| `count_faces()` | take a snapshot and run local YuNet |
| `start_tracking()` | start local YuNet + aiming loop |
| `stop_tracking()` | stop tracking and centre |
| `track_next()` | suppress the current target briefly and acquire another detection |
| `tracking_status()` | detector/target/gimbal state |
| `look()` | take a picture for the active model session when vision is configured |
| `show_map()` | top-down lidar map for that same session; optional `across_m` and `pixels` |
| `drive_to_map_point(across, down)` | drive to a place named as a fraction of the last `show_map` picture |

With `--ros-nav`, the daemon also exposes the navigation/driving tools backed by
Nav2. Their schemas and bounds are in `tool_schemas.py`; navigation behavior and
configuration are in [`../ros_nav/`](../ros_nav/).

`explore` is the one of those that is unlike the rest: it is not a move to a
place, it is the rover driving itself to the edge of the mapped area over and
over until there are no edges left. Two things about it are worth knowing before
it is called.

**It answers at once and the rover keeps going.** Every client here holds one
connection with one lock on it (`RoverClient` in
[`../voice_chat/rover_tools.py`](../voice_chat/rover_tools.py)), so a tool call
that waits for a ten-minute run holds that lock for ten minutes and
`stop_driving` queues behind it — a voice model would set the rover off and then
be unable to stop it, with somebody in the room asking it to. So the run goes on
a thread and the call comes back; `stop_driving` ends it. It still holds the
navigator's move mutex throughout, so `driving` is true and a `drive_to` is
refused as busy exactly as before.

**Calling it again while it runs reports rather than stops.** A model unsure
whether its call landed calls again, and a toggle would answer that by stopping
the rover — the opposite of what was asked, at the moment nobody would notice.
The reply also carries how the *previous* run ended, since nothing waits for one.

`minutes` is capped here rather than in the schema, between one and fifteen
(`EXPLORE_MIN_S`/`EXPLORE_MAX_S` in `rover_nav.py`), because a schema describes
and this is a rule: a model that has talked itself into an hour of unsupervised
driving gets fifteen minutes. Which gap in the map it drives to is
`../ros_nav/frontier.py`, and its README has the account.

`show_map` takes how many metres
of room to show (`across_m`) and how big a picture (`pixels`), the same two knobs
the console uses, and leaves them optional so "show me the map" is still a room.
It is not shown `half_extent_m`: that is how `map_png` talks, and a model handed
the half would pass six meaning six metres across and get twelve.

`drive_to_map_point` is how a model names a place on the map. `drive_to` has taken
a point in the map's own frame since the console learned to send taps, but that
pair is withheld from every model: nothing a model can see says where the rover is
in that frame, so it could only invent the numbers. A fraction of the picture needs
no such knowledge — the model has the image in front of it and the rover is in the
middle of it by construction — so it says where on the picture to go and the daemon
converts, using the pose the picture was drawn at and the same `mapimg.tap_to_point`
the console puts a mouse click through. `across` is 0 at the left edge and 1 at the
right, `down` is 0 at the top and 1 at the bottom.

Four things are refused rather than driven: a picture that has not been taken, one
older than `MAP_POINT_MAX_AGE_S` (the model is no longer looking at it, so a place
on it is a guess), a fraction outside the picture, and a point that is solid or
never-seen on the occupancy grid. Whether a *route* exists is left to Nav2, which
owns that question and answers it in words.

`drive_to` also takes a `heading_deg` — which way to be facing on arrival — and it
is withheld from models for the same reason the coordinates are: it is a bearing
in the map's frame. Left out, the goal faces along the way the rover travelled,
which is what makes a series of clicked destinations read as a journey. The one
caller that passes it is the drive console's world popup, sending the rover to
look at something it has placed: the arrival heading is the difference between a
spot the thing can be seen from and a rover that is actually looking at it. See
`world_state_viewpoint` in [world_state/README.md](../world_state/README.md),
which chooses the spot.

### The room the rover has already looked at

Three more tools appear when this rover has the `world_state` component and a
lidar under it. They are the semantic world state — what the rover recorded as it
drove around — asked in the only vocabulary a model has:

| Tool | Purpose |
|---|---|
| `find_thing(description)` | has the rover seen this, and how far away and which way is it now |
| `go_to_thing(description)` | drive to somewhere it can be seen from, and answer at once |
| `distance_between(first, second)` | how far apart two things it has placed are |

They live in [`rover_recall.py`](rover_recall.py), away from
[`rover_world.py`](rover_world.py), because the two have different audiences and
that is the whole of the difference between them. A person at the console holds a
map and wants `object:12` at (4.31, 2.09) with a cosine beside it; a person in the
room asks "can you find the bed" and wants two metres away, ahead and to your
left, seen a minute ago. So **nothing here hands a model an identifier or a map
coordinate** — the same rule, for the same reason, that keeps `x_m`/`y_m` out of
`drive_to`'s schema.

A phrase is the handle a model actually holds, so all three take one, and the same
phrase goes through the same ranking every time: "find the desk" and "go to the
desk" a minute later are about the same desk without anything being carried
between the calls. The ranking, its floor and the choice of where to stand all
belong to [`../world_state/`](../world_state/README.md); what this file adds is
which row of the ranking to answer about, and how to say it.

`go_to_thing` answers at once and the rover keeps driving, for `explore`'s reason
and through the same machinery — a trip that blocked the one connection a model
holds would block `stop_driving` with it. Asked again for the same thing it
reports; asked for a different one it stops what is running and sets off, which is
somebody changing their mind rather than a call that did not land, and is the
drive console's own rule. `ok` is false for everything that did not end in the
rover moving, because the model reads that field and says out loud what it did.

None of the three writes to the world state, and no model tool does.

A tool is offered only when its backend exists. In particular, `look` is withheld
when no current voice/image destination has been registered. An advertised tool
that can only fail encourages a model to claim it performed something that never
happened.

## Local face detection

The current detector is **YuNet in the daemon process**, implemented by
[`../face_tracking/yunet.py`](../face_tracking/yunet.py). There is no remote
face-detection service in the current architecture.

`RoverCamera` opens `LocalDetector` when face work is requested. OpenCV is a pinned
wheel unpacked beside the deployed source by `face_tracking/install_opencv.sh`.
The detector's scoring thresholds and the gimbal control policy are shared through
`face_tracking/aiming.py` rather than duplicated in the daemon.

`count_faces` normally uses a bounded one-shot capture, not the continuous
tracking feed. If tracking already owns the feed, tools use its newest current
frame rather than trying to open the camera a second time.

See [`../docs/face-tracking.md`](../docs/face-tracking.md) for detector timing,
frame-age and calibration details.

## Camera and `look`

The camera is discovered by stable UVC/by-id identity where possible. On the
Allwinner board `/dev/video0` may be the SoC decoder, so the daemon must not assume
the first video node is the gimbal camera.

A one-shot picture is captured as MJPEG and the camera is closed again. Keeping a
30 fps feed alive just in case another tool call arrives costs CPU/USB bandwidth
for no benefit; only face tracking keeps the continuous feed open.

**Only one thing may have the camera at a time, and the daemon enforces it.** Two
captures overlapping is not two pictures: one of the two exits in about 30 ms
having written no bytes and said nothing on stderr, so the caller gets "the camera
gave no whole picture" with no reason after it. Measured on the rover over 60
grabs: 46 that had the camera to themselves all succeeded, and 12 of the 14 that
overlapped another grab came back empty — the gap between grabs makes no
difference, only the overlap. It became a daily fault once the world state began
looking once a second, because the console asks for a frame every two seconds
through the same path and better than half of its pictures were lost. Captures now
queue behind one another, which costs the loser about a third of a second, and the
tracking feed opens under the same lock so it cannot come up on a camera a
one-shot grab is still holding.

For the current Alibaba voice session, `drive_web/omni_bridge.py` runs a small
frame service on loopback TCP 8774. The voice client registers that destination
through the daemon's control protocol. When Qwen calls `look`:

1. the daemon captures the latest trustworthy JPEG;
2. it POSTs that JPEG directly to the loopback frame service;
3. the tool result carries the short image token;
4. `voice_chat/session.py` attaches the matching frame to the Alibaba turn.

The browser never has to relay the JPEG.

The map renderer also receives the current gimbal direction so it can draw the
camera's view cone. Gimbal pan is positive to the right while map bearing is
positive to the left; the sign conversion belongs in one place and is covered by
self-tests.

## Battery and board telemetry

The ESP32 streams `T:1001` telemetry continuously. The daemon's board link parses
that stream and keeps recent state. Battery voltage comes from `v` in that
telemetry and is reported with an estimated charge state rather than pretending a
single loaded voltage is a laboratory state-of-charge measurement.

Board telemetry also provides the wheel encoders and gyro used by the ROS base
node. The daemon lends this path to ROS over loopback TCP 8772 so the ROS process
does not open the physical UART itself.

Serial reads must not hold the same lock used to send time-critical motor/gimbal
commands while waiting for a line. Stale/quiet board data is represented as stale
rather than blocking STOP/drive commands behind an unrelated telemetry wait.

## ROS / navigation bridge

The daemon itself is not a ROS process. `ros_nav` lives in its own RoboStack
Python environment, so the boundary is explicit loopback TCP:

- **8772**: daemon -> ROS: board telemetry and motor command path;
- **8773**: ROS -> daemon: Nav2 actions/status/map data.

`rover_daemon/ros_navigator.py` presents that remote navigation service through
the same high-level interface used by the daemon tools. A missing ROS stack is
reported as unavailable rather than silently falling back to an obsolete local
planner.

## Control calls not shown to the model

The daemon protocol also carries operational calls deliberately absent from
`list_tools`. These are for the browser console or infrastructure rather than for
model choice, for example:

- vision/frame-destination registration;
- `nav_status`;
- `map_png`;
- `camera_jpeg`;
- map/reset/diagnostic controls;
- detector diagnostics such as running YuNet over a supplied known image;
- the semantic world state -- `world_inspect`, `world_state_summary` and the rest.

Keeping them off the model schema avoids giving the model destructive or
implementation-detail controls simply because the human console needs them.

The world-state calls are off it for a further reason. They are a proof of concept
asking whether the rover builds a description of the room that stays coherent
across views, and until that has an answer no model should have the authority to
write to that description or to throw it away. They also answer in the console's
vocabulary -- identifiers, map coordinates, cosines -- which a model can neither
say out loud nor invent an argument for. A model reads the same store through
`find_thing`, `go_to_thing` and `distance_between` instead, and writes to it
through nothing. The store, the model boundary and the sidecar are
[`../world_state/`](../world_state/README.md); what lives in this daemon is
[`rover_world.py`](rover_world.py), which takes the picture through the same
`_whole_jpeg` that answers `camera_jpeg` -- the camera has one owner, and an
inspection is not a reason for a second process to open it -- and
[`rover_recall.py`](rover_recall.py), which is the model's half.

## Face tracking does not identify people

YuNet is a face **detector**, not a recognition/attribute model. The rover can
locate face boxes and maintain a target by geometric proximity; it cannot know
who somebody is, whether a person who left and returned is the same person, or
classify glasses/age/expression from this stack.

`track_next` therefore means "suppress the current detection briefly and acquire
another available face", not "identify a different person".

## Tests and verification

Offline/host self-test:

```bash
cd rover_daemon
python selftest.py
```

On the deployed rover the normal deployer runs the flat daemon self-test, restarts
the service and then calls `list_tools` over TCP 8769. That last call matters: a
copied file and a passing local test do not prove the supervised daemon restarted
with the new code.

Manual readiness check example:

```bash
python3 - <<'PY'
import json, socket
s = socket.create_connection(("192.168.1.80", 8769), 3)
f = s.makefile("rwb")
f.write(b'{"call":"list_tools"}\n'); f.flush()
print(json.loads(f.readline()))
PY
```

See [`../docs/deploy.md`](../docs/deploy.md) for the deployment/restart rules and
[`../docs/hosts.md`](../docs/hosts.md) for current ports/hardware facts.
