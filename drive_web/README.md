# Browser drive console

The console serves rover status, manual driving, camera controls, maps,
world-state inspection and browser audio over HTTPS on port 8771:

```text
https://192.168.1.80:8771/
```

It is intentionally usable without the voice dependencies. The browser is the
microphone and speaker; the rover holds the Alibaba Qwen Omni session and all
credentials.

## Install and run

```bash
ssh orin 'sh ~/ugv/drive_web/install.sh'
ssh orin 'sh ~/ugv/drive_web/install_websockets.sh'
ssh orin '~/ugv/drive_web/restart.sh'
```

`install.sh` adds the supervisor to the `jetson` user's crontab and creates TLS
material under `~/.ugv/tls/` when needed. A newly generated CA must be trusted on
the workstation. The leaf certificate covers the stable service address and the
rover's local hostname; transient DHCP addresses are not certificate names.

Use `restart.sh` rather than launching the child directly. The supervisor keeps
the console up and preserves its required arguments.

## Source layout

- `drive_web.html` contains the page structure.
- `drive_web.css` contains all presentation.
- `drive_web.js` handles connection, driving, status, maps and voice.
- `drive_world.js` holds world-state data, lists, details and wiring.
- `drive_world_map.js` draws placed entities, observations and uncertainty.
- `drive_world_observations.js` owns the paged observation grid and zoom view.
- `drive_web.py` serves HTTPS/WebSocket traffic and static assets.
- `drive_session.py` holds the rover connection and state snapshots.
- `omni_bridge.py` connects browser audio to the hosted realtime model.

The world scripts load before `drive_web.js`, which calls `start()`. Assets are
read from disk per request, but deployed changes still use the normal restart and
verification path.

## Driving and status

Manual controls send bounded actions through the daemon on TCP 8769. The console
does not open the driver-board UART or talk directly to ROS. It displays the
daemon's navigation, battery, link, camera and movement state.

On a three-column screen, driving, voice, face tracking, headlights, battery and
network controls sit on the left; the map is in the centre; camera, depth and
navigation status sit on the right. The same stacks wrap on narrower screens.

The drive pad surrounds a top view of the rover. Turns run horizontally, from
15 degrees nearest the centre to 45, 90 and 180 degrees farther out; left is on
the left and right on the right. Forward distances run upward and reverse
distances downward. Forward increases from 0.1 m nearest the centre to 0.25,
0.5 and 1 m farther out; reverse stops at 0.5 m, the navigation backend's blind
reverse limit. Longer negative drives turn around first and are absent from
the reverse pad. Each button sends that one bounded movement at automatic speed. Arrow keys
drive 0.5 m forward/backward or turn 90 degrees; space and Escape stop, and
plus/minus zoom the map. Motion controls wait for a connected, idle rover.

Under the gimbal camera's picture is the OAK's depth map. It travels as
millimetres -- zlib-compressed by the daemon, about 34 kB, served at
`/depth.zlib` and inflated in the browser -- so the page colours it itself
(Turbo, near red and far blue, linear from 0.2 to 6 m) and reads the distance under
the pointer in metres. It is fetched at the camera's own pace on the same
connection, and only while the depth lamp is on: the rover's wheels decide when
the OAK is awake and a request never wakes it. When the OAK goes off the last
map stays up, with the time it was taken beneath it.

The map image is fetched only when its generation changes. Manual map clicks and
world-state destinations go through the daemon's existing route planning and
drive checks. Browser disconnection does not bypass the daemon's own movement
timeouts and stop behavior.

**run**, beside **world**, starts an autonomous run with nothing asked: the
daemon's `autonomy_start` with `via` `console`, which gives the run no limit on time,
travel or actions, three failures in a row, and the mapped floor as its area (see
[the daemon](../rover_daemon/README.md) and
[the runbook](../docs/runbooks/autonomy-session.md)). It reads **end run** while a
run is open, including one an agent started, because it is drawn from what
`nav_status` reports and not from the click. Pressing it then is the ordinary stop.
When the last tab has been gone for the orphan grace, the console stops a run that
was started from it, as it stops a move of its own; a run an agent started is left
alone.

A red line under the header stays up for as long as `nav_status` carries a
`rotation_fault`: the daemon's reason why the gyro cannot be trusted to measure a
turn ([R-SAFE-17](../docs/requirements/safety.md#r-safe-17)), in the daemon's own
words. Unlike a notice it does not fade, because the fault comes with a boot and
lasts until the power is cut.

## World-state popup

The popup has entity, map and observation views over one read-only data source.
A search phrase filters all views together. Selecting an entity shows every
stored observation used for it: the source frame with the measured box drawn on
it, and under it a table of every field stored with the look -- pose, bearing,
OAK range or why there is none, uncertainty, the resolver's note -- plus how the
look stands to the entity's settled position.

The map draws one mark per placed entity and nothing else. Bearings, sight lines
and names were removed on 2026-09-05: a room's worth of them hid the map they
were drawn on, and the same facts are read as numbers on each look's own row.
Pointing near a mark selects that entity after a short delay, which fills the
list, the detail pane and the observation stream; the nearest mark wins rather
than the topmost, and the delay is what stops a swept pointer buying a request
per mark. Uncertainty is drawn for the selected entity only.

A bar beside the search box narrows the list and the map together. By default
it keeps only the best-placed entity within 1 m of any other: the smallest
`stated_uncertainty_m` -- the depth camera's figure where it ranged the thing,
the crossing's where not -- among entities seen from three viewpoints or more,
with fewer viewpoints next and an entity whose looks disagree with it last.
Most of a crowded map is one object recorded several times: on 2026-10-10, 630
entities sat a median 0.14 m from their nearest neighbour, and the thinning
left about 75. Ranking by viewpoint count first, as it briefly did, showed a
thing with a 1.12 m error over a neighbour ranged to 0.2 m, because the counts
saturate on anything looked at often. The selected entity is never thinned
away, and under a search the best match wins its patch. Two further steps add a placement bar
(three viewpoints; four viewpoints and 0.3 m), and "every placed thing" turns
both off. What each step hides is counted under the map.

The popup has its own map picture, at `/world_map.png`, drawn wide enough to hold
both the entities and the room; the driving map is a fallback until the first one
arrives. The view is fitted to the map itself with a small margin, using the
`known_box_m` the daemon measures off the occupancy grid — not off the rendered
pixels, where the camera cone and the scale bar would move it. It uses the
server-provided map transform rather than duplicating map geometry in the
browser. Placements from an old map session are shown as stale, are not offered
as current destinations, and are counted under the map rather than silently
absent from it.

The observation stream pages older rows by timestamp and ID. Images load lazily;
the browser retains tile nodes so incoming observations do not move the item
under the pointer or discard an open detail view.

Direct inspection and clearing are console controls, not voice tools. A model can
read placed world state through the daemon's controlled tools and can ask the
navigator to approach a selected thing.

## Voice

The `/audio` WebSocket carries microphone frames to `omni_bridge.py` and returns
model audio. The DashScope key remains at `~/.ugv/alibaba.key` on the rover and
never reaches the page.

Tool calls stay on loopback. A `look` request receives its camera frame through
the loopback frame handoff on port 8774. Browser barge-in cancels pending output
through the realtime session rather than mixing old and new replies.

Port 8774 also carries `POST /notice` in the other direction, which is how a
trip that has finished reaches the model: nothing waits for a background move, so
without it the rover announces that it is setting off and never mentions arriving.
The notice is spoken when the conversation is free and held for the model's next
turn when it is not. With no conversation running it is accepted and dropped, and
the transcript says so.

If voice dependencies or the hosted service are unavailable, manual driving and
status remain available. The page reports current failure state rather than
showing setup instructions in the control surface.

## Verification

```bash
python drive_web/selftest.py
python drive_web/test_page.py
python drive_web/test_network.py
python drive_web/test_session.py
```

The main self-test covers the server, page/assets, protocol, world panel, session
state, pictures and audio framing. Hardware and browser-media behavior still
require the running service:

```bash
ssh orin 'curl -sk https://127.0.0.1:8771/'
```

Final proof is a successful HTTPS response plus live rover state. Camera, audio
and movement tests should be run only when their real devices and safe floor space
are available.
