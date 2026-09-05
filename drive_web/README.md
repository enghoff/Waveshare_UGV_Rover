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

The map image is fetched only when its generation changes. Manual map clicks and
world-state destinations go through the daemon's existing route planning and
drive checks. Browser disconnection does not bypass the daemon's own movement
timeouts and stop behavior.

## World-state popup

The popup has entity, map and observation views over one read-only data source.
A search phrase filters all views together. Selecting an entity shows every
stored observation used for it, including the source frame, measured box, pose,
bearing, range and uncertainty where available.

The map draws observation origins, bearings, visibility, placement extent and
error. It uses the server-provided map transform rather than duplicating map
geometry in the browser. Placements from an old map session are shown as stale
and are not offered as current destinations.

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
