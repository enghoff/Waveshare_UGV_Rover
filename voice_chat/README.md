# Voice chat: Alibaba realtime Qwen Omni

The current voice system is the browser console talking through the rover to
Alibaba DashScope's realtime Qwen Omni service. There is no local GPU inference
server in this repository's current architecture.

```text
phone / desk browser
        |
        |  wss://rover:8771/audio   16 kHz PCM up, 24 kHz PCM down
        v
 drive_web/omni_bridge.py
        |
        +-- voice_chat/session.py -----------------------------+
        |                                                      |
        |         wss://dashscope-intl.aliyuncs.com/...        |
        +--------------------------------------------------> Qwen Omni
        |
        +-- 127.0.0.1:8769  rover_daemon tools
        |
        +-- 127.0.0.1:8774  frame handoff for `look`
```

The rover holds the model session. The browser supplies microphone input and
plays returned audio; it does not hold tool state or an Alibaba API key. Tool
calls execute against the daemon over loopback, so driving, lights, gimbal and
navigation do not make an unnecessary trip through another machine.

## Current files

| File | Purpose |
|---|---|
| [`session.py`](session.py) | Alibaba realtime WebSocket protocol, turn/tool handling and audio events |
| [`prompts.py`](prompts.py) | current spoken system/tool/vision prompt and tool-schema reader |
| [`rover_tools.py`](rover_tools.py) | TCP client/discovery for `rover_daemon` |
| [`talk_frames.py`](talk_frames.py) | small HTTP frame stash used to hand a `look` image into the current session |
| [`console_model.py`](console_model.py) | shared wording/pacing for the drive console |
| [`mock_rover.py`](mock_rover.py) | invented rover used to exercise the client/console without hardware |
| [`mock_room.py`](mock_room.py) | the room that rover drives in: ranges, a planner that refuses, and the map |
| [`test_talk.py`](test_talk.py) | offline tests for the current session helpers, prompts, frames and rover client |

`drive_web/omni_bridge.py` adapts `Session` to the browser's microphone/speaker
WebSocket. The protocol stays in `session.py`; the web console does not carry a
second implementation of the Alibaba API.

## Endpoint and model

`session.py` defaults to the international DashScope realtime endpoint and the
current Qwen Omni realtime model:

```text
QWEN_REALTIME_URL=wss://dashscope-intl.aliyuncs.com/api-ws/v1/realtime
QWEN_REALTIME_MODEL=qwen3.5-omni-plus-realtime-2026-03-15
```

Both may be overridden through environment variables when the service changes.
The model name and event shapes in `session.py` are authoritative; this README
should be updated if they change.

Turn taking is the hosted service's semantic VAD. Audio sent by the browser is
16 kHz mono PCM. Returned audio is 24 kHz PCM and is played by the browser.

## Starting a conversation

Open the rover console:

```text
https://192.168.1.80:8771/
```

The microphone control starts the Qwen session on demand. It is not kept open
simply because `drive_web` is running. The console permits one active microphone
owner at a time; two browsers must not feed one conversation context.

The browser reports how much generated audio was actually played. On interruption
(barge-in), that playback position is sent back into the session so the model's
history reflects what the user heard rather than what the service had already
generated.

## Credentials

The DashScope key lives on the rover:

```text
~/.ugv/alibaba.key
```

Keep it mode 600. It is deliberately outside `~/ugv`, so source deployment cannot
overwrite it or carry it back into Git.

The browser never receives this credential. Starting a conversation has the same
trusted-home-LAN boundary as the console's ordinary driving controls.

## Prompts and tools

The spoken prompt lives in [`prompts.py`](prompts.py). It has three pieces:

- the short spoken-system prompt;
- the tool-use rules appended when tools are available;
- the vision rule appended when the session can take a picture.

Tool schemas themselves do **not** live in `voice_chat`. They are parsed from
`rover_daemon/tool_schemas.py`, which remains the single source of truth for what
the rover offers. A schema improvement therefore reaches the model without a
second copy being maintained here.

The prompt is intentionally plain-spoken: replies should be short sentences that
can be heard, not markdown that a synthesiser would awkwardly read aloud. A tool
request is considered performed only after the corresponding rover call actually
returns; the model is explicitly instructed not to narrate an intention as though
an action happened.

Since 2026-09-04 the list includes two tools that answer from the room the rover
mapped earlier rather than from what is in front of it — `find_thing` and
`go_to_thing`, so that "can you find the bed" and "move to the desk" are things
this rover can be asked. The line both have to draw is against `look`: "can you
find the bed" and "what can you see" are one sentence to a model holding a camera
tool, and the difference is the point — `look` is a photograph of what is in front
of the rover now, and these are the whole place, most of which is behind it. Each
description says in so many words that it takes no picture, which is the shape of
fix that `count_faces` needed for the same collision — naming what a tool is *not*
for is what took "how many people can you see" from 0/6 to 6/6 there.

`find_thing` also hands back where the thing is on the map, and its description
spends three sentences on what that is for: comparing one thing with another, so
that "how far is the bed from the desk" is two calls and a subtraction. **That is
arithmetic done by the model**, which is the thing this repository otherwise
avoids asking of one — `_where` in rover_camera.py describes a face in words for
exactly that reason. It is two coordinates and a Pythagoras rather than a bearing
model, and a wrong answer is a wrong number said out loud rather than a rover
driving somewhere; but it is unmeasured, and if it turns out to be beyond the
model the fix is a tool that does the subtraction.

**Whether any of this reaches the right tool has not been measured**: these two
have no sampled numbers of their own, and until they do, a report that the model
reaches for the wrong one is a wording change waiting to be made in
`rover_daemon/tool_schemas.py` rather than a bug in the rover.

## Seeing through the gimbal camera

`look` is a rover tool, but an image is too large and too easy to misroute to pass
around as a JSON/base64 tool result. The current path is:

1. `drive_web` starts a small frame server on loopback TCP 8774.
2. The voice client tells the daemon where that server is.
3. When Qwen calls `look`, the daemon takes a JPEG and POSTs it to the loopback
   frame server.
4. The tool result contains the short frame token/name.
5. `session.py` attaches the matching JPEG to the Alibaba turn.

The frame stash is bounded by count and age and each frame token is consumed once.
A missing frame is reported as missing rather than silently letting the model
answer from nothing or from an older picture.

The camera is on a moving gimbal, so a new visual question normally takes a new
picture. Reusing an old image after the camera has moved is a confident answer
about somewhere the rover is no longer looking.

`show_map` uses the same frame handoff. It takes how many metres of room to show
(`across_m`) and how big a picture (`pixels`); leave both out for about six metres
across. Those names are deliberate: `map_png`'s `half_extent_m` is half of the
same quantity, and a model handed it would pass six and get twelve.

Once a map has been taken, `drive_to_map_point` lets the model drive to a place on
it by saying whereabouts on the picture that place is — `across` from 0 at the left
edge to 1 at the right, `down` from 0 at the top to 1 at the bottom, with the rover
in the middle at 0.5 and 0.5. The reply to `show_map` says so in its caption, so
the model is told the picture can be pointed at on the turn it is looking at one.

The point of naming a place this way is that it is the only frame the model
genuinely has. Metres on the map would have to be invented, because nothing the
model can see says where the rover is in that frame; a fraction of the picture is
a property of the image on its screen. The daemon holds the pose the picture was
drawn at, so the place stays the place that was pointed at however long the
conversation takes to get round to going there — but not indefinitely: past
`MAP_POINT_MAX_AGE_S` the picture has left the model's context and it is asked to
take a fresh one rather than point at a map it is remembering.

## Rover connection

`rover_tools.RoverClient` talks to TCP 8769. It discovers the rover, remembers the
working resolved address to survive an mDNS wobble, and re-resolves when that
address genuinely stops answering. Tool schemas are fetched from the daemon at
connection time rather than compiled into the client.

Long-running script calls have a longer socket timeout than ordinary hardware
calls; the timeout returns to the normal value afterwards. A daemon restart that
closes a kept connection costs a reconnect, not the rest of the conversation.

## Deploying with the console

The current voice helpers are deployed beside `drive_web`:

```bash
scp voice_chat/{console_model,rover_tools,session,talk_frames,prompts}.py \
    orin:~/ugv/drive_web/
```

Normally use the automated deployer instead:

```bash
python deploy/deploy.py --only drive_web
```

The rover needs the `websockets` wheel used by the Alibaba client; the component
installer handles that separately:

```bash
ssh orin 'sh ~/ugv/drive_web/install_websockets.sh'
```

## Tests

Current offline checks:

```bash
python voice_chat/selftest.py
python drive_web/selftest.py
```

They cover the rover client, connection-error explanations, prompt/schema
extraction, frame handoff, move commentary and the realtime session plumbing that
can be exercised without making a live cloud call.

For console development without hardware:

```bash
python voice_chat/mock_rover.py --drive
python drive_web/drive_web.py --no-idle --bind 127.0.0.1
```

A live Alibaba call depends on the external service and account, so the repository
keeps deterministic protocol/unit checks separate from that integration test.
