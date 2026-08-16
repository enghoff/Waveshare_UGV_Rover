# If we started again with an omni model

A clean-sheet design for the conversational side of the rover, assuming one
omni model — audio and vision in, speech out — on a GPU big enough not to
argue with. Written 2026-08-16.

Nothing here is built, and this is deliberately **not** a migration plan for
[voice_chat](../voice_chat/README.md). That service is a good answer to the
question it was asked, which was "how do we hold a conversation on 8 GB". This
document asks a different question and is allowed to throw away the answer to
the first one. What it would cost to actually move is in
[scaling-voice-chat.md](scaling-voice-chat.md#what-it-would-cost-this-codebase);
which card it would need is in [the same
document](scaling-voice-chat.md#the-vram-is-not-the-weights).

## The model is never invoked

That is the whole difference, and everything below follows from it.

Today a turn is a request: assemble a prompt, take the GPU lock, generate, tear
it down, wait. An omni service is a **process with a live context**, and audio,
video and events are pushed into it. There is no request, so there is no request
timeout, no lock to hold across a tool round trip, and no per-turn assembly. The
session is the primitive; the turn is something the model decides has happened,
not something the transport imposes.

The costs move accordingly. Cold start is paid once per session rather than once
per deploy, which makes it *worse* to get wrong — see [Warm is now a session
property](#warm-is-now-a-session-property).

```
  rover (Pi + mic array + camera)          gpu host
  -------------------------------          --------------------------------
  mic array -> AEC -> VAD gate --\
                                  >-- WebRTC --> session process
  camera -> encode -> change gate/       (one   )      |
                                         (per   )   omni model, live context
  speakers <---- played-samples  <-------(rover )      |
                 watermark ------------->              |
                                                       |
  safety supervisor  <--- modes/commands ---------------+
       |                                                |
  motors, servos, lights          spatial store  <-------
  lidar, bumpers -> stop
```

## The hard problem moves from latency to admission control

Worth stating on its own, because it re-points the whole design.

In a turn-based service the thing you optimise is time-to-first-audio, and
[voice_chat](../voice_chat/README.md#where-it-is-now) has the scars to show for
it — speculative transcription, a hand-rolled K/V rewind, a compiled decode
path. In an always-on service the model is already warm and already listening,
so latency is largely solved by the shape. What replaces it is **deciding what
is allowed into the context at all**, and getting that wrong is not slow, it is
expensive and then incoherent.

The gate must be cheap models at the edge — a VAD, a frame-difference test,
possibly a small salience classifier. **Never the omni model.** A rover parked
in an empty corridor should cost approximately zero tokens per minute, and that
is a property of the gate, not of the model's good judgement.

## Where things run, and the microphone moves

Three tiers, split by what physically cannot move rather than by what is
convenient.

| | runs there | never runs there |
|---|---|---|
| **rover** | actuators, camera, encode, the admission gate, the safety supervisor, the mode controllers | any model with weights worth naming |
| **GPU host** | the omni model, the session process, the compactor, the spatial store | anything on a control deadline |
| **anywhere else** | — | there is no client |

**The microphone belongs on the rover.** The desk-mic split in the current
service is a workaround for the Pi's audio never becoming reliable — see [The
rover client, and why there is not
one](../voice_chat/README.md#the-rover-client-and-why-there-is-not-one) — and
not a design anybody would choose. Starting again, you put a ReSpeaker-class
array on the robot and the conversation follows it around, which is the entire
point of it being a robot.

Two things fall out of an array that a desk microphone cannot give:

* **Hardware AEC**, which is a hard prerequisite for duplex and is otherwise a
  DSP project.
* **Direction of arrival.** "Who said that" becomes a bearing, and a bearing is
  exactly what [face_tracking](../face_tracking/) wants as a prior. Turning
  toward the speaker before anybody asks is a behaviour the current stack cannot
  have at any model size.

The Pi that runs the rover today is a Pi 1 Model B — see [hosts.md](hosts.md) — and
will not do encode plus AEC plus a change gate. This tier assumes it has been
replaced by something in the Pi 5 / Orin Nano class. That is the one hardware
purchase this design requires beyond the GPU.

## The transport is WebRTC, not a WebSocket

The current [protocol](../voice_chat/README.md#protocol) — raw PCM frames and
JSON over one socket — is a good fit for push-to-talk on a LAN and the wrong
one for a duplex session with a video track.

What you would otherwise hand-roll, in order of how much you would regret it:
acoustic echo cancellation, a jitter buffer, Opus, packet loss concealment,
congestion control, and a video track with keyframe negotiation. All of that is
`RTCPeerConnection`. The data channel carries events and tool traffic on the
same session and the same congestion state, so a saturated video track cannot
starve a stop command.

The cost is honest and should be written down: WebRTC brings a signalling path,
ICE, and a much larger dependency than `websockets`. It is worth it here only
because duplex audio is in the requirements. If it were not, the existing socket
would still be the right answer.

## Duplex, and the part everyone gets wrong

Barge-in is not "stop muting the microphone". Muting — `muted_until` in
[talk.py](../voice_chat/talk.py) — is a symptom; AEC removes it. The real
mechanism is what happens to the **context** when someone cuts in.

The model generated four sentences. One and a half of them reached a speaker
before the user interrupted. If you commit all four to the context, the model
now believes it said things nobody heard, and will refer back to them — it will
answer a question it was cut off before asking, or thank the user for agreeing
to something they never heard proposed. This is not a rare edge case; it is
every interruption.

So the audio sink reports a **played-samples watermark** back up the session,
continuously, and interruption commits the assistant turn *truncated at the
watermark* — plus a marker that it was cut off, because "I was interrupted here"
is information the model should have. Three things have to exist for that to
work:

1. Decode must be abortable mid-stream, and must surface a partial.
2. Sentence audio must be tracked from synthesis to speaker, not fire-and-forget.
3. The truncation point is decided by the **sink**, not by the generator, and
   not by the network.

That single mechanism is most of the difference between duplex that works and
duplex that gaslights the person using it.

## Vision is a budget, not a stream

Even on a card with room, you do not push 30 fps into a context window.
Qwen3-Omni's own published floors go from 78.9 GB at 15 seconds of video to
144.8 GB at 120 seconds — see [the VRAM is not the
weights](scaling-voice-chat.md#the-vram-is-not-the-weights). A continuous video
track *is* that curve. It is not a feature flag, it is the difference between a
card you can buy and one you cannot.

The design that fits inside a budget:

* **An ambient track** at roughly 1 fps and low resolution, admitted only when
  the change gate says the scene actually moved.
* **Full resolution on demand**, when the model asks or when something salient
  fires. This is what `look` is today, and it survives the redesign intact.
* **Vision evicted first.** A frame from five minutes ago should have been
  compacted into a sentence long before the audio around it is touched.

The current service already learned the downstream half of this the hard way:
pictures left in history poison the turns after them, which is what [a picture
does not outlive the turn that took
it](../voice_chat/README.md#a-picture-does-not-outlive-the-turn-that-took-it)
is about. In a persistent session that discipline stops being a per-turn purge
and becomes the eviction policy. **Same lesson, different mechanism** — and it
is the reason a naive "just stream the camera in" design fails on coherence
before it fails on VRAM.

## The model never closes a real-time loop

Tool calls stop being one blocking round trip and split three ways:

| shape | example | how it returns |
|---|---|---|
| **command** | set the headlights | fired, acked asynchronously; speech does not stall |
| **query** | how many people can you see | arrives back as an event in the stream |
| **mode** | follow that person | sets a controller running on the rover |

**Modes are the important one.** The model says "follow him"; a 30 Hz control
loop on the rover does the following, using classical CV — [face_detect](../face_detect/)
and [face_tracking](../face_tracking/) survive this redesign unchanged, and so
does whatever avoids obstacles from the [lidar](../lidar/). The model sets and
clears the mode and is told when it ends or fails.

A transformer in a servo loop is the wrong shape at any model size and on any
card. The omni model is a planner and a conversationalist; it is not in the
feedback path of anything with a motor on the end of it.

This also fixes something the current design papers over. Today
[`_run_tool`](../voice_chat/server.py) holds the GPU lock across a LAN round
trip on purpose, because releasing it would let turns interleave. With no lock
and no turn, that reasoning evaporates — but it is replaced by a new one:
**commands are ordered per rover and idempotent where possible**, because a
duplex model can now emit two of them before the first has acked.

## The safety supervisor is code, and it is on the rover

Absent from the current design, because the current design cannot drive. The
moment the model can, this is mandatory and it **cannot be a prompt**:

* speed and acceleration clamps that the model cannot raise;
* lidar, cliff and bumper stops that pre-empt any mode;
* a **deadman timeout on every motion mode**, so a dropped session stops the
  robot rather than leaving it driving into a wall with nobody listening;
* an audible and visible indication of which mode is running.

The model proposes; the supervisor disposes. It runs on the rover so that
losing the GPU host is a stop rather than a runaway, and it is deterministic so
that its behaviour can be argued about without sampling six times a cell.

## Memory in three tiers

A persistent session grows without bound, so compaction is not an optimisation,
it is the thing that makes the design possible at all.

1. **A raw rolling window** — the last few minutes of audio, the admitted
   frames, recent events. Bounded in tokens, not in time.
2. **Compaction to structured summary**, run periodically and on topic change.
   "We are in the kitchen; the user asked me to find their keys; I have checked
   the counter and the table."
3. **A durable spatial store**, retrieved rather than resident: where rooms are,
   where objects were last seen, who lives here. This is a database with an
   index, not a context window, and it is what makes "where did I leave my keys"
   answerable an hour later.

Tier 3 is the one with no counterpart in the current service, and probably the
one with the most product in it.

## Warm is now a session property

The existing service does not bind its port until it is warm, which turns a
~150 s cold start into an operational footnote. A persistent session cannot use
that trick: the model is warm, but *this conversation's* context is not, and a
session that dies takes its compacted state with it.

So the session process needs to checkpoint tiers 1 and 2 somewhere it can
recover them, and reattaching a dropped WebRTC connection must resume a session
rather than start one. Otherwise every network hiccup is an assistant with
amnesia, which is a worse failure than a slow one.

## What this deletes

Worth listing, because it is most of the current service and the deletions are
the evidence that this is a different design rather than a bigger one.

* **STT and TTS as stages.** `distil-large-v3` and Kokoro stop existing. The
  known, chosen voice goes with them — see [what it would
  cost](scaling-voice-chat.md#what-it-would-cost-this-codebase).
* **Speculative transcription.** [endpointing.py](../voice_chat/endpointing.py)
  keeps its VAD role as an admission gate, but `HANG_MS`, `spoke_early` and the
  take-it-back path exist to hide Whisper, and there is no Whisper.
* **The prefix cache.** `VOICE_PREFIX_CACHE` is the single largest win in the
  service today and it has no meaning in a live context — there is no prompt to
  re-prefill, because it was never torn down.
* **The GPU lock and `switch_service.sh`.** One resident model on a card with
  room; nothing to schedule.
* **`talk.py`.** There is no client.

## Build order

The sequencing lives in one place — [omni-build.md](omni-build.md), which
costs each piece and orders the work — rather than being duplicated here to
drift. Two of its principles are design facts rather than scheduling ones, so
they belong in this document:

* **Model-last.** The two genuinely hard parts — barge-in truncation and a
  supervisor that can be trusted — do not depend on which checkpoint wins and
  are miserable to retrofit, so they are built and proven against a fake model
  and a keyboard before anything non-deterministic exists in the system.
* **Measurement-first.** Whether an omni model calls the rover's tools when
  *spoken to* is the question that decides whether any of this is worth doing,
  it has never been measured — here or anywhere — and it costs days rather
  than months to answer. It goes before everything, including the hardware.
  See [testing one before renting
  anything](scaling-voice-chat.md#testing-one-before-renting-anything).

## What is not settled

The open questions live with the costed plan, in [what has to be
decided](omni-build.md#what-has-to-be-decided), each priced by what it costs
to answer. The two that can invalidate this design rather than merely tune it
are worth naming here: whether an omni model calls tools reliably when spoken
to at all — everything above assumes it does — and whether one model is the
right shape, since a single omni model handling both banter and deliberate
perception may be worse at both than a fast conversational model plus a
heavier VLM the modes can call. Neither can be argued from first principles.

Every number in here is a design assumption. The only measured figures are
the VRAM floors quoted from [scaling-voice-chat.md](scaling-voice-chat.md),
and those are Qwen's published minimums rather than anything run here.
