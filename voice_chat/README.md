# Voice chat

Speech in, speech out. The microphone is the drive console
([drive_web/](../drive_web/README.md)); the protocol is
[session.py](session.py). The model is Alibaba's hosted omni service in
Singapore; MEDIA is not in that path.

```
  phone / desk browser                      dashscope-intl, Singapore
  --------------------                      -------------------------
  mic -> wss://rover:8771/audio --16k pcm-->  qwen3.5-omni-plus-realtime-2026-03-15
                                                |  text + tool calls
  speakers <-- playback <--24k pcm--------------+
       |
       +-- tool call --> rover_daemon.py on loopback --> the board, the camera

  the camera --- one JPEG, POST /frame --> 127.0.0.1:8774 --> into the session
```

Turn-taking is the service's (`semantic_vad`). The local GPU stack on MEDIA
([server.py](server.py)) is still in this directory. It has no client anymore.

A second program has no model in it at all. The drive console is the rover's
driving tools wired to buttons, and it exists because a conversation is the wrong
instrument for measuring one: when a turn comes back short you need the number,
not a paraphrase of it several seconds later. The rover hosts it at
`http://<rover>:8771/` — see [drive_web/](../drive_web/README.md) and
[Driving it by hand](#driving-it-by-hand).

It also carries **tools**: the model can switch the headlights, aim the camera,
look through it, count the people it can see and start or stop face tracking.
None of that happens here or on the desk — it is performed by
[rover_daemon/](../rover_daemon/) on the rover, which owns the hardware. See
[Tools](#tools) and [Seeing](#seeing).

## Why it is split here

The measurements in this file are the MEDIA GPU stack ([server.py](server.py)).
Endpointing — deciding the speaker has stopped — is the one piece that could
live on either side, and it goes local. It needs no model, and doing it locally
means silence never crosses the network: the link only ever carries a real
utterance and a real reply. The GPU client therefore needs no torch, no CUDA and no
model weights beyond what the service already loads.

Everything downstream of that is one process rather than three services, because
a turn is a strict chain (audio → text → text → audio) with nothing to overlap
between stages. Splitting it would buy no parallelism and cost two extra network
hops plus two extra CUDA contexts on a card that has ~6.5GB free once Windows
has taken its share.

The one thing that *is* overlapped is TTS against decode. The reply is split into
sentences as it decodes and each is spoken as soon as it is complete, so audio
starts after the model's first sentence rather than its last.

## Measured

Warm, on the 3070, for a two-second question answered in three sentences:

| stage | time |
|---|---|
| STT (`distil-large-v3`, int8_float16) | 0.18s |
| LLM (`Qwen3-4B`, int4, compiled) | 0.6–0.9s, 46 tok/s |
| TTS (`Kokoro-82M`) | 0.14s |
| **first audio out** | **~1.3s** |
| whole turn | ~1.7s |

~5.4GB of the card's 8GB, all three models resident. Cold start is ~60s and the
service does not bind its port until it is warm, so `/health` answering means
the next turn is a fast one.

The first turn used to cost 56s and the second 76s: `torch.compile` was
rebuilding for every new prompt length, and a conversation's prompt grows every
turn. `VOICE_COMPILE_DYNAMIC=1` compiles once for a range of lengths instead, and
`_load()` spends that cost at startup rather than on the first question.

### The table above is the *no tools* case

Measured 2026-08-16 through `/chat`, temperature 0, so the reply — and therefore
the token count — is identical between runs and a wall-clock difference is a real
difference. Ten synthetic tool schemas, sized like the rover's:

| tools offered | time to answer |
|---|---|
| 0 | 0.24s |
| 5 | 1.30s |
| 10 | 1.97s |

The 5- and 10-tool rows return a byte-identical 98-character reply, so the 0.67s
between them is not decoding — it is prompt processing, about **134ms of prefill
per tool schema**. Attach the rover's ten and roughly 1.4s of every question is
spent re-reading schemas that have not changed since the service started. Prefill
is where int4 hurts, for the reason already written down under `VOICE_INT4_SKIP`:
weight-only int4 unpacks for the wide GEMMs prefill is made of.

Which is to say the headline number above was measured on the configuration
nobody runs. A desk with a rover attached was paying ~1.4s a turn that no amount
of faster decoding would have recovered.

### Where it is now

Same conversation, five referential turns with ten tools attached, `/chat` at
temperature 0. The answers are **identical** in both columns — that is the check
that matters, not the timings; a cache that kept the wrong prefix would show up
as a wrong answer, so the two were diffed rather than eyeballed.

| turn | `VOICE_PREFIX_CACHE=0` | `=1` (default) |
|---|---|---|
| 1 (nothing cached yet) | 1.42s | 1.57s |
| 2 | 1.07s | **0.28s** |
| 3 | 1.07s | **0.28s** |
| 4 | 1.15s | **0.34s** |
| 5 | 1.23s | **0.43s** |

Steady-state turns are ~3.8x faster, and the whole exchange 2x. The first turn is
*slower* by 0.15s, which is the trade documented under `VOICE_CUDAGRAPHS` below.

## Running it

The conversation is the drive console's microphone, on the rover:

```
https://bpi-m4zero.local:8771/
```

See [drive_web/README.md](../drive_web/README.md). There is no desk client.

The local GPU service on MEDIA still exists ([server.py](server.py)). It shares
the card with `grounding-dino` and `qwen3-vl` and is not meant to run alongside
them. Switch with the interlock, which now has three options:

```bash
ssh root@media ~/switch_service.sh voice     # or: dino, qwen
```

(That script was `switch_vision_service.sh` until this landed — renamed because
voice-chat is not vision, and what the set has in common is the card. Its source
of truth is `services/switch_service.sh` in the **mt4** repo, not this one.)
It has no desk client in this directory.

One line at the bottom says what the microphone is doing, rewritten in place:

```
● listening     open, room tone only
● hearing you   the endpointer has decided a turn is under way
○ thinking      utterance sent, waiting on the card
○ speaking      the reply is playing, and the mic is muted so it does not
                endpoint the assistant's own voice
```

The filled half is an open microphone and the hollow half a closed one. Both
closed states are deliberate and neither is visible any other way — a muted
client looks exactly like a dead one — which is the whole reason the line
exists. It is drawn only on a terminal: redirected to a file it is silent, and
where the encoding cannot take the dots (a piped stdout on Windows is cp1252)
it falls back to `[ listening ]`.

If the service is not up — which is the usual state, since the card is shared —
the client says which host it tried and how to start it, rather than a `websockets`
traceback. It distinguishes the four cases that need different answers: a name
that will not resolve, a port that refuses (which is also what a service still
loading its weights looks like), a host that answers nothing at all, and
something on the port that is not this. A connection lost mid-conversation —
a restart, or the card being switched away — ends the same way, with a line
rather than a stack trace.

Client dependencies for the MEDIA GPU service are [requirements.txt](requirements.txt).

The service binds `0.0.0.0` rather than loopback, so there is no tunnel. The
microphone is on a different machine from the card and a tunnel between them is a
process and a reconnect loop sitting in the middle of a conversation. The port is
open on the LAN with no authentication in front of it, the same as `face-detect`
beside it; the Hyper-V inbound rule covers 8765-8774. Do not do this on a network
you do not own.

## The rover client, and why there is not one

There was a second client on the rover itself — Bluetooth headset in, JBL Flip
out, PipeWire driven through `pw-record`/`pw-play` pipes because the rover has no
PortAudio, and a hand-rolled RFC 6455 WebSocket because `apt` there needs a
password we do not have from a script. It worked, and it was never reliable
enough to hold a conversation through: the Pi 1 runs its Bluetooth dongle, its
wifi dongle and the camera off one weakly fused USB bus, and an always-open SCO
microphone alongside A2DP is more than that radio comfortably does.

It was removed on 2026-08-15 along with `wsclient.py`, and for nine days after
that speech happened only where there was a desk with a real microphone. Then it
came back to the rover from the other direction: not a sound card on the board,
but a browser page the board serves, with the audio crossing the wifi as PCM and
the conversation held against a hosted model. See [The same conversation, with no
GPU in it](#the-same-conversation-with-no-gpu-in-it). The findings below are about
audio hardware on the Pi 1 and are untouched by that.

Two findings from that work outlive the client and are kept, because they are
properties of the machine rather than of the code that went:

- **PipeWire on the Pi 1 had no realtime priority**, because `admin` was not in the
  `pipewire` group and so the `rtprio 95` in
  `/etc/security/limits.d/25-pw-rlimits.conf` never applied. Fixed with
  `usermod -aG pipewire admin` and a reboot. See [docs/hosts.md](../docs/hosts.md).
- **A process waking 50 times a second breaks audio on that box, and a CPU hog
  does not.** A deliberate spin loop cost 0–2 dropouts; a 20 ms read loop cost 36
  in 15 seconds. Throughput the scheduler handles, latency it does not — so
  anything on that board that reads a pipe should read it in bulk. That is why
  `track_face_pi.py` forwarding whole frames is fine and why its 4 kB read chunk
  would not have been.

## Tools

The rover can be asked to do things, not just talked to: *turn the lights on*,
*dim them a bit*, *look to your left*, *how many people can you see*, *follow
that person*, *find somebody else*, *stop following*.

**Nothing here performs them.** The rover's hardware is a single UART and a
single camera, so exactly one process may own it, and that process is
[rover_daemon.py](../rover_daemon/rover_daemon.py) on the rover. A call travels
from the model, back down this WebSocket to `talk.py`, and on to the daemon:

```
  you --speech--> MEDIA: model decides ---{"type":"tool"}--> talk.py
                         model answers <-{"type":"tool_result"}-  |
  speakers <--audio--                                             | TCP 8769
                                              rover_daemon.py <---+
                                              UART -> ESP32,  camera -> MEDIA
```

**Nobody but the daemon knows what the tools are.** `talk.py` asks it with
`list_tools` on connect and passes the schemas straight up in a `hello`; this
service puts whatever it is given into the prompt. So `server.py` contains no
mention of lights, cameras or serial ports, `rover_tools.py` is ~120 lines of
socket and no schemas at all, and adding a tool is a change to the daemon alone
— nothing else is redeployed. A client that announces nothing gets a plain
conversation with no rover in its context, which is what `--rover none` is for.

The daemon is probed once at startup and the tools are offered only if it
answers. Tools that cannot reach the rover are worse than no tools: the model
says out loud that it has switched the lights on, and nothing happens.

Measured over the LAN, answering with a stub board rather than the rover, so
these are the model's costs with no serial time in them:

| said | tool | first audio |
|---|---|---|
| "Please switch the lights on." | `set_lights{"level":255}` | 2.90s |
| "Are the lights on right now?" | none — answered from history | 1.60s |
| "Dim them to about half." | `set_lights{"level":128}` | 3.24s |

So a tool turn costs about **1.3–1.6s more** than a plain one — a whole second
decode, not the round trip, which is milliseconds. The card is held across that
round trip rather than released, because letting another turn interleave between
a call and its result is worse than leaving the GPU idle for a moment.

Note the middle row: it answered "are they on?" from the previous exchange
instead of calling `get_lights`, which is sensible and also means `get_lights` is
rarely reached. A conversation will therefore keep asserting whatever it last
set, so if the gamepad has moved the lights since, the answer is confidently
wrong. That is the same staleness as the tool's own, one step further removed.

### Getting it to actually call them

The first version of this shipped a rover that lied. Asked *"can you switch the
lights off?"* it answered *"I'm turning the lights off. The headlights are now
off."* and called nothing — which is indistinguishable from working, right up
until you look at the rover. Two independent causes, and it took both fixes.

**The wording of the tool prompt.** The original said to use the tools "when you
are asked to do something they cover", and the model read a question as a
question. Measured over six samples a cell:

| request | original prompt | current prompt |
|---|---|---|
| "Hello, can you switch lights off?" | **0/6** | 6/6 |
| "Can you look to your left?" | **0/6** | 6/6 |
| "Start following me." | 3/6 | 6/6 |
| "What is your name?" | 0/6 | 0/6 *(want 0)* |

Zero out of six, at every temperature. What fixed it was saying two things
explicitly: that a request phrased as a question is still a request, and that it
has done something *only* if it called a tool — never to claim it switched, moved
or started anything otherwise.

**And the temperature.** Whether the model acts turned out to be a sampled
decision, not a determined one: at 0.7, *"Start following me"* called 3/6. See
`VOICE_TEMPERATURE` in [server.py](server.py) for the table; the default is now
0.2. Nothing over-calls at any temperature, so this costs only variety.

Both of those were found through `/chat`, which exists for exactly this: text in,
the *decision* out, no speech synthesised and no tool performed. A spoken attempt
costs a TTS round trip, a decode and a playback — nine seconds to learn one bit,
and a bit that is noisy enough that three samples will mislead you. It did:
mid-investigation a 3-sample spoken test said the new prompt had made things
worse, and 6-sample text runs said it had taken two cases from 0/6 to 6/6. Do not
draw conclusions here from single attempts.

```bash
curl -s -X POST http://media.local:8767/chat -H 'Content-Type: application/json'   -d '{"text": "can you turn the lights off", "tools": [...], "temperature": 0.2}'
# -> {"reply": "...", "tool_calls": [{"name": "set_lights", "arguments": {"level": 0}}]}
```

`system` and `temperature` are overridable per request, which is the point — a
restart to try one wording costs ~150s, and this way a four-prompt sweep is one
script.

Four things about this end are not obvious:

- **A tool call is text, and Kokoro will read it out loud.** It has to be caught
  before the sentence splitter, which is what `_ToolSniffer` in
  [server.py](server.py) does. It watches for two shapes, because which one
  arrives depends on the *tokenizer*, not the model: Qwen wraps a call in
  `<tool_call>` markers, and a streamer built with `skip_special_tokens=True`
  could eat them before this ever sees them, leaving a bare JSON object as the
  whole reply. Measured on the deployed tokenizer, the markers **do** survive —
  they are not special tokens in Qwen3-4B-Instruct-2507 — but both are handled,
  since that is a property of a model file rather than of this code.
- **The markers arrive in sub-word pieces.** `<tool` and `_call>` is an ordinary
  way for one to turn up, so the sniffer holds back any trailing fragment that
  could still become a marker. Getting this wrong speaks the word "tool" and
  swallows the rest of the reply.
- **The last decode of a turn is offered no tools.** Otherwise a model that has
  decided everything is a tool call spends the whole turn calling them and the
  user hears nothing, which is indistinguishable from a crash.
- **`_trim` cuts whole exchanges**, since a turn that called a tool is four
  messages rather than two, and a call stranded without its result makes the
  model start narrating plumbing.

## Seeing

With `VOICE_VISION=1` the reply model is `Qwen3-VL-4B-Instruct` and the rover can
be asked what it can see. The interesting part is not the model, it is where the
picture goes:

```
  rover (the camera)              desk (talk.py)              media (this)
  ----------------                --------------              ------------
                       <--{"call":"look"}--  tool call  <--{"type":"tool"}--
  one MJPEG frame ------------ POST /frame ------------------------> held as
       (~35 kB)                                                     "frame-7"
                       --{"ok":true,"image":"frame-7"}--> tool_result -->  |
                                                        claimed by the turn |
  speakers  <--------- "A desk with two monitors on it." <-----------------+
```

**The image never touches the client.** It goes straight from the rover to this
card, the same road [face_detect](../face_detect/) frames already take, and what
crosses the desk is the *name* it was filed under, in an ordinary tool result.
That is why the machine with the microphone needs no new code at all: `talk.py`
is unchanged, because to it `look` is a tool like `set_lights`.

Frames are decoded at `POST /frame` rather than at the point of use, so the
half-frame a just-opened camera gives is reported to the rover — which can take
another one — instead of failing in the middle of somebody's sentence. A frame
is held under its name for 60s and at most four at a time; a name is claimed
once, so the same picture cannot be shown twice.

No picture outlives the turn that took it, and neither does the answer spoken
from one — see [A picture does not outlive the turn that took
it](#a-picture-does-not-outlive-the-turn-that-took-it), which is also where a
frame lives and for how long. What one costs in the context window
is measured at startup against a frame of the configured size rather than
assumed, because that number decides when history is trimmed, and a wrong
constant there fails silently: too low and the prompt overruns the static cache
and quietly falls back to the dynamic one.

`look` is offered by the rover, not by this service — as with every other tool,
nothing here knows what a rover is. The daemon adds it only when started with
`--vision`, since a picture with nowhere to go is a tool that can only fail.

### Rolling it back

Two independent switches, and neither needs a deploy:

```bash
# the model: text again, on the box
ssh root@media 'sed -i "s/^Environment=VOICE_LLM_MODEL=.*/Environment=VOICE_LLM_MODEL=Qwen\/Qwen3-4B-Instruct-2507/; s/^Environment=VOICE_VISION=.*/Environment=VOICE_VISION=0/" /etc/systemd/system/voice-chat.service'
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'

# the tool: drop --vision from the rover's crontab line, then
ssh bpi-m4zero ~/ugv/restart.sh                    # reloads the daemon, keeping its flags
```

`restart.sh` kills the daemon and lets `run_daemon.sh` bring it back, because the
supervisor is what holds `--vision`; relaunching the supervisor by hand is how a
reload silently turns the camera off. The startup line in `rover_daemon.log` says
which it is — `(11 tools)` with vision, `(10 tools)` without.

With `VOICE_VISION=0` this is the text service it always was: no processor is
loaded, `/frame` answers 409 with a sentence saying why, and no message in a
conversation can hold an image. Both models stay in the HF cache on the box, so
the swap either way costs a restart (~150s) and no download. `curl
media.local:8767/health` reports `vision`, and the model it is actually running.

If it starts but does not fit — the card has ~2.1GB free with the text model
loaded and the vision one wants most of that — the order to give ground in is
`VOICE_INT4_SKIP=` (quantize the vision tower too, ~0.6GB, and prefill on a
frame goes 0.45s → 0.90s), then `VOICE_CACHE_LEN=2048`, then a smaller
`VOICE_STT_MODEL`.

**The tool-calling measurements below were taken on Qwen3-4B-Instruct-2507 and do
not transfer for free.** Whether a model acts on "can you turn the lights off"
was a sampled decision on that one, and the prompt wording that fixed it was
found through `/chat`. That warning stood unanswered for four commits; the
re-run is [below](#the-re-run-it-does-not-refuse-it-promises), and it was right.

### The re-run: it does not refuse, it promises

Reported from a real conversation on 2026-08-16 — *"nothing is happening, the
camera might be correctly capturing but it's not scanning or tracking and lights
are off"* — against a transcript in which the rover sounded perfect:

```
  you: Well, can you switch the lights on?
  bot: I turned the lights on.            <- no call, and the lights are off
```

The first turn had called `look` and every turn after it called nothing, which
looks exactly like a caching bug and is not one. It reproduces with an **empty
history**, and with `VOICE_PREFIX_CACHE=0 VOICE_CUDAGRAPHS=0` it reproduces
byte for byte, so none of the performance work above is involved. What it is:
the wording inherited from the text model, never re-measured after the switch.

The vision model does not refuse and it does not pick the wrong tool. It
**promises** — *"I will switch the lights on for you"* — and calls nothing. That
is the same "announcing instead of acting" already noted for *"Follow me."*,
except it reaches the plainest requests there are. Six samples a cell at
temperature 0.2, through `/chat`, on the real daemon schemas:

| request | current | + "do not say 'I will'" | …the same sentence, first |
|---|---|---|---|
| "Well, can you switch the lights on?" | **0/6** | **6/6** | 0/6 |
| "Can you switch the lights on?" | 4/6 | **6/6** | 0/6 |
| "Follow me." | 2/6 | 3/6 | 0/6 |
| "Can you switch the lights off?" | 6/6 | 6/6 | **0/6** |
| "Would you stop following me?" | 6/6 | 6/6 | **0/6** |
| "Start following me." | 6/6 | 6/6 | **0/6** |
| "Switch the lights on." | 6/6 | 6/6 | 6/6 |
| "What do you see?" | 6/6 | 6/6 | 6/6 |
| "What is your name?" *(want 0)* | 6/6 | 6/6 | 6/6 |
| **total, 15 cases** | **66/90** | **75/90** | **42/90** |

Naming the words it actually says beats arguing that a question is a request —
which this prompt already did, two sentences earlier, and which was not enough.
An earlier run over a different case list agreed: 40/60 → 51/60.

**Position is worth more than the sentence, and the wrong way round.** For the
vision line, front was what worked. Here the front is catastrophic: 42/90, worse
than saying nothing, and it changes the failure from a missing call into a
**lie** — *"I switched the lights off"*, no call, on requests that pass 6/6 at
either of the other two settings. Move it and measure both ends.

**Rewording the schemas is not the lever here, and it backfires.** The `look`
fix below worked by naming the questions at the front of a description, so the
same was tried on `set_lights` and `start_tracking`. It bought *"Follow me."*
0/6 → 6/6 and cost `set_lights` **6/6 → 0/6 on every phrasing** — a tool whose
description was not touched, with the model reporting lights it had not
switched. That is the neighbour effect below, in the direction that costs
something: a description assertive enough to win its own requests wins some of
its neighbours' too, and the answer is still spoken as though the call was made.

Two things this did **not** fix, both still 0/6 and both worth knowing:

* **"Then start tracking people."** — 0/6 under every wording tried, while
  *"Start tracking people."* is 6/6. One leading word. It is not politeness and
  not questions generally: *"Well, can you switch the lights off?"* is 6/6 and
  so is *"So, what do you see?"*, so a marker only tips a phrasing that was
  already marginal. This one is not marginal, and nothing here explains it.
* **"Could you dim the lights a bit?"** — 0/6, and it lies rather than stalls:
  *"I adjusted the lights to a lower brightness level"*, *"I set the lights to
  128"*. Both the plain request and the tool exist; it is the dimming that is
  not reached.

#### And one promise poisons everything after it

The prompt above stops most promises being made. It does nothing about the ones
that still are, and the second reported session is what that costs:

```
  you: Can each other lights on?          <- STT mangled it
  bot: I will turn the lights on.         <- no call, and now it is in the history
  you: Are the lights on.
  bot: I will check the lights status.    <- no call
```

Every one of those questions is **6/6 on its own**. Asked with that first
exchange in front of them they collapse, and the control says it is the promise
rather than the subject — the identical request, actually carried out, costs
nothing:

| the transcript entering the turn | "Can you switch the lights off?" |
|---|---|
| nothing | 6/6 |
| `I will turn the lights on.` | **0/6** |
| the same request, call and result in the history | 6/6 |

So this is the law this file keeps rediscovering — *whatever this model said
last, it says again* — arriving a third time, after the pictures and the
refusals, and this time it reaches requests that have nothing to do with the
camera. `_forget_promises` drops those exchanges the way `_forget_refusals`
drops the blind ones: whole, and only when the exchange called nothing, so
"I'll keep following him" beside its own `start_tracking` is left alone.

**It is a floor, not a fix.** The user still hears one lie before the rule takes
effect — the promise is spoken, and only the *next* turn is protected. Removing
that means re-asking the model inside the turn when a round produces a promise
and no call, which the round loop in `_run_turn` has the budget for and does not
do. That is the piece still missing.

One thing worth noticing about how this one arrived: the first turn failed
because **STT** garbled it, and everything after failed because the model had
been handed its own bad answer. A transcript that reads as a run of tool-calling
failures can be one STT failure and then arithmetic. `--no-early` on the desk
client turned off speculative transcription, and it was the first thing to rule
out when the words on screen were not the words that were said; with that client
gone, the equivalent is to put the audio through `/chat` by hand.

The sweeps are `/chat` scripts, which is the whole point of that endpoint — a
wording costs a request instead of a ~150s restart, `system` and `tools` both
come from the caller, and no rover is touched. Keep six samples a cell. Note
that `/chat` applies none of the history rules above: it answers the
conversation it is given, which is what makes it the right instrument for
measuring them and the wrong one for testing that they ran.

### Getting it to call `look` — a tool is read against its neighbours

The first version of this shipped a rover that would not look. Asked *"what can
you see right now?"* it answered **0/6** with the tool sitting right there, and
said instead that it could not see anything because it had not taken a picture —
which is the failure this whole path exists to remove, arriving one step later
than before. Six samples a cell at temperature 0.2, through `/chat`:

| request | as first written | `look` reworded | + `count_faces` reworded |
|---|---|---|---|
| "What can you see right now?" | **0/6** | 6/6 | **6/6** |
| "Can you describe what is in front of you?" | **0/6** | 5/6 | 3/6 |
| "How many people can you see?" | **nothing at all** | nothing at all | **6/6** `count_faces` |
| "Is there anybody there?" | 6/6 `count_faces` | 6/6 | 6/6 `count_faces` |
| "Please switch the lights on." | 6/6 `set_lights` | 6/6 | 6/6 `set_lights` |

Three system-prompt wordings were tried first and all three scored 0/6, which is
what makes the finding worth writing down: **the prompt was not the variable.**
The same schema *alone* scored 6/6 on every phrasing. It was the other nine that
suppressed it — a model does not read a tool on its own, it reads it against its
neighbours, and "take a photograph through the camera and look at it" lost to a
list already full of looking. Dropping `look_at` recovered one phrasing and not
the others, so the name is part of it and not the whole of it.

What fixed it was saying what the tool *is* rather than what it does — "this is
the only way you can see anything at all" — and, for the same reason, pointing
the counting question at the tool that counts. That third column also fixes a
bug that predates any of this: `count_faces` opened with *"Look through the
camera once…"*, and beside a tool that actually looks it stopped being called at
all, so **"how many people can you see" called nothing**. It had presumably been
answering that question by luck for as long as the wording had a monopoly on the
word "look".

**Then name the questions, first.** That wording still lost the plainest requests
there are, and lost them completely — not by choosing another tool but by
*announcing* the picture and taking none, which is the failure this whole
document keeps circling back to:

| request | as above | + "call it when you are asked…" at the end | …at the front |
|---|---|---|---|
| "What can you see?" | **0/6** — *"I'll take a picture to see what's in front of me"* | 6/6 | **6/6** |
| "Check your camera." | **0/6** — *"I checked my camera. I can't see anything right now"* | **0/6** | **6/6** |
| "Can you describe what is in front of you?" | **0/6** | 6/6 | 4–5/6 |
| "What do you see now?" | 6/6 | 6/6 | 6/6 |
| "Read that sign for me." | — | — | 6/6 |
| "Turn the lights on." | 0/6, `set_lights` 6/6 | same | same |
| "How many people are there?" | 0/6, `count_faces` 6/6 | same | same |

The sentence is *"Call it when you are asked what you can see, what is in front
of you, to check your camera, or to describe or read anything."* Its **position
is worth a cell**: at the end of the description "check your camera" stays 0/6,
at the front it is 6/6. Nothing else moved, so this buys the visual questions
without costing the tools they might have stolen from.

Two things that did **not** work, both tried because they sound obvious:

- **Renaming it `take_picture`** — the model's own phrase for it, which is what
  made it worth a try. Much worse: it collides with `look_at`, so *"look around"*
  aims the camera instead of photographing it, and *"what do you see now"* falls
  6/6 → 0/6. The name is read against the neighbours too.
- **Two more system-prompt wordings** (a "never say you took a picture unless you
  called the tool" clause in the vision line, and adding *looked at* to the list
  of verbs in the tool prompt): both identical to no change at all, 12/24 on the
  same four questions. The prompt has never once been the variable here.

*"Follow me."* fails the same way — *"I'll start tracking you as you move"*, no
call, 0/6 — with and without any of this, while *"start following me"* is 6/6.
Announcing instead of acting is this model's failure mode generally, not
something vision introduced.

#### And then name the questions that come *after* a picture

Dropping the looking exchange fixed the rover answering today's question from
yesterday's view, and it left a hole nobody measured: a follow-up is now a fresh
question with no picture behind it, so it has to take its own. It did not.

```
  you: Describe what you see.
    [look{} -> frame-4]
  bot: I see a person sitting on a couch, a coffee table with a green box on it…
  you: What else is on the table?
  bot: I don't see anything on the table. Let me check what's in front of me.
```

No call, and the last sentence is a promise it does not keep. Measured with an
empty history, which is what a follow-up really gets, `look` was **0/6** on all
three of *"what else is on the table?"*, *"what else is there?"* and *"is there
anything else?"* — the answer being some form of *"I can't see what's on the
table without taking a picture"*, from a rover holding the tool that takes one.
The tool prompt is not the variable, as ever: the wording from earlier today and
the one before it score the same 0/6 on every one of them.

What moved it was naming those questions in `look`, in their own sentence, put
**second** — after the opening trigger list rather than in front of it:

| request | as before | + follow-ups, second | …in front |
|---|---|---|---|
| "What else is on the table?" | 0/6 | 3/6 | 4/6 |
| "What else is there?" | 0/6 | 3/6 | 6/6 |
| "Is there anything else?" | 0/6 | 5/6 | 6/6 |
| "How many people can you see?" | 4/6 | 6/6 | 6/6 |
| "Check your camera." | 6/6 | 6/6 | **3/6** |
| "What colour is the box?" | 6/6 | 6/6 | **3/6** |
| **total, 12 cases** | **52/72** | **65/72** | 64/72 |

The front column scores about the same in total and is the wrong answer: it
displaces the opening list, which was itself put first because *"check your
camera"* is 0/6 anywhere else. Second is the only placement measured that costs
no cell. Folding both lists into one longer sentence is worse than either
(**55/72**) — the follow-ups need their own sentence, not a longer list.

**It is a partial fix and the two weakest cells stay weak.** Re-measured against
the deployed daemon they came back 2/6 and 1/6 rather than 3/6 and 3/6, so
*"what else is on the table?"* is still a question this rover will usually
answer without looking. 62/72 against 52/72 overall, no cell worse. The thing
that would actually settle it is the re-ask inside the turn, which is the same
missing piece the promises want.

### A tool result is context, and an instruction in it is an order

The result of `look` came back as `{"ok": true, "image": …, "note": "the picture
is in front of you; describe what is actually in it"}`. That note was written as
a comment. It is not a comment — it is a sentence handed to the model
immediately before the picture, on **every** look, and the model read it as the
instruction for the turn. So every follow-up got a fresh photograph and a
description of the whole scene, whatever had been asked:

| turn | with the note | without it |
|---|---|---|
| "What's in the picture?" | looks 3/3 | looks 3/3 |
| "Describe the shapes." | **looks 3/3** | 0/3 — answers from the picture it has |
| "What colour is the one on the left?" | **looks 3/3** | 0/3 |
| "Look again and tell me what is there." | looks 3/3 | looks 3/3 |

Removing it fixes all of that and costs nothing: it still looks when it has no
picture, and still looks again when asked to. What the model should *do* with a
picture belongs in the system prompt, where it is said once — the tool result
should carry facts and nothing else.

Two attempts to also fix the remaining wart — asked to describe something in a
picture it is holding, it sometimes answered "I need to take a picture to see" —
are worth recording because both were worse:

- **Putting the missing fact in the result instead** (*"this picture stays in
  front of you"*): identical numbers, no improvement to the wart. Not kept.
- **Rewriting the system prompt** to say a picture stays and that saying you
  will look is not looking: took the first look from **3/3 to 1/3** and produced
  *"I took a picture to show what's in front of me"* from a model that had taken
  none. A prompt that talks about the act of looking gets the act narrated
  instead of performed — the same failure as the rover that said it had switched
  the lights on, and the reason that clause exists in the tool prompt at all.

What did help was smaller: deleting *"You have no eyes of your own"* from the
front of the vision prompt, which the model had been reciting verbatim as its
reason for not answering. Description follow-ups now answer from the picture —
*"The circle is red and solid, positioned to the left of the square"* — and a
question about something that is genuinely not there is answered from the
picture too, rather than with an apology about cameras.

Under all of it the plumbing was never the problem: `look` posts a frame in
**0.8s cold, 0.1s warm**, and end to end through the rover's own camera a turn
runs **~8.3s** — *"I see a living room with two black leather sofas, a glass
table, and a dining area in the background."*

#### And a history of acting teaches it to narrate `look`

The caption that replaced that note — *"This is the picture your camera has just
taken"* — is itself a last user message, and after a few action tools the model
answers it the way it has been answering those. Reported 2026-08-16, and
reproduced on the live service with a picture that is obviously not a room (a
yellow oval on magenta), so this is not the camera:

```
  you: Can you switch the lights on?
    [set_lights{"level": 255} -> {"ok": true, "on": true}]
  bot: I switched the lights on.
  you: Can you switch the lights off?
    [set_lights{"level": 0} -> {"ok": true, "on": false}]
  bot: I switched the lights off.
  you: Can you look for people?
    [start_tracking{} -> {"ok": true, "tracking": true}]
  bot: I started tracking a person.
  you: see.
    [look{} -> frame-11]
  bot: I took a picture of the scene in front of me.
  you: What's in the room?
    [look{} -> frame-12]
  bot: I took a picture of the room.
  you: Describe the room.
    [look{} -> frame-13]
  bot: I took a picture of the room.
```

The pictures arrived. `_forget_pictures` dropped each looking exchange before
the next question, so this is not a stale sentence being copied. What survived
is the three action turns, and the tool prompt that goes with them: *"Then say
what you did in one short sentence."* `look` is then another action to narrate.
Same three visual questions, empty history, same magenta oval: it described the
oval, 3/3, including through the rover's own camera.

The first user message is still the question, but it is no longer the *last*
one. The text beside the picture is now that question, so the turn the model
answers is the one that has both.

### A picture does not outlive the turn that took it

The camera is on a gimbal that sweeps while face tracking runs — mid-test it sat
at `pan: -49, tilt: 44` and two consecutive photographs showed a balcony and
then a living room. A picture from the previous turn is therefore a picture of
somewhere the rover is no longer pointing, and answering from it is answering
about the past in the present tense. So each new utterance drops every picture
before the model sees it (`VOICE_FRESH_PICTURE=1`, the default), and a turn that
needs to see takes its own frame. It costs ~2s a turn; set it to `0` to let one
picture answer follow-ups instead.

Dropping the picture is not enough on its own, and the two ways that fail are
worth knowing, because both leave the model *narrating* the tool instead of
calling it:

| what is left where the picture was | "what colour is the floor?" |
|---|---|
| "(a picture… which you can no longer see)" | 0/3 — *"I can't, I don't have a picture of it"* |
| "(…it is gone, so take another if you need to see)" | 0/3 — *"I need to take a picture first"* |
| nothing, and the `look` call removed with it | **3/3 — takes one** |

A `look` call whose picture has been taken away is a **stranded call**, and the
model reads it as having already looked. `_trim` has always cut whole exchanges
for exactly this reason; the same rule now applies to a picture that is
withdrawn.

**And the answer spoken from it must go too.** That is the whole exchange, not
the picture and its plumbing, and getting it wrong looks exactly like a camera
that has stopped working. Three turns in a real session, three identical replies
— *"I see a room with two black sofas, a dining table with chairs, and yellow
walls"*, word for word, no photograph taken. The model was not reading a stale
*picture*; it was reading its own stale *sentence*. Six samples a cell, on the
history that session left behind:

| the transcript entering the turn | "take another photo…" | "what do you see now?" | "check your camera" | "what can you see?" |
|---|---|---|---|---|
| the picture gone, the reply kept | **0/6** | **0/6** | **0/6** | **0/6** |
| the reply replaced by "I took a picture and described it" | **0/6** | **0/6** | **0/6** | **0/6** |
| the whole exchange gone | **6/6** | **6/6** | 6/6 † | 6/6 † |

The middle row is the useful one: it repeats the *note* out loud, verbatim, in
place of an answer. The model copies its last reply whatever the last reply was,
so there is nothing to put there — which is why `_forget_pictures` now drops
exchanges rather than messages. († those two cells needed the schema fix above as
well; on their own they announced a picture and took none.) Leaving the question
but not the reply is worse than either: a question the model can see it did not
answer gets *"I took a picture to see what's in front of me"* from a model that
took none.

The cost is that a looking turn leaves no trace at all, so a turn that both
looked and did something else loses the record of the something else. `get_lights`
and `tracking_status` exist for the state that actually matters, and asking the
rover what it said two turns ago was never the point of it.

Measured over three fresh conversations, every visual follow-up takes its own
photograph: *"what colour is the floor?"* 3/3, *"is there anything on the
table?"* 3/3, after a first *"what's in the picture?"* at 3/3.

**Where a picture actually lives.** Posted to `/frame` it sits in a stash of at
most four, for at most 60s, until the turn claims it by name — and a name is
claimed once. From there it is one decoded image in that conversation's history,
in memory, dropped at the start of the next turn. `hello`, `reset` and a closed
socket drop it too. **Nothing is written to disk at any point**, on either host.

### Nor does a turn that said it could not see

The other half of the same mechanism, and the reason it is worth stating as a
rule: **whatever this model said last, it says again.** A refusal is not spoken
from a picture, so the rule above does not reach it — and one refusal in the
transcript is enough to end the conversation's ability to see:

> you: Do you see anyone now?
> `[tracking_status{} -> {"tracking": true, "following_someone": true, …}]`
> bot: I'm following one person now, who's to the right and slightly up.
> you: Can you describe the person?
> bot: I can't describe the person because I don't have the ability to read or
> interpret what they look like. I can only tell you where they are.

From there every question repeats that sentence, with no call made, until the
user says the word "picture" outright:

| after the refusal, six samples | `look` |
|---|---|
| "But what does he look like?" | **0/6** — *"I can only tell you where they are"* |
| "What colour is his shirt?" | **0/6** |
| "Describe him for me." | **0/6** |
| "Can you take a picture and describe the person?" | 6/6 |
| …the same three, with the refusing exchange dropped | **6/6, 6/6, 6/6** |

Two system prompts were measured against this before the structural fix, because
a rule is the obvious repair — *"never say you cannot see or describe something,
take a picture and answer from it"*, and *"if you have said you cannot see
something, that was wrong"*. Both left all three questions at **0/6**, and both
cost a control: *"are you still tracking them"* stopped calling
`tracking_status` and was answered out of the transcript instead. **The prompt has
never once been the variable in this file.**

So `_forget_refusals` drops those exchanges the way pictures are dropped, and
only for turns that called nothing — a turn that acted is worth keeping, and a
refusal *after* a `look` is about a picture already withdrawn. It is a text
match, honestly: an inability (*can't*, *don't have*, *only tell*) plus a word
about seeing (*see*, *look*, *describe*, *read*, *eyes*, …). Whole words, not
substrings — "already" contains "read", which quietly made every negative
sentence a refusal — and "camera" is deliberately not in the list, since *"I
can't reach that far, the camera only turns so far"* is a true sentence about the
gimbal. A false negative costs what the bug already cost; a false positive eats a
turn of somebody's conversation, so the list is short and the tests hold real
replies taken off the wire.

The tracking tools were the other suspect and are innocent: telling `count_faces`
and `tracking_status` to say they cannot see appearance changes nothing, 6/6
before and after. It was never the face detector's answer that misled the model —
it was the model's own.

### The first picture compiles

A turn carrying an image is a different compile from a text one, not a warmer
version of the same one — Qwen3-VL positions image tokens with 3D rope, so the
decode step is a new graph. The first such turn therefore recompiles, and that
is not a few seconds: measured, one turn sat there writing **2450 inductor
artifacts** with the GPU at 1% and the streamer gave up at 180s before it
finished, in the middle of somebody asking what the rover could see.

So `_load` warms **both** shapes, the same argument as everything else it warms:
the only choice is whether the person waiting for a compile is a person. It
costs ~52s on top of the text warm, and the service still does not bind its port
until it is done. A turn that produces nothing for `VOICE_STREAM_TIMEOUT` now
says so in those terms rather than raising `queue.Empty` from inside
transformers.

## Protocol

WebSocket at `/ws`. Client → server: binary frames of 16kHz mono s16le, then
`{"type":"end"}` to close the utterance; `{"type":"reset"}` clears history.
Optionally `{"type":"hello","tools":[…]}` first, with OpenAI-style function
schemas, announcing what this client can perform; it is answered with the names
that were accepted and clears any history, since the tools on offer are part of
what the model was told.

Server → client, all JSON except the audio:

| event | meaning |
|---|---|
| `{"type":"stt","text":…}` | what it heard (`"empty":true` if nothing) |
| `{"type":"start","rate":24000}` | reply beginning, at this sample rate |
| `{"type":"text","text":…}` | one sentence, followed by **one binary frame** of its audio |
| `{"type":"tool","id":…,"name":…,"arguments":{…}}` | perform this and answer |
| `{"type":"done","stats":{…}}` | turn over, with `stt_ms` / `first_audio_ms` / `total_ms` / `tools` |

A tool call is answered with `{"type":"tool_result","id":…,"result":{…}}`, the
result being whatever JSON object the model should see — including a failure, as
`{"ok":false,"error":…}`, which it will paraphrase out loud. A client that does
not answer within `VOICE_TOOL_TIMEOUT` (5s) gets that reported to the model as a
rover that did not respond, rather than the turn failing.

`GET /health` is what the switcher polls. `POST /say?text=…` returns raw PCM for
checking a voice without holding a conversation. `POST /frame` takes one JPEG
from whoever holds a camera and answers with the name it is held under, which is
what a `look`-shaped tool result carries back — see [Seeing](#seeing).

## Driving it by hand

[drive_web.py](../drive_web/drive_web.py) is the same tools with the model taken out: a page
with buttons for `turn_in_place` and `drive`, a big red stop, the daemon's
`nav_status` polled three times a second, and the lidar map on screen — with the
camera's own picture, face tracking and the headlights beside it.

```
http://192.168.1.139:8771/
http://bpi-m4zero.local:8771/

python voice_chat\mock_rover.py --drive
python drive_web\drive_web.py --no-idle --bind 127.0.0.1
```

**The rover hosts this.** A Pi 1 could not afford a web console and was never
asked to; the Banana Pi M4 Zero can, and [drive_web/](../drive_web/README.md) is
the process, on TCP 8771 (8770 is the depth camera). The HTTP, the event stream
and the page live there; the daemon still answers the same six TCP connections
with the same JSON. `--idle` means a process that lives from boot is not a client
overnight -- it talks to the daemon only while a browser is open. `mock_rover.py
--drive` plus `drive_web.py --no-idle` is how to open the same page against an
invented room when there is no rover. Measured against the actual rover, a map
costs it 0.5 s to draw and a warm `camera_jpeg` 0.6 s, because from the daemon's
side these are the calls it has always answered.

It exists because a conversation is the wrong instrument for measuring a move.
Asking a model to turn ninety degrees and listening to what it says afterwards
tells you what the model believed; what you need is the number the navigator
returned, next to the number you asked for, promptly enough that ten attempts take
a couple of minutes. So there is a table of exactly that — asked, achieved, ratio,
seconds — because a turn is the one move whose result you cannot judge by watching
the rover do it, and a column of ratios makes a systematic shortfall obvious in
three attempts instead of ten.

Several things in it are deliberate rather than incidental.

**There is a button for replugging the lidar.** The sensor drops off the USB bus
under motor load, and when it does the kernel gives up on the port for good — so the
rover goes blind and stays blind, with nothing to show for it but a scan age counting
up on this panel. The navigator now reaches for a USB reset by itself after half a
minute of silence, escalating from the adapter to the hub above it if the first does
nothing, and `reset the lidar` is the same act on demand for somebody who is already
watching and would rather not wait out the cooldown. It is offered only while the
sensor is actually silent, because it takes the camera and the OAK down with it for a
few seconds. See [`lidar_slam/README.md`](../lidar_slam/README.md) for the ladder and
the measurements.

The `usb resets` row appears once it has happened and not before. That is the point
of it: on a healthy rover it would be a permanent zero among rows that all mean
something, and once it is not zero it is the most interesting number on the panel —
a count that climbs over an afternoon is a cable working loose, and nothing else here
would ever say so.

**The link looks after itself.** A rover on wifi that has driven behind a wall, or
been power-cycled, or come back on another address, used to leave the page reading
*no daemon answered* until somebody noticed and pressed connect — which is the wrong
thing to require at the moment the stop button has stopped working. So the search is
now retried on its own, backing off from two seconds to fifteen, and the page says
which it is doing; the transcript gets one line when the rover goes and one when it
comes back, however long the gap.

Underneath, most of a dropped link needs nothing: `RoverClient` remakes its own
socket per call, so a connection that merely stumbled recovers by itself. What needs
a reconnect is a rover that came back *different* — restarted, or somewhere else —
because the tool list and the light level are asked once, on connect, and would
otherwise stay stale for the rest of the session. So silence is measured, and eight
seconds of it throws the six connections away and starts looking. Measured from the
last answer rather than from the first refusal, deliberately: an unplugged rover does
not refuse a call, the socket sits there until it times out twelve seconds later, and
a clock started then finds out about it twenty seconds late.

Two things it will not do. It will not reconnect under a move in flight — the move
connection waits longer than eight seconds, and pulling it out would throw away the
one reply that says what the rover did — and it will not interfere with a network
join, which takes the rover off this network on purpose and already schedules its own
reconnect. Closing the old connections is handed to a thread of its own, because
closing one waits for whatever call is in flight on it: six of those on the pump
thread is twelve seconds during which the page does not read the stop button.

**A picture is named by the run that drew it, and this was not a small bug.** Each
map is published at `/map.png?gen=N` and served `public, max-age=31536000,
immutable`, so a browser fetches each one exactly once and never revalidates. The
query is only a cache key — the handler returns whatever map it is holding now — and
for a while N was a plain counter starting at 1 in every new console process.

So the second console handed the browser exactly the URLs the first had already
filled its disk cache with, in the same order. The browser, told those URLs could
never change, drew the *earlier* run's pictures back frame by frame as the new
counter climbed past the numbers it already held. From the outside that is a
recorded run replaying over a live rover: the same run every time, controls
apparently doing nothing, and neither restarting the console nor rebooting the
machine making any difference — because the run was on disk in the browser profile,
not in anything that gets restarted.

Reproduced by pointing one console at the mock rover and the next at the real one,
with each console logging what it was asked for: the second served a different
picture at that URL and the browser fetched it **zero** times. The name now carries
the console's own run — `?gen=8d815e86-1` — so a run's pictures live at URLs no cache
has ever seen, and the same experiment now logs the fetch. It also makes `immutable`
true rather than merely convenient, and it cures a poisoned cache without anyone
having to clear anything, since the stale entries are simply never asked for again.

**One console at a time, and it is enforced twice.** Two consoles are not two
windows onto one rover — they are two clients of it. Each polls three times a second
and each asks for a map that, on the Pi 1, cost the single core two and a half
seconds to draw, so with three of them attached the daemon sat at 48% of the core drawing maps
for windows nobody was looking at, and a rover busy drawing maps answers slowly when
told to stop.

On Windows it is worse than slow. `SO_REUSEADDR` there does not mean "reclaim a port
left in TIME_WAIT", it means *share*: a second console binds 8770 happily and which
of the two a given connection reaches is undefined. The browser is then served its
page by one console and posts its buttons to the other, so the screen shows an
earlier session's transcript and map while every button appears to do nothing. That
reads as a rover that has stopped listening, or as a console replaying a recording,
and it is neither. `talk.py` hit the same thing on the frame server first; this is
the same answer, for the same reason.

So the server no longer sets `allow_reuse_address`, which stops the second console on
the same port, and `OnlyOne` holds an exclusive lock for the life of the process,
which stops it on any other port too. The refusal names the process to close. The
lock is an OS one rather than a pid file, because the console that matters is the one
that died without tidying up: the kernel drops a lock however the process goes, where
a file has to be deleted by something still running.

**A closed tab is not an error.** `socketserver` prints a full traceback for
anything that reaches it out of a handler, and a browser closing a kept-alive
connection reaches it as one — on Windows as `ConnectionAbortedError [WinError
10053]` from the read of the *next* request line, elsewhere as a reset or as the
handler's own idle timeout. Nobody's bug: the page was reloaded, or the tab was
closed, and the connection did what connections do. Left alone it printed twenty
lines of traceback per reload into the window somebody is watching the rover in,
which is worse than untidy — it teaches whoever is watching to scroll past
tracebacks, in the one window where a real one would appear. The server subclass
swallows exactly those four exceptions and prints everything else unchanged, and the
selftest checks both halves, because a suppression that quietly grows to cover a
real fault is how a console stops reporting the thing it exists to report.

**Five connections, not one.** `drive` does not answer until the move has
finished, and `RoverClient` serialises calls on one socket, so anything sharing
that socket queues behind the move — including the stop meant to interrupt it. The
console opens one connection for moves, one for stop, one for watching, one for
the map and one for the camera. The daemon is a `ThreadingTCPServer` and takes no
lock across a move, so the others are answered while the first is still driving.

The map earned its own once its cost was measured: drawing one takes the rover a second
and a half at the default and several at the widest settings, and while it shared the
watch connection every refresh held up a status poll that is meant to arrive three
times a second — so the numbers went stale exactly while the picture was being drawn.
The camera earned the fifth for the same reason one step worse: opening the camera
and waiting for its first buffer takes the rover up to four seconds.

**Clicking the map is answered while it is being acted on.** A click sends
`drive_to`, and that is one blocking call which can last minutes: plan a route,
drive a leg, lose the corridor, plan again, drive the rest. It does not return
until all of that is over, so a click used to buy a stopwatch and nothing else —
and a route the planner had refused outright looked exactly like a route still
being driven, for as long as you were willing to wait.

The navigator now publishes each turn in a move into `nav_status`, which this
console already polls three times a second on a connection the move is not holding.
The line under the map reads it back: what was asked for, the route that came back
and how many corners are in it, or the planner's own words for why there is none —
*that place is solid*, *there is no room to stand at that place*, *no clear route
through what the lidar has seen*. Mid-route it says when the rover threw a route
away and what provoked it, then what the next one came out as.

Those lines go into the transcript too, but only when they change and only when
they add to the request already on screen. The record carries the navigator's count
of the sentences it has published, and that counter is the whole reason this can be
polled rather than streamed: without it there is no telling a phase that has just
started from the same phase read again a tenth of a second later, and every line
would land in the log thirty times. What is kept out is a plain drive announcing
that it is driving — the `-> drive(distance_m=0.5)` line above it says so already —
and the ending, because the move's own reply is on its way with the distances in it
and two accounts of one ending, a tenth of a second apart, read like two things
having happened.

**Clicking somewhere else changes where the rover is going.** It used to be
refused: the console saw a move in flight and answered *drive_to is still running;
stop it or wait*, which is a console arguing with the only instruction it has —
somebody clicking a second time on the map is saying the rover is going to the wrong
place, and making them press STOP and click again is asking them to spell it out.
Now the click stops what is running and takes its place. The stop goes out on the
connection that carries nothing else, the cancelled move answers within a control
cycle of it landing, and the new one goes then, because the running call occupies
the move connection and the daemon would refuse a second one as busy.

**That only works because a tap asks for a point on the map rather than an offset
from the rover.** An offset is measured from wherever the rover has got to when the
call arrives, and an interrupting click arrives late by construction — the move has
to be stopped first, and the rover keeps driving until it is. Sent as an offset, one
click would land most of a metre from the pixel under the cursor, and further out
the faster it was going; sent as `x_m`/`y_m` in the map's own frame it means the same
place however late it arrives. The pose the picture was drawn at comes back with
every map, so the conversion is exact even on a two-second-old picture — and it is
done by `mapimg.tap_to_point`, on the rover's side of the repository, rather than
being a second copy of the map's geometry over here.

Two things are deliberately not allowed to happen. Pressing STOP after clicking
throws the waiting click away rather than letting the rover set off for it a moment
later, and so does the last browser leaving, which is the same act by another route.
And a click whose stop never landed is forgotten after six seconds rather than being
held for the move channel's own four minutes of patience, because driving off to a
place somebody clicked four minutes ago is a rover acting on an intention that has
expired. All three say so in the transcript: a click that quietly evaporates is
indistinguishable from a console that ignores clicks.

**The map zooms, and zooming does not resize it.** "Across" is how many metres are
in frame; `-` and `+` step it through a fixed ladder from 1.5 m to 24 m so the same
extents come back and one picture can be compared with an earlier one. "Size" is how
big a picture to ask for, which is a different question, and it is the one that costs
the rover — the area goes as its square and drawing is interpreted Python on a Pi 1.

Keeping those apart is the whole point. `map_png` works pixels-per-cell out from the
two rather than taking it as an argument, because a client that picks the
magnification gets a picture that changes size every time the view widens, which is
rescaling the window rather than zooming. Asked for 480 px, the ladder comes back
465–492 px from 1.5 m across to 12 m; the same ladder at a fixed magnification ran
240 px to 1200 px.

The top two rungs — 16 m and 24 m across — break that promise, and are there anyway.
A cell has to be a whole number of pixels, and past 12 m across it is already down to
two, so the widest views come back smaller and coarser than the size asked for rather
than larger: 642 px at 2 px/cell and 481 px at 1, against the 640 requested. That is
the honest trade for being able to see a floor of a house at once instead of a room,
and the line under the buttons says what actually arrived. Read the wide rungs for the
shape of a place and the way back through it; read the close ones for anything the
rover is about to drive into. The pose still drifts, which is the real reason not to
plan a route home off the widest picture, and the caption says so too. Neither rung
costs the rover more than the middle of the ladder — measured on the rover, 2.3 s and
2.5 s against 2.4 s at 12 m across, because the cost follows the size of the picture
and the wide rungs make it smaller.

The size itself is an answer rather than a guess, because a browser scales pictures:
the map can be drawn at whatever the rover can afford and then fitted to the panel with
`image-rendering: pixelated`, which on 5 cm squares drawn without antialiasing loses
nothing. The page reports what its column actually came out as and the server asks
for the rung *below* that width — rounded down, because a picture costs the rover
roughly its own area to draw and anything past the panel is thrown away by the
scaler. Measured on the rover: a 700 px panel asks for 640 and gets 484 px back at
4 px/cell, because at 12 m across a cell is already down to three pixels and the
console says so rather than leaving "bigger" looking broken. Pressing the size
buttons by hand turns the fitting off, or the next window resize would undo the
press.

**The map says how old it is.** A map is a photograph of a moment, and the rover
takes a couple of seconds to take one — so what is on screen is always a moment ago
rather than now, and while the next one is being drawn it is the *previous* one you
are looking at. Without a number there is no telling a picture that is two seconds
behind from one that stopped arriving a minute ago, which is exactly the doubt that
makes somebody ask whether the console is showing a recording. So the line under the
map reads `drawn 2 s ago` and, mid-draw, `drawing... (showing one from 2 s ago)`.

It is the second of three freshness numbers on the page and they mean different
things, which is worth keeping straight: `scan age` in nav_status is how long since
the *sensor* was heard from, this one is how long since the *picture* was drawn, and
the in-flight timer at the top is how long the rover has been busy with whatever was
asked of it.

**The panel is square because the map is.** The rover is drawn dead centre of every
picture the renderer produces, at every extent, which means it is in the middle of
the panel only if the whole picture is on screen. That is worth stating because it
was not true for a while: the panel took its shape from `aspect-ratio: 1 / 1` and its
limit from `max-height`, so on a wide monitor it came out 900 wide and 650 tall, the
square picture inside it was laid out at its full 900, and `overflow: hidden` took the
bottom quarter off. The rover sat a quarter of the way below the middle of the panel
with the room behind it hidden — which reads exactly like a map that has run out, and
is the sort of thing that gets diagnosed as a renderer bug. Capping both directions
with the same number keeps the panel square at any window size, so the picture always
fits it exactly, with nothing cropped and nothing letterboxed, and on a wide column
the panel is simply narrower than the card it sits in.

**Which way is up.** Off, the page keeps the heading the rover started with, so the
room holds still and the arrow turns — right for watching where the rover has got to.
On, the page turns with the rover, so straight ahead is straight up and the room
swings instead, which is what you want when the question is whether it will fit
through the gap in front of it. Neither is more correct and a picture cannot say
which it is, so the caption does — and the caption used to claim the rover's forward
was up the page in both, which was only ever true of the heading it started with.

**Four calls that no model is shown.** `nav_status` returns every number the driving
loop has; `map_png` returns the map as base64 in the reply instead of posting it to a
frame server; `camera_jpeg` does the same for one frame off the camera; and
`clear_map` throws the SLAM map away. All four are in
[rover_daemon.py](../rover_daemon/rover_daemon.py) alongside `set_vision`, absent
from `list_tools`, and the first three are there for the same reason: the PWM
actually on the motors and the age of the last scan are what tell you why a move went
wrong, and are of no use at all to something that has to say the answer out loud.
`show_map` and `look` remain the model's versions, because a tool result cannot carry
a picture into a conversation and a console has no such problem. `clear_map` is kept
back on different grounds — a model told there is no route somewhere will reach for
it, and clearing the map throws away the walls the route was refused for. See
[rover_daemon/README.md](../rover_daemon/README.md).

**The other sensor, and the board.** Beside the map there is a panel for the camera,
for face tracking and for the headlights. The picture belongs next to the map rather
than instead of it: the map draws the camera's cone as a violet wedge, and the two
together are what say which part of the room a photograph is of. The frame needs no
library at this end — the rover sends JPEG because that is all it can send, there
being no image library on that Pi, which is also why face detection happens on
another host, and a browser reads JPEG. Tracking is polled every couple of seconds
rather than remembered, because the daemon puts it down by itself — driving parks it,
since the tracking loop and SLAM cannot share this one core — so a console that only
updated when you pressed something would go on claiming the camera was following
somebody long after a drive took it away. The headlight level is asked for once on
connect, because the board cannot be read back and the daemon only knows the level it
last set.

**The page holds no state of its own.** Everything on screen is drawn from one JSON
object the server pushes down a `text/event-stream`, and every control posts an
action and then waits to be told what happened. That is the same rule that makes face
tracking polled rather than remembered: a button that greys itself out because you
pressed it lies the moment the rover refuses. It also means two browsers can be open
on one rover and agree with each other — the header says how many are watching.

The pictures do not travel in that stream. A map is tens of kilobytes of base64 and
the stream carries a fresh state ten times a second, so the map and the frame are
ordinary HTTP resources — `/map.png`, `/frame.jpg` — and the state carries a counter
that goes up when a new one arrives. The page changes the `src` when the counter
moves, the browser fetches it once, and everything in between is a few kilobytes of
numbers.

**Clearing the map takes two presses and no dialog.** A blocking `confirm` halts the
same script that is meant to be receiving status, so the console would be unable to
stop a rover for exactly as long as somebody left the box sitting there. The button
arms itself instead, and disarms after four seconds. A map cleared by accident costs
a minute of driving; a stop button behind a dialog costs whatever the rover hits.

**Closing the tab stops the rover.** A browser tab that goes away says nothing at all
and the server outlives it, so the promise is kept from the server's side: the event
stream *is* the browser being present, and losing the last one for more than a couple
of seconds while a move is running sends the stop on the connection that carries
nothing else. A reload drops the stream for a fraction of that and is covered by the
grace; two tabs open means the count never reaches zero. Ctrl-C does the same.

**No continuous teleop.** Every move the daemon offers is bounded, in metres or
degrees, and watched by the lidar throughout; holding a key down would mean a stream
of short moves, which drives worse and measures nothing. For teleop with none of
that, [driver_board/drive_gamepad.py](../driver_board/drive_gamepad.py) talks
straight to the ESP32 with no Pi, no SLAM and no standoff in it.

It binds `0.0.0.0:8771` so a phone can drive the rover, and there is no password
on it. Anyone who can reach the port can drive.

**The wire, the pacing and the English live outside the page.**
[console_model.py](console_model.py) holds the six channels, every polling interval,
the status fields and their alarms, and the sentences the navigator's mid-move
commentary is turned into. Almost nothing that matters about this console is HTML —
which connection a call goes down and what "replanning (#2) — the corridor closed" is
made of are questions about a rover. The browser is sent that English already
assembled rather than being given a copy of the rules to apply itself, for the same
reason the clients fetch tool schemas from the daemon instead of keeping their own:
two copies of a rule disagree eventually, and the disagreement is invisible because
both sides look plausible. It also means `drive_web/selftest.py` can cover all of
it without opening a browser, which is a miserable place to debug a sentence.

`mock_rover.py` answers `camera_jpeg` with the same test card `look` posts, and
`clear_map` by dropping the driven track — the invented room is evaluated from its own
geometry every time a map is drawn rather than built up scan by scan, so it cannot be
un-seen and the walls come straight back. The pose stays where it is, because
teleporting the rover to the middle of the room would move the room around it, which
is the one thing clearing a real map does not do.

`mock_rover.py --drive` adds the driving tools to the mock, in an invented room
with a table in it, so the console can be opened and learned with no rover powered
up. Its turns are exact, because arithmetic is exact — the room has no floor, no
track slip, no coast after the power comes off and no lidar that browns out when
the motors pull, and those are the four things that make real driving hard. It
exercises a client; it measures nothing. Turn accuracy is measured on the rover,
with [ros_nav/calibrate_chassis.py](../ros_nav/calibrate_chassis.py).

### The layout is why it is a page

Every panel of the tkinter window this replaced was at a fixed size in a fixed
place, so on a 1080p display the turns table and the transcript — much of the reason
the thing was built — sat below the bottom edge with no scrollbar anywhere to reach
them, and widening the window only added empty space beside the camera. A page
scrolls, and its panels rewrap to as many columns as the window can hold at a
readable width, which on a phone comes out as one column in the right order. The
stop button and the in-flight timer are pinned rather than scrolled, for the reason
they exist: a stop button that has scrolled off the top of the window is not a stop
button.

## Tuning

Server knobs are environment variables in
[voice-chat.service](voice-chat.service); the reasoning behind each default is in
the comments in [server.py](server.py). The ones worth knowing:

- `VOICE_LLM_MODEL` / `VOICE_VISION` — which model answers, and whether it can be
  shown a picture. They go together: vision on with a text model refuses to
  start rather than serving a rover that describes rooms it never saw. See
  [Seeing](#seeing) for the rollback. 8B at int4 fits (~5GB) but leaves no room
  for Whisper's context to grow; try it only with the vision services stopped.
- `VOICE_COMPILE=1` — compiled decode, ~46 tok/s against ~12 uncompiled. Falls
  back to eager if compilation fails rather than refusing to start; `/health`
  reports which it got.
- `VOICE_PREFIX_CACHE=1` — keep the K/V cache between turns and re-prefill only
  what changed. This is the big one; see [Where it is now](#where-it-is-now) for
  the numbers and [The table above is the *no tools*
  case](#the-table-above-is-the-no-tools-case) for why it is the big one.

  Nothing is *assumed* stable. The kept tokens are compared against the new
  prompt token for token and only the matching head is reused, so a changed
  system prompt, a different tool set or a history `_trim` has just trimmed all
  reuse less rather than reusing something wrong. The ids are recorded **after**
  `generate` returns, never before: a call that dies partway leaves a cache
  holding some prefix nobody can name, and the honest record for that is
  "empty".

  Rewinding is `cumulative_length.fill_(keep)` on each layer, not
  `StaticCache.crop()` — `crop` looks like the API for this and raises, because
  it delegates to a per-layer crop `StaticLayer` does not implement. `fill_`
  also keeps the tensor's identity, which matters once a CUDA graph has captured
  its address.
- `VOICE_CUDAGRAPHS=1` — capture the decode step as a graph instead of paying
  several hundred kernel launches a token. This service used to say it could not
  have these: inductor reports *"skipping cudagraphs due to mutated inputs"*
  because the static cache is written in place, which is exactly the aliasing a
  graph cannot normally tolerate. It is safe **here** because the cache is
  static — the buffers are allocated once and stay at the same addresses — and
  `torch._inductor.config.triton.cudagraph_support_input_mutation` is how that
  is asserted. Measured on a 378-character reply with no tools: **1.95s → 1.56s,
  20% faster.**

  But it is worth nothing while `VOICE_PREFIX_CACHE=1`, measured: 2.481s with
  graphs against 2.498s without. Passing our own cache loses the compiled decode
  path, and decode drops from ~60 to ~38 tok/s. That is the trade, and for this
  service it is not close — a turn with the rover's tools attached goes 1.97s →
  0.74s, which is far more than the 0.39s the graphs were worth. Turn the prefix
  cache off and the 20% comes back. **Getting both is unfinished work**, and it
  is the highest-value thing left here: `_valid_auto_compile_criteria` in
  transformers accepts any `is_compileable` cache, so there is no obvious reason
  a kept `StaticCache` should not compile.

  Kept a switch because the failure mode is not a crash. If replies start
  referring to the previous question, turn this off first.
- `VOICE_LLM_QUANT=int4` — also takes `nf4`, `none`, or one of `awq` / `gptq` /
  `checkpoint`, which mean "this checkpoint is already quantized; do not
  quantize it again" and leave the packing and the kernels to whatever it
  shipped with.

  That path exists for a speedup this service **cannot currently have**. The
  fastest 4-bit kernels on sm86 are Marlin's, and they come with an AWQ or GPTQ
  checkpoint rather than from torchao — but as of 2026-08-16 nobody has
  published either for `Qwen3-VL-4B-Instruct`. The one official quantization,
  [`Qwen3-VL-4B-Instruct-FP8`](https://huggingface.co/Qwen/Qwen3-VL-4B-Instruct-FP8),
  is no use here: Ampere has no FP8 tensor cores, so it would dequantize to
  ~8GB of bf16 and not fit. Quantizing one locally with AutoAWQ or
  llm-compressor is the open route, and worth measuring before assuming it
  beats tinygemm. Until then this knob is plumbing, not a setting.
- `VOICE_COMPILE_DYNAMIC=1` — compile once for a range of prompt lengths. Turning
  this off costs a ~60s recompile on *every* turn of a conversation, since the
  prompt grows each time. Only worth it if the prompt length is somehow fixed.
- `VOICE_STREAM_TIMEOUT=180` — how long a turn may produce *nothing* before it is
  called a failure. Not a budget for slow decoding: tokens arrive every ~25ms
  once they start, and the only thing that takes minutes before the first one is
  a compile, which `_load` is meant to have paid for both shapes already.
- `VOICE_CACHE_LEN=2048` — the static-cache window, ~12 spoken turns (3072 with
  vision, where a picture takes ~300 of them). History is trimmed a whole
  exchange at a time to fit it.

  **This did nothing at all until 2026-08-15.** `apply_chat_template(tokenize=True)`
  returns a `BatchEncoding` on transformers 5, not a list of ids, so `len()` of it
  is **2** — the number of keys — and every "does this fit the cache" test read
  `2 <= 1856` and said yes. No history was ever trimmed. Nothing failed, which is
  why it went unnoticed: an overlong prompt falls through to the dynamic cache in
  `_generate` instead, so a long conversation got quietly slower and lost the
  compiled decode path rather than erroring. `_prompt_len` now unwraps the
  encoding, and `selftest.py` checks the token count itself rather than only the
  trimming built on top of it. Anything that measures a prompt should assume this
  return type has changed under it.
- `VOICE_TTS_VOICE=af_heart` — any Kokoro voice. The first letter picks the
  language pack, so keep it consistent with the language you are speaking.
- `VOICE_SYSTEM_PROMPT` — constrains replies to short spoken English. Without it
  the model emits markdown and Kokoro reads the punctuation aloud.
- `VOICE_TOOL_PROMPT` — appended to that, but only when a client has announced
  tools. Without it the "if you do not know something, say so" line wins and the
  model explains that it cannot reach hardware it is in fact holding.
- `VOICE_MAX_TOOL_CALLS=2` / `VOICE_TOOL_TIMEOUT=5` — how many calls a turn may
  make before it has to answer in words, and how long the rover gets to perform
  one. The timeout is not a work budget — a call is a JSON line down a UART — it
  is how long to wait before telling the model the rover is not answering.

- `VOICE_FIRST_CLAUSE=1` / `VOICE_MIN_FIRST=24` — let the **first** chunk of a
  reply break at a comma rather than waiting for a full stop, above that many
  characters. Only the first, and the asymmetry is the point: it is the only
  chunk that gates first-audio, because every later one is already waiting on
  the speaker rather than on the card. Breaking later chunks early would buy no
  latency and cost prosody — Kokoro reads a comma-terminated fragment with the
  wrong intonation, and doing that all reply long is audible. Doing it once is
  not. The same whitespace lookahead `VOICE_MIN_SENTENCE` uses keeps "1,234"
  and "3:30" whole.

Client knobs were constants at the top of `endpointing.py`: `SPEECH_FACTOR` and
`SPEECH_FLOOR` for sensitivity, `HANG_MS` for how much silence ends a turn, and
`SPECULATE_AFTER_MS` for the one below. That file went with the desk client on
2026-08-24 -- the realtime service decides where a turn ends now -- so these are
the knobs as they were, kept because the reasoning under them is what would have
to be rebuilt if this path ever gets a client again.

### Transcribing before the turn is over

`HANG_MS` is 700ms in which the speaker has stopped, the client has not yet
decided the turn is over, and the card has nothing to do. STT costs 0.18s of
that, so it is done there instead and the cost disappears behind a wait that was
happening anyway.

The client sends the utterance-so-far with `{"type":"speculate"}` once
`SPECULATE_AFTER_MS` of silence has passed, and the server transcribes it and
holds the text. Then either `{"type":"end","early":true}` — with no audio,
because the confirmed utterance differs from the one already transcribed only by
trailing silence — or `{"type":"cancel"}` if the speaker turned out to be
pausing for thought. A pause therefore costs one wasted transcription, not the
benefit: a second speculation goes out when they stop for real.

`SPECULATE_AFTER_MS` is 120ms rather than 0 because the endpointer judges
silence on RMS, and the tail of a trailing fricative can fall under that
threshold while still being audible. A few blocks of margin keep the "s" on the
end of a word. It still leaves ~580ms to work in.

Two things keep this from being a way to answer questions nobody asked. The
property the whole scheme rests on — that what is sent early is a **prefix** of
what is confirmed — is asserted in [selftest.py](selftest.py) rather than
assumed. And a client only speculates if the service said `"early": true` in its
`hello` ack: a service that ignores `speculate` would leave the audio buffered
and then receive the real utterance appended to it, so the model would be asked
a question with its first half said twice. That is worse than a missing feature,
so it is negotiated rather than assumed.

Endpointer timing is counted in 20ms blocks rather than wall-clock seconds —
`sounddevice` delivers in bursts after a scheduling hiccup, and a clock reads
that burst as a long silence and ends the turn mid-sentence.

## The same conversation, with no GPU in it

[session.py](session.py) holds the conversation against one of Alibaba's
hosted omni models over its WebSocket protocol. The drive console runs it
([omni_bridge.py](../drive_web/omni_bridge.py)): MEDIA drops out of the path
entirely — no Whisper, no local weights, no Kokoro, no card — and the microphone
is a browser tab.

```
  phone / desk browser                      dashscope-intl, Singapore
  --------------------                      -------------------------
  mic -> wss://rover:8771/audio --16k pcm-->  qwen3.5-omni-plus-realtime-2026-03-15
                                                |  text + tool calls
  speakers <-- playback <--24k pcm--------------+
       |
       +-- tool call --> rover_daemon.py on loopback --> the board, the camera

  the camera --- one JPEG, POST /frame --> 127.0.0.1:8774 --> into the session
```

The split is the one this directory already had. Audio stays where the
microphone is, the tools stay on the rover, and only the model moves. Nothing was
rewritten to make that work: the schemas come off the daemon over the wire
exactly as [rover_tools.py](rover_tools.py) has always fetched them, in the same
`{"type": "function", "function": {...}}` shape this API happens to want, and the
system prompt is read out of [server.py](server.py) by
[prompts.py](prompts.py) rather than copied. Improve a description on the rover
and it is in force here on the next connection.

The key on the rover is `~/.ugv/alibaba.key` (or `$DASHSCOPE_API_KEY`), and it is
never printed. The voice is `Jennifer`, which the service describes as a premium
American English female voice; `Aiden` is American and male, `Mione` is British
and female, and `$QWEN_REALTIME_VOICE` picks between them. The service's own
default, `Tina`, speaks English with a marked accent, and a name from the wrong
model's voice list is not refused gracefully — it closes the socket.

There is no rover on most desks, so there is a rover:
[mock_rover.py](mock_rover.py) speaks the daemon's protocol, holds the state a
real one would hold — turn the lights on, then ask whether they are on, and the
second answer depends on the first — and hands `look` a drawn test card. The
schemas it offers are the daemon's own, read from its source, because a mock that
described its tools in its own words would be a mock of a different rover.

### It works, and here is a whole conversation

Five turns, five tools, against the mock, on the default model:

```
> can-you-switch-the-lights-on.wav
  [set_lights{"level": 255} -> {"ok": true, "level": 255}]
  bot: I've turned the lights on at full brightness.
> start-tracking-people.wav
  [start_tracking{} -> {"ok": true, "tracking": true}]
  bot: I've started tracking people.
> how-many-people-can-you-see.wav
  [count_faces{} -> {"ok": true, "faces": 2, "where": [...]}]
  bot: I can see two people. One is on your left, quite close, and the other is
       in the centre, further away.
> what-do-you-see.wav
  [look{} -> {"ok": true, "image": "frame-1"}]
  [sent frame-1, 5603 bytes, 320x240]
  bot: I see a red square and a green circle on a light gray background. The word
       "ROVER" is written above them.
> would-you-stop-following-me.wav
  [stop_tracking{} -> {"ok": true, "tracking": false}]
  bot: I've stopped following you.
```

That is the test card, described. Every phrasing that defeated `Qwen3-VL-4B` is
answered correctly too — the leading "Well,", the dimming request (which comes
back as `{"level": 128}`), "Follow me.", "Track someone else." — and "What is
your name?" calls nothing, which is the case that is supposed to.

There is no `_ToolSniffer` on this path. A tool call arrives as its own event on
a control channel rather than as a `<tool_call>` marker racing a speech decoder,
so the interception problem [docs/omni-step0.md](../docs/omni-step0.md) measured
does not arise — as long as the model uses that channel. See [Flash and plus are
not the same model](#flash-and-plus-are-not-the-same-model) for what it looks
like when one does not.

### Five things about this protocol that cost an afternoon

All five fail by going *quiet* rather than by complaining, which is why
`QWEN_REALTIME_TRACE=1` prints every event in and out.

**Start reading the socket before you configure it.** Everything this client
waits to be told — that the session took, that a turn landed — is told through
the receive loop, so setting the session up before that loop is running means
waiting fifteen seconds for an event that arrived in a third of a second and had
nobody to hand it to. It costs a stall at startup rather than a failure, which is
why it survived a whole afternoon of testing before anybody noticed.

**The voice list is per model.** `Cherry` is documented and this model refuses
it, and a refused voice does not fall back — it closes the socket at the first
`session.update`. Ask the service what it defaults to; it says so in
`session.created`, and for this model it is `Tina`.

**A picture cannot go into an empty input buffer.** The rule is "you must send
audio data at least once before you send image data", and a buffer that has just
been committed counts as empty — which is exactly the state a tool result arrives
in. So a frame travels as a user turn of its own, led by 200ms of silence whose
only job is to satisfy that rule. Get it wrong and the reply is
`Error append image before append audio` — and then the model describes a room it
was never shown, in confident detail, without ever saying it could not see.

**A `response.create` sent before the turn it refers to has *finished* is
discarded in silence.** No error, no reply, a conversation that simply stops with
the rover having taken a photograph and said nothing about it. Waiting for the
commit's acknowledgement is not enough, and neither is waiting for the item to be
created — both arrive while the turn is still `in_progress`. The event that is
late enough is the transcription completing.

**A manual commit is rejected outright while the service is deciding turns.**
`Internal service error: null`, on both `server_vad` and `semantic_vad`. Since a
picture needs a manual commit, full duplex hands turn-taking back for about a
second per photograph and takes it again afterwards, holding the microphone shut
in between so that nothing said in that moment joins the frame's turn. That
second is the one real cost: a question asked just after the shutter is not
heard.

### Barge-in, and the headphones it asks for

`--duplex` puts turn-taking on the service, using `semantic_vad` — which is the
mode that knows the difference between somebody saying "mm-hm" and somebody
taking the floor. An interruption then needs both halves: the service stops
generating, and only this client can stop the speaker playing what it has already
received. So `Session.interrupt` cancels, flushes what was never heard, and
reports how many milliseconds actually reached the card.

That last number is the one that keeps the model honest, and there is no
documented way to give it back. `conversation.item.truncate` exists in the OpenAI
protocol this one is modelled on and is not in this service's published client
events; it is tried once and abandoned if refused. Until it works, an interrupted
rover remembers saying the part nobody heard.

**It is the default, so wear headphones.** Barge-in needs the microphone open
while the reply plays, and an open microphone in a room with speakers hears the
rover's own voice, decides it is being interrupted, and stops itself mid-sentence
forever. Alibaba's own documentation says to wear headphones for exactly this.
`Ears` is a crude suppressor for when you will not — it passes the microphone
only when it is clearly louder than what is coming out of the speaker — and it is
not acoustic echo cancellation, has no model of the room and no reference
alignment, and will not save a loud room or a close speaker. Headphones will.

`--half-duplex` is the other way: silence never crosses the network, because this
client decides when a turn ended and only uploads what it judges to be speech. Full duplex gives that
up, since a service that decides when a turn ended cannot decide it from audio it
was never sent.

### Against the actual rover

Everything above is the mock. Run against `bpi-m4zero.local`, the picture path works end
to end — a 44kB, 640×480 frame off the rover's camera, described:

```
pictures: the rover will post to http://192.168.1.206:8767/frame
> what-do-you-see.wav
  [look{} -> {"ok": true, "image": "frame-1"}]
  [sent frame-1, 44524 bytes, 640x480]
  bot: I see a person sitting at a glass desk with a laptop. To the left,
       there's a large window looking out onto a balcony with some green turf.
```

That line about where the rover will post is the fix for the first thing that
went wrong on real hardware. The picture's destination used to be a constant,
given to the daemon at startup by whoever last edited the crontab, and it pointed
at MEDIA. When the model moved off MEDIA the pictures kept going there, and
`look` failed with `No route to host` while every other tool on the rover worked
perfectly — which is a confusing thing to debug, because the rover is plainly
fine and the camera is plainly fine.

So the destination is no longer remembered anywhere. The daemon takes a control
call, `set_vision`, which is dispatched like a tool and deliberately absent from
`list_tools` so no model is ever shown it, and this client sends it on every
connection. The address it sends is the one its own socket to the daemon is bound
to, which is right by construction: the kernel already picked the interface that
reaches the rover, and it picks a different one once the rover is off its dock
and answering on wlan0. Naming no address switches the path off and withdraws
`look` from the tool list, because a tool that cannot reach the model's host is
worse than a missing one.

### Face tracking is not fixed by any of this

The second thing that went wrong on real hardware: `start_tracking` answered
`{"ok": true, "tracking": true}`, the model said "I started tracking people", and
the camera never moved. The daemon's own log says why —
`[rover] the face detector is not answering; holding still` — because face
detection is a *separate* service on MEDIA, on port 8768, and MEDIA was off.
Moving the model to Alibaba took MEDIA out of the conversation; it did not take
it out of face tracking, and it cannot.

The loop is written to hold still through a missing detector rather than to die,
which is right for a loop already running and wrong for one being started: it
starts, holds still, and reports itself as tracking. So `start_tracking` now
checks the detector is reachable first and refuses honestly:

```
{"ok": false, "error": "the face detector at 192.168.1.3:8768 is not answering
 (TimeoutError), so tracking a face is not possible right now"}
```

which the model reads out as a reason rather than saying it has done something it
has not. That is a better failure, not a fix. Face tracking needs MEDIA up, or a
detector somewhere else.

### Two ways the frame server bit back

Neither is about the model, and both were found by running it rather than by
reading it.

**Ctrl-C hung the terminal after any picture.** The rover posts frames over one
kept-open connection, deliberately — and a plain `HTTPServer` handles requests
one at a time, inside `serve_forever`. So after a single `look` the server is
parked in that connection's handler, blocked on a request line that will not
arrive until the *next* look. Nothing else can be accepted, and `shutdown()`
never returns, because the loop it waits on is the blocked one. From outside:
"bye" prints and the shell never comes back. `Frames` is a `ThreadingHTTPServer`
now, which is what makes the `daemon_threads = True` that was already there mean
anything, and handlers time out so an idle connection does not leak a thread.

**Two clients could hold port 8767 at once, and quietly.** `SO_REUSEADDR` on
Windows does not mean "reclaim a port in TIME_WAIT", it means *share*: a second
bind succeeds and which server a connection reaches is undefined. A leftover
client therefore steals the rover's pictures, and the running one is handed a
frame name it is not holding. So the server no longer sets it, and a second
instance refuses to start and says so.

That second bug is why `look`'s result is now rewritten rather than merely warned
about. Handed `{"ok": true, "image": "frame-1"}` and shown no picture, the model
described a wooden table, a white mug and a small green plant, none of which had
ever been in front of the rover. A tool that failed has to read as one:

```
{"ok": false, "error": "the picture was taken but never arrived here, so there
 is nothing to look at"}
```

### Flash and plus are not the same model

[docs/omni-step0.md](../docs/omni-step0.md) could not tell `qwen3.5-omni-plus`
and `qwen3.5-omni-flash` apart: both scored 90/90 typed and spoken and 30/30 on
the five extra tools, and flash cost $0.05 against plus's $0.17 for the same
sweep, so flash was the obvious choice. That was the *chat completions* pair.
Their realtime namesakes behave nothing alike. Three samples a phrase, first turn
of a session, against the mock:

| asked for | `flash-realtime` | `plus-realtime` |
|---|---|---|
| "Could you dim the lights a bit?" | **0/3** | 3/3 |
| "Can you look to your left?" | **0/3** | 3/3 |
| "Start tracking people." | **0/3** | 3/3 |
| "Switch the lights on." | **1/3** | 3/3 |
| "What do you see?" | **12/18** | 6/6 |

Flash fails in the two ways this directory has spent the most words on. It
announces without acting — "I'll pan the camera to my left now", and no call,
which is the failure the [system prompt's closing
sentence](#the-re-run-it-does-not-refuse-it-promises) exists to prevent. And it
*writes the tool call into its own speech*:

```
  bot: <set_lights> <parameter=level> 128 </parameter> </function> </tool_call>
  calls: none
```

That is the rover reading a tool call out loud while doing nothing — the same
thing step 0 caught MiniCPM-o's speech decoder doing, arriving by a different
road. It is not the sniffer's failure, because the sniffer watches a text stream
that this protocol replaces with a control channel; it is a model declining to
use the channel it was given. Worse, it poisons what follows: the next two turns
say "I've dimmed the lights to half brightness" and call nothing, which is
[one promise poisoning everything after
it](#and-one-promise-poisons-everything-after-it) exactly as documented.

Both are recoverable in principle — parse the markup, re-tune the schemas — and
neither was worth doing while the model beside it was simply right. So plus was
the default despite costing about three times as much.

**It is not any more, and not for a reason to do with quality.** On 2026-08-17
`plus-realtime` stopped answering this account: its free tier is exhausted, and
the console offers no pay-as-you-go to fall back to — the Free Quota Only switch
that the refusal tells you to turn off reads *Enabling not supported* for this
model, which means it has no free quota left to gate. The service says so at the
first `response.create`, closing the socket with 1007 and "The free tier of the
model has been exhausted", which is error `AllocationQuota.FreeTierOnly` in the
protocol's own clothing. The reason is easy to miss twice over: the session opens
and `session.created` arrives before anything goes wrong, and the close frame
carrying the text is longer than the 125 bytes the RFC allows a control frame, so
`websockets` throws the reason away and raises a protocol error about the frame
length instead. A hand-rolled socket read it.

The undated alias still cannot bill. A dated snapshot is a different model with
its own quota, and as of 2026-08-19 `qwen3.5-omni-plus-realtime-2026-03-15` still
has 1,000,000/1,000,000 free until 2026-11-15, with Stop-on-Exhaust already off.
That is the default. The alias is `QWEN_REALTIME_MODEL=qwen3.5-omni-plus-realtime`
once that row can bill; flash is `QWEN_REALTIME_MODEL=qwen3.5-omni-flash-realtime`.

### It was one sentence of the prompt all along

Being forced onto flash is what finally got the table above tested rather than
believed, and it does not survive. Holding the microphone out of it — typed input,
so only the prompt and the tool list vary — first-turn calls for "Switch the lights
on.", three samples a cell:

| | one schema | all fourteen |
|---|---|---|
| a three-line instruction | 3/3 | 3/3 |
| the tuned prompt | 3/3 | **0/3** |

Neither the prompt nor the tool count breaks anything alone; together they break
it every time. Bisecting the prompt against the full list then puts it on a single
sentence, and not one of the ones you would suspect — the base prompt calls 3/3,
the clause about never claiming you did something calls 3/3, the closing "do not
say 'I will'" calls 3/3, the vision paragraph calls 3/3. What costs all three
points is this:

> Then say what you did in one short sentence, without reading the tool call or
> its result out loud.

Removing that one sentence and keeping every other word takes flash from 0/3 to
3/3. **The sentence forbidding the model to read the tool call out loud is what
makes it read the tool call out loud** — naming the unwanted behaviour is a way of
asking for it, and this is the cleanest example of that here. It is also why the
markup in the transcript above looks the way it does: the model is not failing to
find the control channel, it is writing what the sentence just described.

So `talk.py` removes that sentence on the way to flash and leaves it alone
otherwise (see `instructions` there). It stays in [server.py](server.py) because
it earns its place there — it is what stops the local model's speech decoder
reciting result JSON — and the local path has no control channel to lose. Plus was
measured with the sentence and is fine.

With it gone, flash on the real client and the same synthetic speech that produced
the 0/3 rows:

```
> switch-the-lights-on.wav        [set_lights{"level": 255}]  "I've switched the lights on at full brightness."
> could-you-dim-the-lights-a-bit  [set_lights{"level": 128}]  "I've dimmed the lights to half brightness."
> can-you-look-to-your-left       [look_at{"pan": -30}]       "I've turned the camera thirty degrees to the left."
> start-tracking-people           [start_tracking{}]          "I've started tracking people."
> are-the-lights-on               [get_lights{}]              "Yes, the lights are on at half brightness."
> what-is-your-name               no call                     "I don't have a name."
```

Three of those rows are 0/3 in the table above. The last one is there on purpose:
removing a sentence that suppressed calling could have pushed the model into
calling for everything, and it did not.

**What this does not license.** Those six phrases were against the mock, which
offers nine tools with vision off. Against the live daemon's fifteen, which is what
you actually talk to, the fix carries most of the way and not all of it — typed,
three samples a phrase:

| | as written | sentence removed |
|---|---|---|
| "Switch the lights on." | 0/3 | 3/3 |
| "Could you dim the lights a bit?" | 0/3 | 3/3 |
| "Can you look to your left?" | 0/3 | 3/3 |
| "Drive forward a little." | 0/3 | 3/3 |
| "What do you see?" | 3/3 | 3/3 |
| "Start tracking people." | 0/3 | **1/3** |
| "Follow me." | 0/3 | **0/3** |

So the tracking family is a second, separate failure, and it is the crowding this
directory already documents rather than anything to do with that sentence. It comes
on gradually as the tool list grows: with the fix applied, "Start tracking people."
calls 3/3 against the nine base tools, 3/3 with `look` added, 2/3 once the three
driving tools arrive, and 1/3 with `describe_surroundings` and `show_map` too.
Tracking with a camera evidently reads as something the driving tools might do.

Two attempts at the wording that fixed `count_faces` — naming what the tool is not
for — did not survive contact. Appending "Only the camera moves: the rover does not
drive after anybody" left it at 2/3, and the blunter "This moves the camera, not
the wheels" made it *worse*, at 0/3. Nor is the prompt short of instruction: flash
fails these by announcing in the past tense, "I've started tracking people", which
the prompt forbids by name — "never say you have switched, moved, **started** or
stopped anything unless the call was made and answered". It is a model ignoring a
rule it was given, not a rule nobody wrote.

Which puts flash's real ceiling here rather than where the original table put it,
and leaves the account question as the thing actually worth solving: `plus` calls
the tracking family 3/3 with no wording help at all. It also leaves a question open
for [omni_bench](../omni_bench/) — whether that one sentence is quietly costing
plus something too, since plus was only ever measured with it.

**The numbers above are not from a room.** They are synthetic speech played into
a socket against a rover that does not exist. The rover half has since been run
for real — see [Against the actual rover](#against-the-actual-rover) — but echo
and interruption still have not been, and those are what full duplex is for.

### What it costs

The prompt and the ten schemas are *sent* once per session rather than with every
question, which is the shape step 0 asked for. They are still counted on every
response: the usage reported back puts `cached_tokens` at 0 or 128 against
1,900–2,400 input tokens a turn. So this saves the upload, not the bill. A spoken
question is 14–105 audio tokens and a picture is 80 video tokens — both rounding
error next to the schemas, exactly as step 0 found. A session is capped at two
hours, announced by the socket closing.

## Checks

`selftest.py` covers the pieces where a bug is silent rather than loud: the
sentence splitter, the endpointer, history trimming, the sniffer that keeps a
tool call from being spoken aloud, the line to the rover daemon, the drive
consoles’ shared model — the mid-move commentary, the status alarms, which map to
ask for at a given panel width, and the rule that stops a rover once the last
browser has gone — and, for the
hosted path — the prompt reader, the `/frame` server, the playback bookkeeping a
barge-in depends on, and the session protocol against a service that only writes
down what it was told. What the daemon *does* with a call has its own checks, in
[rover_daemon/selftest.py](../rover_daemon/selftest.py), which run on the rover.
It has no GPU or microphone dependency, and each part skips where its
dependencies are absent, so run it anywhere:

```bash
python voice_chat/selftest.py               # endpointer, rover client, talk
ssh root@media /opt/voice_chat/.venv/bin/python /opt/voice_chat/selftest.py
ssh bpi-m4zero 'cd ugv && python3 selftest.py'     # the daemon's own, on the rover
```

The hosted path is checked offline in [test_talk.py](test_talk.py) against a
fake WebSocket, and on the rover by opening the console's microphone.

## Deploying

Source of truth is this directory; the guest copy is not authoritative.

```bash
scp voice_chat/{server.py,voice_history.py,voice_stream.py,voice_http.py,requirements.txt,selftest.py,test_harness.py,test_server.py,test_talk.py} root@media:/opt/voice_chat/
scp voice_chat/voice-chat.service root@media:/etc/systemd/system/
# pillow is new with vision; the rest of the venv is unchanged.
ssh root@media 'VIRTUAL_ENV=/opt/voice_chat/.venv /root/.local/bin/uv pip install pillow'
ssh root@media 'systemctl daemon-reload && systemctl restart voice-chat'

scp rover_daemon/*.py bpi-m4zero:~/ugv/
```

`session.py`, `talk_frames.py`, `prompts.py` and `rover_tools.py` deploy with the
drive console. `prompts.py` in particular has to: it reads `server.py`,
`tool_schemas.py` and `rover_nav.py` off the disk beside it, so it only works
from a checkout where both are present — or, on the rover, from the copies
landed next to `drive_web.py`. The rover copy of the daemon is
flat in `~/ugv/` alongside the face-tracking scripts, which is the layout already
there; nothing on the rover needs installing.

First start downloads ~10GB of weights. Unauthenticated HF Hub requests are rate
limited to roughly one file per five minutes — set `HF_TOKEN` in the unit if you
would rather not wait.

Two things about that venv are load-bearing and not obvious:

- `en_core_web_sm` is pinned as a wheel in `requirements.txt`. Kokoro's G2P
  otherwise tries to `spacy download` it at startup, which fails under systemd
  because a uv-built venv has no pip ("No package installer found").
- `LD_LIBRARY_PATH` in the unit points at the venv's `nvidia/{cublas,cudnn}/lib`.
  faster-whisper's CTranslate2 dlopens CUDA 12 while torch 2.13 brings CUDA 13,
  so both runtimes have to be installed and the loader needs the path *before*
  the process starts. Without it the LLM and TTS work fine and only STT fails,
  with `libcublas.so.12 is not found`.
